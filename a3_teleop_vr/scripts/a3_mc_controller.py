#!/usr/bin/env python3.12
"""MoveIt FollowJointTrajectory -> 智元 MC 真机执行适配器。

让 move_group 的轨迹执行（RViz 拖小球 Plan&Execute、move_action 脚本等）直接驱动
真机双臂：本节点对外提供两个标准 FollowJointTrajectory action server
（/left_arm_controller/follow_joint_trajectory、/right_arm_controller/...，
名称与 a3_moveit_demo 的 moveit_controllers.yaml 一致，move_group 无感切换），
收到轨迹后按 100Hz 采样插值，聚合为 14 关节 JointState，经与流式遥操作
（a3_streaming_teleop.py mc_arm 模式）完全相同的安全链后发往
/motion/control/arm_joint_command：

  官方关节限位钳制 -> 相邻指令速度限幅（max_speed，默认 4 rad/s）-> 低通平滑
  （lowpass_alpha，y=(1-a)y_prev+a*x）

未执行轨迹时也按 100Hz 持续发送保持帧（官方要求连续流），保持帧 = 最后下发值，
绝不自动回零。状态反馈从 /motion/control/arm_joint_state 读取（仅作记录与
反馈上报，不进入安全链——安全链以上一帧下发值为基准，与官方建议一致）。

注意：与 a3_streaming_teleop 的 mc_arm 模式互斥，两者同时运行会争抢
/motion/control/arm_joint_command（MC 只认最后到达的帧）。

用法（real_teleop.launch.py 会自动拉起，也可单独跑）：
  /usr/bin/python3.12 scripts/a3_mc_controller.py
"""
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mc_arm_interface import MC_ARM_JOINTS, MC_JOINT_LIMITS  # noqa: E402

# 双臂 7 关节（与 moveit_controllers.yaml 中各控制器 joints 一致）
ARM_JOINTS = {
    "left": [n for n in MC_ARM_JOINTS if n.startswith("left_")],
    "right": [n for n in MC_ARM_JOINTS if n.startswith("right_")],
}


def sample_trajectory(times, positions, t):
    """按时间线性插值采样轨迹；t 越界时钳到端点。"""
    if t <= times[0]:
        return positions[0]
    if t >= times[-1]:
        return positions[-1]
    i = int(np.searchsorted(times, t)) - 1
    t0, t1 = times[i], times[i + 1]
    q0, q1 = positions[i], positions[i + 1]
    ratio = (t - t0) / max(t1 - t0, 1e-9)
    return q0 + ratio * (q1 - q0)


class McArmController(Node):
    def __init__(self):
        super().__init__("a3_mc_controller")

        # --- 参数 ---
        self.declare_parameter("rate", 100.0)
        self.declare_parameter("mc_cmd_topic", "/motion/control/arm_joint_command")
        self.declare_parameter("mc_state_topic", "/motion/control/arm_joint_state")
        self.declare_parameter("max_speed", 4.0)      # rad/s（官方 ≤4）
        self.declare_parameter("lowpass_alpha", 0.5)  # 低通 y=(1-a)y_prev+a*x
        self.declare_parameter("stream_timeout", 0.3)  # 流式话题静默多久后归还 action 控制权
        p = self.get_parameter
        self.rate = float(p("rate").value)
        self.max_speed = float(p("max_speed").value)
        self.lowpass_alpha = min(1.0, max(0.01, float(p("lowpass_alpha").value)))
        self.stream_timeout = float(p("stream_timeout").value)

        self.limits_low = np.array([MC_JOINT_LIMITS[n][0] for n in MC_ARM_JOINTS])
        self.limits_high = np.array([MC_JOINT_LIMITS[n][1] for n in MC_ARM_JOINTS])

        # --- 共享状态（execute 线程与发布线程并发，必须加锁） ---
        self.lock = threading.Lock()
        self.measured = {n: 0.0 for n in MC_ARM_JOINTS}  # 反馈（仅上报用）
        self.state_fresh = False
        self.target = None      # 14 维本周期目标（各控制器写入各自 7 维）
        self.prev_sent = None   # 14 维上一帧实际下发（低通+限速后的）
        self.generation = {"left": 0, "right": 0}  # 每侧目标代际，新 goal 顶掉旧的
        self.stream_until = {"left": 0.0, "right": 0.0}  # 流式话题控制权到期时刻

        # --- 通信 ---
        self.cmd_pub = self.create_publisher(
            JointState, str(p("mc_cmd_topic").value), 10)
        self.create_subscription(
            JointState, str(p("mc_state_topic").value), self.on_joint_state, 50)

        # --- 双臂 action server + 流式话题入口（联动仲裁，见 on_stream_traj） ---
        self.servers = {}
        for side in ("left", "right"):
            name = f"{side}_arm_controller/follow_joint_trajectory"
            self.servers[side] = ActionServer(
                self,
                FollowJointTrajectory,
                name,
                execute_callback=self.make_execute(side),
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
            )
            self.get_logger().info(f"action server 就绪: /{name}")
            joints = ARM_JOINTS[side]
            side_idx = [MC_ARM_JOINTS.index(n) for n in joints]
            self.create_subscription(
                JointTrajectory, f"{side}_arm_controller/joint_trajectory",
                lambda msg, s=side, j=joints, i=side_idx:
                    self.on_stream_traj(s, j, i, msg),
                20)

        # 专用发布线程跑 100Hz 指令流：不经过 executor 定时器——executor 的
        # 多线程唤醒有几十毫秒级抖动（见 test_mc_controller_local.py 踩坑），
        # 而官方要求相邻指令间隔 ≤30ms，MC 侧按最后到达帧持续执行。
        self._stop_event = threading.Event()
        self._pub_thread = threading.Thread(
            target=self._publish_loop, name="mc_cmd_100hz", daemon=True)
        self._pub_thread.start()
        self.get_logger().info(
            f"MC 执行适配器就绪：{self.rate:.0f} Hz，限位/限速 {self.max_speed} rad/s"
            f"/低通 alpha={self.lowpass_alpha}，等待轨迹目标")

    def destroy_node(self):
        self._stop_event.set()
        self._pub_thread.join(timeout=1.0)
        return super().destroy_node()

    def _publish_loop(self):
        period = 1.0 / self.rate
        next_t = time.monotonic()
        while not self._stop_event.is_set():
            self.publish_tick()
            next_t += period
            self._stop_event.wait(max(0.0, next_t - time.monotonic()))

    # ---------- 反馈 ----------
    def on_joint_state(self, msg):
        with self.lock:
            for n, pos in zip(msg.name, msg.position):
                if n in self.measured:
                    self.measured[n] = float(pos)
            if not self.state_fresh:
                self.state_fresh = True
                # 首帧反馈到达后初始化目标/下发基准，之前不发指令（避免盲发）
                if self.target is None:
                    self.target = np.array(
                        [self.measured[n] for n in MC_ARM_JOINTS], dtype=np.float64)
                if self.prev_sent is None:
                    self.prev_sent = self.target.copy()

    # ---------- 流式遥操作入口（联动仲裁，见 execute 内的抢占检查） ----------
    def on_stream_traj(self, side, joints, side_idx, msg):
        """joint_trajectory 话题到达即接管本侧：写目标并续期流式控制权。

        写入 self.target 后仍会走与 action 完全相同的 100Hz 安全链
        （限位/限速/低通）。控制权在 stream_timeout 内自动续期，
        静默超时后 execute 里的 action 目标恢复执行。
        """
        if not msg.points:
            return
        name_to_pos = dict(zip(msg.joint_names, msg.points[-1].positions))
        if any(n not in name_to_pos for n in joints):
            self.get_logger().warn(
                f"[{side}] 流式轨迹关节名不全，忽略: {list(msg.joint_names)}")
            return
        q = np.array([name_to_pos[n] for n in joints], dtype=np.float64)
        with self.lock:
            if self.target is None:
                return  # 无反馈基准，不盲写
            self.target[side_idx] = q
            self.stream_until[side] = time.monotonic() + self.stream_timeout

    # ---------- 100Hz 发布：限位 -> 限速 -> 低通 ----------
    def publish_tick(self):
        with self.lock:
            if self.target is None:
                return  # 还没收到任何反馈，不盲发
            target = self.target.copy()
            prev = self.prev_sent
            target = np.clip(target, self.limits_low, self.limits_high)
            if prev is not None:
                max_delta = self.max_speed / self.rate
                target = prev + np.clip(target - prev, -max_delta, max_delta)
                target = prev + self.lowpass_alpha * (target - prev)
            self.prev_sent = target.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "a3_mc_controller"
        msg.name = list(MC_ARM_JOINTS)
        msg.position = target.tolist()
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        self.cmd_pub.publish(msg)

    # ---------- action server ----------
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_request):
        return CancelResponse.ACCEPT

    def make_execute(self, side):
        joints = ARM_JOINTS[side]
        side_idx = [MC_ARM_JOINTS.index(n) for n in joints]
        err_ok = FollowJointTrajectory.Result.SUCCESSFUL
        err_bad_joints = FollowJointTrajectory.Result.INVALID_JOINTS
        err_bad_goal = FollowJointTrajectory.Result.INVALID_GOAL

        def execute(goal_handle):
            traj = goal_handle.request.trajectory
            gen = self.bump_generation(side, goal_handle)

            # --- 校验：关节名集合必须与本侧 7 关节完全一致 ---
            if set(traj.joint_names) != set(joints):
                self.get_logger().error(
                    f"[{side}] 拒绝目标：关节名不匹配 got={list(traj.joint_names)}")
                goal_handle.abort()
                r = FollowJointTrajectory.Result()
                r.error_code = err_bad_joints
                r.error_string = "joint_names mismatch"
                return r
            # 轨迹点按本侧关节顺序重排
            order = [list(traj.joint_names).index(n) for n in joints]
            points = []
            for pt in traj.points:
                if len(pt.positions) != len(traj.joint_names):
                    goal_handle.abort()
                    r = FollowJointTrajectory.Result()
                    r.error_code = err_bad_goal
                    r.error_string = "point positions size mismatch"
                    return r
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                points.append((t, np.asarray(pt.positions, dtype=np.float64)[order]))
            if not points:
                goal_handle.abort()
                r = FollowJointTrajectory.Result()
                r.error_code = err_bad_goal
                r.error_string = "empty trajectory"
                return r
            points.sort(key=lambda x: x[0])
            times = np.array([p[0] for p in points])
            positions = np.stack([p[1] for p in points])
            duration = float(times[-1])

            # 等 MC 反馈到位再动：没有反馈就没有合法的下发基准，宁可拒绝
            deadline = time.monotonic() + 2.0
            while not self.state_fresh and time.monotonic() < deadline and rclpy.ok():
                time.sleep(0.01)
            if not self.state_fresh:
                self.get_logger().error(
                    f"[{side}] 拒绝目标：未收到 MC 关节状态（{self.state_fresh}）")
                goal_handle.abort()
                r = FollowJointTrajectory.Result()
                r.error_code = err_bad_goal
                r.error_string = "no MC joint state feedback"
                return r

            self.get_logger().info(
                f"[{side}] 开始执行：{len(points)} 点，时长 {duration:.2f}s")

            t0 = time.monotonic()
            period = 1.0 / self.rate
            next_tick = t0
            n_tick = 0
            result = FollowJointTrajectory.Result()
            result.error_code = err_ok

            while rclpy.ok():
                if self.generation[side] != gen:
                    # 被同侧新目标抢占（旧句柄已被 abort）
                    result.error_code = getattr(
                        FollowJointTrajectory.Result, "PREEMPTED", -6)
                    return result
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f"[{side}] 目标取消，保持当前位姿")
                    goal_handle.canceled()
                    return result
                if time.monotonic() < self.stream_until[side]:
                    # 流式遥操作（joint_trajectory 话题）接管本侧，抢占 action
                    result.error_code = getattr(
                        FollowJointTrajectory.Result, "PREEMPTED", -6)
                    result.error_string = "preempted by joint_trajectory stream"
                    self.get_logger().info(f"[{side}] 流式控制接管，action 目标被抢占")
                    goal_handle.abort()
                    return result
                t = time.monotonic() - t0
                q = sample_trajectory(times, positions, t)
                with self.lock:
                    if self.target is None:
                        self.target = np.zeros(14, dtype=np.float64)
                    self.target[side_idx] = q
                    measured_side = np.array(
                        [self.measured[n] for n in joints])
                # 反馈 20Hz
                if n_tick % 5 == 0:
                    fb = FollowJointTrajectory.Feedback()
                    fb.joint_names = list(joints)
                    fb.desired = JointTrajectoryPoint(
                        positions=q.tolist(),
                        time_from_start=self.sec_to_duration(t))
                    fb.actual = JointTrajectoryPoint(positions=measured_side.tolist())
                    fb.error = JointTrajectoryPoint(
                        positions=(measured_side - q).tolist())
                    goal_handle.publish_feedback(fb)
                n_tick += 1
                if t >= duration:
                    break
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))

            self.get_logger().info(f"[{side}] 轨迹执行完成，保持终点")
            goal_handle.succeed()
            return result

        return execute

    def bump_generation(self, side, goal_handle):
        with self.lock:
            self.generation[side] += 1
            old = getattr(self, f"_active_{side}", None)
            if old is not None and old.is_active:
                old.abort()
            setattr(self, f"_active_{side}", goal_handle)
            return self.generation[side]

    @staticmethod
    def sec_to_duration(t):
        return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def main():
    rclpy.init()
    node = McArmController()
    # 必须用多线程 executor：execute 回调是阻塞式的（轨迹时长级），
    # 单线程 executor 会被它独占——100Hz 发布定时器、取消请求、
    # 另一侧 goal 的 execute 全部排队（见 test_mc_controller_local.py 踩坑）。
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
