#!/usr/bin/env python3.12
"""a3_mc_controller（FJT->MC 执行适配器）本机自测：fake MC，无需机器人。

进程内起全部节点，executor.spin() 放后台线程（execute 回调是阻塞式的，
单线程 spin_once 喂不活并发 goal/取消——见文件尾踩坑记录）：
  FakeMc            发 /motion/control/arm_joint_state（14 关节非零静态位姿 100Hz），
                    记录收到的 /motion/control/arm_joint_command；
  McStateEcho       被测节点：14 -> 31 关节 /joint_states 回显；
  JointStateProbe   订阅 /joint_states 记录最新帧；
  McArmController   被测节点：双臂 FollowJointTrajectory action server。

测试序列：
  1. 左右臂各发一个 2s 轨迹目标（并发）-> 断言双 SUCCESSFUL；
  2. 取消测试：左臂 8s 慢轨迹，1.2s 后取消 -> 断言 CANCELED 且指令流不断；
  3. 抢占测试：右臂 8s 慢轨迹，0.5s 后同侧再发 1s 快轨迹 -> 第一个被
     抢占（非 SUCCESSFUL），第二个 SUCCESSFUL；
  4. 联动仲裁：左臂 8s 慢轨迹执行中，经 joint_trajectory 话题 20Hz 注入
     单点目标 -> action 被抢占（注入值经同一安全链下发）；话题静默后
     action 恢复可用。

全程断言（对齐官方接口要求）：
  - 指令流 ~100Hz、p99 间隔 <=30ms、14 关节名固定顺序、velocity/effort 全 0
  - 相邻指令速度 <=4 rad/s（+10% 余量）、值在官方限位内
  - 轨迹终点收敛（低通保持后到位误差 <0.05 rad）
  - 回显 /joint_states 为 31 关节，手臂值=MC 反馈值，其余补 0

用法（先 source ROS 2 Jazzy 环境）：
  /usr/bin/python3.12 scripts/test_mc_controller_local.py
退出码 0 = 全部通过。
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a3_mc_controller import McArmController  # noqa: E402
from mc_arm_interface import MC_ARM_JOINTS, MC_JOINT_LIMITS  # noqa: E402
from mc_state_to_joint_states import McStateEcho  # noqa: E402

# fake MC 的静态反馈位姿（非零，验证回显与轨迹起点绑定）
FAKE_POSE = {n: 0.0 for n in MC_ARM_JOINTS}
FAKE_POSE["left_shoulder_pitch_joint"] = 0.3
FAKE_POSE["right_elbow_joint"] = -0.2

LEFT_TARGET = {"left_shoulder_pitch_joint": 0.8}   # 测试 1 左臂终点
RIGHT_TARGET = {"right_elbow_joint": -0.6}         # 测试 1 右臂终点


class FakeMc(Node):
    """替身 MC：发非零静态状态、记指令流。"""

    def __init__(self):
        super().__init__("fake_mc")
        self.cmds = []  # (monotonic, pos14, msg)
        self.pub = self.create_publisher(
            JointState, "/motion/control/arm_joint_state", 10)
        self.create_subscription(
            JointState, "/motion/control/arm_joint_command", self.on_cmd, 200)
        self.create_timer(0.01, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MC_ARM_JOINTS)
        msg.position = [FAKE_POSE[n] for n in MC_ARM_JOINTS]
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        self.pub.publish(msg)

    def on_cmd(self, msg):
        self.cmds.append((time.monotonic(), np.asarray(msg.position), msg))


class JointStateProbe(Node):
    def __init__(self):
        super().__init__("joint_state_probe")
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)

    def on_js(self, msg):
        self.latest = msg


def build_goal(side, targets, duration, start=None, n_points=41):
    """构造单臂 FJT goal：start（默认 FAKE_POSE）线性插值到 targets。

    start 必须传当前指令位姿——真实场景 move_group 的规划就从当前状态出发，
    轨迹首点=机器人实际位置；这里手动对齐以免"轨迹从旧反馈位姿出发"的假象。
    """
    joints = [n for n in MC_ARM_JOINTS if n.startswith(f"{side}_")]
    if start is None:
        start = FAKE_POSE
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.header = Header()
    goal.trajectory.joint_names = joints
    for i in range(n_points):
        t = duration * i / (n_points - 1)
        p = JointTrajectoryPoint()
        p.positions = [
            start.get(n, FAKE_POSE[n])
            + (targets.get(n, start.get(n, FAKE_POSE[n]))
               - start.get(n, FAKE_POSE[n])) * t / duration
            for n in joints
        ]
        p.time_from_start.sec = int(t)
        p.time_from_start.nanosec = int((t - int(t)) * 1e9)
        goal.trajectory.points.append(p)
    return goal


def wait_future(future, timeout=15.0):
    """executor 在后台线程跑，主线程用 Event 等 future 完成。"""
    ev = threading.Event()
    future.add_done_callback(lambda f: ev.set())
    if not ev.wait(timeout):
        raise TimeoutError("future 超时未完成")
    return future.result()


checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond)))
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def main():
    rclpy.init()
    controller = McArmController()
    fake = FakeMc()
    echo = McStateEcho()
    probe = JointStateProbe()

    left_client = ActionClient(controller, FollowJointTrajectory,
                               "/left_arm_controller/follow_joint_trajectory")
    right_client = ActionClient(controller, FollowJointTrajectory,
                                "/right_arm_controller/follow_joint_trajectory")
    for c, n in ((left_client, "left"), (right_client, "right")):
        if not c.wait_for_server(timeout_sec=5.0):
            print(f"[FAIL] {n} action server 未出现")
            return 1

    executor = MultiThreadedExecutor(num_threads=8)
    for n in (controller, fake, echo, probe):
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 等反馈链路稳定（controller 收到首帧反馈后才会开始发指令）
    t_end = time.monotonic() + 3.0
    while not controller.state_fresh and time.monotonic() < t_end:
        time.sleep(0.05)
    check("MC 反馈到达（controller 就绪）", controller.state_fresh)

    time.sleep(0.5)  # 预热：保持帧应该已经在流
    n_cmd_before = len(fake.cmds)
    t_start_main = time.monotonic()

    # ========== 测试 1：双臂并发轨迹 ==========
    f_left = left_client.send_goal_async(build_goal("left", LEFT_TARGET, 2.0))
    f_right = right_client.send_goal_async(build_goal("right", RIGHT_TARGET, 2.0))
    h_left = wait_future(f_left)
    h_right = wait_future(f_right)
    r_left = h_left.get_result_async()
    r_right = h_right.get_result_async()
    r_left_res = wait_future(r_left)
    r_right_res = wait_future(r_right)

    time.sleep(0.7)  # 低通收敛
    codes = {"left": (r_left_res.result.error_code, r_left_res.result.error_string),
             "right": (r_right_res.result.error_code, r_right_res.result.error_string)}
    check("测试1 双臂 goal 均 SUCCESSFUL",
          all(c[0] == FollowJointTrajectory.Result.SUCCESSFUL for c in codes.values()),
          str(codes))

    # ========== 测试 2：取消（轨迹从当前指令位姿 0.8 出发，与真实规划一致） ==========
    f_slow = left_client.send_goal_async(build_goal(
        "left", {"left_shoulder_pitch_joint": 1.2}, 8.0,
        start={**FAKE_POSE, "left_shoulder_pitch_joint": 0.8}))
    h_slow = wait_future(f_slow)
    time.sleep(1.2)
    cancel_fut = h_slow.cancel_goal_async()
    wait_future(cancel_fut, timeout=5.0)
    r_slow_res = wait_future(h_slow.get_result_async(), timeout=5.0)
    check("测试2 取消后结果为 CANCELED",
          r_slow_res.status == 5,  # 5=STATUS_CANCELED
          f"status={r_slow_res.status} code={r_slow_res.result.error_code}")

    # ========== 测试 3：同侧抢占 ==========
    f_a = right_client.send_goal_async(build_goal(
        "right", {"right_elbow_joint": 0.3}, 8.0,
        start={**FAKE_POSE, "right_elbow_joint": -0.6}))
    h_a = wait_future(f_a)
    time.sleep(0.5)
    # 新轨迹从抢占瞬间的指令位姿出发（与真实重规划一致）：-0.6+0.9*0.5/8=-0.54375
    f_b = right_client.send_goal_async(build_goal(
        "right", {"right_elbow_joint": -0.4}, 1.0,
        start={**FAKE_POSE, "right_elbow_joint": -0.54375}))
    h_b = wait_future(f_b)
    r_a_res = wait_future(h_a.get_result_async())
    r_b_res = wait_future(h_b.get_result_async())
    check("测试3 旧目标被抢占（非 SUCCESSFUL）",
          r_a_res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL,
          f"code={r_a_res.result.error_code}")
    check("测试3 新目标 SUCCESSFUL",
          r_b_res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)

    time.sleep(0.7)  # 低通收敛

    # ========== 测试 4：流式话题接管（联动仲裁） ==========
    # 左臂 8s 慢轨迹执行中，joint_trajectory 话题以 20Hz 注入单点目标
    # （模拟联动模式下 a3_streaming_teleop 默认 backend 的流式遥操作）：
    # action 应被抢占，注入值经同一安全链下发；话题静默后 action 恢复可用。
    stream_pub = controller.create_publisher(
        JointTrajectory, "/left_arm_controller/joint_trajectory", 20)
    left_joints = [n for n in MC_ARM_JOINTS if n.startswith("left_")]
    f_slow4 = left_client.send_goal_async(build_goal(
        "left", {"left_shoulder_pitch_joint": 1.2}, 8.0,
        start={**FAKE_POSE, "left_shoulder_pitch_joint": 0.86}))
    h_slow4 = wait_future(f_slow4)
    time.sleep(0.8)  # 慢轨迹跑 0.8s（q≈0.894）后流式接管
    t_inj_end = time.monotonic() + 1.5
    while time.monotonic() < t_inj_end:
        msg = JointTrajectory()
        msg.joint_names = left_joints
        pt = JointTrajectoryPoint()
        pt.positions = [0.5 if n == "left_shoulder_pitch_joint"
                        else FAKE_POSE[n] for n in left_joints]
        msg.points.append(pt)
        stream_pub.publish(msg)
        time.sleep(0.05)  # 20Hz，每次续期 stream_timeout
    r_slow4_res = wait_future(h_slow4.get_result_async(), timeout=5.0)
    check("测试4 流式接管后 action 被抢占（非 SUCCESSFUL）",
          r_slow4_res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL,
          f"code={r_slow4_res.result.error_code} "
          f"str={r_slow4_res.result.error_string!r}")
    time.sleep(0.5)  # 等流式控制权过期（stream_timeout=0.3s）+ 收敛
    lsp_cmd = fake.cmds[-1][1][MC_ARM_JOINTS.index("left_shoulder_pitch_joint")]
    check("测试4 注入值经安全链下发：左肩 pitch ≈0.5",
          abs(lsp_cmd - 0.5) < 0.05, f"实际 {lsp_cmd:.4f}")
    f_new = left_client.send_goal_async(build_goal(
        "left", {"left_shoulder_pitch_joint": 0.7}, 1.0,
        start={**FAKE_POSE, "left_shoulder_pitch_joint": 0.5}))
    h_new = wait_future(f_new)
    r_new_res = wait_future(h_new.get_result_async())
    check("测试4 流式静默后 action 恢复可用（新 goal SUCCESSFUL）",
          r_new_res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
          f"code={r_new_res.result.error_code}")

    time.sleep(0.7)  # 低通收敛

    # 空闲窗口：所有 goal 结束后单独量 2s——生产形态是独立进程，常态保持帧
    # 必须严守官方 ≤30ms（MC 看门狗盯的就是这个）；执行窗口在上面单进程压测
    # （4 节点挤一个进程，GIL 争抢）只要求 p99 < 50ms。
    t_idle0 = time.monotonic()
    idx_idle0 = len(fake.cmds)
    time.sleep(2.0)
    idle_cmds = list(fake.cmds[idx_idle0:])
    idle_ts = np.array([c[0] for c in idle_cmds])
    idle_gaps = np.diff(idle_ts)
    idle_rate = 1.0 / np.mean(idle_gaps)
    check("空闲窗口平均频率 ≈100Hz", 95.0 < idle_rate < 105.0, f"{idle_rate:.1f} Hz")
    check("空闲窗口 p99 间隔 <=30ms", np.percentile(idle_gaps, 99) < 0.030,
          f"p99={np.percentile(idle_gaps, 99) * 1000:.2f}ms")

    # ========== 指令流断言 ==========
    cmds = list(fake.cmds)
    n_stream = len(cmds) - n_cmd_before
    wall = time.monotonic() - t_start_main
    check("全程持续 100Hz 指令流", n_stream > wall * 100.0 * 0.7,
          f"新增 {n_stream} 帧 / {wall:.1f}s")
    ts = np.array([c[0] for c in cmds])
    gaps = np.diff(ts)
    rate = 1.0 / np.mean(gaps)
    check("全程平均频率 ≈100Hz", 90.0 < rate < 115.0, f"{rate:.1f} Hz")
    check("全程 p99 间隔 <50ms（单进程压测容差，空闲窗口已单独严判）",
          np.percentile(gaps, 99) < 0.050,
          f"p99={np.percentile(gaps, 99) * 1000:.2f}ms")
    pos = np.stack([c[1] for c in cmds])
    check("14 关节", pos.shape[1] == 14)
    dq = np.abs(np.diff(pos, axis=0)).max(axis=1)
    check("相邻速度 <=4 rad/s（含 10% 余量）",
          dq.max() < 4.0 / 100.0 * 1.1, f"max={dq.max() * 100.0:.3f} rad/100Hz帧")
    low = np.array([MC_JOINT_LIMITS[n][0] for n in MC_ARM_JOINTS])
    high = np.array([MC_JOINT_LIMITS[n][1] for n in MC_ARM_JOINTS])
    check("全部指令在官方限位内",
          (pos >= low - 1e-6).all() and (pos <= high + 1e-6).all())
    check("关节名固定顺序正确", list(cmds[0][2].name) == MC_ARM_JOINTS)
    check("velocity/effort 全 0",
          all(v == 0.0 for v in cmds[-1][2].velocity)
          and all(e == 0.0 for e in cmds[-1][2].effort))

    # 终点收敛（最后一个保持帧 vs 各测试终点）
    # 左臂最后目标是测试 4 收尾的 1s 轨迹终点 0.7（流式注入 0.5 -> 0.7）
    # 右臂最后目标 -0.4（测试 3 的 1s 快轨迹）
    last = pos[-1]
    lsp = last[MC_ARM_JOINTS.index("left_shoulder_pitch_joint")]
    rel = last[MC_ARM_JOINTS.index("right_elbow_joint")]
    check("终点收敛：左肩 pitch ≈0.7", abs(lsp - 0.7) < 0.05,
          f"实际 {lsp:.4f}")
    check("终点收敛：右肘 ≈-0.4", abs(rel - (-0.4)) < 0.05, f"实际 {rel:.4f}")

    # 结束后流未断（最后 1s 仍有新帧）
    check("结束后仍发保持帧", ts[-1] > time.monotonic() - 1.0)

    # ========== 回显断言 ==========
    js = probe.latest
    if js is not None:
        check("/joint_states 31 关节", len(js.name) == 31, f"{len(js.name)} 个")
        d = dict(zip(js.name, js.position))
        check("回显手臂=MC 反馈值",
              abs(d["left_shoulder_pitch_joint"] - FAKE_POSE["left_shoulder_pitch_joint"]) < 1e-6
              and abs(d["right_elbow_joint"] - FAKE_POSE["right_elbow_joint"]) < 1e-6)
        check("回显非手臂关节补 0",
              d["left_hip_pitch_joint"] == 0.0 and d["waist_yaw_joint"] == 0.0
              and d["head_yaw_joint"] == 0.0)
    else:
        check("/joint_states 收到过消息", False)

    failed = [n for n, ok in checks if not ok]
    print(f"共 {len(checks)} 项，失败 {len(failed)} 项")
    sys.stdout.flush()

    executor.shutdown()
    for n in (controller, fake, echo, probe):
        n.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
