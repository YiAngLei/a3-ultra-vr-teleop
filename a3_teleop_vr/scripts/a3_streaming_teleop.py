#!/usr/bin/env python3.12
"""Pico VR 手柄 -> A3 双臂流式遥操作（复刻 OpenArm 逻辑，MoveIt 作 IK 引擎）。

与 a3_moveit_demo/scripts/arm_teleop.py（触发式）的区别：本节点是连续伺服——
握住 grip 期间手臂持续跟随手柄，松开即停，不再有"按一次规划一次"的离散感。

映射逻辑与 openarmx_teleop_vr 的相对模式一致：
  grip > grip_threshold 上升沿：捕获手柄参考位姿 T_hand0 与当前末端 T_ee0
  增量（手柄体坐标系）：D = T_hand0^-1 · T_hand(t)
      位置分量过 axis_matrix 重映射并乘 position_scale_xyz
      姿态分量过 orientation_matrix 共轭重映射
  目标位姿：T_target = T_ee0 · D（pelvis_link 系，即 RobotState 的 model frame）
  每周期：MoveIt IK（当前关节作种子）-> 关节空间步进限幅（慢/快两档，带迟滞）
          -> 单点 JointTrajectory 发给 <side>_arm_controller/joint_trajectory

在 OpenArm 之上增加的保护：
  - 工作空间球限幅（workspace_radius，以捕获点为中心）
  - IK 失败保持上一帧（不发送新目标）
  - pose 流 0.3s 超时自动释放

按键：右手 B = 双臂回零（move_action 规划），左手 Y = 双臂举手，
动作期间流式暂停，grip 按下取消（与 OpenArm 相同语义）。

真机模式（output_backend:=mc_arm）：输出改为智元 MC 官方接口
/motion/control/arm_joint_command（sensor_msgs/JointState，14 关节），
状态反馈改从 /motion/control/arm_joint_state 读取。按官方要求实现：
100Hz 连续流（未跟随时也发保持帧）、相邻指令速度限幅（mc_max_speed，默认
4 rad/s）、低通平滑（mc_lowpass_alpha）、官方关节限位钳制。
真机模式下 B/Y 脚本动作禁用（真机没有 move_action 可规划）。

注意：手柄位姿在 pico_hmd 系下，操作期间转头会耦合进增量（OpenArm 相对模式同样如此）。
"""
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PARAMS_PATH = os.environ.get(
    "A3_PARAMS_FILE",
    str(Path(__file__).resolve().parents[2] / "a3_moveit_demo"
        / "src/a3_moveit_config/config/a3_plan_demo_params.yaml"),
)

ARMS = {
    "left": {
        "group": "left_arm",
        "ee_link": "left_wrist_yaw_Link",
        "pose_topic": "/pico_left_controller/pose",
        "grip_topic": "/pico_left_controller/grip",
        "rate_topic": "/pico_left_controller/rate",
        "cmd_topic": "/left_arm_controller/joint_trajectory",
    },
    "right": {
        "group": "right_arm",
        "ee_link": "right_wrist_yaw_Link",
        "pose_topic": "/pico_right_controller/pose",
        "grip_topic": "/pico_right_controller/grip",
        "rate_topic": "/pico_right_controller/rate",
        "cmd_topic": "/right_arm_controller/joint_trajectory",
    },
}

SLOW_STEP_DEG = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
FAST_STEP_DEG = [20.0, 16.0, 12.0, 12.0, 14.0, 8.0, 12.0]  # OpenArm 快速档原值

# 智元 MC 官方手臂接口常量（固定 14 关节顺序 + 官方限位，见 mc_arm_interface.py）
from mc_arm_interface import MC_ARM_JOINTS, MC_JOINT_LIMITS  # noqa: E402,F401


def quat_to_rot(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(r):
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        q[3] = (r[k, j] - r[j, k]) / s
    return q / np.linalg.norm(q)


def pose_msg_to_T(msg):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    T[:3, 3] = [msg.position.x, msg.position.y, msg.position.z]
    return T


def T_to_pose_msg(T):
    pose = Pose()
    q = rot_to_quat(T[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    return pose


def load_moveit_config(params_path):
    """读取 a3_plan_demo_params.yaml 并按 MoveItCpp 的 schema 改写后交给 MoveItPy。

    MoveItCpp 读 planning_pipelines.pipeline_names（嵌套），move_group 读平铺的
    planning_pipelines，两者不兼容，必须改写（见 REPRODUCE.md 踩坑记录）。
    """
    flat = yaml.safe_load(open(params_path))["/**"]["ros__parameters"]
    names = flat.get("planning_pipelines", ["ompl"])
    if isinstance(names, dict):
        names = names.get("pipeline_names", ["ompl"])
    flat["planning_pipelines"] = {"pipeline_names": names, "namespace": ""}
    return flat


class ArmStream:
    """单臂流式状态机：idle -> streaming ->（松开/超时）idle。"""

    def __init__(self, node, side, cfg):
        self.node = node
        self.side = side
        self.group = cfg["group"]
        self.ee_link = cfg["ee_link"]
        self.logger = node.get_logger()

        self.latest_pose = None
        self.latest_pose_time = 0.0
        self.grip = 0.0
        self.rate = 0.1
        self.streaming = False
        self.T_hand0 = None
        self.T_ee0 = None
        self.ik_fail_streak = 0

        node.create_subscription(PoseStamped, cfg["pose_topic"], self.on_pose, 20)
        node.create_subscription(Float32, cfg["grip_topic"], self.on_grip, 10)
        node.create_subscription(Float32, cfg["rate_topic"], self.on_rate, 10)
        self.cmd_pub = node.create_publisher(JointTrajectory, cfg["cmd_topic"], 20)

        self.joint_names = None  # 首帧 joint_states 后填充

    def on_pose(self, msg):
        self.latest_pose = msg.pose
        self.latest_pose_time = time.monotonic()

    def on_grip(self, msg):
        self.grip = float(msg.data)

    def on_rate(self, msg):
        self.rate = float(msg.data)

    def fresh(self, timeout):
        return self.latest_pose is not None and (time.monotonic() - self.latest_pose_time) <= timeout

    def release(self, reason):
        if self.streaming:
            self.logger.info(f"[{self.side}] 释放（{reason}），手臂保持当前位姿")
        self.streaming = False
        self.T_hand0 = None
        self.T_ee0 = None

    def step(self, teleop):
        """执行一个控制周期；sim 模式直接发轨迹，mc 模式返回 q_cmd 由主循环聚合。

        返回 None（本周期无指令）或 7 维目标关节角 ndarray。
        """
        enabled = self.grip > teleop.grip_threshold
        if not enabled:
            self.release("grip 松开")
            return None
        if not self.fresh(teleop.stream_timeout):
            self.release("pose 流超时")
            return None

        T_hand = pose_msg_to_T(self.latest_pose)
        if not self.streaming:
            self.T_hand0 = T_hand.copy()
            self.T_ee0 = teleop.fk(self.ee_link)
            self.streaming = True
            self.ik_fail_streak = 0
            self.logger.info(
                f"[{self.side}] 已使能：手柄参考与 {self.ee_link} 当前位姿绑定，移动手柄即可跟随"
            )
            return None

        # --- OpenArm 相对模式映射 ---
        D = np.linalg.inv(self.T_hand0) @ T_hand
        p_delta = teleop.axis_matrix @ D[:3, 3] * teleop.position_scale
        R_delta = teleop.orientation_matrix @ D[:3, :3] @ teleop.orientation_matrix.T
        T_target = self.T_ee0.copy()
        T_target[:3, :3] = self.T_ee0[:3, :3] @ R_delta
        T_target[:3, 3] = self.T_ee0[:3, 3] + self.T_ee0[:3, :3] @ p_delta

        # 工作空间球限幅（OpenArm 没有，这里防 IK 追飞）
        offset = T_target[:3, 3] - self.T_ee0[:3, 3]
        dist = float(np.linalg.norm(offset))
        if dist > teleop.workspace_radius:
            T_target[:3, 3] = self.T_ee0[:3, 3] + offset * (teleop.workspace_radius / dist)

        # --- IK（当前状态作种子，小步进局部收敛） ---
        state = teleop.build_state()
        t0 = time.monotonic()
        ok = state.set_from_ik(self.group, T_to_pose_msg(T_target), self.ee_link, teleop.ik_timeout)
        if not ok:
            self.ik_fail_streak += 1
            if self.ik_fail_streak in (1, 20, 100):
                self.logger.warn(f"[{self.side}] IK 失败 x{self.ik_fail_streak}，保持上一帧")
            return None
        self.ik_fail_streak = 0
        q_target = np.asarray(state.get_joint_group_positions(self.group), dtype=np.float64)
        q_current = np.asarray(teleop.current_group_positions(self.group), dtype=np.float64)

        # --- OpenArm 关节步进限幅：|误差|>阈值才限幅，小误差不碰（迟滞） ---
        fast = self.rate >= 0.999
        max_step = np.deg2rad(teleop.fast_step_rad if fast else teleop.slow_step_rad)
        dq = q_target - q_current
        mask = np.abs(dq) > teleop.step_threshold_rad
        dq = np.where(mask, np.clip(dq, -max_step, max_step), dq)
        q_cmd = q_current + dq

        if not teleop.mc_mode:
            traj = JointTrajectory()
            traj.header.stamp = self.node.get_clock().now().to_msg()
            traj.joint_names = list(self.joint_names)
            point = JointTrajectoryPoint()
            point.positions = q_cmd.tolist()
            point.time_from_start = Duration(seconds=1.0 / teleop.control_rate).to_msg()
            traj.points.append(point)
            self.cmd_pub.publish(traj)
        return q_cmd


class StreamingTeleop(Node):
    def __init__(self, **kwargs):
        # kwargs 透传给 rclpy.Node（自测用它覆盖 output_backend/control_rate）
        super().__init__("a3_streaming_teleop", **kwargs)

        # --- 可调参数 ---
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("grip_threshold", 0.5)
        self.declare_parameter("stream_timeout", 0.3)
        self.declare_parameter("ik_timeout", 0.01)
        self.declare_parameter("workspace_radius", 0.5)
        self.declare_parameter("position_scale_xyz", [1.0, 1.0, 0.9])
        self.declare_parameter("slow_step_deg", SLOW_STEP_DEG)
        self.declare_parameter("fast_step_deg", FAST_STEP_DEG)
        # 输出后端：sim_jtc = ros2_control 仿真（默认）；mc_arm = 智元 MC 真机接口
        self.declare_parameter("output_backend", "sim_jtc")
        self.declare_parameter("mc_cmd_topic", "/motion/control/arm_joint_command")
        self.declare_parameter("mc_state_topic", "/motion/control/arm_joint_state")
        self.declare_parameter("mc_max_speed", 4.0)       # rad/s（官方要求 ≤4）
        self.declare_parameter("mc_lowpass_alpha", 0.5)   # 低通 y=(1-a)y_prev+a*x
        p = self.get_parameter
        self.control_rate = float(p("control_rate").value)
        self.grip_threshold = float(p("grip_threshold").value)
        self.stream_timeout = float(p("stream_timeout").value)
        self.ik_timeout = float(p("ik_timeout").value)
        self.workspace_radius = float(p("workspace_radius").value)
        self.position_scale = np.asarray(p("position_scale_xyz").value, dtype=np.float64)
        # OpenArm 默认 gripper 模式：axis/orientation 矩阵均为恒等
        self.axis_matrix = np.eye(3)
        self.orientation_matrix = np.eye(3)
        self.slow_step_rad = np.asarray(p("slow_step_deg").value, dtype=np.float64)
        self.fast_step_rad = np.asarray(p("fast_step_deg").value, dtype=np.float64)
        # 迟滞阈值 = 快速档步进（OpenArm 同款：小误差不限幅）
        self.step_threshold_rad = self.fast_step_rad.copy()
        # 真机（MC）模式
        self.mc_mode = str(p("output_backend").value) == "mc_arm"
        self.mc_max_speed = float(p("mc_max_speed").value)
        self.mc_lowpass_alpha = min(1.0, max(0.01, float(p("mc_lowpass_alpha").value)))
        self.mc_limits_low = np.array([MC_JOINT_LIMITS[n][0] for n in MC_ARM_JOINTS])
        self.mc_limits_high = np.array([MC_JOINT_LIMITS[n][1] for n in MC_ARM_JOINTS])
        self.mc_prev_sent = None  # 14 维，上一周期实际下发的角度（低通+限速后的）
        if self.mc_mode and abs(self.control_rate - 100.0) > 1e-6:
            self.get_logger().warn(
                f"mc_arm 模式推荐 control_rate:=100（当前 {self.control_rate:.0f}），"
                "且需保证相邻指令间隔 ≤30ms")

        # --- MoveIt ---
        config = load_moveit_config(PARAMS_PATH)
        self.get_logger().info("构造 MoveItPy（复用 a3_plan_demo_params.yaml）...")
        self.moveit_py = MoveItPy(node_name="a3_streaming_teleop_moveit", config_dict=config)
        self.robot_model = self.moveit_py.get_robot_model()
        self.get_logger().info(f"model_frame = {self.robot_model.model_frame}")
        self.joint_names_all = []
        self.group_joints = {}
        for gname in self.robot_model.joint_model_group_names:
            names = list(self.robot_model.get_joint_model_group(gname).joint_model_names)
            self.group_joints[gname] = names
        self.joint_states = {}
        self.js_received = False
        state_topic = (str(p("mc_state_topic").value) if self.mc_mode else "/joint_states")
        self.create_subscription(JointState, state_topic, self.on_joint_states, 50)
        if self.mc_mode:
            self.mc_cmd_pub = self.create_publisher(
                JointState, str(p("mc_cmd_topic").value), 10)
        else:
            self.mc_cmd_pub = None

        # --- 双臂与按键 ---
        self.arms = {side: ArmStream(self, side, cfg) for side, cfg in ARMS.items()}
        self.create_subscription(Bool, "/pico_right_controller/button_b", self.on_button_b, 10)
        self.create_subscription(Bool, "/pico_left_controller/button_y", self.on_button_y, 10)

        # --- move_action（B/Y 脚本动作，双臂顺序执行） ---
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.script_goal = None      # 正在执行的脚本 goal handle
        self.script_kind = None      # home / hands_up
        self.script_group = None     # 当前执行中的规划组
        self.script_cancelled = False
        self.pending_script = None
        self.prev_button_b = False
        self.prev_button_y = False

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self._loop_errors = 0
        mode_desc = ("真机 MC 接口" if self.mc_mode else "仿真 ros2_control")
        self.get_logger().info(
            f"流式遥操作就绪：{self.control_rate:.0f} Hz [{mode_desc}]，"
            f"握 grip(>{self.grip_threshold}) 使能，"
            + ("mc_arm 模式下 B/Y 脚本动作已禁用" if self.mc_mode
               else "右手 B=回零 左手 Y=举手")
        )

    # ---------- 状态 ----------
    def on_joint_states(self, msg):
        for n, pos in zip(msg.name, msg.position):
            self.joint_states[n] = float(pos)
        if not self.js_received:
            self.js_received = True
            for arm in self.arms.values():
                arm.joint_names = self.group_joints[arm.group]

    def build_state(self):
        # mc 模式只有 14 个手臂关节有反馈，其余关节按 0 处理（不影响手臂 IK）
        st = RobotState(self.robot_model)
        for gname, names in self.group_joints.items():
            st.set_joint_group_positions(gname, [self.joint_states.get(n, 0.0) for n in names])
        st.update()
        return st

    def fk(self, ee_link):
        return self.build_state().get_global_link_transform(ee_link)

    def current_group_positions(self, group):
        return [self.joint_states.get(n, 0.0) for n in self.group_joints[group]]

    # ---------- 真机（MC）14 关节聚合输出 ----------
    def publish_mc_command(self, q_cmds):
        """把双臂本周期目标聚合成 14 关节 JointState，经限位/限速/低通后发 MC。

        任何时候都保持发送（未跟随时发保持帧），保证 100Hz 连续流、指令顺滑。
        q_cmds: {side: 7 维 ndarray 或 None}
        """
        measured = np.array([self.joint_states.get(n, 0.0) for n in MC_ARM_JOINTS])
        if self.mc_prev_sent is None:
            target = measured.copy()
        else:
            target = self.mc_prev_sent.copy()
        for side, q in q_cmds.items():
            if q is None:
                continue
            names = self.group_joints[ARMS[side]["group"]]
            for name, val in zip(names, q):
                target[MC_ARM_JOINTS.index(name)] = val

        # 官方关节限位
        target = np.clip(target, self.mc_limits_low, self.mc_limits_high)
        # 相邻指令速度限幅（官方 ≤4 rad/s）
        if self.mc_prev_sent is not None:
            max_delta = self.mc_max_speed / self.control_rate
            delta = np.clip(target - self.mc_prev_sent, -max_delta, max_delta)
            target = self.mc_prev_sent + delta
        # 低通平滑（官方建议）
        if self.mc_prev_sent is not None:
            smooth = self.mc_prev_sent + self.mc_lowpass_alpha * (target - self.mc_prev_sent)
        else:
            smooth = target
        self.mc_prev_sent = smooth.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "a3_teleop_vr"
        msg.name = list(MC_ARM_JOINTS)
        msg.position = smooth.tolist()
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        self.mc_cmd_pub.publish(msg)

    # ---------- 按键脚本（B=回零 Y=举手） ----------
    def on_button_b(self, msg):
        pressed = bool(msg.data)
        if pressed and not self.prev_button_b:
            self.pending_script = "home"
        self.prev_button_b = pressed

    def on_button_y(self, msg):
        pressed = bool(msg.data)
        if pressed and not self.prev_button_y:
            self.pending_script = "hands_up"
        self.prev_button_y = pressed

    def script_targets(self, kind, group):
        """某个规划组的目标构型（关节名 -> 弧度）。
        home=全零；hands_up=肩抬负向（抬臂）+ 屈肘（FK 实测：pitch=-1.4 时腕部 z 最高；
        肘 1.745 + 其余全零会与髋碰撞，OMPL 报 goal 不可采样）。
        """
        names = self.group_joints[group]
        targets = {n: 0.0 for n in names}
        if kind == "hands_up":
            side = "left" if group == "left_arm" else "right"
            jmg = self.robot_model.get_joint_model_group(group)
            active = jmg.active_joint_model_names
            bounds = jmg.active_joint_model_bounds
            elbow = f"{side}_elbow_joint"
            elbow_upper = float(bounds[active.index(elbow)][0].max_position)
            targets[f"{side}_shoulder_pitch_joint"] = -1.2
            targets[elbow] = min(1.2, elbow_upper)
        return targets

    def send_script_goal(self, kind, group):
        targets = self.script_targets(kind, group)
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        c = Constraints()
        for name, val in targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.on_script_accepted)

    def on_script_accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"脚本动作 {self.script_kind}[{self.script_group}] 被拒绝")
            self.script_goal = None
            self.script_kind = None
            self.script_group = None
            return
        self.script_goal = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_script_done)

    def on_script_done(self, future):
        error = future.result().result.error_code.val
        self.get_logger().info(
            f"脚本动作 {self.script_kind}[{self.script_group}] 结束, code={error}"
            f"{'（已取消）' if self.script_cancelled else ''}"
        )
        self.script_goal = None
        done_kind, done_group, was_cancelled = self.script_kind, self.script_group, self.script_cancelled
        self.script_kind = None
        self.script_group = None
        self.script_cancelled = False
        if not was_cancelled and done_group == "left_arm":
            # 左臂正常完成后接着右臂，保证双臂都到位
            self.script_kind = done_kind
            self.script_group = "right_arm"
            self.send_script_goal(done_kind, self.script_group)

    def cancel_script(self, reason):
        if self.script_goal is not None and not self.script_cancelled:
            self.get_logger().info(f"grip 按下，取消脚本动作（{reason}）")
            self.script_goal.cancel_goal_async()
            self.script_cancelled = True
        self.pending_script = None

    # ---------- 主循环 ----------
    def control_loop(self):
        try:
            self._control_loop()
        except Exception:
            # 单 tick 异常只记日志不杀节点（异常穿出定时器回调会让进程直接退出）
            self._loop_errors += 1
            if self._loop_errors == 1 or self._loop_errors % 200 == 0:
                import traceback

                self.get_logger().error(
                    f"control_loop 第 {self._loop_errors} 次异常：\n{traceback.format_exc()}"
                )

    def _control_loop(self):
        if not self.js_received:
            return

        if self.mc_mode:
            # 真机模式：B/Y 脚本动作禁用（真机没有 move_action），纯手动流式
            if self.pending_script is not None:
                self.pending_script = None
                self.get_logger().warn("mc_arm 模式禁用 B/Y 脚本动作（真机无 move_action 规划栈）")
            q_cmds = {side: arm.step(self) for side, arm in self.arms.items()}
            self.publish_mc_command(q_cmds)
            return

        # grip 任意一侧按下时取消脚本动作（OpenArm 同款语义）
        manual = any(arm.grip > self.grip_threshold for arm in self.arms.values())
        if manual:
            self.cancel_script("手动接管")

        # 脚本动作期间双臂停流，让 move_action 独占控制器
        if self.script_goal is not None:
            return
        if self.pending_script is not None:
            kind = self.pending_script
            self.pending_script = None
            if not self.move_client.server_is_ready():
                self.get_logger().warn("move_action 不可用，忽略本次脚本动作")
                return
            self.script_kind = kind
            self.script_group = "left_arm"
            self.get_logger().info(f"开始脚本动作：{kind}（先左臂后右臂）")
            self.send_script_goal(kind, self.script_group)

        for arm in self.arms.values():
            arm.step(self)


def main():
    rclpy.init()
    node = StreamingTeleop()
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # 定时器回调里的异常会从这里冒出来；必须打印 traceback 再退出，
        # 否则 finally 的 os._exit 会把错误痕迹全部吞掉（踩过的坑）
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        node.moveit_py.shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)  # 跳过 wrapper GC：MoveItPy 析构会 rclcpp::shutdown 导致重复释放


if __name__ == "__main__":
    main()
