#!/usr/bin/env python3
"""Pico VR 手柄 -> A3 双臂相对运动遥操作（触发式）。

映射原理（相对运动，不需要头显世界位姿）：
  校准时刻记录  T_hmd_c0（手柄在位姿起点）与 T_world_ee0（机器人当前末端）
  任意时刻增量  D = T_hmd_c0^-1 · T_hmd_c(t)      （手相对起点的 6DOF 位移）
  目标位姿      T_world_ee = T_world_ee0 · D

交互（左右臂对称）：
  第一次按 button_x / button_a  -> 校准该手臂
  之后再按                     -> 以当前增量发送 pose goal 给 move_action

注意：话题给的是"手柄相对头"的位姿，头部运动会耦合进增量；
操作员面向机器人、头部基本不动时影响很小（见 REPRODUCE.md 踩坑与对策）。
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

WORLD_FRAME = "world"

# (规划组, 末端 link, pose 话题, 按键话题)
ARMS = {
    "left": (
        "left_arm",
        "left_wrist_yaw_Link",
        "/pico_left_controller/pose",
        "/pico_left_controller/button_x",
    ),
    "right": (
        "right_arm",
        "right_wrist_yaw_Link",
        "/pico_right_controller/pose",
        "/pico_right_controller/button_a",
    ),
}


def quat_to_rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def rot_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array(
            [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
        )
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q / np.linalg.norm(q)


def pose_to_T(pose):
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    r = quat_to_rot([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def T_to_pose(T):
    pose = PoseStamped()
    q = rot_to_quat(T[:3, :3])
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3]
    return pose


def transform_to_T(tf):
    """geometry_msgs/Transform（translation+rotation）-> 4x4 齐次矩阵。"""
    T = np.eye(4)
    q = tf.rotation
    T[:3, :3] = quat_to_rot([q.x, q.y, q.z, q.w])
    T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return T


def make_pose_goal(group, ee_link, target: PoseStamped):
    """把目标位姿写成 MoveGroup action 的位置+姿态约束（move_group_interface 同款做法）。"""
    msg = MoveGroup.Goal()
    msg.request.group_name = group
    msg.request.num_planning_attempts = 5
    msg.request.allowed_planning_time = 10.0
    msg.request.max_velocity_scaling_factor = 0.3
    msg.request.max_acceleration_scaling_factor = 0.3

    pos = PositionConstraint()
    pos.header.frame_id = target.header.frame_id
    pos.link_name = ee_link
    pos.constraint_region.primitives.append(SolidPrimitive())
    pos.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
    pos.constraint_region.primitives[0].dimensions = [0.005]
    pos.constraint_region.primitive_poses.append(target.pose)
    pos.weight = 1.0

    ori = OrientationConstraint()
    ori.header.frame_id = target.header.frame_id
    ori.link_name = ee_link
    ori.orientation = target.pose.orientation
    ori.absolute_x_axis_tolerance = 0.02
    ori.absolute_y_axis_tolerance = 0.02
    ori.absolute_z_axis_tolerance = 0.02
    ori.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pos)
    c.orientation_constraints.append(ori)
    msg.request.goal_constraints.append(c)
    msg.planning_options.plan_only = False
    msg.planning_options.look_around = False
    msg.planning_options.replan = False
    return msg


class ArmTeleop:
    def __init__(self, node: Node, side: str, shared: dict):
        self.node = node
        self.shared = shared
        self.logger = get_logger(f"arm_teleop.{side}")
        self.group, self.ee_link, pose_topic, button_topic = ARMS[side]
        self.side = side

        self.latest_pose = None
        self.T_hand0 = None
        self.T_ee0 = None
        self.prev_pressed = False

        node.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        node.create_subscription(Bool, button_topic, self.on_button, 10)
        self.client = ActionClient(node, MoveGroup, "move_action")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def on_pose(self, msg):
        self.latest_pose = msg

    def lookup_ee(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, self.ee_link, rclpy.time.Time(), Duration(seconds=0.2)
            )
        except Exception as exc:  # noqa: BLE001
            self.logger.warn(f"TF 查询失败 ({WORLD_FRAME}<-{self.ee_link}): {exc}")
            return None
        return transform_to_T(tf.transform)

    def on_button(self, msg):
        pressed = bool(msg.data)
        if pressed == self.prev_pressed:
            return
        self.prev_pressed = pressed
        if not pressed:
            return

        if self.latest_pose is None:
            self.logger.warn("还没收到手柄 pose，忽略")
            return

        T_hand = pose_to_T(self.latest_pose.pose)
        T_ee = self.lookup_ee()
        if T_ee is None:
            return

        if self.T_hand0 is None:
            self.T_hand0 = T_hand
            self.T_ee0 = T_ee
            self.logger.info(
                f"已校准：手柄起点与 {self.ee_link} 当前位姿绑定，再按一次发送目标"
            )
            return

        D = np.linalg.inv(self.T_hand0) @ T_hand
        target = self.T_ee0 @ D
        goal_pose = T_to_pose(target)
        goal_pose.header.frame_id = WORLD_FRAME
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()

        if not self.client.server_is_ready():
            self.logger.warn("move_action 不可用，忽略本次触发")
            return

        # move_group 不支持并发 goal：两个 goal 同时进入会把 TrajectoryExecutionManager
        # 打出段错误。单执行器回调本身串行，用共享标志互斥即可。
        if self.shared["busy"]:
            self.logger.warn("已有目标在执行中，本次触发忽略（等上一个完成再按）")
            return
        self.shared["busy"] = True

        self.logger.info(
            f"发送目标: pos=({goal_pose.pose.position.x:.3f}, "
            f"{goal_pose.pose.position.y:.3f}, {goal_pose.pose.position.z:.3f})"
        )
        future = self.client.send_goal_async(make_pose_goal(self.group, self.ee_link, goal_pose))
        future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.shared["busy"] = False
            self.logger.error("目标被拒绝")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        self.shared["busy"] = False
        error = future.result().result.error_code.val
        if error == MoveItErrorCodes.SUCCESS:
            self.logger.info("规划+执行成功")
        else:
            self.logger.error(f"失败, MoveItErrorCodes = {error}")


def main():
    rclpy.init()
    node = Node("arm_teleop")
    shared = {"busy": False}
    teleops = [ArmTeleop(node, side, shared) for side in ARMS]
    get_logger("arm_teleop").info(
        "等待按键：左臂 button_x / 右臂 button_a（首次按下=校准，之后=发送目标）"
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
