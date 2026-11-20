#!/usr/bin/env python3
"""急停保持：把手臂冻结在当前位置（机器人端运行）。

场景：主机端 teleop 异常退出/网络中断，手臂停在半空——先停掉主机 teleop，
再运行本脚本：读取一次当前关节状态，然后以 100Hz 持续发送"保持当前角度"指令，
让 MC 稳稳停在原位。Ctrl+C 停止发送。

注意：
  - 本接口仅可在 MC MOTION 状态下调用（同官方 arm_joint_command 限制）；
  - 运行前必须停止主机端的流式 teleop，否则两组指令会互相覆盖；
  - 这是软件保持手段，不能替代物理急停。

用法（机器人上，先 source ROS 2 环境）：
  python3 hold_position.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Hold(Node):
    def __init__(self):
        super().__init__("arm_hold_position")
        self.positions = None
        self.pub = self.create_publisher(
            JointState, "/motion/control/arm_joint_command", 10)
        self.create_subscription(
            JointState, "/motion/control/arm_joint_state", self.on_js, 10)
        self.create_timer(0.01, self.tick)  # 100Hz
        self.get_logger().info("等待一帧 /motion/control/arm_joint_state 作为保持目标...")

    def on_js(self, msg):
        if self.positions is None and len(msg.position) == 14:
            self.positions = list(msg.position)
            self.get_logger().info(
                "已锁定保持位姿，开始 100Hz 发送保持指令（Ctrl+C 停止）")

    def tick(self):
        if self.positions is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "hold_position"
        msg.name = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
        ]
        msg.position = list(self.positions)
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Hold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("停止保持发送")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
