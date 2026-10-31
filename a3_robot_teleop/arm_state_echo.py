#!/usr/bin/env python3
"""查看 MC 手臂关节状态（机器人端运行）。

订阅 /motion/control/arm_joint_state，以 1Hz 打印 14 个手臂关节的当前角度(度)。
用于：确认 ROS 2 环境/域配置正确、MC 在正常反馈关节状态。

用法（机器人上，先 source ROS 2 环境）：
  python3 arm_state_echo.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Echo(Node):
    def __init__(self):
        super().__init__("arm_state_echo")
        self.latest = None
        self.create_subscription(
            JointState, "/motion/control/arm_joint_state", self.on_js, 10)
        self.create_timer(1.0, self.print_state)
        self.get_logger().info("等待 /motion/control/arm_joint_state ...")

    def on_js(self, msg):
        self.latest = msg

    def print_state(self):
        if self.latest is None:
            self.get_logger().warn("还没收到关节状态（检查 MC 是否运行、ROS_DOMAIN_ID、网络）")
            return
        msg = self.latest
        parts = [f"{n.split('_joint')[0]}={p * 57.29578:7.1f}" for n, p in zip(msg.name, msg.position)]
        line = " ".join(parts[:7]) + "\n" + " ".join(parts[7:])
        print(f"[arm_joint_state] {len(msg.name)} 关节(deg)：\n{line}", flush=True)


def main():
    rclpy.init()
    node = Echo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
