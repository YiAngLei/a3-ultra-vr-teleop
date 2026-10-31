#!/usr/bin/env python3
"""监视外部手臂指令流（机器人端运行，遥操作联调时用）。

订阅 /motion/control/arm_joint_command，每 2 秒统计：
  指令频率、最大相邻间隔、最大关节速度。
官方接口要求：推荐 100Hz、相邻间隔 ≤30ms、速度 ≤4 rad/s——超标会打印 WARN。

用法（机器人上，先 source ROS 2 环境）：
  python3 command_monitor.py
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

GAP_LIMIT = 0.030   # s
SPEED_LIMIT = 4.0   # rad/s


class Monitor(Node):
    def __init__(self):
        super().__init__("arm_command_monitor")
        self.count = 0
        self.last_t = None
        self.last_pos = None
        self.max_gap = 0.0
        self.max_speed = 0.0
        self.window_start = time.monotonic()
        self.create_subscription(
            JointState, "/motion/control/arm_joint_command", self.on_cmd, 50)
        self.create_timer(2.0, self.report)
        self.get_logger().info("监视 /motion/control/arm_joint_command ...")

    def on_cmd(self, msg):
        now = time.monotonic()
        if self.last_t is not None:
            gap = now - self.last_t
            self.max_gap = max(self.max_gap, gap)
            if msg.position and self.last_pos is not None and len(msg.position) == len(self.last_pos):
                speed = max(abs(a - b) for a, b in zip(msg.position, self.last_pos)) / max(gap, 1e-6)
                self.max_speed = max(self.max_speed, speed)
        self.last_t = now
        self.last_pos = list(msg.position)
        self.count += 1

    def report(self):
        elapsed = time.monotonic() - self.window_start
        if self.count == 0:
            self.get_logger().warn("窗口内没有指令（主机端 teleop 是否在运行？）")
            return
        rate = self.count / max(elapsed, 1e-6)
        status = "OK "
        if self.max_gap > GAP_LIMIT or self.max_speed > SPEED_LIMIT:
            status = "WARN"
        print(f"[{status}] 频率={rate:6.1f}Hz 最大间隔={self.max_gap * 1000:6.2f}ms "
              f"最大速度={self.max_speed:5.2f}rad/s （限值: ≤30ms, ≤4rad/s）", flush=True)
        if self.max_gap > GAP_LIMIT:
            self.get_logger().warn(f"相邻指令间隔超 30ms（max={self.max_gap * 1000:.1f}ms），"
                                   "官方提示可能抖动")
        if self.max_speed > SPEED_LIMIT:
            self.get_logger().warn(f"关节速度超 4 rad/s（max={self.max_speed:.2f}）")
        self.count = 0
        self.max_gap = 0.0
        self.max_speed = 0.0
        self.window_start = time.monotonic()


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
