#!/usr/bin/env python3
"""机器人端：/teleop/locomotion_cmd -> MC 行走指令（官方 RosMsgWrapper JSON）。

数据链：PC 端 locomotion_teleop.py 发 /teleop/locomotion_cmd（Float32MultiArray
[fwd, lat, ang]）-> 域桥 locomotion 管道（42->232）-> 本脚本 -> MC。

发布目标：/motion/control/locomotion_velocity/pb_3Aaimdk_2Eprotocol_2E
MotionControlLocomotionVelocityChannel（ros2_plugin_proto/msg/RosMsgWrapper，
JSON 格式与官方示例 walk.py 完全一致，50Hz，best_effort）。

安全设计：
  - 首次收到指令前**不发布任何东西**（不碰 MC）
  - 指令静默超过 stale_timeout（默认 0.5s）自动发零 = 站立（持续流）
  - 非零/零切换打印日志（哪天机器人自己动了，第一时间可查）
  - 仅 MOTION 状态下 MC 才会执行，其他状态拒收（失效安全）

运行前提（start_domain_bridge.sh loco 已代劳）：
  source /opt/ros/jazzy/setup.bash
  source .../ros2_plugin_proto/local_setup.bash
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from ros2_plugin_proto.msg import RosMsgWrapper
from std_msgs.msg import Float32MultiArray

LOCOMOTION_TOPIC = (
    "/motion/control/locomotion_velocity/"
    "pb_3Aaimdk_2Eprotocol_2EMotionControlLocomotionVelocityChannel")


class LocomotionRelay(Node):
    def __init__(self):
        super().__init__("locomotion_relay")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("stale_timeout", 0.5)
        self._stale = float(self.get_parameter("stale_timeout").value)
        self._cmd = None            # 最近一帧 [fwd, lat, ang]
        self._last_rx = 0.0
        self._streaming = False     # 当前是否在发非零指令

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(RosMsgWrapper, LOCOMOTION_TOPIC, qos)
        self.create_subscription(
            Float32MultiArray, "/teleop/locomotion_cmd", self._on_cmd, 10)
        self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"行走转发就绪：/teleop/locomotion_cmd -> {LOCOMOTION_TOPIC} "
            f"@ {self.get_parameter('rate_hz').value}Hz（等待首个指令后才发布）")

    def _on_cmd(self, msg: Float32MultiArray):
        self._cmd = list(msg.data)[:3]
        self._last_rx = self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clamp(v):
        return max(-1.0, min(1.0, float(v)))

    def _tick(self):
        if self._cmd is None:
            return  # 从未收到指令：不碰 MC
        now = self.get_clock().now().nanoseconds * 1e-9
        stale = (now - self._last_rx) > self._stale
        if stale or len(self._cmd) < 3:
            fwd = lat = ang = 0.0
        else:
            fwd, lat, ang = (self._clamp(v) for v in self._cmd)
        nonzero = not stale and any(abs(v) > 1e-6 for v in (fwd, lat, ang))
        if nonzero != self._streaming:
            self._streaming = nonzero
            if nonzero:
                self.get_logger().info(
                    f"开始转发行走指令 fwd={fwd:.2f} lat={lat:.2f} ang={ang:.2f}")
            else:
                self.get_logger().info("行走指令回零（站立）")

        payload = {"data": {
            "mode": 0,
            "forward_velocity": round(fwd, 4),
            "lateral_velocity": round(lat, 4),
            "angular_velocity": round(ang, 4),
        }}
        msg = RosMsgWrapper()
        msg.serialization_type = "json"
        msg.data = [bytes([b]) for b in json.dumps(payload).encode()]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
