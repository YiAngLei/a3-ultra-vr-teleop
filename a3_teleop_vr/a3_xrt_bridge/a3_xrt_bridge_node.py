#!/usr/bin/env python3.12
"""XRoboToolkit SDK -> ROS 2 桥接节点（替代 a3_teleop_bridge 的 OpenArmX UDP 方案）。

数据源：XRoboToolkit PC Service（/opt/apps/roboticsservice，需先启动）经官方
xrobotoolkit_sdk 提供的**房间锚定**手柄/头显位姿——与 XRoboToolkit 双 UR5 官方案例
（XRoboToolkit-Teleop-Sample-Python）完全同源，遥操作映射因此可用世界系增量
+ 默认 world_rotation（官方 R_HEADSET_TO_WORLD）直接工作。

发布话题与 a3_teleop_bridge（OpenArmX 兼容桥）**完全同名**，a3_streaming_teleop
及 fake_pico_publisher 无需任何改动即可切换本桥：

  /pico_<side>_controller/{pose,grip,trigger,rate}
  /pico_right_controller/button_b、/pico_left_controller/button_y

与 OpenArmX 桥的语义差异：
- rate 调速档：SDK 无档位概念，默认用**扳机当快进键**（trigger>0.5 -> 1.0 快档，
  否则 0.1 慢档，左右手各自独立）；-p rate_from_trigger:=false 可固定慢档。
- 位姿参考系：SDK 房间锚定（转头不改手柄读数），frame_id 默认 "xrt"
  （旧桥是 pico_hmd 头显系，两者不同源，勿混用旧桥标定结论）。
- SDK 会偶发吐零四元数：模长 < 0.5 的帧直接跳过（该周期不发布该手）。

用法（先启动 PC Service：bash /opt/apps/roboticsservice/runService.sh）：
  /usr/bin/python3.12 a3_xrt_bridge_node.py
  # 或随 launch：ros2 launch .../teleop_vr.launch.py bridge:=true bridge_impl:=xrt
依赖：xrobotoolkit-sdk（pip，cp310-abi3 wheel，python3.12 可用：
      ~/.local/lib/python3.12/site-packages）；ROS 2 rclpy 用系统 python3.12。
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster

import xrobotoolkit_sdk as xrt

HANDS = ("left", "right")


class XrtBridge(Node):
    """XRoboToolkit SDK -> /pico_* 话题桥。"""

    def __init__(self):
        super().__init__("a3_xrt_bridge")
        self.declare_parameter("frame_id", "xrt")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rate_from_trigger", True)
        self.declare_parameter("loop_hz", 100.0)
        self._frame_id = self.get_parameter("frame_id").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._rate_from_trigger = bool(self.get_parameter("rate_from_trigger").value)

        try:
            xrt.init()
        except Exception as exc:  # PC Service 未启动时给出可操作提示
            raise RuntimeError(
                "xrobotoolkit_sdk 初始化失败：请先启动 PC Service：\n"
                "  bash /opt/apps/roboticsservice/runService.sh\n"
                f"原始错误: {exc}"
            ) from exc
        self.get_logger().info(
            "XRoboToolkit SDK 已连接（位姿为房间锚定系，frame_id="
            f"{self._frame_id}）"
        )

        self._pose_pub = {
            s: self.create_publisher(PoseStamped, f"pico_{s}_controller/pose", 20)
            for s in HANDS
        }
        self._grip_pub = {
            s: self.create_publisher(Float32, f"pico_{s}_controller/grip", 10)
            for s in HANDS
        }
        self._trigger_pub = {
            s: self.create_publisher(Float32, f"pico_{s}_controller/trigger", 10)
            for s in HANDS
        }
        self._rate_pub = {
            s: self.create_publisher(Float32, f"pico_{s}_controller/rate", 10)
            for s in HANDS
        }
        self._btn_pub = {
            "right": self.create_publisher(Bool, "pico_right_controller/button_b", 10),
            "left": self.create_publisher(Bool, "pico_left_controller/button_y", 10),
        }
        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        loop_hz = float(self.get_parameter("loop_hz").value)
        self.create_timer(1.0 / loop_hz, self._tick)
        self._seen_device = False

    # ------------------------------------------------------------------ 主循环
    def _tick(self):
        pose_fn = {"left": xrt.get_left_controller_pose,
                   "right": xrt.get_right_controller_pose}
        grip_fn = {"left": xrt.get_left_grip, "right": xrt.get_right_grip}
        trigger_fn = {"left": xrt.get_left_trigger, "right": xrt.get_right_trigger}
        stamp_ns = xrt.get_time_stamp_ns()
        stamp = self.get_clock().now().to_msg() if stamp_ns <= 0 else \
            rclpy.time.Time(nanoseconds=stamp_ns).to_msg()

        if not self._seen_device and stamp_ns > 0:
            self._seen_device = True
            self.get_logger().info("首次收到 XRoboToolkit 数据流")

        for side in HANDS:
            raw = pose_fn[side]()
            grip = float(grip_fn[side]())
            trig = float(trigger_fn[side]())
            self._grip_pub[side].publish(Float32(data=grip))
            self._trigger_pub[side].publish(Float32(data=trig))
            if self._rate_from_trigger:
                self._rate_pub[side].publish(
                    Float32(data=1.0 if trig > 0.5 else 0.1))
            else:
                self._rate_pub[side].publish(Float32(data=0.1))

            if raw is None:
                continue
            pos = raw[:3]
            quat = raw[3:7]  # SDK: [x, y, z, qx, qy, qz, qw]
            norm2 = sum(float(v) * float(v) for v in quat)
            if norm2 < 0.25:  # 偶发零四元数跳帧
                continue
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
                (float(v) for v in pos)
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = \
                (float(v) for v in quat)
            self._pose_pub[side].publish(msg)

            if self._tf is not None:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = self._frame_id
                tf.child_frame_id = f"xrt_{side}_controller"
                tf.transform.translation.x = msg.pose.position.x
                tf.transform.translation.y = msg.pose.position.y
                tf.transform.translation.z = msg.pose.position.z
                tf.transform.rotation = msg.pose.orientation
                self._tf.sendTransform(tf)

        self._btn_pub["right"].publish(Bool(data=bool(xrt.get_B_button())))
        self._btn_pub["left"].publish(Bool(data=bool(xrt.get_Y_button())))

    def destroy_node(self):
        try:
            xrt.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = XrtBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
