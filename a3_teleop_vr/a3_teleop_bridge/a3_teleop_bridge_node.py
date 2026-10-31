#!/usr/bin/env python3.12
"""A3 Pico VR 手柄 UDP 桥接节点（自研兼容实现，接收 Pico 端 App 的报文）。

收听 Pico 头显上 VR 遥操作 App 发来的 UDP 文本报文（默认 0.0.0.0:5100），
解析后发布为 ROS 2 话题，供 a3_streaming_teleop.py 等节点消费。

实现说明：本节点按报文格式（事实性协议信息）编写，代码为全新实现，
未复用 OpenArmX 的代码（其仓库以 CC BY-NC-SA 4.0 许可，见本目录 README.md）。

报文格式（空白分隔的文本，单报文 < 512 字节）：
  LEFT | L  /  RIGHT | R     相对位姿：
      px py pz qx qy qz qw [trigger] [grip] [a] [b] [x] [y] [rate|ts] [ts]
  LEFT_ABS / RIGHT_ABS       绝对位姿（字段同上）
  HEAD                       头显位姿：px py pz qx qy qz qw [ts]
  JOY LEFT|RIGHT x y [ts]
  BTN LEFT|RIGHT JOYSTICK 0|1 ts
  CFG POSE_MODE 0|1 ts       （0=gripper，1=hand）
  MODE <字符串> [ts]
  CALIBRATE_DONE [ts]

发布话题（相对量）：pico_{left,right}_controller/{pose,trigger,grip,rate,
                      joystick,button_joystick}
                  右手报文附带 button_a/button_b，左手报文附带 button_x/button_y
发布话题（绝对量）：vr/{left,right}/{pose_absolute,trigger,grip}、vr/rate、
                  vr/{right,left}/{button_a,button_b,button_x,button_y}、
                  vr/head/pose、vr/control_mode、vr/calibrate_done、vr/controller_pose_mode

语义约定：
  - rate 是全局档位（0.1 或 1.0），任一相对位姿报文都会同时刷新双手的 rate 话题；
  - 位姿报文的时间戳：报文自带 ns 时间戳则采用，否则取本机当前时间；
  - 报文头 frame_id 固定为参数 frame_id（默认 pico_hmd）。

用法：
  /usr/bin/python3.12 a3_teleop_bridge_node.py
  # 或随 launch 启动：ros2 launch .../teleop_vr.launch.py bridge:=true
"""
import socket
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from tf2_ros import TransformBroadcaster

HANDS = ("left", "right")

# 单手话题后缀 -> 消息类型（相对量两套：pico_left_controller/* / pico_right_controller/*）
PICO_MSG_BY_SUFFIX = {
    "pose": PoseStamped,
    "trigger": Float32,
    "grip": Float32,
    "rate": Float32,
    "joystick": Float32MultiArray,
    "button_joystick": Bool,
    "button_a": Bool,
    "button_b": Bool,
    "button_x": Bool,
    "button_y": Bool,
}
PICO_SUFFIXES = {
    "left": ("pose", "trigger", "grip", "rate", "joystick",
             "button_joystick", "button_x", "button_y"),
    "right": ("pose", "trigger", "grip", "rate", "joystick",
              "button_joystick", "button_a", "button_b"),
}

# 绝对量话题（vr/*），含头显、模式与标定完成通知
VR_TOPICS = {
    "left/pose_absolute": PoseStamped,
    "right/pose_absolute": PoseStamped,
    "left/trigger": Float32,
    "right/trigger": Float32,
    "left/grip": Float32,
    "right/grip": Float32,
    "rate": Float32,
    "right/button_a": Bool,
    "right/button_b": Bool,
    "left/button_x": Bool,
    "left/button_y": Bool,
    "head/pose": PoseStamped,
    "control_mode": String,
    "calibrate_done": Bool,
    "controller_pose_mode": String,
}


class Sample:
    """一条解析后的报文。"""

    __slots__ = (
        "kind", "hand", "position", "orientation", "trigger", "grip",
        "button_a", "button_b", "button_x", "button_y", "rate",
        "joystick_x", "joystick_y", "button_name", "button_pressed",
        "mode_text", "timestamp_ns",
    )

    def __init__(self, kind):
        self.kind = kind
        self.hand = "left"
        self.position = (0.0, 0.0, 0.0)
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.trigger = 0.0
        self.grip = 0.0
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.rate = 0.1
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.button_name = ""
        self.button_pressed = False
        self.mode_text = ""
        self.timestamp_ns = 0


def _leading_floats(tokens):
    """从头开始连续解析浮点数，遇到非数值 token 即停。"""
    values = []
    for token in tokens:
        try:
            values.append(float(token))
        except ValueError:
            break
    return values


def _parse_hand_body(tokens, sample):
    """解析位姿报文公共字段：位置 + 四元数 + [扳机/握把/按键/档位/时间戳]。"""
    if len(tokens) < 7:
        return False
    try:
        head = [float(v) for v in tokens[:7]]
    except ValueError:
        return False
    sample.position = tuple(head[0:3])
    sample.orientation = tuple(head[3:7])

    # 其余字段全部可选，缺省即 0 / 松开 / 0.1 档
    values = _leading_floats(tokens[7:])
    if values:
        sample.trigger = values[0]
    if len(values) > 1:
        sample.grip = values[1]
    if len(values) > 2:
        sample.button_a = values[2] != 0
    if len(values) > 3:
        sample.button_b = values[3] != 0
    if len(values) > 4:
        sample.button_x = values[4] != 0
    if len(values) > 5:
        sample.button_y = values[5] != 0

    # 第 7 个数值：0.1/1.0 视为档位，否则视为时间戳（档位回落 0.1）
    if len(values) > 6:
        if values[6] == 0.1 or values[6] == 1.0:
            sample.rate = values[6]
            if len(values) > 7:
                sample.timestamp_ns = int(values[7])
        else:
            sample.timestamp_ns = int(values[6])
    return True


def _parse_int(token):
    try:
        return int(float(token))
    except (TypeError, ValueError):
        return 0


def parse_datagram(text):
    """解析一条报文；成功返回 Sample，失败返回 None。"""
    tokens = text.split()
    if not tokens:
        return None
    head = tokens[0].upper()
    rest = tokens[1:]

    if head in ("LEFT", "L", "RIGHT", "R"):
        sample = Sample("relative_hand")
        sample.hand = "right" if head in ("RIGHT", "R") else "left"
        return sample if _parse_hand_body(rest, sample) else None

    if head in ("LEFT_ABS", "RIGHT_ABS"):
        sample = Sample("absolute_hand")
        sample.hand = "right" if head.startswith("RIGHT") else "left"
        return sample if _parse_hand_body(rest, sample) else None

    if head == "HEAD":
        if len(rest) < 7:
            return None
        try:
            numbers = [float(v) for v in rest[:7]]
        except ValueError:
            return None
        sample = Sample("head")
        sample.position = tuple(numbers[0:3])
        sample.orientation = tuple(numbers[3:7])
        if len(rest) > 7:
            sample.timestamp_ns = _parse_int(rest[7])
        return sample

    if head == "JOY" and len(rest) >= 3 and rest[0].upper() in ("LEFT", "RIGHT"):
        try:
            sample = Sample("joystick")
            sample.hand = rest[0].lower()
            sample.joystick_x = float(rest[1])
            sample.joystick_y = float(rest[2])
        except ValueError:
            return None
        if len(rest) > 3:
            sample.timestamp_ns = _parse_int(rest[3])
        return sample

    if head == "BTN" and len(rest) >= 4 and rest[0].upper() in ("LEFT", "RIGHT"):
        sample = Sample("button")
        sample.hand = rest[0].lower()
        sample.button_name = rest[1]
        sample.button_pressed = _parse_int(rest[2]) != 0
        sample.timestamp_ns = _parse_int(rest[3])
        return sample

    if head == "CFG" and len(rest) >= 3 and rest[0].upper() == "POSE_MODE":
        if rest[1] not in ("0", "1"):
            return None
        sample = Sample("pose_source_config")
        sample.mode_text = "hand" if rest[1] == "1" else "gripper"
        sample.timestamp_ns = _parse_int(rest[2])
        return sample

    if head == "MODE" and rest:
        sample = Sample("mode")
        sample.mode_text = rest[0]
        if len(rest) > 1:
            sample.timestamp_ns = _parse_int(rest[1])
        return sample

    if head == "CALIBRATE_DONE":
        sample = Sample("calibrate_done")
        if rest:
            sample.timestamp_ns = _parse_int(rest[0])
        return sample

    return None


class PicoUdpBridge(Node):
    """UDP -> ROS 2 话题桥接节点。"""

    def __init__(self, **kwargs):
        # kwargs 透传给 rclpy.Node（测试用它覆盖 listen_address/listen_port）
        super().__init__("a3_teleop_bridge", **kwargs)
        self._listen_address = self.declare_parameter("listen_address", "0.0.0.0").value
        self._listen_port = int(self.declare_parameter("listen_port", 5100).value)
        self._frame_id = self.declare_parameter("frame_id", "pico_hmd").value
        self._publish_tf = bool(self.declare_parameter("publish_tf", False).value)

        # pico_{hand}_controller/* 话题（相对量）
        self._pico = {}
        for hand in HANDS:
            self._pico[hand] = {
                suffix: self.create_publisher(
                    PICO_MSG_BY_SUFFIX[suffix], f"pico_{hand}_controller/{suffix}", 10)
                for suffix in PICO_SUFFIXES[hand]
            }
        # vr/* 话题（绝对量与全局状态）
        self._vr = {
            key: self.create_publisher(msg_type, f"vr/{key}", 10)
            for key, msg_type in VR_TOPICS.items()
        }

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._seen_kinds = set()
        self._bad_count = 0
        self._last_bad_log = 0.0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setblocking(False)
        try:
            self._sock.bind((self._listen_address, self._listen_port))
        except OSError as exc:
            self._sock.close()
            raise RuntimeError(
                f"无法绑定 UDP {self._listen_address}:{self._listen_port}（{exc}）；"
                "请检查端口是否被其他 bridge 占用") from exc

        self.create_timer(0.01, self._drain_socket)  # 100 Hz 轮询收包
        self.get_logger().info(
            f"A3 Pico UDP 桥接已启动：{self._listen_address}:{self._listen_port} "
            f"(frame_id={self._frame_id}, tf={'开' if self._publish_tf else '关'})")

    # ------------------------------------------------------------------ 收包
    def _drain_socket(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(512)
            except (BlockingIOError, OSError):
                return
            text = data.decode("utf-8", errors="replace")
            sample = parse_datagram(text)
            if sample is None:
                self._bad_count += 1
                now = time.monotonic()
                if now - self._last_bad_log > 2.0:
                    self._last_bad_log = now
                    self.get_logger().warn(
                        f"无法解析的报文（累计 {self._bad_count} 条）: {text[:80]!r}")
                continue
            self._dispatch(sample)

    def _dispatch(self, sample):
        if sample.kind not in self._seen_kinds:
            self._seen_kinds.add(sample.kind)
            self.get_logger().info(f"首次收到 {sample.kind} 报文")
        handler = getattr(self, f"_on_{sample.kind}", None)
        if handler is not None:
            handler(sample)

    # ------------------------------------------------------------------ 发布
    def _stamp(self, sample):
        if sample.timestamp_ns > 0:
            return rclpy.time.Time(nanoseconds=sample.timestamp_ns).to_msg()
        return self.get_clock().now().to_msg()

    def _pose_msg(self, sample):
        msg = PoseStamped()
        msg.header.stamp = self._stamp(sample)
        msg.header.frame_id = self._frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = sample.position
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = sample.orientation
        return msg

    def _maybe_tf(self, pose_msg, hand):
        if self._tf_broadcaster is None:
            return
        tf = TransformStamped()
        tf.header = pose_msg.header
        tf.child_frame_id = f"pico_{hand}_controller"
        tf.transform.translation.x = pose_msg.pose.position.x
        tf.transform.translation.y = pose_msg.pose.position.y
        tf.transform.translation.z = pose_msg.pose.position.z
        tf.transform.rotation = pose_msg.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def _on_relative_hand(self, sample):
        pubs = self._pico[sample.hand]
        pose_msg = self._pose_msg(sample)
        pubs["pose"].publish(pose_msg)
        pubs["trigger"].publish(Float32(data=float(sample.trigger)))
        pubs["grip"].publish(Float32(data=float(sample.grip)))
        for hand in HANDS:  # 档位是全局量，双手话题同步刷新
            self._pico[hand]["rate"].publish(Float32(data=float(sample.rate)))
        if sample.hand == "right":
            pubs["button_a"].publish(Bool(data=sample.button_a))
            pubs["button_b"].publish(Bool(data=sample.button_b))
        else:
            pubs["button_x"].publish(Bool(data=sample.button_x))
            pubs["button_y"].publish(Bool(data=sample.button_y))
        self._maybe_tf(pose_msg, sample.hand)

    def _on_absolute_hand(self, sample):
        self._vr[f"{sample.hand}/pose_absolute"].publish(self._pose_msg(sample))
        self._vr[f"{sample.hand}/trigger"].publish(Float32(data=float(sample.trigger)))
        self._vr[f"{sample.hand}/grip"].publish(Float32(data=float(sample.grip)))
        self._vr["rate"].publish(Float32(data=float(sample.rate)))
        if sample.hand == "right":
            self._vr["right/button_a"].publish(Bool(data=sample.button_a))
            self._vr["right/button_b"].publish(Bool(data=sample.button_b))
        else:
            self._vr["left/button_x"].publish(Bool(data=sample.button_x))
            self._vr["left/button_y"].publish(Bool(data=sample.button_y))

    def _on_head(self, sample):
        self._vr["head/pose"].publish(self._pose_msg(sample))

    def _on_joystick(self, sample):
        msg = Float32MultiArray(data=[float(sample.joystick_x), float(sample.joystick_y)])
        self._pico[sample.hand]["joystick"].publish(msg)

    def _on_button(self, sample):
        if sample.button_name.upper() != "JOYSTICK":
            return
        self._pico[sample.hand]["button_joystick"].publish(Bool(data=sample.button_pressed))

    def _on_mode(self, sample):
        self._vr["control_mode"].publish(String(data=sample.mode_text))

    def _on_calibrate_done(self, sample):
        self._vr["calibrate_done"].publish(Bool(data=True))

    def _on_pose_source_config(self, sample):
        self._vr["controller_pose_mode"].publish(String(data=sample.mode_text))

    def destroy_node(self):
        self._sock.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = PicoUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
