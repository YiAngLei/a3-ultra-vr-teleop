#!/usr/bin/env python3.12
"""a3_teleop_bridge 节点自测：本机 UDP 环回，无需头显。

在进程内启动 PicoUdpBridge（监听 127.0.0.1:55100），订阅受测话题，
按报文协议发送各类 UDP 报文，逐条断言发布结果；含垃圾报文容错检查。

用法（先 source ROS 2 Jazzy 环境，如 a3_teleop_vr 的 setup_teleop_env.bash）：
  /usr/bin/python3.12 test_bridge_udp.py
退出码 0 = 全部通过。
"""
import socket
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a3_teleop_bridge_node import PicoUdpBridge  # noqa: E402

PORT = 55100


class Collector(Node):
    """订阅全部受测话题，保留每条话题的最后一条消息。"""

    def __init__(self):
        super().__init__("bridge_selftest_collector")
        self.latest = {}

        def keep(name):
            def cb(msg):
                self.latest[name] = msg
            return cb

        for side in ("left", "right"):
            self.create_subscription(
                PoseStamped, f"/pico_{side}_controller/pose", keep(f"{side}.pose"), 10)
            for suffix, msg_type in (
                ("trigger", Float32), ("grip", Float32), ("rate", Float32),
                ("joystick", Float32MultiArray), ("button_joystick", Bool),
            ):
                self.create_subscription(
                    msg_type, f"/pico_{side}_controller/{suffix}", keep(f"{side}.{suffix}"), 10)
        self.create_subscription(Bool, "/pico_right_controller/button_a", keep("right.button_a"), 10)
        self.create_subscription(Bool, "/pico_right_controller/button_b", keep("right.button_b"), 10)
        self.create_subscription(Bool, "/pico_left_controller/button_x", keep("left.button_x"), 10)
        self.create_subscription(Bool, "/pico_left_controller/button_y", keep("left.button_y"), 10)
        self.create_subscription(PoseStamped, "/vr/left/pose_absolute", keep("vr.left.pose"), 10)
        self.create_subscription(PoseStamped, "/vr/head/pose", keep("vr.head.pose"), 10)
        self.create_subscription(Float32, "/vr/rate", keep("vr.rate"), 10)
        self.create_subscription(String, "/vr/control_mode", keep("vr.mode"), 10)
        self.create_subscription(String, "/vr/controller_pose_mode", keep("vr.pose_mode"), 10)
        self.create_subscription(Bool, "/vr/calibrate_done", keep("vr.calibrate_done"), 10)


def send(*datagrams):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for text in datagrams:
        sock.sendto(text.encode(), ("127.0.0.1", PORT))
    sock.close()


def main():
    rclpy.init()
    bridge = PicoUdpBridge(parameter_overrides=[
        Parameter("listen_address", Parameter.Type.STRING, "127.0.0.1"),
        Parameter("listen_port", Parameter.Type.INTEGER, PORT),
    ])
    collector = Collector()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(collector)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    checks = []

    def check(name, cond):
        checks.append((name, bool(cond)))

    def close(msg, expected):
        return msg is not None and abs(msg.data - expected) < 1e-6

    # 1. 相对左手位姿：位置/扳机/握把/档位/X 键 + rate 双手同步刷新
    send("LEFT 0.1 0.2 0.3 0 0 0 1 0.5 0.7 0 0 1 0 0.1 123456789")
    spin_for(0.4)
    pose = collector.latest.get("left.pose")
    check("left.pose 位置解析", pose and abs(pose.pose.position.x - 0.1) < 1e-6
          and abs(pose.pose.position.y - 0.2) < 1e-6 and abs(pose.pose.position.z - 0.3) < 1e-6)
    check("left.pose frame_id=pico_hmd", pose and pose.header.frame_id == "pico_hmd")
    check("left.pose 采用报文时间戳", pose and pose.header.stamp.nanosec == 123456789)
    check("left.trigger=0.5", close(collector.latest.get("left.trigger"), 0.5))
    check("left.grip=0.7", close(collector.latest.get("left.grip"), 0.7))
    check("left.rate=0.1", close(collector.latest.get("left.rate"), 0.1))
    check("right.rate 被同报文刷新为 0.1", close(collector.latest.get("right.rate"), 0.1))
    check("left.button_x=True（X 随左手报文）",
          collector.latest.get("left.button_x") is not None
          and collector.latest["left.button_x"].data is True)
    check("left.button_y=False",
          collector.latest.get("left.button_y") is not None
          and collector.latest["left.button_y"].data is False)

    # 2. 右手：B 键 + 1.0 档
    send("RIGHT 1 2 3 0 0 0 1 0 0 0 1 0 0 1.0 123")
    spin_for(0.4)
    check("right.button_b=True（B 随右手报文）",
          collector.latest.get("right.button_b") is not None
          and collector.latest["right.button_b"].data is True)
    check("right.button_a=False",
          collector.latest.get("right.button_a") is not None
          and collector.latest["right.button_a"].data is False)
    check("双手 rate 切到 1.0",
          close(collector.latest.get("left.rate"), 1.0)
          and close(collector.latest.get("right.rate"), 1.0))

    # 3. 绝对位姿 / 头显 / 模式 / 标定完成
    send("LEFT_ABS 4 5 6 0 0 0 1", "HEAD 7 8 9 0 0 0 1",
         "MODE follow 99", "CFG POSE_MODE 1 99", "CALIBRATE_DONE 99")
    spin_for(0.4)
    abs_pose = collector.latest.get("vr.left.pose")
    check("vr.left.pose_absolute 位置", abs_pose and abs(abs_pose.pose.position.x - 4) < 1e-6
          and abs(abs_pose.pose.position.z - 6) < 1e-6)
    head_pose = collector.latest.get("vr.head.pose")
    check("vr.head.pose 位置", head_pose and abs(head_pose.pose.position.x - 7) < 1e-6)
    check("vr.control_mode='follow'",
          collector.latest.get("vr.mode") is not None
          and collector.latest["vr.mode"].data == "follow")
    check("vr.controller_pose_mode='hand'（CFG POSE_MODE 1）",
          collector.latest.get("vr.pose_mode") is not None
          and collector.latest["vr.pose_mode"].data == "hand")
    check("vr.calibrate_done=True",
          collector.latest.get("vr.calibrate_done") is not None
          and collector.latest["vr.calibrate_done"].data is True)

    # 4. 摇杆与摇杆按键
    send("JOY LEFT 0.25 -0.5 1", "BTN RIGHT JOYSTICK 1 2")
    spin_for(0.4)
    joy = collector.latest.get("left.joystick")
    check("left.joystick=[0.25, -0.5]", joy and len(joy.data) == 2
          and abs(joy.data[0] - 0.25) < 1e-6 and abs(joy.data[1] + 0.5) < 1e-6)
    check("right.button_joystick=True",
          collector.latest.get("right.button_joystick") is not None
          and collector.latest["right.button_joystick"].data is True)

    # 5. 垃圾报文容错（节点存活且计数正确）
    send("GARBAGE", "LEFT 1 2", "")
    spin_for(0.4)
    check("垃圾/残缺报文被计数且节点存活", bridge._bad_count >= 3)

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    print(f"共 {len(checks)} 项，失败 {len(failed)} 项")
    sys.stdout.flush()

    bridge.destroy_node()
    collector.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
