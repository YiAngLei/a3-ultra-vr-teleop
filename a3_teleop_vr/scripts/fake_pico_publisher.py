#!/usr/bin/env python3.12
"""无头显端到端测试：合成 /pico_* 话题驱动 a3_streaming_teleop。

场景（自动推进）：
  0-3s    grip=0 空闲，手臂不应动
  3-13s   左 grip=1，手柄在半径 0.1m 画圆 2 圈（右臂不动）
  13-16s  grip=0，手臂应保持
  16s     右手 B 键触发回零脚本（左臂->右臂顺序）
  ~28s    左手 Y 键触发举手脚本
  ~40s    左手 grip=1 按下 -> 取消脚本动作
全程记录左臂 7 关节快照，结束时打印各阶段对比。

用法（栈和 teleop 节点已由 launch 拉起后）：
  /usr/bin/python3.12 scripts/fake_pico_publisher.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

PHASES = [
    (0.0, "idle: grip 全松，手臂应保持不动"),
    (3.0, "stream LEFT: 左 grip=1，手柄画圆（期望左臂连续跟随，右臂不动）"),
    (13.0, "hold: grip 全松（期望两臂保持）"),
    (16.0, "button B: 双臂回零脚本（先左后右）"),
    (30.0, "button Y: 双臂举手脚本"),
    (31.8, "grip 左按下：应取消脚本动作并恢复手动"),
    (40.0, "结束"),
]


class FakePico(Node):
    def __init__(self):
        super().__init__("fake_pico")
        self.pub_pose = {}
        self.pub_grip = {}
        self.pub_rate = {}
        for side in ("left", "right"):
            self.pub_pose[side] = self.create_publisher(PoseStamped, f"/pico_{side}_controller/pose", 20)
            self.pub_grip[side] = self.create_publisher(Float32, f"/pico_{side}_controller/grip", 10)
            self.pub_rate[side] = self.create_publisher(Float32, f"/pico_{side}_controller/rate", 10)
        self.pub_b = self.create_publisher(Bool, "/pico_right_controller/button_b", 10)
        self.pub_y = self.create_publisher(Bool, "/pico_left_controller/button_y", 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.joints = {}
        self.snapshots = []
        self.t0 = time.monotonic()
        self.phase_idx = -1
        self.b_pressed_at = None
        self.y_pressed_at = None
        self.cancel_pressed = False
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def left_arm_snapshot(self):
        names = [f"left_{j}_joint" for j in
                 ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
        return np.array([self.joints.get(n, 0.0) for n in names])

    def publish_hand(self, side, pos, grip):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pico_hmd"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.pose.orientation.w = 1.0
        self.pub_pose[side].publish(msg)
        g = Float32()
        g.data = float(grip)
        self.pub_grip[side].publish(g)
        r = Float32()
        r.data = 0.1
        self.pub_rate[side].publish(r)

    def tick(self):
        t = time.monotonic() - self.t0

        # 阶段推进日志 + 关节快照
        while self.phase_idx + 1 < len(PHASES) and t >= PHASES[self.phase_idx + 1][0]:
            self.phase_idx += 1
            stamp, desc = PHASES[self.phase_idx]
            snap = self.left_arm_snapshot()
            self.snapshots.append((stamp, snap))
            self.get_logger().info(f"[t={stamp:5.1f}s] {desc} | 左臂关节快照(deg)={np.round(np.rad2deg(snap), 1)}")

        # 手柄数据
        if t < 3.0:
            self.publish_hand("left", (0.0, 0.0, 0.0), 0.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)
        elif t < 13.0:
            phase = (t - 3.0) / 10.0 * 4.0 * math.pi  # 2 圈
            pos = (0.1 * math.cos(phase), 0.1 * math.sin(phase), 0.05 * math.sin(phase * 0.5))
            self.publish_hand("left", pos, 1.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)
        elif t < 30.0:
            self.publish_hand("left", (0.0, 0.0, 0.0), 0.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)
        elif t < 31.8:
            self.publish_hand("left", (0.0, 0.0, 0.0), 0.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)
        elif t < 33.5:
            # 脚本执行期间按下 grip：期望取消脚本、恢复手动跟随
            self.publish_hand("left", (0.05, 0.0, 0.0), 1.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)
        else:
            self.publish_hand("left", (0.05, 0.0, 0.0), 0.0)
            self.publish_hand("right", (0.0, 0.0, 0.0), 0.0)

        # 按键脉冲（0.3s）
        b = Bool()
        b.data = 16.0 <= t <= 16.3
        self.pub_b.publish(b)
        y = Bool()
        y.data = 30.0 <= t <= 30.3
        self.pub_y.publish(y)

        if t > 41.0:
            self.finish()

    def finish(self):
        self.get_logger().info("=== 关节快照对比 ===")
        for stamp, snap in self.snapshots:
            self.get_logger().info(f"t={stamp:5.1f}s  {np.round(np.rad2deg(snap), 1)}")
        # 基本断言
        snaps = {s: v for s, v in self.snapshots}
        checks = [
            ("空闲->画圆期间左臂有运动", np.max(np.abs(snaps.get(13.0, snaps[list(snaps)[-1]]) - snaps.get(3.0, snaps[list(snaps)[0]]))) > np.deg2rad(5)),
            ("画圆->保持期间左臂基本不动", np.max(np.abs(snaps.get(16.0, 0) - snaps.get(13.0, 1))) < np.deg2rad(2)),
        ]
        for name, ok in checks:
            self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {name}")
        rclpy.shutdown()
        sys.exit(0)


def main():
    rclpy.init()
    rclpy.spin(FakePico())


if __name__ == "__main__":
    main()
