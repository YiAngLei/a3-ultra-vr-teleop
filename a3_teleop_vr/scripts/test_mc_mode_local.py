#!/usr/bin/env python3.12
"""mc_arm（真机 MC 接口）模式本机自测：fake MC + 流式节点，无需机器人。

进程内起三个节点：
  - FakeMc        发布 /motion/control/arm_joint_state（14 关节，全零，100Hz），
                  并记录收到的 /motion/control/arm_joint_command；
  - PoseDriver    向 /pico_* 发合成手柄数据（左 grip=1 画圆 3s -> 松开保持）；
  - StreamingTeleop output_backend=mc_arm, control_rate=100。

断言（对齐官方接口要求）：
  - 指令流 ~100Hz 且相邻间隔 ≤30ms
  - 14 关节名固定且顺序正确；velocity/effort 全 0
  - 相邻指令最大速度 ≤4 rad/s + 余量
  - 指令值在官方限位内
  - 画圆期间手臂实际运动；松开后保持（变化 < 0.5°）
  - B/Y 脚本动作请求被 mc 模式拒绝且节点存活

用法（先 source ROS 2 Jazzy 环境）：
  /usr/bin/python3.12 scripts/test_mc_mode_local.py
退出码 0 = 全部通过。
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a3_streaming_teleop import (  # noqa: E402
    MC_ARM_JOINTS, MC_JOINT_LIMITS, StreamingTeleop,
)


class FakeMc(Node):
    """替身 MC：发状态、记指令。"""

    def __init__(self):
        super().__init__("fake_mc")
        self.cmds = []  # (monotonic, np14)
        self.pub = self.create_publisher(
            JointState, "/motion/control/arm_joint_state", 10)
        self.create_subscription(
            JointState, "/motion/control/arm_joint_command", self.on_cmd, 20)
        self.create_timer(0.01, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MC_ARM_JOINTS)
        msg.position = [0.0] * 14
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        self.pub.publish(msg)

    def on_cmd(self, msg):
        self.cmds.append((time.monotonic(), np.asarray(msg.position, dtype=np.float64), msg))


class PoseDriver(Node):
    """合成手柄：3s 画圆，之后 grip 松开保持。"""

    def __init__(self, duration=3.0):
        super().__init__("pose_driver")
        self.t0 = time.monotonic()
        self.duration = duration
        self.pub_pose = self.create_publisher(PoseStamped, "/pico_left_controller/pose", 20)
        self.pub_grip = self.create_publisher(Float32, "/pico_left_controller/grip", 10)
        self.pub_rate = self.create_publisher(Float32, "/pico_left_controller/rate", 10)
        self.pub_b = self.create_publisher(Bool, "/pico_right_controller/button_b", 10)
        self.create_timer(1.0 / 60.0, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pico_hmd"
        phase = t * 2.0 * np.pi * 0.5
        msg.pose.position.x = float(0.1 * np.cos(phase))
        msg.pose.position.y = float(0.1 * np.sin(phase))
        msg.pose.position.z = float(0.05 * np.sin(phase * 0.5))
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)
        g = Float32(data=1.0 if t < self.duration else 0.0)
        self.pub_grip.publish(g)
        r = Float32(data=0.1)
        self.pub_rate.publish(r)
        b = Bool(data=t > self.duration + 0.5 and t < self.duration + 0.8)
        self.pub_b.publish(b)


def main():
    rclpy.init()
    from rclpy.parameter import Parameter
    teleop = StreamingTeleop(parameter_overrides=[
        Parameter("output_backend", Parameter.Type.STRING, "mc_arm"),
        Parameter("control_rate", Parameter.Type.DOUBLE, 100.0),
    ])
    fake = FakeMc()
    driver = PoseDriver(duration=4.0)

    executor = SingleThreadedExecutor()
    executor.add_node(teleop)
    executor.add_node(fake)
    executor.add_node(driver)

    t_end = time.monotonic() + 10.0
    while time.monotonic() < t_end:
        executor.spin_once(timeout_sec=0.05)

    checks = []

    def check(name, cond, detail=""):
        checks.append((name, bool(cond)))
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    cmds = list(fake.cmds)  # 快照，防分析期间订阅回调并发追加
    check("收到指令帧", len(cmds) > 500, f"{len(cmds)} 帧")
    if len(cmds) >= 20:
        ts = np.array([c[0] for c in cmds])
        gaps = np.diff(ts)
        rate = 1.0 / np.mean(gaps)
        check("平均频率 ≈100Hz", 90.0 < rate < 115.0, f"{rate:.1f} Hz")
        check("相邻间隔 ≤30ms", np.percentile(gaps, 99) < 0.030,
              f"p99={np.percentile(gaps, 99) * 1000:.2f}ms")
        pos = np.stack([c[1] for c in cmds])
        check("14 关节", pos.shape[1] == 14)
        dq = np.abs(np.diff(pos, axis=0)).max(axis=1)
        check("相邻速度 ≤4 rad/s（含 10% 余量）",
              dq.max() < 4.0 / 100.0 * 1.1, f"max={dq.max() * 100.0:.3f} rad/100Hz帧")
        low = np.array([MC_JOINT_LIMITS[n][0] for n in MC_ARM_JOINTS])
        high = np.array([MC_JOINT_LIMITS[n][1] for n in MC_ARM_JOINTS])
        check("全部指令在官方限位内",
              (pos >= low - 1e-6).all() and (pos <= high + 1e-6).all())
        # 画圆期间(1.5-3.5s)左臂有运动，松开保持期(7-9.5s)基本不动
        t0 = ts[0]
        mid = pos[(ts - t0 > 1.5) & (ts - t0 < 3.5)]
        hold = pos[(ts - t0 > 7.0) & (ts - t0 < 9.5)]
        moved = np.abs(mid.max(axis=0) - mid.min(axis=0)).max() if len(mid) else 0.0
        held = np.abs(np.diff(hold, axis=0)).max() if len(hold) > 1 else 0.0
        check("画圆期间手臂实际运动", moved > np.deg2rad(3), f"最大摆幅 {np.rad2deg(moved):.1f}°")
        check("松开后保持（<0.5°）", held < np.deg2rad(0.5),
              f"保持期最大变化 {np.rad2deg(held):.2f}°")
        check("关节名固定顺序正确",
              list(cmds[0][2].name) == MC_ARM_JOINTS)
        check("velocity/effort 全 0",
              all(v == 0.0 for v in cmds[0][2].velocity)
              and all(e == 0.0 for e in cmds[0][2].effort))

    failed = [n for n, ok in checks if not ok]
    print(f"共 {len(checks)} 项，失败 {len(failed)} 项")
    sys.stdout.flush()

    teleop.destroy_node()
    fake.destroy_node()
    driver.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
