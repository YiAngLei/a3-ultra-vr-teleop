#!/usr/bin/env python3.12
"""Pico 手柄运动查看器：只订阅遥操作话题，实时显示捕获以来的位移/旋转分量。

与 a3_streaming_teleop 完全相同的捕获语义：grip 按下瞬间记参考位姿，
之后每 0.1s 在终端原地刷新一行：

  [L●] 手柄系Δ(m) x +0.051 y -0.104 z +0.021 | 头显系Δ(m) x +0.050 y -0.101 z +0.018 | 绕捕获轴(°) x +3.1 y +12.0 z -45.2

三列分别是：
  手柄系Δ   D 的平移分量（捕获时刻手柄体坐标系；旧版 OpenArm 映射的输入，现仅作
            报文系观测，world_rotation 标定即要测出这个系的轴向）
  头显系Δ   位置差直接取 p(t)-p(0)（pico_hmd 系，用来认头显系哪根轴是"上"）
  绕捕获轴  D 的旋转部分折成"绕捕获系 x/y/z 各转了多少度"（旋转矢量分解）

用途（诊断映射/镜像时的 ground truth，不需要机器人、MoveIt、RViz）：
  - 手握 grip 往你的左动：看"手柄系Δ"哪个分量在变——若变的是 y，说明手柄
    体系 y=左；若 x 变且符号和你预期相反，就是镜像的指纹；
  - 手往上抬：看"头显系Δ"哪个分量变——那就是头显系的竖直轴（Pico/Unity
    通常是 y；若数据里 z 在动，说明 bridge 已做过 y/z 变换）；
  - 绕某轴拧：看"绕捕获轴"哪列变。转 45° 读数就是 ±45°（2 倍才是镜像）。

用法（用 /usr/bin/python3.12；与 bridge 同一 ROS 域）：
  /usr/bin/python3.12 scripts/pico_motion_viewer.py            # 双臂两行
  /usr/bin/python3.12 scripts/pico_motion_viewer.py --side left
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)


def rot_to_rv_deg(R):
    """旋转矩阵 -> 旋转矢量（度）：返回绕各轴分量，直观对应"绕 x/y/z 转多少"。"""
    x, y, z, w = rot_to_quat(R)
    s = np.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(s, w)  # 0~π，轴自动带符号
    axis = np.array([x, y, z]) / s
    return np.degrees(angle * axis)


def fmt_vec(v, unit="", width=7, prec=3):
    return " ".join(f"{a:+{width}.{prec}f}" for a in v) + (f" {unit}" if unit else "")


class HandView:
    """单臂状态：grip 捕获参考，之后每周期算 D 的三个分解。"""

    def __init__(self, node, side):
        self.side = side
        self.grip = 0.0
        self.p = None
        self.R = None
        self.p0 = None
        self.R0 = None
        self.t_latest = 0.0
        node.create_subscription(
            PoseStamped, f"/pico_{side}_controller/pose", self.on_pose, 20)
        node.create_subscription(
            Float32, f"/pico_{side}_controller/grip", self.on_grip, 10)
        self.log = node.get_logger()

    def on_pose(self, msg):
        m = msg.pose
        self.p = np.array([m.position.x, m.position.y, m.position.z])
        self.R = quat_to_rot([m.orientation.x, m.orientation.y,
                              m.orientation.z, m.orientation.w])
        self.t_latest = time.monotonic()

    def on_grip(self, msg):
        self.grip = float(msg.data)

    def line(self, timeout=0.5):
        if self.p is None or (time.monotonic() - self.t_latest) > timeout:
            return f"[{self.side.upper()}?] 无 pose 数据"
        active = self.grip > 0.5
        if not active:
            self.p0 = None  # 松开后清参考，下次 grip 重新捕获
            return f"[{self.side.upper()}○] 松开（握 grip 捕获参考）  pos=({fmt_vec(self.p, prec=2)})"
        if self.p0 is None:
            self.p0, self.R0 = self.p, self.R
            self.log.info(f"[{self.side}] 已捕获参考 pos={self.p.round(3)}")
        Dp_body = self.R0.T @ (self.p - self.p0)          # 手柄系Δ（=遥操作的 D 平移）
        DR = self.R0.T @ self.R                           # D 旋转
        rv = rot_to_rv_deg(DR)                            # 绕捕获轴分解
        dp_hmd = self.p - self.p0                         # 头显系Δ
        return (f"[{self.side.upper()}●] 手柄系Δ(m) {fmt_vec(Dp_body)}"
                f" | 头显系Δ(m) {fmt_vec(dp_hmd)}"
                f" | 绕捕获轴(°) x {rv[0]:+6.1f} y {rv[1]:+6.1f} z {rv[2]:+6.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    args = ap.parse_args()
    sides = ["left", "right"] if args.side == "both" else [args.side]

    rclpy.init()
    node = Node("pico_motion_viewer")
    views = [HandView(node, s) for s in sides]
    print(f"查看器就绪（{'/'.join(sides)}）：握 grip 捕获参考，移动即可看分量；Ctrl+C 退出",
          flush=True)
    n_lines = len(sides)
    first = True
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            lines = [v.line() for v in views]
            if first:
                sys.stdout.write("\n".join(lines) + "\n")
                first = False
            else:
                sys.stdout.write("\x1b[%dA\r" % n_lines + "\n".join(lines) + "\n")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
