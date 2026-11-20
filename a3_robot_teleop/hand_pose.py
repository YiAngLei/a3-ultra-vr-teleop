#!/usr/bin/env python3
"""手指姿势控制：发送一次目标手势并保持（默认：左手食指伸直、其余四指握拳）。

- 先读 /motion/control/hand_joint_state 当前值，右手原样回发（不动右手）
- 左手 10 关节替换为目标值（默认官方握拳姿势仅食指弯曲置 0 = 伸直）
- /motion/control/hand_joint_command 以 --rate 频率持续发 --hold-sec 秒
- 仅 MOTION 状态下 MC 才会执行，其他状态拒收（失效安全）

用法（机器人上，域 232）：
  python3 hand_pose.py                # 食指伸直手势，2Hz 保持 8 秒
  python3 hand_pose.py --dry-run      # 只打印将发送的数值，不发
  python3 hand_pose.py --left "v,v,..." --rate 2.0 --hold-sec 8
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState

# 官方 hand.py 同款关节顺序（左 10 + 右 10）
JOINT_NAMES = [
    "left_thumb_roration_0", "left_thumb_wiggles_1", "left_thumb_bent_2",
    "left_index_wiggles_0", "left_index_bent_1",
    "left_middle_bent",
    "left_ring_wiggles_0", "left_ring_bent_1",
    "left_pinky_wiggles_0", "left_pinky_bent_1",
    "right_thumb_rotation_0", "right_thumb_wiggles_1", "right_thumb_bent_2",
    "right_index_wiggles_0", "right_index_bent_1",
    "right_middle_bent",
    "right_ring_wiggles_0", "right_ring_bent_1",
    "right_pinky_wiggles_0", "right_pinky_bent_1",
]

# 默认目标：官方握拳姿势，仅 index_bent 置 0（食指伸直）
DEFAULT_LEFT = "4095,0,500,4095,0,3730,4095,3143,0,3227"


class HandPose(Node):
    def __init__(self, left_vals):
        super().__init__("hand_pose")
        self._state = None
        self.create_subscription(
            JointState, "/motion/control/hand_joint_state", self._on_state,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10))
        self._pub = self.create_publisher(
            JointState, "/motion/control/hand_joint_command",
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        self._left = left_vals

    def _on_state(self, msg: JointState):
        if self._state is None:
            self._state = (list(msg.name), list(msg.position))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left", default=DEFAULT_LEFT,
                    help="左手 10 关节值（逗号分隔，默认：食指伸直其余握拳）")
    ap.add_argument("--rate", type=float, default=2.0, help="发布频率 Hz")
    ap.add_argument("--hold-sec", type=float, default=8.0, help="持续发送秒数")
    ap.add_argument("--dry-run", action="store_true", help="只打印不发")
    args = ap.parse_args()

    left = [float(v) for v in args.left.split(",")]
    if len(left) != 10:
        raise SystemExit(f"--left 需要 10 个值，收到 {len(left)} 个")

    rclpy.init()
    node = HandPose(left)
    print("读取当前手指状态 ...")
    t0 = time.time()
    while node._state is None and time.time() - t0 < 3.0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if node._state is None:
        raise SystemExit("3 秒内未读到 hand_joint_state，中止（不盲发）")

    names, pos = node._state
    if len(pos) < 20:
        raise SystemExit(f"hand_joint_state 只有 {len(pos)} 个关节，预期 20，中止")
    right = pos[10:20]
    cmd = left + right

    print("将发送的 20 关节指令（左手=目标，右手=当前值回发）：")
    for i, nm in enumerate(JOINT_NAMES):
        tag = "左手" if i < 10 else "右手"
        print(f"  [{tag}] {nm}: {cmd[i]:.0f}")
    if args.dry_run:
        print("--dry-run：未发送")
        return

    msg = JointState()
    msg.header.frame_id = "O10Hand"
    msg.name = JOINT_NAMES
    msg.position = cmd
    n = 0
    end = time.time() + args.hold_sec
    print(f"\n开始发送 {args.hold_sec}s @ {args.rate}Hz ...")
    while time.time() < end and rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        node._pub.publish(msg)
        n += 1
        rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
    print(f"已发送 {n} 帧，停止发布（MC 保持最后姿势）")

    # 回读对照
    node._state = None
    t0 = time.time()
    while node._state is None and time.time() - t0 < 3.0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if node._state is not None:
        _, after = node._state
        print("\n发送后左手实际状态（对比：食指 bent 应≈0，其余三指 bent 应变大）：")
        for i, nm in enumerate(JOINT_NAMES[:10]):
            print(f"  {nm}: {after[i]:.0f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
