#!/usr/bin/env python3.12
"""真机遥操作前预检（主机端运行，连的是真机上的 MC）。

逐项检查并给出 [OK]/[FAIL]/[WARN]，全部 OK 才建议启动 mc_arm 模式遥操作：
  1. MC 的 ROS 2 状态话题 /motion/control/arm_joint_state 有数据（14 关节）
  2. MC HTTP 服务可达（127.0.0.1:56322 GetState），并打印当前状态供人工确认
  3. 关节名与官方接口一致

注意（人工确认项，脚本无法替你完成）：
  - MC 处于 MOTION 状态（官方接口仅限 MOTION 状态下调用）
  - motion_player 模块已关闭（官方要求下发命令前关闭）
用法：
  source setup_teleop_env.bash
  /usr/bin/python3.12 scripts/preflight_mc.py [--mc-host 10.42.10.12]
退出码 0 = 关键项全 OK。
"""
import argparse
import json
import sys
import urllib.request

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

MC_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

checks = []


def report(name, ok, detail=""):
    checks.append((name, ok))
    tag = "OK  " if ok else "FAIL"
    print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mc-host", default="127.0.0.1",
                        help="运行 MC 的主机（本机调试留默认；真机填机器人 IP）")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("preflight_mc")
    got = {}

    def on_js(msg):
        got["msg"] = msg

    node.create_subscription(JointState, "/motion/control/arm_joint_state", on_js, 10)
    print("等待 /motion/control/arm_joint_state ...")
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=args.timeout)
    while node.get_clock().now() < deadline and "msg" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)

    if "msg" in got:
        msg = got["msg"]
        report("关节状态话题有数据", True, f"{len(msg.name)} 个关节")
        report("关节名与官方接口一致",
               sorted(msg.name) == sorted(MC_JOINTS),
               f"实际 {sorted(msg.name)[:3]}..." if sorted(msg.name) != sorted(MC_JOINTS)
               else "14/14 匹配")
        print(f"       当前角度(deg): {[round(p * 57.29578, 1) for p in msg.position]}")
    else:
        report("关节状态话题有数据", False, f"{args.timeout}s 内无消息（ROS_DOMAIN_ID/网络/MC 是否运行？）")

    url = f"http://{args.mc_host}:56322/rpc/aimdk.protocol.McBaseService/GetState"
    try:
        req = urllib.request.Request(
            url, data=json.dumps({"header": {}}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            data = json.loads(resp.read().decode())
        report("MC HTTP GetState 可达", True, url)
        text = json.dumps(data, ensure_ascii=False)
        print(f"       当前状态: {text[:300]}")
        print("       >>> 请人工确认处于 MOTION 状态、且 motion_player 已关闭 <<<")
    except Exception as exc:  # noqa: BLE001
        report("MC HTTP GetState 可达", False, f"{exc}")

    node.destroy_node()
    rclpy.shutdown()
    failed = [n for n, ok in checks if not ok]
    print(f"\n关键项 {'全部通过' if not failed else '存在失败：' + ', '.join(failed)}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
