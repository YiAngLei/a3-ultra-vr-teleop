#!/usr/bin/env python3.12
"""映射标定的"阈值报数"监视器：只在偏差角超阈值时打印一行。

解决 ros2 topic echo 50Hz 刷屏、无法把读数和手上动作对应起来的问题。
平时无输出 = 一切正常；出现打印 = 刚做的那个动作方向/转轴有问题。

用法（用 /usr/bin/python3.12，不要用 conda 的 python；与遥操作同一 ROS 域）：
  /usr/bin/python3.12 scripts/debug_mapping_monitor.py                # 双臂，阈值 10°
  /usr/bin/python3.12 scripts/debug_mapping_monitor.py --side right --threshold 5

判读（对应 README「映射矩阵观测与标定」）：
  - 平移某方向时出现 ≈180°：axis_matrix 该轴符号反；
  - 持续同向拧手柄，旋转偏差跟着涨到 ≈2 倍（拧 45° 报 ~90°、拧 88° 报 ~176°）：
    orientation_matrix 该轴旋转方向全反（偏差角=两倍实际转角，不是 bug）；
  - 旋转偏差稳定 ≈90°/270° 或乱跳：轴向对错（矩阵应为带符号的轴置换阵）；
  - 手柄不动时的底噪只有几度，不报数属正常。
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Monitor(Node):
    def __init__(self, threshold, sides):
        super().__init__("debug_mapping_monitor")
        self.threshold = threshold
        self._last_t = {}
        for side in sides:
            for kind, topic in (("平移", "pos_align_deg"), ("旋转", "rot_align_deg")):
                self.create_subscription(
                    Float32, f"/teleop_debug/{side}/{topic}",
                    lambda m, s=side, k=kind: self.on_val(s, k, m), 10)
        print(f"监视器就绪（{ '/'.join(sides) } 臂，阈值 {threshold}°）："
              "无输出=映射正常，出现打印=刚做的动作有问题", flush=True)

    def on_val(self, side, kind, msg):
        v = msg.data
        if abs(v) < self.threshold:
            return
        now = time.monotonic()
        key = (side, kind)
        if now - self._last_t.get(key, 0.0) < 0.4:  # 节流到 2.5Hz，同向连报没意义
            return
        self._last_t[key] = now
        tag = ""
        if v > 150.0:
            tag = "  <-- 方向反"
        elif kind == "旋转" and 60.0 < v < 120.0:
            tag = "  <-- 轴向可能错(≈90°)"
        print(f"[{side}] {kind}偏差 {v:6.1f}°{tag}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--threshold", type=float, default=10.0, help="报数阈值（度）")
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    args = ap.parse_args()
    sides = ["left", "right"] if args.side == "both" else [args.side]
    rclpy.init()
    node = Monitor(args.threshold, sides)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # SIGINT/SIGTERM 关停时 rclpy 抛 ExternalShutdownException/RCLError
    finally:
        node.destroy_node()
        if rclpy.ok():  # 同上，信号路径下 rclpy 已自行 shutdown
            rclpy.shutdown()


if __name__ == "__main__":
    main()
