#!/usr/bin/env python3.12
"""智元 MC 手臂状态回显：/motion/control/arm_joint_state -> /joint_states。

MC 只反馈 14 个手臂关节，而 robot_state_publisher / RViz / move_group 的
PlanningScene 需要全身 31 个关节的 /joint_states。本节点把 MC 的 14 关节
按名字合入全身关节表（来源：a3_moveit_demo 的 joint_limits.yaml，共 31 个），
其余 17 个（腿 12 + 腰 3 + 头 2）补 0.0——真机上这些部位由 MC 自身的
站立/行走栈控制，本工程不动它们，显示为默认站姿即可。

MC 状态到达即转发（MC 100Hz），另加 10Hz 看门狗重发最后一帧，防止 RViz 过期。

用法（real_teleop.launch.py 会自动拉起，也可单独跑）：
  /usr/bin/python3.12 scripts/mc_state_to_joint_states.py
"""
import os
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

REPO_ROOT = Path(__file__).resolve().parents[2]  # 与 a3_streaming_teleop.py 同规则
DEFAULT_LIMITS_FILE = (
    REPO_ROOT / "a3_moveit_demo" / "src/a3_moveit_config/config/joint_limits.yaml"
)


def load_all_joint_names(limits_file):
    """joint_limits.yaml: robot_description_planning.joint_limits.<name>。"""
    data = yaml.safe_load(open(limits_file, encoding="utf-8"))
    names = list(data["robot_description_planning"]["joint_limits"].keys())
    if len(names) != 31:
        raise RuntimeError(f"{limits_file} 应有 31 个关节，实际 {len(names)}")
    return names


class McStateEcho(Node):
    def __init__(self):
        super().__init__("mc_state_to_joint_states")
        self.declare_parameter("mc_state_topic", "/motion/control/arm_joint_state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "joint_limits_file",
            os.environ.get("A3_JOINT_LIMITS_FILE", str(DEFAULT_LIMITS_FILE)))
        p = self.get_parameter
        limits_file = str(p("joint_limits_file").value)
        if not Path(limits_file).exists():
            raise RuntimeError(
                f"找不到关节限位文件：{limits_file}\n"
                "请把 a3_teleop_vr 与 a3_moveit_demo 放在同一目录下，或设置 "
                "A3_JOINT_LIMITS_FILE / joint_limits_file 参数。")
        self.all_names = load_all_joint_names(limits_file)
        self.arm_positions = {}   # 只缓存手臂 14 关节
        self.last_publish_time = 0.0

        self.pub = self.create_publisher(
            JointState, str(p("joint_states_topic").value), 10)
        self.create_subscription(
            JointState, str(p("mc_state_topic").value), self.on_mc_state, 50)
        self.create_timer(0.1, self.watchdog)  # 10Hz 看门狗
        self.get_logger().info(
            f"MC 状态回显就绪：{len(self.all_names)} 关节"
            f"（手臂 14 来自 MC，其余补 0）")

    def on_mc_state(self, msg):
        for n, pos in zip(msg.name, msg.position):
            self.arm_positions[n] = float(pos)
        self.publish()

    def watchdog(self):
        # MC 停发时保持最后一帧（不清零——真机关节不会自己归零）
        if self.arm_positions and self.get_clock().now().nanoseconds * 1e-9 - \
                self.last_publish_time > 0.5:
            self.publish()

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.all_names)
        msg.position = [self.arm_positions.get(n, 0.0) for n in self.all_names]
        msg.velocity = [0.0] * len(self.all_names)
        msg.effort = [0.0] * len(self.all_names)
        self.pub.publish(msg)
        self.last_publish_time = self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = McStateEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
