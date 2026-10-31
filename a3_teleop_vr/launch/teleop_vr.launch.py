#!/usr/bin/env python3
"""A3 流式 VR 遥操作 launch。

复用 a3_moveit_demo 的整套仿真栈（demo_plan/teleop 关闭，见该 launch 的新参数），
再拉起 a3_streaming_teleop.py 流式遥操作节点；可选同时启动本工程自带的
a3_teleop_bridge（UDP 接收 Pico 报文）。

用法（本包未做 colcon 打包，按绝对路径启动）：
  cd a3_teleop_vr && source setup_teleop_env.bash
  ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py            # 带 RViz
  ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py rviz:=false

启动参数：
  rviz    true/false（默认 true；real:=true 时忽略，不启动 RViz）
  bridge  true/false（默认 false）：true 时启动本工程的 a3_teleop_bridge。
          若你已在运行 OpenArmX 的 bridge（或用它自测的 fake 发布器），保持 false，
          两者会争抢 5100 端口。
  real    true/false（默认 false）：真机模式。不拉起仿真栈（move_group/ros2_control/
          RViz 都不需要），teleop 节点以 output_backend:=mc_arm + 100Hz 启动，
          直接对机器人 MC 的 /motion/control/arm_joint_command 发指令，
          状态从 /motion/control/arm_joint_state 读。前提：机器人 MC 已运行且处于
          MOTION 状态、motion_player 已关闭（见 a3_robot_teleop/README.md）。

a3_moveit_demo 的定位：默认取本仓库（a3_teleop_vr 的上级目录）下的 a3_moveit_demo，
也可用环境变量 A3_MOVEIT_DEMO_DIR 指向其他位置。real:=true 时仅使用其参数文件
（teleop 节点内嵌 MoveItPy 做 IK），其余皆不启动。
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[2]  # a3_teleop_vr 与 a3_moveit_demo 的共同上级
DEMO_DIR = Path(os.environ.get("A3_MOVEIT_DEMO_DIR", REPO_ROOT / "a3_moveit_demo"))
A3_DEMO_LAUNCH = DEMO_DIR / "src/a3_moveit_config/launch/demo.launch.py"
PARAMS = DEMO_DIR / "src/a3_moveit_config/config/a3_plan_demo_params.yaml"


def generate_launch_description():
    if not PARAMS.exists():
        raise RuntimeError(
            f"找不到 MoveIt 参数文件：{PARAMS}\n"
            "请把 a3_teleop_vr 与 a3_moveit_demo 放在同一目录下，"
            "或设置环境变量 A3_MOVEIT_DEMO_DIR 指向 a3_moveit_demo 所在位置。")
    rviz = LaunchConfiguration("rviz")
    bridge = LaunchConfiguration("bridge")
    real = LaunchConfiguration("real")
    real_cond = IfCondition(real)
    not_real = UnlessCondition(real)
    teleop_args = [
        str(PKG / "scripts" / "a3_streaming_teleop.py"),
        "--ros-args",
        "--params-file",
        str(PARAMS),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("bridge", default_value="false"),
            DeclareLaunchArgument("real", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(A3_DEMO_LAUNCH)),
                launch_arguments={
                    "rviz": rviz,
                    "demo_plan": "false",
                    "teleop": "none",
                }.items(),
                condition=not_real,
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=[
                    str(PKG / "a3_teleop_bridge" / "a3_teleop_bridge_node.py"),
                ],
                condition=IfCondition(bridge),
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=teleop_args + [
                    "-p", "output_backend:=mc_arm",
                    "-p", "control_rate:=100.0",
                ],
                condition=real_cond,
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=teleop_args,
                condition=not_real,
                output="screen",
            ),
        ]
    )
