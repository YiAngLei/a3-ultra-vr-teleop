#!/usr/bin/env python3
"""A3 流式 VR 遥操作 launch。

复用 a3_moveit_demo 的整套仿真栈（demo_plan/teleop 关闭，见该 launch 的新参数），
再拉起 a3_streaming_teleop.py 流式遥操作节点；可选同时启动桥接节点。

用法（本包未做 colcon 打包，按绝对路径启动）：
  cd a3_teleop_vr && source setup_teleop_env.bash
  ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py            # 带 RViz
  ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py rviz:=false

启动参数：
  rviz    true/false（默认 true；real:=true 时忽略，不启动 RViz）
  bridge  true/false（默认 false）：true 时按 bridge_impl 启动桥接节点。
  bridge_impl xrt（默认）/openarmx：
          xrt     — a3_xrt_bridge，收 XRoboToolkit PC Service 官方 SDK
                    （房间锚定位姿，与遥操作的世界系增量映射同源配套；
                    需先启动 PC Service：bash /opt/apps/roboticsservice/runService.sh；
                    头显用 XRoboToolkit 官方 App，Head+Controller 勾上、Send ON；
                    默认还发 TF：xrt -> xrt_<side>_controller）。
          openarmx — a3_teleop_bridge，兼容 OpenArmX APK 的 UDP 报文
                    （位姿为 pico_hmd 头显系，与世界系增量映射不配套，仅供对照/
                    回退；两者端口/话题不冲突但不要同时开）。
          rate 调速档：xrt 桥默认用扳机当快进键（trigger>0.5=快档），
                    -p rate_from_trigger:=false 固定慢档。
  real    true/false（默认 false）：真机模式。不拉起仿真栈（move_group/ros2_control/
          RViz 都不需要），teleop 节点以 output_backend:=mc_arm + 100Hz 启动，
          直接对机器人 MC 的 /motion/control/arm_joint_command 发指令，
          状态从 /motion/control/arm_joint_state 读。前提：机器人 MC 已运行且处于
          MOTION 状态、motion_player 已关闭（见 a3_robot_teleop/README.md）。
  映射按 XRoboToolkit 双 UR5 官方案例（世界系增量）：
          - world_rotation：报文系->世界系固定转换（行主序 9 个数，默认即
            官方 R_HEADSET_TO_WORLD），方向不对时按 README 标定流程重测；
          - pose_source：relative（默认，/pico_*_controller/pose）或 absolute
            （/vr/<side>/pose_absolute，仅 openarmx 桥有该数据）。

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
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    bridge_impl = LaunchConfiguration("bridge_impl")
    real = LaunchConfiguration("real")
    real_cond = IfCondition(real)
    not_real = UnlessCondition(real)
    bridge_xrt = IfCondition(PythonExpression(
        ["'", bridge, "' == 'true' and '", bridge_impl, "' == 'xrt'"]))
    bridge_openarmx = IfCondition(PythonExpression(
        ["'", bridge, "' == 'true' and '", bridge_impl, "' == 'openarmx'"]))
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
            DeclareLaunchArgument("bridge_impl", default_value="xrt"),
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
                    str(PKG / "a3_xrt_bridge" / "a3_xrt_bridge_node.py"),
                ],
                condition=bridge_xrt,
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=[
                    str(PKG / "a3_teleop_bridge" / "a3_teleop_bridge_node.py"),
                ],
                condition=bridge_openarmx,
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
