#!/usr/bin/env python3
"""真机联动 launch：RViz 规划执行 + 可选流式遥操作，共用一套安全链驱动真机双臂。

默认拉起（无 ros2_control / 无仿真栈）：
  robot_state_publisher（发布 /robot_description 与 /tf，关节值来自回显的 /joint_states）
  static_transform_publisher（world->pelvis_link，RViz Fixed Frame）
  mc_state_to_joint_states（/motion/control/arm_joint_state 14 关节 -> 全身 31 关节 /joint_states）
  a3_mc_controller（双臂 FollowJointTrajectory action server + joint_trajectory 话题
                    双入口 -> /motion/control/arm_joint_command 100Hz，内置仲裁：
                    流式话题到达即接管该侧并抢占正在执行的 action 目标，
                    话题静默 stream_timeout（默认 0.3s）后控制权归还 action）
  move_group（MoveIt 规划主节点，控制器白名单收敛到双臂，见 config/real_move_group_override.yaml）
  rviz2（可选，MotionPlanning 显示；拖小球 Plan&Execute = 真机手臂）

联动（teleop:=true，一条命令同起规划栈 + 流式遥操作）：Pico 经 a3_teleop_bridge
收手柄报文，a3_streaming_teleop 用默认 backend 发 /<side>_arm_controller/
joint_trajectory 话题，进 a3_mc_controller 的流式入口，与 RViz/motion_action
的 action goal 走完全相同的限位/限速/低通安全链。手柄流式控制期间 RViz 正在
执行的轨迹会被抢占（显示失败属预期，机器人保持被接管时刻的安全位姿）；
手柄停 stream_timeout 后 RViz 规划可继续用。

注意：联动模式下遥操作节点用默认 backend（发话题给 a3_mc_controller）。
要单独跑"纯流式直发 MC"（不经 move_group、不带仲裁，如调试单臂平滑性），
用 teleop_vr.launch.py real:=true，且不要同时开本 launch——两种模式都会
直接写 /motion/control/arm_joint_command，同时运行会抢指令流。状态回显
（/joint_states）对各模式都安全，可同时开。

前提：机器人 MC 已运行且处于 MOTION 状态、motion_player 已关闭
（检查方法见 a3_robot_teleop/README.md）。

用法：
  cd a3_teleop_vr && source setup_teleop_env.bash
  ros2 launch a3_teleop_vr/launch/real_teleop.launch.py                     # 仅 RViz 规划
  ros2 launch a3_teleop_vr/launch/real_teleop.launch.py teleop:=true bridge:=true   # 联动
启动参数：
  rviz    true/false（默认 true）
  teleop  true/false（默认 false）：true 时启动 a3_streaming_teleop 流式遥操作节点
          （默认 backend，话题进 a3_mc_controller；B/Y 键在 move_group 在场时也可用）
  bridge  true/false（默认 false）：true 时启动本工程的 a3_teleop_bridge
          （UDP 接收 Pico 报文）。若已在运行 OpenArmX 的 bridge，保持 false，
          两者会争抢 5100 端口。
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[2]  # a3_teleop_vr 与 a3_moveit_demo 的共同上级
DEMO_DIR = Path(os.environ.get("A3_MOVEIT_DEMO_DIR", REPO_ROOT / "a3_moveit_demo"))
ROS2CONTROL_FILE = DEMO_DIR / "src/a3_moveit_config/urdf/a3_ultra_t2d0_ros2control.urdf"
PARAMS = DEMO_DIR / "src/a3_moveit_config/config/a3_plan_demo_params.yaml"
RVIZ_FILE = DEMO_DIR / "src/a3_moveit_config/rviz/moveit.rviz"
OVERRIDE = PKG / "config" / "real_move_group_override.yaml"


def generate_launch_description():
    for f in (ROS2CONTROL_FILE, PARAMS, RVIZ_FILE, OVERRIDE):
        if not f.exists():
            raise RuntimeError(
                f"找不到文件：{f}\n请把 a3_teleop_vr 与 a3_moveit_demo 放在同一目录下，"
                "或设置环境变量 A3_MOVEIT_DEMO_DIR 指向 a3_moveit_demo 所在位置。")
    rviz = LaunchConfiguration("rviz")
    teleop = LaunchConfiguration("teleop")
    bridge = LaunchConfiguration("bridge")
    ros2control_text = ROS2CONTROL_FILE.read_text(encoding="utf-8")
    teleop_args = [
        str(PKG / "scripts" / "a3_streaming_teleop.py"),
        "--ros-args",
        "--params-file",
        str(PARAMS),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("teleop", default_value="false"),
            DeclareLaunchArgument("bridge", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": ros2control_text}],
                output="screen",
            ),
            # URDF 根 link 是 pelvis_link（无 world link），补静态变换当 RViz Fixed Frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "world",
                    "--child-frame-id", "pelvis_link",
                ],
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=[str(PKG / "scripts" / "mc_state_to_joint_states.py")],
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=[str(PKG / "scripts" / "a3_mc_controller.py")],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                arguments=["--ros-args", "--params-file", str(PARAMS),
                           "--params-file", str(OVERRIDE)],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(RVIZ_FILE),
                           "--ros-args", "--params-file", str(PARAMS)],
                condition=IfCondition(rviz),
                output="screen",
            ),
            # 联动遥操作：默认 backend 发 /<side>_arm_controller/joint_trajectory
            # 话题进 a3_mc_controller 的流式入口（与 action 共用安全链 + 仲裁）。
            # 注意不能像 teleop_vr.launch.py real:=true 那样覆盖 mc_arm backend。
            Node(
                executable="/usr/bin/python3.12",
                arguments=teleop_args,
                condition=IfCondition(teleop),
                output="screen",
            ),
            Node(
                executable="/usr/bin/python3.12",
                arguments=[
                    str(PKG / "a3_teleop_bridge" / "a3_teleop_bridge_node.py"),
                ],
                condition=IfCondition(bridge),
                output="screen",
            ),
        ]
    )
