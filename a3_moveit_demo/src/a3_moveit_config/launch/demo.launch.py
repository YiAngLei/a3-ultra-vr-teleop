#!/usr/bin/env python3
"""A3_ULTRA_T2D0 MoveIt 全身规划仿真 launch。

启动：
  robot_state_publisher（带 ros2_control 标签的 URDF，提供 /tf 与 robot_description）
  ros2_control_node（mock_components/GenericSystem 仿真硬件 + 轨迹控制器）
  move_group（MoveIt 规划主节点，加载 a3_plan_demo_params.yaml）
  rviz2（可选，MotionPlanning 显示）
  plan_demo.py（move_group 动作客户端，延时等待控制器就绪）

启动参数：
  rviz          true/false（默认 true）
  demo_plan     true/false（默认 true）：false 时不启动 plan_demo.py
  teleop        trigger/none（默认 trigger）：none 时不启动 arm_teleop.py
                （流式 VR 遥操作由独立的 a3_teleop_vr 工作区拉起，见 ../a3_teleop_vr）
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parents[1]
# robot_state_publisher 与 ros2_control_node 必须使用同一份带 <ros2_control> 标签的 URDF：
# controller_manager 会订阅 /robot_description 话题并覆盖本地参数，两份内容不一致会直接崩溃
ROS2CONTROL_FILE = PKG / "urdf" / "a3_ultra_t2d0_ros2control.urdf"
PARAMS_FILE = PKG / "config" / "a3_plan_demo_params.yaml"
RVIZ_FILE = PKG / "rviz" / "moveit.rviz"


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    ros2control_text = ROS2CONTROL_FILE.read_text(encoding="utf-8")

    nodes = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("demo_plan", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="trigger"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": ros2control_text}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": ros2control_text},
                str(PKG / "config" / "ros2_controllers.yaml"),
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_controller", "right_arm_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_leg_controller", "right_leg_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["waist_controller", "head_controller"],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            arguments=["--ros-args", "--params-file", str(PARAMS_FILE)],
            output="screen",
        ),
        # URDF 根 link 是 pelvis_link（无 world link），/tf 中没有它的坐标系，
        # RViz 的 Fixed Frame 会失效导致什么都不显示，这里补一个 world->pelvis_link 静态变换
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
        # 等控制器激活后再开始规划演示
        # demo_plan/teleop 两个参数让外部 launch（如 a3_teleop_vr）可以复用整套
        # 仿真栈而不拉起默认的演示/遥操作脚本，默认行为与原来完全一致
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    executable="/usr/bin/python3.12",
                    arguments=[str(PKG / "scripts" / "plan_demo.py")],
                    condition=IfCondition(LaunchConfiguration("demo_plan")),
                    output="screen",
                ),
                Node(
                    executable="/usr/bin/python3.12",
                    arguments=[str(PKG / "scripts" / "arm_teleop.py")],
                    condition=IfCondition(
                        PythonExpression(["'", LaunchConfiguration("teleop"), "' == 'trigger'"])
                    ),
                    output="screen",
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(RVIZ_FILE),
                "--ros-args",
                "--params-file",
                str(PARAMS_FILE),
            ],
            condition=IfCondition(rviz),
            output="screen",
        ),
    ]
    return LaunchDescription(nodes)
