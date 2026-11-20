#!/usr/bin/env python3
"""MoveIt 全身规划演示（move_group 动作客户端）：左臂 -> 右臂 -> 左腿 -> 腰 -> 全身（31 关节）。

流程：对每个规划组发送关节空间目标 -> move_group 用 OMPL 规划并经
FollowJointTrajectory 控制器执行到 ros2_control 仿真硬件（mock_components）。
"""
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.logging import get_logger


def make_goal(group_name, joint_goal):
    msg = MoveGroup.Goal()
    msg.request.group_name = group_name
    msg.request.num_planning_attempts = 10
    msg.request.allowed_planning_time = 15.0
    msg.request.max_velocity_scaling_factor = 0.3
    msg.request.max_acceleration_scaling_factor = 0.3
    constraints = Constraints()
    for joint, value in joint_goal.items():
        jc = JointConstraint()
        jc.joint_name = joint
        jc.position = value
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    msg.request.goal_constraints.append(constraints)
    msg.planning_options.plan_only = False
    msg.planning_options.look_around = False
    msg.planning_options.replan = False
    return msg


def main():
    rclpy.init()
    logger = get_logger("a3_plan_demo")
    node = rclpy.create_node("a3_plan_demo")

    client = ActionClient(node, MoveGroup, "move_action")
    logger.info("等待 move_group 动作服务器 (/move_action) ...")
    if not client.wait_for_server(timeout_sec=60.0):
        logger.error("move_group 动作服务器不可用，退出")
        rclpy.shutdown()
        return

    demos = [
        (
            "left_arm",
            {
                "left_shoulder_pitch_joint": 0.25,
                "left_shoulder_roll_joint": 0.2,
                "left_shoulder_yaw_joint": 0.15,
                "left_elbow_joint": -0.5,
                "left_wrist_roll_joint": 0.2,
                "left_wrist_pitch_joint": 0.1,
                "left_wrist_yaw_joint": 0.1,
            },
        ),
        (
            "right_arm",
            {
                "right_shoulder_pitch_joint": 0.25,
                "right_shoulder_roll_joint": -0.2,
                "right_shoulder_yaw_joint": -0.15,
                "right_elbow_joint": -0.5,
                "right_wrist_roll_joint": -0.2,
                "right_wrist_pitch_joint": -0.1,
                "right_wrist_yaw_joint": -0.1,
            },
        ),
        (
            "left_leg",
            {
                "left_hip_pitch_joint": -0.25,
                "left_hip_roll_joint": 0.1,
                "left_hip_yaw_joint": 0.05,
                "left_knee_joint": 0.4,
                "left_ankle_pitch_joint": -0.15,
                "left_ankle_roll_joint": -0.05,
            },
        ),
        (
            "waist",
            {
                "waist_yaw_joint": 0.2,
                "waist_roll_joint": 0.05,
                "waist_pitch_joint": 0.08,
            },
        ),
        (
            "whole_body",
            {
                "waist_yaw_joint": -0.1,
                "waist_roll_joint": -0.05,
                "waist_pitch_joint": -0.05,
                "head_yaw_joint": 0.15,
                "head_pitch_joint": -0.1,
                "left_shoulder_pitch_joint": -0.15,
                "left_shoulder_roll_joint": 0.2,
                "left_shoulder_yaw_joint": -0.1,
                "left_elbow_joint": 0.35,
                "left_wrist_roll_joint": -0.1,
                "left_wrist_pitch_joint": -0.1,
                "left_wrist_yaw_joint": -0.05,
                "right_shoulder_pitch_joint": -0.15,
                "right_shoulder_roll_joint": -0.2,
                "right_shoulder_yaw_joint": 0.1,
                "right_elbow_joint": 0.35,
                "right_wrist_roll_joint": 0.1,
                "right_wrist_pitch_joint": 0.1,
                "right_wrist_yaw_joint": 0.05,
                "left_hip_pitch_joint": 0.1,
                "left_hip_roll_joint": -0.05,
                "left_hip_yaw_joint": -0.03,
                "left_knee_joint": 0.15,
                "left_ankle_pitch_joint": 0.08,
                "left_ankle_roll_joint": 0.03,
                "right_hip_pitch_joint": 0.1,
                "right_hip_roll_joint": 0.05,
                "right_hip_yaw_joint": 0.03,
                "right_knee_joint": 0.15,
                "right_ankle_pitch_joint": 0.08,
                "right_ankle_roll_joint": -0.03,
            },
        ),
    ]

    for group_name, joint_goal in demos:
        logger.info(f"===== 规划组 [{group_name}] =====")
        send_future = client.send_goal_async(make_goal(group_name, joint_goal))
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error(f"[{group_name}] 目标被拒绝")
            continue
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        error = result.error_code.val
        if error == MoveItErrorCodes.SUCCESS:
            traj = result.planned_trajectory.joint_trajectory
            logger.info(
                f"[{group_name}] 规划+执行成功: {len(traj.points)} 个轨迹点, "
                f"{len(traj.joint_names)} 个关节"
            )
        else:
            logger.error(f"[{group_name}] 失败, MoveItErrorCodes = {error}")
        time.sleep(0.5)

    logger.info("全部演示完成")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
