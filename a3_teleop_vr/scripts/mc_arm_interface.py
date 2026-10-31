#!/usr/bin/env python3.12
"""智元 MC 官方手臂接口（AimDK A3 Ultra 文档 7.1.5）共享常量。

a3_streaming_teleop.py（流式遥操作 mc_arm 模式）与 a3_mc_controller.py
（FollowJointTrajectory 执行适配器）共用：固定 14 关节顺序 + 官方角度限位。
"""
# 固定 14 关节顺序（name 字段必须按此顺序填写，先左后右）
MC_ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# 官方角度限位（rad，文档 7.1.5 备注）
MC_JOINT_LIMITS = {
    "left_shoulder_pitch_joint": (-2.87979, 2.87979),
    "left_shoulder_roll_joint": (-0.08727, 2.61799),
    "left_shoulder_yaw_joint": (-2.79253, 2.79253),
    "left_elbow_joint": (-0.95993, 1.74533),
    "left_wrist_roll_joint": (-2.79253, 2.79253),
    "left_wrist_pitch_joint": (-1.62316, 1.62316),
    "left_wrist_yaw_joint": (-1.62316, 1.62316),
    "right_shoulder_pitch_joint": (-2.87979, 2.87979),
    "right_shoulder_roll_joint": (-2.61799, 0.08727),
    "right_shoulder_yaw_joint": (-2.79253, 2.79253),
    "right_elbow_joint": (-0.95993, 1.74533),
    "right_wrist_roll_joint": (-2.79253, 2.79253),
    "right_wrist_pitch_joint": (-1.62316, 1.62316),
    "right_wrist_yaw_joint": (-1.62316, 1.62316),
}
