#!/usr/bin/env python3
"""由 a3_ultra_isaaclab/urdf/model_planning.* 派生 MoveIt 演示所需文件。

生成：
  config/a3_ultra_t2d0.urdf            # 原样复制（URDF）
  config/a3_ultra_t2d0.srdf            # 原 SRDF + 规划组定义
  config/joint_limits.yaml             # 从 URDF <limit> 提取
  urdf/a3_ultra_t2d0_ros2control.urdf  # 追加 <ros2_control> 标签，供 ros2_control 仿真硬件使用

同时校验 URDF 引用的网格文件名与 a3_ultra_isaaclab/meshes 是否一致。

模型来源（智元官方资产，不入库）用 --model-src 或环境变量 A3_MODEL_SRC 指定，
默认取本工作区的 a3_ultra_isaaclab/。
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

WS = Path(__file__).resolve().parents[4]          # 工作区根（a3_moveit_demo 的上级目录）
PKG = Path(__file__).resolve().parents[1]          # src/a3_moveit_config
DEFAULT_MODEL_SRC = WS / "a3_ultra_isaaclab"

# 规划组定义（追加到 SRDF）
GROUPS_XML = """
    <!-- 规划组定义（本 demo 补充；原始 SRDF 只有 disable_collisions） -->
    <group name="left_arm">
        <chain base_link="torso_Link" tip_link="left_wrist_yaw_Link"/>
    </group>
    <group name="right_arm">
        <chain base_link="torso_Link" tip_link="right_wrist_yaw_Link"/>
    </group>
    <group name="left_leg">
        <chain base_link="pelvis_link" tip_link="left_ankle_roll_Link"/>
    </group>
    <group name="right_leg">
        <chain base_link="pelvis_link" tip_link="right_ankle_roll_Link"/>
    </group>
    <group name="waist">
        <joint name="waist_yaw_joint"/>
        <joint name="waist_roll_joint"/>
        <joint name="waist_pitch_joint"/>
    </group>
    <group name="head">
        <joint name="head_yaw_joint"/>
        <joint name="head_pitch_joint"/>
    </group>
    <group name="whole_body">
        <joint name="waist_yaw_joint"/>
        <joint name="waist_roll_joint"/>
        <joint name="waist_pitch_joint"/>
        <joint name="head_yaw_joint"/>
        <joint name="head_pitch_joint"/>
        <joint name="left_shoulder_pitch_joint"/>
        <joint name="left_shoulder_roll_joint"/>
        <joint name="left_shoulder_yaw_joint"/>
        <joint name="left_elbow_joint"/>
        <joint name="left_wrist_roll_joint"/>
        <joint name="left_wrist_pitch_joint"/>
        <joint name="left_wrist_yaw_joint"/>
        <joint name="right_shoulder_pitch_joint"/>
        <joint name="right_shoulder_roll_joint"/>
        <joint name="right_shoulder_yaw_joint"/>
        <joint name="right_elbow_joint"/>
        <joint name="right_wrist_roll_joint"/>
        <joint name="right_wrist_pitch_joint"/>
        <joint name="right_wrist_yaw_joint"/>
        <joint name="left_hip_pitch_joint"/>
        <joint name="left_hip_roll_joint"/>
        <joint name="left_hip_yaw_joint"/>
        <joint name="left_knee_joint"/>
        <joint name="left_ankle_pitch_joint"/>
        <joint name="left_ankle_roll_joint"/>
        <joint name="right_hip_pitch_joint"/>
        <joint name="right_hip_roll_joint"/>
        <joint name="right_hip_yaw_joint"/>
        <joint name="right_knee_joint"/>
        <joint name="right_ankle_pitch_joint"/>
        <joint name="right_ankle_roll_joint"/>
    </group>
</robot>
"""

ARM_JOINTS = [
    "shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
    "elbow_joint", "wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint",
]
LEG_JOINTS = [
    "hip_pitch_joint", "hip_roll_joint", "hip_yaw_joint",
    "knee_joint", "ankle_pitch_joint", "ankle_roll_joint",
]
WAIST_HEAD_JOINTS = [
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "head_yaw_joint", "head_pitch_joint",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-src", type=Path, default=DEFAULT_MODEL_SRC,
        help="含 urdf/model_planning.{urdf,srdf} 与 meshes/ 的目录"
             "（也可用环境变量 A3_MODEL_SRC 指定）")
    args = parser.parse_args()
    model_src = Path(os.environ.get("A3_MODEL_SRC", args.model_src))
    src_urdf = model_src / "urdf/model_planning.urdf"
    src_srdf = model_src / "urdf/model_planning.srdf"
    mesh_dir = model_src / "meshes"
    if not src_urdf.exists() or not src_srdf.exists() or not mesh_dir.is_dir():
        raise SystemExit(
            f"未找到机器人模型：{model_src}\n"
            "请用 --model-src 或环境变量 A3_MODEL_SRC 指定本机智元官方安装中"
            "包含 urdf/ 与 meshes/ 的目录（如 AimSim 工作区的 a3_ultra_isaaclab）。")

    urdf_text = src_urdf.read_text(encoding="utf-8")

    # ---- 1. 校验网格文件名 ----
    refs = sorted(set(re.findall(r"package://robot_descriptions/[^\"'\s]+", urdf_text)))
    missing = [r for r in refs if not (mesh_dir / r.split("meshes/")[-1]).exists()]
    print(f"URDF 引用网格 {len(refs)} 个，缺失 {len(missing)} 个")
    for m in missing:
        print("  MISSING:", m)

    # ---- 2. 复制 URDF，网格路径改为绝对 file:// 路径 ----
    # package:// 依赖 ament 环境变量，手动启动的 RViz 经常解析失败；
    # file:// 绝对路径任何进程都能直接加载
    urdf_abs = urdf_text.replace(
        "package://robot_descriptions/models/a3_ultra_t2d0/meshes/",
        f"file://{mesh_dir}/",
    )
    (PKG / "config/a3_ultra_t2d0.urdf").write_text(urdf_abs, encoding="utf-8")

    # ---- 3. 生成 SRDF（注入规划组） ----
    srdf_text = src_srdf.read_text(encoding="utf-8")
    assert "</robot>" in srdf_text and "<group" not in srdf_text
    srdf_out = srdf_text.replace("</robot>", GROUPS_XML)
    (PKG / "config/a3_ultra_t2d0.srdf").write_text(srdf_out, encoding="utf-8")

    # ---- 4. 解析 revolute 关节 ----
    root = ET.fromstring(urdf_text)
    joints = []
    for j in root.iter("joint"):
        if j.get("type") != "revolute":
            continue
        limit = j.find("limit")
        joints.append(
            dict(
                name=j.get("name"),
                lower=float(limit.get("lower")),
                upper=float(limit.get("upper")),
                velocity=float(limit.get("velocity")),
                effort=float(limit.get("effort")),
            )
        )
    print(f"revolute 关节数: {len(joints)}")
    names = [j["name"] for j in joints]
    expected = (
        [f"left_{n}" for n in ARM_JOINTS] + [f"right_{n}" for n in ARM_JOINTS]
        + [f"left_{n}" for n in LEG_JOINTS] + [f"right_{n}" for n in LEG_JOINTS]
        + WAIST_HEAD_JOINTS
    )
    assert sorted(names) == sorted(expected), (
        set(names) ^ set(expected))
    print("关节清单与预期一致")

    # ---- 5. joint_limits.yaml ----
    # 注意：AddTimeOptimalParameterization 适配器需要加速度上限，URDF 未提供，
    # 这里给一个保守统一的默认值 5.0 rad/s^2（仅用于仿真演示）
    lines = ["robot_description_planning:", "  joint_limits:"]
    for j in joints:
        lines += [
            f"    {j['name']}:",
            "      has_velocity_limits: true",
            f"      max_velocity: {j['velocity']:.6g}",
            "      has_acceleration_limits: true",
            "      max_acceleration: 5.0",
            "      has_jerk_limits: false",
        ]
    (PKG / "config/joint_limits.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # ---- 6. ros2_control 版 URDF ----
    joint_xml = "\n".join(
        f'''    <joint name="{j['name']}">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>'''
        for j in joints
    )
    ros2control = f'''
  <ros2_control name="MockA3System" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
{joint_xml}
  </ros2_control>
</robot>
'''
    rc_text = urdf_abs.replace("</robot>", ros2control)
    (PKG / "urdf/a3_ultra_t2d0_ros2control.urdf").write_text(rc_text, encoding="utf-8")

    print("派生文件生成完成")
    return 0 if not missing else 1


if __name__ == "__main__":
    sys.exit(main())
