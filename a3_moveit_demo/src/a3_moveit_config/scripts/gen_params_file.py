#!/usr/bin/env python3
"""生成 config/a3_plan_demo_params.yaml。

把 MoveItConfigsBuilder 组装的配置写成标准 ROS 2 参数文件，plan_demo 节点启动时
通过 `--params-file` 加载。用 `/**` 通配节点名，兼容 moveit_py 内部对节点的重命名。
"""
import sys
from pathlib import Path

import yaml
from moveit_configs_utils import MoveItConfigsBuilder

PKG = Path(__file__).resolve().parents[1]


def build_params():
    builder = MoveItConfigsBuilder("a3_ultra_t2d0", package_name="a3_moveit_config")
    builder.robot_description()
    builder.robot_description_semantic()
    builder.robot_description_kinematics()
    builder.joint_limits()
    builder.trajectory_execution()
    builder.planning_scene_monitor()
    # 只加载 ompl 管线，避免默认 pilz 管线要求额外的 cartesian limits 配置
    builder.planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
    return builder.to_dict()


def main():
    params = build_params()
    # builder 对 joint_limits.yaml 多包了一层 robot_description_planning，解掉
    rdp = params.get("robot_description_planning")
    if isinstance(rdp, dict) and "robot_description_planning" in rdp:
        params["robot_description_planning"] = rdp["robot_description_planning"]
    out = {"/**": {"ros__parameters": params}}
    target = PKG / "config" / "a3_plan_demo_params.yaml"
    target.write_text(
        yaml.safe_dump(out, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"written: {target} ({target.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
