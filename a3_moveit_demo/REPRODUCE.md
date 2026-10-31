# A3_ULTRA_T2D0 MoveIt 全身规划仿真 —— 从零复现指南

> 目标读者：拿到**原始 AimSim 工作区**（只有 `a3_ultra_isaaclab/urdf/model_planning.{urdf,srdf}`
> 和三个 motion_control 发布包，**没有** `a3_moveit_demo/`）的人，按本文档操作后得到：
> 一条命令启动的 MoveIt 2 全身规划仿真（OMPL 规划 + ros2_control 虚拟执行 + RViz 可视化）。

---

## 0. 起点与终点

**起点**（原始工程提供）：

- `a3_ultra_isaaclab/urdf/model_planning.urdf` —— A3_ULTRA_T2D0 **全身**模型：
  31 个驱动关节（双臂 7×2、双腿 6×2、腰 3、头 2），根 link 是 `pelvis_link`（**无 world link**）。
- `a3_ultra_isaaclab/urdf/model_planning.srdf` —— 只有 284 条 `<disable_collisions>`，
  **没有任何 `<group>`**，直接给 MoveIt 无法规划。
- `a3_ultra_isaaclab/meshes/` —— 47 个 STL 网格。

**终点**：`a3_moveit_demo/` 目录，

```bash
cd ~/AimSim/a3_moveit_demo
source setup_demo_env.bash
ros2 launch a3_moveit_config demo.launch.py    # 含 RViz；rviz:=false 可无界面
```

成功标准（launch 日志中出现）：

```
[left_arm]     规划+执行成功: 13 个轨迹点, 7 个关节
[right_arm]    规划+执行成功: 13 个轨迹点, 7 个关节
[left_leg]     规划+执行成功: 12 个轨迹点, 6 个关节
[waist]        规划+执行成功: 9 个轨迹点, 3 个关节
[whole_body]   规划+执行成功: 17 个轨迹点, 31 个关节
```

## 1. 环境前提（本机实测）

- Ubuntu 24.04 + ROS 2 **Jazzy**（`/opt/ros/jazzy`），已装完整 MoveIt 2.12.4
  （`moveit_ros_move_group`、`moveit_py`、`moveit_configs_utils`、rviz2 及
  moveit rviz 插件、ros2_control 全家桶：controller_manager / JTC / JSB / mock_components）。
- **系统的 `/usr/bin/python3` 是 3.12**（Jazzy 原生匹配），但机器上装了 miniconda，
  `python3` 默认指向 **conda 的 3.14**——这是后面一切 Python 报错的根源，见坑 1。
- 无源码编译（免 colcon），用"手工 ament 前缀"让 ROS 找到我们的配置包，见第 2 节。

自检命令：

```bash
ls /opt/ros/jazzy/share | grep -E 'moveit_ros_move_group|moveit_py|ros2_control|^rviz2$'
/usr/bin/python3 --version                      # 必须是 3.12.x
bash -c 'source /opt/ros/jazzy/setup.bash && /usr/bin/python3.12 \
  -c "from moveit.planning import MoveItPy; print(\"moveit_py OK\")"'
```

## 2. 创建目录骨架（免 colcon 的手工 ament 前缀）

ROS 通过 ament 索引找包。不构建源码的最简办法：建一个 `prefix/`，
里面用符号链接把包"挂"进 `share/`，再在 `ament_index` 里放标记文件。
之后 `ros2 launch <pkg>`、`get_package_share_directory('<pkg>') 都能解析到。

```bash
cd ~/AimSim
mkdir -p a3_moveit_demo/src/a3_moveit_config/{config,urdf,launch,scripts,rviz}
mkdir -p a3_moveit_demo/prefix/share/ament_index/resource_index/packages
ln -sfn ../../src/a3_moveit_config a3_moveit_demo/prefix/share/a3_moveit_config
touch a3_moveit_demo/prefix/share/ament_index/resource_index/packages/a3_moveit_config
```

最终结构（文件内容在第 3 节逐个给出）：

```
a3_moveit_demo/
├── REPRODUCE.md                     # 本文档
├── setup_demo_env.bash              # 环境入口（source 它）
├── prefix/                          # 手工 ament 前缀
│   └── share/
│       ├── ament_index/resource_index/packages/a3_moveit_config
│       └── a3_moveit_config -> ../../src/a3_moveit_config
└── src/a3_moveit_config/
    ├── package.xml
    ├── config/
    │   ├── a3_ultra_t2d0.urdf        # 派生：原始 URDF 复制 + 网格路径改 file://
    │   ├── a3_ultra_t2d0.srdf        # 派生：原始 SRDF + 7 个规划组
    │   ├── joint_limits.yaml         # 派生：从 URDF <limit> 提取 + 补加速度
    │   ├── kinematics.yaml           # 手臂/腿的 KDL IK 配置
    │   ├── moveit_controllers.yaml   # MoveIt 轨迹执行控制器映射（每组一个）
    │   ├── ros2_controllers.yaml     # ros2_control 控制器（mock 硬件）
    │   └── a3_plan_demo_params.yaml  # 派生：move_group/RViz 共用的参数文件
    ├── urdf/a3_ultra_t2d0_ros2control.urdf  # 派生：追加 <ros2_control> 标签
    ├── launch/demo.launch.py
    ├── scripts/gen_derived_files.py  # 派生文件生成器
    ├── scripts/gen_params_file.py    # 参数文件生成器
    ├── scripts/plan_demo.py          # move_group 动作客户端演示
    ├── scripts/arm_teleop.py         # Pico VR 手柄双臂遥操作（可选扩展）
    └── rviz/moveit.rviz
```

## 3. 逐个创建文件

### 3.1 环境脚本 `a3_moveit_demo/setup_demo_env.bash`

三个作用：加载 Jazzy；把手工前缀加进 `AMENT_PREFIX_PATH`（包发现）；
把 `/usr/bin` 前置，**挡住 conda 的 python3.14**（见坑 1）。

```bash
#!/usr/bin/bash
# A3_ULTRA_T2D0 MoveIt 演示环境。用法: source 本文件
# 要点：
#   1. 加载 ROS 2 Jazzy
#   2. 把手工搭建的 ament 前缀加入 AMENT_PREFIX_PATH，使 a3_moveit_config
#      （launch/配置/参数生成器）可被解析；网格已改用 file:// 绝对路径，
#      不再需要 robot_descriptions 资源包
#   3. /usr/bin 前置，避免 conda 的 python3.14 抢在 ROS 的 python3.12 前面
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="~/AimSim/a3_moveit_demo/prefix:${AMENT_PREFIX_PATH}"
export PATH=/usr/bin:$PATH
echo "[a3_moveit_demo] 环境就绪，运行: ros2 launch a3_moveit_config demo.launch.py"
```

### 3.2 包描述 `a3_moveit_demo/src/a3_moveit_config/package.xml`

内容随意，有合法的 `name` 即可（ament 索引靠目录名 + 标记文件）。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>a3_moveit_config</name>
  <version>0.0.1</version>
  <description>
    智元 A3_ULTRA_T2D0 全身模型的 MoveIt 规划演示配置。
    URDF/SRDF 来源于 a3_ultra_isaaclab/urdf/model_planning.*，
    在原始 SRDF（只有 disable_collisions）基础上补充了规划组定义。
  </description>
  <maintainer email="demo@example.com">demo</maintainer>
  <license>BSD</license>
</package>
```

### 3.3 派生文件生成器 `scripts/gen_derived_files.py`

一次生成四样东西（**这是核心脚本**，直接决定成败）：

1. `config/a3_ultra_t2d0.urdf`：原样复制 `model_planning.urdf`，但把网格路径
   `package://robot_descriptions/...` 全部替换为绝对路径 `file://<工作区>/a3_ultra_isaaclab/meshes/...`。
   —— `package://` 依赖 ament 环境变量，手动起的 RViz 解析不到网格（见坑 9），
   `file://` 任何进程都能读。
2. `config/a3_ultra_t2d0.srdf`：在原始 SRDF（只有 disable_collisions）的 `</robot>` 前
   注入 **7 个规划组**：`left_arm`/`right_arm`（7 关节链，torso→手腕）、
   `left_leg`/`right_leg`（6 关节链，pelvis→脚踝）、`waist`（3 关节）、`head`（2 关节）、
   `whole_body`（全部 31 关节）。**没有 group，MoveIt 什么都不能规划。**
3. `config/joint_limits.yaml`：从 URDF `<limit>` 提取速度上限，
   并给每个关节补 `max_acceleration: 5.0` —— OMPL 规划成功后的
   `AddTimeOptimalParameterization` 适配器**必须有加速度上限**否则报 FAILURE（见坑 5）。
4. `urdf/a3_ultra_t2d0_ros2control.urdf`：在 URDF 末尾追加 `<ros2_control>` 标签，
   31 个关节每个声明 `position` 命令接口 + `position/velocity` 状态接口，
   硬件插件 `mock_components/GenericSystem`（ros2_control 的仿真硬件，所有关节初值为 0）。

同时校验 URDF 引用的网格文件是否都存在、31 个关节清单是否符合预期。

```python
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
```

### 3.4 运动学配置 `config/kinematics.yaml`

四个链式组用 KDL 数值解（7 自由度手臂无解析解，KDL 够用）。
`waist`/`head`/`whole_body` 只做关节空间规划，不需要 IK，不配。

```yaml
# 运动学求解器配置：为各规划组指定 IK 插件
# 全身/腰/头组只做关节空间规划，不需要 IK，可不配

left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3

right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3

left_leg:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3

right_leg:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

### 3.5 MoveIt 控制器映射 `config/moveit_controllers.yaml`

告诉 MoveIt 的轨迹执行管理器：轨迹发给哪个 FollowJointTrajectory 动作。
**每组一个控制器**，且关节列表必须与规划组**完全一致**——Jazzy 的
JointTrajectoryController 不再接受"轨迹关节是控制器关节子集"（见坑 6）。
注意第一行 `moveit_controller_manager` 插件声明不能少。

```yaml
# MoveIt 轨迹执行控制器配置（moveit_simple_controller_manager）
# 与 ros2_controllers.yaml 一一对应；每个控制器的关节列表必须与对应规划组完全一致
# （Jazzy 的 JointTrajectoryController 要求轨迹关节列表与控制器关节精确匹配，不接受子集）

trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01
  execution_duration_monitoring: true

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - left_arm_controller
    - right_arm_controller
    - left_leg_controller
    - right_leg_controller
    - waist_controller
    - head_controller

  left_arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - left_shoulder_pitch_joint
      - left_shoulder_roll_joint
      - left_shoulder_yaw_joint
      - left_elbow_joint
      - left_wrist_roll_joint
      - left_wrist_pitch_joint
      - left_wrist_yaw_joint

  right_arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - right_shoulder_pitch_joint
      - right_shoulder_roll_joint
      - right_shoulder_yaw_joint
      - right_elbow_joint
      - right_wrist_roll_joint
      - right_wrist_pitch_joint
      - right_wrist_yaw_joint

  left_leg_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - left_hip_pitch_joint
      - left_hip_roll_joint
      - left_hip_yaw_joint
      - left_knee_joint
      - left_ankle_pitch_joint
      - left_ankle_roll_joint

  right_leg_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - right_hip_pitch_joint
      - right_hip_roll_joint
      - right_hip_yaw_joint
      - right_knee_joint
      - right_ankle_pitch_joint
      - right_ankle_roll_joint

  waist_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - waist_yaw_joint
      - waist_roll_joint
      - waist_pitch_joint

  head_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - head_yaw_joint
      - head_pitch_joint
```

### 3.6 ros2_control 控制器 `config/ros2_controllers.yaml`

controller_manager 本体 + joint_state_broadcaster + 6 个轨迹控制器
（与 3.5 一一对应）。

```yaml
# ros2_control 控制器配置（配合 mock_components/GenericSystem 仿真硬件）
# 每个 FollowJointTrajectory 控制器的关节列表与对应规划组完全一致

controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    left_leg_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_leg_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    waist_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    head_controller:
      type: joint_trajectory_controller/JointTrajectoryController

left_arm_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - left_shoulder_pitch_joint
      - left_shoulder_roll_joint
      - left_shoulder_yaw_joint
      - left_elbow_joint
      - left_wrist_roll_joint
      - left_wrist_pitch_joint
      - left_wrist_yaw_joint
    allow_nonzero_velocity_at_trajectory_end: true

right_arm_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - right_shoulder_pitch_joint
      - right_shoulder_roll_joint
      - right_shoulder_yaw_joint
      - right_elbow_joint
      - right_wrist_roll_joint
      - right_wrist_pitch_joint
      - right_wrist_yaw_joint
    allow_nonzero_velocity_at_trajectory_end: true

left_leg_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - left_hip_pitch_joint
      - left_hip_roll_joint
      - left_hip_yaw_joint
      - left_knee_joint
      - left_ankle_pitch_joint
      - left_ankle_roll_joint
    allow_nonzero_velocity_at_trajectory_end: true

right_leg_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - right_hip_pitch_joint
      - right_hip_roll_joint
      - right_hip_yaw_joint
      - right_knee_joint
      - right_ankle_pitch_joint
      - right_ankle_roll_joint
    allow_nonzero_velocity_at_trajectory_end: true

waist_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - waist_yaw_joint
      - waist_roll_joint
      - waist_pitch_joint
    allow_nonzero_velocity_at_trajectory_end: true

head_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - head_yaw_joint
      - head_pitch_joint
    allow_nonzero_velocity_at_trajectory_end: true
```

### 3.7 参数文件生成器 `scripts/gen_params_file.py`

把 MoveItConfigsBuilder 组装好的全部配置写成**标准 ROS 2 参数文件**
`config/a3_plan_demo_params.yaml`，用 `/**` 通配节点名。
这个文件同时给 move_group 和 RViz 用（见坑 3、坑 8）。

```python
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
```

### 3.8 启动文件 `launch/demo.launch.py`

节点及理由（顺序即启动顺序）：

| 节点 | 要点 |
|---|---|
| robot_state_publisher | 发布 `/robot_description`（话题）与 `/tf`。**必须用带 ros2_control 标签的那份 URDF**——controller_manager 会订阅 `/robot_description` 话题并覆盖自己的本地参数，两份不一致直接崩（见坑 2）。 |
| ros2_control_node | 加载 3.6 的 yaml；GenericSystem 仿真硬件。 |
| spawner ×3 | 激活 JSB + 6 个轨迹控制器。 |
| static_transform_publisher | 发布 `world → pelvis_link` 静态变换。URDF 根 link 没有 world link，`/tf` 里不存在 `pelvis_link`，**RViz 的 Fixed Frame 会失效导致整个视图空白**（见坑 7）。 |
| move_group | MoveIt 规划主节点，`--params-file` 挂 3.7 的产物。 |
| plan_demo.py | 延时 10s 启动（等控制器激活），move_group 动作客户端。 |
| rviz2 | `-d` 加载 3.10 的配置，`--params-file` 挂同一份参数文件（RViz 的 MotionPlanning 面板需要 `robot_description_semantic` 参数，见坑 8）。 |

```python
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
```

### 3.9 演示脚本 `scripts/plan_demo.py`

move_group 的 **action 名是 `/move_action`**（不是 `/move_group`！见坑 4）。
对 5 个规划组依次构造 `JointConstraint` 目标（当前零位附近的小幅偏移，
避开自碰撞），发 MoveGroup action，MoveIt 2 内部完成
"OMPL 规划 → 时间参数化 → 按控制器拆分轨迹 → 执行"，返回错误码。

```python
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
```

### 3.10 RViz 配置 `rviz/moveit.rviz`

手写 RViz 配置必须字段齐全（`Name: root`、Panels、各 Display 的完整属性），
否则会被静默丢弃导致一个 display 都没有（这就是"打开 RViz 什么都看不到"的
第二个原因）。关键三处：`Fixed Frame: world`（配合静态 TF）、
RobotModel 从 `/robot_description` **话题**取模型（`Durability: Transient Local`）、
MotionPlanning 面板。

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /MotionPlanning1
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
      Value: false
    - Class: moveit_rviz_plugin/MotionPlanning
      Enabled: true
      Name: MotionPlanning
      Planning Request:
        Planning Group: left_arm
      Planning Scene Topic: /monitored_planning_scene
      Robot Description: robot_description
      Scene Robot:
        Attached Body Color: 150; 50; 150
        Show Robot Visual: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Angle: -1.5708
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0.5
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 3.1416
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: true
  QMainWindow State: ""
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1280
  X: 50
  Y: 50
```

### 3.11 遥操作脚本 `scripts/arm_teleop.py`（可选扩展）

Pico VR 手柄 -> 双臂的**相对运动**遥操作。不要求头显世界位姿：

- 首次按 `button_x`（左）/`button_a`（右）= 校准：记录手柄起点 `T_hmd_c0`
  与机器人当前末端 `T_ee0`（TF 查 `*_wrist_yaw_Link`）；
- 之后再按 = 计算增量 `D = T_hmd_c0⁻¹·T_hmd_c(t)`，目标 `T_ee0·D`
  以 PositionConstraint + OrientationConstraint 形式发给 `/move_action`。

注意：手柄 pose 相对**头**而非肩膀，头部运动会耦合进增量；操作员面向机器人、
头部基本不动时可用。连续跟随（实时拖拽）应改用 moveit_servo，本脚本为触发式。

```python
#!/usr/bin/env python3
"""Pico VR 手柄 -> A3 双臂相对运动遥操作（触发式）。

映射原理（相对运动，不需要头显世界位姿）：
  校准时刻记录  T_hmd_c0（手柄在位姿起点）与 T_world_ee0（机器人当前末端）
  任意时刻增量  D = T_hmd_c0^-1 · T_hmd_c(t)      （手相对起点的 6DOF 位移）
  目标位姿      T_world_ee = T_world_ee0 · D

交互（左右臂对称）：
  第一次按 button_x / button_a  -> 校准该手臂
  之后再按                     -> 以当前增量发送 pose goal 给 move_action

注意：话题给的是"手柄相对头"的位姿，头部运动会耦合进增量；
操作员面向机器人、头部基本不动时影响很小（见 REPRODUCE.md 踩坑与对策）。
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

WORLD_FRAME = "world"

# (规划组, 末端 link, pose 话题, 按键话题)
ARMS = {
    "left": (
        "left_arm",
        "left_wrist_yaw_Link",
        "/pico_left_controller/pose",
        "/pico_left_controller/button_x",
    ),
    "right": (
        "right_arm",
        "right_wrist_yaw_Link",
        "/pico_right_controller/pose",
        "/pico_right_controller/button_a",
    ),
}


def quat_to_rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def rot_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array(
            [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
        )
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q / np.linalg.norm(q)


def pose_to_T(pose):
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    r = quat_to_rot([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def T_to_pose(T):
    pose = PoseStamped()
    q = rot_to_quat(T[:3, :3])
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3]
    return pose


def transform_to_T(tf):
    """geometry_msgs/Transform（translation+rotation）-> 4x4 齐次矩阵。"""
    T = np.eye(4)
    q = tf.rotation
    T[:3, :3] = quat_to_rot([q.x, q.y, q.z, q.w])
    T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return T


def make_pose_goal(group, ee_link, target: PoseStamped):
    """把目标位姿写成 MoveGroup action 的位置+姿态约束（move_group_interface 同款做法）。"""
    msg = MoveGroup.Goal()
    msg.request.group_name = group
    msg.request.num_planning_attempts = 5
    msg.request.allowed_planning_time = 10.0
    msg.request.max_velocity_scaling_factor = 0.3
    msg.request.max_acceleration_scaling_factor = 0.3

    pos = PositionConstraint()
    pos.header.frame_id = target.header.frame_id
    pos.link_name = ee_link
    pos.constraint_region.primitives.append(SolidPrimitive())
    pos.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
    pos.constraint_region.primitives[0].dimensions = [0.005]
    pos.constraint_region.primitive_poses.append(target.pose)
    pos.weight = 1.0

    ori = OrientationConstraint()
    ori.header.frame_id = target.header.frame_id
    ori.link_name = ee_link
    ori.orientation = target.pose.orientation
    ori.absolute_x_axis_tolerance = 0.02
    ori.absolute_y_axis_tolerance = 0.02
    ori.absolute_z_axis_tolerance = 0.02
    ori.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pos)
    c.orientation_constraints.append(ori)
    msg.request.goal_constraints.append(c)
    msg.planning_options.plan_only = False
    msg.planning_options.look_around = False
    msg.planning_options.replan = False
    return msg


class ArmTeleop:
    def __init__(self, node: Node, side: str, shared: dict):
        self.node = node
        self.shared = shared
        self.logger = get_logger(f"arm_teleop.{side}")
        self.group, self.ee_link, pose_topic, button_topic = ARMS[side]
        self.side = side

        self.latest_pose = None
        self.T_hand0 = None
        self.T_ee0 = None
        self.prev_pressed = False

        node.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        node.create_subscription(Bool, button_topic, self.on_button, 10)
        self.client = ActionClient(node, MoveGroup, "move_action")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def on_pose(self, msg):
        self.latest_pose = msg

    def lookup_ee(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, self.ee_link, rclpy.time.Time(), Duration(seconds=0.2)
            )
        except Exception as exc:  # noqa: BLE001
            self.logger.warn(f"TF 查询失败 ({WORLD_FRAME}<-{self.ee_link}): {exc}")
            return None
        return transform_to_T(tf.transform)

    def on_button(self, msg):
        pressed = bool(msg.data)
        if pressed == self.prev_pressed:
            return
        self.prev_pressed = pressed
        if not pressed:
            return

        if self.latest_pose is None:
            self.logger.warn("还没收到手柄 pose，忽略")
            return

        T_hand = pose_to_T(self.latest_pose.pose)
        T_ee = self.lookup_ee()
        if T_ee is None:
            return

        if self.T_hand0 is None:
            self.T_hand0 = T_hand
            self.T_ee0 = T_ee
            self.logger.info(
                f"已校准：手柄起点与 {self.ee_link} 当前位姿绑定，再按一次发送目标"
            )
            return

        D = np.linalg.inv(self.T_hand0) @ T_hand
        target = self.T_ee0 @ D
        goal_pose = T_to_pose(target)
        goal_pose.header.frame_id = WORLD_FRAME
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()

        if not self.client.server_is_ready():
            self.logger.warn("move_action 不可用，忽略本次触发")
            return

        # move_group 不支持并发 goal：两个 goal 同时进入会把 TrajectoryExecutionManager
        # 打出段错误。单执行器回调本身串行，用共享标志互斥即可。
        if self.shared["busy"]:
            self.logger.warn("已有目标在执行中，本次触发忽略（等上一个完成再按）")
            return
        self.shared["busy"] = True

        self.logger.info(
            f"发送目标: pos=({goal_pose.pose.position.x:.3f}, "
            f"{goal_pose.pose.position.y:.3f}, {goal_pose.pose.position.z:.3f})"
        )
        future = self.client.send_goal_async(make_pose_goal(self.group, self.ee_link, goal_pose))
        future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.shared["busy"] = False
            self.logger.error("目标被拒绝")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        self.shared["busy"] = False
        error = future.result().result.error_code.val
        if error == MoveItErrorCodes.SUCCESS:
            self.logger.info("规划+执行成功")
        else:
            self.logger.error(f"失败, MoveItErrorCodes = {error}")


def main():
    rclpy.init()
    node = Node("arm_teleop")
    shared = {"busy": False}
    teleops = [ArmTeleop(node, side, shared) for side in ARMS]
    get_logger("arm_teleop").info(
        "等待按键：左臂 button_x / 右臂 button_a（首次按下=校准，之后=发送目标）"
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

## 4. 生成派生文件并启动

```bash
cd ~/AimSim/a3_moveit_demo
source setup_demo_env.bash

# 生成 4 个派生文件 + 校验（应输出：网格缺失 0、revolute 关节 31、清单一致）
/usr/bin/python3.12 src/a3_moveit_config/scripts/gen_derived_files.py

# 生成 move_group/RViz 参数文件（MoveItConfigsBuilder 需要 ament 能找到 a3_moveit_config，
# 所以必须先 source 环境）
/usr/bin/python3.12 src/a3_moveit_config/scripts/gen_params_file.py

# 启动（无界面服务器加 rviz:=false）
ros2 launch a3_moveit_config demo.launch.py
```

以后改了原始 URDF/SRDF 或任何 config，重跑两个生成脚本即可。

## 5. 验收清单

- [ ] `ros2 launch` 后 RViz 显示完整带网格的 A3 机器人（立于原点）。
- [ ] 日志出现 5 组"规划+执行成功"（见第 0 节）。
- [ ] RViz MotionPlanning 面板的 Planning Group 下拉能选到 left_arm 等 7 个组。
- [ ] 拖动机器人手臂目标点、点 Plan & Execute，能看到机器人动起来。

## 6. 踩坑记录（全部实测踩过）

**坑 1 — conda python3.14 抢 PATH**
现象：`import moveit.planning` 报 `ModuleNotFoundError`，或直接加载 .so 时报
`undefined symbol: _PyThreadState_UncheckedGet`。
原因：conda 的 Python 3.14 在 PATH 最前，而 ROS Jazzy/moveit_py 是 3.12 编译的。
对策：`setup_demo_env.bash` 里 `export PATH=/usr/bin:$PATH`；launch 里的 Python
节点显式用 `/usr/bin/python3.12`；所有 ros2 CLI 操作也用 source 过的 shell。

**坑 2 — ros2_control_node 崩溃 `no 'ros2_control' tag found in the URDF`**
原因：Jazzy 的 controller_manager 会订阅 `/robot_description` **话题**并用它覆盖
本地参数；RSP 发布的是普通 URDF（无 ros2_control 标签）时，CM 拿到的就是不带标签的版本。
对策：RSP 与 CM 使用**同一份**带标签的 URDF（本文方案）。

**坑 3 — 放弃 moveit_py，改用 move_group**
现象：`MoveItPy(node_name=..., config_dict=...)` 初始化时
`Failed to load any planning pipelines`；改用 `--params-file` 后参数明明都在
（逐键验证过）仍失败。原因未完全定位（疑似 pybind 参数转换丢失嵌套结构）。
对策：用久经考验的 **move_group 节点 + 标准参数文件**（`/**` 通配，兼容
moveit_py 内部对节点的随机改名；move_group 节点名固定为 `move_group` 也能匹配）。
参数文件由 `MoveItConfigsBuilder.to_dict()` 生成，保证 key 与 MoveIt 2.12 期望的一致。

**坑 4 — action 名是 `/move_action` 不是 `/move_group`**
现象：动作客户端 `wait_for_server` 超时。
MoveIt 2 的 move_group 节点提供的能力列表里写的是 `move_action`。

**坑 5 — 缺加速度上限导致规划结果报废**
现象：`No acceleration limit was defined for joint ...` →
`AddTimeOptimalParameterization failed` → MoveItErrorCodes=99999(FAILURE)。
URDF 只有 velocity/effort 没有 acceleration，必须在 joint_limits.yaml 补。

**坑 6 — JTC 要求轨迹关节与控制器关节精确一致**
现象：`Joints on incoming trajectory don't match the controller joints`，
Goal rejected，MoveItErrorCodes=-4(CONTROL_FAILED)。
把左右腿合并成一个 12 关节控制器、轨迹只含单侧 6 关节时必然触发。
对策：每个规划组一个控制器，关节列表与组完全一致。

**坑 7 — URDF 无 world link 导致 RViz 全空白**
现象：RViz 顶部 "Fixed Frame [pelvis_link] does not exist"，网格/网格都没有。
`/tf` 只为"关节"发变换，根 link 永远没有坐标系。
对策：static_transform_publisher 补 `world→pelvis_link`，Fixed Frame 用 `world`。

**坑 8 — RViz 拿不到 SRDF 参数**
现象：`Could not find parameter robot_description_semantic → Unable to parse SRDF`。
RViz 的 MotionPlanning 面板从**自己节点**的参数读 URDF/SRDF，不是从话题。
对策：RViz 节点同样 `--params-file a3_plan_demo_params.yaml`。

**坑 9 — RViz 加载不了 `package://` 网格**
现象：`Could not load resource [package://robot_descriptions/...]`。
RViz 进程没有 `AMENT_PREFIX_PATH`（比如手动 `ros2 run rviz2 rviz2`）。
对策（本文采用）：URDF 网格路径用 `file://` 绝对路径，彻底摆脱 ament 环境依赖。
（备选方案：建一个只放 meshes 软链的 `robot_descriptions` ament 包——本工作区
`prefix/share/robot_descriptions` 即历史遗留，现已不需要。）

**坑 10 — 同一台机器同时跑两个实例**
两套同名节点/服务在同一个 DDS 域互相发现，spawner 可能连上另一个实例的
controller_manager，报出"no controller with this name exists"之类的**假故障**。
验证前务必确认没有残留进程：`pgrep -af "ros2 launch|ros2_control_node"`。

**坑 11 — 并发 goal 会打崩 move_group**
现象：左右手柄同时按下，两个 move_action goal 间隔毫秒级进入同一 move_group，
先 CONTROL_FAILED，随后 move_group 在 TrajectoryExecutionManager 中段错误退出，
之后所有触发只剩 "move_action 不可用"。
原因：move_group 的 TrajectoryExecutionManager 非线程安全，同一时刻只接受一个 goal。
对策：客户端侧互斥——`arm_teleop.py` 用共享 `busy` 标志，有目标在执行时
丢弃新触发（rclpy 单线程执行器下 check-and-set 是原子的）。双臂严格"同时"动
需要起两个 move_group 实例或事先合并为双臂 goal，触发式场景用互斥即可。

## 7. 关键原理回顾（回答"为什么能跑"）

- **MoveIt 规划组**：URDF 是"硬件描述"，SRDF 是"语义描述"。原始 SRDF 只有碰撞对，
  本文档注入 group 后 MoveIt 才知道哪些关节一起动。
- **通信链路**：`plan_demo.py` --(MoveGroup action)--> `move_group` --(OMPL 规划)-->
  时间参数化 --> 按控制器拆分 --> `FollowJointTrajectory action` -->
  `ros2_control_node`(mock 硬件) --> `/joint_states` --> RSP --> `/tf` --> RViz。
- **"仿真"在哪**：`mock_components/GenericSystem` 把轨迹命令直接写回关节状态，
  形成一个假电机闭环，因此 Plan 和 Execute 都是真实流程，只是没有物理引擎。
