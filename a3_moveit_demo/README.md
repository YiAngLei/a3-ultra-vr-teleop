# A3_ULTRA_T2D0 MoveIt 全身规划演示

基于 `a3_ultra_isaaclab/urdf/model_planning.urdf`（31 个驱动关节的**全身**模型） +
 在原始 `model_planning.srdf` 基础上**补充了规划组定义**的 SRDF，
用 MoveIt 2 (Jazzy, move_group + OMPL) + ros2_control 仿真硬件（mock_components/GenericSystem）
完成"规划 + 执行"全流程演示。

## 背景

- **MoveIt 不是只能规划手臂**。它对任何"规划组（planning group）"都能规划：
  手臂、腿、腰、头，甚至把 31 个关节全部编成一个 `whole_body` 组做全身关节空间规划。
  实际使用中对手臂最常用，只是因为抓取类任务最成熟。
- **URDF vs SRDF**：URDF 只描述运动学/几何；SRDF 是 MoveIt 的语义层，
  定义规划组（group）、末端执行器、禁用碰撞对（disable_collisions）、虚拟关节。
- **原始 model_planning.srdf 不完整**：284 条 `<disable_collisions>` 之外**没有任何 group**，
  直接给 MoveIt 无法规划。本 demo 用脚本在原始内容基础上注入 7 个规划组。

## 目录结构

```
a3_moveit_demo/
├── setup_demo_env.bash              # 环境入口（source 它）
├── prefix/                          # 手工 ament 前缀（免 colcon 构建）
│   └── share/
│       ├── ament_index/resource_index/packages/{a3_moveit_config,robot_descriptions}
│       ├── a3_moveit_config -> ../../src/a3_moveit_config
│       └── robot_descriptions -> ../../src/robot_descriptions
└── src/
    ├── robot_descriptions/          #（历史遗留，现已不需要）网格已改用 file:// 绝对路径
    │   └── models/a3_ultra_t2d0/meshes -> ../../../a3_ultra_isaaclab/meshes
    └── a3_moveit_config/
        ├── config/
        │   ├── a3_ultra_t2d0.urdf        # 由 model_planning.urdf 复制（脚本生成）
        │   ├── a3_ultra_t2d0.srdf        # 原始 SRDF + 规划组（脚本生成）
        │   ├── joint_limits.yaml         # 从 URDF limit 提取（脚本生成）
        │   ├── kinematics.yaml           # 手臂/腿 KDL IK 配置
        │   ├── moveit_controllers.yaml   # MoveIt 轨迹执行控制器映射
        │   └── ros2_controllers.yaml     # ros2_control 控制器（mock 硬件）
        ├── urdf/a3_ultra_t2d0_ros2control.urdf  # 追加 <ros2_control> 标签（脚本生成）
        ├── launch/demo.launch.py         # 一键启动（rsp + ros2_control + move_group + demo + rviz）
        ├── scripts/gen_derived_files.py  # 派生文件生成器（可重复执行）
        ├── scripts/gen_params_file.py    # 由 MoveItConfigsBuilder 生成 move_group 参数文件
        ├── scripts/plan_demo.py          # move_group 动作客户端：逐组发送关节目标并执行
        ├── scripts/arm_teleop.py         # Pico VR 手柄双臂相对运动遥操作（触发式）
        └── rviz/moveit.rviz
```

规划组：`left_arm` / `right_arm`（7 关节链）、`left_leg` / `right_leg`（6 关节链）、
`waist`（3）、`head`（2）、`whole_body`（31 关节）。

## 模型来源（重要）

URDF/STL/SRDF 是**智元机器人的版权资产，本仓库不包含**（`.gitignore` 已排除
全部派生模型文件）。首次使用需从**本机智元官方安装**（AimSim 工作区或运动控制
发布包的 `share/robot_model`）生成派生文件：

```bash
export A3_MODEL_SRC=<含 urdf/ 与 meshes/ 的目录>   # 如 <AimSim>/a3_ultra_isaaclab
bash scripts/setup_robot_model.sh
```

该脚本调用 `gen_derived_files.py`（支持 `--model-src` 参数或 `A3_MODEL_SRC`
环境变量）与 `gen_params_file.py`，生成上文标注"脚本生成"的全部文件，
可重复执行（输出稳定）。

## 运行

```bash
cd a3_moveit_demo
source setup_demo_env.bash
ros2 launch a3_moveit_config demo.launch.py            # 含 RViz
ros2 launch a3_moveit_config demo.launch.py rviz:=false  # 无界面服务器
```

演示流程：`left_arm` → `right_arm` → `left_leg` → `waist` → `whole_body`，
每组：向 move_group 发送关节目标 → OMPL 规划 → FollowJointTrajectory 控制器执行到仿真硬件。
RViz 的 MotionPlanning 面板可看到规划场景与机器人模型。

## 扩展：Pico VR 手柄双臂遥操作（arm_teleop.py）

launch 启动后会同时拉起遥操作节点（需 pico 桥接在跑，话题 `/pico_*_controller/*`）：

- 首次按 `button_x`（左手）/ `button_a`（右手）：**校准**——记录手柄起点位姿与机器人当前末端位姿的绑定；
- 之后再按：把"手柄相对起点的 6DOF 增量"复合到机器人末端，作为 pose goal 发给 move_group（KDL IK + OMPL 规划 + 执行）。

增量公式 `D = T_hmd_c0⁻¹·T_hmd_c(t)`，`T_target = T_ee0·D`——相对运动映射，
不依赖头显世界位姿；代价是头部运动会耦合进增量（面向机器人操作即可，见 REPRODUCE.md）。

## 注意

- **Python 版本**：本机 conda 的 python3.14 在 PATH 最前，但 ROS Jazzy / moveit_py
  是 python3.12 的。`setup_demo_env.bash` 已把 /usr/bin 前置；launch 中 demo 节点
  也显式使用 `/usr/bin/python3.12`。
- 如需重新派生文件（URDF/SRDF 更新后）：`bash scripts/setup_robot_model.sh`，
  或单独运行 `gen_derived_files.py`（`--model-src` / `A3_MODEL_SRC` 指定模型来源）。
