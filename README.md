# a3-vr-teleop

用 Pico VR 手柄流式遥操作智元（AgiBot）A3 Ultra 双臂：50Hz 连续跟随 + MoveIt IK 解算，
支持**仿真**（RViz + ros2_control）与**真机**（官方 MC 接口，RViz 规划/手柄遥操作/二者联动）三种形态。

> **免责声明**：本项目为个人/社区开源项目，**与智元机器人（AgiBot）无任何隶属或授权关系**。
> "AgiBot"、"智元"、"A3 Ultra" 等名称及机器人模型文件（URDF/STL/SRDF）的知识产权归智元所有，
> 本仓库**不含**任何模型文件，需使用者从智元官方渠道（AimDK）自行获取。

## 架构与数据流

```
Pico 手柄 --UDP--> a3_teleop_bridge --/pico_*_controller/*-->
a3_streaming_teleop（捕获增量 -> 轴映射 -> MoveIt IK -> 关节步进限幅）
  --/<side>_arm_controller/joint_trajectory-->
  ┌─ 仿真：ros2_control JTC ──► 仿真模型 ──► RViz
  └─ 真机：a3_mc_controller（100Hz 限位/限速/低通 + action/流式双入口仲裁）
       --/motion/control/arm_joint_command--> 机器人 MC
机器人状态 --/motion/control/arm_joint_state--> mc_state_to_joint_states
  --/joint_states--> robot_state_publisher --/tf--> RViz 回显
```

RViz 拖小球 Plan&Execute / move_action 脚本经 move_group 规划后走同一适配器（真机模式），
与手柄流式控制通过仲裁互斥切换（手柄静默 0.3s 后规划自动恢复）。

## 目录结构

```
a3_moveit_demo/    主机端：MoveIt 配置包（OMPL 规划、控制器配置、RViz）
                   + scripts/setup_robot_model.sh（模型派生文件一次性生成）
                   + REPRODUCE.md（从零复现本工程的手册）
a3_teleop_vr/      主机端：流式遥操作节点 + Pico UDP bridge + 启动 launch
a3_robot_teleop/   机器人端：域桥（跨 ROS 域转发 MC 话题）/ 状态回显 / 指令监视 / 急停保持
                   部署 = 把整个文件夹拷到机器人（见该目录 README.md）
```

## 快速开始

**0. 前提**

- Ubuntu + ROS 2 Jazzy（本工程在 Jazzy/python3.12 验证）
- 本机装有智元官方模型（含 `urdf/` 与 `meshes/` 的目录，如 AimDK 安装里的
  `a3_ultra_isaaclab`）——版权原因不入库

**1. 一次性生成模型派生文件**（URDF/SRDF/joint_limits/参数包）

```bash
export A3_MODEL_SRC=<本机智元官方模型目录>
bash a3_moveit_demo/scripts/setup_robot_model.sh
```

**2. 仿真模式**（无真机，RViz 里看仿真手臂跟随手柄）

```bash
source a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=76        # 与本机其他 ROS 栈隔离
ros2 launch "$PWD"/a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

头显端需有向本机 UDP 5100 端口发送手柄报文的桥（可用 OpenArmX 的
[openarmx_teleop_vr_apk](https://github.com/openarmx/openarmx_teleop_vr_apk)，
或任何发布 `/pico_<side>_controller/{pose,grip,rate,...}` 话题的兼容实现）。

**3. 真机模式**（前提：MC 处于 MOTION 状态、motion_player 已关闭，网络/ROS 域互通见
`a3_robot_teleop/README.md`）

```bash
# RViz 规划执行（拖小球直驱真机）
ros2 launch "$PWD"/a3_teleop_vr/launch/real_teleop.launch.py
# 联动 = 规划 + 手柄遥操作（仲裁自动切换）
ros2 launch "$PWD"/a3_teleop_vr/launch/real_teleop.launch.py teleop:=true bridge:=true
# 纯手柄遥操作（不经 MoveIt 规划栈）
ros2 launch "$PWD"/a3_teleop_vr/launch/teleop_vr.launch.py real:=true bridge:=true
```

## 已知问题

- 手柄轴映射当前沿用 OpenArm 默认的恒等矩阵，未按 A3 标定：某些捕获构型下
  可能出现**左右镜像**（我往左它往右）。修复方案（标定常量 / 捕获时刻相对标定）
  已在计划中，欢迎提 issue 讨论。

## 许可证与致谢

- 本项目代码以 [Apache License 2.0](LICENSE) 发布。
- 遥操作交互范式与 UDP 报文格式参考自 OpenArmX
  （[openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr)，
  版权 © 成都长数机器人有限公司，许可 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)）。
  本仓库的桥接节点为按报文格式编写的兼容实现，未复用其代码。
- 机器人模型文件（URDF/STL/SRDF）版权归智元机器人所有，本仓库不包含、
  亦未取得分发授权，请从官方渠道获取。
