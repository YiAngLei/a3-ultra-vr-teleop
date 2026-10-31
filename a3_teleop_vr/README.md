# a3_teleop_vr — A3 双臂流式 VR 遥操作

复刻 `openarmx_teleop_vr` 控制范式的 A3 版：握住 grip **持续跟随**、松开即停。
与 `a3_moveit_demo`（触发式：按一次键 → OMPL 规划 → 执行 → 停住）并存，互不干扰；
如果本方案有问题，继续用 `a3_moveit_demo` 即可（其默认行为未做任何改变）。

## 数据流

```
Pico APK --UDP--> a3_teleop_bridge --/pico_*--> a3_streaming_teleop.py
   (OpenArmX)    (本工程, 自研)                    |  MoveItPy IK (KDL, 当前状态作种子)
                                                   v
                                         /<side>_arm_controller/joint_trajectory
                                                   |
                                         ros2_control (mock 硬件, 100Hz)
                                                   v
                                         /joint_states + /tf + RViz
```

报文接收端 `a3_teleop_bridge` 是本工程自研的兼容实现（见 `a3_teleop_bridge/README.md`），
不再依赖 OpenArmX 的 bridge 节点；Pico 端的 App 仍来自 OpenArmX（见「开源与合规」）。

`teleop_vr.launch.py` 直接 include `a3_moveit_demo` 的 `demo.launch.py`
（传 `demo_plan:=false teleop:=none`，见那边新加的两个 launch 参数），
再拉起流式节点；a3_moveit_demo 本身只需 source 即可复用。
两个目录需放在同一上级目录下（或用环境变量 `A3_MOVEIT_DEMO_DIR` 指定）。

## 快速开始

```bash
# 首次：从本机智元官方安装生成模型派生文件（见 a3_moveit_demo/scripts/setup_robot_model.sh）
export A3_MODEL_SRC=<含 urdf/ 与 meshes/ 的目录>   # 如 AimSim 工作区的 a3_ultra_isaaclab
bash ../a3_moveit_demo/scripts/setup_robot_model.sh

# Pico 头显：安装并运行 OpenArmX 的 VR App，目标 IP 指向本机

# 启动仿真栈 + 桥接 + 流式遥操作（一条命令）
cd a3_teleop_vr
source setup_teleop_env.bash
ros2 launch "$PWD"/launch/teleop_vr.launch.py bridge:=true
```

若你更想继续用 OpenArmX 原版 bridge（`ros2 run openarmx_teleop_bridge_vr ...`），
把 `bridge:=true` 去掉即可，两者发布完全相同的话题。

## 操作

| 输入 | 作用 |
|---|---|
| 握 grip（>0.5） | 该臂进入跟随：以按下瞬间手柄位置为参考点、末端当前位置为基准点 |
| 握住时移动手柄 | 该臂末端连续跟随（增量映射，见下） |
| 松开 grip | 该臂立即停止并保持当前位姿 |
| rate（0.1/1.0） | 速度档位：0.1=慢速档（每周期每关节 ≤4°），1.0=快速档（8~20°） |
| 右手 B | 双臂回零（move_action 规划，先左后右，规划期间流式暂停） |
| 左手 Y | 双臂举手（肩 pitch -1.2 + 肘 1.2，FK 实测的安全构型，规划执行） |
| 脚本动作中握 grip | 取消脚本动作，回到手动（与 OpenArm 相同语义） |
| 扳机 | 未接线（本模型 31 关节无夹爪） |

映射公式（与 OpenArm 相对模式一致）：

```
按下瞬间：  T_hand0 = 手柄位姿(pico_hmd 系)   T_ee0 = FK(当前关节) (pelvis 系)
每周期：    D = T_hand0^-1 · T_hand(t)                # 手柄体坐标系增量
           p' = axis_matrix · p_D · position_scale    # 默认恒等/[1,1,0.9]
           R' = ori_matrix · R_D · ori_matrix^T
           T_target = T_ee0 · [R', p']                # 右复合，pelvis 系
           位置限幅：|T_target - T_ee0| ≤ workspace_radius(0.5m)
IK：        RobotState(当前 /joint_states 播种) → set_from_ik(group, T_target, ee, 10ms)
限幅：      |q_target - q_now| > 阈值 才按慢/快档 max_step 截断（OpenArm 迟滞设计）
输出：      单点 JointTrajectory → <side>_arm_controller/joint_trajectory @ control_rate
```

## 调参（-p 覆盖，均有默认值）

| 参数 | 默认 | 说明 |
|---|---|---|
| control_rate | 50.0 | 控制环频率 Hz（mc_arm 模式用 100） |
| grip_threshold | 0.5 | 握把使能阈值 |
| stream_timeout | 0.3 | pose 流超时（秒）后自动释放 |
| ik_timeout | 0.01 | 单次 IK 超时（秒） |
| workspace_radius | 0.5 | 工作空间球半径（米） |
| position_scale_xyz | [1.0, 1.0, 0.9] | 位置增量缩放 |
| slow_step_deg | 全 4.0 | 慢速档每关节每周期最大步进 |
| fast_step_deg | [20,16,12,12,14,8,12] | 快速档每关节每周期最大步进 |
| output_backend | sim_jtc | sim_jtc=仿真 JTC；**mc_arm=真机 MC 官方接口** |
| mc_cmd_topic | /motion/control/arm_joint_command | 真机指令话题 |
| mc_state_topic | /motion/control/arm_joint_state | 真机状态话题 |
| mc_max_speed | 4.0 | 相邻指令速度限幅 rad/s（官方 ≤4） |
| mc_lowpass_alpha | 0.5 | 指令低通系数（官方建议） |

例：`--ros-args -p workspace_radius:=0.3 -p control_rate:=30.0`

## 无头显自测

```bash
# 桥接节点自测（本机 UDP 环回，20 项断言，无需任何外设）：
/usr/bin/python3.12 a3_teleop_bridge/test_bridge_udp.py

# 栈 + teleop 节点跑起来后（fake_pico_publisher 合成 /pico_* 话题）：
/usr/bin/python3.12 scripts/fake_pico_publisher.py

# mc_arm（真机 MC 接口）模式本机自测（fake MC + 流式节点，10 项断言）：
/usr/bin/python3.12 scripts/test_mc_mode_local.py

# FJT->MC 执行适配器本机自测（fake MC + 双臂并发/取消/抢占/联动仲裁/回显，24 项断言）：
/usr/bin/python3.12 scripts/test_mc_controller_local.py
```

fake_pico 自动执行：空闲 → 左臂画圆跟随 → 松开保持 → B 回零 → Y 举手 → grip 取消脚本，
并打印各阶段左臂关节快照与 PASS/FAIL 断言。

## 真机模式（mc_arm，对接智元 MC 官方接口）

真机有两种基础用法，另有二者叠加的联动模式：

**① Pico 流式遥操作**（手柄连续跟随，同仿真手感）：

```bash
ros2 launch "$PWD"/launch/teleop_vr.launch.py real:=true bridge:=true
```

**② RViz 规划执行**（拖小球 Plan&Execute / move_action 脚本直驱真机）：

```bash
ros2 launch "$PWD"/launch/real_teleop.launch.py
```

**③ 联动 = ② + 流式遥操作**（一条命令同起规划栈与遥操作，可随时切换）：

```bash
ros2 launch "$PWD"/launch/real_teleop.launch.py teleop:=true bridge:=true
```

模式 ③ 的数据流：Pico 报文经 a3_teleop_bridge 进 a3_streaming_teleop（默认
backend），发单点 `JointTrajectory` 到 `/<side>_arm_controller/joint_trajectory`；
a3_mc_controller 对该话题与 FollowJointTrajectory action **双入口**受理，共用
同一条 100Hz 限位/限速/低通安全链。内置仲裁：流式话题到达即接管该侧并
**抢占**正在执行的 action 目标（RViz 显示"执行失败"属预期，机器人保持被接管
时刻的安全位姿）；手柄静默超过 `stream_timeout`（默认 0.3s）后控制权自动归还
action。此模式下 B/Y 键动作（经 move_group 规划）也可用。

模式 ② 的数据流（模式 ① 无 move_group，teleop 节点直接发 MC）：

```
RViz 小球 / move_action
   -> move_group (OMPL 规划, 控制器白名单=双臂)
   -> /left|right_arm_controller/follow_joint_trajectory (a3_mc_controller 提供)
   -> 100Hz 限位/限速/低通安全链 -> /motion/control/arm_joint_command
真机 MC --/motion/control/arm_joint_state--> mc_state_to_joint_states
   --31 关节 /joint_states--> robot_state_publisher --/tf--> RViz 显示真机姿态
```

两个真机模式的共同约束（官方硬性要求，均已内置）：

- 100Hz 连续流（未执行时也发保持帧，相邻间隔 ≤30ms；发布用专用线程，不经 executor）
- 相邻指令速度限幅 4 rad/s、低通平滑（alpha 0.5）、官方关节限位钳制
- 状态反馈从 `/motion/control/arm_joint_state` 读取
- 腿/腰/头不经 MoveIt（由 MC 自身站立/行走栈控制），回显里固定补 0

启动前用预检脚本确认链路（关节状态在流动、MC HTTP 可达）：

```bash
/usr/bin/python3.12 scripts/preflight_mc.py --mc-host <机器人IP>
```

**网络/域要求**：主机侧 shell 需 `unset ROS_LOCALHOST_ONLY; export ROS_DOMAIN_ID=<与机器人约定的互通域>; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`。
机器人 MC 所在域与互通域不一致时（如 MC 在 232、互通在 42），需在机器人端
先起域桥转发两个话题：`bash start_domain_bridge.sh all`
（见 `a3_robot_teleop/README.md`，state 只读随时可开，command 方向=允许主机驱动机器人）。

机器人端配套工具见同级的 `a3_robot_teleop/`（角度回显 / 指令流监视 / 急停保持）。
前提：MC 处于 MOTION 状态、motion_player 已关闭（官方要求）。

## 已知限制

- **头部耦合**：手柄位姿在 pico_hmd 系，操作期间转头会等效移动手柄
  （OpenArm 相对模式同样如此）。操作时面向机器人、头部少动。
- **无碰撞检测**：沿用了 OpenArm 的思路（限位 + 步进限幅 + 工作空间球），
  IK 本身不查碰撞；需要碰撞安全就走 moveit_servo 方案或触发式规划。
- **脚本动作是关节空间目标**（全零 / 举手构型），靠 OMPL 规划保证无碰撞；
  若规划失败看 move_group 日志。
- KDL 对 7 自由度冗余臂是局部收敛：小步进 + 当前状态作种子在 50Hz 下表现良好，
  快速档猛拉时如出现 IK 失败日志属正常保护（该帧不发送）。

## 开源与合规

- **许可证**：Apache License 2.0（见 LICENSE）。a3_teleop_vr 与 a3_moveit_demo
  两个目录为同一工程，整体适用。
- **致谢**：遥操作交互范式与 UDP 报文格式参考自 OpenArmX
  （[openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr)，
  版权 © 成都长数机器人有限公司，许可 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)）。
  本工程的桥接节点为按报文格式编写的兼容实现，未复用其代码。
- **需用户自备、不在本仓库中的内容**：
  - 智元 A3 机器人模型（URDF/STL/SRDF）——智元机器人版权资产，
    由 `a3_moveit_demo/scripts/setup_robot_model.sh` 从本机官方安装生成；
  - Pico 端 VR App——见 [openarmx_teleop_vr_apk](https://github.com/openarmx/openarmx_teleop_vr_apk)。
- **免责声明**：本项目为社区非官方项目，与智元机器人、OpenArmX 均无隶属或背书关系；
  A3/A3 Ultra 及相关商标归智元机器人所有，OpenArmX 及相关商标归成都长数机器人有限公司所有。
  代码按"原样"提供，用于真实机器人前请自行评估安全风险。

## 踩坑记录（调试时血泪史）

1. **MoveItPy 与 move_group 的参数 schema 不同**：MoveItCpp 读
   `planning_pipelines.pipeline_names`（嵌套 dict），move_group 读平铺的
   `planning_pipelines: [ompl]`，且 moveit_py 的 `launch_params_filepaths` 在本机
   对 `/**` 通配文件不生效。解法：`config_dict=yaml["/**"]["ros__parameters"]`
   （内部写成以节点名为键的临时 yaml），并把 `planning_pipelines` 改写为嵌套形式。
2. **rclpy 定时器回调里的异常会把进程整个打出**（穿过 `rclpy.spin`），
   若 finally 里 `os._exit(0)` 则连 traceback 都不剩——日志里只看到"进程干净退出"。
   解法：定时器回调每 tick 包 try/except；`main()` 兜底打印 traceback。
3. **`spin_until_future_complete` 喂不活 action client 的 future**
   （goal 请求都到 move_group 并执行完了，future 还没完成）。
   必须用 `rclpy.spin(node)` + `add_done_callback` 的写法。
4. **moveit_py 的 `active_joint_model_bounds` 元素是 VariableBounds 对象**
   （属性 `min_position`/`max_position`），不是 (low, high) 元组。
5. **"举手"构型要 FK 实测**：全零 + 肘 1.745rad 会让手撞髋
   （OMPL 报 "Unable to sample any valid states for goal tree"）；
   肩 pitch 负向才是抬臂（pitch=-1.4 时腕部 z 最高）。
6. **`os._exit` 前必须 `sys.stdout.flush()`**，否则 print 全丢（管道缓冲）。
7. **rclpy ActionServer 的 execute 回调跑在 executor 的任务线程上**
   （`create_task` → `Task.__call__` 内联执行，不是独立后台线程）：
   阻塞式 execute（轨迹执行常见）必须用 `MultiThreadedExecutor`，
   否则它独占 executor——100Hz 定时器、取消请求、另一侧 goal 全部排队，
   表现为"取消不生效、goal 串行执行、指令流掉到 15Hz"。
8. **executor 多线程唤醒有几十毫秒级抖动**（多节点挤一个进程时 p99 可到 34ms，
   官方要求相邻指令 ≤30ms）：对周期抖动敏感的发布（MC 指令流）用专用
   `threading.Thread + Event.wait` 发布，不经 executor 定时器——
   a3_mc_controller 的 100Hz 流即此实现（本机压测全程 100.0Hz）。

## 文件

```
a3_teleop_vr/
├── a3_teleop_bridge/                # Pico UDP 桥接（自研兼容实现）
│   ├── a3_teleop_bridge_node.py     #   桥接节点
│   ├── test_bridge_udp.py           #   本机 UDP 环回自测（20 项断言）
│   └── README.md
├── scripts/a3_streaming_teleop.py   # 流式遥操作主节点（MoveItPy IK；sim/mc_arm 双输出后端）
├── scripts/mc_arm_interface.py      # MC 官方接口共享常量（14 关节顺序 + 官方限位表）
├── scripts/a3_mc_controller.py      # FJT action server + joint_trajectory 流式入口 -> MC 100Hz（含联动仲裁）
├── scripts/mc_state_to_joint_states.py  # MC 14 关节状态 -> 全身 31 关节 /joint_states 回显
├── scripts/preflight_mc.py          # 真机预检（关节状态流 + MC HTTP 状态）
├── scripts/test_mc_mode_local.py    # mc_arm 流式模式本机自测（fake MC，10 项断言）
├── scripts/test_mc_controller_local.py  # 执行适配器本机自测（fake MC，24 项断言）
├── scripts/fake_pico_publisher.py   # 无头显端到端自测
├── launch/teleop_vr.launch.py       # 仿真栈/真机流式遥操作（real:=true 为真机模式 ①）
├── launch/real_teleop.launch.py     # 真机 RViz 规划执行（模式 ②）+ teleop/bridge:=true 联动（模式 ③）
├── config/real_move_group_override.yaml # 真机 move_group 控制器白名单（仅双臂）
├── setup_teleop_env.bash            # source a3_moveit_demo 环境
└── LICENSE                            # Apache-2.0

a3_robot_teleop/                     #（同级目录）机器人端工具：角度回显/指令监视/急停保持
```
