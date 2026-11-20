# a3_teleop_vr — A3 双臂流式 VR 遥操作

数据链与遥操作范式按 **XRoboToolkit 官方**（双 UR5 成熟案例）对齐：握住 grip **持续跟随**、
松开即停；位姿为**房间锚定**数据，映射为**世界系增量**（`world_rotation` 默认即官方
R_HEADSET_TO_WORLD）。与 `a3_moveit_demo`（触发式：按一次键 → OMPL 规划 → 执行 →
停住）并存，互不干扰；如果本方案有问题，继续用 `a3_moveit_demo` 即可。

## 数据流

```
PICO 眼镜（XRoboToolkit 官方 App, Head+Controller, Send ON）
   --WiFi--> PC Service（/opt/apps/roboticsservice）
   --SDK--> a3_xrt_bridge --/pico_*--> a3_streaming_teleop.py
   （房间锚定位姿，官方同源）            |  世界系增量映射 + MoveItPy IK (KDL)
                                        v
                              /<side>_arm_controller/joint_trajectory
                                        |
                              ros2_control (mock 硬件, 100Hz)
                                        v
                              /joint_states + /tf + RViz
```

桥接节点二选一（`bridge_impl:=xrt`（默认）/ `openarmx`）：

- **a3_xrt_bridge（默认，推荐）**：收 XRoboToolkit PC Service 官方 SDK。
  前提：先启动 PC Service（`bash /opt/apps/roboticsservice/runService.sh`），头显装
  XRoboToolkit 官方 App 并勾选 Head+Controller、Send ON。位姿**房间锚定**（转头不改
  手柄读数），与世界系增量映射配套。rate 调速档默认用**扳机当快进键**
  （`-p rate_from_trigger:=false` 固定慢档）。
- **a3_teleop_bridge（openarmx，对照/回退）**：兼容 OpenArmX APK 的 UDP 报文。
  其位姿为 pico_hmd **头显系**（转头会耦合进增量），与世界系增量映射**不配套**，
  仅供对照；`pose_source:=absolute` 也只在它有意义。

`teleop_vr.launch.py` 直接 include `a3_moveit_demo` 的 `demo.launch.py`
（传 `demo_plan:=false teleop:=none`，见那边新加的两个 launch 参数），
再拉起流式节点；a3_moveit_demo 本身只需 source 即可复用。
两个目录需放在同一上级目录下（或用环境变量 `A3_MOVEIT_DEMO_DIR` 指定）。

## 快速开始

```bash
# 首次：从本机智元官方安装生成模型派生文件（见 a3_moveit_demo/scripts/setup_robot_model.sh）
export A3_MODEL_SRC=<含 urdf/ 与 meshes/ 的目录>   # 如 AimSim 工作区的 a3_ultra_isaaclab
bash ../a3_moveit_demo/scripts/setup_robot_model.sh

# 1) 启动 PC Service（XRoboToolkit，常驻）
bash /opt/apps/roboticsservice/runService.sh

# 2) 头显：XRoboToolkit 官方 App，PC Service IP 指向本机，勾 Head+Controller，Send ON

# 3) 启动仿真栈 + 桥接 + 流式遥操作（一条命令）
cd a3_teleop_vr
source setup_teleop_env.bash
ros2 launch "$PWD"/launch/teleop_vr.launch.py bridge:=true   # bridge_impl 默认 xrt
```

无头显回归测试：`scripts/fake_pico_publisher.py` 直接发布 /pico_* 话题（与桥输出
同名），bridge:=false 启动栈后运行即可，无需任何 VR 设备。

## 命令速查（2026-11-20 全部实测验证）

### 实机每日流程（模式③：RViz + 流式遥操作 + 可选摇杆走路）

**启动（按顺序）：**

```bash
# 0) 前提：机器人已开机、MC 处于 MOTION 状态、眼镜 XRoboToolkit App 已连
#    （PC Service IP 填 192.168.43.5，Head+Controller 勾上，Send ON）

# 1) 机器人端（ssh agi@192.168.43.10）
~/.local/bin/aimcli em stop motion_player        # 若在运行（官方要求停止才收指令）
cd /agibot/data/home/agi/Desktop/teleop_vr_ws/a3_robot_teleop
bash start_domain_bridge.sh all                  # 状态桥+指令桥+走路链（loco）

# 2) PC 端
bash /opt/apps/roboticsservice/runService.sh     # PC Service（脚本自行后台化）
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=42
ros2 launch ~/AimSim/a3_teleop_vr/launch/real_teleop.launch.py teleop:=true bridge:=true

# 3) 摇杆走路（可选，另开终端，先 source 同上的 env 且同域）：
/usr/bin/python3.12 ~/AimSim/a3_teleop_vr/scripts/locomotion_teleop.py --ros-args -p enable:=true
#    ⚠ enable=true 后轻推摇杆机器人即走（max_scale 0.3）；不走路就不开这步

# 4) 机器人端监视（可选）：
cd /agibot/data/home/agi/Desktop/teleop_vr_ws/a3_robot_teleop
nohup bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=232; python3 command_monitor.py" >/tmp/command_monitor.log 2>&1 &
```

**关闭（按顺序）：**

```bash
# PC 端
pkill -INT -f 'real_teleop[.]launch[.]py'        # 主栈（RViz/遥操作/MC 适配器）
pkill -INT -f 'locomotion_teleop[.]py'           # 摇杆走路（若开着）
kill -TERM $(pgrep -f RoboticsServiceProcess)    # PC Service

# 机器人端
cd /agibot/data/home/agi/Desktop/teleop_vr_ws/a3_robot_teleop
bash stop_domain_bridge.sh                       # 全部管道 + 走路转发
pkill -f 'command_monitor[.]py'                  # 监视器
# 如需恢复官方动作播放：~/.local/bin/aimcli em start motion_player
```

### 一次性准备（只做一次）

```bash
# 模型派生文件（需智元官方模型目录，如 AimSim 工作区自带）
export A3_MODEL_SRC=$HOME/AimSim/a3_ultra_isaaclab
bash ~/AimSim/a3_moveit_demo/scripts/setup_robot_model.sh

# xrt 桥依赖：xrobotoolkit-sdk 装进系统 python3.12（cp310-abi3，已装可跳过）
/usr/bin/python3.12 -m pip install --user --break-system-packages \
    --proxy http://127.0.0.1:7897 xrobotoolkit-sdk
```

### 每次使用（按顺序）

```bash
# 1) PC Service（常驻，必须先起；脚本会自行后台化）
bash /opt/apps/roboticsservice/runService.sh

# 2) 头显：XRoboToolkit App → PC Service IP 填本机 IP（hostname -I 的第一个）
#    → 勾 Head + Controller → Send 拨 ON

# 3) 仿真栈 + 桥接 + RViz（一条命令）
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=76               # 所有相关终端必须同域
ros2 launch "$HOME"/AimSim/a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

### 启动变体

```bash
ros2 launch .../teleop_vr.launch.py bridge:=true pos_scale:=0.5      # 降位置灵敏度
ros2 launch .../teleop_vr.launch.py bridge:=true rviz:=false         # 无 RViz
ros2 launch .../teleop_vr.launch.py real:=true bridge:=true          # 真机（前提见「真机模式」）
ros2 launch .../teleop_vr.launch.py bridge:=true bridge_impl:=openarmx  # 旧 OpenArmX 桥（对照/回退）
```

### 调试命令（终端都要先 source setup_teleop_env.bash 且同 ROS_DOMAIN_ID）

```bash
ros2 topic hz /pico_left_controller/pose                  # 数据流频率（应 ~100Hz）
ros2 topic echo /pico_right_controller/grip               # 握把值（>0.5 激活该臂）
ros2 run tf2_ros tf2_echo pelvis_link teleop/left_target  # 目标坐标架实时位姿
ros2 run tf2_ros tf2_echo xrt xrt_left_controller         # 手柄原始坐标架（报文系）
/usr/bin/python3.12 scripts/pico_motion_viewer.py         # 报文系位移三列读数（标定 R 用）
/usr/bin/python3.12 scripts/debug_mapping_monitor.py      # 指令 vs 实测偏差（静默=健康）
ros2 topic echo /teleop_debug/left/pos_align_deg          # 平移偏差角数值
```

### 无头显回归

```bash
ros2 launch .../teleop_vr.launch.py bridge:=false     # 栈起来后另开终端：
/usr/bin/python3.12 scripts/fake_pico_publisher.py    # 40s 自动剧本，结尾打印 PASS×2
```

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

映射公式（按 XRoboToolkit 双 UR5 官方案例，世界系增量；旧 OpenArm 相对模式见 git 历史）：

```
固定转换：   R = world_rotation（pico_hmd 系 -> pelvis 系，默认官方 R_HEADSET_TO_WORLD）
             T_w(t) = [R·R_raw·Rᵀ, R·p_raw]        # 姿态共轭，等价官方 q'=Rq⊗q⊗Rq⁻¹
按下瞬间：   T_w0 = T_w(按下帧)   T_ee0 = FK(当前关节) (pelvis 系)
每周期：    dp_raw = p_w(t) − p_w0                  # 世界系位置增量
           dp = dp_raw · position_scale             # [1,1,1]，逐轴
           DR  = R_w(t) · R_w0ᵀ                    # 世界系旋转增量（左复合）
           T_target = [DR·R_ee0, p_ee0 + dp]       # pelvis 系
           位置限幅：|dp| > workspace_radius(0.5m) 时等比缩到球面
IK：        RobotState(当前 /joint_states 播种) → set_from_ik(group, T_target, ee, 10ms)
限幅：      |q_target - q_now| > 阈值 才按慢/快档 max_step 截断（OpenArm 迟滞设计）
输出：      单点 JointTrajectory → <side>_arm_controller/joint_trajectory @ control_rate
```

世界系增量的好处：不依赖捕获瞬间的手柄朝向（旧版 D 在手柄体系，转头/换手会耦合），
也没有"某些构型左右镜像"的问题。`axis/orientation_matrix` 与 `auto_calib` 已移除，
唯一标定量是固定转换 R（见「映射观测与标定」）。

若 Pico 端 App 会发 `LEFT_ABS/RIGHT_ABS` 报文（bridge 日志出现"首次收到 absolute_hand"
即在发），用 `-p pose_source:=absolute` 切换到位姿锚定世界原点的绝对流，可彻底消除
"转头带动参考系"的耦合（XRoboToolkit 官方数据即世界锚定）。

## 调参（-p 覆盖，均有默认值）

| 参数 | 默认 | 说明 |
|---|---|---|
| control_rate | 50.0 | 控制环频率 Hz（mc_arm 模式用 100） |
| grip_threshold | 0.5 | 握把使能阈值 |
| stream_timeout | 0.3 | pose 流超时（秒）后自动释放 |
| ik_timeout | 0.01 | 单次 IK 超时（秒） |
| workspace_radius | 0.5 | 工作空间球半径（米） |
| position_scale_xyz | [1.0, 1.0, 1.0] | 位置增量缩放（逐轴）；launch 快捷参数 `pos_scale`（标量，如 `pos_scale:=0.5`） |
| slow_step_deg | 全 4.0 | 慢速档每关节每周期最大步进 |
| fast_step_deg | [20,16,12,12,14,8,12] | 快速档每关节每周期最大步进 |
| output_backend | sim_jtc | sim_jtc=仿真 JTC；**mc_arm=真机 MC 官方接口** |
| mc_cmd_topic | /motion/control/arm_joint_command | 真机指令话题 |
| mc_state_topic | /motion/control/arm_joint_state | 真机状态话题 |
| mc_max_speed | 4.0 | 相邻指令速度限幅 rad/s（官方 ≤4） |
| mc_lowpass_alpha | 0.5 | 指令低通系数（官方建议） |
| debug_pub | true | 发布 /teleop_debug/* 映射观测话题（标定用，见下节） |
| world_rotation | [0,0,-1,-1,0,0,0,1,0] | pico_hmd 系→世界系固定转换（行主序；官方 R_HEADSET_TO_WORLD） |
| pose_source | relative | relative=/pico_*_controller/pose；absolute=/vr/*_pose_absolute（需 APK 发 ABS 报文） |

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

模式 ③ 的数据流：Pico 报文经 a3_xrt_bridge（bridge_impl:=xrt 默认）进 a3_streaming_teleop（默认
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

## 摇杆行走（locomotion_teleop，2026-11-20 新增）

手柄摇杆 -> `/motion/control/locomotion_velocity`（MC 行走接口，仅 MOTION 状态生效）。
与手臂流式遥操作**互不锁定，可同时使用**（全身遥操作）。

```
左摇杆：前推/后拉=前进/后退，左右拨=横移（+为左移）
右摇杆：左右拨=原地旋转（+为左转）
```

数据链：

```
/pico_<side>_controller/axis（桥发布，Float32MultiArray [x,y]）
  -> locomotion_teleop.py（PC，本脚本）
       死区 0.15 | max_scale 默认 0.3 | 20Hz | 松开补 3 帧零 | 数据超时 0.3s 按零
  -> /teleop/locomotion_cmd
  -> 域桥 loco 管道（42->232，start_domain_bridge.sh loco）
  -> locomotion_relay.py（机器人端，组官方 JSON，50Hz，静默 0.5s 自动发零）
  -> /motion/control/locomotion_velocity/pb_3A...（RosMsgWrapper，格式同官方 walk.py）
```

启用（**默认禁用，enable 不显式 true 不发任何指令**；enable 后启动有 8s 全零预热
用于跨 WiFi 发现链路，机器人不会动）：

```bash
# 机器人端（首次或重启后）：
ssh agi@192.168.43.10
cd /agibot/data/home/agi/Desktop/teleop_vr_ws/a3_robot_teleop
bash start_domain_bridge.sh loco          # loco 管道 + locomotion_relay

# PC 端（与遥操作同域）：
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=42
/usr/bin/python3.12 ~/AimSim/a3_teleop_vr/scripts/locomotion_teleop.py \
    --ros-args -p enable:=true            # 可加 -p max_scale:=0.5 提速
```

方向若反了改符号参数即可：`-p lat_sign:=1.0` / `-p ang_sign:=1.0` / `-p fwd_sign:=-1.0`。
**让机器人真走之前：确认 MC 处于 MOTION 状态、周围安全，并征得操作者同意。**

## 映射观测与 world_rotation 标定（debug_pub 默认开启）

映射全部在世界系进行，RViz（Fixed Frame=pelvis_link，launch 默认配置已带显示）
里每臂可见：**小坐标架 teleop/<side>_target**（IK 目标，等价 XRoboToolkit 双 UR5
官方案例的 mocap 体）+ 头侧两根旋转箭头 + 两个数值夹角（话题
`/teleop_debug/<side>/*`，frame 均为 model_frame=pelvis_link）：

| 话题 | 类型 | 含义 |
|---|---|---|
| target_pose / TF teleop/<side>_target | PoseStamped/TF | 发给 IK 的目标位姿（在手腕应去的位置） |
| hand_delta_world | PoseStamped | 手柄自捕获以来的世界系旋转增量（基准；固定在 y=+0.55,z=1.1） |
| cmd_delta_world | PoseStamped | 实际施加的世界系旋转增量（正常应与 hand 完全一致；固定在 y=-0.55,z=1.1） |
| rot_align_deg | Float32 | 指令旋转增量 vs 实测末端增量的偏差角（度）；正常 ≈0 |
| pos_align_deg | Float32 | 指令平移 vs 实测末端位移方向的偏差角（度）；正常 ≈0 |

**配一个阈值报数监视器**（平时无输出=正常，超阈值才打印，解决 echo 刷屏没法
对照动作的问题）：

```bash
/usr/bin/python3.12 scripts/debug_mapping_monitor.py          # 双臂，阈值 10°
# 打印形如 "[left] 旋转方向偏差 12.3°"，判读见脚本头注释
```

**world_rotation 标定（唯一需要标定的量）**：

1. 开 RViz（launch 默认带 Pose/TF 显示），握 grip，让 RViz 小坐标架跟随；
2. **平移判定**：手柄沿 RViz 世界系 +x 方向移动——小坐标架应同向移动：
   - 方向正确 → x 行标定完毕；反向 → x 行取反；
   - 走的不是 x 而是别的轴 → 记录"手柄物理 +x 对应世界哪根轴"；
   对 +y、+z 重复，得到三张"物理轴 → 世界轴"映射（含符号）；
3. **旋转判定**：握 grip 后绕某世界轴拧手柄 45°，末端应绕同轴转 45°（不是 2 倍、
   不是反方向）；
4. 把三张映射拼成带符号的轴置换矩阵（3×3 每行每列恰好一个 ±1），以行主序 9 个数
   传给 `-p world_rotation:="[r11,r12,r13,r21,r22,r23,r31,r32,r33]"`。

   快捷测定法：用 `scripts/pico_motion_viewer.py` 的"头显系Δ"列——手柄沿 RViz
   +x 移动时它在报文系里的位移 v，要求 R·v = +x，三次移动定出 R 的三列。
5. 默认值即 XRoboToolkit 官方 UR5 案例的 R_HEADSET_TO_WORLD（报文系≈Unity 惯用
   约定时可直接用）；OpenArmX 报文系若不同才需要重测。

**"指令 vs 实测"夹角（rot/pos_align_deg）不再是矩阵标定判据**——它们现在监视
"目标是否被工作空间限幅卡住 / IK 是否跟不上"：手柄大幅移动时若持续非 0，
先看 target_pose 小坐标架是否顶在 workspace_radius 球面上。

## 已知限制

- **头部耦合**：手柄位姿在 pico_hmd 系，操作期间转头会等效移动手柄。缓解：
  操作时面向机器人、头部少动；**根治**：Pico 端 App 支持的话改发
  `LEFT_ABS/RIGHT_ABS` 报文，并 `-p pose_source:=absolute`（bridge 日志出现
  "首次收到 absolute_hand" 即说明在发；XRoboToolkit 官方链路数据即世界锚定，
  无此问题）。
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
├── scripts/debug_mapping_monitor.py # 映射标定阈值报数监视器（配合 /teleop_debug/*）
├── scripts/pico_motion_viewer.py    # Pico 手柄运动查看器（纯遥操话题，位移/旋转分量）
├── launch/teleop_vr.launch.py       # 仿真栈/真机流式遥操作（real:=true 为真机模式 ①）
├── launch/real_teleop.launch.py     # 真机 RViz 规划执行（模式 ②）+ teleop/bridge:=true 联动（模式 ③）
├── config/real_move_group_override.yaml # 真机 move_group 控制器白名单（仅双臂）
├── setup_teleop_env.bash            # source a3_moveit_demo 环境
└── LICENSE                            # Apache-2.0

a3_robot_teleop/                     #（同级目录）机器人端工具：角度回显/指令监视/急停保持
```
