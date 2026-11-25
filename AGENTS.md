# AGENTS.md — AimSim 工作区说明

> 本文件面向 AI 编码代理，假设读者对本项目一无所知。所有内容均基于对仓库实际文件的勘察，编写语言与项目注释/文档保持一致（中文为主）。

## 1. 项目概览

本目录不是传统意义上的源码仓库，而是一个**机器人运动控制（Motion Control, MC）仿真/部署工作区**，面向智元（AgiBot / AGIMA）人形机器人（A2 / A3 / A3_ULTRA 系列）。工作区包含两类交付物：

1. **三个预编译的运动控制发布包**（`bin/ config/ scripts/ share/` 结构，另有同名 `.tar` 存档）：
   - `A3_ULTRA_motion_control_x86_humble/` — 最新，支持 A3_ULTRA 机型，含 valgrind 启动模式、`fall_protect` / `foundation_policy` / `recover_policy` / `wbc_policy` 等新增模块。
   - `motion_control_v3.0.19_x86_humble/` — v3.0.19，ROS 2 Humble，机器人配置更全（含 `a3_t2d0`、`a3_ultra_t2d5` 的 datasets/policies 目录）。
   - `motion_control_x86_v3.0.5_jazzy/` — v3.0.5，ROS 2 Jazzy。
   - 三个包的 `start_motion_control.sh` 略有差异（robot_map 映射、valgrind 选项等），修改脚本时注意按包分别核对。
2. **一个 Python wheel 包** `aimsim-3.3-py3-none-any.whl`（约 422MB）：AimSim 仿真启动器（"MuJoCo + Isaac 仿真启动器"），内含 `mujoco_simulator` 与 `isaac_simulator` 两个仿真器包、命令行入口 `aimsim`、以及 AimRT 原生库（`core_py.so`、`aimrt_main`、`iox-roudi` 等）。

**注意：本工作区没有任何源码构建系统**（无 pyproject.toml / CMakeLists.txt / package.xml）。所有二进制均为预编译产物，这里的"开发"主要是：修改 YAML 配置、编写/修改 `tools/` 下的 Python 调试脚本、维护 wheel 内的仿真器配置。

## 2. 目录结构

### 2.1 运动控制发布包（以 `A3_ULTRA_motion_control_x86_humble/` 为例）

```
A3_ULTRA_motion_control_x86_humble/
├── bin/                          # 全部预编译二进制与共享库
│   ├── motion_control            # MC 主程序（C++ / AimRT，约 17MB）
│   ├── aimrt_main                # AimRT 通用主程序
│   ├── iox-roudi                 # iceoryx 守护进程（SIL/进程间通信必需）
│   ├── libaimrt_*_plugin.so      # AimRT 插件：net, ros2, iceoryx, grpc,
│   │                             #   mqtt, zenoh, lcm, record_playback,
│   │                             #   opentelemetry, parameter, proxy ...
│   ├── libdance_policy_main_lib_x86.so / librecover_policy_main_lib_x86.so
│   ├── libtc-engine.so / libtcsdk.so   # 任务控制引擎/SDK
│   └── libonnxruntime.so.1.19.2  # ONNX Runtime（含 CUDA/TensorRT 后端，
│                                 #   libonnxruntime_providers_cuda.so 约 236MB）
├── scripts/motion_control/
│   └── start_motion_control.sh   # 唯一启动入口（详见第 4 节）
├── config/
│   ├── deployment/deployment.yaml    # AimRT 部署描述（声明 mc/pnc/motion_player/
│   │                                 #   ota_master 等 app 及端口，供部署工具使用）
│   └── motion_control/
│       ├── configuration/robot/<robot_name>/   # 按机型组织的模块配置（见 2.2）
│       └── tools/                  # 测试/调试脚本（见第 5 节）
└── share/robot_model/models/<model>/
    ├── urdf/                       # 机器人 URDF
    └── meshes/                     # STL 网格
```

机器人机型（config 目录名）：`a3_t2d5`、`a3_ultra_t2d0`、`a3_ultra_t3d0` 等；
`share/robot_model/models/` 下还额外有 `a3_p1d0`、`a3_t2d0`、`a3_t3d0`、`a3_ultra_t2d5`、`teleop_avatar` 的模型文件。

### 2.2 机型配置 `config/motion_control/configuration/robot/<robot_name>/`

每个模块一个目录、内含 `default.yaml`（运行时用 `MC_PARAM_ROOT_PATH` 定位）：

| 目录 | 作用 |
|---|---|
| `aimrt/default.yaml` | **核心运行时配置**：插件加载（net/ros2/iceoryx）、线程执行器（绑核 + `SCHED_FIFO`/`SCHED_RR` 实时调度）、channel pub/sub topic 及后端（iceoryx/ros2/http/udp）、RPC（http+ros2）、`MotionControlModule` 的 PublisherManager / SubscriberManager / ServiceManager |
| `policy/` | RL 策略：`dance_policy`、`foundation_policy`、`human_like`、`ling_chuang_policy(_dof29)`、`recover_policy`、`rl_wbc`、`wbc_policy`（ONNX 模型 + 归一化数据集） |
| `get_up/`、`lie_down/`、`package_get_up/`、`package_lie_down/` | 起身/躺下动作及其策略 |
| `pd_stand/`、`ik/`、`damping/`、`butterworth_map/` | PD 站立、逆运动学、阻尼、滤波 |
| `fall_protect/`、`security_guard/`、`service_whitelisting/` | 安全相关 |
| `action_manager/`、`action_setting/`、`motion_list/` | 动作管理；`motion_list/` 含 `README_FORMAT.md`（NPY 动作文件格式：float32，shape `(帧数, 35)`）与若干 `*_ma_cat.npy` 舞蹈动作 |
| `model_info/`、`debug_parameters/`、`fusion/` | 模型信息、调试参数、传感器融合 |

### 2.3 wheel 包 `aimsim-3.3-py3-none-any.whl` 内容

- `aimsim/`：CLI 包，`aimsim mujoco start` / `aimsim isaac start`，入口 `aimsim=aimsim.cli:main`；另有 `config.yaml`、`start.sh`、`start_mujoco.sh`、`kill_sim.sh`、`runtime.py`。
- `mujoco_simulator/`：MuJoCo 仿真器（Python 3.10，依赖 `mujoco==3.1.6`、`mujoco-python-viewer`、`fastapi==0.110.0`、`pydantic==2.8.2` 等），含 `core/`（handler：joint/imu/foot_sensor）、`server/http_server.py`、`app/cfg/<robot>/`（`mujoco_simulator_cfg_sil.yaml` / `_cfg_pv.yaml`）与大量机器人资源（URDF/MJCF/STL/地形）。
- `isaac_simulator/`：Isaac Sim 仿真器，结构类似（`isaac/command/`、`isaac/handler/`、`server/`、`app/cfg/<robot>/`，含 `_cfg_sil.yaml` / `_cfg_hil.yaml`）。**依赖外部 Isaac Sim 4.2**（`isaac-sim-4.2/python.sh`，由环境变量 `USER_ISAAC_PATH` 定位）。
- `run.py` / `run_mujoco.py`：顶层拉起脚本（解析 robot/scenario/sim_mode/modules，拉起 iox-roudi，按 `aimsim/config.yaml` 中的 `modules_bringup` 命令启动各算法模块）。
- `scenarios/`：场景资产（occupancy_map、scenario_config.yaml），如 `warehouse_full`、`home_01`、`hospital` 等。
- `usd/`：机器人 USD 模型（`raise_a3_ultra.usd` 等）。
- `motion_control_service_a3.py`：MC HTTP 服务的示例客户端（摇臂/头/腰关节命令演示）。

## 3. 技术栈与运行时架构

- **运行时框架：AimRT**（C++）。MC 主程序通过插件化方式接入多种通信后端：
  - **iceoryx**（共享内存零拷贝，需先启动 `iox-roudi`）— 与底层 body_drive（驱动/仿真）进程间通信；
  - **ROS 2**（FastDDS，`librmw_fastrtps.so` 随包提供）— `/motion/control/*` 等 topic；
  - **HTTP/UDP**（`libaimrt_net_plugin.so`）— RPC 与外部命令通道。
- **通信 topic 命名约定**：
  - `/body_drive/*` — 与驱动层的接口：`*_joint_command`（MC 发出，500Hz）与 `*_joint_state`、`pelvis_imu/data` 等（MC 订阅）；
  - `/motion/control/*` — 对外控制接口（如 `locomotion_velocity`、各关节命令）与状态反馈（`*_joint_state`，100Hz）；
  - `/mc/state/*` — 动作/动作序列/姿态状态（20Hz）；`/debug_anywhere/*` — 调试扭矩；`/ta/whole_body_command` — 全身控制命令。
- **实时性**：线程绑核（cpu 4/5）+ `SCHED_FIFO:70/71`、`SCHED_RR:50/79` 调度策略；控制环 500Hz（仿真 1000Hz）。
- **策略推理**：ONNX Runtime 1.19.2，打包了 CUDA（236MB）与 TensorRT 后端。
- **依赖的第三方库**（均在 `bin/` 内自带，不依赖系统安装）：absl 2501、boost 1.82、protobuf 30.0.0、Fast-DDS 2.14.5、iceoryx 2.95.4、zenoh-c、LCM 1.5、mosquitto/paho（MQTT）、OpenTelemetry、openssl 3、assimp、octomap 等。

## 4. 构建与运行

### 4.1 运行 MC（真实或 SIL 仿真均适用）

```bash
cd <pkg>/scripts/motion_control
./start_motion_control.sh [选项]
#   -c <绝对路径>  指定自定义 aimrt 配置文件
#   -p <端口>      调试端口（默认 65001）
#   -g             lldb-server 调试模式
#   -v             valgrind 模式（仅 A3_ULTRA 包支持）
```

脚本行为：
1. 从环境变量 `AGIBOT_ROBOT_MODEL`（如 `A3_ULTRA_T2D0`）经 `robot_map` 查表得到配置目录名（如 `a3_ultra_t2d0`）；未设置时交互式列出可选机型；
2. 若 `iox-roudi` 未运行则自动拉起；
3. 设置环境变量后启动 `./motion_control --cfg_file_path=../config/motion_control/configuration/robot/<robot>/aimrt/default.yaml`。
4. 脚本带 SIGINT/SIGTERM 优雅退出处理（最长等待 30 秒后 SIGKILL）。

关键环境变量：`AGIBOT_ROBOT_MODEL`（必选）、`AGIBOT_ROBOT_FORM`（如 `with_casing`）、`LOG_PATH`（默认 `../log`）、`PNC_IP`（默认 `127.0.0.1`）、`MC_PARAM_ROOT_PATH`、`MC_RESOURCE_ROOT_PATH`、`MC_ROBOT_NAME`、`RMW_LIBRARY_PATH`（脚本内自动指向包内 `librmw_fastrtps.so`）。

默认端口：HTTP/RPC **56322**、UDP **50041**。

### 4.2 运行 AimSim 仿真

```bash
pip install aimsim-3.3-py3-none-any.whl   # 需要 Python 3.10/3.11
aimsim mujoco start    # MuJoCo 仿真（wheel 模式，自动设置 LD_LIBRARY_PATH/PYTHONPATH）
aimsim isaac start     # Isaac Sim 仿真（需另装 Isaac Sim 4.2 并设置 USER_ISAAC_PATH）
```

- 通过 `~/.config/aimsim/`（或 `AIMSIM_USER_CONFIG`）下的用户配置选择 `robot`、`scenario`（见 wheel 内 `scenarios/`）、`sim_mode`（`sil` 软件在环 / `hil` 硬件在环）、`ros_domain_id` 与要拉起的算法模块（MC 必选，可选 PNC、SLAM、MOTION_PLAYER 等；见 wheel 内 `aimsim/config.yaml` 的 `modules_bringup`，其 MC 项就是调用本工作区 `motion_control_tar` 包的 `start_motion_control.sh`）。
- 首次运行会把 wheel 内的机型配置 seed 到用户配置目录（机制见 wheel 内 `run_mujoco.py` 的 `seed_manifest.txt`）。
- 停止仿真用 `aimsim/kill_sim.sh`（或包内提供的 kill 脚本）。

### 4.3 "构建"

无源码可构建。发布流程为：打成 `.tar`（本目录中的 `*_humble.tar` / `*_jazzy.tar` 即示例）→ 解压后直接用启动脚本运行。wheel 的构建配置不在本工作区内（wheel 内 `MANIFEST.in` 等仅在其源码仓 `aimsim_distribution` 中存在）。

## 5. 测试与调试

- **无自动化测试框架**。验证方式为：启动 MC + 仿真器，然后用 `config/motion_control/tools/` 下的脚本做接口级验证：
  - `S_*.py` — 通过 HTTP RPC（`http://127.0.0.1:56322/rpc/aimdk.protocol.McBaseService/...`，如 `GetState`、`GetAction`、`PlayMotion`、`SetAction`）查询/触发服务；文件头注释说明功能（中文）；
  - `T_*.py` — 向 topic 通道发布命令（`T_LocomotionVelocity.py` 速度控制、`T_*JointCommand.py` 各关节命令等）；
  - `humanlike_motion_sequence.py` — 拟人动作序列；
  - `deploy_policy_model.sh <跳板机IP> <onnx路径>` — 经跳板机把 ONNX 策略上传到目标主板（10.42.10.12）并改写 `default.yaml` 的 `model_path`；
  - `proto_gen/aimdk/protocol/` — 生成好的 protobuf Python 桩代码（aimdk 协议）。
- 日志：默认 `../log/motion_control.log`（rotate_file 后端，单文件 100MB、最多 50 个）；console 彩色输出。ASAN/TSAN/UBSAN 选项已在启动脚本中预置（log 到 `${LOG_PATH}/*.log`）。

## 6. 安全注意事项

- `config/motion_control/tools/deploy_policy_model.sh` 内**硬编码了跳板机/目标机的用户名密码**（`JUMP_PASS="1"` 等）。修改或外传该文件时注意脱敏；不要把它复制到公开仓库。
- `service_whitelisting/` 与 `security_guard/` 配置涉及 MC 服务的访问控制，改动会直接影响机器人安全行为，修改前务必理解字段含义（注释为中文）。
- 启动脚本用 `export LD_LIBRARY_PATH=./` 优先加载包内自带 so，避免被系统库污染；请勿随意调整加载顺序，否则可能混载不同版本的 absl/protobuf 导致崩溃。
- MC 是 500Hz 实时控制进程，绑核/调度策略与 CPU 拓扑强相关（配置假定存在 cpu 4/5）；在低核数机器上直接运行可能调度失败。
- `deploy_policy_model.sh` 会改写目标板上的 `model_path` 并涉及 sudo/mkdir 操作，属高危操作，执行前确认目标 IP 正确。

## 7. 给编码代理的实用提示

- 改动配置后无需"编译"，重启 `start_motion_control.sh` 即生效；aimrt 配置支持 `${LOG_PATH}`、`${PWD}` 等环境变量展开。
- 三个发布包内容高度同构但不同步：robot_map、机型配置目录、bin 内库集合均有差异（如 `libhpp-fcl.so` 只在 v3.0.19 包中）。做任何跨包修改前先 `diff` 确认。
- 项目注释、文档、日志均使用中文，新增注释请保持中文。
- 工具脚本里 `S_`/`T_` 前缀约定：**S = Service（RPC 查询/设置），T = Topic（发布命令）**。

## 8. 版本管理（2026-11-20 建立）

本工作区已纳入 git 管理（仓库根即本目录）。`.gitignore` 排除了 MC 发布包、wheel、
`a3_ultra_isaaclab/` 模型等大文件；跟踪范围是 `a3_teleop_vr/`、`a3_moveit_demo/`、
`a3_robot_teleop/` 三个自研目录及本文档。

- 基线备份：`main` 分支 + 标签 `backup/pre-xrobot-refactor`
  （XRoboToolkit 世界系映射重构前的 OpenArm 相对模式版本）
- 后续开发在 `xrobot-refactor` 分支进行；回退：`git checkout main` 或
  `git reset --hard backup/pre-xrobot-refactor`
- 启动/验证方式见 `a3_teleop_vr/README.md`（仿真回归用 `scripts/fake_pico_publisher.py`，
  无需头显/眼镜）
- **VR 客户端工作区不在本仓库**：Unity 编辑器、XRoboToolkit 官方工程与自制
  A3TeleopVR 工程都在 Windows/Linux 共用盘 `/media/ubuntu20/软件/Unity*`、
  `…/UnityProjects/` 下（git 各自独立）。方案与路线变更记录见
  `docs/custom-vr-apk-plan.md` §7。

## 8.1 机器人端部署位置（2026-11-20 增补）

机器人端配套（`a3_robot_teleop/` 等）**只能部署到数据盘路径**
`/agibot/data/home/agi/Desktop/`（如 `Desktop/teleop_vr_ws/a3_robot_teleop/`）。
系统盘 home `/home/agi/` 等其他位置会被官方系统**定期清理**，曾导致部署丢失。
部署/更新一律用 scp 覆盖到上述数据盘路径，不要往 `/home/agi/` 拷。

## 8.2 机器人上的工作规矩（2026-11-20 增补，用户亲立）

机器人（agi@192.168.43.10，hdu/mdu/adu 三芯片）跑的是智元官方系统，是贵重资产：

1. **动之前必须说**：修改/新增/删除机器人上的任何文件、配置，或改变运行态
   （`aimcli em stop/start`、起停进程等），**必须先征得用户同意**，说明改什么、为什么。
   唯一例外：用户当场明确指示的操作。
2. **保持干净**：只部署必要文件，位置只能是数据盘
   `/agibot/data/home/agi/Desktop/<工程名>/`（见 §8.1）；不往 `/home/agi`、`/tmp`、
   系统目录随手放东西；运行日志确需落盘的，记入清单（见下条）并定期清理。
3. **测试代码要记账**：在机器人上放的任何临时/测试文件，必须登记到
   `docs/robot-side-inventory.md`（路径 + 用途 + 日期），并**定期询问用户是否删除**，
   不允许留在机器人上无人问津。
4. **工程隔离**：不同工程各待各的文件夹（`Desktop/<工程名>/`），
   文件不散落到 home 根目录等其他地方。

## 9. 工作原则（2026-11-20 起，最高优先级）

**涉及本工程（a3_teleop_vr / a3_moveit_demo / a3_robot_teleop）的任何问题，一律：
先不动代码。** 流程必须是：

1. 先讨论可能的问题点，并指到对应代码位置（文件/函数/参数）；
2. 提出候选方案或中间验证步骤（优先只读：日志、echo、TF 观察、已有脚本）；
3. 等用户明确同意后，才开始改代码或跑有副作用的操作。

任何"直接上手改"的行为都视为违规。唯一的例外：用户当场明确指示的修改
（如"把这条原则写进 AGENTS.md"这类文档同步）。

背景：2026-11-20 映射重构后出现"目标坐标架多方向无法跟随/方向错乱"，
用户判断不像标定问题。为避免在病因未确诊前继续动刀，立此规矩。

## 10. 备忘与待办（长期）

### 10.1 A3TeleopVR 仓库将来若要转公开（2026-11-25 记）
- 现状：Unity 工程已推 `github.com/YiAngLei/A3TeleopVR`（**私有**），master+tag 齐全。
- 背景：XRoboToolkit 官方客户端是 MIT 开源（保留根 LICENSE.txt 即可）；
  但 `PICO Unity Integration SDK/` 目录是专有许可（禁止再分发）。
- 转公开前必须做"排雷"：
  1. `git filter-repo` 把 `PICO Unity Integration SDK/` 从历史**全部**清除并强推；
  2. 加 `.gitignore`；README 注明"SDK 自行从 PICO 开发者平台下载后放到此目录"；
  3. 审计 `Assets/Plugins` 下其余 PICO 二进制（tobframwork aar 等）是否同属 SDK。
- 时机：用户决定公开时执行，不在功能开发期做历史改写。

### 10.2 视频回传链路结论（M3-P0 摸底，2026-11-25）
- XRobotToolkit 视频链**不走 ROS**：PC Service 无任何 ROS/DDS 库；
  APK 发 `StartReceivePcCamera{ip=头显IP,port,...}`（RemoteCameraWindow.cs），
  视频为裸 UDP 推流，头显端 PICO Java 库 MediaCodec 硬解（`MediaDecoder(UDP)`）。
- 显示参数照抄 `~/xrt-config-backup/files/video_source.yml` 的 PICO4U 档。
- 唯一未验证环：机器人端守护进程（待机器人上线后 `ps aux | grep -iE 'xrt|xrobot|pcp|agent'` 1分钟确认）。
- 详情与 M3 计划：`docs/custom-vr-apk-plan.md`。

### 10.3 代理排雷（2026-11-25）
- 本机 Clash Verge 出口节点池有坏节点：对 github git 端点间歇 403/断流（API 正常）。
- 推送前多重试或让用户在 Verge 里给 github.com 指定固定节点。
- 环境全局带代理（http/https/ALL_PROXY=socks），"直连"测试前先 `env | grep -i proxy`。
