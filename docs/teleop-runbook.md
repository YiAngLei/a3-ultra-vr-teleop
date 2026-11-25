# A3 遥操作拉起手册（Pico 自制客户端）

链路拓扑（每一环都可能是断点，排障从下往上查）：

```
头显 A3 Teleop APK ──WiFi TCP:63901──▶ PC Service ──本地SDK──▶ a3_xrt_bridge
      （位姿上行）                                                    │
                                          /pico_<side>_controller/pose │
                                                                     ▼
                                            a3_streaming_teleop（50Hz IK）
                                                                     │
                            ┌──────────────────┬─────────────────────┘
                            ▼                  ▼
                     RViz 仿真（UR5）      真机 G1（MC 关节指令）
```

**组网铁律**：手机和电脑开热点组网（电脑 ≈192.168.43.5、头显 ≈192.168.43.11）。
访客 WiFi（如 "Visitors"）开 AP 隔离，客户端互访全断，不可用。电脑开热点会断
自身 WiFi；手机热点最省事。

---

## 1. PC Service（任何组合都需要，常驻）

```bash
cd /opt/apps/roboticsservice && nohup bash runService.sh >/tmp/pcservice.log 2>&1 &
```

- **必须用 `bash` 跑**（直接 sh 会 "Bad substitution" 且起不来）。
- 卡死症状：进程在但 63901 不监听、或头显连上无数据——`pkill -f RoboticsServiceProcess` 后重跑上面命令。
- 验证在监听：`ss -tlnp | grep 63901`。

## 2. 环境（每个新终端跑一次）

```bash
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
```

ROS 2 Jazzy 已系统安装；脚本会防 conda python 抢位。ros2 命令异常时先检查是否 source。

---

## 3. 组合 A：只验证头显↔PC（不用 ROS，30 秒）

只要 PC Service。头显里打开 A3 Teleop → 输 192.168.43.5 → 连接。
- 界面显示"已连接 + 约 72 Hz"即 APK 正常。
- **真数据验证**（显示有频率≠数据在流）：PC 上执行
  `ss -ti state established '( sport = :63901 )'`
  看 `bytes_received`：每秒几十 KB 在涨=数据在流；长期只有几十~一百多字节=握手没完成
  （先重启 PC Service 再看，若复现联系排查）。

## 4. 组合 B：Pico + UR5 仿真（RViz 跟随）

```bash
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
ros2 launch ~/AimSim/a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

头显连接后，**握 grip（>0.5）使能**，移动手柄 → RViz 双臂跟随。
右手柄 B=回零，左手柄 Y=举手。嫌灵敏加参数 `pos_scale:=0.5`（改后需重启栈）。
验证数据：`ros2 topic hz /pico_right_controller/pose`。

## 5. 组合 C：Pico + 真机 G1

### 5.1 互斥关系（先搞清楚有三种真机模式）

都往 `/motion/control/arm_joint_command` 发指令，**同一时刻只能开一种**：

| 模式 | 命令 | 用途 |
|---|---|---|
| ① Pico 流式直驱 | `teleop_vr.launch.py real:=true bridge:=true` | 手柄连续跟随，不经 MoveGroup，延迟最低 |
| ② RViz 拖小球驱动真机 | `real_teleop.launch.py` | 打开 RViz，拖小球点 Plan&Execute 直驱双臂 |
| ③ RViz 联动 + 流式遥操作 | `real_teleop.launch.py teleop:=true bridge:=true` | RViz 显示 G1 模型，遥操作随时切入——**RViz 和真机联动就是这条** |

① 和 ②/③ 不要同时开（指令打架）；② 和 ③ 是同一个 launch 的不同开关，自然二选一。

### 5.2 机器人侧完整操作（ssh agi@192.168.43.10，密码 1）

前提确认：
1. `source /opt/ros/<机器人自带版本>/setup.bash`
2. `export ROS_DOMAIN_ID=<与主机一致>`（主机联调的关键；若有 `ROS_LOCALHOST_ONLY=1` 需 unset）
3. `python3 arm_state_echo.py` —— 确认 14 个手臂关节有状态反馈（首次部署必做）
4. 确认 MC 处于 **MOTION 状态**、**motion_player 已关闭**（官方硬性要求）
5. **动机器人前必须再跟我确认一次**（两段式确认规矩）

联调：
6. `bash start_domain_bridge.sh all`（只想先安全观察可只起 `state`；`command` 一开主机即可驱动机器人）
7. 遥操作期间挂着 `python3 command_monitor.py`（超 30ms/4rad/s 官方限值会告警）

异常处置：
8. 主机 Ctrl+C 停主机端程序 → 机器人端 `bash stop_domain_bridge.sh`；
   手臂要冻结就 `python3 hold_position.py`（**它与主机 teleop 不可同时发指令**）
9. 物理急停始终优先于一切软件处置

### 5.3 主机侧命令

```bash
source ~/AimSim/a3_teleop_vr/setup_teleop_env.bash
# 按 5.1 表格三选一，例如模式③：
ros2 launch ~/AimSim/a3_teleop_vr/launch/real_teleop.launch.py teleop:=true bridge:=true
```

## 6. 组合 D：仿真 + 真机同时开？

**不要**。仿真栈（ros2_control 后端）与真机模式（MC 后端）同时跑会让
a3_streaming_teleop 把 IK 结果发往两条指令通道，打架。
注意：**"RViz + 真机联动"是支持的**（§5.1 模式③），禁止的是"仿真遥操作与真机遥操作同时在线"。

---

## 排障速查（按出现频率排序）

| 症状 | 第一动作 |
|---|---|
| 头显连接失败 | 两端是否同一热点；PC Service 是否刚重启过（重启后头显要**断开重连**） |
| 连接成功但机器人不动 | `ss -ti ...` 看 bytes_received；不涨=数据没出 PC Service（重启它）；涨=查 `ros2 topic hz /pico_*_controller/pose` |
| ROS 命令报 context not valid | 没 source 环境（见第 2 节） |
| 官方客户端正常、自制不行 | 先把官方客户端连一次做对照（包 com.xrobotoolkit.client），一刀切开问题在头显端还是 PC 端 |
| 起错桥 | 现在一律用 xrt 桥；`a3_teleop_bridge` 是旧的 OpenArmX UDP 桥，别起 |
| 换网络后连接失败 | 新网段重新输 PC 的 IP（▼ 发现可自动扫）；访客网 AP 隔离不可用大退 |

## 头显端操作速记

1. 打开 A3 Teleop → 发现菜单自动弹（"上次连接"置顶，可直点）
2. 连接 → 状态"已连接"、发送 ≈72 Hz
3. 握 grip 使能 → 动；B/Y 回零/举手
4. 断开按钮可重连
