# a3_robot_teleop — 机器人端遥操作配套工具

配合主机端 `a3_teleop_vr` 的两种真机模式使用（两者互斥，都往
`/motion/control/arm_joint_command` 发指令，勿同时开）：

- **模式 ① Pico 流式遥操作**：`teleop_vr.launch.py real:=true`
- **模式 ② RViz 规划执行**：`real_teleop.launch.py`（move_group 规划 →
  FollowJointTrajectory 适配器 → MC；机器人端无任何变化，仍只跑 MC）

**把本目录 scp 到机器人上**（纯 rclpy + 标准消息，无其他依赖，不依赖智元任何 SDK）：

```bash
scp -r a3_robot_teleop/ <机器人用户>@<机器人IP>:~/
```

## 域桥（dds_domain_pipe_*）：MC 域 ↔ 主机域

智元 MC 等系统节点在本机的 ROS 域（如 232）发布话题，而主机 PC 与机器人
只在另一个域（如 42）互通。rclpy 单进程只能属于一个域，所以用
"订阅进程 + 本机 TCP + 重发进程"成对桥接，只转发两个手臂话题：

```
/motion/control/arm_joint_state   232 --> 42   （状态，只读，随时可开）
/motion/control/arm_joint_command  232 <-- 42   （指令，开了主机就能驱动机器人！）
```

```bash
bash start_domain_bridge.sh state     # 只起状态方向（只读，安全）
bash start_domain_bridge.sh command   # 只起指令方向（确认 MOTION 状态后再开）
bash start_domain_bridge.sh all       # 两个都起
bash stop_domain_bridge.sh            # 全部停掉
```

- 进程与日志在 `logs/`（pipe_*.log / pipe_*.pid），`stop` 按 pid 文件精确清理
- 订阅 QoS 按方向区分：状态方向必须 `best_effort`（MC 是尽力发布），
  指令方向 `reliable`（跨 WiFi 重传防丢帧）；`--qos` 参数已内置在启动脚本里
- 改域号：编辑 `start_domain_bridge.sh` 顶部 `SYS_DOMAIN` / `EXT_DOMAIN`
- 本机自测（不碰真实话题）：

```bash
source /opt/ros/jazzy/setup.bash
cd a3_robot_teleop
ROS_DOMAIN_ID=232 python3 dds_domain_pipe_sub.py --topic /lpt --type std_msgs/msg/Int32 \
    --port 53930 --qos best_effort &
ROS_DOMAIN_ID=42  python3 dds_domain_pipe_pub.py --topic /lpt --type std_msgs/msg/Int32 \
    --port 53930 &
sleep 2
ROS_DOMAIN_ID=232 ros2 topic pub -r 10 /lpt std_msgs/msg/Int32 "data: 0" &
ROS_DOMAIN_ID=42  timeout 5 ros2 topic echo /lpt std_msgs/msg/Int32 --once
```

## 运行前提（机器人上）

```bash
source /opt/ros/<ros2版本>/setup.bash      # 机器人自带的 ROS 2 环境
export ROS_DOMAIN_ID=<与主机端一致>          # 主机 teleop 能连上机器人的关键
python3 arm_state_echo.py                   # 三个脚本任选一个
```

## 脚本

| 脚本 | 作用 | 什么时候用 |
|---|---|---|
| `arm_state_echo.py` | 1Hz 打印 14 个手臂关节当前角度 | 首次部署：确认 ROS 域/网络/MC 状态反馈正常 |
| `command_monitor.py` | 每 2s 统计指令流的频率/间隔/速度，超官方限值(30ms/4rad/s)告警 | 遥操作联调全程挂着 |
| `hold_position.py` | 锁定当前关节角 100Hz 发送保持指令 | 急停保持：主机 teleop 异常时先停 teleop 再跑它 |
| `dds_domain_pipe_sub.py` | 域间管道发送端：本域订阅 -> 本机 TCP | 被 start_domain_bridge.sh 调用，一般不单独跑 |
| `dds_domain_pipe_pub.py` | 域间管道接收端：本机 TCP -> 本域重发 | 同上 |
| `start_domain_bridge.sh` | 一键起 state/command/all 桥 | 主机联调前（见上节） |
| `stop_domain_bridge.sh` | 按 pid 文件停掉全部管道 | 联调结束或异常处置 |

## 标准真机遥操作流程

0. 选定模式：手柄连续跟随用 ①，RViz 拖小球/脚本规划执行用 ②；
   确认主机 PC 与机器人的互通域一致（`ROS_DOMAIN_ID` 两边相同，
   机器人侧若有 `ROS_LOCALHOST_ONLY=1` 需 unset）
1. 机器人端：`python3 arm_state_echo.py` 确认状态反馈正常
2. 确认 MC 处于 **MOTION 状态**、**motion_player 已关闭**（官方硬性要求）
3. 机器人端起域桥：`bash start_domain_bridge.sh all`
   （只想先安全观察可只起 `state`；`command` 一开主机即可驱动机器人）
4. 主机端（一条命令，均不拉起仿真栈）：
   ```bash
   # ① Pico 流式遥操作
   ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py real:=true bridge:=true
   # ② RViz 规划执行（打开 RViz，拖小球点 Plan&Execute 即直驱真机双臂）
   ros2 launch <repo>/a3_teleop_vr/launch/real_teleop.launch.py
   ```
   ① 的 teleop 节点内嵌 MoveItPy 做 IK，直接对机器人 MC 发指令、收状态；
   ② 由 a3_mc_controller 提供标准 FollowJointTrajectory action server，
   经 100Hz 安全链（限位/限速/低通）发 MC，真机状态经回显显示在 RViz。
5. 机器人端：`python3 command_monitor.py` 挂监视
6. 异常处置：主机 Ctrl+C 停主机端程序 → 机器人端 `bash stop_domain_bridge.sh`
   或 `python3 hold_position.py` 冻结手臂（hold_position 与主机端不可同时发指令）

## 安全须知

- 三个脚本都只是"接口使用者"，安全边界在 MC；**物理急停始终优先**
- `hold_position.py` 与主机 teleop **不可同时发指令**
- 手臂接口仅限 MOTION 状态调用（MC 会拒绝其他状态下的指令）
