# 机器人文件清单（robot-side inventory）

> 规矩见 `AGENTS.md` §8.2：机器人上只放必要文件，测试代码记账于此，定期问用户是否删除。
> 机器人：`agi@192.168.43.10`（部署一律走数据盘 `/agibot/data/home/agi/Desktop/`）。

## 正式部署（长期保留）

| 路径 | 用途 | 登记日期 |
|---|---|---|
| `/agibot/data/home/agi/Desktop/teleop_vr_ws/a3_robot_teleop/` | 机器人端配套：域桥（232↔42）、状态回显、指令监视、急停保持。日志在其 `logs/` | 2026-11-20（既往部署，当日核对一致） |

## 运行态变更（非文件，2026-11-20 收摊时已全部停止）

| 项目 | 状态 | 重新启动 |
|---|---|---|
| `motion_player` 应用 | **已停止**（发手臂指令的官方硬性前提；收摊时保持停止） | `~/.local/bin/aimcli em start motion_player` |
| 域桥 6 个 pipe 进程 + locomotion_relay | 已停止 | `bash start_domain_bridge.sh all`（Desktop 那份） |
| `command_monitor.py` | 已停止 | 见 README「实机每日流程」第 4 步 |

## 配置变更

| 项目 | 内容 | 登记日期 |
|---|---|---|
| `/home/agi/.ssh/authorized_keys` | 追加 PC 的 ed25519 公钥（免密运维） | 2026-11-20 |

## 临时/待清理候选（定期问用户）

| 路径 | 用途 | 登记日期 | 状态 |
|---|---|---|---|
| `/tmp/command_monitor.log`（机器人） | command_monitor 运行日志 | 2026-11-20 | 监视器还开着，会话结束可删 |

## 已删除

| 路径 | 说明 | 删除日期 |
|---|---|---|
| `/home/agi/a3_robot_teleop/` | 误部署到系统盘 home（会被定期清理），已改放数据盘 | 2026-11-20 |
