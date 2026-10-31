# a3_teleop_bridge — Pico VR UDP 桥接（自研兼容实现）

收听 Pico 头显上 VR 遥操作 App 通过 UDP 发送的手柄/头显报文，发布为 ROS 2 话题，
供 `a3_teleop_vr` 的遥操作节点（或任意订阅 `/pico_*` 话题的节点）消费。
是 OpenArmX `openarmx_teleop_bridge_vr` 接收端的**功能等价替代品**。

## 来源与许可说明

- 本目录代码为**全新实现**（Apache-2.0，与本工程一致），按报文格式这一事实性协议
  信息编写，**未复制** OpenArmX 的代码。
- 报文格式参考来源：[openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr)，
  版权 © 成都长数机器人有限公司，许可 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)。
  感谢 OpenArmX 公开协议格式。
- **Pico 端仍需 OpenArmX 的 App**（[openarmx_teleop_vr_apk](https://github.com/openarmx/openarmx_teleop_vr_apk)），
  请从其官方仓库获取并在头显上安装运行；本工程不包含 APK。

## 用法

```bash
# 直接运行（默认 0.0.0.0:5100）
/usr/bin/python3.12 a3_teleop_bridge_node.py

# 或随遥操作栈一起启动（launch 参数 bridge:=true）
ros2 launch a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

参数（ROS 参数均可通过 `--ros-args -p name:=value` 覆盖）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `listen_address` | `0.0.0.0` | 监听地址 |
| `listen_port` | `5100` | 监听端口（与 Pico 端 App 配置一致） |
| `frame_id` | `pico_hmd` | 发布位姿的 frame_id |
| `publish_tf` | `false` | 是否同时广播 `pico_hmd -> pico_{left,right}_controller` TF |

## 报文协议

UDP 文本报文（空白分隔，单报文 < 512 字节），完整定义见
`a3_teleop_bridge_node.py` 模块 docstring。要点：

- `LEFT`/`RIGHT`（及缩写 `L`/`R`）：相对位姿 + 扳机 + 握把 + 按键 + 档位；
- `LEFT_ABS`/`RIGHT_ABS`：绝对位姿（发到 `vr/*` 话题）；
- `HEAD`：头显位姿；`JOY`：摇杆；`BTN`：摇杆按键；
- `CFG POSE_MODE`、`MODE`、`CALIBRATE_DONE`：模式与标定通知。

## 自测

无需头显，本机 UDP 环回验证全部报文类型与容错：

```bash
source <工程>/setup_teleop_env.bash
/usr/bin/python3.12 a3_teleop_bridge/test_bridge_udp.py
```
