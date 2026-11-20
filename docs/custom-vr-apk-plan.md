# 自研 PICO VR 客户端（路线 B）详细方案

> 状态：**设计定稿，未实施**。日后动工以本文档为准。
> 定调：数据链路**协议层原样复用 XRoboToolkit 官方实现**（MIT 许可，`XRoboToolkit-Unity-Client` 仓库
> `Assets/Scripts/Network/*.cs` 整体移植，消息格式一字不改）——PC Service、`a3_xrt_bridge`
> 及 XRoboToolkit 全生态（双 UR5 示例、TWIST2 等）**零改动兼容**。
> 目标机型：PICO 4 Ultra（消费者版即可，不依赖企业 API）。

## 1. 需求摘要（按 2026-11-20 谈话定稿）

### 1.1 功能（仅此这些，多一分都不做）

| 项 | 要求 |
|---|---|
| 数据 | 头显 + 双手柄位姿/扳机/握把/按键，走 **XRoboToolkit 官方协议**到 PC Service；高质量（预测位姿、固定发送频率、官方时间戳/校验机制） |
| 主界面 | **唯一界面，无任何菜单层级**：IP 输入框（带下拉，列出自动搜索到的 PC）+ 数字键盘（10 数字 + 1 点 + 1 退格，布局参考 OpenArmX APK）+ 连接按钮 |
| 连接反馈 | 连上后显示"成功" + **实时发送频率（Hz）** |
| 机器人视角 | 底部大字"按下A键进入机器人视角"；A 键**两段式安全确认**（见 1.2）；进入后为左右眼分屏的机器人双目全景视频，再按 A 退出，可往复 |
| 不做 | 手势追踪、体感追踪器、录像、远程视觉配置页、企业功能、任何二级页面 |

### 1.2 A 键安全互锁状态机（防误触，遮挡真实环境有安全风险）

```
[IDLE]  大字提示"按下A键进入机器人视角"
   │ 按 A（第一次）
   ▼
[CONFIRM] 强制模态框："再次按下A键进入机器人全景视角，请确保周围环境安全"
          · 框至少显示 3 秒，3 秒内按 A 无效
          · 框显示期间其他 UI 全部禁用
   │ 3 秒后按 A（第二次）
   ▼
[ROBOT_VIEW] 全屏双目视频（左相机→左眼，右相机→右眼，分眼渲染）
   │ 按 A
   ▼
[IDLE] 回到大字提示 —— 下次进入需重新两段确认
```

规则：**每次**进入都必须重新"两按 A"；第二次确认与第一次之间若超过 10 秒自动作废（回到 IDLE）。

### 1.3 UI 布局（唯一界面，ASCII 示意）

```
┌──────────────────────────────────┐
│  PC IP  [ 192.168.43.5    ▼ ]    │ ← 下拉列出自动搜索到的 PC（可点选填入）
│ ┌────┬────┬────┬────┐            │
│ │ 1  │ 2  │ 3  │ 4  │            │
│ ├────┼────┼────┼────┤            │
│ │ 5  │ 6  │ 7  │ 8  │            │ ← 数字键盘（OpenArmX 风格大按键）
│ ├────┼────┼────┼────┤            │
│ │ 9  │ 0  │  . │  ⌫ │            │ ← 点 + 退格
│ └────┴────┴────┴────┘            │
│ ┌──────────────┐  状态: 已连接    │
│ │    连  接     │  发送 72 Hz      │ ← 连接后显示频率
│ └──────────────┘                  │
│                                  │
│     按下 A 键进入机器人视角        │ ← 常驻大字（VR 中也固定于面板底部）
└──────────────────────────────────┘
```

## 2. 技术方案

### 2.1 工程基座

- **Unity 2022.3 LTS** + Android 构建模块。
- **PICO Unity Integration SDK**：直接取自 `XRoboToolkit-Unity-Client` 仓库 vendor 的
  `PICO Unity Integration SDK/` 目录（版本已与官方工程验证匹配），或 PICO 开发者站同版本下载。
- **新建干净工程**（不从官方工程删改）：`Packages`/`ProjectSettings` 按官方仓库对齐，
  仅复制以下官方代码（MIT，保留 LICENSE 声明）：
  - `Assets/Scripts/Network/**`（≈18 个 C#：**协议层整体移植，禁止改动消息格式**）
  - `Assets/Scripts/TrackingData.cs` 中的**采集逻辑**（改写成我们的 `PoseCollector`）
  - `Assets/Scripts/Utils.cs` 中协议需要的工具方法
- 其余（UI/Camera/Audio/MediaDecoder/Diagnostics 等官方模块）**一律不写不搬**。

### 2.2 数据链路（高质量要求逐项落实）

```
PoseCollector（每发送周期）：
  头部位姿 ← PXR_System.GetPredictedMainSensorStateNew(predictedDisplayTime)
              ↑ 官方同款"预测位姿"，比查询式位姿低 10~20ms 延迟 —— 必须照抄
  手柄位姿/扳机/握把/A/B/X/Y ← UnityEngine.XR.InputDevice（CommonUsages）
  组包     ← 官方 NetworkDataProtocol 消息类型（格式不变）
  发送     ← 官方 NetPacket/OperatorControlClient 通道（TCP 63901，心跳/重连沿用官方实现）
  频率     ← 独立定时器 72Hz（不跟渲染帧率耦合）；主界面实时统计实发频率显示
```

- **坐标系不做任何转换**：发送 Unity 原始 tracking-origin 系（即房间锚定系），
  轴向转换继续由 PC 端 `world_rotation` 负责（与现状一致，双端职责不变）。
- 时间戳、校验和、心跳：全部沿用官方协议机制（`NetworkDataProtocolSerializer` 等），不重写。

### 2.3 PC 侧自动发现（下拉列表的数据源）

- 复用官方发现机制：PC Service 启动后会广播/应答发现消息（官方 App 的"弹窗确认服务器"
  即源于此；实现细节以 `NetUtils.cs`/`OperatorControlClient.cs` 代码为准，动工时提取）。
- 我们的 App 常驻监听发现应答 → 收编为下拉候选 IP；同时保留手动输入兜底。
- 动工第一步先做一个**协议嗅探/对齐验证**：跑官方 App + 我们的最小客户端并抓包对比，
  确认握手/心跳/消息序列完全一致后再写业务（降低协议黑盒风险，见 §5 风险表）。

### 2.4 机器人双目视频（M3，分眼渲染）

- **发送端（PC/机载）**：A3 Ultra 的 RealSense D415 双目 → librealsense 采集 →
  H.264 编码（720p，目标 2~4 Mbps）→ RTP/UDP 推流。可参考 XR-Robotics 生态的
  `XRoboToolkit-Orin-Video-Sender`。
- **眼镜端**：低延迟解码（FFmpeg 硬解或 `RobotVisionUnityPluginQuest` 思路自研插件；
  **禁用** Unity VideoPlayer 网络流，延迟不可控）。
- **分眼渲染**：两个 Quad + 两个 Camera 分别 `stereoTargetEye = Left/Right`；
  进入 ROBOT_VIEW 时启用，退出即隐藏。
- **延迟预算**：端到端 < 300ms（采集 30 + 编码 20 + 传输 30 + 解码 40 + 渲染 30 ≈ 150ms，留余量）。

## 3. 里程碑

| 里程碑 | 内容 | 预估 | 验收 |
|---|---|---|---|
| **M1 协议对齐** | 建工程、搬 Network 层、最小客户端连上 PC Service，官方 SDK（`a3_xrt_bridge`/`xrt` 探针）能读到头/手柄位姿 | 3~5 天（含 Unity 环境） | `ros2 topic hz /pico_left_controller/pose` ≈ 72Hz；数据为房间锚定系；grip/扳机/A/B 可用 |
| **M2 定制 UI** | §1.3 唯一界面：IP 框+下拉、数字键盘、连接、频率显示 | 1~2 天 | 界面与需求图一致；自动发现填充下拉；断线重连有状态提示 |
| **M3 视角互锁+视频** | §1.2 状态机 + 双目接收分眼显示 + PC 端推流 | 3~5 天 | 两段确认时序逐条通过；视频 <300ms；进出视角不崩不断流 |

## 4. 目录规划（未来仓库新增）

```
vr-client/                      # 本方案产物（Unity 工程）
├── Assets/
│   ├── Scripts/
│   │   ├── Network/            # 官方 MIT 移植（勿动）
│   │   ├── PoseCollector.cs    # 采集（官方 TrackingData 改写）
│   │   ├── Main.cs             # 状态机 + UI 事件
│   │   ├── RobotView/          # 视频接收/解码/分眼渲染（M3）
│   │   └── Ui/                 # IP 面板 + 键盘（M2）
│   └── Plugins/                # PICO SDK / 解码插件
├── Packages/  ProjectSettings/
└── README.md                   # 构建+安装说明（adb 开发者模式）
```

## 5. 风险表

| 风险 | 缓解 |
|---|---|
| 协议握手/注册序列有隐藏步骤 | M1 先抓包对齐官方 App；Network 层整搬不重写；逐步加日志比对 |
| 自动发现机制与文档不符 | 同上，以官方代码为准；最坏情况退化为"手动 IP + 下拉仅历史记录" |
| PICO SDK 版本漂移 | 锁定官方工程 vendor 的 SDK 版本；升级需回归 M1 验收 |
| 视频延迟超标 | 编码参数/缓冲可调；预留"单眼小窗"降级模式 |
| 消费者版 sideload/签名 | 开发者模式 + adb install（本机已有 adb 通道，已验证） |

## 6. 不在本期范围（防范围蔓延）

手势追踪、体感追踪器、透视相机/MR 叠加（企业授权）、录像与回放、多机管理、
英文界面（首版中文即可）、任何设置页/二级页面。

---

*设计：2026-11-20 · 依据 XRoboToolkit-Unity-Client（MIT）源码勘察与 OpenArmX APK 界面参考。*
*动工前请重读 `AGENTS.md` §9：先讨论、后动手。*
