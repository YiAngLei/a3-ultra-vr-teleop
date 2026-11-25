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


---

## 7. 路线变更日志

### 2026-11-24：Spatial SDK 路线否决，回到 Unity；环境落地

**否决记录**：曾调研 PICO Spatial SDK（原生 Android/Kotlin，spatial-ui 组件 +
spatial-video SBS 立体）作为 Unity 的替代。闸口检查不通过：头显为 **PICO OS
5.15.4**（Android 14，API 34，2026-01 构建），而 Spatial SDK 要求 **PICO OS 6+**
（示例 minSdk=35 / Android 15）。OS 6 是 2026-03 预告的下一代空间系统，绑定新旗舰
"Project Swan" 封闭先锋测试，PICO 4 Ultra 无升级渠道。结论：该路线在现有设备上
不可行，仅留档（SpatialUI 示例克隆在 `~/AimSim/vr-client/reference/`，已 gitignore）。

**实际环境（本机）**：

| 项 | 位置/值 |
|---|---|
| Unity 编辑器 | **2022.3.16f1**（changeset `d2c21f0ef2f1`，照官方客户端工程锁定），含 Android Build Support |
| 安装根 | `/media/ubuntu20/软件/Unity/editors`（Windows/Linux 共用 NTFS 盘，459G 可用；根分区零占用） |
| 编辑器下载 | Unity CLI 直链 `download.unity3d.com/.../Unity-2022.3.16f1.tar.xz`，curl `-C -` 断点续传（CLI 自带下载在国内网络会被 TLS reset） |
| Unity CLI | `~/.local/bin/unity`（`unity.com/install.sh`，免 sudo）；登录身份 cs1759328280@gmail.com，Personal 许可证已激活 |
| 官方参考工程 | `/media/ubuntu20/软件/UnityProjects/XRoboToolkit-Unity-Client`（zip 方式取得；含 `Assets/Scripts/Network/` 协议层与 vendor 的 PICO Unity Integration SDK） |
| 自制客户端规划位置 | `/media/ubuntu20/软件/UnityProjects/A3TeleopVR`（独立 git 仓库，建成后推到 YiAngLei 名下） |
| Android SDK（备用） | `~/Android/Sdk`（platform-35 / build-tools 35 已装） |

**侦察副产物（影响后续里程碑）**：

1. **A3 官方相机接口**（aimdk v3.3 文档 `07-06-sensor_data`）：机器人已在 ROS2 话题上
   发布全部相机的 **H265 编码流**（`foxglove_msgs/CompressedVideo`）——头部双目左右各
   一路 2400×1800@30、胸前 D457 / 腰前 D415 为 720p@30。M3 视频推流端**不需要在机器人
   上装任何编码器**，只需订阅转发；注意本机系统是 v3.2，话题存在性需只读验证。
2. **OpenArmX APK 技术栈**：纯原生 C++ OpenXR（Khronos "BasicDemo" 骨架，`classes.dex`
   仅 2.9KB），**闭源**（CC BY-NC-SA，仅分发二进制）。它**自带视频接收端**（头+双手
   相机，UDP 5600/5601/5602，H264）——是视频链路的最短验证路径，但 codec 与 A3 的
   H265 不匹配，需转码或实测兼容性；且为平面面板非双目分眼。
3. **OpenArmX 桥出局**：`bridge_impl` 双选保留 xrt 唯一有效路径；openarmx 分支待自制
   客户端跑通后删除（用户已定调，删除前会再确认）。

*§2 的 Unity 2022.3 LTS、M1/M2/M3 里程碑与"协议层原样复用"基调继续有效；本日志仅
修正实现路径与补充环境事实。*

### 2026-11-24 补记：M1 协议验证闭环 ✅

自建 APK（`xrt-client-debug.apk`，官方工程 + SimpleBuild.cs 单产物调试构建）在
PICO 4 Ultra 上跑通全链路：头显 TCP 63901 → PC Service → xrobotoolkit_sdk →
`a3_xrt_bridge/a3_xrt_bridge_node.py` → `/pico_<side>_controller/pose` **100Hz**
（双手柄+扳机实测 99.99~100.02Hz，超预期 72Hz；房间锚定系）。

本机 toolchain 踩坑全记录（后续构建照此办理）：
1. **Unity 编辑器/Android 模块**：CLI 自带下载被 GFW 掐流（TLS reset），改 curl
   `-C -` 断点续传手动装；Android 主模块官方 feed 只给 mac .pkg（连 Hub 数据也
   一样），按 manifest 子组件（dl.google.com 可达）+ 解 pkg Payload 手动落位
   `Editor/Data/PlaybackEngines/AndroidPlayer/`。
2. **代理**：本机 Clash 端口 **127.0.0.1:7897**。Unity 编辑器/UPM 认
   `HTTPS_PROXY` 环境变量；**gradle 不认环境变量**，必须写
   `~/.gradle/gradle.properties`（systemProp.http/https.proxyHost/Port）。
3. **非 ASCII 路径**：Android 构建硬性拒绝中文路径——`/media/ubuntu20/软件` 已
   bind 挂载 `/mnt/soft`（fstab 持久化），Unity 工程一律走 `/mnt/soft/...`。
4. **签名**：原工程 PlayerSettings 带自定义 keystore（密码不可得），构建脚本里
   `PlayerSettings.Android.useCustomKeystore = false` 改调试签名。
5. **PC Service 会卡死**：`RoboticsServiceProcess` 老进程可能不监听 63901，
   症状"connect error"——pkill 后用 `/opt/apps/roboticsservice/runService.sh`
   重启即恢复。
6. **别起错桥**：`a3_teleop_bridge`（OpenArmX UDP）与 `a3_xrt_bridge`（官方
   SDK）是两个节点，M1 以后一律用后者。
7. 头显换包提示：卸载官方包前已备份 `files/video_source.yml` 等至
   `~/xrt-config-backup/`（内含官方双目 SBS 渲染 shader 参数，M3 参考）；
   企业版透视相机库 `libpxr_xrsdk_native` 在自建包缺失（DllNotFoundException，
   非致命，M3 不用它）。

### 2026-11-25 补记：M2（自制 UI 交互）+ M2.5（真连接）闭环 ✅

自制客户端工程 `XRoboToolkit-Unity-Client` 的副本 **`A3TeleopVR`**
（`/mnt/soft/UnityProjects/A3TeleopVR`，已建 git；官方工程原封不动作参照）。
包名 `com.yianglei.vrteleop`，与官方包并存头显。单场景 `A3Main.unity`
（复制自 Main.unity，禁用 UIOperate/CameraSource/SlitView/LogWindow/AndroidProxy，
注入 A3UI 根挂 `A3UiBootstrap`）。

**UI 交互定案（v10 趟通，v11 清理后 ~200 行）**——全部界面运行时代码构建：
IP 输入框 + ▼发现下拉(mock) + 3×4 数字键盘(0./⌫) + 连接按钮 + 状态/发送频率 +
底部"A 键进入机器人视角"大字。交互链路（勿回退）：
`官方 rig 的 XRRayInteractor + XRUIInputModule → 画布上挂 PICO 官方
TrackedDeviceGraphicRaycasterEventCamera（com.picoxr.tobframwork.UI）+ GraphicRaycaster，
worldCamera 运行时继承官方（已禁用）画布的 worldCamera（=rig 内中央相机）`。
三个血泪要点：
1. **所有 Text raycastTarget=false**——代码创建的 Text 默认 true，会盖在按钮
   上层截走射线，且 uGUI 事件不冒泡，按钮永远收不到点击（v2–v9 的"射线穿面板/
   点不动"的真正根因，其余什么 worldCamera/注册时机全是噪音）。
2. **全部 UI 元素 layer=5**（官方 UIOperate 同款；DefaultRaycastLayers 虽含
   layer0，但要对齐官方）。
3. **别自己造 EventSystem**——官方 prefab（PICO XR Interaction Setup）里那套
   是验证过的；只在其完全缺失时新建补齐，绝不删除/禁用/重建。

**M2.5 真连接（7813403）**：新增 `A3NetBridge.cs`（~60 行）组装官方验证过的
组件，一行官方代码不改：
- `FPSDisplay` 必须有实例——TcpHandler 发送线程每发一帧调其 static
  `UpdateTime()`，stopwatch 在 Awake new，缺实例则发送线程 NRE 自杀（隐藏依赖）。
- `TcpHandler.SetDeviceSn(sn)` 必须先于 Connect——ConnectCallback 仅在
  _deviceSN 非空时发握手包（PACKET_CCMD_CONNECT / SEND_VERSION）；
  SN 用 `PXR_Enterprise.StateGetDeviceInfo(SystemInfoEnum.EQUIPMENT_SN)`。
- 位姿开关显式全开 `TrackingData.SetHeadOn/SetControllerOn/SetHandTrackingOn`
  （UIOperate 被禁用，没人替我们开；TrackingType 保持 None——无体感追踪器）。
- 连接按钮 → `TcpHandler.Connect(ip)`（63901，协议原样）；状态轮询
  `SocketState` 状态机；频率=主线程采集帧率（链路不积压时==PC 端实测接收频率）。
- 断开 = `SendTrackingData=false + Close()`；IP 先用 IPAddress.TryParse 校验
  （TcpHandler 的 Parse 在 try 外，格式错会炸主线程）。

**实测（2026-11-25）**：手机热点组网（电脑 wlp0s20f3=192.168.43.5、头显
wlan0=192.168.43.11），头显填 192.168.43.5 点连接 → 状态"已连接"、**72 Hz**。
注意：72Hz=头显渲染帧率（Update 每帧采集一次），不是 bug；M1 的"100Hz"是
PC 端 bridge 发布频率。访客 WiFi（Visitors 5.x 网段）开了 AP 隔离，客户端
互访全断——组网必须用手机热点或主网络。

**新增脚本坑**：新 .cs 首次批处理构建时 .meta 未生成，首轮编译会报
CS0246"找不到类型"但仍 EXIT=0 出 APK（静默错包！）——加脚本后必须构建两次、
或以 errcount=0 + APK 时间戳为准才算数。

### 2026-11-25 补记：M2.5 全链路闭环 ✅（自制 APK 位姿流驱动仿真）

里程碑：自制客户端 `com.yianglei.vrteleop` 连接后，位姿流 → PC Service →
a3_xrt_bridge → a3_streaming_teleop → RViz 双臂实时跟随，验证通过。

**关键根因（排查链值得记住）**：
1. 症状：头显显示"已连接 + 72Hz"，但 RViz 不动。72Hz 只是**主线程采集帧率**
   （A3NetBridge.SampleRate 数的是 `SendTrackingData==true && State==WORKING` 的帧），
   不代表 socket 实发成功——显示有频率≠数据在流，这个指标有误导性。
2. 硬证据：`ss -ti state established '( sport = :63901 )'` 看 bytes_received，
   卡在 75~135 字节（只有心跳包）→ 数据根本没出头显。
3. **对照实验**（最有力一刀）：头显里装着的官方客户端 com.xrobotoolkit.client
   连同一 PC Service → 机器人动。环境/桥/PC Service 全排除，问题锁死自制 APK 内部。
4. **诊断窗定位**：给官方 TcpHandler 加只读诊断属性（DiagConnectInited/
   DiagQueueCount/DiagSendTracking/DiagSendThreadAlive），状态栏实时显示。
   读出 `inited=0 q=2 send=1 th=1`：采集在产（队列积压满）、线程活着、开关开着，
   唯一缺的是握手包。
5. **根因**：TcpHandler.ConnectCallback 仅在 `_deviceSN` 非空时调 ConnectInit
   （发 CONNECT/VERSION 握手包）；VERSION 发出后才置 `_connectInited=true`，
   而 tracking 发送分支有 `if (_connectInited && SendTrackingData)` 门槛。
   SN 来自 `PXR_Enterprise.StateGetDeviceInfo(EQUIPMENT_SN)`——该**企业接口在
   调试签名包上返回空字符串**（官方包能拿到真值）。SN 空 → 握手跳过 →
   inited 恒 0 → tracking 永被拦截。
6. 修复（v13k）：SN 为空时用 `SystemInfo.deviceUniqueIdentifier` 兜底
   （PC Service 认"VERSION 包到达"这个行为，不校验 SN 内容）。
   附带：HandTracking 开关保持 false（无体感追踪器；开着曾疑似中断采集）。

**排障方法论沉淀**：
- TCP 链路存否数据：`ss -ti` 的 bytes_received 两次采样对比，比猜快得多。
- 黑盒内部状态可视化：加"诊断串"直接显示在头显 UI 上，用户念数字即可远程定位。
- 先做对照实验（官方包/官方 APK）再怀疑自己的代码。
- Unity 跨线程坑汇总：①后台线程 new GameObject 崩（Dispatcher 单例首次 Instance）
  ②线程池 SemaphoreSlim 自锁 ③企业 API 调试签名返空——都在本项目踩实了。

诊断属性与状态栏诊断串保留至 M3 前再清理。
