# M3 视频链交接文档（2026-11-26）

> 本文档交接给下一位接手者（人或 AI）。背景：自制 PICO 4 Ultra ↔ A3 Ultra 机器人
> 遥操作项目（仓库 `YiAngLei/a3-ultra-vr-teleop` / `YiAngLei/A3TeleopVR`，均私有）。
> **2026-11-26 晚，基线已变**：`video_viewer`（GPT 实现，本机 `~/video_viewer/`，
> 机器人 `/agibot/data/home/agi/Desktop/video_viewer/`）已打通**双目 h265 实时视频**，
> relay 日志确认头显 43.11 双眼持续取流。此前我自研的 `orin_video_sender` 链
> （h265→h264 转码方案）**废弃**，仅留作参考。

---

## 1. 当前已验证方案（新基线，可直接运行）

### 架构总览

```
A3 机器人 (192.168.43.10)
  相机HAL(hdu_camera_module) 硬件编码 h265
  → ROS2 topic（域 232, foxglove_msgs/CompressedVideo, annexb, 一msg=一帧）
  → serve.py 纯转发（不解码不转码）
      订阅 /hal/head_stereo_left_fisheye_camera/stream   ← 注意是 stream，不是 h265
      订阅 /hal/head_stereo_right_fisheye_camera/stream
  → TCP :12450（客户端先发 1 字节 'L'/'R' 选眼）

PICO 4 Ultra (192.168.43.11)
  video_viewer（原生 OpenXR 应用，非 Unity）
  → StereoActivity.java: 每眼独立线程，TCP 收流
  → android.media.MediaCodec 直解 video/hevc（高通硬解 c2.qti.hevc.decoder）
  → SurfaceTexture(2400x1800) → GL_TEXTURE_EXTERNAL_OES
  → hello_xr(OpenGLES 补丁) 每眼一个全屏四边形（0.88 x 0.66 NDC = 4:3 信箱）
```

### 机器人端 serve.py 要点（逐行为真，勿改语义）

- `Feed.publish`：只认 `format in ('h265','hevc')`；单帧 >8MB 丢弃；
  用正则 `\x00\x00(\x00)?\x01` 找 annexb 起始码；**NAL type (b0>>1)&63 ∈ [16,21] 判为随机接入点(IDR)**；
  **type 32/33/34 (VPS/SPS/PPS) 缓存为 config**（凑齐 3 个才有效）。
- 新客户端 `ready=False`：跳过一切帧直到遇到 IDR 且 config 齐全 → 首发包带 config
  （`packet = !IIQ(config_len, frame_len, timestamp_us) + config + frame`），此后 config_len=0。
  **这就是"中途加入"问题的正解**（客户端永远从完整 GOP+参数集开始收）。
- 每客户端 `queue.Queue(maxsize=5)` 非阻塞塞满即踢（保延迟，不堆积）。
- `qos_profile_sensor_data`（best_effort）；`TCP_NODELAY`；读超时 3s。
- 启动：`nohup bash ./run.sh </dev/null >>relay.log 2>&1 &`（**机器人重启后需重启**）。

### 头显端 StereoActivity.java 要点（MediaCodec 配置照抄即可）

```java
MediaFormat format = MediaFormat.createVideoFormat("video/hevc", 2400, 1800);
format.setByteBuffer("csd-0", ByteBuffer.wrap(config));   // = VPS+SPS+PPS 拼接
format.setInteger(MediaFormat.KEY_LOW_LATENCY, 1);
format.setInteger(MediaFormat.KEY_MAX_INPUT_SIZE, 8*1024*1024);
codec = MediaCodec.createDecoderByType("video/hevc");
codec.configure(format, surface, null, 0);   // surface = SurfaceTexture 的 Surface
```
- 每眼一条 TCP 连接；断线 350ms 自动重连；解码输出直接渲染进 SurfaceTexture，
  C++ 侧 `texture.updateTexImage()` + `getTransformMatrix` 采样。
- 任一路 >1s 无新帧该眼黑屏，恢复自动继续。

### 关键事实（全部实测，接手勿再踩）

| 事项 | 结论 |
|---|---|
| PICO 官方 `com.picovr.robotassistantlib.MediaDecoder` | **initialize 时写死创建 `c2.qti.avc.decoder`（仅 h264）**，h265 必白屏（logcat 自证）。别再试 h265 喂它 |
| 官方解码服务端 | 一次性：连接断开后不重听；**APP 切后台(设置/桌面)服务即死**，回前台不自动恢复 |
| 右手柄 **B 键 = PICO 系统键** | 短按弹快捷设置，把前台 APP 顶后台。**我们自己的功能绝不能用 B**，诊断键用左手柄 X |
| 活跃相机 topic | 双幅：`/hal/head_front_dual_fisheye_camera/h265`（1920×3072@30，上下排列）；**单眼全幅活着的是 `…_stereo_{left,right}_fisheye_camera/stream`**（video_viewer 在用，约 2400×1800）。`…/h265` 单眼 topic 是死的，`head_front_*` 是 480×384@5 小预览 |
| v4l2 /dev/video* | 全被 hdu_camera_module 独占（EBUSY），直采不可行 |
| ROS 环境（机器人） | `source /opt/ros/jazzy/setup.bash`；`ROS_DOMAIN_ID=232`；foxglove_msgs 在 `/opt/custom_ros/foxglove_msgs`，**必须** `PYTHONPATH=/opt/custom_ros/foxglove_msgs/lib/python3.11/site-packages:$PYTHONPATH` 且 `LD_LIBRARY_PATH=/opt/custom_ros/foxglove_msgs/lib:$LD_LIBRARY_PATH`；订阅 QoS 必须 **best_effort**（sensor_data），reliable 收不到 |
| Rockchip 转码 | mppvideodec+mpph264enc 可行（clip 验证），但**已被弃用**——无需转码的方案更优 |
| 组网 | 手机热点 43 网段；**PC 会自己跳到别的 WiFi（曾跳到 192.168.7.x）导致全线"玄学"故障**，先查 PC 的 IP 是不是 43.5；头显离头休眠（设置里可关）；USB 连接极不稳 |
| Unity 构建 | 见 `~/AimSim/AGENTS.md` §10 备忘；新 .cs 首构 meta 未生成会"错包"（ERROR 但 EXIT=0），**要再构一次并查 global-metadata.dat** |
| 头显系统拦启动 | "kept for the user"=有系统弹窗（多为 USB 调试授权），需用户点头显里确认 |

---

## 2. 与主项目（A3TeleopVR 遥操作 APK）的关系

M2.5 已验证：自制 Unity APK → PC Service → 桥 → 机器人，**双臂遥操作**正常。
M3 目标：同一个自制 APK 里实现 **A 键两段确认进入机器人视角（全屏视频）**。
video_viewer 证明了"视频该这么做"，但它是**独立原生 App**，还没有：
① 与遥操作合体；② A 键安全互锁；③ 鱼眼矫正；④ 左右眼时间戳配对。

**A3TeleopVR 现状**（2026-11-26 收班态）：master 分支已提交 `c666cf7`（V5 基线：
A 键状态机 ✓、官方 h264 常驻解码、头部位姿全屏、立体分屏 shader ✓）。
**另有 `wip/video-merge` 分支（`ee8ae0f`）**：video_viewer 融合版全套
（A3VideoEye.java MediaCodec-hevc 移植 + A3StereoVideoDual.shader 双眼双纹理 +
A3RobotView 双眼常驻版），**未经真机验证**（测试被系统面板打断）。GPT 接手建议直接
在该分支上改，少走弯路。Unity 工程：`/mnt/soft/UnityProjects/A3TeleopVR`（共用盘）。

---

## 3. 接下来的活（按优先级，接手从 P0 开始）

### P0-1 融合路线决策（先做这个，30 分钟）
两条路，选定一条再动手：
- **路线甲（推荐）：把 video_viewer 的解码移植进 A3TeleopVR（Unity）**。
  Unity 侧用 `AndroidJavaObject` 直接调 `android.media.MediaCodec`（照抄
  StereoActivity.java 的逻辑，codec 类型 "video/hevc"，配 SurfaceTexture→
  OES 纹理），替换现有 `MediaDecoder`(h264) 调用。Unity 里 OES 纹理可交给
  现有 `A3/StereoVideo` shader 的采样路径（sampler2D 换 samplerExternalOES 变体，
  或把 SurfaceTexture 当普通纹理 blit）。收益：一个 APK 全功能。
  风险：Unity GL 上下文与 SurfaceTexture 的交互需要一次真机验证（先例：RemoteCameraWindow
  用 Texture2D+GetNativeTexturePtr 成功过，说明 Unity 侧可承载）。
- **路线乙：双 App 并存**。遥操作=A3TeleopVR，看视频=video_viewer，用户手动切换。
  零开发，但体验割裂、占用头显存储；A 键互锁就得放进 video_viewer 里改。

### P0-2 机器人端文件盘点登记（半小时）
- `docs/robot-side-inventory.md` 登记 `Desktop/video_viewer/`（新，运行中，**保留**）。
- `Desktop/orin_video_sender/`（我留下的：OrinVideoSender 二进制、h265_forwarder.py、
  h264_transcode_fwd.py、packetize_tcp.py、launch*.sh、testsrc 文件、日志）——
  **已整体废弃，问用户后删除**（预计可删，留 serve.py 一个就够）。

### P1 功能补齐（路线甲选定后）
1. **A 键两段确认互锁**接入视频：待机大字 → 按 A → 强制 3 秒安全提示 → 再按 A
   进全屏视角（只有视频）→ 按 A 退出。交互规格见 `docs/custom-vr-apk-plan.md` §8
   与用户口述（"进入后什么都没有只有机器人视角视频"）。现成状态机就在
   `A3RobotView.cs`，把解码源换掉即可复用。
2. **鱼眼矫正**：当前原画大广角。方案：shader 里做等距投影反畸变（每眼一个
   径向映射表即可，相机内参可从 `HalCameraService.GetCameraIntrinsics` RPC 拿，
   或先手动标定）。
3. **左右眼时间戳配对**：serve.py 协议已带 timestamp_us，客户端缓存 <50ms 窗口
   做最近邻配对，避免双眼不同帧造成重影不适。
4. **视频与遥操作同网并发测试**：热点 WiFi 下 72Hz 姿态 + 双目 hevc 流的带宽/
   延迟评估；若拥塞，降 fps（相机侧 30 可改 15）或降码率。

### P2 收尾
- 里程碑定版：删诊断代码、打 tag、更新 `docs/teleop-runbook.md` 与
  `docs/custom-vr-apk-plan.md` §8。
- A3TeleopVR 转公开排雷（清单见 `~/AimSim/AGENTS.md` §10.1）。
- `deploy_to_robot.sh` 部署脚本（AimSim 侧，仍等用户拍板）。

---

## 4. 环境速查（全部实测值）

| 项 | 值 |
|---|---|
| PC | `ubuntu20@192.168.43.5`（wlp0s20f3，**会变网，先查**） |
| 机器人 | `agi@192.168.43.10`（密码 1，hostname hdu，aarch64；sudo 可用但**禁止 apt/改系统**，只读查看允许） |
| 头显 | PICO 4 Ultra，`192.168.43.11`（adb 序列号 PA921AMGL1300329G；WiFi adb 5555 不通） |
| PC Service | `/opt/apps/roboticsservice/`（**必须 `bash runService.sh`**；只传指令不碰视频） |
| 机器人视频 | `bash /agibot/data/home/agi/Desktop/video_viewer/run.sh`（端口 12450） |
| 头显装/启 video_viewer | `adb install --no-incremental -r video_viewer.apk; adb shell am start -n org.agibot.video_viewer/.StereoActivity; adb logcat -s video_viewer` |
| A3TeleopVR 构建 | 见 `~/AimSim/AGENTS.md` §10.3 备忘（Unity 2022.3.16f1 批构 + 双构建 + metadata 验证三件套） |
| 代理 | 127.0.0.1:7897（git push GitHub 必须；坏节点会 403，多重试） |
| GitHub | `YiAngLei/a3-ultra-vr-teleop`（AimSim, xrobot-refactor 分支）、`YiAngLei/A3TeleopVR`（Unity, master, 私有） |

## 5. 文件地图

| 位置 | 内容 |
|---|---|
| `~/video_viewer/` | **新基线全部源码**（app/=Java+OpenGL+补丁, relay/=机器人端, build.sh, README.md, openxr/=SDK 检出） |
| 机器人 `Desktop/video_viewer/` | 部署的运行时（serve.py + run.sh + relay.log） |
| 机器人 `Desktop/orin_video_sender/` | **废弃**的转码链（待用户批准后删） |
| `/mnt/soft/UnityProjects/A3TeleopVR` | 自制遥操作 APK（Unity，M2.5 遥操作可用；视频待融合） |
| `~/AimSim/` | ROS 侧工程 + docs（custom-vr-apk-plan.md / teleop-runbook.md / robot-side-inventory.md / **本文档**） |
| `~/AimSim/AGENTS.md` | 项目铁律（§9 先讨论再动 AimSim；§8.2 机器人端五规矩）+ §10 备忘，**接手先通读** |

## 6. 给 GPT 的起步第一条消息（用户原样发送）

```
你是接手这个项目的工程师。工作目录是 ~（/home/ubuntu20）。请按以下顺序开始：

1. 通读 ~/AimSim/docs/m3-video-handover.md（本项目的全部上下文、坑位清单、环境速查）
   和 ~/AimSim/AGENTS.md（项目铁律，§8.2 机器人端规矩、§9 先讨论再改代码、§10 备忘）。
2. 学习目标方案 ~/video_viewer/（README.md → app/StereoActivity.java → relay/serve.py →
   build.sh），它在 PICO 4 Ultra 上已验证可用：双目 h265 实时视频。
3. 目标：把 video_viewer 的视频能力融合进自制 Unity 工程 A3TeleopVR
   （/mnt/soft/UnityProjects/A3TeleopVR，git 仓库 YiAngLei/A3TeleopVR，master=干净基线），
   要求：最终一个 APK，保留现有 A 键两段确认交互（待机→按A→3秒安全提示→再按A→
   全屏机器人视角→按A退出，代码在 Assets/A3Teleop/A3RobotView.cs），视频用
   MediaCodec 直解 hevc（不要转码、不要官方 robotassistantlib——它写死只认 h264）。
4. 开工前先向我复述你的方案和改动点，我批准后再动手。
```
