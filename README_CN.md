# A3 Ultra VR 遥操作

**智元（AgiBot）A3 Ultra 双臂流式 VR 遥操作** —— PICO 眼镜（XRoboToolkit 官方 App）→ PC Service SDK → 世界系增量映射（与 XRoboToolkit 双 UR5 官方案例同款数学）→ MoveIt 2 IK → 仿真或真机执行。

[English](README.md) | [中文](README_CN.md)

## 特性

- **连续伺服遥操作**：握住 grip 持续跟随、松开即冻结，没有"按一次规划一次"的离散感。
- **官方级数据源**：手柄位姿来自 XRoboToolkit PC Service SDK——**房间锚定**（转头不会污染数据流），与 XRoboToolkit 成熟的双 UR5 示例同一链路。
- **世界系增量映射**：位置增量 `(p_w − p_w0) × scale`、世界系旋转增量复合到末端参考点——不依赖捕获瞬间的体坐标系，没有镜像意外。唯一标定量是固定轴转换矩阵（`world_rotation`，默认即官方 `R_HEADSET_TO_WORLD`）。
- **MoveIt 2 作 IK 引擎**（KDL，当前状态作种子），带关节步进限幅（慢/快两档）、工作空间球限幅、IK 失败保持。
- **两种输出后端**：`sim_jtc`（ros2_control 仿真 + RViz）与 `mc_arm`（智元官方 MC 接口，100Hz 连续流、限速、低通、限位钳制）。
- **RViz 实时调试**：每臂目标坐标架 TF（`teleop/<side>_target`，等价官方案例的 mocap 体）、手柄原始坐标架、"指令 vs 实测"偏差监视器。
- **无头显回归**：`fake_pico_publisher.py` 回放 40 秒自动化剧本（无需 VR 设备）。

## 架构

```
PICO 眼镜（XRoboToolkit App，勾 Head+Controller，Send ON）
   --WiFi--> XRoboToolkit PC Service（/opt/apps/roboticsservice）
   --SDK--> a3_xrt_bridge --/pico_*--> a3_streaming_teleop
   （房间锚定位姿）                       |  世界系增量映射
                                        v
                              MoveIt 2 IK（KDL，种子=当前状态）
                                        v
                 <side>_arm_controller/joint_trajectory
                     |                              |
        ros2_control（仿真 + RViz）    a3_mc_controller（真机 MC 接口）
```

## 仓库结构

| 目录 | 内容 |
|---|---|
| `a3_teleop_vr/` | 流式遥操作节点、`a3_xrt_bridge`（XRoboToolkit SDK 桥）、`a3_teleop_bridge`（旧 OpenArmX 兼容 UDP 桥，仅对照/回退）、launch、调试工具、完整命令速查（见其 `README.md`） |
| `a3_moveit_demo/` | MoveIt 配置包（OMPL 规划、控制器、RViz）、模型派生脚本 |
| `a3_robot_teleop/` | 机器人端配套（域桥、状态回显、急停保持） |
| `AGENTS.md` | 给 AI 编码代理的工作区说明（含立规矩：*动代码前必须先讨论*） |

## 快速开始

可直接复制粘贴的完整命令见 **`a3_teleop_vr/README.md`「命令速查」**。简版：

```bash
# 一次性：从智元官方模型目录派生模型文件
export A3_MODEL_SRC=<含 urdf/ 与 meshes/ 的目录>
bash a3_moveit_demo/scripts/setup_robot_model.sh

# 每次使用
bash /opt/apps/roboticsservice/runService.sh                    # 1) PC Service
# 2) 眼镜：XRoboToolkit App → 填 PC Service IP → 勾 Head+Controller → Send ON

# 3) 仿真栈 + 桥 + RViz
source a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=76
ros2 launch "$PWD"/a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

手感调节：`pos_scale:=0.5`–`0.7` 让末端位移小于手部位移（默认 `1.0`）。

## 说明与致谢

- 遥操作交互范式与映射数学对齐 [XRoboToolkit](https://github.com/XR-Robotics)（MIT）双 UR5 官方案例。
- OpenArmX 协议参考（[openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr)，CC BY-NC-SA 4.0）仅作报文格式参考；`a3_teleop_bridge` 为其净室兼容实现，保留作对照。
- 智元模型文件（URDF/STL/SRDF）**不含**在本仓库（版权归智元），请从官方 AimDK 渠道获取后运行派生脚本。
- 分支：`main`（当前）· `xrobot-refactor`；标签 `backup/pre-xrobot-refactor` 为重构前基线。

## 许可证

本仓库代码：Apache License 2.0（见 `a3_teleop_vr/LICENSE`）。
