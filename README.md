# A3 Ultra VR Teleop

**Streaming VR teleoperation of the AgiBot (Zhiyuan) A3 Ultra dual arms** — PICO headset (XRoboToolkit official app) → PC Service SDK → world-frame delta mapping (same math as the XRoboToolkit dual-UR5 reference) → MoveIt 2 IK → simulated or real robot.

[English](README.md) | [中文](README_CN.md)

## Features

- **Continuous servo teleop**: hold the grip to follow, release to freeze — no discrete "plan per click".
- **Official-grade data source**: controller poses come from the XRoboToolkit PC Service SDK — **room-anchored** (turning your head does not corrupt the stream), the same pipeline as XRoboToolkit's mature dual-UR5 sample.
- **World-frame delta mapping**: position delta `(p_w − p_w0) × scale` and world-frame rotation delta applied to the end-effector reference — no per-capture body-frame tricks, no mirroring surprises. The only calibration constant is a fixed axis-conversion matrix (`world_rotation`, default = the official `R_HEADSET_TO_WORLD`).
- **MoveIt 2 as the IK engine** (KDL, seeded with current state), with joint step limiting (slow/fast gear), workspace-sphere clamping, and IK-failure hold.
- **Two backends**: `sim_jtc` (ros2_control mock hardware + RViz) and `mc_arm` (AgiBot official MC interface, 100 Hz stream, speed limits, low-pass, joint-limit clamping).
- **Live debugging in RViz**: per-arm target triad TF (`teleop/<side>_target`, the mocap-body equivalent), raw controller TF, "command vs achieved" deviation monitors.
- **Headless regression**: `fake_pico_publisher.py` replays a 40 s scripted scenario (no VR device needed).

## Architecture

```
PICO headset (XRoboToolkit app, Head+Controller, Send ON)
   --WiFi--> XRoboToolkit PC Service (/opt/apps/roboticsservice)
   --SDK--> a3_xrt_bridge --/pico_*--> a3_streaming_teleop
   (room-anchored poses)                |  world-frame delta mapping
                                        v
                              MoveIt 2 IK (KDL, seeded)
                                        v
                 <side>_arm_controller/joint_trajectory
                     |                              |
        ros2_control (mock, RViz)      a3_mc_controller (real robot MC)
```

## Repository layout

| Directory | What it is |
|---|---|
| `a3_teleop_vr/` | Streaming teleop node, `a3_xrt_bridge` (XRoboToolkit SDK bridge), `a3_teleop_bridge` (legacy OpenArmX-compatible UDP bridge, reference/fallback only), launch files, debug tools, full command reference (see its `README.md`) |
| `a3_moveit_demo/` | MoveIt config package (OMPL planning, controllers, RViz), model derivation script |
| `a3_robot_teleop/` | Robot-side helpers (domain bridge, state echo, e-stop hold) |
| `AGENTS.md` | Workspace notes for AI coding agents (includes the standing rule: *discuss before changing code*) |

## Quick start

Full, copy-paste-ready commands live in **`a3_teleop_vr/README.md` § 命令速查**. Short version:

```bash
# once: derive model files from the official AgiBot model
export A3_MODEL_SRC=<dir with urdf/ and meshes/>
bash a3_moveit_demo/scripts/setup_robot_model.sh

# every session
bash /opt/apps/roboticsservice/runService.sh                    # 1) PC Service
# 2) headset: XRoboToolkit app -> PC Service IP -> Head+Controller ON -> Send ON

# 3) sim stack + bridge + RViz
source a3_teleop_vr/setup_teleop_env.bash
export ROS_DOMAIN_ID=76
ros2 launch "$PWD"/a3_teleop_vr/launch/teleop_vr.launch.py bridge:=true
```

Hand-feel tuning: `pos_scale:=0.5`–`0.7` makes the arm move less than your hand (default `1.0`).

## Notes & credits

- Built on [XRoboToolkit](https://github.com/XR-Robotics) (MIT) — the mapping math follows their dual-UR5 sample line by line.
- The legacy OpenArmX protocol reference ([openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr), CC BY-NC-SA 4.0) was used only as a datagram-format reference; `a3_teleop_bridge` is a clean-room compatible implementation kept for comparison.
- AgiBot model files (URDF/STL/SRDF) are **not** included (AgiBot IP); obtain them via the official AimDK channel and run the derivation script.
- Branches: `main` (current) · `xrobot-refactor`; tag `backup/pre-xrobot-refactor` marks the pre-rewrite baseline.

## License

Code in this repository: Apache License 2.0 (see `a3_teleop_vr/LICENSE`).
