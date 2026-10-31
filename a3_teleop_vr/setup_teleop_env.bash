#!/bin/bash
# A3 流式 VR 遥操作环境准备：source 后运行
#   ros2 launch <repo>/a3_teleop_vr/launch/teleop_vr.launch.py
# 前提：Pico 报文源已在运行——OpenArmX bridge
# （ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node）
# 或本工程自带的 a3_teleop_bridge（launch 参数 bridge:=true）。
TELEOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${TELEOP_DIR}/../a3_moveit_demo/setup_demo_env.bash"
export PATH=/usr/bin:$PATH  # 防 conda python 抢位（同 a3_moveit_demo 的坑）
echo "[a3_teleop_vr] 环境就绪，运行: ros2 launch ${TELEOP_DIR}/launch/teleop_vr.launch.py"
