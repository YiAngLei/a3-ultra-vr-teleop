#!/usr/bin/bash
# A3_ULTRA_T2D0 MoveIt 演示环境。用法: source 本文件
# 要点：
#   1. 加载 ROS 2 Jazzy
#   2. 把手工搭建的 ament 前缀加入 AMENT_PREFIX_PATH，使 a3_moveit_config
#      （launch/配置/参数生成器）可被解析；网格已改用 file:// 绝对路径，
#      不再需要 robot_descriptions 资源包
#   3. /usr/bin 前置，避免 conda 的 python3.14 抢在 ROS 的 python3.12 前面
source /opt/ros/jazzy/setup.bash
DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${DEMO_DIR}/prefix:${AMENT_PREFIX_PATH}"
export PATH=/usr/bin:$PATH
echo "[a3_moveit_demo] 环境就绪，运行: ros2 launch a3_moveit_config demo.launch.py"
