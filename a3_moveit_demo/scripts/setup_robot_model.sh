#!/bin/bash
# 机器人模型获取 + 派生文件生成（一次性设置脚本）。
#
# 本仓库不含智元机器人模型（URDF/STL/SRDF 为智元版权资产，需用户自备）。
# 本脚本从【本机智元官方安装】读取模型，生成 MoveIt 仿真所需的全部派生文件：
#   src/a3_moveit_config/config/a3_ultra_t2d0.urdf
#   src/a3_moveit_config/config/a3_ultra_t2d0.srdf
#   src/a3_moveit_config/config/joint_limits.yaml
#   src/a3_moveit_config/urdf/a3_ultra_t2d0_ros2control.urdf
#   src/a3_moveit_config/config/a3_plan_demo_params.yaml
#
# 用法：
#   export A3_MODEL_SRC=<含 urdf/ 与 meshes/ 的目录>   # 如 AimSim 工作区的 a3_ultra_isaaclab
#   bash scripts/setup_robot_model.sh
# 未设置 A3_MODEL_SRC 时，默认猜测 <本仓库上级>/a3_ultra_isaaclab。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="$(dirname "${SCRIPT_DIR}")"
GEN_DIR="${DEMO_DIR}/src/a3_moveit_config/scripts"   # 生成器所在目录

MODEL_SRC="${A3_MODEL_SRC:-$(dirname "${DEMO_DIR}")/a3_ultra_isaaclab}"
if [ ! -f "${MODEL_SRC}/urdf/model_planning.urdf" ]; then
    echo "未找到机器人模型：${MODEL_SRC}/urdf/model_planning.urdf" >&2
    echo "请设置 A3_MODEL_SRC 指向本机智元官方安装中包含 urdf/ 与 meshes/ 的目录" >&2
    exit 1
fi
echo "[setup] 模型来源: ${MODEL_SRC}"

set +u  # ROS 的 setup.bash 会引用未绑定变量，不能与 nounset 共存
source "${DEMO_DIR}/setup_demo_env.bash" >/dev/null
set -u

/usr/bin/python3.12 "${GEN_DIR}/gen_derived_files.py" --model-src "${MODEL_SRC}"
# gen_params_file 需要 ament 能找到 a3_moveit_config（环境已在上一步就绪）
/usr/bin/python3.12 "${GEN_DIR}/gen_params_file.py"

echo "[setup] 完成。启动仿真: source setup_demo_env.bash && ros2 launch a3_moveit_config demo.launch.py"
