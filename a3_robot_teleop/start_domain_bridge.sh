#!/bin/bash
# 域间桥接一键启动：把 MC 的手臂/行走话题在机器人系统域(232)与对外域(42)之间转发。
#   state    只起状态方向 232->42（只读，安全，随时可开）
#   command  只起指令方向 42->232（主机指令从此可达 MC，确认 MOTION 状态后再开！）
#   loco     摇杆行走链：/teleop/locomotion_cmd 42->232 + locomotion_relay 转 MC
#   all      全部
# 用法: bash start_domain_bridge.sh [state|command|loco|all]   （默认 state）
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$HERE/logs"
mkdir -p "$LOGS"
MODE="${1:-state}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
set +u  # jazzy 的 setup.bash 引用未定义变量，set -u 下会炸（COLCON_TRACE）
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
set -u
SYS_DOMAIN=232    # MC 等系统节点所在域
EXT_DOMAIN=42     # 与主机 PC 互通的域
STATE_TOPIC=/motion/control/arm_joint_state
CMD_TOPIC=/motion/control/arm_joint_command
MSG_TYPE=sensor_msgs/msg/JointState
STATE_PORT=53200
CMD_PORT=53201
LOC_TOPIC=/teleop/locomotion_cmd
LOC_MSG=std_msgs/msg/Float32MultiArray
LOC_PORT=53202
PLUGIN_PROTO_SETUP=/agibot/data/home/agi/Desktop/agibot_a3_Ultra_aimdk-dev3.2/prebuilt/ros2_plugin_proto_aarch64/share/ros2_plugin_proto/local_setup.bash

start_one() {  # name domain script port topic qos mtype
    local name="$1" domain="$2" script="$3" port="$4" topic="$5" qos="${6:-reliable}" mtype="${7:-$MSG_TYPE}"
    if [ -f "$LOGS/$name.pid" ] && kill -0 "$(cat "$LOGS/$name.pid")" 2>/dev/null; then
        echo "[skip] $name 已在运行 (pid $(cat "$LOGS/$name.pid"))"
        return
    fi
    ROS_DOMAIN_ID=$domain nohup python3 "$HERE/$script" \
        --topic "$topic" --type "$mtype" --port "$port" --qos "$qos" \
        >"$LOGS/$name.log" 2>&1 &
    echo $! >"$LOGS/$name.pid"
    echo "[start] $name pid=$! (ROS_DOMAIN_ID=$domain $topic type=$mtype port=$port qos=$qos)"
}

start_relay() {  # 摇杆行走转发：locomotion_relay（域 232，需 ros2_plugin_proto）
    local name="locomotion_relay"
    if [ -f "$LOGS/$name.pid" ] && kill -0 "$(cat "$LOGS/$name.pid")" 2>/dev/null; then
        echo "[skip] $name 已在运行 (pid $(cat "$LOGS/$name.pid"))"
        return
    fi
    ROS_DOMAIN_ID=$SYS_DOMAIN nohup bash -c \
        "source /opt/ros/jazzy/setup.bash; [ -f '$PLUGIN_PROTO_SETUP' ] && source '$PLUGIN_PROTO_SETUP'; exec python3 '$HERE/locomotion_relay.py'" \
        >"$LOGS/$name.log" 2>&1 &
    echo $! >"$LOGS/$name.pid"
    echo "[start] $name pid=$! (ROS_DOMAIN_ID=$SYS_DOMAIN $LOC_TOPIC -> MC 行走)"
}

case "$MODE" in
    state)
        start_one pipe_state_sub "$SYS_DOMAIN" dds_domain_pipe_sub.py "$STATE_PORT" "$STATE_TOPIC" best_effort
        start_one pipe_state_pub "$EXT_DOMAIN" dds_domain_pipe_pub.py "$STATE_PORT" "$STATE_TOPIC"
        ;;
    command)
        start_one pipe_cmd_sub "$EXT_DOMAIN" dds_domain_pipe_sub.py "$CMD_PORT" "$CMD_TOPIC" reliable
        start_one pipe_cmd_pub "$SYS_DOMAIN" dds_domain_pipe_pub.py "$CMD_PORT" "$CMD_TOPIC"
        ;;
    loco)
        start_one pipe_loco_sub "$EXT_DOMAIN" dds_domain_pipe_sub.py "$LOC_PORT" "$LOC_TOPIC" reliable "$LOC_MSG"
        start_one pipe_loco_pub "$SYS_DOMAIN" dds_domain_pipe_pub.py "$LOC_PORT" "$LOC_TOPIC" reliable "$LOC_MSG"
        start_relay
        ;;
    all)
        bash "$0" state
        bash "$0" command
        bash "$0" loco
        ;;
    *)
        echo "用法: bash $0 [state|command|loco|all]" >&2
        exit 1
        ;;
esac
sleep 1
echo "--- 日志（最新 3 行/个）---"
for f in "$LOGS"/pipe_*.log; do
    [ -f "$f" ] && { echo "## $f"; tail -3 "$f"; }
done
echo "验证：ROS_DOMAIN_ID=$EXT_DOMAIN ros2 topic hz $STATE_TOPIC"
