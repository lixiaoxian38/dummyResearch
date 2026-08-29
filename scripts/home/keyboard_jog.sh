#!/usr/bin/env bash
# 键盘按轴 jog（需前台终端；按键 1-6 控制 Joint1-6，R 反转，Q 退出）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

echo "=========================================="
echo " 键盘按轴 Jog"
echo "=========================================="
echo "  1-6 : 对应 Joint1-6 正向"
echo "  R   : 反转方向"
echo "  方向键 / . ; : 笛卡尔（可选）"
echo "  Q   : 退出"
echo "=========================================="
echo ""
echo "确认 Servo 已启动后，本终端保持焦点再按键..."
echo ""

# Jazzy: ensure joint jog mode (0=JOINT_JOG)
ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}" >/dev/null || true
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}" >/dev/null || true

# 兼容未重编译的旧二进制（topic 仍指向 servo_demo_node）
exec ros2 run dummy_moveit_config dummy_servo_keyboard_input --ros-args \
  -r /servo_demo_node/delta_joint_cmds:=/servo_node/delta_joint_cmds \
  -r /servo_demo_node/delta_twist_cmds:=/servo_node/delta_twist_cmds
