#!/usr/bin/env bash
# 启动机械臂全套服务（MoveIt Servo + Stream API）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

START_SERVO_DELAY="${START_SERVO_DELAY:-8}"

echo "=========================================="
echo " 启动 Dummy 机械臂服务"
echo "=========================================="

# 已在运行则提示
if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  echo "⚠️  会话 '${TMUX_SESSION}' 已在运行"
  echo "   查看: bash scripts/home/status.sh"
  echo "   进入: tmux attach -t ${TMUX_SESSION}"
  echo "   停止: bash scripts/home/stop.sh"
  exit 0
fi

# 创建 tmux 会话
tmux new-session -d -s "${TMUX_SESSION}" -n moveit

# 窗格 0: MoveIt Servo
tmux send-keys -t "${TMUX_SESSION}:moveit" \
  "source '${SCRIPT_DIR}/env.sh' && cd '${WS_DIR}' && echo '🚀 启动 MoveIt Servo...' && ros2 launch dummy_moveit_config servo_streaming.launch.py" C-m

echo "⏳ 等待 MoveIt 启动 (${START_SERVO_DELAY}s)..."
sleep "${START_SERVO_DELAY}"

# 激活 Servo
echo "🔧 激活 Servo 节点..."
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger >/dev/null 2>&1 || {
  echo "⚠️  Servo 激活失败，8 秒后会自动重试一次..."
  sleep 8
  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger >/dev/null 2>&1 || true
}

# 避免零位奇点：轻微 joint jog
ros2 topic pub --once /servo_node/delta_joint_cmds control_msgs/msg/JointJog \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, joint_names: ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6'], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}" >/dev/null 2>&1 || true

# 窗格 1: Stream API
tmux new-window -t "${TMUX_SESSION}" -n stream_api
tmux send-keys -t "${TMUX_SESSION}:stream_api" \
  "source '${SCRIPT_DIR}/env.sh' && cd '${WS_DIR}' && echo '🌐 启动 Stream API :${STREAM_API_PORT} ...' && python3 dummy_server/server/stream_api_server.py" C-m

sleep 3

echo ""
echo "=========================================="
echo " ✅ 服务已启动"
echo ""
echo "  tmux 会话: ${TMUX_SESSION}"
echo "  进入查看:  tmux attach -t ${TMUX_SESSION}"
echo "  退出 tmux: Ctrl+B 然后 D（服务继续后台跑）"
echo ""
echo "  Stream API: http://0.0.0.0:${STREAM_API_PORT}/api/status"
echo "  本机测试:   curl http://127.0.0.1:${STREAM_API_PORT}/api/status"
if command -v tailscale >/dev/null 2>&1; then
  TS_IP="$(tailscale ip -4 2>/dev/null || true)"
  if [[ -n "${TS_IP}" ]]; then
    echo "  远程访问:   curl http://${TS_IP}:${STREAM_API_PORT}/api/status"
  fi
fi
echo ""
echo "  停止服务:  bash scripts/home/stop.sh"
echo "  查看状态:  bash scripts/home/status.sh"
echo "=========================================="
