#!/usr/bin/env bash
# 查看服务状态

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

echo "=========================================="
echo " Dummy 服务状态"
echo "=========================================="

# tmux
if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  echo "✅ tmux 会话 [${TMUX_SESSION}] 运行中"
  tmux list-windows -t "${TMUX_SESSION}" 2>/dev/null | sed 's/^/   /'
else
  echo "❌ tmux 会话 [${TMUX_SESSION}] 未运行"
fi

echo ""

# ROS nodes
if ros2 node list 2>/dev/null | grep -q servo_node; then
  echo "✅ ROS servo_node 已启动"
else
  echo "❌ ROS servo_node 未检测到"
fi

echo ""

# Stream API
if curl -sf "http://127.0.0.1:${STREAM_API_PORT}/api/status" >/dev/null 2>&1; then
  echo "✅ Stream API 响应正常 (:${STREAM_API_PORT})"
  curl -s "http://127.0.0.1:${STREAM_API_PORT}/api/status" | head -c 200
  echo ""
else
  echo "❌ Stream API 无响应 (:${STREAM_API_PORT})"
fi

echo ""

# Tailscale
if command -v tailscale >/dev/null 2>&1; then
  TS_IP="$(tailscale ip -4 2>/dev/null || true)"
  if [[ -n "${TS_IP}" ]]; then
    echo "✅ Tailscale IP: ${TS_IP}"
    echo "   单位远程测试: curl http://${TS_IP}:${STREAM_API_PORT}/api/status"
  else
    echo "⚠️  Tailscale 未连接，运行: sudo tailscale up"
  fi
else
  echo "⚠️  未安装 Tailscale"
fi

echo "=========================================="
