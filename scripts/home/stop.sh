#!/usr/bin/env bash
# 停止机械臂服务

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  tmux kill-session -t "${TMUX_SESSION}"
  echo "✅ 已停止 tmux 会话: ${TMUX_SESSION}"
else
  echo "ℹ️  没有运行中的会话: ${TMUX_SESSION}"
fi

# 清理可能残留的 stream_api 进程
pkill -f "stream_api_server.py" 2>/dev/null && echo "✅ 已清理 stream_api 进程" || true

echo "完成。"
