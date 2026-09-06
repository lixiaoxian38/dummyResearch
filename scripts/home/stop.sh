#!/usr/bin/env bash
# 停止机械臂服务。默认先收起臂（断电前必须）；SKIP_STOW=1 可跳过。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

if [[ "${SKIP_STOW:-0}" != "1" ]]; then
  echo "收起机械臂（SKIP_STOW=1 跳过）..."
  bash "${SCRIPT_DIR}/stow.sh" || echo "WARN: stow failed — do not power off until the arm is folded"
fi

if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  tmux kill-session -t "${TMUX_SESSION}"
  echo "✅ 已停止 tmux 会话: ${TMUX_SESSION}"
else
  echo "ℹ️  没有运行中的会话: ${TMUX_SESSION}"
fi

# 清理可能残留的 stream_api 进程
pkill -f "stream_api_server.py" 2>/dev/null && echo "✅ 已清理 stream_api 进程" || true

echo "完成。"
