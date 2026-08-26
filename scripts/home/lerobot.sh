#!/usr/bin/env bash
# 启动 LeRobot 手机遥操作（需 stream API 已运行）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [[ -f "${SCRIPT_DIR}/config.env" ]]; then
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/config.env"
fi

LEROBOT_DIR="${REPO_ROOT}/lerobot"
LEROBOT_CONDA_ENV="${LEROBOT_CONDA_ENV:-lerobot}"
STREAM_API_PORT="${STREAM_API_PORT:-8001}"
ROBOT_IP="${ROBOT_IP:-127.0.0.1}"

# 检查 stream API
if ! curl -sf "http://${ROBOT_IP}:${STREAM_API_PORT}/api/status" >/dev/null 2>&1; then
  echo "❌ Stream API 未响应 (http://${ROBOT_IP}:${STREAM_API_PORT})"
  echo "   请先运行: bash scripts/home/start.sh"
  exit 1
fi

echo "=========================================="
echo " 启动 LeRobot 遥操作"
echo " Stream API: http://${ROBOT_IP}:${STREAM_API_PORT}"
echo "=========================================="

cd "${LEROBOT_DIR}"

# 激活 conda
if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${LEROBOT_CONDA_ENV}" 2>/dev/null || {
    echo "❌ conda 环境 '${LEROBOT_CONDA_ENV}' 不存在"
    echo "   请先创建 LeRobot 环境，或修改 scripts/home/config.env"
    exit 1
  }
else
  echo "⚠️  未找到 conda，尝试直接运行 lerobot-teleoperate"
fi

lerobot-teleoperate \
  --robot.type=dummy_stream \
  --robot.id=my_awesome_arm \
  --robot.ip="${ROBOT_IP}" \
  --robot.port="${STREAM_API_PORT}" \
  --teleop.type=mobile \
  --teleop.id=my_awesome_teleop \
  --fps=10
