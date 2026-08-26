#!/usr/bin/env bash
# 加载项目环境变量（其他脚本 source 此文件）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/config.env"

ROS_DISTRO="${ROS_DISTRO:-humble}"
STREAM_API_PORT="${STREAM_API_PORT:-8001}"
LEROBOT_CONDA_ENV="${LEROBOT_CONDA_ENV:-lerobot}"
TMUX_SESSION="${TMUX_SESSION:-dummy}"

WS_DIR="${REPO_ROOT}/dummy_moveit_ws"
LOG_DIR="${REPO_ROOT}/.logs"
mkdir -p "${LOG_DIR}"

if [[ -f "${CONFIG_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
fi

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "❌ 未找到 ROS ${ROS_DISTRO}，请先安装 ROS 2 Humble"
  exit 1
fi

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "⚠️  工作空间未编译，请先运行: bash scripts/home/setup_once.sh"
fi

export REPO_ROOT WS_DIR LOG_DIR STREAM_API_PORT TMUX_SESSION
