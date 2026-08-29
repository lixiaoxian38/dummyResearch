#!/usr/bin/env bash
# 用 venv 安装 D415 / vision 依赖（避开 PEP 668）
# 用法: bash scripts/vision/install_deps.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv-vision"

echo "=========================================="
echo " 安装 Vision 依赖到虚拟环境"
echo " ${VENV_DIR}"
echo "=========================================="

sudo apt-get update
sudo apt-get install -y python3-venv python3-full python3-opencv python3-numpy

if [[ ! -d "${VENV_DIR}" ]]; then
  python3 -m venv --system-site-packages "${VENV_DIR}"
  echo "✅ 已创建 venv（--system-site-packages，可复用 apt 的 opencv）"
else
  echo "ℹ️  venv 已存在，继续安装 pip 包"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install -U pip
python -m pip install "pyrealsense2==2.58.3.10794"

echo ""
echo "验证："
python - <<'PY'
import cv2
import numpy as np
import pyrealsense2 as rs
print("python:", __import__("sys").version.split()[0])
print("cv2:", cv2.__version__)
print("numpy:", np.__version__)
print("pyrealsense2: OK")
PY

echo ""
echo "=========================================="
echo " 完成。请用虚拟环境里的 python 运行："
echo "   source ${VENV_DIR}/bin/activate"
echo "   python scripts/vision/print_d415_intrinsics.py"
echo "   python scripts/vision/d415_aruco_demo.py --serial 033522060492"
echo "=========================================="
