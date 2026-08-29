#!/usr/bin/env bash
# 安装 Intel RealSense SDK（librealsense2 + viewer）
# 用法: bash scripts/home/install_realsense.sh

set -euo pipefail

echo "=========================================="
echo " 安装 RealSense SDK (Ubuntu)"
echo "=========================================="
echo "内核: $(uname -r)"
echo ""

sudo apt-get update
sudo apt-get install -y curl gnupg2 apt-transport-https ca-certificates lsb-release

sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update

# dkms: 内核补丁（新内核可能失败，失败则至少装 utils）
echo ""
echo "[1/2] 尝试安装 librealsense2-dkms ..."
if sudo apt-get install -y librealsense2-dkms; then
  echo "✅ dkms 安装成功"
else
  echo "⚠️  dkms 安装失败（常见于较新内核）。可继续用用户态驱动，先装工具包。"
fi

echo ""
echo "[2/2] 安装 librealsense2-utils / librealsense2-dev ..."
sudo apt-get install -y librealsense2-utils librealsense2-dev

echo ""
echo "=========================================="
echo " 安装完成。请重新插拔一次相机，然后运行："
echo "   rs-enumerate-devices"
echo "   realsense-viewer"
echo "=========================================="
