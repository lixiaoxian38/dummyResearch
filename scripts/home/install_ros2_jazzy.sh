#!/usr/bin/env bash
# 在 Ubuntu 24.04 (noble) 上安装 ROS 2 Jazzy + 常用依赖
# 用法: bash scripts/home/install_ros2_jazzy.sh
#
# 说明: 本仓库文档多为 Humble；24.04 官方对应 Jazzy。
#       装完后再 colcon 编译 dummy_moveit_ws，遇 API 差异再改。

set -euo pipefail

if [[ "$(lsb_release -cs)" != "noble" ]]; then
  echo "此脚本按 Ubuntu 24.04 noble 编写，当前: $(lsb_release -cs)"
  exit 1
fi

echo "=========================================="
echo " 安装 ROS 2 Jazzy (Desktop) + 开发工具"
echo "=========================================="

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release software-properties-common apt-transport-https

sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt-get install -y ros-apt-source || true
# 官方推荐方式（若尚未添加源）
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
fi

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf-transformations \
  ros-jazzy-image-transport \
  ros-jazzy-diagnostic-updater

# RealSense ROS 包装（名称随源可能变化）
sudo apt-get install -y ros-jazzy-realsense2-camera || \
  echo "⚠️  ros-jazzy-realsense2-camera 暂不可用，稍后用源码编译"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update || true

grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo ""
echo "=========================================="
echo " 安装完成。新开终端或执行:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 doctor"
echo "=========================================="
