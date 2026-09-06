#!/usr/bin/env bash
# 眼在手上标定 — 一键启动（家里 Linux）
#
# 用法:
#   bash scripts/home/calibrate_eye_in_hand.sh
#   bash scripts/home/calibrate_eye_in_hand.sh --no-arm   # 臂已在别处启动
#
# 前置:
#   1. 打印 scripts/vision/aruco_markers/aruco_original_id0_50mm_a4.pdf（100% 比例）
#   2. 标定板固定在桌上，腕部相机能看见
#   3. sudo apt install ros-${ROS_DISTRO}-easy-handeye2 ros-${ROS_DISTRO}-realsense2-camera

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO_ROOT}/dummy_moveit_ws"
START_ARM=1

for arg in "$@"; do
  case "$arg" in
    --no-arm) START_ARM=0 ;;
    -h|--help)
      sed -n '1,20p' "$0"
      exit 0
      ;;
  esac
done

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

echo "=========================================="
echo " 眼在手上标定 (easy_handeye2)"
echo " 标定板: DICT_ARUCO_ORIGINAL id=0  边长 50mm"
echo " 保存名: dummy_eih_calib"
echo "=========================================="

if ! ros2 pkg prefix easy_handeye2 &>/dev/null; then
  echo "❌ 未安装 easy_handeye2"
  echo "   sudo apt install ros-${ROS_DISTRO:-jazzy}-easy-handeye2"
  exit 1
fi

if [[ "${START_ARM}" -eq 1 ]]; then
  echo "启动 MoveIt Servo（提供 base_link / link6_1_1 TF）..."
  echo "  另开终端可: tmux attach -t dummy  或看 RViz"
  ros2 launch dummy_moveit_config servo_streaming.launch.py &
  ARM_PID=$!
  sleep 8
  echo "Arm launch PID=${ARM_PID}"
fi

echo "启动标定: RealSense + ArUco + easy_handeye2 GUI"
echo "  → 摆 15+ 姿态 Take Sample → Compute → Save"
echo ""

ros2 launch dummy_vision eye_in_hand_calibrate.launch.py \
  marker_length:=0.05 \
  marker_id:=0 \
  calibration_name:=dummy_eih_calib
