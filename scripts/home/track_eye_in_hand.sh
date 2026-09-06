#!/usr/bin/env bash
# 眼在手上追踪 — dry-run 或真机（家里）
#
# 用法:
#   bash scripts/home/track_eye_in_hand.sh              # dry-run（默认）
#   bash scripts/home/track_eye_in_hand.sh --live       # 真机 Servo（需 CDC）
#   bash scripts/home/track_eye_in_hand.sh --calibrated # 用 easy_handeye2 外参

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO_ROOT}/dummy_moveit_ws"

DRY_RUN=true
USE_EASY=false
START_SERVO=false
FOLLOW_ORI=true

for arg in "$@"; do
  case "$arg" in
    --live) DRY_RUN=false; START_SERVO=true ;;
    --calibrated) USE_EASY=true ;;
    --pos-only) FOLLOW_ORI=false ;;
    -h|--help)
      grep '^#' "$0" | head -15
      exit 0
      ;;
  esac
done

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

echo "eye_in_hand track  dry_run=${DRY_RUN}  use_easy_handeye=${USE_EASY}"

ros2 launch dummy_vision eye_in_hand_track.launch.py \
  dry_run:=${DRY_RUN} \
  start_servo:=${START_SERVO} \
  use_easy_handeye:=${USE_EASY} \
  follow_orientation:=${FOLLOW_ORI} \
  start_realsense:=true \
  marker_length:=0.05
