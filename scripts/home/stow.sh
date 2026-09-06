#!/usr/bin/env bash
# 收起机械臂（断电 / 关 LIVE / 收工前必须先跑）。
# 独占 /dev/ttyACM0：会停 tracker、CDC bridge、滑条 GUI。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${PORT:-/dev/ttyACM0}"

echo "=== Dummy stow: stop motion, then fold to [0, -75, 180, 0, 0, 0] ==="

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "${REPO_ROOT}/dummy_moveit_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/dummy_moveit_ws/install/setup.bash"
  fi
  timeout 4 ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1 || true
fi

kill_py() {
  local pat="$1"
  local pids
  pids=$(ps -C python3 -o pid=,args= 2>/dev/null | awk -v p="$pat" 'index($0,p){print $1}')
  if [[ -n "${pids}" ]]; then
    echo "stop ${pat}: ${pids}"
    kill ${pids} 2>/dev/null || true
  fi
}
kill_py aruco_servo_tracker
kill_py cdc_servo_bridge.py
kill_py dummy_slider_gui.py
sleep 0.6

if [[ ! -e "${PORT}" ]]; then
  echo "ERROR: ${PORT} missing — plug USB / power the arm, then retry"
  exit 1
fi
if fuser "${PORT}" >/dev/null 2>&1; then
  echo "WARN: ${PORT} still busy:"
  fuser -v "${PORT}" 2>&1 || true
  echo "Free the port, then: python3 ${REPO_ROOT}/scripts/home/cdc_home_seven.py --preset stow"
  exit 1
fi

python3 "${REPO_ROOT}/scripts/home/cdc_home_seven.py" --preset stow --steps 8
echo "Stowed. Safe to power off the arm (e-stop still recommended until motors disable)."
