#!/usr/bin/env bash
# Dummy Tracking Hub — 桌面双击入口。
#
# 会按需启动（已在跑则跳过，不杀进程、不自动跟、不收起）：
#   ROS2 环境、MoveIt Servo（无 Fibre）、D415、CDC bridge、手眼 TF、检测节点、HUD
# LIVE 跟随仍由窗口里的「开始跟随」触发（会先回初始位请用「回初始」）。
# Stream API / tmux / git pull 不在这里（那是 go.sh）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOG_DIR="${REPO_ROOT}/.logs"
mkdir -p "${LOG_DIR}" /tmp/dummy_track_ctrl
HUB_LOG="${LOG_DIR}/tracking_hub.log"

# OpenCV HUD on GNOME/Wayland
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" ]]; then
  for xa in /run/user/"${UID}"/.mutter-Xwaylandauth.* /run/user/"${UID}"/gdm/Xauthority; do
    if [[ -f "${xa}" ]]; then
      export XAUTHORITY="${xa}"
      break
    fi
  done
fi
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${UID}}"

say() { echo "$*"; echo "$(date '+%F %T') $*" >>"${HUB_LOG}"; }

_running() {
  pgrep -f "$1" >/dev/null 2>&1
}

_bg() {
  local name="$1"
  shift
  nohup bash -lc "$*" >>"${LOG_DIR}/${name}.log" 2>&1 &
  echo $! >"/tmp/dummy_track_ctrl/${name}.pid"
  say "started ${name} (log: ${LOG_DIR}/${name}.log)"
}

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

cd "${REPO_ROOT}"
say "========== Dummy Tracking Hub =========="

if _running "scripts/vision/cam_live_view.py"; then
  say "Tracking Hub 窗口已在运行。把板放到相机前，用窗口里的按钮。"
  wmctrl -a "Dummy Tracking Hub" 2>/dev/null || true
  exit 0
fi

if [[ ! -e /dev/ttyACM0 ]]; then
  say "等待机械臂 USB /dev/ttyACM0 …（通电并接上 USB）"
  for _ in $(seq 1 40); do
    [[ -e /dev/ttyACM0 ]] && break
    sleep 0.5
  done
fi
if [[ ! -e /dev/ttyACM0 ]]; then
  say "ERROR: 没有 /dev/ttyACM0。检查电源和 USB 后再双击。"
  read -r -p "按回车关闭…" _ || true
  exit 1
fi

if _running "dummy_servo_hardware"; then
  say "ERROR: Fibre dummy_servo_hardware 在跑，会和 CDC 抢口。先停 Fibre 再开本程序。"
  read -r -p "按回车关闭…" _ || true
  exit 1
fi

if ! _running "dummy_moveit_config servo_streaming.launch.py" && ! _running "/servo_node" && ! pgrep -x servo_node >/dev/null 2>&1; then
  say "启动 ROS MoveIt Servo（不开 Fibre、不开 RViz）…"
  _bg servo_launch \
    "source '${SCRIPT_DIR}/env.sh' && cd '${WS_DIR}' && exec ros2 launch dummy_moveit_config servo_streaming.launch.py start_fibre_hw:=false start_rviz:=false"
  for _ in $(seq 1 30); do
    pgrep -x servo_node >/dev/null 2>&1 && break
    sleep 0.5
  done
else
  say "MoveIt Servo 已在运行，跳过。"
fi

if ! _running "realsense2_camera_node" && ! _running "rs_launch.py"; then
  say "启动 D415 RealSense…"
  _bg realsense \
    "source '${SCRIPT_DIR}/env.sh' && exec ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true rgb_camera.color_profile:=640,480,15"
else
  say "RealSense 已在运行，跳过。"
fi

if ! _running "cdc_servo_bridge.py"; then
  say "启动 CDC bridge（力矩保持）…"
  _bg cdc_bridge \
    "source '${SCRIPT_DIR}/env.sh' && cd '${REPO_ROOT}' && exec python3 -u scripts/home/cdc_servo_bridge.py /dev/ttyACM0 --ros-args -p joint_soft_half_range_deg:=0.0 -p lock_j4:=true -p lock_j6:=true"
  sleep 1.2
else
  say "CDC bridge 已在运行，跳过。"
fi

say "等待 /joint_states 与相机…"
for _ in $(seq 1 25); do
  timeout 1 ros2 topic echo /joint_states --once >/dev/null 2>&1 && break
  sleep 0.4
done
for _ in $(seq 1 20); do
  timeout 1 ros2 topic echo /camera/camera/color/image_raw --once >/dev/null 2>&1 && break
  sleep 0.5
done

timeout -k 1 8 ros2 service call /servo_node/start_servo std_srvs/srv/Trigger >/dev/null 2>&1 || true
timeout -k 1 8 ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true

say "检测节点 + 手眼 TF…"
python3 "${SCRIPT_DIR}/track_control.py" hub || say "WARN: hub vision 未完全就绪，HUD 仍会打开"

say "打开 Tracking Hub 窗口（不自动跟随；点「回初始」再「开始跟随」）…"
set +e
python3 "${REPO_ROOT}/scripts/vision/cam_live_view.py"
rc=$?
set -e
say "HUD 已退出 rc=${rc}。ROS / 相机 / bridge 仍在跑。断电前请在窗口点「停止收起」，或: bash scripts/home/stow.sh"
if [[ "${rc}" -ne 0 ]]; then
  read -r -p "按回车关闭…" _ || true
fi
exit "${rc}"
