#!/usr/bin/env bash
# Software-in-the-loop test for WDLS Follow (no real arm / no CDC / no camera).
#
# Brings up: robot_state_publisher + fixed handeye joints + handeye/optical TF stubs
#            + wdls_follow_resolver (dry_run) + fake Target bus publisher
# Prints:    /tracking/wdls_qdot and /tracking/status
#
# Usage:
#   bash scripts/home/sim_wdls_follow.sh
#   bash scripts/home/sim_wdls_follow.sh --duration 20   # auto-stop after N seconds
# Stop: Ctrl+C (kills child processes)
set -euo pipefail

DURATION_SEC=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration|-d)
      DURATION_SEC="${2:-0}"
      shift 2
      ;;
    *)
      echo "Unknown arg: $1 (supported: --duration N)"
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

LOG_DIR="${REPO_ROOT}/.logs/wdls_sim"
PID_DIR="/tmp/dummy_wdls_sim"
mkdir -p "${LOG_DIR}" "${PID_DIR}"

PIDS=()
cleanup() {
  echo ""
  echo "=== stopping WDLS sim ==="
  for p in "${PIDS[@]:-}"; do
    kill "${p}" 2>/dev/null || true
  done
  # Also clear anything we tagged
  for f in "${PID_DIR}"/*.pid; do
    [[ -f "${f}" ]] || continue
    kill "$(cat "${f}")" 2>/dev/null || true
  done
  rm -rf "${PID_DIR}"
}
trap cleanup EXIT INT TERM

bg() {
  local name="$1"
  shift
  echo "[sim] start ${name}"
  nohup bash -lc "$*" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  echo "${pid}" >"${PID_DIR}/${name}.pid"
}

echo "=========================================="
echo " WDLS Follow — software sim (no arm)"
echo " logs: ${LOG_DIR}"
echo "=========================================="

# Avoid clobbering a live Tracking Hub session that owns /joint_states.
if pgrep -f 'cdc_servo_bridge.py' >/dev/null 2>&1; then
  echo "WARN: cdc_servo_bridge is running. Sim will publish /joint_states too."
  echo "      Prefer pausing LIVE / killing bridge before sim, or use another machine."
fi

DESC_SHARE="$(ros2 pkg prefix dummy-ros2_description)/share/dummy-ros2_description"
URDF="${DESC_SHARE}/urdf/dummy.urdf"
if [[ ! -f "${URDF}" ]]; then
  echo "ERROR: missing ${URDF} — colcon build dummy-ros2_description first"
  exit 1
fi
# Avoid ARG_MAX: stuff URDF into a params YAML for robot_state_publisher.
RSP_YAML="${PID_DIR}/robot_description.yaml"
python3 - "${URDF}" "${RSP_YAML}" <<'PY'
from pathlib import Path
import sys
urdf_path, out_path = Path(sys.argv[1]), Path(sys.argv[2])
urdf = urdf_path.read_text(encoding="utf-8")
lines = "\n".join("      " + line for line in urdf.splitlines())
out_path.write_text(
    "robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n"
    + lines
    + "\n",
    encoding="utf-8",
)
PY

# 1) Robot TF from fixed handeye joints (ROS rad ≈ all 0 at FW [0,0,90,0,0,0])
bg robot_state \
  "source '${SCRIPT_DIR}/env.sh' && exec ros2 run robot_state_publisher robot_state_publisher --ros-args \
    --params-file '${RSP_YAML}'"

bg joint_states \
  "source '${SCRIPT_DIR}/env.sh' && exec python3 - <<'PY'
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = [f'Joint{i}' for i in range(1, 7)]
# handeye FW [0,0,90,0,0,0] → ROS rad with RAD_VOLUMN J3=π/2 → zeros
Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

rclpy.init()
node = Node('sim_joint_states')
pub = node.create_publisher(JointState, '/joint_states', 10)

def tick():
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = NAMES
    msg.position = Q
    pub.publish(msg)

node.create_timer(0.05, tick)
rclpy.spin(node)
PY"

# 2) Hand-eye + optical stubs (no RealSense)
bg handeye_tf \
  "source '${SCRIPT_DIR}/env.sh' && cd '${WS_DIR}' && exec ros2 launch dummy_vision eye_in_hand_publish.launch.py \
    use_easy_handeye:=false publish_optical_stub:=true"

sleep 1.5

# 3) WDLS resolver dry_run (no Servo / no CDC)
bg wdls \
  "source '${SCRIPT_DIR}/env.sh' && cd '${WS_DIR}' && exec ros2 run dummy_vision wdls_follow_resolver_node --ros-args \
    -r __node:=wdls_follow_resolver \
    --params-file '${REPO_ROOT}/dummy_moveit_ws/dummy_vision/config/follow_live.yaml' \
    -p dry_run:=true"

# 4) Fake Target on optical bus
bg target \
  "source '${SCRIPT_DIR}/env.sh' && exec python3 '${REPO_ROOT}/scripts/vision/sim_wdls_target.py'"

sleep 1.0
echo ""
echo "Watching /tracking/wdls_qdot (J1..J6 rad/s) and status…"
echo "Expect: non-zero qdot while target ellipses; near-zero during dwell/Hold."
echo "Ctrl+C to stop."
echo ""

# Monitor (foreground)
export SIM_WDLS_DURATION_SEC="${DURATION_SEC}"
python3 - <<'PY'
import json, os, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

duration = float(os.environ.get("SIM_WDLS_DURATION_SEC") or "0")

rclpy.init()
node = Node("sim_wdls_monitor")
last_q = [0.0] * 6
last_st = {}

def on_q(msg: Float64MultiArray):
    global last_q
    d = list(msg.data)[:6]
    while len(d) < 6:
        d.append(0.0)
    last_q = d

def on_st(msg: String):
    global last_st
    try:
        last_st = json.loads(msg.data)
    except Exception:
        last_st = {"raw": msg.data}

node.create_subscription(Float64MultiArray, "/tracking/wdls_qdot", on_q, 10)
node.create_subscription(String, "/tracking/status", on_st, 10)

def spin():
    rclpy.spin(node)

th = threading.Thread(target=spin, daemon=True)
th.start()
t0 = time.time()
try:
    while True:
        time.sleep(0.5)
        if duration > 0 and (time.time() - t0) >= duration:
            print(f"[sim] duration {duration:.0f}s reached — exit")
            break
        q = last_q
        n = sum(x * x for x in q) ** 0.5
        phase = last_st.get("phase", "?")
        xy = last_st.get("xy")
        z = last_st.get("z")
        xy_s = f"{xy:.3f}" if isinstance(xy, (int, float)) else "-"
        z_s = f"{z:.3f}" if isinstance(z, (int, float)) else "-"
        print(
            f"t={time.time()-t0:5.1f}s  phase={phase:6s}  xy={xy_s}  z={z_s}  "
            f"|qdot|={n:.3f}  qdot=[{q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},"
            f"{q[3]:+.3f},{q[4]:+.3f},{q[5]:+.3f}]"
        )
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
