#!/usr/bin/env bash
# 1) !START（可能需要 dialout / sudo）
# 2) Fibre usb + 小幅 Joint1（必须用普通用户，不要整段 sudo）
#
# 推荐：
#   bash scripts/home/arm_start_and_nudge.sh
# 若 ACM 无权限：
#   sudo bash scripts/home/arm_start_and_nudge.sh --start-only
#   bash scripts/home/arm_start_and_nudge.sh --nudge-only

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

MODE="${1:-all}"
export PYTHONPATH="/home/lxx/Projects/dummyResearch/dummy_moveit_ws/dummy_controller${PYTHONPATH:+:$PYTHONPATH}"

do_start() {
  python3 <<'PY'
import serial, time
print("=== serial !START ===")
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
ok = False
for i in range(6):
    ser.reset_input_buffer()
    ser.write(b"!START\n")
    time.sleep(1.0)
    data = ser.read(500)
    print(f"try{i}: {data!r}")
    if b"Started ok" in data or b"started" in data.lower():
        ok = True
        break
ser.close()
print("START ok?", ok)
if not ok:
    raise SystemExit(1)
PY
}

do_nudge() {
  # 禁止 root：sudo 下常无 pyusb，Fibre path=usb 会挂
  if [[ "$(id -u)" -eq 0 ]]; then
    echo "❌ 不要用 sudo 跑 Fibre/move_j。"
    echo "   请先：sudo bash $0 --start-only"
    echo "   再：  bash $0 --nudge-only"
    exit 1
  fi
  python3 <<'PY'
import sys, time
sys.path.insert(0, "/home/lxx/Projects/dummyResearch/dummy_moveit_ws/dummy_controller")
import dummy_controller.dummy_cli_tool.ref_tool as ref_tool

print("=== Fibre usb + Joint1 +8° ===")
# 确认 usb 通道已注册
import dummy_controller.dummy_cli_tool.fibre.discovery as disc
print("channel_types:", list(disc.channel_types.keys()))
if "usb" not in disc.channel_types:
    raise SystemExit("pyusb 未加载，无法用 path=usb。请: python3 -c 'import usb' 检查")

d = ref_tool.find_any(path="usb", timeout=20)
if d is None:
    raise SystemExit("find_any 超时，未找到设备")

try:
    print("voltage(逻辑电参考):", round(float(d.get_voltage()), 3))
except Exception as e:
    print("voltage skip:", e)

joints = (d.robot.joint_1, d.robot.joint_2, d.robot.joint_3,
          d.robot.joint_4, d.robot.joint_5, d.robot.joint_6)
before = [float(j.angle) for j in joints]
print("before deg:", [round(x, 3) for x in before])
d.robot.set_enable(True)
time.sleep(0.3)
target = list(before)
target[0] = before[0] + 8.0
print("move_j once:", [round(x, 3) for x in target])
print("请盯着真机 Joint1…")
d.robot.move_j(*target)
time.sleep(3.5)
after = [float(j.angle) for j in joints]
print("after deg:", [round(x, 3) for x in after])
print("delta:", [round(after[i] - before[i], 3) for i in range(6)])
PY
}

case "$MODE" in
  --start-only) do_start ;;
  --nudge-only) do_nudge ;;
  all|*)
    if [[ "$(id -u)" -eq 0 ]]; then
      echo "检测到 sudo：只执行 !START，然后请用【普通用户】再跑："
      echo "  bash $0 --nudge-only"
      do_start
      exit 0
    fi
    do_start
    do_nudge
    ;;
esac
