#!/usr/bin/env bash
# 串口 Write timeout 时用：复位 Dummy CDC，不必重启电脑。
# 用法: bash scripts/home/cdc_recover.sh
set -euo pipefail

VIDPID="1209:0d32"
echo "=== Dummy CDC recover (no PC reboot) ==="
echo "1) pause Servo if ROS is up"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash || true
  timeout 3 ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1 || true
fi

echo "2) no process should hold ttyACM*"
fuser /dev/ttyACM* 2>/dev/null && echo "WARN: something still has the port" || true

echo "3) usbreset ${VIDPID} (needs usbreset; sudo if permission denied)"
if command -v usbreset >/dev/null; then
  usbreset "${VIDPID}" && echo "usbreset ok" || {
    echo "usbreset failed — 请拔插机械臂 USB 线，然后回车"
    read -r _
  }
else
  echo "没有 usbreset — 请拔插机械臂 USB 线，然后回车"
  read -r _
fi

echo "4) wait for /dev/ttyACM*"
ok=0
for _ in $(seq 1 20); do
  if ls /dev/ttyACM* >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 0.3
done
ls -l /dev/ttyACM* /dev/serial/by-id/*CDC* 2>/dev/null || true
if [[ "${ok}" -ne 1 ]]; then
  echo "ERROR: USB 还没回来。再拔插一次，或给机械臂重新上电。"
  exit 1
fi
echo "CDC 回来了。下一步：python3 scripts/home/cdc_servo_bridge.py"
echo "或 HUD 再点 回初始 / 停止收起。"
