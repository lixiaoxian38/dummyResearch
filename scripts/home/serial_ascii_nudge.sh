#!/usr/bin/env bash
# 纯串口（CDC ASCII）试控 Dummy — 不启 ROS
# 协议来自 D:/dummy/Dummy相关资料/Windows能连通的办法.md
#
# 用法：
#   bash scripts/home/serial_ascii_nudge.sh              # 读角 + Joint2 +5°
#   bash scripts/home/serial_ascii_nudge.sh --port /dev/ttyACM0
#   sudo bash scripts/home/serial_ascii_nudge.sh         # 无 dialout 时

set -euo pipefail

PORT="${PORT:-/dev/ttyACM0}"
BAUD=115200
DELTA_J2="${DELTA_J2:-5}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --delta) DELTA_J2="$2"; shift 2 ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

if [[ ! -e "$PORT" ]]; then
  echo "找不到 $PORT。请检查 USB（lsusb -d 1209:0d32）"
  exit 1
fi

python3 - "$PORT" "$BAUD" "$DELTA_J2" <<'PY'
import sys, time, serial

port, baud, delta_j2 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])

def read_lines(ser, wait=1.0):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < wait:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            t0 = time.time()  # extend while data flows
        else:
            time.sleep(0.05)
    return buf.decode(errors="replace")

def send(ser, cmd, wait=1.0):
    if not cmd.endswith("\n"):
        cmd += "\n"
    print(f">>> {cmd.strip()!r}")
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii", errors="ignore"))
    ser.flush()
    resp = read_lines(ser, wait=wait)
    print(f"<<< {resp!r}")
    return resp

print(f"打开 {port} @ {baud}")
ser = serial.Serial(port, baud, timeout=0.2)
time.sleep(0.3)

# 1) START
r = send(ser, "!START", wait=1.5)
if "Started" not in r and "started" not in r.lower() and "ok" not in r.lower():
    # 有的固件应答略有不同，再试一次
    r = send(ser, "!START", wait=1.5)

# 2) 实时覆盖模式（Studio 拖条关键）
send(ser, "#CMDMODE 2", wait=1.0)

# 3) 读关节角
pos_raw = send(ser, "#GETJPOS", wait=1.0)

def parse_angles(text: str):
    # 常见形式：一串逗号分隔数字，或带前缀
    import re
    nums = re.findall(r"[-+]?\d*\.?\d+", text.replace("\r", " "))
    vals = [float(x) for x in nums]
    if len(vals) >= 6:
        return vals[:6]
    return None

angles = parse_angles(pos_raw)
if not angles:
    print("未能解析 #GETJPOS，用「7」字默认角继续试：0,-73,180,0,0,0")
    angles = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
else:
    print("当前角(度):", [round(a, 3) for a in angles])

# 4) Joint2 小动
target = list(angles)
target[1] = angles[1] + delta_j2
cmd = ">" + ",".join(f"{v:.3f}" for v in target)
print(f"目标 Joint2 {angles[1]:.2f} → {target[1]:.2f}  (Δ={delta_j2})")
print("请盯着真机 Joint2…")
send(ser, cmd, wait=2.0)
time.sleep(1.5)
after_raw = send(ser, "#GETJPOS", wait=1.0)
after = parse_angles(after_raw)
if after:
    print("之后角(度):", [round(a, 3) for a in after])
    print("ΔJ2:", round(after[1] - angles[1], 3))
else:
    print("之后角度未能解析，请肉眼看是否动了")

ser.close()
print("完成")
PY
