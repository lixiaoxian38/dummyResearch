#!/usr/bin/env python3
"""Initialize Dummy arm over CDC ASCII for eye-in-hand tracking / calib.

Presets (FW degrees, economy-kit CDC):
  handeye    -8.7, 20, 90, 0, 60, 0   # user-tuned EIH calib start (2026-09-06)
  lookdown   0, -48, 125, 0, -80, 0   # camera roughly toward -Z (table)
  soft7      0, -55, 150, 0, 0, 0     # side/oblique camera (legacy)
  exact7     0, -75, 180, 0, 0, 0     # REST_POSE (sits on old URDF limits)
  stow       same as exact7           # 收起/断电前姿态

Canonical handeye pose file: scripts/home/poses/handeye_start.json

Usage:
  python3 scripts/home/cdc_home_seven.py --preset handeye   # EIH calib start
  python3 scripts/home/cdc_home_seven.py                     # default: lookdown
  python3 scripts/home/cdc_home_seven.py --preset soft7
  python3 scripts/home/cdc_home_seven.py --exact
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import numpy as np
import serial

PRESETS = {
    "handeye": np.array([-8.7, 20.0, 90.0, 0.0, 60.0, 0.0]),
    "lookdown": np.array([0.0, -48.0, 125.0, 0.0, -80.0, 0.0]),
    "soft7": np.array([0.0, -55.0, 150.0, 0.0, 0.0, 0.0]),
    "exact7": np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0]),
    "stow": np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0]),
}


def send(ser: serial.Serial, cmd: str, wait: float = 0.3) -> str:
    if not cmd.endswith("\n"):
        cmd += "\n"
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii", errors="ignore"))
    # no flush()/tcdrain — wedges ACM as Write timeout
    time.sleep(wait)
    return ser.read(400).decode(errors="replace")


def get_jpos(ser: serial.Serial) -> np.ndarray | None:
    raw = send(ser, "#GETJPOS", wait=0.25)
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
    return np.array(nums[:6], dtype=float) if len(nums) >= 6 else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--speed", type=float, default=18.0)
    ap.add_argument(
        "--preset",
        choices=sorted(PRESETS.keys()),
        default="lookdown",
        help="Start pose preset (default: lookdown)",
    )
    ap.add_argument("--exact", action="store_true", help="Alias for --preset exact7")
    ap.add_argument("--steps", type=int, default=6)
    args = ap.parse_args()
    target = PRESETS["exact7"] if args.exact else PRESETS[args.preset]

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.4
    ser.write_timeout = 0.6
    ser.dsrdtr = False
    ser.rtscts = False
    ser.open()
    time.sleep(0.35)
    ser.reset_input_buffer()

    print(send(ser, "!START", 0.45).strip())
    print(send(ser, "#CMDMODE 2", 0.35).strip())
    pos = get_jpos(ser)
    if pos is None:
        print("ERROR: #GETJPOS failed", file=sys.stderr)
        return 1
    name = "exact7" if args.exact else args.preset
    print("from", np.round(pos, 2), "->", target, f"({name})")

    for i in range(1, max(args.steps, 1) + 1):
        t = pos + (target - pos) * (i / args.steps)
        body = ",".join(f"{d:.2f}" for d in t.tolist())
        print(send(ser, f">{body},{args.speed:.1f}", wait=0.9).strip())
        got = get_jpos(ser)
        if got is not None:
            print(" now", np.round(got, 2))
            pos = got

    final = get_jpos(ser)
    print("done", np.round(final, 2) if final is not None else "?")
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
