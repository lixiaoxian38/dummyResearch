#!/usr/bin/env python3
"""Initialize Dummy arm over CDC ASCII for eye-in-hand tracking / calib.

Presets (FW degrees, economy-kit CDC):
  handeye    0, 0, 90, 0, 0, 0        # follow start (J1/J2=0, J3=90, J4/J5=0)
  handeye_calib -8.7, 20, 90, 0, 60, 0  # old EIH calib pose
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
    "handeye": np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0]),
    "handeye_calib": np.array([-8.7, 20.0, 90.0, 0.0, 60.0, 0.0]),
    "lookdown": np.array([0.0, -48.0, 125.0, 0.0, -80.0, 0.0]),
    "soft7": np.array([0.0, -55.0, 150.0, 0.0, 0.0, 0.0]),
    "exact7": np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0]),
    "stow": np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0]),
}


def send(ser: serial.Serial, cmd: str, wait: float = 0.3, reset: bool = True) -> str:
    if not cmd.endswith("\n"):
        cmd += "\n"
    if reset:
        ser.reset_input_buffer()
    ser.write(cmd.encode("ascii", errors="ignore"))
    # no flush()/tcdrain — wedges ACM as Write timeout
    time.sleep(wait)
    return ser.read(400).decode(errors="replace") if wait >= 0.05 else ""


def get_jpos(ser: serial.Serial) -> np.ndarray | None:
    raw = send(ser, "#GETJPOS", wait=0.25)
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
    return np.array(nums[:6], dtype=float) if len(nums) >= 6 else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--speed", type=float, default=28.0)
    ap.add_argument(
        "--preset",
        choices=sorted(PRESETS.keys()),
        default="lookdown",
        help="Start pose preset (default: lookdown)",
    )
    ap.add_argument("--exact", action="store_true", help="Alias for --preset exact7")
    ap.add_argument(
        "--steps",
        type=int,
        default=0,
        help="0=continuous stream (default). >0=legacy equal-time hops (jerky).",
    )
    args = ap.parse_args()
    target = PRESETS["exact7"] if args.exact else PRESETS[args.preset]

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.4
    ser.write_timeout = 0.6
    ser.dsrdtr = False
    ser.rtscts = False
    try:
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

        # One firmware target (smooth). N hops = 一下一下; racing a virtual
        # stream finishes the script while the arm is still halfway.
        body = ",".join(f"{d:.2f}" for d in target.tolist())
        err0 = float(np.max(np.abs(pos - target)))
        wait_s = max(err0 / max(args.speed, 8.0) + 1.2, 1.5)
        print(send(ser, f">{body},{args.speed:.1f}", wait=0.05).strip())
        time.sleep(wait_s)
        final = get_jpos(ser)
        if final is not None and float(np.max(np.abs(final - target))) > 5.0:
            print("retry from", np.round(final, 2))
            send(ser, f">{body},{args.speed:.1f}", wait=0.05)
            time.sleep(max(float(np.max(np.abs(final - target))) / max(args.speed, 8.0) + 0.8, 1.0))
            final = get_jpos(ser)
        print("done", np.round(final, 2) if final is not None else "?")
        return 0
    except serial.SerialTimeoutException:
        print(
            "ERROR: CDC Write timeout — 串口卡死。不必重启电脑，拔插机械臂 USB 后再试。",
            file=sys.stderr,
        )
        return 2
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
