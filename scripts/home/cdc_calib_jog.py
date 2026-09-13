#!/usr/bin/env python3
"""CDC ASCII joint publisher + keyboard jog for hand-eye calibration.

Stops relying on Fibre move_j (clicks but no motion). Opens /dev/ttyACM0,
publishes /joint_states for robot_state_publisher / easy_handeye2, and jogs
with keys 1-6 (R reverse, Q quit).

IMPORTANT: stop dummy_servo_hardware first (Fibre holds the same USB device).

  # terminal A (after killing Fibre hardware):
  source /opt/ros/jazzy/setup.bash
  source dummy_moveit_ws/install/setup.bash
  python3 scripts/home/cdc_calib_jog.py
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Match dummy_servo_hardware.py conversions (MoveIt rad <-> firmware deg).
RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])
JOINT_NAMES = [f"Joint{i}" for i in range(1, 7)]
STEP_DEG = 3.0


def fw_deg_to_ros_rad(deg: np.ndarray) -> np.ndarray:
    rad_cmd = np.deg2rad(deg)
    return (rad_cmd / RAD_DIRECT) - RAD_VOLUMN


def ros_rad_to_fw_deg(rad: np.ndarray) -> np.ndarray:
    rad_fixed = (rad + RAD_VOLUMN) * RAD_DIRECT
    return np.rad2deg(rad_fixed)


class SerialCDC:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.15)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def _read_for(self, wait: float) -> str:
        t0 = time.time()
        buf = b""
        last = t0
        while time.time() - t0 < wait:
            chunk = self.ser.read(512)
            if chunk:
                buf += chunk
                last = time.time()
            elif time.time() - last > 0.12:
                break
            else:
                time.sleep(0.01)
        return buf.decode(errors="replace")

    def send(self, cmd: str, wait: float = 0.6) -> str:
        with self._lock:
            if not cmd.endswith("\n"):
                cmd += "\n"
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode("ascii", errors="ignore"))
            # Do not flush()/tcdrain — wedges Dummy CDC as Write timeout.
            return self._read_for(wait)

    def get_jpos(self) -> list[float] | None:
        raw = self.send("#GETJPOS", wait=0.25)
        nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
        if len(nums) >= 6:
            return nums[:6]
        return None

    def move_j(self, deg: list[float], speed: float = 25.0) -> None:
        body = ",".join(f"{d:.2f}" for d in deg)
        self.send(f">{body},{speed:.1f}", wait=0.15)


class CdcCalibJog(Node):
    def __init__(self, port: str):
        super().__init__("cdc_calib_jog")
        self.cdc = SerialCDC(port)
        self.get_logger().info(self.cdc.send("!START", wait=1.5).strip() or "START")
        self.get_logger().info(self.cdc.send("#CMDMODE 2", wait=1.0).strip() or "CMDMODE 2")
        pos = self.cdc.get_jpos()
        if pos is None:
            raise RuntimeError("CDC #GETJPOS failed — is Fibre hardware still holding USB?")
        self.deg = np.array(pos, dtype=float)
        self.sign = 1.0
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self._last_poll = 0.0
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            f"CDC ready at {self.deg.round(1)} deg. Keys: 1-6 jog, R reverse, Q quit."
        )

    def _tick(self) -> None:
        # Do not #GETJPOS every tick — a hung ACM blocks /joint_states and splits TF.
        now = time.time()
        if now - self._last_poll > 0.25:
            self._last_poll = now
            got = self.cdc.get_jpos()
            if got is not None:
                self.deg = np.array(got, dtype=float)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = JOINT_NAMES
        ros_rad = fw_deg_to_ros_rad(self.deg)
        msg.position = [float(x) for x in ros_rad]
        self.pub.publish(msg)

    def nudge(self, joint_idx: int) -> None:
        self.deg[joint_idx] += self.sign * STEP_DEG
        self.cdc.move_j(self.deg.tolist())
        self.get_logger().info(f"J{joint_idx+1} -> {self.deg[joint_idx]:.1f} deg")


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument(
        "--publish-only",
        action="store_true",
        help="Only publish /joint_states (no keyboard jog; safe for background)",
    )
    args = ap.parse_args()

    rclpy.init()
    node = CdcCalibJog(args.port)

    if args.publish_only:
        print("CDC publish-only: /joint_states from #GETJPOS (Ctrl+C to stop).", flush=True)
        try:
            rclpy.spin(node)
        finally:
            try:
                node.cdc.send("!DISABLE", wait=0.5)
            except Exception:
                pass
            node.cdc.close()
            node.destroy_node()
            rclpy.shutdown()
        return

    print("CDC jog ready. 1-6 / R / Q — keep this terminal focused.", flush=True)

    stop = False

    def spin():
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    try:
        while rclpy.ok() and not stop:
            ch = _getch()
            if ch in ("q", "Q", "\x03"):
                stop = True
                break
            if ch in ("r", "R"):
                node.sign *= -1
                print(f"direction={node.sign:+.0f}", flush=True)
                continue
            if ch in "123456":
                node.nudge(int(ch) - 1)
    finally:
        stop = True
        try:
            node.cdc.send("!DISABLE", wait=0.5)
        except Exception:
            pass
        node.cdc.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
