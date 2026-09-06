#!/usr/bin/env python3
"""Headless CDC -> /joint_states (+ world->base_link static TF).

Opens serial BEFORE rclpy to avoid silent hangs during node init.
"""

from __future__ import annotations

import re
import sys
import time

import numpy as np
import rclpy
import serial
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster

RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])
NAMES = [f"Joint{i}" for i in range(1, 7)]


def fw_deg_to_ros_rad(deg: np.ndarray) -> np.ndarray:
    rad_cmd = np.deg2rad(deg)
    return (rad_cmd / RAD_DIRECT) - RAD_VOLUMN


def open_cdc(port: str) -> serial.Serial:
    print(f"[cdc_js_pub] opening {port}...", flush=True)
    ser = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()
    for cmd in (b"!START\n", b"#CMDMODE 2\n"):
        ser.write(cmd)
        ser.flush()
        time.sleep(0.2)
        resp = ser.read(300)
        print(f"[cdc_js_pub] {cmd.strip()!r} -> {resp!r}", flush=True)
    return ser


def get_jpos(ser: serial.Serial) -> list[float] | None:
    ser.reset_input_buffer()
    ser.write(b"#GETJPOS\n")
    ser.flush()
    time.sleep(0.1)
    raw = ser.read(300).decode(errors="replace")
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
    return nums[:6] if len(nums) >= 6 else None


class CdcJsPub(Node):
    def __init__(self, ser: serial.Serial, deg0: np.ndarray) -> None:
        super().__init__("cdc_js_pub")
        self.ser = ser
        self.deg = deg0
        self._lock = __import__("threading").Lock()
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        self.static.sendTransform(t)
        # Publish fast; poll serial slower so TF stays fresh.
        self.create_timer(0.05, self.tick_pub)
        self.create_timer(0.25, self.tick_poll)
        self.get_logger().info(f"publishing /joint_states at {self.deg.round(1)} deg")

    def tick_poll(self) -> None:
        got = get_jpos(self.ser)
        if got is not None:
            with self._lock:
                self.deg = np.array(got, dtype=float)

    def tick_pub(self) -> None:
        with self._lock:
            deg = self.deg.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NAMES
        msg.position = [float(x) for x in fw_deg_to_ros_rad(deg)]
        self.pub.publish(msg)


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    ser = open_cdc(port)
    pos = get_jpos(ser)
    if pos is None:
        print("[cdc_js_pub] #GETJPOS failed", flush=True)
        ser.close()
        raise SystemExit(1)
    print(f"[cdc_js_pub] jpos={pos}", flush=True)

    print("[cdc_js_pub] rclpy.init...", flush=True)
    rclpy.init()
    node = CdcJsPub(ser, np.array(pos, dtype=float))
    print("[cdc_js_pub] spinning", flush=True)
    try:
        rclpy.spin(node)
    finally:
        try:
            ser.write(b"!DISABLE\n")
        except Exception:
            pass
        ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
