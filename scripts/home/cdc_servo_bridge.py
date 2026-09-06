#!/usr/bin/env python3
"""CDC bridge for MoveIt Servo: /servo_node/command -> ASCII >j… + /joint_states.

Replaces Fibre dummy_servo_hardware for live visual servoing. Safety:
  - low default move speed
  - per-tick joint step clamp (max_delta_deg)
  - optional soft joint box around the pose at start
  - stop commanding if no Servo points (Servo already has command timeout)

Usage (stop cdc_js_pub first — same /dev/ttyACM0):
  source install/setup.bash
  python3 scripts/home/cdc_servo_bridge.py
"""

from __future__ import annotations

import re
import sys
import termios
import threading
import time

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

HANDEYE_FW = np.array([-8.7, 20.0, 90.0, 0.0, 60.0, 0.0])
STOW_FW = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])

RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])
NAMES = [f"Joint{i}" for i in range(1, 7)]


def fw_deg_to_ros_rad(deg: np.ndarray) -> np.ndarray:
    return (np.deg2rad(deg) / RAD_DIRECT) - RAD_VOLUMN


def ros_rad_to_fw_deg(rad: np.ndarray) -> np.ndarray:
    return np.rad2deg((rad + RAD_VOLUMN) * RAD_DIRECT)


class CdcServoBridge(Node):
    def __init__(self, port: str = "/dev/ttyACM0") -> None:
        super().__init__("cdc_servo_bridge")
        self.declare_parameter("port", port)
        self.declare_parameter("move_speed", 12.0)  # firmware speed units; keep low
        self.declare_parameter("max_delta_deg", 2.0)  # clamp each command vs last sent
        self.declare_parameter("joint_soft_half_range_deg", 20.0)  # around start; leave room to leave limits
        self.declare_parameter("enable_commands", True)

        port = str(self.get_parameter("port").value)
        self.move_speed = float(self.get_parameter("move_speed").value)
        self.max_delta = float(self.get_parameter("max_delta_deg").value)
        self.half_range = float(self.get_parameter("joint_soft_half_range_deg").value)
        self.enable_commands = bool(self.get_parameter("enable_commands").value)

        self._lock = threading.Lock()
        port = str(self.get_parameter("port").value)
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.25
        self.ser.write_timeout = 0.4
        self.ser.dsrdtr = False
        self.ser.rtscts = False
        self.ser.open()
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self._last_send_ok = True
        self._fail_t = 0.0
        for cmd in ("!START", "#CMDMODE 2"):
            self._send(cmd, wait=0.3)

        pos = self._get_jpos()
        if pos is None:
            raise RuntimeError("CDC #GETJPOS failed")
        self.deg = np.array(pos, dtype=float)
        self.center_deg = self.deg.copy()
        self.last_cmd_deg = self.deg.copy()
        self._last_cmd_t = 0.0

        self._goto_active = False
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointTrajectory, "/servo_node/command", self.on_servo, 10)
        self.create_service(Trigger, "~/goto_handeye", self._srv_goto_handeye)
        self.create_service(Trigger, "~/goto_stow", self._srv_goto_stow)
        self.create_timer(0.05, self.tick_pub)
        self.create_timer(0.2, self.tick_poll)

        self.get_logger().info(
            f"CDC servo bridge on {port}: speed={self.move_speed} "
            f"max_delta={self.max_delta}deg soft_box=±{self.half_range}deg "
            f"enable_commands={self.enable_commands} center={self.center_deg.round(1)}"
        )

    def _send(self, cmd: str, wait: float = 0.15) -> str:
        with self._lock:
            if not cmd.endswith("\n"):
                cmd += "\n"
            try:
                if not self.ser.is_open:
                    self._last_send_ok = False
                    self._fail_t = time.time()
                    return ""
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii", errors="ignore"))
                # Do not flush()/tcdrain — that blocks ACM as "Write timeout".
                time.sleep(wait)
                raw = self.ser.read(400).decode(errors="replace")
                self._last_send_ok = True
                return raw
            except (serial.SerialException, OSError, termios.error) as exc:
                self._last_send_ok = False
                self._fail_t = time.time()
                self.get_logger().error(f"CDC serial error: {exc}")
                return ""

    def _get_jpos(self) -> list[float] | None:
        raw = self._send("#GETJPOS", wait=0.12)
        if not raw:
            return None
        nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
        return nums[:6] if len(nums) >= 6 else None

    def _move_j(self, deg: np.ndarray, speed: float | None = None) -> bool:
        body = ",".join(f"{d:.2f}" for d in deg.tolist())
        sp = self.move_speed if speed is None else speed
        self._send(f">{body},{sp:.1f}", wait=0.08)
        return self._last_send_ok

    def tick_poll(self) -> None:
        if self._goto_active:
            return
        if time.time() - self._last_cmd_t < 0.15:
            return
        if not self._last_send_ok and time.time() - self._fail_t < 1.5:
            return
        got = self._get_jpos()
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

    def _srv_goto_handeye(self, _req, resp):
        return self._goto_preset("handeye", HANDEYE_FW, resp)

    def _srv_goto_stow(self, _req, resp):
        return self._goto_preset("stow", STOW_FW, resp)

    def _goto_preset(self, name: str, target: np.ndarray, resp) -> Trigger.Response:
        if self._goto_active:
            resp.success = False
            resp.message = "goto already running"
            return resp
        try:
            self._goto_fw(target)
            resp.success = True
            resp.message = f"at {name}, soft_box recentered"
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _goto_fw(self, target: np.ndarray, steps: int = 8) -> None:
        """Interpolate to FW deg, bypassing soft-box, then recenter the box."""
        self._goto_active = True
        try:
            pos = self.deg.copy()
            self.get_logger().info(
                f"goto {np.round(pos, 1)} -> {np.round(target, 1)} steps={steps}"
            )
            fails = 0
            for left in range(steps, 0, -1):
                q = pos + (target - pos) / float(left)
                self.last_cmd_deg = q
                if not self._move_j(q, speed=18.0):
                    fails += 1
                    if fails >= 2:
                        raise RuntimeError("CDC write failed — arm did not move")
                else:
                    fails = 0
                time.sleep(0.85)
                got = self._get_jpos()
                if got is not None:
                    self.deg = np.array(got, dtype=float)
                    pos = self.deg.copy()
            err = float(np.max(np.abs(self.deg - target)))
            if err > 20.0:
                raise RuntimeError(
                    f"goto incomplete err={err:.1f}deg now={self.deg.round(1)}"
                )
            self.center_deg = np.array(target, dtype=float)
            self.last_cmd_deg = self.center_deg.copy()
            self.get_logger().info(
                f"goto done, soft_box recentered at {self.center_deg.round(1)}"
            )
        finally:
            self._goto_active = False

    def on_servo(self, msg: JointTrajectory) -> None:
        if self._goto_active or not self.enable_commands:
            return
        if not msg.points:
            return
        target_rad = np.array(msg.points[0].positions[:6], dtype=float)
        if target_rad.shape[0] < 6:
            return
        target_deg = ros_rad_to_fw_deg(target_rad)

        # Soft joint box around start pose
        lo = self.center_deg - self.half_range
        hi = self.center_deg + self.half_range
        clipped = np.clip(target_deg, lo, hi)
        if np.any(np.abs(clipped - target_deg) > 0.05):
            if not hasattr(self, "_box_warn_t") or time.time() - self._box_warn_t > 2.0:
                self._box_warn_t = time.time()
                hit = np.where(np.abs(clipped - target_deg) > 0.05)[0] + 1
                self.get_logger().warn(
                    f"soft_box clip J{list(hit)} at edge ±{self.half_range:.0f}° "
                    f"center={self.center_deg.round(1)} want={target_deg.round(1)}"
                )
        target_deg = clipped

        # Per-command step clamp vs last command (limits jerk)
        delta = target_deg - self.last_cmd_deg
        max_d = self.max_delta
        big = np.abs(delta) > max_d
        if np.any(big):
            delta = np.clip(delta, -max_d, max_d)
            target_deg = self.last_cmd_deg + delta

        now = time.time()
        if now - self._last_cmd_t < 0.04:  # ~25 Hz max out
            return
        self._last_cmd_t = now
        self.last_cmd_deg = target_deg
        if not hasattr(self, "_cmd_count"):
            self._cmd_count = 0
        self._cmd_count += 1
        if self._cmd_count <= 3 or self._cmd_count % 50 == 0:
            self.get_logger().info(
                f"CDC cmd#{self._cmd_count} deg={np.round(target_deg, 1)}"
            )
        self._move_j(target_deg)


def main() -> None:
    # Optional positional port only; ignore ROS remaps / --ros-args.
    port = "/dev/ttyACM0"
    for a in sys.argv[1:]:
        if a.startswith("-"):
            break
        port = a
        break
    rclpy.init()
    node = CdcServoBridge(port=port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send("!DISABLE", wait=0.3)
        except Exception:
            pass
        try:
            node.ser.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
