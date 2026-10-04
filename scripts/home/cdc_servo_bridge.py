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

import os
import re
import sys
import termios
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from joint_limits_fw import FOLLOW_LIMITS_FW_DEG, JOINT_LIMITS_FW_DEG, SOFTWARE_MARGIN_FW_DEG

HANDEYE_FW = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
STOW_FW = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])

RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])
NAMES = [f"Joint{i}" for i in range(1, 7)]

_ABS_LO = np.array([lo for lo, _ in JOINT_LIMITS_FW_DEG], dtype=float)
_ABS_HI = np.array([hi for _, hi in JOINT_LIMITS_FW_DEG], dtype=float)
_FOL_LO = np.array([lo for lo, _ in FOLLOW_LIMITS_FW_DEG], dtype=float)
_FOL_HI = np.array([hi for _, hi in FOLLOW_LIMITS_FW_DEG], dtype=float)


def fw_deg_to_ros_rad(deg: np.ndarray) -> np.ndarray:
    return (np.deg2rad(deg) / RAD_DIRECT) - RAD_VOLUMN


def ros_rad_to_fw_deg(rad: np.ndarray) -> np.ndarray:
    return np.rad2deg((rad + RAD_VOLUMN) * RAD_DIRECT)


class CdcServoBridge(Node):
    def __init__(self, port: str = "/dev/ttyACM0") -> None:
        super().__init__("cdc_servo_bridge")
        self.declare_parameter("port", port)
        self.declare_parameter("move_speed", 18.0)  # firmware units; stream often, not huge hops
        self.declare_parameter("max_delta_deg", 2.2)  # small steps at high rate = smooth
        # 0 = no start-centered box; only hardware-15° absolute clip
        self.declare_parameter("joint_soft_half_range_deg", 0.0)
        self.declare_parameter("enable_commands", True)
        # Translation-only follow used to freeze both; orientation follow needs J4.
        # J6 stays locked by default so flange spin does not stack with wrist roll.
        self.declare_parameter("lock_j4", True)
        self.declare_parameter("lock_j6", True)

        port = str(self.get_parameter("port").value)
        self.move_speed = float(self.get_parameter("move_speed").value)
        self.max_delta = float(self.get_parameter("max_delta_deg").value)
        self.half_range = float(self.get_parameter("joint_soft_half_range_deg").value)
        self.enable_commands = bool(self.get_parameter("enable_commands").value)
        self.lock_j4 = bool(self.get_parameter("lock_j4").value)
        self.lock_j6 = bool(self.get_parameter("lock_j6").value)

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
        self._servo_hold = False
        self._running = True
        self._pending_move: np.ndarray | None = None
        self._want_jpos = False
        self._cmd_count = 0
        self._last_sent_deg = np.full(6, np.nan)
        # Snapshot roll axes for optional freeze (see lock_j4 / lock_j6).
        self._lock_roll_axes(self.deg)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointTrajectory, "/servo_node/command", self.on_servo, 10)
        self.create_service(Trigger, "~/goto_handeye", self._srv_goto_handeye)
        self.create_service(Trigger, "~/goto_stow", self._srv_goto_stow)
        self.create_service(Trigger, "~/hold_servo", self._srv_hold)
        self.create_service(Trigger, "~/release_servo", self._srv_release)
        self.create_subscription(Int32, "~/nudge", self.on_nudge, 10)
        self.create_subscription(Float64MultiArray, "~/jog_fw", self.on_jog_fw, 10)
        self.create_timer(0.05, self.tick_pub)
        self.create_timer(0.25, self.tick_poll)
        self._writer = threading.Thread(target=self._serial_loop, daemon=True, name="cdc-write")
        self._writer.start()
        # One hold of the pose we just read — firmware drops torque if stream stops.
        self._queue_move(self.deg)

        self.get_logger().info(
            f"CDC servo bridge on {port}: speed={self.move_speed} "
            f"max_delta={self.max_delta}deg start_box=±{self.half_range}deg "
            f"abs=HW-{SOFTWARE_MARGIN_FW_DEG}° enable_commands={self.enable_commands} "
            f"lock_j4={self.lock_j4} lock_j6={self.lock_j6} "
            f"center={self.center_deg.round(1)}"
        )

    def _send(self, cmd: str, wait: float = 0.15) -> str:
        """Serial I/O. Stream writes use wait=0 (no sleep). Never call from ROS callbacks."""
        with self._lock:
            if not cmd.endswith("\n"):
                cmd += "\n"
            try:
                if not self.ser.is_open:
                    self._last_send_ok = False
                    self._fail_t = time.time()
                    return ""
                if wait >= 0.05:
                    self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii", errors="ignore"))
                if wait > 1e-4:
                    time.sleep(wait)
                raw = self.ser.read(400).decode(errors="replace") if wait >= 0.05 else ""
                self._last_send_ok = True
                return raw
            except (serial.SerialException, OSError, termios.error) as exc:
                self._last_send_ok = False
                self._fail_t = time.time()
                self.get_logger().error(f"CDC serial error: {exc}")
                time.sleep(0.25)
                return ""

    def _queue_move(self, deg: np.ndarray) -> None:
        with self._lock:
            self._pending_move = np.asarray(deg, dtype=float).copy()

    def _serial_loop(self) -> None:
        """Single writer: latest setpoint wins; skip no-change; cap ~25 Hz."""
        min_dt = 0.04
        eps = 0.08
        while self._running:
            if self._goto_active:
                time.sleep(0.02)
                continue
            with self._lock:
                pending = self._pending_move
                self._pending_move = None
                want_jpos = self._want_jpos
                self._want_jpos = False
            if pending is not None:
                if np.isfinite(self._last_sent_deg).all():
                    delta = float(np.max(np.abs(pending - self._last_sent_deg)))
                else:
                    delta = 1e9
                if delta < eps:
                    with self._lock:
                        self.deg = pending.copy()
                    time.sleep(0.015)
                    continue
                wait = min_dt - (time.time() - self._last_cmd_t)
                if wait > 0:
                    time.sleep(wait)
                ok = self._move_j(pending)
                if ok:
                    self._last_sent_deg = pending.copy()
                    self.last_cmd_deg = pending.copy()
                    with self._lock:
                        self.deg = pending.copy()
                    self._cmd_count += 1
                    if self._cmd_count <= 3 or self._cmd_count % 50 == 0:
                        self.get_logger().info(
                            f"CDC cmd#{self._cmd_count} deg={np.round(pending, 1)}"
                        )
                continue
            if want_jpos and (time.time() - self._last_cmd_t) > 0.45:
                got = self._get_jpos()
                if got is not None:
                    with self._lock:
                        self.deg = np.array(got, dtype=float)
            time.sleep(0.02)

    def _get_jpos(self) -> list[float] | None:
        raw = self._send("#GETJPOS", wait=0.12)
        if not raw:
            return None
        nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
        return nums[:6] if len(nums) >= 6 else None

    def _move_j(self, deg: np.ndarray, speed: float | None = None) -> bool:
        body = ",".join(f"{d:.2f}" for d in deg.tolist())
        sp = self.move_speed if speed is None else speed
        self._send(f">{body},{sp:.1f}", wait=0.0)
        self._last_cmd_t = time.time()
        return self._last_send_ok

    def tick_poll(self) -> None:
        if self._goto_active:
            return
        if time.time() - self._last_cmd_t < 0.45:
            return
        if not self._last_send_ok and time.time() - self._fail_t < 1.5:
            return
        with self._lock:
            self._want_jpos = True

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

    def _srv_hold(self, _req, resp):
        self._servo_hold = True
        resp.success = True
        resp.message = "servo cmds ignored"
        return resp

    def _srv_release(self, _req, resp):
        self._servo_hold = False
        self.lock_j4 = bool(self.get_parameter("lock_j4").value)
        self.lock_j6 = bool(self.get_parameter("lock_j6").value)
        self._lock_roll_axes(self.deg)
        parts = []
        if self.lock_j4:
            parts.append(f"J4@{self._j4_fw:.1f}")
        if self.lock_j6:
            parts.append(f"J6@{self._j6_fw:.1f}")
        freeze = ",".join(parts) if parts else "none"
        resp.success = True
        resp.message = f"servo cmds enabled; freeze={freeze}"
        return resp

    def _lock_roll_axes(self, deg: np.ndarray) -> None:
        self._j4_fw = float(deg[3])
        self._j6_fw = float(deg[5])

    def _refresh_lock_flags(self) -> None:
        self.lock_j4 = bool(self.get_parameter("lock_j4").value)
        self.lock_j6 = bool(self.get_parameter("lock_j6").value)

    def on_nudge(self, msg: Int32) -> None:
        """Jog one joint by ±3° without opening a second serial writer.

        data = ±1..±6 (sign is direction). Ignored during goto.
        """
        if self._goto_active or not self.enable_commands:
            return
        raw = int(msg.data)
        if raw == 0:
            return
        idx = abs(raw) - 1
        if idx < 0 or idx > 5:
            return
        sign = 1.0 if raw > 0 else -1.0
        q = self.deg.copy()
        q[idx] = float(np.clip(q[idx] + sign * 3.0, _ABS_LO[idx], _ABS_HI[idx]))
        self._servo_hold = True
        self._queue_move(q)
        self.get_logger().info(f"nudge J{idx+1} -> {q[idx]:.1f}°")

    def on_jog_fw(self, msg: Float64MultiArray) -> None:
        """Absolute FW-deg from HUD sliders. Preempts Servo so streams cannot pile up."""
        if self._goto_active or not self.enable_commands:
            return
        if len(msg.data) < 6:
            return
        self._servo_hold = True
        q = np.clip(np.asarray(msg.data[:6], dtype=float), _ABS_LO, _ABS_HI)
        self._queue_move(q)

    def _goto_preset(self, name: str, target: np.ndarray, resp) -> Trigger.Response:
        if self._goto_active:
            resp.success = False
            resp.message = "goto already running"
            return resp
        try:
            self._goto_fw(target)
            resp.success = True
            resp.message = f"at {name}"
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _goto_fw(self, target: np.ndarray, steps: int = 8) -> None:
        """One firmware target + wait. Mid-path hops / racing streams are jerky or stop short."""
        del steps
        self._goto_active = True
        self._servo_hold = True
        try:
            target = np.clip(np.asarray(target, dtype=float), _ABS_LO, _ABS_HI)
            got = self._get_jpos()
            pos = np.array(got, dtype=float) if got is not None else self.last_cmd_deg.copy()
            self.get_logger().info(
                f"goto {np.round(pos, 1)} -> {np.round(target, 1)}"
            )
            speed = 36.0
            err0 = float(np.max(np.abs(pos - target)))
            if not self._move_j(target, speed=speed):
                raise RuntimeError("CDC write failed — arm did not move")
            time.sleep(max(err0 / speed + 1.2, 1.5))
            got = self._get_jpos()
            if got is not None:
                self.deg = np.array(got, dtype=float)
            err = float(np.max(np.abs(self.deg - target)))
            if err > 5.0:
                self._move_j(target, speed=speed)
                time.sleep(max(err / speed + 0.8, 1.0))
                got = self._get_jpos()
                if got is not None:
                    self.deg = np.array(got, dtype=float)
                err = float(np.max(np.abs(self.deg - target)))
            if err > 10.0:
                raise RuntimeError(
                    f"goto incomplete err={err:.1f}deg now={self.deg.round(1)}"
                )
            self.center_deg = np.array(target, dtype=float)
            self.last_cmd_deg = self.center_deg.copy()
            self._lock_roll_axes(self.center_deg)
            self.get_logger().info(f"goto done at {self.center_deg.round(1)} err={err:.1f}°")
        finally:
            self._goto_active = False

    def on_servo(self, msg: JointTrajectory) -> None:
        if self._goto_active or self._servo_hold or not self.enable_commands:
            return
        if not msg.points:
            return
        target_rad = np.array(msg.points[0].positions[:6], dtype=float)
        if target_rad.shape[0] < 6:
            return
        target_deg = ros_rad_to_fw_deg(target_rad)
        self._refresh_lock_flags()
        # Optionally freeze wrist/flange roll (translation-only follow).
        if self.lock_j4:
            target_deg[3] = float(self._j4_fw)
        else:
            self._j4_fw = float(target_deg[3])
        if self.lock_j6:
            target_deg[5] = float(self._j6_fw)
        else:
            self._j6_fw = float(target_deg[5])
        # Clip only the axis that hits the box; inward commands still pass.
        fol = np.clip(target_deg, _FOL_LO, _FOL_HI)
        if np.any(np.abs(fol - target_deg) > 0.2):
            if not hasattr(self, "_fol_warn_t") or time.time() - self._fol_warn_t > 2.0:
                self._fol_warn_t = time.time()
                hit = np.where(np.abs(fol - target_deg) > 0.2)[0] + 1
                self.get_logger().warn(
                    f"follow_box clip J{list(hit)} want={target_deg.round(1)} "
                    f"-> {fol.round(1)} (J5 −85…75°, J1 no fake ±120 wall)"
                )
        target_deg = fol

        # Optional start-centered box (off when half_range<=0). Follow uses abs only.
        if self.half_range > 1e-3:
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

        # Clip only the joints that hit the bumper. Holding the other axes
        # made the arm freeze when J5 sat at 88.5° (last LIVE).
        abs_clipped = np.clip(target_deg, _ABS_LO, _ABS_HI)
        saturated = np.abs(abs_clipped - target_deg) > 0.05
        if np.any(saturated):
            if not hasattr(self, "_abs_warn_t") or time.time() - self._abs_warn_t > 2.0:
                self._abs_warn_t = time.time()
                hit = np.where(saturated)[0] + 1
                self.get_logger().warn(
                    f"abs_limit clip J{list(hit)} want={target_deg.round(1)} "
                    f"-> {abs_clipped.round(1)}"
                )
        target_deg = abs_clipped

        # Per-command step clamp vs last command (limits jerk)
        delta = target_deg - self.last_cmd_deg
        max_d = self.max_delta
        big = np.abs(delta) > max_d
        if np.any(big):
            delta = np.clip(delta, -max_d, max_d)
            target_deg = self.last_cmd_deg + delta

        self.last_cmd_deg = target_deg
        with self._lock:
            self.deg = target_deg.copy()
        self._queue_move(target_deg)


def main() -> None:
    # Optional positional port only; ignore ROS remaps / --ros-args.
    port = os.environ.get("PORT", "")
    for a in sys.argv[1:]:
        if a.startswith("-"):
            break
        port = a
        break
    if not port or not Path(port).exists():
        acms = sorted(Path("/dev").glob("ttyACM*"))
        if not acms:
            raise SystemExit("no /dev/ttyACM* — Dummy USB CDC missing")
        port = str(acms[-1])
    rclpy.init()
    node = CdcServoBridge(port=port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
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
