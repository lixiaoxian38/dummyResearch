#!/usr/bin/env python3
"""WDLS Follow resolver: Target bus → Spray Aim → JointJog (Servo).

Primary task: optical center on target + standoff (translation only).
Same-cycle secondary: weighted null-space joint preference (favor J5, discourage J1).
J6 locked (camera does not move with J6). Hold → stop JointJog (CDC Quiet).
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = [f"Joint{i}" for i in range(1, 7)]
# Child link of each joint (axis expressed in this link after joint origin).
LINK_AFTER_JOINT = [
    "link1_1_1",
    "link2_1_1",
    "link3_1_1",
    "link4_1_1",
    "link5_1_1",
    "link6_1_1",
]
# URDF axis xyz in the child link frame (see dummy.urdf).
JOINT_AXIS_CHILD = [
    np.array([0.0, 0.0, 1.0]),
    np.array([1.0, 0.0, 0.0]),
    np.array([1.0, 0.0, 0.0]),
    np.array([0.0, -1.0, 0.0]),
    np.array([-1.0, 0.0, 0.0]),
    np.array([0.0, 1.0, 0.0]),
]


def tf_to_matrix(tf_msg) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


class WdlsFollowResolver(Node):
    def __init__(self) -> None:
        super().__init__("wdls_follow_resolver")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("quality_gate", 0.4)
        self.declare_parameter("coast_sec", 0.4)
        # Require continuous good quality this long before first motion (charger false-positive guard).
        self.declare_parameter("confirm_sec", 0.0)
        self.declare_parameter("follow_source", "board")
        self.declare_parameter("desired_marker_in_ee_x", 0.0)
        self.declare_parameter("desired_marker_in_ee_y", 0.0)
        self.declare_parameter("desired_marker_in_ee_z", 0.20)
        self.declare_parameter("desired_charger_in_ee_z", 0.10)
        self.declare_parameter("desired_vlm_in_ee_z", 0.12)
        self.declare_parameter("desired_cup_in_ee_z", 0.10)  # legacy alias
        self.declare_parameter("linear_gain", 1.15)
        self.declare_parameter("max_linear_vel", 0.085)
        self.declare_parameter("max_joint_vel", 0.75)
        self.declare_parameter("near_err_m", 0.035)
        self.declare_parameter("near_gain_min", 0.50)
        self.declare_parameter("hold_xy_m", 0.005)
        self.declare_parameter("hold_z_m", 0.012)
        self.declare_parameter("hold_resume_xy_m", 0.014)
        self.declare_parameter("hold_resume_z_m", 0.025)
        self.declare_parameter("leave_hold_sec", 0.45)
        self.declare_parameter("opt_filter_alpha", 0.42)
        self.declare_parameter("dls_lambda", 0.05)
        self.declare_parameter("nullspace_gain", 0.35)
        # Joint weights for WDLS (larger → less motion). Prefer J5, discourage J1.
        self.declare_parameter(
            "joint_weights",
            [4.0, 1.5, 1.5, 1.2, 0.55, 8.0],
        )
        # Null-space bias toward handeye-ish posture (FW-like ROS rad after conversion
        # is awkward; use soft attractors in ROS rad near handeye [0,0,π/2? wait J3]).
        # handeye FW [0,0,90,0,0,0] → ROS with RAD_VOLUMN J3=π/2 so ROS J3≈0.
        # Prefer small |q1|, encourage negative J5 (look down) via soft attractor 0.
        self.declare_parameter(
            "nullspace_attract_ros",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        # VLM desk start in ROS rad: FW [3, 0, 90, 0, 75, 0].
        self.declare_parameter(
            "vlm_nullspace_attract_ros",
            [0.0524, 0.0, 0.0, 0.0, -1.3090, 0.0],
        )
        self.declare_parameter("lock_j6", True)
        self.declare_parameter("joint_jog_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("max_marker_z", 0.60)
        self.declare_parameter("max_marker_xy", 0.30)
        self.declare_parameter("lost_timeout_sec", 2.5)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        jog_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_jog = self.create_publisher(
            JointJog, str(self.get_parameter("joint_jog_topic").value), jog_qos
        )
        # Always published (including dry_run) for software-in-the-loop tests.
        self.pub_qdot = self.create_publisher(Float64MultiArray, "/tracking/wdls_qdot", 10)
        self.pub_status = self.create_publisher(String, "/tracking/status", 10)

        self._tgt_pose: PoseStamped | None = None
        self._tgt_quality = 0.0
        self._tgt_stamp_s = 0.0
        self._q = np.zeros(6)
        self._have_js = False
        self._opt_filt: np.ndarray | None = None
        self._phase = "approach"
        self._leave_hold_t = 0.0
        self._settle_t0 = 0.0
        self._last_cmd_qdot = np.zeros(6)
        self._coast_t0 = 0.0
        self._coasting = False
        self._confirm_t0 = 0.0
        self._last_marker_ok = False
        self._servo_ready = self.dry_run
        self._servo_setup_attempts = 0
        self._last_warn_ns = 0
        self._last_status_ns = 0

        self.create_subscription(PoseStamped, "/tracking/target_pose", self._on_pose, 10)
        self.create_subscription(Float32, "/tracking/target_quality", self._on_quality, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        if not self.dry_run:
            self._cli_cmd_type = self.create_client(
                ServoCommandType, "/servo_node/switch_command_type"
            )
            self._cli_pause = self.create_client(SetBool, "/servo_node/pause_servo")
            self.create_timer(1.0, self._ensure_servo_joint_mode)

        self.create_timer(1.0 / max(rate_hz, 1.0), self.control_tick)
        self.get_logger().info(
            f"WDLS Follow resolver [{'DRY' if self.dry_run else 'LIVE'}] "
            f"optical→JointJog board_z="
            f"{float(self.get_parameter('desired_marker_in_ee_z').value):.2f}m "
            f"charger_z={float(self.get_parameter('desired_charger_in_ee_z').value):.2f}m "
            f"lock_j6={bool(self.get_parameter('lock_j6').value)}"
        )

    def _follow_source(self) -> str:
        src = str(self.get_parameter("follow_source").value).strip().lower()
        if src in ("charger", "cup"):
            return "charger"
        if src == "vlm":
            return "vlm"
        return "board"

    def _desired_z(self) -> float:
        src = self._follow_source()
        if src == "charger":
            return float(self.get_parameter("desired_charger_in_ee_z").value)
        if src == "vlm":
            return float(self.get_parameter("desired_vlm_in_ee_z").value)
        return float(self.get_parameter("desired_marker_in_ee_z").value)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._tgt_pose = msg
        self._tgt_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def _on_quality(self, msg: Float32) -> None:
        # Mux publishes q=0 when the board is briefly stale (no pose). Ignoring
        # those zeros avoids LOST/coast thrash; lost is decided by pose age.
        q = float(msg.data)
        if q < 1e-6:
            # VLM query changes must drop the previous point at once.
            # Waiting out lost_timeout made the arm revisit the old object.
            if self._follow_source() == "vlm":
                self._tgt_pose = None
                self._tgt_stamp_s = 0.0
                self._tgt_quality = 0.0
                self._opt_filt = None
                self._last_marker_ok = False
            return
        self._tgt_quality = q

    def _on_js(self, msg: JointState) -> None:
        if len(msg.position) < 6:
            return
        name_to_i = {n: i for i, n in enumerate(msg.name)} if msg.name else {}
        q = np.zeros(6)
        for i, n in enumerate(JOINT_NAMES):
            if n in name_to_i:
                q[i] = float(msg.position[name_to_i[n]])
            elif i < len(msg.position):
                q[i] = float(msg.position[i])
        self._q = q
        self._have_js = True

    def _ensure_servo_joint_mode(self) -> None:
        if self._servo_ready or self.dry_run:
            return
        self._servo_setup_attempts += 1
        if self._servo_setup_attempts > 30:
            self.get_logger().error("Giving up enabling Servo JOINT_JOG mode")
            self._servo_ready = True
            return
        if not self._cli_cmd_type.service_is_ready() or not self._cli_pause.service_is_ready():
            if self._servo_setup_attempts % 5 == 1:
                self.get_logger().warn("Waiting for /servo_node services...")
            return
        req = ServoCommandType.Request()
        # Jazzy enum is JOINT_JOG (not JOINT).
        req.command_type = ServoCommandType.Request.JOINT_JOG
        fut = self._cli_cmd_type.call_async(req)

        def _after_type(_fut) -> None:
            try:
                res = _fut.result()
            except Exception as exc:
                self.get_logger().warn(f"switch_command_type failed: {exc}")
                return
            if not res or not res.success:
                self.get_logger().warn(f"switch_command_type JOINT_JOG rejected: {res}")
                return
            preq = SetBool.Request()
            preq.data = False
            pfut = self._cli_pause.call_async(preq)

            def _after_pause(__fut) -> None:
                try:
                    pres = __fut.result()
                except Exception as exc:
                    self.get_logger().warn(f"pause_servo failed: {exc}")
                    return
                self._servo_ready = True
                self.get_logger().info(
                    f"MoveIt Servo JOINT_JOG enabled ({pres.message if pres else ''})"
                )

            pfut.add_done_callback(_after_pause)

        fut.add_done_callback(_after_type)

    def _lookup(self, parent: str, child: str):
        return self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())

    def _camera_jacobians(self) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Optical rotation, 3x6 linear Jacobian, 3x6 angular Jacobian, all in base."""
        try:
            T_base_opt = tf_to_matrix(self._lookup(self.base_frame, self.optical_frame))
        except Exception:
            return None
        p_opt = T_base_opt[:3, 3]
        Jv = np.zeros((3, 6))
        Jw = np.zeros((3, 6))
        for i, link in enumerate(LINK_AFTER_JOINT):
            try:
                T = tf_to_matrix(self._lookup(self.base_frame, link))
            except Exception:
                return None
            z = T[:3, :3] @ JOINT_AXIS_CHILD[i]
            z = z / max(float(np.linalg.norm(z)), 1e-9)
            p_i = T[:3, 3]
            Jv[:, i] = np.cross(z, p_opt - p_i)
            Jw[:, i] = z
        if bool(self.get_parameter("lock_j6").value):
            Jv[:, 5] = 0.0
            Jw[:, 5] = 0.0
        return T_base_opt[:3, :3], Jv, Jw

    def _geometric_jacobian_optical(self) -> np.ndarray | None:
        """3x6 linear Jacobian of optical-frame origin in base."""
        jac = self._camera_jacobians()
        if jac is None:
            return None
        return jac[1]

    @staticmethod
    def _feature_jacobian_xy(
        p_cam: np.ndarray, R: np.ndarray, Jv: np.ndarray, Jw: np.ndarray
    ) -> np.ndarray:
        """2x6 map from qdot to (px_dot, py_dot) of a fixed point in the optical frame.

        Translation-only Jacobians ignore that J2/J3 also sweep the view, so a
        corner box runs away. This includes that sweep.
        """
        px, py, pz = (float(p_cam[0]), float(p_cam[1]), float(p_cam[2]))
        Rt = R.T
        Js = np.zeros((2, 6))
        for i in range(6):
            v = Rt @ Jv[:, i]
            w = Rt @ Jw[:, i]
            Js[0, i] = -v[0] - w[1] * pz + w[2] * py
            Js[1, i] = -v[1] + w[0] * pz - w[2] * px
        return Js

    @staticmethod
    def _wdls(J: np.ndarray, v: np.ndarray, w_joint: np.ndarray, lam: float) -> np.ndarray:
        """Weighted damped least-squares: min ||Jx-v||^2 + λ²||W^{1/2}x||^2."""
        w = np.asarray(w_joint, dtype=float).clip(1e-3, 1e6)
        Winv = np.diag(1.0 / w)
        JWinv = J @ Winv
        A = JWinv @ J.T + (lam * lam) * np.eye(J.shape[0])
        try:
            y = np.linalg.solve(A, v)
        except np.linalg.LinAlgError:
            y = np.linalg.lstsq(A, v, rcond=None)[0]
        return Winv @ J.T @ y

    def _nullspace_term(self, J: np.ndarray, qdot_task: np.ndarray) -> np.ndarray:
        gain = float(self.get_parameter("nullspace_gain").value)
        if gain < 1e-6:
            return np.zeros(6)
        key = (
            "vlm_nullspace_attract_ros"
            if self._follow_source() == "vlm"
            else "nullspace_attract_ros"
        )
        attract = np.asarray(self.get_parameter(key).value, dtype=float)
        if attract.shape[0] < 6:
            attract = np.zeros(6)
        # Board: stronger on J1 (anti-orbit). VLM: hold J2/J3 near the desk start.
        if self._follow_source() == "vlm":
            k = np.array([0.5, 1.4, 1.4, 0.35, 0.2, 0.0])
        else:
            k = np.array([1.0, 0.4, 0.4, 0.3, 0.15, 0.0])
        qdot_pref = -gain * k * (self._q - attract[:6])
        if bool(self.get_parameter("lock_j6").value):
            qdot_pref[5] = 0.0
        # Projector onto null space of J (task-preserving).
        try:
            Jplus = np.linalg.pinv(J)
        except np.linalg.LinAlgError:
            return np.zeros(6)
        N = np.eye(6) - Jplus @ J
        return N @ qdot_pref

    def _publish_qdot_debug(self, qdot: np.ndarray) -> None:
        dbg = Float64MultiArray()
        dbg.data = [float(x) for x in np.asarray(qdot, dtype=float).reshape(-1)[:6]]
        while len(dbg.data) < 6:
            dbg.data.append(0.0)
        self.pub_qdot.publish(dbg)

    def _publish_zero_jog(self, stamp) -> None:
        self._publish_qdot_debug(np.zeros(6))
        if self.dry_run:
            return
        msg = JointJog()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.joint_names = list(JOINT_NAMES)
        msg.velocities = [0.0] * 6
        self.pub_jog.publish(msg)

    def _publish_status(self, **kwargs) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_status_ns < 2e8 and kwargs.get("phase") == self._phase:
            return
        self._last_status_ns = now_ns
        dz = self._desired_z()
        src = self._follow_source()
        phase = str(kwargs.get("phase", self._phase))
        z = kwargs.get("z")
        xy = kwargs.get("xy")
        qdot_norm = float(kwargs.get("qdot_norm", float(np.linalg.norm(self._last_cmd_qdot))))
        opt_z_cm = None if z is None else float(z) * 100.0
        xy_cm = None if xy is None else float(xy) * 100.0
        z_err_cm = None if z is None else (float(z) - dz) * 100.0
        on_tgt = phase == "hold"
        payload = {
            "resolver": "wdls",
            "phase": phase,
            "dry_run": self.dry_run,
            "follow_source": src,
            "spray": on_tgt,
            "on_target": on_tgt,
            "moving": qdot_norm > 1e-3 and phase in ("track", "approach", "coast"),
            "opt_z_cm": opt_z_cm,
            "xy_cm": xy_cm,
            "z_err_cm": z_err_cm,
            "standoff_m": dz,
            **kwargs,
        }
        # Keep HUD field names authoritative even if kwargs used short aliases.
        payload["phase"] = phase
        payload["spray"] = on_tgt
        payload["on_target"] = on_tgt
        if opt_z_cm is not None:
            payload["opt_z_cm"] = opt_z_cm
        if xy_cm is not None:
            payload["xy_cm"] = xy_cm
        if z_err_cm is not None:
            payload["z_err_cm"] = z_err_cm
        out = String()
        out.data = json.dumps(payload)
        self.pub_status.publish(out)

    def control_tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        now_s = self.get_clock().now().nanoseconds / 1e9
        quality_gate = float(self.get_parameter("quality_gate").value)
        coast_sec = float(self.get_parameter("coast_sec").value)
        confirm_sec = float(self.get_parameter("confirm_sec").value)
        lost_timeout = float(self.get_parameter("lost_timeout_sec").value)

        if not self._have_js:
            self._publish_zero_jog(stamp)
            return

        age = now_s - self._tgt_stamp_s if self._tgt_stamp_s else 1e9
        # Pose age is the source of truth. Mux q=0 heartbeats must not force LOST
        # (that caused DETECTED/LOST thrash and never reaching Hold).
        lost = self._tgt_pose is None or age > lost_timeout
        if not lost and self._tgt_quality < quality_gate and age < 0.15:
            # Fresh pose but bad quality — treat as lost.
            lost = True
        if lost:
            self._confirm_t0 = 0.0
            if self._last_marker_ok:
                self.get_logger().warn(
                    f"TARGET LOST q={self._tgt_quality:.2f} — coast {coast_sec:.2f}s"
                )
                self._coast_t0 = now_s
                self._coasting = True
                self._last_marker_ok = False
            elapsed = now_s - self._coast_t0 if self._coast_t0 else 1e9
            if self._coasting and coast_sec > 1e-4 and elapsed < coast_sec:
                scale = max(0.0, 1.0 - elapsed / coast_sec)
                qdot = self._last_cmd_qdot * scale
                self._phase = "coast"
                self._emit_jog(stamp, qdot)
            else:
                self._coasting = False
                self._phase = "lost"
                self._last_cmd_qdot[:] = 0.0
                self._publish_zero_jog(stamp)
            self._publish_status(board_ok=False, quality=float(self._tgt_quality))
            return

        mx = float(self._tgt_pose.pose.position.x)
        my = float(self._tgt_pose.pose.position.y)
        mz = float(self._tgt_pose.pose.position.z)
        max_xy = float(self.get_parameter("max_marker_xy").value)
        max_z = float(self.get_parameter("max_marker_z").value)
        if abs(mx) > max_xy or abs(my) > max_xy or mz > max_z or mz < 0.05:
            self._confirm_t0 = 0.0
            self._publish_zero_jog(stamp)
            self._phase = "reject"
            return

        # Hold still until quality has been good for confirm_sec (blocks flicker-driven motion).
        if not self._last_marker_ok:
            if self._confirm_t0 <= 0.0:
                self._confirm_t0 = now_s
            if confirm_sec > 1e-4 and (now_s - self._confirm_t0) < confirm_sec:
                self._phase = "confirm"
                self._last_cmd_qdot[:] = 0.0
                self._publish_zero_jog(stamp)
                self._publish_status(
                    board_ok=False, phase="confirm", quality=float(self._tgt_quality)
                )
                return

        # Frozen VLM point is geometric. Filtering it lags the motion and hunts.
        alpha = 1.0 if self._follow_source() == "vlm" else float(
            np.clip(self.get_parameter("opt_filter_alpha").value, 0.05, 1.0)
        )
        raw = np.array([mx, my, mz], dtype=float)
        if self._opt_filt is None:
            self._opt_filt = raw
        else:
            self._opt_filt = (1.0 - alpha) * self._opt_filt + alpha * raw
        mx, my, mz = (float(self._opt_filt[i]) for i in range(3))

        dx = float(self.get_parameter("desired_marker_in_ee_x").value)
        dy = float(self.get_parameter("desired_marker_in_ee_y").value)
        dz = self._desired_z()
        xy_err = math.hypot(mx - dx, my - dy)
        # VLM only recenters the box. Depth is reported, not driven.
        center_only = self._follow_source() == "vlm"
        z_err = 0.0 if center_only else abs(mz - dz)
        # ~20 px / ~45 px at desk range (z≈0.26, fx≈600): stop hunting, hold AIM in box.
        hold_xy = 0.008 if center_only else float(self.get_parameter("hold_xy_m").value)
        hold_z = float(self.get_parameter("hold_z_m").value)
        resume_xy = float(self.get_parameter("hold_resume_xy_m").value)
        if center_only:
            resume_xy = max(resume_xy, 0.020)
        resume_z = float(self.get_parameter("hold_resume_z_m").value)
        leave_hold = float(self.get_parameter("leave_hold_sec").value)
        if center_only:
            leave_hold = max(leave_hold, 0.30)
        on_target = xy_err <= hold_xy and z_err <= hold_z

        if not self._last_marker_ok:
            self.get_logger().info(
                f"TARGET DETECTED q={self._tgt_quality:.2f} opt=[{mx:.3f},{my:.3f},{mz:.3f}]"
            )
        self._last_marker_ok = True
        self._confirm_t0 = 0.0
        self._coasting = False

        if self._phase == "hold":
            leaving = xy_err > resume_xy or (not center_only and z_err > resume_z)
            if leaving:
                if self._leave_hold_t <= 0.0:
                    self._leave_hold_t = now_s
                if (now_s - self._leave_hold_t) < leave_hold:
                    self._last_cmd_qdot[:] = 0.0
                    self._publish_zero_jog(stamp)
                    self._publish_status(
                        board_ok=True, phase="hold", xy=xy_err, z=mz, quality=self._tgt_quality
                    )
                    return
                self._phase = "approach"
                self._leave_hold_t = 0.0
            else:
                self._leave_hold_t = 0.0
                self._last_cmd_qdot[:] = 0.0
                self._publish_zero_jog(stamp)
                self._publish_status(
                    board_ok=True, phase="hold", xy=xy_err, z=mz, quality=self._tgt_quality
                )
                return

        if on_target:
            if self._settle_t0 <= 0.0:
                self._settle_t0 = now_s
            settle_need = 0.20 if center_only else 0.0
            if (now_s - self._settle_t0) < settle_need:
                self._last_cmd_qdot[:] = 0.0
                self._publish_zero_jog(stamp)
                self._publish_status(
                    board_ok=True, phase="settle", xy=xy_err, z=mz, quality=self._tgt_quality
                )
                return
            if self._phase != "hold":
                self.get_logger().info(
                    f"ON TARGET — hold xy={xy_err*100:.1f}cm z={mz*100:.1f}cm (CDC Quiet)"
                )
            self._phase = "hold"
            self._leave_hold_t = 0.0
            self._settle_t0 = 0.0
            self._last_cmd_qdot[:] = 0.0
            self._publish_zero_jog(stamp)
            self._publish_status(
                board_ok=True, phase="hold", xy=xy_err, z=mz, quality=self._tgt_quality
            )
            return
        self._settle_t0 = 0.0

        # Camera-frame velocity that reduces optical Spray Aim error.
        # Marker at +x ⇒ move camera +x so axis catches up (PBVS point feature).
        gain = float(self.get_parameter("linear_gain").value)
        if center_only:
            gain = min(gain, 0.85)
        vz = 0.0 if center_only else (mz - dz)
        v_cam = gain * np.array([mx - dx, my - dy, vz], dtype=float)
        err_n = float(np.linalg.norm(v_cam) / max(gain, 1e-6))
        near_m = max(1e-4, float(self.get_parameter("near_err_m").value))
        if center_only:
            near_m = max(near_m, 0.06)
        near_min = (
            0.0
            if center_only
            else float(np.clip(self.get_parameter("near_gain_min").value, 0.05, 1.0))
        )
        v_cam *= float(np.clip(err_n / near_m, near_min, 1.0))
        v_cam *= float(np.clip(self._tgt_quality, 0.0, 1.0))
        # Fade command while waiting for the next detection (brief occlusion).
        if lost_timeout > 1e-3 and age > 0.2:
            v_cam *= float(np.clip(1.0 - (age - 0.2) / max(lost_timeout - 0.2, 1e-3), 0.0, 1.0))
        max_lin = float(self.get_parameter("max_linear_vel").value)
        if center_only:
            max_lin = min(max_lin, 0.05)
        nrm = float(np.linalg.norm(v_cam))
        if nrm > max_lin and nrm > 1e-9:
            v_cam *= max_lin / nrm

        jac = self._camera_jacobians()
        if jac is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_warn_ns > 2e9:
                self._last_warn_ns = now_ns
                self.get_logger().warn("Waiting for TF: camera Jacobian")
            self._publish_zero_jog(stamp)
            return
        R, Jv, Jw = jac

        w_joint = np.asarray(self.get_parameter("joint_weights").value, dtype=float)
        if w_joint.shape[0] < 6:
            w_joint = np.array([4.0, 1.5, 1.5, 1.2, 0.55, 8.0])
        w_joint = w_joint.copy()
        lam = float(self.get_parameter("dls_lambda").value)
        if center_only:
            # Drive the box center toward AIM. J5 pitches the view; J2/J3 stay dear.
            p_cam = np.array([mx, my, mz], dtype=float)
            J = self._feature_jacobian_xy(p_cam, R, Jv, Jw)
            task = -v_cam[:2]
            attract = np.asarray(
                self.get_parameter("vlm_nullspace_attract_ros").value, dtype=float
            )
            w_joint[1] = max(float(w_joint[1]), 8.0)
            w_joint[2] = max(float(w_joint[2]), 8.0)
            if attract.shape[0] >= 3 and self._have_js:
                for i in (1, 2):
                    err = abs(float(self._q[i]) - float(attract[i]))
                    w_joint[i] *= 1.0 + (err / math.radians(18.0)) ** 2
            qdot = self._wdls(J, task, w_joint, lam)
            qdot = qdot + self._nullspace_term(J, qdot)
        else:
            J = Jv
            v_base = R @ v_cam
            qdot = self._wdls(J, v_base, w_joint, lam)
            qdot = qdot + self._nullspace_term(J, qdot)
        if bool(self.get_parameter("lock_j6").value):
            qdot[5] = 0.0
        max_j = float(self.get_parameter("max_joint_vel").value)
        qdot = np.clip(qdot, -max_j, max_j)
        if center_only:
            # Shoulder and elbow may creep to help, not fold the arm away.
            qdot[1] = float(np.clip(qdot[1], -0.22, 0.22))
            qdot[2] = float(np.clip(qdot[2], -0.22, 0.22))

        self._phase = "track"
        self._last_cmd_qdot = qdot.copy()
        self._emit_jog(stamp, qdot)
        self._publish_status(
            board_ok=True,
            phase="track",
            xy=xy_err,
            z=mz,
            quality=self._tgt_quality,
            qdot_norm=float(np.linalg.norm(qdot)),
        )

    def _emit_jog(self, stamp, qdot: np.ndarray) -> None:
        self._publish_qdot_debug(qdot)
        if self.dry_run:
            return
        msg = JointJog()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.joint_names = list(JOINT_NAMES)
        msg.velocities = [float(x) for x in qdot.tolist()]
        self.pub_jog.publish(msg)


def main() -> None:
    rclpy.init()
    node = WdlsFollowResolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
