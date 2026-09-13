#!/usr/bin/env python3
"""Position-based visual servoing: keep EE at a fixed pose relative to ArUco marker.

Uses TF tree (base_link, link6_1_1, camera_marker) and publishes TwistStamped to
MoveIt Servo (/servo_node/delta_twist_cmds). With dry_run:=true (default), only
publishes /tracking/desired_ee_pose and TF desired_ee — no motion commands.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

from dummy_vision.follow_trace import FollowTrace, ros_rad_to_fw_deg


def pose_from_matrix(mat: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    q = tf_transformations.quaternion_from_matrix(mat)
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(mat[0, 3])
    pose.pose.position.y = float(mat[1, 3])
    pose.pose.position.z = float(mat[2, 3])
    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])
    return pose


def tf_to_matrix(tf_msg) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


def rotation_error_axis_angle(q_cur: np.ndarray, q_des: np.ndarray) -> np.ndarray:
    """Angular error (rad) in base frame as axis-angle vector (xyz)."""
    q_err = np.asarray(
        tf_transformations.quaternion_multiply(
            q_des, tf_transformations.quaternion_inverse(q_cur)
        ),
        dtype=float,
    )
    # q = [x,y,z,w]; angle from w
    w = float(np.clip(q_err[3], -1.0, 1.0))
    angle = 2.0 * math.acos(w)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    s = math.sqrt(max(1.0 - w * w, 0.0))
    if s < 1e-6:
        return np.zeros(3)
    axis = q_err[:3] / s
    return axis * angle


class ArucoServoTracker(Node):
    def __init__(self):
        super().__init__("aruco_servo_tracker")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "link6_1_1")
        self.declare_parameter("marker_frame", "camera_marker")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("follow_orientation", True)
        # ee: flange (link6) aims at board; optical: keep board on camera axis.
        self.declare_parameter("control_frame", "ee")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        # If true (optical mode), keep current camera-to-board distance (XY only).
        self.declare_parameter("hold_current_distance", False)

        # Dummy Joint6 axis is link6 +Y (URDF). Camera sits ~1.3 cm off that axis
        # and looks along -Y, so aiming -Y at the board keeps the marker in view.
        # Standoff 0.20 m is along that axis, flange plane ⟂ J6.
        self.declare_parameter("desired_marker_in_ee_x", 0.0)
        self.declare_parameter("desired_marker_in_ee_y", 0.0)
        self.declare_parameter("desired_marker_in_ee_z", 0.20)
        # Desired marker orientation relative to control frame (RPY, rad).
        self.declare_parameter("desired_marker_rpy", [0.0, 0.0, 0.0])

        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("max_linear_vel", 0.05)
        self.declare_parameter("max_angular_vel", 0.25)
        self.declare_parameter("pos_deadband", 0.0025)
        self.declare_parameter("ori_deadband", 0.02)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("max_marker_z", 0.50)
        self.declare_parameter("max_marker_xy", 0.18)
        self.declare_parameter("max_marker_jump", 0.12)
        # Visible-set guard. Hysteresis: hold (do not invert) when XY is large.
        self.declare_parameter("keep_in_view_xy", 0.10)
        self.declare_parameter("keep_in_view_resume_xy", 0.06)
        self.declare_parameter("leave_view_step", 0.012)
        # Lock one EE goal and drive. Do NOT replan just because EE cartesian
        # is close — that chased ArUco noise and walked off a good aim.
        self.declare_parameter("replan_period_sec", 4.0)
        self.declare_parameter("replan_reach_m", 0.015)
        # Optical success: image-center + 20 cm. Hold until the board moves.
        self.declare_parameter("hold_xy_m", 0.005)
        self.declare_parameter("hold_z_m", 0.012)
        self.declare_parameter("hold_resume_xy_m", 0.012)
        self.declare_parameter("hold_resume_z_m", 0.025)
        # If the board is off-center, only slide in the image plane first.
        self.declare_parameter("center_first_xy_m", 0.035)
        self.declare_parameter("center_done_xy_m", 0.018)
        self.declare_parameter("replan_opt_change_m", 0.045)
        self.declare_parameter("replan_min_sec", 0.80)
        self.declare_parameter("stall_sec", 4.0)
        self.declare_parameter("opt_filter_alpha", 0.35)
        self.declare_parameter("leave_hold_sec", 0.40)
        self.declare_parameter("trace_dir", "/tmp/dummy_track_ctrl/runs")
        self.declare_parameter("trace_every_n", 5)

        # Absolute workspace box (base_link). Ignored when ws_around_current > 0.
        self.declare_parameter("ws_x_min", -0.50)
        self.declare_parameter("ws_x_max", 0.50)
        self.declare_parameter("ws_y_min", -0.60)
        self.declare_parameter("ws_y_max", 0.20)
        self.declare_parameter("ws_z_min", 0.05)
        self.declare_parameter("ws_z_max", 0.45)
        # Clip desired EE to current EE ± this (m). Prevents lunging to a stale box corner.
        self.declare_parameter("ws_around_current", 0.12)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.control_frame = str(self.get_parameter("control_frame").value).lower()
        self.hold_current_distance = bool(self.get_parameter("hold_current_distance").value)
        twist_topic = str(self.get_parameter("twist_topic").value)
        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.follow_orientation = bool(self.get_parameter("follow_orientation").value)

        dx = float(self.get_parameter("desired_marker_in_ee_x").value)
        dy = float(self.get_parameter("desired_marker_in_ee_y").value)
        dz = float(self.get_parameter("desired_marker_in_ee_z").value)
        rpy = list(self.get_parameter("desired_marker_rpy").value)
        self._T_ee_marker_des = tf_transformations.euler_matrix(rpy[0], rpy[1], rpy[2])
        self._T_ee_marker_des[0, 3] = dx
        self._T_ee_marker_des[1, 3] = dy
        self._T_ee_marker_des[2, 3] = dz
        self._T_marker_ee_des = tf_transformations.inverse_matrix(self._T_ee_marker_des)

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_lin = float(self.get_parameter("max_linear_vel").value)
        self.max_ang = float(self.get_parameter("max_angular_vel").value)
        self.pos_db = float(self.get_parameter("pos_deadband").value)
        self.ori_db = float(self.get_parameter("ori_deadband").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_sec").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Match MoveIt Servo subscription (BEST_EFFORT) or twists are silently dropped.
        twist_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_desired = self.create_publisher(PoseStamped, "/tracking/desired_ee_pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, twist_topic, twist_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(JointTrajectory, "/servo_node/command", self._on_servo, 10)

        self._last_marker_ok = False
        self._last_opt_xyz: np.ndarray | None = None
        self._last_cmd_xy: float | None = None
        self._view_mode = "track"
        self._view_hold = False
        self._latch_T: np.ndarray | None = None
        self._latch_t = 0.0
        self._latch_reason = ""
        self._latch_opt: np.ndarray | None = None
        self._phase = ""
        self._best_opt_err: float | None = None
        self._stall_t = 0.0
        self._stall_replans = 0
        self._opt_filt: np.ndarray | None = None
        self._des_filt: np.ndarray | None = None
        self._v_filt: np.ndarray | None = None
        self._leave_hold_t = 0.0
        self._reject_n = 0
        self._joints_fw: list[float] | None = None
        self._servo_fw: list[float] | None = None
        self._hold_z: float | None = None
        self._trace = FollowTrace(
            str(self.get_parameter("trace_dir").value),
            every_n=int(self.get_parameter("trace_every_n").value),
        )
        self._last_warn_ns = 0
        self._last_status_ns = 0
        self._last_moving = False
        self._servo_ready = self.dry_run
        self._servo_setup_attempts = 0
        period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(period, self.control_tick)
        if not self.dry_run:
            # MoveIt Servo ignores twists until command type is TWIST and servoing is enabled.
            self._cli_cmd_type = self.create_client(
                ServoCommandType, "/servo_node/switch_command_type"
            )
            self._cli_pause = self.create_client(SetBool, "/servo_node/pause_servo")
            self.create_timer(1.0, self._ensure_servo_twist_mode)

        mode = "DRY_RUN" if self.dry_run else "LIVE"
        self.get_logger().info(
            f"Aruco servo tracker [{mode}] control={self.control_frame} "
            f"ee={self.ee_frame} optical={self.optical_frame} marker={self.marker_frame} "
            f"desired=[{dx:.3f},{dy:.3f},{dz:.3f}] hold_z={self.hold_current_distance} "
            f"follow_ori={self.follow_orientation}"
        )
        if self.dry_run:
            self.get_logger().info(
                "dry_run=true: publishing /tracking/desired_ee_pose only; no Servo twists"
            )
        if self._trace.path:
            self.get_logger().info(f"follow trace → {self._trace.path}")

    def _ensure_servo_twist_mode(self):
        if self._servo_ready or self.dry_run:
            return
        self._servo_setup_attempts += 1
        if self._servo_setup_attempts > 30:
            self.get_logger().error(
                "Giving up enabling MoveIt Servo TWIST mode; check /servo_node is running"
            )
            self._servo_ready = True  # stop retrying
            return
        if not self._cli_cmd_type.service_is_ready() or not self._cli_pause.service_is_ready():
            if self._servo_setup_attempts % 5 == 1:
                self.get_logger().warn("Waiting for /servo_node services...")
            return
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        fut = self._cli_cmd_type.call_async(req)

        def _after_type(_fut):
            try:
                res = _fut.result()
            except Exception as exc:
                self.get_logger().warn(f"switch_command_type failed: {exc}")
                return
            if not res or not res.success:
                self.get_logger().warn(f"switch_command_type TWIST rejected: {res}")
                return
            preq = SetBool.Request()
            preq.data = False  # unpause / enable servoing
            pfut = self._cli_pause.call_async(preq)

            def _after_pause(__fut):
                try:
                    pres = __fut.result()
                except Exception as exc:
                    self.get_logger().warn(f"pause_servo failed: {exc}")
                    return
                self._servo_ready = True
                self.get_logger().info(
                    f"MoveIt Servo TWIST enabled ({pres.message if pres else ''})"
                )

            pfut.add_done_callback(_after_pause)

        fut.add_done_callback(_after_type)

    def _on_joints(self, msg: JointState) -> None:
        if len(msg.position) >= 6:
            self._joints_fw = ros_rad_to_fw_deg(msg.position[:6])

    def _on_servo(self, msg: JointTrajectory) -> None:
        if msg.points and len(msg.points[0].positions) >= 6:
            self._servo_fw = ros_rad_to_fw_deg(msg.points[0].positions[:6])

    def _clear_latch(self, why: str) -> None:
        if self._latch_T is not None:
            self._trace.write_event("latch_clear", reason=why)
        self._latch_T = None
        self._latch_reason = ""
        self._latch_opt = None
        self._view_hold = False
        self._last_cmd_xy = None
        self._best_opt_err = None
        self._stall_t = 0.0
        if why in ("stale", "reject"):
            self._phase = ""
            self._stall_replans = 0
            self._opt_filt = None
            self._des_filt = None
            self._v_filt = None
            self._leave_hold_t = 0.0

    def control_tick(self):
        stamp = self.get_clock().now().to_msg()
        try:
            # Lookup each hop at its own "latest" time, then compose.
            # A single lookup(base→marker, Time()) often fails when marker TF is
            # briefly older than the joint TF buffer (extrapolation into the past).
            tf_base_ee = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            tf_base_optical = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time()
            )
            tf_optical_marker = self.tf_buffer.lookup_transform(
                self.optical_frame, self.marker_frame, rclpy.time.Time()
            )
            tf_ee_optical = self.tf_buffer.lookup_transform(
                self.ee_frame, self.optical_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self._publish_zero_twist(stamp)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_warn_ns > 2e9:
                self._last_warn_ns = now_ns
                self.get_logger().warn(f"Waiting for TF: {exc}")
            self._last_marker_ok = False
            return

        # Stale marker TF → stop
        marker_age = (
            self.get_clock().now()
            - rclpy.time.Time.from_msg(tf_optical_marker.header.stamp)
        ).nanoseconds / 1e9
        if marker_age > self.lost_timeout:
            self._publish_zero_twist(stamp)
            if self._last_marker_ok:
                self.get_logger().warn(f"Marker TF stale ({marker_age:.2f}s); stopping")
            if self._last_marker_ok:
                self.get_logger().warn("BOARD LOST — holding")
            self._last_marker_ok = False
            self._last_moving = False
            self._hold_z = None
            self._last_opt_xyz = None
            self._clear_latch("stale")
            return
        T_optical_marker = tf_to_matrix(tf_optical_marker)
        mx, my, mz = (float(T_optical_marker[i, 3]) for i in range(3))
        opt_xyz = np.array([mx, my, mz], dtype=float)
        max_z = float(self.get_parameter("max_marker_z").value)
        max_xy = float(self.get_parameter("max_marker_xy").value)
        max_jump = float(self.get_parameter("max_marker_jump").value)
        xy = math.hypot(mx, my)
        bad = mz > max_z or mz < 0.08 or xy > max_xy
        if self._last_opt_xyz is not None and not bad:
            if float(np.linalg.norm(opt_xyz - self._last_opt_xyz)) > max_jump:
                bad = True
        if bad:
            self._reject_n += 1
            if self._reject_n < 5:
                return
            self._publish_zero_twist(stamp)
            if self._last_marker_ok:
                self.get_logger().warn(
                    f"REJECT marker opt=[{mx:.3f},{my:.3f},{mz:.3f}] — hold"
                )
            self._last_marker_ok = False
            self._last_moving = False
            self._clear_latch("reject")
            return

        if not self._last_marker_ok:
            self.get_logger().info(
                f"BOARD DETECTED — tracking opt=[{mx:.3f},{my:.3f},{mz:.3f}]"
            )
            self._trace.write_event("board_detected", opt=[mx, my, mz])
        self._last_marker_ok = True
        self._last_opt_xyz = opt_xyz
        self._reject_n = 0

        T_base_ee = tf_to_matrix(tf_base_ee)
        T_base_optical = tf_to_matrix(tf_base_optical)
        T_ee_optical = tf_to_matrix(tf_ee_optical)
        T_optical_ee = tf_transformations.inverse_matrix(T_ee_optical)
        T_base_marker = T_base_optical @ T_optical_marker

        now_s = self.get_clock().now().nanoseconds / 1e9
        p_now = T_base_ee[:3, 3]
        xy_now = math.hypot(mx, my)

        if self.control_frame == "optical":
            T_des_raw, T_base_ee_des = self._plan_optical(
                mx, my, mz, T_base_ee, T_base_optical, now_s
            )
        else:
            # J6 axis (link6 ±Y) through board center; flange plane ⟂ axis at standoff.
            # Keep current EE +X (projected) so ArUco yaw does not spin J6.
            dz = float(self.get_parameter("desired_marker_in_ee_z").value)
            T_base_ee_des = self._ee_axis_on_board(
                T_base_marker, T_base_ee, dz, align_normal=self.follow_orientation
            )
            T_des_raw = T_base_ee_des.copy()
            T_des_inst = self._clip_desired(T_base_ee_des, T_base_ee)
            keep_xy = float(self.get_parameter("keep_in_view_xy").value)
            resume_xy = float(self.get_parameter("keep_in_view_resume_xy").value)
            replan_period = float(self.get_parameter("replan_period_sec").value)
            reach_m = float(self.get_parameter("replan_reach_m").value)
            if keep_xy > 1e-4 and xy_now > keep_xy:
                self._view_hold = True
            elif xy_now < resume_xy:
                self._view_hold = False
            reached = False
            if self._latch_T is not None:
                reached = float(np.linalg.norm(self._latch_T[:3, 3] - p_now)) < reach_m
            in_view = xy_now < resume_xy if resume_xy > 1e-4 else True
            need_replan = in_view and (
                self._latch_T is None
                or reached
                or (now_s - self._latch_t) >= replan_period
            )
            if need_replan:
                why = "first" if self._latch_T is None else ("reached" if reached else "period")
                self._latch_goal(T_des_inst, T_des_raw, [mx, my, mz], p_now, why)
            if self._latch_T is None:
                T_base_ee_des = T_base_ee.copy()
                self._view_mode = "WAIT_LOCK"
            elif self._view_hold:
                T_base_ee_des = T_base_ee.copy()
                self._view_mode = "KEEP_VIEW hold"
            else:
                T_base_ee_des = self._latch_T
                self._view_mode = "track"
        self._last_cmd_xy = xy_now

        desired_pose = pose_from_matrix(T_base_ee_des, self.base_frame, stamp)
        self.pub_desired.publish(desired_pose)
        self._broadcast_desired_tf(desired_pose)

        # Pose error in base
        p_err = T_base_ee_des[:3, 3] - T_base_ee[:3, 3]
        q_cur = tf_transformations.quaternion_from_matrix(T_base_ee)
        q_des = tf_transformations.quaternion_from_matrix(T_base_ee_des)
        w_err = rotation_error_axis_angle(q_cur, q_des)

        v = self.linear_gain * p_err
        w = self.angular_gain * w_err if self.follow_orientation else np.zeros(3)

        if np.linalg.norm(p_err) < self.pos_db or self._view_mode.endswith("hold"):
            v[:] = 0.0
            self._v_filt = np.zeros(3)
        if np.linalg.norm(w_err) < self.ori_db:
            w[:] = 0.0

        if self._v_filt is None:
            self._v_filt = v.copy()
        else:
            self._v_filt = 0.65 * self._v_filt + 0.35 * v
        v = self._v_filt

        v = self._clamp_vec(v, self.max_lin)
        w = self._clamp_vec(w, self.max_ang)

        err_norm = float(np.linalg.norm(p_err))
        moving = float(np.linalg.norm(v)) > 1e-3
        now_ns = self.get_clock().now().nanoseconds
        if moving != self._last_moving or now_ns - self._last_status_ns > 2e9:
            self._last_status_ns = now_ns
            self._last_moving = moving
            state = "MOVING toward target" if moving else "HOLD (err small / ws clamp)"
            if self._view_mode != "track":
                state = self._view_mode
            ox, oy, oz = (float(T_optical_marker[i, 3]) for i in range(3))
            self.get_logger().info(
                f"{state} |err|={err_norm*100:.1f}cm "
                f"|v|={float(np.linalg.norm(v))*100:.1f}cm/s "
                f"opt_marker=[{ox:.3f},{oy:.3f},{oz:.3f}] "
                f"des=[{T_base_ee_des[0,3]:.3f},{T_base_ee_des[1,3]:.3f},{T_base_ee_des[2,3]:.3f}]"
            )

        y_ee = T_base_ee[:3, 1]
        y_n = max(float(np.linalg.norm(y_ee)), 1e-9)
        y_hat = y_ee / y_n
        p_m = T_base_marker[:3, 3]
        axis_along = float(np.dot(p_m - p_now, -y_hat))
        axis_miss = float(np.linalg.norm(np.cross(p_m - p_now, y_hat)))
        self._trace.maybe_sample(
            {
                "mode": self._view_mode,
                "opt": [mx, my, mz],
                "marker": p_m.tolist(),
                "ee": p_now.tolist(),
                "des": T_base_ee_des[:3, 3].tolist(),
                "des_raw": T_des_raw[:3, 3].tolist(),
                "latch": None if self._latch_T is None else self._latch_T[:3, 3].tolist(),
                "latch_reason": self._latch_reason,
                "p_err": p_err.tolist(),
                "v": v.tolist(),
                "w": w.tolist(),
                "axis_along_m": axis_along,
                "axis_miss_m": axis_miss,
                "joints_fw": self._joints_fw,
                "servo_fw": self._servo_fw,
                "dry_run": self.dry_run,
                "phase": self._phase,
            }
        )

        if self.dry_run:
            return

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = float(v[0])
        twist.twist.linear.y = float(v[1])
        twist.twist.linear.z = float(v[2])
        twist.twist.angular.x = float(w[0])
        twist.twist.angular.y = float(w[1])
        twist.twist.angular.z = float(w[2])
        self.pub_twist.publish(twist)

    def _latch_goal(
        self,
        T_des: np.ndarray,
        T_des_raw: np.ndarray,
        opt: list[float],
        p_now: np.ndarray,
        why: str,
    ) -> None:
        self._latch_T = T_des.copy()
        self._latch_t = self.get_clock().now().nanoseconds / 1e9
        self._latch_reason = why
        self._latch_opt = np.array(opt, dtype=float)
        self._best_opt_err = None
        self._stall_t = self._latch_t
        self._trace.write_event(
            "replan",
            reason=why,
            opt=opt,
            ee=p_now.tolist(),
            des=self._latch_T[:3, 3].tolist(),
            des_raw=T_des_raw[:3, 3].tolist(),
            phase=self._phase,
        )
        self.get_logger().info(
            f"REPLAN ({why}/{self._phase or '-'}) "
            f"des={np.round(self._latch_T[:3, 3], 3).tolist()} "
            f"opt=[{opt[0]:.3f},{opt[1]:.3f},{opt[2]:.3f}]"
        )

    def _optical_slide(self, mx: float, my: float, mz: float, *, xy_only: bool) -> np.ndarray:
        dx = float(self.get_parameter("desired_marker_in_ee_x").value)
        dy = float(self.get_parameter("desired_marker_in_ee_y").value)
        if xy_only or bool(self.get_parameter("hold_current_distance").value):
            if bool(self.get_parameter("hold_current_distance").value) and self._hold_z is None:
                self._hold_z = mz
            slide = np.array([mx - dx, my - dy, 0.0], dtype=float)
            n = float(np.linalg.norm(slide))
            if n > 0.04:
                slide *= 0.04 / n
            return slide
        dz = float(self.get_parameter("desired_marker_in_ee_z").value)
        slide = np.array([mx - dx, my - dy, mz - dz], dtype=float)
        n = float(np.linalg.norm(slide))
        if n > 0.04:
            slide *= 0.04 / n
        return slide

    def _plan_optical(
        self,
        mx: float,
        my: float,
        mz: float,
        T_base_ee: np.ndarray,
        T_base_optical: np.ndarray,
        now_s: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Continuous optical servo: every tick slide toward image-center + 20 cm.

        Do not lock a Cartesian waypoint. Latch/replan was the start-stop jerk.
        """
        dx = float(self.get_parameter("desired_marker_in_ee_x").value)
        dy = float(self.get_parameter("desired_marker_in_ee_y").value)
        dz = float(self.get_parameter("desired_marker_in_ee_z").value)
        hold_xy = float(self.get_parameter("hold_xy_m").value)
        hold_z = float(self.get_parameter("hold_z_m").value)
        resume_xy = float(self.get_parameter("hold_resume_xy_m").value)
        resume_z = float(self.get_parameter("hold_resume_z_m").value)
        center_xy = float(self.get_parameter("center_first_xy_m").value)
        center_done = float(self.get_parameter("center_done_xy_m").value)
        alpha = float(np.clip(self.get_parameter("opt_filter_alpha").value, 0.05, 1.0))
        leave_hold = float(self.get_parameter("leave_hold_sec").value)

        raw = np.array([mx, my, mz], dtype=float)
        if self._opt_filt is None:
            self._opt_filt = raw
        else:
            self._opt_filt = (1.0 - alpha) * self._opt_filt + alpha * raw
        mx, my, mz = (float(self._opt_filt[i]) for i in range(3))

        xy_now = math.hypot(mx - dx, my - dy)
        z_err = abs(mz - dz)
        on_target = xy_now <= hold_xy and z_err <= hold_z
        p_now = T_base_ee[:3, 3]

        if self._phase == "hold":
            leaving = xy_now > resume_xy or z_err > resume_z
            if leaving:
                if self._leave_hold_t <= 0.0:
                    self._leave_hold_t = now_s
                if (now_s - self._leave_hold_t) < leave_hold:
                    self._view_mode = "ON_TARGET hold"
                    return T_base_ee.copy(), T_base_ee.copy()
                self._phase = "approach" if xy_now <= resume_xy else "center"
                self._leave_hold_t = 0.0
                self._des_filt = None
                self._trace.write_event("board_moved", opt=[mx, my, mz], ee=p_now.tolist())
            else:
                self._leave_hold_t = 0.0
                self._view_mode = "ON_TARGET hold"
                return T_base_ee.copy(), T_base_ee.copy()

        if on_target:
            if self._phase != "hold":
                self._trace.write_event("on_target", opt=[mx, my, mz], ee=p_now.tolist())
                self.get_logger().info(
                    f"ON TARGET — hold xy={xy_now*100:.1f}cm z={mz*100:.1f}cm"
                )
            self._phase = "hold"
            self._leave_hold_t = 0.0
            self._des_filt = p_now.copy()
            self._view_mode = "ON_TARGET hold"
            return T_base_ee.copy(), T_base_ee.copy()

        if self._phase == "center":
            xy_only = xy_now > center_done
        else:
            xy_only = xy_now > center_xy
        self._phase = "center" if xy_only else "approach"
        slide_opt = self._optical_slide(mx, my, mz, xy_only=xy_only)
        T_inst = T_base_ee.copy()
        T_inst[:3, 3] = T_base_ee[:3, 3] + T_base_optical[:3, :3] @ slide_opt
        if not self.follow_orientation:
            T_inst[:3, :3] = T_base_ee[:3, :3]
        T_raw = T_inst.copy()
        T_inst = self._clip_desired(T_inst, T_base_ee)
        p_des = T_inst[:3, 3]
        if self._des_filt is None:
            self._des_filt = p_des.copy()
        else:
            self._des_filt = 0.72 * self._des_filt + 0.28 * p_des
        T_smooth = T_base_ee.copy()
        T_smooth[:3, 3] = self._des_filt
        self._latch_T = T_smooth
        self._view_mode = f"track {self._phase}"
        return T_raw, T_smooth

    @staticmethod
    def _ee_axis_on_board(
        T_base_marker: np.ndarray,
        T_base_ee: np.ndarray,
        standoff_m: float,
        *,
        align_normal: bool,
    ) -> np.ndarray:
        """Place flange so J6 axis (link6 −Y) aims at the board center.

        Dummy Joint6 is URDF +Y on link6, not +Z. The D415 sits ~1 cm off that
        axis and looks along −Y, so −Y through the board keeps the marker in view.
        Marker +Z comes out of the printed face; desired −Y is −marker_z.
        Yaw about the axis is taken from the current EE +X, not from ArUco.
        """
        p_m = T_base_marker[:3, 3]
        if align_normal:
            z_m = T_base_marker[:3, 2]
            n = float(np.linalg.norm(z_m))
            pointing = (-z_m / n) if n > 1e-9 else (-T_base_ee[:3, 1])
            pointing = pointing / max(float(np.linalg.norm(pointing)), 1e-9)
            y_ee = -pointing
            x_cur = T_base_ee[:3, 0]
            x_proj = x_cur - float(np.dot(x_cur, y_ee)) * y_ee
            if float(np.linalg.norm(x_proj)) < 1e-4:
                x_cur = T_base_ee[:3, 2]
                x_proj = x_cur - float(np.dot(x_cur, y_ee)) * y_ee
            x_ee = x_proj / max(float(np.linalg.norm(x_proj)), 1e-9)
            z_ee = np.cross(x_ee, y_ee)
            z_ee = z_ee / max(float(np.linalg.norm(z_ee)), 1e-9)
            x_ee = np.cross(y_ee, z_ee)
            R = np.column_stack((x_ee, y_ee, z_ee))
        else:
            R = T_base_ee[:3, :3]
            y_ee = T_base_ee[:3, 1]
            y_ee = y_ee / max(float(np.linalg.norm(y_ee)), 1e-9)
            pointing = -y_ee
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p_m - float(standoff_m) * pointing
        return T

    def _clip_desired(self, T_des: np.ndarray, T_ee: np.ndarray) -> np.ndarray:
        out = T_des.copy()
        around = float(self.get_parameter("ws_around_current").value)
        p_now = T_ee[:3, 3]
        if around > 1e-4:
            out[:3, 3] = np.clip(out[:3, 3], p_now - around, p_now + around)
        for i, axis in enumerate(("x", "y", "z")):
            lo = float(self.get_parameter(f"ws_{axis}_min").value)
            hi = float(self.get_parameter(f"ws_{axis}_max").value)
            out[i, 3] = float(np.clip(out[i, 3], lo, hi))
        return out

    def _broadcast_desired_tf(self, pose: PoseStamped):
        tf_msg = TransformStamped()
        tf_msg.header = pose.header
        tf_msg.child_frame_id = "desired_ee"
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_zero_twist(self, stamp):
        if self.dry_run:
            return
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        self.pub_twist.publish(twist)

    @staticmethod
    def _clamp_vec(v: np.ndarray, limit: float) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n > limit and n > 1e-9:
            return v * (limit / n)
        return v


def main(args=None):
    rclpy.init(args=args)
    node = ArucoServoTracker()
    try:
        rclpy.spin(node)
    finally:
        node._trace.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
