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
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


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
        # optical: keep board on camera optical axis (image center). ee: legacy flange +Z.
        self.declare_parameter("control_frame", "optical")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        # If true, keep current camera-to-board distance and only center XY in the image.
        self.declare_parameter("hold_current_distance", True)

        # Desired marker in the chosen control frame (meters).
        # optical + (0,0,z): board on optical axis, z in front of the lens.
        self.declare_parameter("desired_marker_in_ee_x", 0.0)
        self.declare_parameter("desired_marker_in_ee_y", 0.0)
        self.declare_parameter("desired_marker_in_ee_z", 0.25)
        # Desired marker orientation relative to control frame (RPY, rad).
        self.declare_parameter("desired_marker_rpy", [0.0, 0.0, 0.0])

        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("max_linear_vel", 0.03)
        self.declare_parameter("max_angular_vel", 0.15)
        self.declare_parameter("pos_deadband", 0.005)
        self.declare_parameter("ori_deadband", 0.02)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("max_marker_z", 0.50)
        self.declare_parameter("max_marker_xy", 0.18)
        self.declare_parameter("max_marker_jump", 0.12)

        # Absolute workspace box (base_link). Ignored when ws_around_current > 0.
        self.declare_parameter("ws_x_min", -0.50)
        self.declare_parameter("ws_x_max", 0.50)
        self.declare_parameter("ws_y_min", -0.60)
        self.declare_parameter("ws_y_max", 0.20)
        self.declare_parameter("ws_z_min", 0.05)
        self.declare_parameter("ws_z_max", 0.45)
        # Clip desired EE to current EE ± this (m). Prevents lunging to a stale box corner.
        self.declare_parameter("ws_around_current", 0.08)

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

        self._last_marker_ok = False
        self._last_opt_xyz: np.ndarray | None = None
        self._hold_z: float | None = None
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
            self._publish_zero_twist(stamp)
            if self._last_marker_ok:
                self.get_logger().warn(
                    f"REJECT marker opt=[{mx:.3f},{my:.3f},{mz:.3f}] — hold"
                )
            self._last_marker_ok = False
            self._last_moving = False
            return

        if not self._last_marker_ok:
            self.get_logger().info(
                f"BOARD DETECTED — tracking opt=[{mx:.3f},{my:.3f},{mz:.3f}]"
            )
        self._last_marker_ok = True
        self._last_opt_xyz = opt_xyz

        T_base_ee = tf_to_matrix(tf_base_ee)
        T_base_optical = tf_to_matrix(tf_base_optical)
        T_ee_optical = tf_to_matrix(tf_ee_optical)
        T_optical_ee = tf_transformations.inverse_matrix(T_ee_optical)
        T_base_marker = T_base_optical @ T_optical_marker

        if self.control_frame == "optical":
            # Translate EE by the camera-XY error only. Lock standoff on first lock
            # so a receding false step cannot ratchet distance out.
            dx = float(self.get_parameter("desired_marker_in_ee_x").value)
            dy = float(self.get_parameter("desired_marker_in_ee_y").value)
            if bool(self.get_parameter("hold_current_distance").value):
                if self._hold_z is None:
                    self._hold_z = mz
                # ignore z error
                slide_opt = np.array([mx - dx, my - dy, 0.0], dtype=float)
            else:
                dz = float(self.get_parameter("desired_marker_in_ee_z").value)
                slide_opt = np.array([mx - dx, my - dy, mz - dz], dtype=float)
            T_base_ee_des = T_base_ee.copy()
            T_base_ee_des[:3, 3] = T_base_ee[:3, 3] + T_base_optical[:3, :3] @ slide_opt
        else:
            # Legacy: marker on EE +Z (usually not the image center).
            T_base_ee_des = T_base_marker @ self._T_marker_ee_des

        if not self.follow_orientation:
            T_base_ee_des[:3, :3] = T_base_ee[:3, :3]

        # Soft clamp: first around current EE, then optional absolute box.
        around = float(self.get_parameter("ws_around_current").value)
        p_now = T_base_ee[:3, 3]
        if around > 1e-4:
            T_base_ee_des[:3, 3] = np.clip(
                T_base_ee_des[:3, 3], p_now - around, p_now + around
            )
        ws = {
            "x": (
                float(self.get_parameter("ws_x_min").value),
                float(self.get_parameter("ws_x_max").value),
            ),
            "y": (
                float(self.get_parameter("ws_y_min").value),
                float(self.get_parameter("ws_y_max").value),
            ),
            "z": (
                float(self.get_parameter("ws_z_min").value),
                float(self.get_parameter("ws_z_max").value),
            ),
        }
        for i, axis in enumerate(("x", "y", "z")):
            lo, hi = ws[axis]
            T_base_ee_des[i, 3] = float(np.clip(T_base_ee_des[i, 3], lo, hi))

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

        if np.linalg.norm(p_err) < self.pos_db:
            v[:] = 0.0
        if np.linalg.norm(w_err) < self.ori_db:
            w[:] = 0.0

        v = self._clamp_vec(v, self.max_lin)
        w = self._clamp_vec(w, self.max_ang)

        err_norm = float(np.linalg.norm(p_err))
        moving = float(np.linalg.norm(v)) > 1e-3
        now_ns = self.get_clock().now().nanoseconds
        if moving != self._last_moving or now_ns - self._last_status_ns > 2e9:
            self._last_status_ns = now_ns
            self._last_moving = moving
            state = "MOVING toward target" if moving else "HOLD (err small / ws clamp)"
            ox, oy, oz = (float(T_optical_marker[i, 3]) for i in range(3))
            self.get_logger().info(
                f"{state} |err|={err_norm*100:.1f}cm "
                f"|v|={float(np.linalg.norm(v))*100:.1f}cm/s "
                f"opt_marker=[{ox:.3f},{oy:.3f},{oz:.3f}] "
                f"des=[{T_base_ee_des[0,3]:.3f},{T_base_ee_des[1,3]:.3f},{T_base_ee_des[2,3]:.3f}]"
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
