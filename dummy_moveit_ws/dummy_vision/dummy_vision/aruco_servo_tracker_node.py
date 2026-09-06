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
from rclpy.duration import Duration
from rclpy.node import Node
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

        # Desired marker pose in EE frame: marker in front of flange along EE +Z by default.
        # For eye-in-hand, optical Z is often forward; EE frame may differ — tune after calib.
        self.declare_parameter("desired_marker_in_ee_x", 0.0)
        self.declare_parameter("desired_marker_in_ee_y", 0.0)
        self.declare_parameter("desired_marker_in_ee_z", 0.25)
        # Desired marker orientation relative to EE (RPY, rad). Identity = marker axes aligned with EE.
        self.declare_parameter("desired_marker_rpy", [0.0, 0.0, 0.0])

        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("max_linear_vel", 0.08)
        self.declare_parameter("max_angular_vel", 0.4)
        self.declare_parameter("pos_deadband", 0.005)
        self.declare_parameter("ori_deadband", 0.02)
        self.declare_parameter("lost_timeout_sec", 0.5)

        # Soft workspace box for desired EE (base_link), meters
        self.declare_parameter("ws_x_min", -0.35)
        self.declare_parameter("ws_x_max", 0.35)
        self.declare_parameter("ws_y_min", -0.55)
        self.declare_parameter("ws_y_max", 0.05)
        self.declare_parameter("ws_z_min", 0.05)
        self.declare_parameter("ws_z_max", 0.55)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
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

        self.ws = {
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

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub_desired = self.create_publisher(PoseStamped, "/tracking/desired_ee_pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, twist_topic, 10)

        self._last_marker_ok = False
        self._last_warn_ns = 0
        period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(period, self.control_tick)

        mode = "DRY_RUN" if self.dry_run else "LIVE"
        self.get_logger().info(
            f"Aruco servo tracker [{mode}] ee={self.ee_frame} marker={self.marker_frame} "
            f"desired_marker_in_ee=[{dx:.3f},{dy:.3f},{dz:.3f}] follow_ori={self.follow_orientation}"
        )
        if self.dry_run:
            self.get_logger().info(
                "dry_run=true: publishing /tracking/desired_ee_pose only; no Servo twists"
            )

    def control_tick(self):
        stamp = self.get_clock().now().to_msg()
        try:
            tf_base_ee = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            tf_base_marker = self.tf_buffer.lookup_transform(
                self.base_frame, self.marker_frame, rclpy.time.Time()
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
        marker_age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf_base_marker.header.stamp)).nanoseconds / 1e9
        if marker_age > self.lost_timeout:
            self._publish_zero_twist(stamp)
            if self._last_marker_ok:
                self.get_logger().warn(f"Marker TF stale ({marker_age:.2f}s); stopping")
            self._last_marker_ok = False
            return
        self._last_marker_ok = True

        T_base_ee = tf_to_matrix(tf_base_ee)
        T_base_marker = tf_to_matrix(tf_base_marker)
        # EE should move so that T_ee_marker == T_ee_marker_des
        # => T_base_ee_des = T_base_marker @ inv(T_ee_marker_des) = T_base_marker @ T_marker_ee_des
        T_base_ee_des = T_base_marker @ self._T_marker_ee_des

        if not self.follow_orientation:
            T_base_ee_des[:3, :3] = T_base_ee[:3, :3]

        # Soft clamp desired translation
        for i, axis in enumerate(("x", "y", "z")):
            lo, hi = self.ws[axis]
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
