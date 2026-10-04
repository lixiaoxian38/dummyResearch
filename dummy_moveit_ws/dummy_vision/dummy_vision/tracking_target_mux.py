#!/usr/bin/env python3
"""Select board or nut pose → /tracking/target_pose + quality + TF tracking_target."""

from __future__ import annotations

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32


class TrackingTargetMux(Node):
    def __init__(self) -> None:
        super().__init__("tracking_target_mux")
        self.declare_parameter("follow_source", "board")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("target_frame", "tracking_target")
        self.declare_parameter("stale_sec", 0.35)

        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.stale_sec = float(self.get_parameter("stale_sec").value)

        self._board: PoseStamped | None = None
        self._nut: PoseStamped | None = None
        self._board_q = 0.0
        self._nut_q = 0.0
        self._board_t = 0.0
        self._nut_t = 0.0

        self.pub_pose = self.create_publisher(PoseStamped, "/tracking/target_pose", 10)
        self.pub_q = self.create_publisher(Float32, "/tracking/target_quality", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(PoseStamped, "/tracking/board_pose", self._on_board, 10)
        self.create_subscription(Float32, "/tracking/board_quality", self._on_board_q, 10)
        self.create_subscription(PoseStamped, "/tracking/nut_pose", self._on_nut, 10)
        self.create_subscription(Float32, "/tracking/nut_quality", self._on_nut_q, 10)
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info(
            f"mux ready optical={self.optical_frame} child={self.target_frame}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_board(self, msg: PoseStamped) -> None:
        self._board = msg
        self._board_t = self._now()

    def _on_board_q(self, msg: Float32) -> None:
        self._board_q = float(msg.data)

    def _on_nut(self, msg: PoseStamped) -> None:
        self._nut = msg
        self._nut_t = self._now()

    def _on_nut_q(self, msg: Float32) -> None:
        self._nut_q = float(msg.data)

    def _tick(self) -> None:
        src = str(self.get_parameter("follow_source").value).strip().lower()
        now = self._now()
        if src == "nut":
            pose, q, t = self._nut, self._nut_q, self._nut_t
        else:
            src = "board"
            pose, q, t = self._board, self._board_q, self._board_t

        if pose is None or (now - t) > self.stale_sec:
            out_q = Float32()
            out_q.data = 0.0
            self.pub_q.publish(out_q)
            return

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.optical_frame
        out.pose = pose.pose
        self.pub_pose.publish(out)
        fq = Float32()
        fq.data = float(max(0.0, min(1.0, q)))
        self.pub_q.publish(fq)

        tf_msg = TransformStamped()
        tf_msg.header = out.header
        tf_msg.child_frame_id = self.target_frame
        tf_msg.transform.translation.x = out.pose.position.x
        tf_msg.transform.translation.y = out.pose.position.y
        tf_msg.transform.translation.z = out.pose.position.z
        tf_msg.transform.rotation = out.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingTargetMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
