#!/usr/bin/env python3
"""Publish a fake optical Target for WDLS software-in-the-loop tests.

Moves the marker in a slow ellipse around Spray Aim (0,0,0.20), briefly
pauses near the aim so Hold can trigger, then drifts away again.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32


class SimWdlsTarget(Node):
    def __init__(self) -> None:
        super().__init__("sim_wdls_target")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("standoff_m", 0.20)
        self.declare_parameter("amp_xy_m", 0.04)
        self.declare_parameter("period_sec", 12.0)
        self.declare_parameter("hold_dwell_sec", 2.5)
        self.declare_parameter("quality", 0.95)
        self.declare_parameter("rate_hz", 30.0)

        self.optical = str(self.get_parameter("optical_frame").value)
        self.standoff = float(self.get_parameter("standoff_m").value)
        self.amp = float(self.get_parameter("amp_xy_m").value)
        self.period = max(1.0, float(self.get_parameter("period_sec").value))
        self.dwell = max(0.0, float(self.get_parameter("hold_dwell_sec").value))
        self.quality = float(self.get_parameter("quality").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.pub_pose = self.create_publisher(PoseStamped, "/tracking/target_pose", 10)
        self.pub_q = self.create_publisher(Float32, "/tracking/target_quality", 10)
        self._t0 = time.time()
        self._dwell_until = 0.0
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"sim target → /tracking/target_pose in {self.optical} "
            f"ellipse amp={self.amp:.3f}m period={self.period:.1f}s dwell={self.dwell:.1f}s"
        )

    def _tick(self) -> None:
        now = time.time()
        # Every half-period, dwell on Spray Aim so Hold can fire.
        phase = (now - self._t0) % self.period
        half = self.period * 0.5
        near_hold_slot = abs(phase - half) < 0.05 or phase < 0.05
        if near_hold_slot and self._dwell_until <= now:
            self._dwell_until = now + self.dwell

        if now < self._dwell_until:
            mx = my = 0.0
            mz = self.standoff
            mode = "dwell(aim)"
        else:
            th = 2.0 * math.pi * (phase / self.period)
            mx = self.amp * math.cos(th)
            my = self.amp * math.sin(th) * 0.7
            mz = self.standoff + 0.015 * math.sin(th * 2.0)
            mode = "ellipse"

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.optical
        pose.pose.position.x = float(mx)
        pose.pose.position.y = float(my)
        pose.pose.position.z = float(mz)
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)
        q = Float32()
        q.data = float(self.quality)
        self.pub_q.publish(q)

        # Sparse log
        if int(now * 2) != int((now - 0.05) * 2):
            self.get_logger().info(
                f"{mode} opt=[{mx:.3f},{my:.3f},{mz:.3f}] q={self.quality:.2f}"
            )


def main() -> None:
    rclpy.init()
    node = SimWdlsTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
