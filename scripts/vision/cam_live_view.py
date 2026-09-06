#!/usr/bin/env python3
"""Live D415 preview for hand-eye calibration (QoS matched to realsense2_camera).

Usage:
  source /opt/ros/jazzy/setup.bash
  source dummy_moveit_ws/install/setup.bash
  python3 scripts/vision/cam_live_view.py
Press q to quit.
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC = "/camera/camera/color/image_raw"


class CamLiveView(Node):
    def __init__(self) -> None:
        super().__init__("cam_live_view")
        self.br = CvBridge()
        self.frame = None
        # RealSense on this machine publishes RELIABLE (+ sometimes TRANSIENT_LOCAL).
        for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=durability,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(Image, TOPIC, self._cb, qos)
        self.get_logger().info(f"Viewing {TOPIC} — press q in the window to quit")

    def _cb(self, msg: Image) -> None:
        enc = (msg.encoding or "").lower()
        if enc in ("rgb8", "rgb"):
            img = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main() -> None:
    rclpy.init()
    node = CamLiveView()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.frame is not None:
                cv2.imshow("D415 color (calibration)", node.frame)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
