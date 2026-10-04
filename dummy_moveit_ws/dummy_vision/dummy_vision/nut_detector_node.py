#!/usr/bin/env python3
"""RGB-D hex nut detector: dark hex on skin/light bg → optical pose + HUD pixel."""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
import tf_transformations


class NutDetector(Node):
    def __init__(self) -> None:
        super().__init__("nut_detector")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("nut_diameter_m", 0.025)
        self.declare_parameter("nut_diameter_min_m", 0.018)
        self.declare_parameter("nut_diameter_max_m", 0.035)
        self.declare_parameter("nut_z_min", 0.10)
        self.declare_parameter("nut_z_max", 0.50)
        self.declare_parameter("nut_v_max", 90.0)
        self.declare_parameter("nut_s_max", 90.0)

        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.br = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self._depth: np.ndarray | None = None
        self._last_warn_ns = 0

        image_topic = str(self.get_parameter("image_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        for reliability in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
            for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
                qos = QoSProfile(
                    depth=2,
                    reliability=reliability,
                    durability=durability,
                    history=HistoryPolicy.KEEP_LAST,
                )
                self.create_subscription(Image, image_topic, self._on_image, qos)
                self.create_subscription(Image, depth_topic, self._on_depth, qos)
                self.create_subscription(CameraInfo, info_topic, self._on_info, qos)

        self.pub_pose = self.create_publisher(PoseStamped, "/tracking/nut_pose", 10)
        self.pub_q = self.create_publisher(Float32, "/tracking/nut_quality", 10)
        self.pub_px = self.create_publisher(PointStamped, "/tracking/nut_px", 10)

        self.get_logger().info("nut detector ready (dark hex + aligned depth)")

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        if depth.dtype == np.uint16:
            self._depth = depth.astype(np.float32) / 1000.0
        else:
            self._depth = depth.astype(np.float32)

    def _on_image(self, msg: Image) -> None:
        if self.camera_matrix is None or self._depth is None:
            return
        try:
            enc = (msg.encoding or "").lower()
            if enc in ("rgb8", "rgb"):
                rgb = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                bgr = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        hit = self._detect(bgr, self._depth)
        if hit is None:
            q = Float32()
            q.data = 0.0
            self.pub_q.publish(q)
            return

        u, v, x, y, z, quality, yaw = hit
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.optical_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        self.pub_pose.publish(pose)

        fq = Float32()
        fq.data = float(quality)
        self.pub_q.publish(fq)

        px = PointStamped()
        px.header.stamp = stamp
        px.header.frame_id = "image"
        px.point.x = float(u)
        px.point.y = float(v)
        px.point.z = 0.0
        self.pub_px.publish(px)

    def _detect(
        self, bgr: np.ndarray, depth: np.ndarray
    ) -> tuple[float, float, float, float, float, float, float] | None:
        h, w = bgr.shape[:2]
        if depth.shape[0] != h or depth.shape[1] != w:
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        z_min = float(self.get_parameter("nut_z_min").value)
        z_max = float(self.get_parameter("nut_z_max").value)
        v_max = float(self.get_parameter("nut_v_max").value)
        s_max = float(self.get_parameter("nut_s_max").value)
        d_nom = float(self.get_parameter("nut_diameter_m").value)
        d_min = float(self.get_parameter("nut_diameter_min_m").value)
        d_max = float(self.get_parameter("nut_diameter_max_m").value)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        dark = (hsv[:, :, 2] < v_max) & (hsv[:, :, 1] < s_max)
        z_ok = (depth >= z_min) & (depth <= z_max) & np.isfinite(depth) & (depth > 0.01)
        mask = (dark & z_ok).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        fx = float(self.camera_matrix[0, 0])
        best = None
        best_score = 0.0

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < 40.0:
                continue
            peri = float(cv2.arcLength(cnt, True))
            if peri < 1e-3:
                continue
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            nvert = len(approx)
            if nvert < 5 or nvert > 8:
                continue
            M = cv2.moments(cnt)
            if abs(M["m00"]) < 1e-6:
                continue
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= vi < h and 0 <= ui < w):
                continue
            patch = depth[max(0, vi - 3) : vi + 4, max(0, ui - 3) : ui + 4]
            patch = patch[(patch > z_min) & (patch < z_max)]
            if patch.size < 4:
                continue
            z = float(np.median(patch))
            eq_d_px = 2.0 * math.sqrt(area / math.pi)
            d_m = eq_d_px * z / max(fx, 1e-6)
            if d_m < d_min or d_m > d_max:
                continue
            circ = 4.0 * math.pi * area / (peri * peri)
            hex_q = 1.0 - min(1.0, abs(nvert - 6) / 3.0)
            size_q = 1.0 - min(1.0, abs(d_m - d_nom) / max(d_nom, 1e-6))
            circ_q = float(np.clip((circ - 0.65) / 0.30, 0.0, 1.0))
            fill = float(patch.size) / 49.0
            score = 0.40 * hex_q + 0.35 * size_q + 0.15 * circ_q + 0.10 * min(1.0, fill)
            if score > best_score:
                yaw = 0.0
                if nvert >= 6:
                    pts = approx.reshape(-1, 2).astype(np.float64)
                    pts -= pts.mean(axis=0)
                    _, _, vt = np.linalg.svd(pts)
                    yaw = float(math.atan2(vt[0, 1], vt[0, 0]))
                best_score = score
                best = (u, v, z, score, yaw)

        if best is None or best_score < 0.25:
            return None

        u, v, z, score, yaw = best
        K = self.camera_matrix
        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        quality = float(np.clip(score, 0.0, 1.0))
        return u, v, float(x), float(y), float(z), quality, yaw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NutDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
