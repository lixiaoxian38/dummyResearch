#!/usr/bin/env python3
"""RGB-D sage-green square charger detector → optical pose + HUD pixel.

Aim: top-face center (minAreaRect) + neighborhood depth median.
"""

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


class ChargerDetector(Node):
    def __init__(self) -> None:
        super().__init__("charger_detector")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        # ~45 mm GaN brick; wide band for oblique foreshortening.
        self.declare_parameter("charger_side_m", 0.045)
        self.declare_parameter("charger_side_min_m", 0.030)
        self.declare_parameter("charger_side_max_m", 0.075)
        self.declare_parameter("charger_z_min", 0.08)
        self.declare_parameter("charger_z_max", 0.55)
        # Sage green from desk photo: H≈40, S≈60.
        self.declare_parameter("charger_h_min", 25.0)
        self.declare_parameter("charger_h_max", 70.0)
        self.declare_parameter("charger_s_min", 20.0)
        self.declare_parameter("charger_s_max", 200.0)
        self.declare_parameter("charger_v_min", 40.0)
        self.declare_parameter("charger_v_max", 255.0)
        self.declare_parameter("charger_aspect_max", 1.90)
        self.declare_parameter("charger_rect_fill_min", 0.50)
        self.declare_parameter("charger_depth_half_px", 8)

        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.br = CvBridge()
        self.camera_matrix = None
        self._depth: np.ndarray | None = None

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

        self.pub_pose = self.create_publisher(PoseStamped, "/tracking/charger_pose", 10)
        self.pub_q = self.create_publisher(Float32, "/tracking/charger_quality", 10)
        self.pub_px = self.create_publisher(PointStamped, "/tracking/charger_px", 10)

        self.get_logger().info("charger detector ready (HSV sage square + depth median)")

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

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

        u, v, x, y, z, quality = hit
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.optical_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
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

    def _green_mask(self, hsv: np.ndarray) -> np.ndarray:
        lo = np.array(
            [
                float(self.get_parameter("charger_h_min").value),
                float(self.get_parameter("charger_s_min").value),
                float(self.get_parameter("charger_v_min").value),
            ],
            dtype=np.float32,
        )
        hi = np.array(
            [
                float(self.get_parameter("charger_h_max").value),
                float(self.get_parameter("charger_s_max").value),
                float(self.get_parameter("charger_v_max").value),
            ],
            dtype=np.float32,
        )
        return cv2.inRange(hsv, lo, hi)

    def _detect(
        self, bgr: np.ndarray, depth: np.ndarray
    ) -> tuple[float, float, float, float, float, float] | None:
        h, w = bgr.shape[:2]
        if depth.shape[0] != h or depth.shape[1] != w:
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        z_min = float(self.get_parameter("charger_z_min").value)
        z_max = float(self.get_parameter("charger_z_max").value)
        side_nom = float(self.get_parameter("charger_side_m").value)
        side_min = float(self.get_parameter("charger_side_min_m").value)
        side_max = float(self.get_parameter("charger_side_max_m").value)
        aspect_max = float(self.get_parameter("charger_aspect_max").value)
        fill_min = float(self.get_parameter("charger_rect_fill_min").value)
        half = int(max(3, self.get_parameter("charger_depth_half_px").value))

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = self._green_mask(hsv)
        z_ok = (depth >= z_min) & (depth <= z_max) & np.isfinite(depth) & (depth > 0.01)
        mask = cv2.bitwise_and(mask, mask, mask=z_ok.astype(np.uint8) * 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        fx = float(self.camera_matrix[0, 0])
        best = None
        best_score = 0.0

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < 200.0:
                continue
            rect = cv2.minAreaRect(cnt)
            (cu, cv_), (rw, rh), _ang = rect
            rw, rh = float(abs(rw)), float(abs(rh))
            if rw < 8.0 or rh < 8.0:
                continue
            short, long = (rw, rh) if rw <= rh else (rh, rw)
            aspect = long / max(short, 1e-6)
            if aspect > aspect_max:
                continue
            rect_area = rw * rh
            fill = area / max(rect_area, 1e-6)
            if fill < fill_min:
                continue

            u, v = float(cu), float(cv_)
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= vi < h and 0 <= ui < w):
                continue

            y0, y1 = max(0, vi - half), min(h, vi + half + 1)
            x0, x1 = max(0, ui - half), min(w, ui + half + 1)
            patch = depth[y0:y1, x0:x1]
            patch = patch[(patch > z_min) & (patch < z_max) & np.isfinite(patch)]
            if patch.size < 6:
                continue
            z = float(np.median(patch))
            # Mean side in metres (perspective: use geometric mean of sides).
            side_px = math.sqrt(max(rw * rh, 1e-6))
            side_m = side_px * z / max(fx, 1e-6)
            if side_m < side_min or side_m > side_max:
                continue

            size_q = 1.0 - min(1.0, abs(side_m - side_nom) / max(side_nom, 1e-6))
            aspect_q = float(np.clip(1.0 - (aspect - 1.0) / max(aspect_max - 1.0, 1e-6), 0.0, 1.0))
            fill_q = float(np.clip((fill - fill_min) / max(1e-3, 1.0 - fill_min), 0.0, 1.0))
            depth_fill = float(patch.size) / float(max(1, (2 * half + 1) ** 2))
            score = 0.35 * size_q + 0.30 * aspect_q + 0.25 * fill_q + 0.10 * min(1.0, depth_fill)
            if score > best_score:
                best_score = score
                best = (u, v, z, score)

        if best is None or best_score < 0.22:
            return None

        u, v, z, score = best
        K = self.camera_matrix
        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        quality = float(np.clip(score, 0.0, 1.0))
        return u, v, float(x), float(y), float(z), quality


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargerDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
