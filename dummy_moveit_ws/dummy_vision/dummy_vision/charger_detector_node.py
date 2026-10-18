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
        # ~45–55 mm GaN brick face; both rect sides must fall in band.
        self.declare_parameter("charger_side_m", 0.045)
        self.declare_parameter("charger_side_min_m", 0.030)
        self.declare_parameter("charger_side_max_m", 0.070)
        self.declare_parameter("charger_z_min", 0.08)
        self.declare_parameter("charger_z_max", 0.55)
        # Live D415: top face HSV≈(91,68,171) cyan-sage (OpenCV H). Desk photo H≈40 was wrong WB.
        self.declare_parameter("charger_h_min", 75.0)
        self.declare_parameter("charger_h_max", 105.0)
        self.declare_parameter("charger_s_min", 40.0)
        self.declare_parameter("charger_s_max", 110.0)
        self.declare_parameter("charger_v_min", 80.0)
        self.declare_parameter("charger_v_max", 200.0)
        self.declare_parameter("charger_aspect_max", 1.70)
        self.declare_parameter("charger_rect_fill_min", 0.55)
        self.declare_parameter("charger_depth_half_px", 8)
        self.declare_parameter("charger_min_score", 0.35)
        self.declare_parameter("charger_min_area_px", 500.0)
        # Require N consecutive hits before publishing pose/quality>0 (kills 1–2 frame flickers).
        self.declare_parameter("charger_persist_frames", 5)
        self.declare_parameter("charger_miss_reset", 3)

        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.br = CvBridge()
        self.camera_matrix = None
        self._depth: np.ndarray | None = None
        self._hit_streak = 0
        self._miss_streak = 0
        self._locked = False
        self._last_hit: tuple[float, float, float, float, float, float] | None = None

        image_topic = str(self.get_parameter("image_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        # RealSense: color often TRANSIENT_LOCAL; camera_info VOLATILE; depth BEST_EFFORT/VOLATILE.
        img_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        depth_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, image_topic, self._on_image, img_qos)
        self.create_subscription(Image, depth_topic, self._on_depth, depth_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info, info_qos)

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
        persist = int(max(1, self.get_parameter("charger_persist_frames").value))
        miss_reset = int(max(1, self.get_parameter("charger_miss_reset").value))

        if hit is None:
            self._miss_streak += 1
            # Do not zero the warm-up streak on a single miss (flicker never locked).
            if self._miss_streak >= miss_reset:
                self._hit_streak = 0
                self._locked = False
                self._last_hit = None
                q = Float32()
                q.data = 0.0
                self.pub_q.publish(q)
                return
            if self._locked and self._last_hit is not None:
                # Hold last pose briefly through short dropouts.
                u, v, x, y, z, quality = self._last_hit
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
                self.pub_px.publish(px)
                return
            q = Float32()
            q.data = 0.0
            self.pub_q.publish(q)
            return

        self._miss_streak = 0
        self._hit_streak += 1
        self._last_hit = hit
        if not self._locked and self._hit_streak < persist:
            # Warm-up: do not advertise a track yet (mux/WDLS stay quiet).
            q = Float32()
            q.data = 0.0
            self.pub_q.publish(q)
            return
        self._locked = True

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
        # Color-first: brick depth is often 0 / flying; contour size bloated by bleed.
        mask = self._green_mask(hsv)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )

        fx = float(self.camera_matrix[0, 0])
        min_score = float(self.get_parameter("charger_min_score").value)
        min_area = float(self.get_parameter("charger_min_area_px").value)
        _Hv, Sv, Vv = cv2.split(hsv)
        z_ok = (depth >= z_min) & (depth <= z_max) & np.isfinite(depth)
        wood = (Vv > 130) & (Sv < 40) & z_ok
        wood_z = depth[wood]
        if wood_z.size >= 400:
            desk_z = float(np.percentile(wood_z, 20))
        else:
            z_valid = depth[z_ok]
            desk_z = float(np.percentile(z_valid, 20)) if z_valid.size >= 200 else None
        if desk_z is None:
            return None

        # Distance-transform peaks ≈ solid blob centers (robust to outline bleed).
        nlab, lab, stats, _cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = None
        best_score = 0.0
        z = desk_z - 0.012
        half_px = 0.5 * side_nom * fx / max(z, 1e-3)

        for lid in range(1, nlab):
            area = float(stats[lid, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            comp = np.where(lab == lid, 255, 0).astype(np.uint8)
            # Squareness via minAreaRect on component (allow bleed aspect).
            cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            rect = cv2.minAreaRect(cnts[0])
            (_cu, _cv), (rw, rh), _ang = rect
            rw, rh = float(abs(rw)), float(abs(rh))
            short, long = (rw, rh) if rw <= rh else (rh, rw)
            aspect = long / max(short, 1e-6)
            if aspect > max(aspect_max, 2.2):
                continue
            fill = area / max(rw * rh, 1e-6)
            if fill < max(0.35, fill_min - 0.2):
                continue

            dt = cv2.distanceTransform(comp, cv2.DIST_L2, 5)
            _mn, max_val, _mn_loc, max_loc = cv2.minMaxLoc(dt)
            # Inscribed radius should be near half of 45 mm (bleed → larger OK up to ~2×).
            if max_val < 0.45 * half_px:
                continue
            if max_val > 2.4 * half_px:
                continue
            u, v = float(max_loc[0]), float(max_loc[1])
            ui, vi = int(u), int(v)

            # Local depth if it agrees with desk; else desk plane.
            z_raw = None
            depth_fill = 0.2
            for rad in (half, half * 3, half * 5):
                y0, y1 = max(0, vi - rad), min(h, vi + rad + 1)
                x0, x1 = max(0, ui - rad), min(w, ui + rad + 1)
                patch = depth[y0:y1, x0:x1]
                patch = patch[(patch > z_min) & (patch < z_max) & np.isfinite(patch)]
                if patch.size >= 10:
                    z_raw = float(np.median(patch))
                    depth_fill = float(patch.size) / float(max(1, (2 * rad + 1) ** 2))
                    break
            z_use = z
            if z_raw is not None and abs(z_raw - desk_z) < 0.10:
                z_use = z_raw
                depth_fill = max(depth_fill, 0.35)

            # Size from inscribed diameter, clipped to brick band.
            side_m = float(np.clip(2.0 * max_val * z_use / max(fx, 1e-6), side_min, side_max))
            # Prefer peaks whose radius matches nominal brick.
            radius_q = float(
                np.clip(1.0 - abs(max_val - half_px) / max(half_px, 1e-3), 0.0, 1.0)
            )
            size_q = 1.0 - min(1.0, abs(side_m - side_nom) / max(side_nom, 1e-6))
            aspect_q = float(
                np.clip(1.0 - (aspect - 1.0) / max(max(aspect_max, 2.2) - 1.0, 1e-6), 0.0, 1.0)
            )
            fill_q = float(np.clip(fill, 0.0, 1.0))
            cx_n = abs(u - 0.5 * w) / max(0.5 * w, 1.0)
            cy_n = abs(v - 0.5 * h) / max(0.5 * h, 1.0)
            center_q = float(np.clip(1.0 - 0.7 * max(cx_n, cy_n), 0.0, 1.0))
            score = (
                0.28 * radius_q
                + 0.22 * size_q
                + 0.20 * aspect_q
                + 0.15 * fill_q
                + 0.08 * min(1.0, depth_fill)
                + 0.07 * center_q
            )
            if score > best_score:
                best_score = score
                best = (u, v, z_use, score)

        if best is None or best_score < min_score:
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
