#!/usr/bin/env python3
"""VLM locator: model finds the box, tracker keeps it live, AIM follows box center.

Agreed policy (2026-10-18):
- The drawn box center is the only follow target (optical pose from u,v + depth).
- Model ~1 Hz; OpenCV tracker refreshes the box between model frames.
- Model overrides tracker when centers differ by >80 px.
- Miss / no box: keep last box, publish quality 0 so the arm stays put.
- New phrase clears the target immediately (query switch).
"""

from __future__ import annotations

import base64
import json
import math
import threading
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String


class VlmLocator(Node):
    def __init__(self) -> None:
        super().__init__("vlm_locator")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("vlm_query", "")
        self.declare_parameter("query", "")
        self.declare_parameter("vlm_server_url", "http://127.0.0.1:8765")
        self.declare_parameter("server_url", "")
        self.declare_parameter("vlm_infer_hz", 1.0)
        self.declare_parameter("infer_hz", 0.0)
        self.declare_parameter("z_min", 0.08)
        self.declare_parameter("z_max", 0.90)
        self.declare_parameter("depth_half_px", 10)
        self.declare_parameter("vlm_mock", False)
        self.declare_parameter("mock", False)
        self.declare_parameter("jpeg_quality", 85)
        # Model vs tracker disagreement → take the model box.
        self.declare_parameter("vlm_model_override_px", 80.0)

        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.br = CvBridge()
        self.camera_matrix = None
        self._color: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._lock = threading.Lock()
        self._busy = False
        self._agree_uv: tuple[float, float] | None = None
        self._agree_n = 0
        # search → tracking (live box). No separate freeze point.
        self._phase = "search"
        self._box: tuple[float, float, float, float] | None = None  # u,v,w,h
        self._quality = 0.0
        self._tracker = None
        self._have_lock = False
        self._epoch = 0
        self._wire()

    def _query(self) -> str:
        q = str(self.get_parameter("query").value).strip()
        if not q:
            q = str(self.get_parameter("vlm_query").value).strip()
        return q

    def _server_url(self) -> str:
        u = str(self.get_parameter("server_url").value).strip()
        if not u:
            u = str(self.get_parameter("vlm_server_url").value).strip()
        return u or "http://127.0.0.1:8765"

    def _is_mock(self) -> bool:
        return bool(self.get_parameter("mock").value) or bool(
            self.get_parameter("vlm_mock").value
        )

    def _infer_hz(self) -> float:
        hz = float(self.get_parameter("infer_hz").value)
        if hz <= 1e-6:
            hz = float(self.get_parameter("vlm_infer_hz").value)
        return max(0.2, hz)

    def _wire(self) -> None:
        img_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        image_topic = str(self.get_parameter("image_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        for qos in (img_qos, be):
            self.create_subscription(Image, image_topic, self._on_image, qos)
            self.create_subscription(Image, depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info, info_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info, be)
        self.create_subscription(String, "/tracking/vlm_query", self._on_query, 10)

        self.pub_pose = self.create_publisher(PoseStamped, "/tracking/vlm_pose", 10)
        self.pub_q = self.create_publisher(Float32, "/tracking/vlm_quality", 10)
        self.pub_px = self.create_publisher(PointStamped, "/tracking/vlm_px", 10)
        self.pub_dbg = self.create_publisher(String, "/tracking/vlm_debug", 10)

        self.create_timer(1.0 / self._infer_hz(), self._tick_model)
        self.create_timer(0.05, self._tick_live)
        self.get_logger().info(
            f"vlm_locator → {self._server_url()} query={self._query()!r} "
            f"mock={self._is_mock()} live-box @ {self._infer_hz():.1f}Hz model"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        d = (
            depth.astype(np.float32) / 1000.0
            if depth.dtype == np.uint16
            else depth.astype(np.float32)
        )
        with self._lock:
            self._depth = d

    def _on_image(self, msg: Image) -> None:
        try:
            enc = (msg.encoding or "").lower()
            if enc in ("rgb8", "rgb"):
                rgb = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                bgr = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._color = bgr

    def _on_query(self, msg: String) -> None:
        q = (msg.data or "").strip()
        self.set_parameters(
            [
                rclpy.parameter.Parameter("vlm_query", rclpy.Parameter.Type.STRING, q),
                rclpy.parameter.Parameter("query", rclpy.Parameter.Type.STRING, q),
            ]
        )
        self._agree_uv = None
        self._agree_n = 0
        self._phase = "search"
        self._box = None
        self._quality = 0.0
        self._tracker = None
        self._have_lock = False
        self._epoch += 1
        self._publish_zero()
        if not q:
            self.get_logger().info("vlm query cleared")
            return
        self.get_logger().info(f"vlm query ← {q!r}")

    def _stable_uv(self, u: float, v: float) -> bool:
        prev = self._agree_uv
        if prev is None or math.hypot(u - prev[0], v - prev[1]) > 70.0:
            self._agree_uv = (u, v)
            self._agree_n = 1
            return False
        self._agree_n += 1
        self._agree_uv = (0.5 * (prev[0] + u), 0.5 * (prev[1] + v))
        return self._agree_n >= 2

    def _publish_zero(self) -> None:
        q = Float32()
        q.data = 0.0
        self.pub_q.publish(q)

    def _desk_z(self, depth: np.ndarray, hsv: np.ndarray) -> float | None:
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        _h, s, v = cv2.split(hsv)
        z_ok = (depth >= z_min) & (depth <= z_max) & np.isfinite(depth)
        wood = (v > 130) & (s < 40) & z_ok
        wood_z = depth[wood]
        if wood_z.size >= 400:
            return float(np.percentile(wood_z, 20))
        valid = depth[z_ok]
        if valid.size >= 200:
            return float(np.percentile(valid, 20))
        return None

    def _depth_at(
        self, depth: np.ndarray, u: float, v: float, desk_z: float | None
    ) -> tuple[float | None, float]:
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        half = int(max(3, self.get_parameter("depth_half_px").value))
        h, w = depth.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= vi < h and 0 <= ui < w):
            return None, 0.0
        z_raw = None
        fill = 0.0
        for rad in (half, half * 3, half * 5):
            y0, y1 = max(0, vi - rad), min(h, vi + rad + 1)
            x0, x1 = max(0, ui - rad), min(w, ui + rad + 1)
            patch = depth[y0:y1, x0:x1]
            patch = patch[(patch > z_min) & (patch < z_max) & np.isfinite(patch)]
            if patch.size >= 8:
                z_raw = float(np.median(patch))
                fill = float(patch.size) / float(max(1, (2 * rad + 1) ** 2))
                break
        if desk_z is None and z_raw is None:
            return None, 0.0
        if desk_z is not None:
            z = desk_z - 0.012
            if z_raw is not None and abs(z_raw - desk_z) < 0.12:
                z = z_raw
                fill = max(fill, 0.35)
            else:
                fill = max(fill, 0.2)
            return z, fill
        return float(z_raw), max(fill, 0.2)

    def _call_server(self, bgr: np.ndarray, query: str) -> dict:
        if self._is_mock():
            h, w = bgr.shape[:2]
            return {
                "ok": True,
                "answer": "mock",
                "boxes": [
                    {
                        "kind": "box",
                        "u": 0.55 * w,
                        "v": 0.40 * h,
                        "w": 80.0,
                        "h": 80.0,
                        "label": "mock",
                    }
                ],
                "dt_s": 0.01,
            }
        q = int(self.get_parameter("jpeg_quality").value)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        if not ok:
            return {"ok": False, "error": "jpeg encode failed"}
        payload = json.dumps(
            {
                "query": query,
                "image_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
            }
        ).encode("utf-8")
        url = self._server_url().rstrip("/") + "/detect"
        req = urllib.request.Request(
            url, data=payload, headers={"Content-Type": "application/json"}, method="POST"
        )
        try:
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            return {"ok": False, "error": f"server: {exc}"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def _init_tracker(self, bgr: np.ndarray, u: float, v: float, bw: float, bh: float) -> None:
        h, w = bgr.shape[:2]
        x0 = int(np.clip(round(u - bw / 2), 0, w - 2))
        y0 = int(np.clip(round(v - bh / 2), 0, h - 2))
        ww = int(np.clip(round(bw), 8, w - x0))
        hh = int(np.clip(round(bh), 8, h - y0))
        try:
            tracker = cv2.TrackerKCF_create()
            tracker.init(bgr, (x0, y0, ww, hh))
        except Exception as exc:
            self._tracker = None
            self.get_logger().warn(f"VLM tracker unavailable: {exc}")
            return
        self._tracker = tracker
        self._box = (u, v, float(ww), float(hh))

    def _set_box(
        self,
        bgr: np.ndarray,
        u: float,
        v: float,
        bw: float,
        bh: float,
        quality: float,
        *,
        reinit_tracker: bool,
    ) -> None:
        self._box = (u, v, bw, bh)
        self._quality = quality
        self._phase = "tracking"
        self._have_lock = True
        if reinit_tracker:
            self._init_tracker(bgr, u, v, bw, bh)

    def _publish_box(self, depth: np.ndarray | None, desk_z: float | None = None) -> None:
        if self._box is None or self.camera_matrix is None:
            return
        u, v, bw, bh = self._box
        z = None
        fill = 0.2
        if depth is not None:
            if desk_z is None:
                hsv = None
                with self._lock:
                    bgr = None if self._color is None else self._color.copy()
                if bgr is not None and depth.shape[:2] == bgr.shape[:2]:
                    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                    desk_z = self._desk_z(depth, hsv)
            z, fill = self._depth_at(depth, u, v, desk_z)
        if z is None:
            # Keep the drawn box; stop driving the arm until depth returns.
            self._publish_zero()
            self._emit_dbg(u, v, bw, bh, 0.0, z=None, tag="无深度")
            return
        K = self.camera_matrix
        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        q = float(self._quality if self._quality > 0.35 else 0.7)
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.optical_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)
        fq = Float32()
        fq.data = q
        self.pub_q.publish(fq)
        px = PointStamped()
        px.header.stamp = stamp
        px.header.frame_id = "image"
        px.point.x = u
        px.point.y = v
        self.pub_px.publish(px)
        self._emit_dbg(u, v, bw, bh, q, z=z, tag="")

    def _emit_dbg(
        self,
        u: float,
        v: float,
        bw: float,
        bh: float,
        q: float,
        *,
        z: float | None,
        tag: str,
    ) -> None:
        label = self._query() or "VLM"
        if tag:
            label = f"{label} · {tag}"
        dbg = {
            "query": self._query(),
            "ok": True,
            "phase": self._phase,
            "u": round(u, 1),
            "v": round(v, 1),
            "w": round(bw, 1),
            "h": round(bh, 1),
            "q": round(q, 3),
            "label": label,
        }
        if z is not None:
            dbg["z"] = round(z, 3)
        out = String()
        out.data = json.dumps(dbg)
        self.pub_dbg.publish(out)

    def _tick_live(self) -> None:
        """20 Hz: refresh box from tracker and publish AIM target = box center."""
        query = self._query()
        if not query or self._phase != "tracking" or self._box is None:
            self._publish_zero()
            return
        with self._lock:
            bgr = None if self._color is None else self._color.copy()
            depth = None if self._depth is None else self._depth.copy()
        if bgr is None:
            return
        if self._tracker is not None:
            try:
                ok, box = self._tracker.update(bgr)
            except Exception:
                ok = False
                box = None
            if ok and box is not None:
                x, y, ww, hh = (float(v) for v in box)
                self._box = (x + ww / 2.0, y + hh / 2.0, ww, hh)
            else:
                # Keep last box, stop arm until model refreshes.
                self._publish_zero()
                u, v, bw, bh = self._box
                self._emit_dbg(u, v, bw, bh, 0.0, z=None, tag="跟丢")
                return
        if depth is not None and depth.shape[:2] != bgr.shape[:2]:
            depth = cv2.resize(
                depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST
            )
        self._publish_box(depth)

    def _tick_model(self) -> None:
        if self._busy:
            return
        with self._lock:
            bgr = None if self._color is None else self._color.copy()
            depth = None if self._depth is None else self._depth.copy()
        if bgr is None or depth is None or self.camera_matrix is None:
            return
        query = self._query()
        epoch = self._epoch
        if not query:
            self._phase = "search"
            self._box = None
            self._tracker = None
            self._have_lock = False
            self._publish_zero()
            return
        self._busy = True
        try:
            res = self._call_server(bgr, query)
            dbg = {
                "query": query,
                "ok": bool(res.get("ok")),
                "dt_s": res.get("dt_s"),
                "raw": str(res.get("answer", res.get("error", "")))[:240],
            }
            if not res.get("ok"):
                out = String()
                out.data = json.dumps(dbg)
                self.pub_dbg.publish(out)
                return
            hits = list(res.get("boxes") or [])
            dbg["n"] = len(hits)
            if not hits:
                # Stay put with last box; do not clear quality target with a new lock.
                out = String()
                out.data = json.dumps(dbg)
                self.pub_dbg.publish(out)
                return
            if depth.shape[:2] != bgr.shape[:2]:
                depth = cv2.resize(
                    depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST
                )
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            desk_z = self._desk_z(depth, hsv)
            boxes = [h for h in hits if h.get("kind") == "box"]
            best = (
                max(boxes, key=lambda h: float(h.get("w", 0)) * float(h.get("h", 0)))
                if boxes
                else hits[0]
            )
            u, v = float(best["u"]), float(best["v"])
            bw, bh = float(best.get("w", 0.0)), float(best.get("h", 0.0))
            ih, iw = bgr.shape[:2]
            if bw > 0.85 * iw and bh > 0.85 * ih:
                dbg["reject"] = "full_frame"
                out = String()
                out.data = json.dumps(dbg)
                self.pub_dbg.publish(out)
                return

            override_px = float(self.get_parameter("vlm_model_override_px").value)
            if self._have_lock and self._box is not None:
                cu, cv_, _, _ = self._box
                jump = math.hypot(u - cu, v - cv_)
                if jump < override_px:
                    # Tracker is already on the object; keep it, avoid 1 Hz yank.
                    return
                self.get_logger().info(
                    f"VLM model override {jump:.0f}px → uv=({u:.0f},{v:.0f})"
                )
            else:
                if not self._stable_uv(u, v):
                    dbg.update(
                        {
                            "u": round(u, 1),
                            "v": round(v, 1),
                            "w": round(bw, 1),
                            "h": round(bh, 1),
                            "reject": "unstable",
                        }
                    )
                    out = String()
                    out.data = json.dumps(dbg)
                    self.pub_dbg.publish(out)
                    self.get_logger().info(
                        f"VLM hold uv=({u:.0f},{v:.0f}) until the box stops jumping"
                    )
                    return

            z, fill = self._depth_at(depth, u, v, desk_z)
            if z is None:
                dbg["reject"] = "no_depth"
                out = String()
                out.data = json.dumps(dbg)
                self.pub_dbg.publish(out)
                return
            area = float(bw) * float(bh)
            area_q = float(np.clip(area / 8000.0, 0.0, 1.0))
            quality = float(np.clip(0.55 * area_q + 0.35 * fill + 0.10, 0.35, 0.95))
            if epoch != self._epoch or self._query() != query:
                return
            self._set_box(bgr, u, v, bw, bh, quality, reinit_tracker=True)
            self._publish_box(depth, desk_z)
            self.get_logger().info(
                f"VLM box q={quality:.2f} uv=({u:.0f},{v:.0f}) z={z:.3f} "
                f"dt={res.get('dt_s')}s"
            )
        finally:
            self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmLocator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
