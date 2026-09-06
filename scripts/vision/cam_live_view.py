#!/usr/bin/env python3
"""Live D415 preview with tracking HUD (detection + motion status).

Shows:
  - Optical-axis aim (+) at the camera principal point (tracker XY target)
  - Board center (o) and a line to the aim point
  - ArUco outline when OpenCV sees the board
  - DETECTED / LOST from /aruco_camera_pose age
  - EE error to /tracking/desired_ee_pose
  - MOVING / HOLD from /servo_node/delta_twist_cmds

Usage:
  source /opt/ros/jazzy/setup.bash
  source dummy_moveit_ws/install/setup.bash
  python3 scripts/vision/cam_live_view.py
Buttons: 开始跟随 / 回初始 / 停止收起. Keys: 1/s start, 2/h handeye, 3/x stow, q quit.
"""

from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from PIL import Image as PilImage
from PIL import ImageDraw, ImageFont
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import ServoStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"
WIN = "D415 tracking HUD"
REPO = Path(__file__).resolve().parents[2]
CTRL = REPO / "scripts" / "home" / "track_control.py"
BTN_H = 48
BTN_GAP = 8
BAR_H = BTN_H + 16

# cmd, label, idle BGR, hover-hint
BUTTONS = (
    ("start", "开始跟随", (40, 150, 70)),
    ("handeye", "回初始", (200, 140, 30)),
    ("stop", "停止收起", (40, 40, 200)),
)
BUSY_TXT = {
    "start": "BUSY: 停止跟随 → 回初始 → 启动跟随…",
    "handeye": "BUSY: 停止跟随 → 回初始位置…",
    "stop": "BUSY: 停止跟随 → 回收集位置…",
}
CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_FONT_CACHE: dict[int, ImageFont.FreeTypeFont | ImageFont.ImageFont] = {}


def _cjk_font(size: int):
    cached = _FONT_CACHE.get(size)
    if cached is not None:
        return cached
    try:
        font = ImageFont.truetype(CJK_FONT, size)
    except OSError:
        font = ImageFont.load_default()
    _FONT_CACHE[size] = font
    return font


def _put_zh(
    bgr: np.ndarray,
    text: str,
    xy: tuple[int, int],
    color_bgr: tuple[int, int, int],
    size: int = 22,
) -> None:
    """Draw CJK/ASCII text; xy is top-left."""
    if not text:
        return
    font = _cjk_font(size)
    dummy = ImageDraw.Draw(PilImage.new("RGB", (1, 1)))
    box = dummy.textbbox((0, 0), text, font=font)
    tw, th = max(1, box[2] - box[0]), max(1, box[3] - box[1])
    x, y = xy
    h, w = bgr.shape[:2]
    x1, y1 = min(w, x + tw + 4), min(h, y + th + 4)
    if x1 <= x or y1 <= y:
        return
    patch = PilImage.fromarray(cv2.cvtColor(bgr[y:y1, x:x1], cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(patch)
    draw.text(
        (-box[0], -box[1]),
        text,
        font=font,
        fill=(color_bgr[2], color_bgr[1], color_bgr[0]),
    )
    bgr[y:y1, x:x1] = cv2.cvtColor(np.array(patch), cv2.COLOR_RGB2BGR)


def _put_zh_center(
    bgr: np.ndarray,
    text: str,
    rect: tuple[int, int, int, int],
    color_bgr: tuple[int, int, int],
    size: int = 22,
) -> None:
    font = _cjk_font(size)
    dummy = ImageDraw.Draw(PilImage.new("RGB", (1, 1)))
    box = dummy.textbbox((0, 0), text, font=font)
    tw, th = box[2] - box[0], box[3] - box[1]
    x0, y0, x1, y1 = rect
    tx = x0 + max(4, (x1 - x0 - tw) // 2)
    ty = y0 + max(2, (y1 - y0 - th) // 2)
    _put_zh(bgr, text, (tx, ty), color_bgr, size=size)


class CamLiveView(Node):
    def __init__(self) -> None:
        super().__init__("cam_live_view")
        self.br = CvBridge()
        self.frame = None
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.marker_length = 0.05

        self._last_pose_t = 0.0
        self._last_pose_xyz = None
        self._last_des_t = 0.0
        self._last_des_xyz = None
        self._last_twist_t = 0.0
        self._last_twist_norm = 0.0
        self._last_ee_xyz = None
        self._servo_code = None
        self._servo_msg = ""
        self._servo_t = 0.0
        self._busy = False
        self._busy_msg = ""
        self._mode = "idle"  # idle | following
        self._hit = None
        self._lock = threading.Lock()
        self._btn_rects: list[tuple[str, tuple[int, int, int, int]]] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        dict_id = cv2.aruco.DICT_ARUCO_ORIGINAL
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters()

        # RealSense QoS: RELIABLE (+ sometimes TRANSIENT_LOCAL).
        for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=durability,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(Image, TOPIC, self._cb_img, qos)
            self.create_subscription(CameraInfo, INFO_TOPIC, self._cb_info, qos)

        self.create_subscription(PoseStamped, "/aruco_camera_pose", self._cb_pose, 10)
        self.create_subscription(PoseStamped, "/tracking/desired_ee_pose", self._cb_des, 10)
        twist_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            TwistStamped, "/servo_node/delta_twist_cmds", self._cb_twist, twist_qos
        )
        self.create_subscription(ServoStatus, "/servo_node/status", self._cb_status, 10)
        self.create_timer(0.05, self._tick_ee_tf)

        self.get_logger().info(
            f"Viewing {TOPIC} — 开始跟随 / 回初始 / 停止收起；q 退出"
        )

    def request_cmd(self, cmd: str) -> None:
        with self._lock:
            if self._busy or cmd not in BUSY_TXT:
                return
            self._busy = True
            self._busy_msg = BUSY_TXT[cmd]
        self.get_logger().info(f"HUD {cmd}")

        def _run() -> None:
            try:
                proc = subprocess.run(
                    [sys.executable, str(CTRL), cmd],
                    cwd=str(REPO),
                    capture_output=True,
                    text=True,
                    check=False,
                )
                out = ((proc.stdout or "") + (proc.stderr or "")).strip()
                if proc.returncode != 0:
                    msg = out.splitlines()[-1] if out else f"{cmd} failed"
                    with self._lock:
                        self._busy_msg = f"ERROR: {msg}"
                    self.get_logger().error(msg)
                    time.sleep(5.0)
                    return
                with self._lock:
                    if cmd == "start":
                        self._mode = "following"
                        self._busy_msg = "跟随已启动（handeye）"
                    elif cmd == "handeye":
                        self._mode = "idle"
                        self._busy_msg = "已回初始位置，跟随已停"
                    else:
                        self._mode = "idle"
                        self._busy_msg = "已收起，跟随已停"
                time.sleep(3.0)
            except Exception as exc:
                with self._lock:
                    self._busy_msg = f"ERROR: {exc}"
                self.get_logger().error(str(exc))
                time.sleep(2.5)
            finally:
                with self._lock:
                    self._busy = False
                    self._busy_msg = ""

        threading.Thread(target=_run, daemon=True).start()

    def on_mouse(self, event: int, x: int, y: int, _flags: int, _param) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            self._hit = None
            for cmd, rect in self._btn_rects:
                x0, y0, x1, y1 = rect
                if x0 <= x <= x1 and y0 <= y <= y1:
                    self._hit = cmd
                    break
            return
        if event != cv2.EVENT_LBUTTONUP:
            return
        for cmd, rect in self._btn_rects:
            x0, y0, x1, y1 = rect
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.request_cmd(cmd)
                return

    def _cb_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def _cb_pose(self, msg: PoseStamped) -> None:
        self._last_pose_t = time.time()
        p = msg.pose.position
        self._last_pose_xyz = (p.x, p.y, p.z)

    def _cb_des(self, msg: PoseStamped) -> None:
        self._last_des_t = time.time()
        p = msg.pose.position
        self._last_des_xyz = (p.x, p.y, p.z)

    def _cb_twist(self, msg: TwistStamped) -> None:
        self._last_twist_t = time.time()
        L = msg.twist.linear
        self._last_twist_norm = math.sqrt(L.x * L.x + L.y * L.y + L.z * L.z)

    def _cb_status(self, msg: ServoStatus) -> None:
        self._servo_code = int(msg.code)
        self._servo_msg = str(msg.message)
        self._servo_t = time.time()

    def _tick_ee_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "link6_1_1", rclpy.time.Time())
            t = tf.transform.translation
            self._last_ee_xyz = (t.x, t.y, t.z)
        except Exception:
            pass

    def _cb_img(self, msg: Image) -> None:
        enc = (msg.encoding or "").lower()
        if enc in ("rgb8", "rgb"):
            img = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        else:
            bgr = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params
        )
        local_seen = ids is not None and len(ids) > 0
        if local_seen:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            if self.camera_matrix is not None:
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                    cv2.drawFrameAxes(
                        bgr,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[0],
                        tvecs[0],
                        self.marker_length * 0.5,
                    )
                except Exception:
                    pass

        board_px = None
        if local_seen:
            pts = corners[0].reshape(-1, 2)
            board_px = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        self.frame = self._draw_hud(bgr, local_seen, board_px)

    def _aim_px(self, w: int, h: int) -> tuple[int, int]:
        """Tracker optical (0,0,*) projects to the principal point, not raw image mid."""
        if self.camera_matrix is not None:
            return int(round(self.camera_matrix[0, 2])), int(round(self.camera_matrix[1, 2]))
        return w // 2, h // 2

    def _draw_aim(self, bgr: np.ndarray, board_px: tuple[float, float] | None) -> str:
        h, w = bgr.shape[:2]
        cx, cy = self._aim_px(w, h)
        # Crosshair + diamond: desired optical-axis / image-center target
        cyan = (255, 220, 0)
        arm = 22
        cv2.line(bgr, (cx - arm, cy), (cx + arm, cy), cyan, 2, cv2.LINE_AA)
        cv2.line(bgr, (cx, cy - arm), (cx, cy + arm), cyan, 2, cv2.LINE_AA)
        cv2.circle(bgr, (cx, cy), 10, cyan, 2, cv2.LINE_AA)
        cv2.putText(
            bgr,
            "AIM",
            (cx + 16, cy - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            cyan,
            2,
            cv2.LINE_AA,
        )
        if board_px is None:
            return "aim: principal pt (optical XY=0)  board: —"
        bx, by = int(round(board_px[0])), int(round(board_px[1]))
        cv2.circle(bgr, (bx, by), 8, (0, 255, 80), 2, cv2.LINE_AA)
        cv2.circle(bgr, (bx, by), 2, (0, 255, 80), -1, cv2.LINE_AA)
        cv2.line(bgr, (bx, by), (cx, cy), (0, 255, 80), 1, cv2.LINE_AA)
        dx, dy = bx - cx, by - cy
        pix = math.hypot(dx, dy)
        return f"aim vs board: {pix:.0f}px  d=[{dx:+.0f},{dy:+.0f}]  (0=on optical axis)"

    def _draw_hud(
        self, bgr: np.ndarray, local_seen: bool, board_px: tuple[float, float] | None
    ) -> np.ndarray:
        now = time.time()
        ros_seen = (now - self._last_pose_t) < 1.0
        des_ok = (now - self._last_des_t) < 1.0
        twist_fresh = (now - self._last_twist_t) < 0.5
        moving = twist_fresh and self._last_twist_norm > 1e-3

        # Banner color: red=lost, yellow=local-only, green=ROS detected
        if ros_seen:
            banner = (40, 180, 40)
            det_txt = "BOARD: DETECTED (ROS)"
        elif local_seen:
            banner = (0, 200, 255)
            det_txt = "BOARD: SEEN locally (ROS pose missing)"
        else:
            banner = (40, 40, 220)
            det_txt = "BOARD: LOST — hold ArUco ORIGINAL in view"

        with self._lock:
            busy = self._busy
            busy_msg = self._busy_msg
        if busy or busy_msg:
            banner = (0, 140, 220)

        overlay = bgr.copy()
        cv2.rectangle(overlay, (0, 0), (bgr.shape[1], 156), banner, -1)
        cv2.addWeighted(overlay, 0.45, bgr, 0.55, 0, bgr)

        aim_txt = self._draw_aim(bgr, board_px)
        lines = [det_txt, aim_txt]
        if busy_msg:
            lines.insert(0, busy_msg)
        if self._last_pose_xyz is not None and ros_seen:
            x, y, z = self._last_pose_xyz
            lines.append(f"marker cam xyz [{x:.3f},{y:.3f},{z:.3f}] m")
        elif local_seen:
            lines.append("OpenCV sees marker; check aruco_detector node")

        err_txt = "target err: —"
        if des_ok and self._last_des_xyz is not None and self._last_ee_xyz is not None:
            dx = self._last_des_xyz[0] - self._last_ee_xyz[0]
            dy = self._last_des_xyz[1] - self._last_ee_xyz[1]
            dz = self._last_des_xyz[2] - self._last_ee_xyz[2]
            err = math.sqrt(dx * dx + dy * dy + dz * dz)
            err_txt = f"target err: {err*100:.1f} cm  d=[{dx:.3f},{dy:.3f},{dz:.3f}]"
        elif not des_ok:
            err_txt = "target err: no desired_ee (tracker waiting)"
        lines.append(err_txt)

        if moving:
            move_txt = f"MOTION: MOVING toward target  |v|={self._last_twist_norm*100:.1f} cm/s"
            move_color = (40, 220, 40)
        elif twist_fresh:
            move_txt = "MOTION: HOLD (twist~0 — at target, clamped, or lost)"
            move_color = (0, 200, 255)
        else:
            move_txt = "MOTION: no twist msgs (tracker dry-run / down?)"
            move_color = (40, 40, 220)
        lines.append(move_txt)

        if self._servo_code is not None and (now - self._servo_t) < 1.0:
            if self._servo_code == 0:
                lines.append("SERVO: OK")
            else:
                lines.append(f"SERVO HALT code={self._servo_code}: {self._servo_msg}")
                move_color = (40, 40, 220)
        else:
            lines.append("SERVO: no status")

        y0 = 22
        for i, line in enumerate(lines):
            color = (255, 255, 255) if i < 3 else move_color
            cv2.putText(
                bgr,
                line,
                (10, y0 + i * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        # Status chips sit above the button bar
        chip_y = bgr.shape[0] - BAR_H - 48
        self._chip(bgr, 10, chip_y, "DET", ros_seen or local_seen, ros_seen)
        self._chip(bgr, 90, chip_y, "TGT", des_ok, des_ok and moving)
        self._chip(bgr, 170, chip_y, "MOV", moving, moving)
        with self._lock:
            following = self._mode == "following"
        self._chip(bgr, 250, chip_y, "RUN", following, following)
        self._draw_buttons(bgr)
        return bgr

    def _draw_buttons(self, bgr: np.ndarray) -> None:
        h, w = bgr.shape[:2]
        y0 = h - BAR_H + 8
        y1 = y0 + BTN_H
        n = len(BUTTONS)
        usable = w - BTN_GAP * (n + 1)
        bw = usable // n
        with self._lock:
            busy = self._busy
            busy_msg = self._busy_msg
        overlay = bgr.copy()
        cv2.rectangle(overlay, (0, h - BAR_H), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.72, bgr, 0.28, 0, bgr)
        rects: list[tuple[str, tuple[int, int, int, int]]] = []
        for i, (cmd, label, color) in enumerate(BUTTONS):
            x0 = BTN_GAP + i * (bw + BTN_GAP)
            x1 = x0 + bw
            fill = color
            if busy:
                fill = (70, 70, 70)
            elif self._hit == cmd:
                fill = tuple(min(255, c + 40) for c in color)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), fill, -1)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (230, 230, 230), 1)
            text = "…" if busy else label
            _put_zh_center(bgr, text, (x0, y0, x1, y1), (255, 255, 255), size=22)
            rects.append((cmd, (x0, y0, x1, y1)))
        self._btn_rects = rects
        if busy_msg:
            _put_zh(bgr, busy_msg, (12, h - BAR_H - 28), (0, 220, 255), size=18)

    @staticmethod
    def _chip(img: np.ndarray, x: int, y: int, label: str, on: bool, good: bool) -> None:
        color = (40, 180, 40) if good else ((0, 200, 255) if on else (60, 60, 60))
        cv2.rectangle(img, (x, y), (x + 70, y + 36), color, -1)
        cv2.putText(
            img,
            label,
            (x + 12, y + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main() -> None:
    rclpy.init()
    node = CamLiveView()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, node.on_mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.frame is not None:
                cv2.imshow(WIN, node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in (ord("1"), ord("s")):
                node.request_cmd("start")
            elif key in (ord("2"), ord("h")):
                node.request_cmd("handeye")
            elif key in (ord("3"), ord("x")):
                node.request_cmd("stop")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
