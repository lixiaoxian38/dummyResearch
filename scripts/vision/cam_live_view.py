#!/usr/bin/env python3
"""Live D415 preview — Dummy tracking operator console + 6-axis jog.

LIVE: optical AIM — board 20 cm / charger (sage brick) 10 cm.
Jog: OpenCV trackbars → /cdc_servo_bridge/jog_fw (no second serial writer).

Usage:
  source /opt/ros/jazzy/setup.bash && source dummy_moveit_ws/install/setup.bash
  python3 scripts/vision/cam_live_view.py
Buttons: 开始跟随 / 回初始 / 停止收起 / 六轴拖动
Toggles: 标定板|充电头 — LIVE 中可切；运动走 WDLS（J4 自由 / J6 锁）
Keys: 1/s start, 2/h handeye, 3/x stow, 4/j jog, q quit
"""

from __future__ import annotations

import json
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
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from moveit_msgs.msg import ServoStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "home"))
from follow_session import load as load_follow_session  # noqa: E402
from follow_session import save as save_follow_session  # noqa: E402
from joint_limits_fw import JOINT_LIMITS_FW_DEG  # noqa: E402

TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"
WIN = "Dummy Tracking Hub"
JOG_WIN = "Dummy 六轴拖动"
CTRL = REPO / "scripts" / "home" / "track_control.py"
BTN_H = 34
BTN_GAP = 6
BAR_H = BTN_H + 16

RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])

JOINT_NAMES = [f"J{i}" for i in range(1, 7)]
JOG_HINTS = [
    "底座左右",
    "肩 远近高低",
    "肘 远近高低",
    "腕滚",
    "相机俯仰",
    "法兰旋转(相机不动)",
]

BUTTONS = (
    ("start", "开始跟随", (40, 150, 70)),
    ("handeye", "回初始", (200, 140, 30)),
    ("stop", "停止收起", (40, 40, 200)),
    ("jog", "六轴拖动", (160, 90, 40)),
)
BUSY_TXT = {
    "start": "BUSY: 启动跟随（若折叠会先回初始 · 光心对准·20cm）…",
    "handeye": "BUSY: 取消跟随 → 回初始位置…",
    "stop": "BUSY: 取消跟随 → 收起到折叠位…",
    "jog": "BUSY: 暂停跟随 → 打开六轴拖动…",
    "pause": "BUSY: 停止跟随（保持当前姿态）…",
}
CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_FONT_CACHE: dict[int, ImageFont.FreeTypeFont | ImageFont.ImageFont] = {}


def ros_rad_to_fw_deg(rad: np.ndarray) -> np.ndarray:
    return np.rad2deg((rad + RAD_VOLUMN) * RAD_DIRECT)


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
    size: int = 20,
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
        self._status: dict = {}
        self._status_t = 0.0
        self._handeye_ok = False
        self._busy = False
        self._busy_msg = ""
        self._mode = "idle"  # idle | following | jogging
        self._hit = None
        self._lock = threading.Lock()
        self._btn_rects: list[tuple[str, tuple[int, int, int, int]]] = []
        self._toggle_rects: list[tuple[str, tuple[int, int, int, int]]] = []
        sess = load_follow_session()
        self._follow_source = str(sess["follow_source"])
        if self._follow_source == "cup":
            self._follow_source = "charger"
        if self._follow_source not in ("board", "charger", "vlm"):
            self._follow_source = "board"
        self._chg_px = None
        self._chg_q = 0.0
        self._chg_t = 0.0
        self._vlm_px = None
        self._vlm_q = 0.0
        self._vlm_t = 0.0
        self._vlm_query = ""
        self._vlm_box = None  # (u, v, w, h, label, reject, t)
        self._tgt_q = 0.0
        self._tgt_q_t = 0.0

        # Jog panel (via CDC bridge topic — no second serial)
        self._jog_open = False
        self._jog_syncing = False
        self._jog_pending = False
        self._jog_synced = False  # sliders matched to joint_states at least once
        self._open_jog_requested = False
        self._jog_cmd = np.zeros(6, dtype=float)
        self._jog_drag: int | None = None
        self._jog_x0, self._jog_x1 = 148, 620
        self._cam_h = 10**9
        self._fw = np.zeros(6, dtype=float)
        self._have_js = False
        self._jog_pub = self.create_publisher(
            Float64MultiArray, "/cdc_servo_bridge/jog_fw", 10
        )
        self._last_jog_sent: np.ndarray | None = None
        self.create_subscription(JointState, "/joint_states", self._cb_js, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        dict_id = cv2.aruco.DICT_ARUCO_ORIGINAL
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters()

        # Match RealSense: color image is TRANSIENT_LOCAL; camera_info is VOLATILE.
        # Dual-subscribe used to spam DURABILITY incompatible warnings.
        img_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, TOPIC, self._cb_img, img_qos)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._cb_info, info_qos)

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
        self.create_subscription(String, "/tracking/status", self._cb_track_status, 10)
        self.create_subscription(PointStamped, "/tracking/charger_px", self._cb_chg_px, 10)
        self.create_subscription(Float32, "/tracking/charger_quality", self._cb_chg_q, 10)
        self.create_subscription(PointStamped, "/tracking/vlm_px", self._cb_vlm_px, 10)
        self.create_subscription(Float32, "/tracking/vlm_quality", self._cb_vlm_q, 10)
        self.create_subscription(String, "/tracking/vlm_debug", self._cb_vlm_dbg, 10)
        self.create_subscription(Float32, "/tracking/target_quality", self._cb_tgt_q, 10)
        self.create_timer(0.05, self._tick_ee_tf)
        self.create_timer(1.0, self._tick_handeye_tf)
        self.create_timer(0.08, self._tick_jog_send)

        self.get_logger().info(
            "跟随控制台：板/充电头（LIVE 可切）+ WDLS 跟随三键 + 六轴拖动；q 退出。"
            "识别指令：http://127.0.0.1:8770"
        )

    def request_cmd(self, cmd: str) -> None:
        if cmd.startswith("src_"):
            self._request_toggle(cmd)
            return
        if cmd == "jog":
            self._request_jog()
            return
        with self._lock:
            if cmd == "start" and self._mode == "following":
                cmd = "pause"
            if self._busy or cmd not in BUSY_TXT:
                return
            self._busy = True
            self._busy_msg = BUSY_TXT[cmd]
        self.get_logger().info(f"HUD {cmd}")

        def _run() -> None:
            try:
                if cmd == "start":
                    self.close_jog()
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
                    if "Write timeout" in out or "串口卡死" in out:
                        msg = "串口卡死，臂不会动。拔插机械臂USB（不必重启电脑）后再点"
                    with self._lock:
                        self._busy_msg = f"ERROR: {msg}"
                    self.get_logger().error(msg)
                    return
                with self._lock:
                    if cmd == "start":
                        self._mode = "following"
                        self._busy_msg = (
                            f"跟随中（{self._src_label()}）· AIM对准绿点且 SPRAY亮=到位"
                        )
                    elif cmd == "handeye":
                        self._mode = "idle"
                        self._busy_msg = "已取消跟随，回到初始位置"
                    elif cmd == "pause":
                        self._mode = "idle"
                        self._busy_msg = "已停止跟随，姿态保持。可改目标后再点开始跟随"
                    else:
                        self._mode = "idle"
                        self._busy_msg = "已取消跟随，收到折叠位"
                time.sleep(0.3)
                with self._lock:
                    if not self._busy_msg.startswith("ERROR"):
                        self._busy_msg = ""
            except Exception as exc:
                with self._lock:
                    self._busy_msg = f"ERROR: {exc}"
                self.get_logger().error(str(exc))
            finally:
                with self._lock:
                    self._busy = False

        threading.Thread(target=_run, daemon=True).start()

    def _src_label(self) -> str:
        return {"charger": "充电头", "vlm": "VLM"}.get(self._follow_source, "标定板")

    def _request_toggle(self, cmd: str) -> None:
        with self._lock:
            mode = self._mode
            busy = self._busy
        if busy:
            return
        src = self._follow_source
        if cmd == "src_board":
            src = "board"
        elif cmd == "src_charger":
            src = "charger"
        elif cmd == "src_vlm":
            src = "vlm"
        else:
            return
        if src == self._follow_source:
            return
        self._follow_source = src
        save_follow_session(follow_source=self._follow_source)

        def _push() -> None:
            try:
                proc = subprocess.run(
                    [sys.executable, str(CTRL), "source", src],
                    cwd=str(REPO),
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if proc.returncode != 0:
                    err = ((proc.stdout or "") + (proc.stderr or "")).strip()
                    self.get_logger().warn(f"set source failed: {err[-200:]}")
            except Exception as exc:
                self.get_logger().warn(f"set source: {exc}")

        threading.Thread(target=_push, daemon=True).start()
        with self._lock:
            if mode == "following":
                self._busy_msg = f"已切到{self._src_label()}（LIVE）"
            else:
                self._busy_msg = f"下次跟随：{self._src_label()}（WDLS）"
        self.get_logger().info(f"HUD session src={self._follow_source} live={mode}")

    def _request_jog(self) -> None:
        with self._lock:
            if self._busy:
                return
            if self._jog_open:
                self.close_jog()
                self._busy_msg = "已关闭六轴拖动"
                self._mode = "idle"
                return
            self._busy = True
            self._busy_msg = BUSY_TXT["jog"]

        def _run() -> None:
            try:
                # Pause follow without moving home; keep bridge for jog_fw.
                proc = subprocess.run(
                    [sys.executable, str(CTRL), "pause"],
                    cwd=str(REPO),
                    capture_output=True,
                    text=True,
                    check=False,
                )
                out = ((proc.stdout or "") + (proc.stderr or "")).strip()
                if proc.returncode != 0:
                    msg = out.splitlines()[-1] if out else "pause failed"
                    with self._lock:
                        self._busy_msg = f"ERROR: {msg}"
                    return
                # OpenCV windows must be created on the GUI thread. Doing it
                # here deadlocks the hub window.
                self._open_jog_requested = True
                with self._lock:
                    self._mode = "jogging"
                    self._busy_msg = "六轴拖动中（经 CDC 桥，不占串口）"
                time.sleep(2.0)
                with self._lock:
                    if not self._busy_msg.startswith("ERROR"):
                        self._busy_msg = ""
            except Exception as exc:
                with self._lock:
                    self._busy_msg = f"ERROR: {exc}"
            finally:
                with self._lock:
                    self._busy = False

        threading.Thread(target=_run, daemon=True).start()

    def pump_jog_ui(self) -> None:
        if not self._open_jog_requested:
            return
        self._open_jog_requested = False
        try:
            self.open_jog()
        except cv2.error as exc:
            self.get_logger().error(f"jog window: {exc}")
            with self._lock:
                self._mode = "idle"
                self._busy_msg = "六轴窗口没打开，再点一次"

    def open_jog(self) -> None:
        if self._jog_open:
            return
        if self._have_js:
            self._jog_cmd = np.array(self._fw, dtype=float)
            self._jog_synced = True
        else:
            self._jog_cmd = np.zeros(6, dtype=float)
            self._jog_synced = False
        self._jog_drag = None
        self._jog_pending = False
        self._jog_x0, self._jog_x1 = 150, 620
        # Drawn on the main hub image. A second Qt window has no handler here.
        self._jog_open = True
        self.get_logger().info("jog sliders on the hub window → /cdc_servo_bridge/jog_fw")

    def close_jog(self) -> None:
        if not self._jog_open:
            return
        self._jog_open = False
        self._jog_pending = False
        self._jog_synced = False
        self._jog_drag = None
        self._last_jog_sent = None

    def _sync_jog_from_js(self) -> None:
        if not self._jog_open or not self._have_js or self._jog_drag is not None:
            return
        self._jog_cmd = np.array(self._fw, dtype=float)
        self._jog_synced = True

    def _jog_from_x(self, idx: int, x: int) -> float:
        lo, hi = JOINT_LIMITS_FW_DEG[idx]
        x0, x1 = self._jog_x0, self._jog_x1
        t = float(np.clip((x - x0) / max(1, x1 - x0), 0.0, 1.0))
        return float(lo + t * (hi - lo))

    def on_jog_mouse(self, event: int, x: int, y: int, flags: int, _param) -> None:
        if not self._jog_open:
            return
        idx = None
        for i in range(6):
            row_y = 28 + i * 36
            if row_y - 4 <= y <= row_y + 26:
                idx = i
                break
        if event == cv2.EVENT_LBUTTONDOWN and idx is not None:
            self._jog_drag = idx
            self._jog_cmd[idx] = self._jog_from_x(idx, x)
            self._jog_pending = True
        elif event == cv2.EVENT_MOUSEMOVE and self._jog_drag is not None:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                i = self._jog_drag
                self._jog_cmd[i] = self._jog_from_x(i, x)
                self._jog_pending = True
        elif event == cv2.EVENT_LBUTTONUP:
            self._jog_drag = None

    def _tick_jog_send(self) -> None:
        if not self._jog_open:
            return
        if self._have_js and not self._jog_synced and self._jog_drag is None and not self._jog_pending:
            self._sync_jog_from_js()
            return
        if not self._jog_pending or self._mode == "following":
            return
        self._jog_pending = False
        deg = []
        for i in range(6):
            lo, hi = JOINT_LIMITS_FW_DEG[i]
            deg.append(float(np.clip(self._jog_cmd[i], lo, hi)))
        arr = np.asarray(deg, dtype=float)
        if self._last_jog_sent is not None and float(np.max(np.abs(arr - self._last_jog_sent))) < 0.05:
            return
        self._last_jog_sent = arr
        msg = Float64MultiArray()
        msg.data = deg
        self._jog_pub.publish(msg)

    def compose_view(self, frame: np.ndarray) -> np.ndarray:
        self._cam_h = int(frame.shape[0])
        if not self._jog_open:
            return frame
        w = int(frame.shape[1])
        self._jog_x0, self._jog_x1 = 148, max(260, w - 12)
        strip_h = 36 + 6 * 36
        strip = np.full((strip_h, w, 3), (42, 44, 48), dtype=np.uint8)
        _put_zh(strip, "六轴拖动：按住色条左右拖，再点按钮关闭", (8, 4), (235, 235, 235), size=15)
        x0, x1 = self._jog_x0, self._jog_x1
        for i in range(6):
            lo, hi = JOINT_LIMITS_FW_DEG[i]
            val = float(np.clip(self._jog_cmd[i], lo, hi))
            y = 28 + i * 36
            _put_zh(strip, f"J{i+1} {val:5.0f}°", (8, y - 2), (230, 236, 242), size=14)
            cv2.rectangle(strip, (x0, y + 4), (x1, y + 18), (70, 74, 80), -1)
            t = (val - lo) / max(1e-6, hi - lo)
            knob = int(x0 + float(np.clip(t, 0.0, 1.0)) * (x1 - x0))
            cv2.rectangle(strip, (x0, y + 4), (knob, y + 18), (40, 160, 230), -1)
            cv2.circle(strip, (knob, y + 11), 8, (250, 250, 250), -1)
        return np.vstack([frame, strip])

    def on_mouse(self, event: int, x: int, y: int, flags: int, _param) -> None:
        if self._jog_open and y >= getattr(self, "_cam_h", 10**9):
            self.on_jog_mouse(event, x, y - self._cam_h, flags, _param)
            return
        if event == cv2.EVENT_MOUSEMOVE:
            self._hit = None
            for cmd, rect in list(self._toggle_rects) + list(self._btn_rects):
                x0, y0, x1, y1 = rect
                if x0 <= x <= x1 and y0 <= y <= y1:
                    self._hit = cmd
                    break
            return
        if event != cv2.EVENT_LBUTTONUP:
            return
        for cmd, rect in list(self._toggle_rects) + list(self._btn_rects):
            x0, y0, x1, y1 = rect
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.request_cmd(cmd)
                return

    def _cb_js(self, msg: JointState) -> None:
        if len(msg.position) < 6:
            return
        fw = ros_rad_to_fw_deg(np.array(msg.position[:6], dtype=float))
        with self._lock:
            self._fw = fw
            self._have_js = True

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

    def _cb_track_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
            self._status_t = time.time()
        except json.JSONDecodeError:
            pass

    def _cb_chg_px(self, msg: PointStamped) -> None:
        self._chg_px = (float(msg.point.x), float(msg.point.y))
        self._chg_t = time.time()

    def _cb_chg_q(self, msg: Float32) -> None:
        self._chg_q = float(msg.data)

    def _cb_vlm_px(self, msg: PointStamped) -> None:
        self._vlm_px = (float(msg.point.x), float(msg.point.y))
        self._vlm_t = time.time()

    def _cb_vlm_q(self, msg: Float32) -> None:
        self._vlm_q = float(msg.data)

    def _cb_vlm_dbg(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        q = str(data.get("query") or "").strip()
        if q:
            self._vlm_query = q
        if "u" in data and "v" in data:
            self._vlm_box = (
                float(data["u"]),
                float(data["v"]),
                float(data.get("w") or 0.0),
                float(data.get("h") or 0.0),
                str(data.get("label") or q or self._vlm_query),
                str(data.get("reject") or ""),
                time.time(),
            )

    def _cb_tgt_q(self, msg: Float32) -> None:
        self._tgt_q = float(msg.data)
        self._tgt_q_t = time.time()

    def _tick_ee_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "link6_1_1", rclpy.time.Time())
            t = tf.transform.translation
            self._last_ee_xyz = (t.x, t.y, t.z)
        except Exception:
            pass

    def _tick_handeye_tf(self) -> None:
        try:
            self.tf_buffer.lookup_transform("link5_1_1", "camera_link", rclpy.time.Time())
            self._handeye_ok = True
        except Exception:
            self._handeye_ok = False

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
        if self.camera_matrix is not None:
            return int(round(self.camera_matrix[0, 2])), int(round(self.camera_matrix[1, 2]))
        return w // 2, h // 2

    def _draw_aim(self, bgr: np.ndarray, board_px: tuple[float, float] | None) -> str:
        h, w = bgr.shape[:2]
        cx, cy = self._aim_px(w, h)
        cyan = (255, 220, 0)
        arm = 22
        cv2.line(bgr, (cx - arm, cy), (cx + arm, cy), cyan, 2, cv2.LINE_AA)
        cv2.line(bgr, (cx, cy - arm), (cx, cy + arm), cyan, 2, cv2.LINE_AA)
        cv2.circle(bgr, (cx, cy), 10, cyan, 2, cv2.LINE_AA)
        cv2.putText(
            bgr, "AIM", (cx + 16, cy - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cyan, 2, cv2.LINE_AA
        )
        chg_fresh = (
            self._follow_source == "charger"
            and (time.time() - self._chg_t) < 0.4
            and self._chg_px is not None
        )
        if chg_fresh:
            nx, ny = int(round(self._chg_px[0])), int(round(self._chg_px[1]))
            mag = (180, 80, 255)
            cv2.drawMarker(bgr, (nx, ny), mag, cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
            cv2.circle(bgr, (nx, ny), 12, mag, 1, cv2.LINE_AA)
            cv2.putText(
                bgr, "CHG", (nx + 14, ny + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mag, 2, cv2.LINE_AA
            )
        vlm_fresh = (time.time() - self._vlm_t) < 1.5 and self._vlm_px is not None
        box = self._vlm_box
        box_fresh = box is not None and (time.time() - box[6]) < 2.0
        if box_fresh:
            u, v, bw, bh, label, reject, _ = box
            held = bool(reject)
            color = (0, 220, 255) if held else (0, 140, 255)
            if bw > 4 and bh > 4:
                x0, y0 = int(round(u - bw / 2)), int(round(v - bh / 2))
                x1, y1 = int(round(u + bw / 2)), int(round(v + bh / 2))
                cv2.rectangle(bgr, (x0, y0), (x1, y1), color, 1 if held else 2, cv2.LINE_AA)
            tag = self._vlm_query or label or "VLM"
            _put_zh(
                bgr,
                tag,
                (int(round(u)) + 8, max(4, int(round(v - bh / 2)) - 28)),
                color,
                size=22,
            )
        elif vlm_fresh:
            vx, vy = int(round(self._vlm_px[0])), int(round(self._vlm_px[1]))
            orn = (0, 165, 255)
            cv2.circle(bgr, (vx, vy), 14, orn, 2, cv2.LINE_AA)
            cv2.drawMarker(bgr, (vx, vy), orn, cv2.MARKER_TILTED_CROSS, 20, 2, cv2.LINE_AA)
            cv2.putText(
                bgr, "VLM", (vx + 14, vy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, orn, 2, cv2.LINE_AA
            )
        if board_px is None:
            if self._follow_source == "vlm":
                vlm_txt = f"vlm q={self._vlm_q:.2f}" if vlm_fresh or box_fresh else "vlm: —"
                return f"aim: 光轴主点   {vlm_txt}"
            if self._follow_source == "charger":
                chg_txt = f"chg q={self._chg_q:.2f}" if chg_fresh else "chg: —"
                return f"aim: 光轴主点   {chg_txt}"
            return "aim: 光轴主点   board: —"
        bx, by = int(round(board_px[0])), int(round(board_px[1]))
        cv2.circle(bgr, (bx, by), 8, (0, 255, 80), 2, cv2.LINE_AA)
        cv2.circle(bgr, (bx, by), 2, (0, 255, 80), -1, cv2.LINE_AA)
        cv2.line(bgr, (bx, by), (cx, cy), (0, 255, 80), 1, cv2.LINE_AA)
        dx, dy = bx - cx, by - cy
        pix = math.hypot(dx, dy)
        return f"aim vs board: {pix:.0f}px  d=[{dx:+.0f},{dy:+.0f}]  (0=已对中)"

    def _draw_hud(
        self, bgr: np.ndarray, local_seen: bool, board_px: tuple[float, float] | None
    ) -> np.ndarray:
        now = time.time()
        ros_seen = (now - self._last_pose_t) < 1.0
        des_ok = (now - self._last_des_t) < 1.0
        twist_fresh = (now - self._last_twist_t) < 0.5
        st = self._status if (now - self._status_t) < 1.5 else {}
        moving = bool(st.get("moving")) or (
            twist_fresh and self._last_twist_norm > 1e-3
        )

        if ros_seen:
            banner = (36, 120, 48)
        elif local_seen:
            banner = (0, 150, 210)
        else:
            banner = (50, 50, 190)

        with self._lock:
            busy = self._busy
            busy_msg = self._busy_msg
            mode = self._mode
        if busy or busy_msg.startswith("ERROR"):
            banner = (0, 110, 200)

        h, w = bgr.shape[:2]
        panel_h = 122
        overlay = bgr.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), (18, 18, 22), -1)
        cv2.rectangle(overlay, (0, 0), (w, 4), banner, -1)
        cv2.addWeighted(overlay, 0.45, bgr, 0.55, 0, bgr)

        _put_zh(bgr, "Dummy 跟随控制台", (12, 14), (245, 245, 245), size=22)
        mode_map = {
            "following": ("LIVE 跟随", (80, 220, 120)),
            "jogging": ("六轴拖动", (80, 180, 255)),
            "idle": ("待机", (180, 180, 180)),
        }
        mode_txt, mode_col = mode_map.get(mode, mode_map["idle"])
        _put_zh(bgr, mode_txt, (w - 150, 14), mode_col, size=20)

        aim_txt = self._draw_aim(bgr, board_px)

        z_cm = st.get("opt_z_cm")
        z_err = st.get("z_err_cm")
        xy_cm = st.get("xy_cm")
        phase = st.get("phase") or ("—" if mode != "following" else "?")
        src = st.get("follow_source") or self._follow_source
        want_z = {"charger": 10.0, "vlm": 12.0}.get(src, 20.0)
        if z_cm is None and self._last_pose_xyz is not None and ros_seen:
            z_cm = self._last_pose_xyz[2] * 100.0
            xy_cm = math.hypot(self._last_pose_xyz[0], self._last_pose_xyz[1]) * 100.0
            z_err = z_cm - want_z

        q = st.get("quality")
        if q is None and (now - self._tgt_q_t) < 0.5:
            q = self._tgt_q
        metrics = [
            f"相位 {phase}",
            f"目标 {({'charger': '充电头', 'vlm': 'VLM'}.get(str(src), '板'))}",
            f"q {q:.2f}" if q is not None else "q —",
            f"距 {z_cm:.1f}cm" if z_cm is not None else "距 —",
            f"Δz {z_err:+.1f}cm" if z_err is not None else "Δz —",
            f"偏心 {xy_cm:.1f}cm" if xy_cm is not None else "偏心 —",
            f"|v| {self._last_twist_norm*100:.1f}cm/s" if twist_fresh else "|v| —",
        ]
        _put_zh(bgr, "  ·  ".join(metrics), (12, 36), (220, 230, 240), size=16)
        _put_zh(bgr, aim_txt, (12, 56), (200, 220, 255), size=14)

        if mode == "jogging":
            board_line = "模式：六轴拖动（滑条窗口）· 点「开始跟随」会关闭拖动"
        elif ros_seen:
            board_line = "标定板：ROS 已检测"
        elif local_seen:
            board_line = "标定板：仅本地看见（检查 aruco_detector）"
        else:
            board_line = "标定板：丢失 — 请将 ORIGINAL id0 举入画面"
        chg_line = (
            f"充电头：q={self._chg_q:.2f}"
            if (now - self._chg_t) < 0.4
            else "充电头：未检出"
        )
        if not self._vlm_query:
            vlm_line = "VLM：浏览器打开 127.0.0.1:8770 输入"
        elif (now - self._vlm_t) < 1.5 and self._vlm_q > 0.0:
            vlm_line = f"VLM：{self._vlm_query}  q={self._vlm_q:.2f}"
        else:
            vlm_line = f"VLM：正在找 {self._vlm_query}"
        _put_zh(
            bgr,
            board_line + "  ·  " + chg_line + "  ·  " + vlm_line,
            (12, 74),
            (230, 230, 230),
            size=14,
        )
        self._draw_toggles(bgr, w)

        if busy_msg:
            _put_zh(bgr, busy_msg, (12, panel_h - 22), (0, 220, 255), size=16)

        chip_y = h - BAR_H - 28
        chg_on = (now - self._chg_t) < 0.4
        vlm_on = (now - self._vlm_t) < 1.5
        servo_ok = self._servo_code == 0 and (now - self._servo_t) < 1.0
        servo_on = self._servo_code is not None and (now - self._servo_t) < 1.0
        spray = bool(st.get("spray") or st.get("on_target"))
        chips = (
            ("CAL", self._handeye_ok, self._handeye_ok),
            ("DET", ros_seen or local_seen, ros_seen),
            ("TGT", des_ok, spray),
            ("SPRAY", spray, spray),
            ("CHG", chg_on, chg_on and self._chg_q >= 0.4),
            ("VLM", vlm_on, vlm_on and self._vlm_q >= 0.4),
            ("MOV", moving, moving),
            ("RUN", mode == "following", mode == "following"),
            ("JOG", mode == "jogging", mode == "jogging"),
            ("SRV", servo_on, servo_ok),
        )
        cw = 56
        gap = 4
        x0 = 6
        for i, (lab, on, good) in enumerate(chips):
            self._chip(bgr, x0 + i * (cw + gap), chip_y, lab, on, good, width=cw)

        if self._have_js:
            jtxt = " ".join(f"J{i+1}:{self._fw[i]:.0f}" for i in range(6))
            wall = abs(self._fw[2] - 20.0) < 1.5 or self._fw[4] >= 74.0
            _put_zh(bgr, jtxt, (12, chip_y - 22), (170, 190, 210), size=13)
            if wall and mode == "following":
                _put_zh(
                    bgr,
                    "软限位：J3≥20° 或 J5≤75° — 板太近/太侧，把板拿远或点停止跟随→回初始",
                    (12, chip_y - 40),
                    (0, 180, 255),
                    size=13,
                )

        self._draw_buttons(bgr)
        return bgr

    def _draw_toggles(self, bgr: np.ndarray, _w: int) -> None:
        with self._lock:
            mode = self._mode
            busy = self._busy
        locked = busy
        y0, y1 = 94, 116
        pair = (
            ("src_board", "标定板", self._follow_source == "board"),
            ("src_charger", "充电头", self._follow_source == "charger"),
            ("src_vlm", "VLM", self._follow_source == "vlm"),
        )
        rects: list[tuple[str, tuple[int, int, int, int]]] = []
        x = 12
        for cmd, label, on in pair:
            x1 = x + (56 if cmd == "src_vlm" else 72)
            fill = (70, 70, 70) if locked else ((40, 130, 80) if on else (50, 50, 55))
            if self._hit == cmd and not locked:
                fill = tuple(min(255, c + 35) for c in fill)
            cv2.rectangle(bgr, (x, y0), (x1, y1), fill, -1)
            cv2.rectangle(bgr, (x, y0), (x1, y1), (200, 200, 200), 1)
            _put_zh_center(bgr, label, (x, y0, x1, y1), (255, 255, 255), size=13)
            rects.append((cmd, (x, y0, x1, y1)))
            x = x1 + 4
        self._toggle_rects = rects

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
            mode = self._mode
        rects: list[tuple[str, tuple[int, int, int, int]]] = []
        for i, (cmd, label, color) in enumerate(BUTTONS):
            x0 = BTN_GAP + i * (bw + BTN_GAP)
            x1 = x0 + bw
            fill = color
            if busy:
                fill = (70, 70, 70)
            elif cmd == "jog" and mode == "jogging":
                fill = (40, 160, 220)
                label = "关闭拖动"
            elif cmd == "start" and mode == "following":
                fill = (40, 40, 190)
                label = "停止跟随"
            elif self._hit == cmd:
                fill = tuple(min(255, c + 40) for c in color)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), fill, -1)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (230, 230, 230), 1)
            text = "…" if busy else label
            _put_zh_center(bgr, text, (x0, y0, x1, y1), (255, 255, 255), size=18)
            rects.append((cmd, (x0, y0, x1, y1)))
        self._btn_rects = rects
        if busy_msg:
            _put_zh(bgr, busy_msg, (12, h - BAR_H - 28), (0, 220, 255), size=16)

    @staticmethod
    def _chip(
        img: np.ndarray,
        x: int,
        y: int,
        label: str,
        on: bool,
        good: bool,
        width: int = 70,
    ) -> None:
        color = (40, 180, 40) if good else ((0, 200, 255) if on else (60, 60, 60))
        cv2.rectangle(img, (x, y), (x + width, y + 22), color, -1)
        cv2.putText(
            img,
            label,
            (x + 4, y + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45 if width < 64 else 0.7,
            (255, 255, 255),
            1 if width < 64 else 2,
            cv2.LINE_AA,
        )


def main() -> None:
    rclpy.init()
    node = CamLiveView()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN, np.zeros((480, 640, 3), dtype=np.uint8))
    cv2.waitKey(1)
    cv2.setMouseCallback(WIN, node.on_mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.pump_jog_ui()
            if node.frame is not None:
                cv2.imshow(WIN, node.compose_view(node.frame))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in (ord("1"), ord("s")):
                node.request_cmd("start")
            elif key in (ord("2"), ord("h")):
                node.request_cmd("handeye")
            elif key in (ord("3"), ord("x")):
                node.request_cmd("stop")
            elif key in (ord("4"), ord("j")):
                node.request_cmd("jog")
    finally:
        node.close_jog()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
