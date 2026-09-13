#!/usr/bin/env python3
"""Drag joints through the running CDC bridge (no second serial writer).

  source /opt/ros/jazzy/setup.bash
  source dummy_moveit_ws/install/setup.bash
  python3 scripts/home/cdc_bridge_slider.py
"""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_limits_fw import JOINT_LIMITS_FW_DEG

RAD_VOLUMN = np.array([0.0, 0.0, 1.57079, 0.0, 0.0, 0.0])
RAD_DIRECT = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0])


def ros_rad_to_fw_deg(rad: np.ndarray) -> np.ndarray:
    return np.rad2deg((rad + RAD_VOLUMN) * RAD_DIRECT)

JOINT_NAMES = [f"J{i}" for i in range(1, 7)]
HINTS = [
    "底座左右（多动）",
    "肩 远近高低",
    "肘 远近高低",
    "腕滚",
    "相机俯仰（外壳）",
    "法兰旋转 — 不带动相机，标定少动",
]


class BridgeSlider(Node):
    def __init__(self) -> None:
        super().__init__("cdc_bridge_slider")
        self._lock = threading.Lock()
        self.fw = np.zeros(6)
        self._have = False
        self.pub = self.create_publisher(Float64MultiArray, "/cdc_servo_bridge/jog_fw", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState) -> None:
        if len(msg.position) < 6:
            return
        fw = ros_rad_to_fw_deg(np.array(msg.position[:6], dtype=float))
        with self._lock:
            self.fw = fw
            self._have = True

    def current_fw(self) -> np.ndarray | None:
        with self._lock:
            return self.fw.copy() if self._have else None

    def send_fw(self, deg: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in deg]
        self.pub.publish(msg)


class SliderApp(tk.Tk):
    def __init__(self, node: BridgeSlider) -> None:
        super().__init__()
        self.node = node
        self.title("Dummy 标定拖动（经 CDC 桥，不占串口）")
        self.geometry("720x420")
        self.vars = [tk.DoubleVar(value=0.0) for _ in range(6)]
        self._job: str | None = None
        self._syncing = False
        self._build()
        self.after(150, self._tick_ros)
        self.after(400, self._pull_once)

    def _build(self) -> None:
        ttk.Label(
            self,
            text="拖滑条移动。相机在 J5 外壳上：标定请多动 J1–J5，少动 J6。",
            padding=8,
        ).pack(fill=tk.X)
        mid = ttk.Frame(self, padding=8)
        mid.pack(fill=tk.BOTH, expand=True)
        for i in range(6):
            lo, hi = JOINT_LIMITS_FW_DEG[i]
            row = ttk.Frame(mid)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=JOINT_NAMES[i], width=4).pack(side=tk.LEFT)
            val = ttk.Label(row, text="—", width=8)
            val.pack(side=tk.RIGHT)

            def on_move(idx: int = i, label: ttk.Label = val, _=None):
                if self._syncing:
                    return
                label.config(text=f"{self.vars[idx].get():.1f}°")
                self._schedule()

            sc = ttk.Scale(
                row,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                variable=self.vars[i],
                command=lambda _e, fn=on_move: fn(),
            )
            sc.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
            sc.bind("<ButtonRelease-1>", lambda _e: self._send())
            ttk.Label(row, text=HINTS[i], width=22).pack(side=tk.LEFT)
        self.status = tk.StringVar(value="等待 /joint_states …")
        ttk.Label(self, textvariable=self.status, relief=tk.SUNKEN, anchor=tk.W, padding=6).pack(
            fill=tk.X, side=tk.BOTTOM
        )

    def _tick_ros(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.after(40, self._tick_ros)

    def _pull_once(self) -> None:
        fw = self.node.current_fw()
        if fw is None:
            self.status.set("还没有关节反馈 — 确认 CDC 桥在跑")
            self.after(400, self._pull_once)
            return
        self._syncing = True
        try:
            for i in range(6):
                self.vars[i].set(float(fw[i]))
        finally:
            self._syncing = False
        self.status.set(f"已同步 {[round(float(x), 1) for x in fw]}  可拖")

    def _schedule(self) -> None:
        if self._job:
            self.after_cancel(self._job)
        self._job = self.after(80, self._send)

    def _send(self) -> None:
        self._job = None
        deg = [float(v.get()) for v in self.vars]
        self.node.send_fw(deg)
        self.status.set(f"发送 {[round(x, 1) for x in deg]}")


def main() -> None:
    rclpy.init()
    node = BridgeSlider()
    app = SliderApp(node)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
