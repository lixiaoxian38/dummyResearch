#!/usr/bin/env python3
"""Dummy 六轴拖动 GUI（CDC ASCII，不依赖 ROS）。

协议对齐 Windows DummyStudio：
  !START → #CMDMODE 2 → #GETJPOS → >j1,j2,j3,j4,j5,j6

用法：
  python3 scripts/home/dummy_slider_gui.py
  python3 scripts/home/dummy_slider_gui.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as e:
    raise SystemExit("需要 pyserial：sudo apt install python3-serial") from e


HOME_7 = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
# 宽松限位（度）
JOINT_LIMITS = [
    (-160.0, 160.0),
    (-120.0, 30.0),
    (0.0, 180.0),
    (-160.0, 160.0),
    (-120.0, 120.0),
    (-160.0, 160.0),
]
JOINT_NAMES = [f"J{i}" for i in range(1, 7)]


class DummySerial:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None
        self._lock = threading.Lock()

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.15)
        time.sleep(0.25)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def _read_for(self, wait: float) -> str:
        assert self.ser is not None
        t0 = time.time()
        buf = b""
        last = t0
        while time.time() - t0 < wait:
            chunk = self.ser.read(512)
            if chunk:
                buf += chunk
                last = time.time()
            elif time.time() - last > 0.12:
                break
            else:
                time.sleep(0.01)
        return buf.decode(errors="replace")

    def send(self, cmd: str, wait: float = 0.8) -> str:
        with self._lock:
            if not self.ser or not self.ser.is_open:
                raise RuntimeError("串口未打开")
            if not cmd.endswith("\n"):
                cmd += "\n"
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode("ascii", errors="ignore"))
            self.ser.flush()
            return self._read_for(wait)

    def start(self) -> str:
        return self.send("!START", wait=1.5)

    def cmdmode_realtime(self) -> str:
        return self.send("#CMDMODE 2", wait=1.0)

    def get_jpos(self) -> tuple[list[float] | None, str]:
        raw = self.send("#GETJPOS", wait=1.0)
        nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+", raw.replace("\r", " "))]
        if len(nums) >= 6:
            return nums[:6], raw
        return None, raw

    def set_jpos(self, angles: list[float], wait: float = 0.35) -> str:
        cmd = ">" + ",".join(f"{a:.3f}" for a in angles)
        return self.send(cmd, wait=wait)


class SliderApp(tk.Tk):
    def __init__(self, default_port: str):
        super().__init__()
        self.title("Dummy 六轴拖动（CDC ASCII）")
        self.geometry("640x520")
        self.minsize(560, 480)

        self.dev = DummySerial(default_port)
        self.vars = [tk.DoubleVar(value=HOME_7[i]) for i in range(6)]
        self._drag_job: str | None = None
        self._last_send = 0.0
        self._min_interval = 0.05
        self._busy = False

        self._build()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(200, self._auto_fill_ports)

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=self.dev.port)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18)
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新", command=self._auto_fill_ports).pack(side=tk.LEFT)

        self.btn_connect = ttk.Button(top, text="连接+START", command=self.connect_start)
        self.btn_connect.pack(side=tk.LEFT, padx=6)
        self.btn_disconnect = ttk.Button(
            top, text="断开", command=self.disconnect, state=tk.DISABLED
        )
        self.btn_disconnect.pack(side=tk.LEFT)

        mid = ttk.Frame(self, padding=8)
        mid.pack(fill=tk.BOTH, expand=True)

        for i in range(6):
            row = ttk.Frame(mid)
            row.pack(fill=tk.X, pady=4)
            lo, hi = JOINT_LIMITS[i]
            ttk.Label(row, text=JOINT_NAMES[i], width=4).pack(side=tk.LEFT)
            val_lbl = ttk.Label(row, text=f"{self.vars[i].get():.1f}°", width=8)
            val_lbl.pack(side=tk.RIGHT)

            def make_cmd(idx: int, label: ttk.Label):
                def on_move(_=None):
                    label.config(text=f"{self.vars[idx].get():.1f}°")
                    self._schedule_send()

                return on_move

            scale = ttk.Scale(
                row,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                variable=self.vars[i],
                command=make_cmd(i, val_lbl),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
            scale.bind("<ButtonRelease-1>", lambda _e: self._send_now())

        btns = ttk.Frame(self, padding=8)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="读当前角 #GETJPOS", command=self.read_pos).pack(side=tk.LEFT)
        ttk.Button(btns, text="回「7」字", command=self.go_home7).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="再发当前位置", command=self._send_now).pack(side=tk.LEFT)

        self.status = tk.StringVar(value="未连接。上电后点「连接+START」。")
        ttk.Label(
            self, textvariable=self.status, relief=tk.SUNKEN, anchor=tk.W, padding=6
        ).pack(fill=tk.X, side=tk.BOTTOM)

        ttk.Label(
            self,
            text="拖滑条即发送 >j1..j6（#CMDMODE 2）。急停请断电或拔 USB。",
            foreground="#444",
        ).pack(fill=tk.X, padx=8, pady=(0, 6))

    def _auto_fill_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        ports = sorted(ports, key=lambda d: (0 if "ACM" in d else 1, d))
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            acm = next((p for p in ports if "ACM" in p), ports[0])
            self.port_var.set(acm)

    def _set_status(self, msg: str) -> None:
        self.status.set(msg)
        self.update_idletasks()

    def connect_start(self) -> None:
        if self._busy:
            return
        self._busy = True
        try:
            if self.dev.is_open:
                self.dev.close()
            self.dev.port = self.port_var.get().strip()
            self._set_status(f"打开 {self.dev.port} …")
            self.dev.open()
            r1 = self.dev.start()
            r2 = self.dev.cmdmode_realtime()
            angles, raw = self.dev.get_jpos()
            if angles:
                self._apply_angles(angles)
                self._set_status(
                    f"已连接并 START。{r1.strip()} | {r2.strip()} | "
                    f"角={[round(a, 1) for a in angles]}"
                )
            else:
                self._apply_angles(HOME_7)
                self._set_status(f"已 START，解析角度失败，滑条置「7」字。原始: {raw!r}")
            self.btn_connect.config(state=tk.DISABLED)
            self.btn_disconnect.config(state=tk.NORMAL)
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            self._set_status(f"失败: {e}")
            self.dev.close()
        finally:
            self._busy = False

    def disconnect(self) -> None:
        self.dev.close()
        self.btn_connect.config(state=tk.NORMAL)
        self.btn_disconnect.config(state=tk.DISABLED)
        self._set_status("已断开")

    def _apply_angles(self, angles: list[float]) -> None:
        for i, a in enumerate(angles):
            lo, hi = JOINT_LIMITS[i]
            self.vars[i].set(max(lo, min(hi, a)))

    def _current_angles(self) -> list[float]:
        return [float(v.get()) for v in self.vars]

    def _schedule_send(self) -> None:
        if self._drag_job is not None:
            self.after_cancel(self._drag_job)
        self._drag_job = self.after(40, self._send_throttled)

    def _send_throttled(self) -> None:
        self._drag_job = None
        if time.time() - self._last_send < self._min_interval:
            self._schedule_send()
            return
        self._send_now()

    def _send_now(self, _event=None) -> None:
        if not self.dev.is_open or self._busy:
            return
        self._last_send = time.time()
        angles = self._current_angles()

        def work() -> None:
            try:
                resp = self.dev.set_jpos(angles)
                text = (
                    f"已发送 >{','.join(f'{a:.1f}' for a in angles)}  |  "
                    f"{resp.strip()[:60]}"
                )
                self.after(0, lambda: self._set_status(text))
            except Exception as e:
                self.after(0, lambda: self._set_status(f"发送失败: {e}"))

        threading.Thread(target=work, daemon=True).start()

    def read_pos(self) -> None:
        if not self.dev.is_open:
            messagebox.showwarning("未连接", "请先连接+START")
            return

        def work() -> None:
            try:
                angles, raw = self.dev.get_jpos()
                if angles:
                    self.after(0, lambda: self._apply_angles(angles))
                    self.after(
                        0,
                        lambda: self._set_status(f"读回 {[round(a, 2) for a in angles]}"),
                    )
                else:
                    self.after(0, lambda: self._set_status(f"解析失败: {raw!r}"))
            except Exception as e:
                self.after(0, lambda: self._set_status(f"读角失败: {e}"))

        threading.Thread(target=work, daemon=True).start()

    def go_home7(self) -> None:
        self._apply_angles(HOME_7)
        self._send_now()

    def _on_close(self) -> None:
        try:
            self.dev.close()
        finally:
            self.destroy()


def main() -> None:
    ap = argparse.ArgumentParser(description="Dummy 六轴 CDC 拖动 GUI")
    ap.add_argument("--port", default="/dev/ttyACM0")
    args = ap.parse_args()
    app = SliderApp(args.port)
    app.mainloop()


if __name__ == "__main__":
    main()
