#!/usr/bin/env python3
"""Browser page for VLM follow queries. http://127.0.0.1:8770

Sends text to /tracking/vlm_query and shows the locator debug stream.
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

PAGE = (Path(__file__).resolve().parent / "vlm_console.html").read_text(encoding="utf-8")
TRACK = Path(__file__).resolve().parents[1] / "home" / "track_control.py"
HOST = "127.0.0.1"
PORT = 8770


class VlmConsole(Node):
    def __init__(self) -> None:
        super().__init__("vlm_console")
        self.query = ""
        self.quality = 0.0
        self.debug: dict = {}
        self._lock = threading.Lock()
        self._pub = self.create_publisher(String, "/tracking/vlm_query", 10)
        self.create_subscription(String, "/tracking/vlm_debug", self._on_debug, 10)
        self.create_subscription(Float32, "/tracking/vlm_quality", self._on_q, 10)

    def _on_debug(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}
        with self._lock:
            self.debug = data
            if data.get("query"):
                self.query = str(data["query"])

    def _on_q(self, msg: Float32) -> None:
        with self._lock:
            self.quality = float(msg.data)

    def submit(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        self.get_logger().info(f"rehome before {text!r}")
        proc = subprocess.run(
            [sys.executable, str(TRACK), "rehome"],
            cwd=str(TRACK.parents[2]),
            capture_output=True,
            text=True,
            timeout=45,
            check=False,
        )
        if proc.returncode != 0:
            err = ((proc.stdout or "") + (proc.stderr or "")).strip()
            raise RuntimeError(err[-240:] or "rehome failed")
        msg = String()
        msg.data = text
        self._pub.publish(msg)
        with self._lock:
            self.query = text
            self.quality = 0.0
            self.debug = {"query": text, "raw": "已在初始位，正在识别"}
        self.get_logger().info(f"vlm query → {text!r} (held at start)")
        deadline = time.time() + 12.0
        locked = False
        while time.time() < deadline:
            with self._lock:
                dbg = dict(self.debug)
            if (
                str(dbg.get("query") or "") == text
                and dbg.get("u") is not None
                and not dbg.get("reject")
            ):
                locked = True
                break
            time.sleep(0.1)
        rel = subprocess.run(
            [sys.executable, str(TRACK), "release"],
            cwd=str(TRACK.parents[2]),
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        if rel.returncode != 0:
            err = ((rel.stdout or "") + (rel.stderr or "")).strip()
            raise RuntimeError(err[-240:] or "release failed")
        if not locked:
            self.get_logger().warn(f"no box for {text!r} at start pose; servo released")
        else:
            self.get_logger().info(f"locked {text!r} at start, servo released")

    def status(self) -> dict:
        with self._lock:
            dbg = dict(self.debug)
            q = self.quality
            query = self.query
        dbg["query"] = dbg.get("query") or query
        if "q" not in dbg:
            dbg["q"] = q
        return dbg


def main() -> None:
    rclpy.init()
    node = VlmConsole()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def handler_factory(*args, **kwargs):
        return Handler(node, *args, **kwargs)

    server = ThreadingHTTPServer((HOST, PORT), handler_factory)
    node.get_logger().info(f"识别指令页面 http://{HOST}:{PORT}")
    print(f"识别指令页面 http://{HOST}:{PORT}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


class Handler(BaseHTTPRequestHandler):
    def __init__(self, node: VlmConsole, *args, **kwargs) -> None:
        self._node = node
        super().__init__(*args, **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.startswith("/api/status"):
            body = json.dumps(self._node.status(), ensure_ascii=False).encode("utf-8")
            self._send(200, body, "application/json; charset=utf-8")
            return
        self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")

    def do_POST(self) -> None:  # noqa: N802
        if not self.path.startswith("/api/query"):
            self._send(404, b"{}", "application/json")
            return
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            self._send(400, b'{"ok":false}', "application/json")
            return
        text = str(data.get("text") or "").strip()
        if not text:
            self._send(400, b'{"ok":false,"error":"empty"}', "application/json")
            return
        try:
            self._node.submit(text)
        except Exception as exc:
            body = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False).encode("utf-8")
            self._send(500, body, "application/json; charset=utf-8")
            return
        self._send(200, b'{"ok":true}', "application/json")


if __name__ == "__main__":
    main()
