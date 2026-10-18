#!/usr/bin/env python3
"""LocateAnything-3B HTTP sidecar (run inside the locateanything venv).

  POST /detect  JSON { "image_b64": "...", "query": "charger" }
  → { "ok": true, "answer": "...", "boxes": [{u,v,w,h,label}], "dt_s": 0.4 }

Start:
  bash scripts/vision/install_locateanything.sh   # once
  bash scripts/vision/start_locateanything_server.sh
"""

from __future__ import annotations

import argparse
import base64
import io
import os
import re
import time
from typing import Any

from flask import Flask, jsonify, request
from PIL import Image

_BOX_RE = re.compile(
    r"<box><(\d+)><(\d+)><(\d+)><(\d+)></box>",
    re.IGNORECASE,
)
_POINT_RE = re.compile(r"<point><(\d+)><(\d+)></point>", re.IGNORECASE)

app = Flask(__name__)
WORKER = None
MODEL_ID = os.environ.get("LOCATEANYTHING_MODEL", "nvidia/LocateAnything-3B")


def parse_answer(answer: str, w: int, h: int) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    text = answer or ""
    for m in _BOX_RE.finditer(text):
        x1, y1, x2, y2 = (int(m.group(i)) / 1000.0 for i in range(1, 5))
        label = ""
        u1, v1, u2, v2 = x1 * w, y1 * h, x2 * w, y2 * h
        out.append(
            {
                "kind": "box",
                "u": 0.5 * (u1 + u2),
                "v": 0.5 * (v1 + v2),
                "w": abs(u2 - u1),
                "h": abs(v2 - v1),
                "label": label,
            }
        )
    for m in _POINT_RE.finditer(text):
        out.append(
            {
                "kind": "point",
                "u": int(m.group(1)) / 1000.0 * w,
                "v": int(m.group(2)) / 1000.0 * h,
                "w": 0.0,
                "h": 0.0,
                "label": "",
            }
        )
    return out


def load_worker() -> None:
    global WORKER
    eagle = os.environ.get(
        "EAGLE_EMBODIED",
        os.path.expanduser("~/models/eagle/Embodied"),
    )
    if eagle and os.path.isdir(eagle):
        import sys

        if eagle not in sys.path:
            sys.path.insert(0, eagle)
    from locateanything_worker import LocateAnythingWorker  # type: ignore

    local = os.environ.get("LOCATEANYTHING_LOCAL", "").strip()
    model = local or MODEL_ID
    try:
        WORKER = LocateAnythingWorker(
            model,
            use_batch_runtime=os.environ.get("LA_USE_BATCH", "1") == "1",
            attn=os.environ.get("LA_ATTN", "sdpa"),
        )
    except TypeError:
        WORKER = LocateAnythingWorker(model)
    print(f"[locateanything_server] loaded {model}", flush=True)


@app.get("/health")
def health():
    return jsonify({"ok": WORKER is not None, "model": MODEL_ID})


@app.post("/detect")
def detect():
    if WORKER is None:
        return jsonify({"ok": False, "error": "model not loaded"}), 503
    body = request.get_json(force=True, silent=True) or {}
    query = str(body.get("query") or "object").strip() or "object"
    b64 = body.get("image_b64") or ""
    if not b64:
        return jsonify({"ok": False, "error": "missing image_b64"}), 400
    try:
        raw = base64.b64decode(b64)
        img = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as exc:
        return jsonify({"ok": False, "error": f"bad image: {exc}"}), 400

    t0 = time.time()
    answer = ""
    try:
        res = WORKER.detect(img, [query])
        answer = str(res.get("answer", res))
    except Exception:
        try:
            res = WORKER.point(img, query)
            answer = str(res.get("answer", res))
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500
    dt = time.time() - t0
    boxes = parse_answer(answer, img.width, img.height)
    return jsonify(
        {
            "ok": True,
            "answer": answer,
            "boxes": boxes,
            "dt_s": round(dt, 3),
            "query": query,
            "wh": [img.width, img.height],
        }
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    load_worker()
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
