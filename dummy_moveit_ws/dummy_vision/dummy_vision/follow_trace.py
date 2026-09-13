"""Append-only JSONL recorder for one follow session.

Each line is one sample: observation, latched goal, twist, joints, Servo IK.
"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any


def _now_name() -> str:
    return datetime.now().strftime("follow_%Y%m%d_%H%M%S.jsonl")


class FollowTrace:
    def __init__(self, log_dir: str, *, every_n: int = 5) -> None:
        self.enabled = bool(log_dir)
        self.every_n = max(int(every_n), 1)
        self._n = 0
        self.path: Path | None = None
        self._fp = None
        if not self.enabled:
            return
        d = Path(log_dir).expanduser()
        d.mkdir(parents=True, exist_ok=True)
        self.path = d / _now_name()
        self._fp = self.path.open("a", encoding="utf-8")

    def close(self) -> None:
        if self._fp:
            self._fp.close()
            self._fp = None

    def write_event(self, kind: str, **fields: Any) -> None:
        if not self._fp:
            return
        rec = {"t": time.time(), "kind": kind, **fields}
        self._fp.write(json.dumps(rec, ensure_ascii=False, default=_json_default) + "\n")
        self._fp.flush()

    def maybe_sample(self, rec: dict[str, Any]) -> None:
        if not self._fp:
            return
        self._n += 1
        if self._n % self.every_n != 0:
            return
        rec = {"t": time.time(), "kind": "tick", **rec}
        self._fp.write(json.dumps(rec, ensure_ascii=False, default=_json_default) + "\n")


def _json_default(o: Any):
    if hasattr(o, "tolist"):
        return o.tolist()
    raise TypeError(type(o))


def ros_rad_to_fw_deg(rad: list[float] | tuple[float, ...]) -> list[float]:
    """Same mapping as scripts/home/cdc_servo_bridge.py."""
    vol = (0.0, 0.0, 1.57079, 0.0, 0.0, 0.0)
    direct = (1.0, 1.0, 1.0, 1.0, -1.0, -1.0)
    out = []
    for i, r in enumerate(list(rad)[:6]):
        out.append(math.degrees((float(r) + vol[i]) * direct[i]))
    return out
