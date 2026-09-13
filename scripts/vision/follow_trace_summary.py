#!/usr/bin/env python3
"""Print a short summary of a follow JSONL run.

  python3 scripts/vision/follow_trace_summary.py
  python3 scripts/vision/follow_trace_summary.py /tmp/dummy_track_ctrl/runs/follow_....jsonl
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _latest(d: Path) -> Path | None:
    files = sorted(d.glob("follow_*.jsonl"))
    return files[-1] if files else None


def main() -> int:
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        found = _latest(Path("/tmp/dummy_track_ctrl/runs"))
        if not found:
            print("no follow_*.jsonl under /tmp/dummy_track_ctrl/runs")
            return 1
        path = found
    ticks = []
    events = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            rec = json.loads(line)
            if rec.get("kind") == "tick":
                ticks.append(rec)
            else:
                events.append(rec)
    print(f"file {path}")
    print(f"events={len(events)} ticks={len(ticks)}")
    for e in events:
        print(f"  {e.get('kind')} {e.get('reason', '')} des={e.get('des')}")
    if not ticks:
        return 0
    modes = {}
    for t in ticks:
        modes[t.get("mode", "?")] = modes.get(t.get("mode", "?"), 0) + 1
    miss = [t.get("axis_miss_m") for t in ticks if t.get("axis_miss_m") is not None]
    along = [t.get("axis_along_m") for t in ticks if t.get("axis_along_m") is not None]
    err = [
        (sum(x * x for x in t["p_err"]) ** 0.5)
        for t in ticks
        if t.get("p_err")
    ]
    print("modes", modes)
    if err:
        print(f"|p_err| cm  first={err[0]*100:.1f} last={err[-1]*100:.1f} min={min(err)*100:.1f}")
    if miss:
        print(f"axis_miss cm first={miss[0]*100:.1f} last={miss[-1]*100:.1f} min={min(miss)*100:.1f}")
    if along:
        print(f"axis_along cm first={along[0]*100:.1f} last={along[-1]*100:.1f}")
    last = ticks[-1]
    print("last joints_fw", last.get("joints_fw"))
    print("last servo_fw ", last.get("servo_fw"))
    print("last des      ", last.get("des"))
    print("last ee       ", last.get("ee"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
