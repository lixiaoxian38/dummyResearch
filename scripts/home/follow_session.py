#!/usr/bin/env python3
"""HUD session prefs for the next follow start (source + orientation)."""

from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SESSION_YAML = REPO / "dummy_moveit_ws" / "dummy_vision" / "config" / "follow_session.yaml"

VALID_SOURCES = ("board", "nut")


def load() -> dict:
    source = "board"
    ori = False
    try:
        text = SESSION_YAML.read_text(encoding="utf-8")
    except OSError:
        return {"follow_source": source, "follow_orientation": ori}
    for line in text.splitlines():
        s = line.split("#", 1)[0].strip()
        if s.startswith("follow_source:"):
            val = s.split(":", 1)[1].strip().strip("'\"")
            if val in VALID_SOURCES:
                source = val
        elif s.startswith("follow_orientation:"):
            ori = s.split(":", 1)[1].strip().lower() in ("true", "1", "yes")
    return {"follow_source": source, "follow_orientation": ori}


def save(*, follow_source: str, follow_orientation: bool) -> None:
    src = follow_source if follow_source in VALID_SOURCES else "board"
    SESSION_YAML.parent.mkdir(parents=True, exist_ok=True)
    ori = "true" if follow_orientation else "false"
    SESSION_YAML.write_text(
        (
            "# Written by cam_live_view HUD. Next 开始跟随 reads this.\n"
            "# Speeds / standoff stay in follow_live.yaml.\n"
            "/**:\n"
            "  ros__parameters:\n"
            f"    follow_source: {src}\n"
            f"    follow_orientation: {ori}\n"
        ),
        encoding="utf-8",
    )
