#!/usr/bin/env python3
"""HUD session prefs for the next follow start (Target source)."""

from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SESSION_YAML = REPO / "dummy_moveit_ws" / "dummy_vision" / "config" / "follow_session.yaml"

VALID_SOURCES = ("board", "charger", "vlm")


def load() -> dict:
    source = "board"
    try:
        text = SESSION_YAML.read_text(encoding="utf-8")
    except OSError:
        return {"follow_source": source}
    for line in text.splitlines():
        s = line.split("#", 1)[0].strip()
        if s.startswith("follow_source:"):
            val = s.split(":", 1)[1].strip().strip("'\"")
            if val == "cup":
                val = "charger"
            if val in VALID_SOURCES:
                source = val
    return {"follow_source": source}


def save(*, follow_source: str, follow_orientation: bool | None = None) -> None:
    del follow_orientation  # deprecated HUD mode; kept for call-site compat
    src = follow_source if follow_source in VALID_SOURCES else "board"
    if src == "cup":
        src = "charger"
    SESSION_YAML.parent.mkdir(parents=True, exist_ok=True)
    SESSION_YAML.write_text(
        (
            "# Written by cam_live_view HUD. Next 开始跟随 reads this.\n"
            "# Speeds / standoff / WDLS weights stay in follow_live.yaml.\n"
            "/**:\n"
            "  ros__parameters:\n"
            f"    follow_source: {src}\n"
        ),
        encoding="utf-8",
    )
