#!/usr/bin/env python3
"""Preflight checks before eye-in-hand calibration or tracking.

Usage:
  source /opt/ros/jazzy/setup.bash
  source dummy_moveit_ws/install/setup.bash
  python3 scripts/vision/check_calibration_ready.py
"""

from __future__ import annotations

import subprocess
import sys


FRAMES = [
    ("base_link", "link6_1_1"),
    ("link6_1_1", "camera_link"),
    ("camera_link", "camera_color_optical_frame"),
    ("camera_color_optical_frame", "camera_marker"),
]

TOPICS = [
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/joint_states",
]


def run(cmd: list[str]) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return r.returncode, (r.stdout + r.stderr).strip()
    except Exception as e:
        return 1, str(e)


def check_tf(parent: str, child: str) -> bool:
    code, out = run(
        ["ros2", "run", "tf2_ros", "tf2_echo", parent, child, "--once"]
    )
    ok = code == 0 and "Translation" in out
    mark = "OK" if ok else "MISSING"
    print(f"  [{mark}] {parent} -> {child}")
    if not ok and out:
        print(f"         {out.split(chr(10))[0][:80]}")
    return ok


def check_topic(topic: str) -> bool:
    code, out = run(["ros2", "topic", "info", topic])
    ok = code == 0 and "Publisher count" in out and "Publisher count: 0" not in out
    mark = "OK" if ok else "MISSING"
    print(f"  [{mark}] {topic}")
    return ok


def main() -> int:
    print("=== Eye-in-hand preflight ===\n")

    print("Packages:")
    for pkg in ("dummy_vision", "easy_handeye2", "realsense2_camera"):
        code, _ = run(["ros2", "pkg", "prefix", pkg])
        print(f"  [{'OK' if code == 0 else 'MISSING'}] {pkg}")

    print("\nTopics (need arm + camera + optional marker while board visible):")
    topic_ok = all(check_topic(t) for t in TOPICS[:2])
    for t in TOPICS[2:]:
        check_topic(t)

    print("\nTF chain (marker needs board in view during tracking):")
    tf_ok = True
    for p, c in FRAMES[:3]:
        if not check_tf(p, c):
            tf_ok = False
    print("  [optional while board visible]")
    check_tf(*FRAMES[3])

    calib = run(["bash", "-lc", "ls ~/.ros2/easy_handeye2/calibrations/dummy_eih_calib.calib 2>/dev/null"])
    has_calib = calib[0] == 0
    print(f"\nSaved calib dummy_eih_calib: {'YES' if has_calib else 'NO'}")

    print("\n--- Summary ---")
    if not topic_ok:
        print("Start RealSense: ros2 launch realsense2_camera rs_launch.py")
    if not tf_ok:
        print("Start arm TF: ros2 launch dummy_moveit_config servo_streaming.launch.py")
        print("Hand-eye: ros2 launch dummy_vision eye_in_hand_publish.launch.py")
    if topic_ok and tf_ok:
        print("Ready for calibration or dry_run tracking.")
    return 0 if topic_ok else 1


if __name__ == "__main__":
    sys.exit(main())
