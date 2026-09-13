#!/usr/bin/env python3
"""Start / handeye / stow helpers for the tracking HUD and CLI.

  python3 scripts/home/track_control.py start     # keep LIVE follow (J6 axis, 20 cm)
  python3 scripts/home/track_control.py handeye   # cancel follow, go handeye
  python3 scripts/home/track_control.py stop      # cancel follow, fold to stow
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOME = REPO / "scripts" / "home"
WS = REPO / "dummy_moveit_ws"
LOG_DIR = Path("/tmp/dummy_track_ctrl")


def find_dummy_cdc() -> str:
    env = os.environ.get("PORT")
    if env and Path(env).exists():
        return env
    ports = sorted(Path("/dev").glob("ttyACM*"))
    if ports:
        return str(ports[-1])
    raise RuntimeError("no /dev/ttyACM* — Dummy USB CDC missing")


PORT = os.environ.get("PORT", "/dev/ttyACM0")


def _log(msg: str) -> None:
    print(msg, flush=True)


def _pids(needle: str) -> list[int]:
    try:
        out = subprocess.check_output(["ps", "-C", "python3", "-o", "pid=,args="], text=True)
    except subprocess.CalledProcessError:
        return []
    found: list[int] = []
    for line in out.splitlines():
        if needle in line:
            found.append(int(line.split()[0]))
    # ros2 wrappers
    try:
        out2 = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
    except subprocess.CalledProcessError:
        out2 = ""
    for line in out2.splitlines():
        if needle in line and "cursorsandbox" not in line and "awk" not in line:
            try:
                found.append(int(line.split()[0]))
            except ValueError:
                pass
    return sorted(set(found))


def _kill(needle: str) -> None:
    pids = _pids(needle)
    if not pids:
        return
    _log(f"stop {needle}: {pids}")
    subprocess.call(["kill", *[str(p) for p in pids]])
    time.sleep(0.4)
    still = _pids(needle)
    if still:
        subprocess.call(["kill", "-9", *[str(p) for p in still]])


def _pause_servo() -> None:
    env = os.environ.copy()
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "timeout -k 1 3 ros2 service call /servo_node/pause_servo "
        "std_srvs/srv/SetBool '{data: true}' >/dev/null"
    )
    subprocess.call(["bash", "-lc", cmd], env=env)


def _bridge_trigger(srv: str, timeout_s: int = 8) -> None:
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout {timeout_s} ros2 service call {srv} std_srvs/srv/Trigger"
    )
    subprocess.call(["bash", "-lc", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop_follow(*, kill_bridge: bool = False) -> None:
    """Pause Servo, stop tracker, hold CDC so residual Servo cannot yank the arm."""
    _pause_servo()
    _kill("aruco_servo_tracker")
    _kill("dummy_slider_gui.py")
    if _bridge_alive():
        _bridge_trigger("/cdc_servo_bridge/hold_servo")
    if kill_bridge:
        _kill("cdc_servo_bridge.py")
    time.sleep(0.4)


def _bridge_alive() -> bool:
    return bool(_pids("cdc_servo_bridge.py"))


def _goto_via_bridge(preset: str) -> None:
    srv = {
        "handeye": "/cdc_servo_bridge/goto_handeye",
        "stow": "/cdc_servo_bridge/goto_stow",
    }[preset]
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout 120 ros2 service call {srv} std_srvs/srv/Trigger"
    )
    try:
        out = subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"bridge goto {preset} failed: {(exc.output or '')[-300:]}") from exc
    if "success=True" not in out and "success=true" not in out:
        raise RuntimeError(f"bridge goto {preset} failed: {out.strip()[-300:]}")
    _log(f"bridge goto {preset} ok")


def _home_exclusive(preset: str) -> None:
    port = find_dummy_cdc()
    busy = subprocess.call(["fuser", port], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if busy == 0:
        raise RuntimeError(f"{port} busy — another process holding serial")
    rc = subprocess.call(
        [sys.executable, str(HOME / "cdc_home_seven.py"), "--port", port, "--preset", preset, "--steps", "0", "--speed", "40"]
    )
    if rc == 2:
        raise RuntimeError("CDC 串口卡死（Write timeout）。不必重启电脑：拔插机械臂 USB，再点一次。")
    if rc != 0:
        raise RuntimeError(f"home {preset} failed rc={rc}")


def _home(preset: str) -> None:
    if _bridge_alive():
        _goto_via_bridge(preset)
        return
    _log("no CDC bridge — exclusive home (Servo is not the writer)")
    _home_exclusive(preset)


def _spawn(name: str, args: list[str], cwd: Path) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log = open(LOG_DIR / f"{name}.log", "ab")
    env = os.environ.copy()
    env.setdefault("ROS_HOME", f"/tmp/rh_{name}")
    Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    subprocess.Popen(
        args,
        cwd=str(cwd),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
    )


def start_follow() -> None:
    """Start / keep continuous LIVE follow from the current pose (no auto-home)."""
    _pause_servo()
    _kill("aruco_servo_tracker")
    _kill("dummy_slider_gui.py")
    if not _bridge_alive():
        bash = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"exec python3 -u {HOME}/cdc_servo_bridge.py {find_dummy_cdc()} --ros-args "
            "-p joint_soft_half_range_deg:=0.0"
        )
        _spawn("cdc_bridge", ["bash", "-lc", bash], REPO)
        time.sleep(1.2)
    _bridge_trigger("/cdc_servo_bridge/release_servo")
    # Image center on the board, camera distance 20 cm. No orientation chase.
    tr = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "exec ros2 run dummy_vision aruco_servo_tracker_node --ros-args "
        "-r __node:=aruco_servo_tracker "
        "-p dry_run:=false -p follow_orientation:=false "
        "-p control_frame:=optical -p hold_current_distance:=false "
        "-p desired_marker_in_ee_x:=0.0 -p desired_marker_in_ee_y:=0.0 "
        "-p desired_marker_in_ee_z:=0.20 "
        "-p desired_marker_rpy:=[0.0,0.0,0.0] "
        "-p ws_around_current:=0.15 "
        "-p max_marker_z:=0.60 -p max_marker_xy:=0.30 "
        "-p max_marker_jump:=0.22 -p lost_timeout_sec:=2.5 "
        "-p max_linear_vel:=0.08 -p max_angular_vel:=0.15 "
        "-p linear_gain:=1.2 -p angular_gain:=0.4 "
        "-p keep_in_view_xy:=0.10 -p keep_in_view_resume_xy:=0.06 "
        "-p replan_period_sec:=4.0 -p replan_reach_m:=0.015 "
        "-p hold_xy_m:=0.005 -p hold_z_m:=0.012 "
        "-p hold_resume_xy_m:=0.012 -p hold_resume_z_m:=0.025 "
        "-p center_first_xy_m:=0.035 -p replan_opt_change_m:=0.045 "
        "-p stall_sec:=4.0 -p ws_around_current:=0.22 "
        "-p opt_filter_alpha:=0.35 -p leave_hold_sec:=0.40 "
        "-p center_done_xy_m:=0.018 -p replan_min_sec:=0.80 "
        "-p trace_dir:=/tmp/dummy_track_ctrl/runs"
    )
    _spawn("tracker", ["bash", "-lc", tr], WS)
    _log("LIVE follow: image center on board, camera 20cm, latched replan")


def go_handeye() -> None:
    _stop_follow()
    _home("handeye")
    _log("at handeye, follow stopped")


def go_stow() -> None:
    _stop_follow()
    _home("stow")
    _log("stowed, follow stopped")


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in {"start", "handeye", "stop"}:
        print(__doc__)
        return 2
    try:
        {"start": start_follow, "handeye": go_handeye, "stop": go_stow}[sys.argv[1]]()
    except Exception as exc:
        _log(f"ERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
