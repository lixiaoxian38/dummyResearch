#!/usr/bin/env python3
"""Start / handeye / stow / pause helpers for the tracking HUD and CLI.

  python3 scripts/home/track_control.py start     # LIVE follow (optical, camera 20 cm)
  python3 scripts/home/track_control.py handeye   # cancel follow, go handeye
  python3 scripts/home/track_control.py stop      # cancel follow, fold to stow
  python3 scripts/home/track_control.py pause     # cancel follow, hold pose (for jog)

  python3 scripts/home/track_control.py hub       # vision nodes + handeye TF, no LIVE

Follow params: dummy_moveit_ws/dummy_vision/config/follow_live.yaml
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
FOLLOW_YAML = WS / "dummy_vision" / "config" / "follow_live.yaml"
SESSION_YAML = WS / "dummy_vision" / "config" / "follow_session.yaml"
CALIB_FILE = Path.home() / ".ros2/easy_handeye2/calibrations/dummy_eih_calib.calib"

sys.path.insert(0, str(HOME))
from follow_session import load as load_follow_session  # noqa: E402


def _follow_orientation_enabled() -> bool:
    return bool(load_follow_session()["follow_orientation"])


def _follow_source() -> str:
    return str(load_follow_session()["follow_source"])


def _bridge_ros_args() -> str:
    """CDC locks: J4 free when following orientation; J6 stays frozen."""
    if _follow_orientation_enabled():
        locks = "-p lock_j4:=false -p lock_j6:=true"
    else:
        locks = "-p lock_j4:=true -p lock_j6:=true"
    return f"-p joint_soft_half_range_deg:=0.0 {locks}"


def _apply_bridge_locks() -> None:
    """Push lock flags to a running bridge (ros2 param set)."""
    if not _bridge_alive():
        return
    lock_j4 = "false" if _follow_orientation_enabled() else "true"
    env = os.environ.copy()
    for name, val in (("lock_j4", lock_j4), ("lock_j6", "true")):
        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"timeout 5 ros2 param set /cdc_servo_bridge {name} {val}"
        )
        subprocess.call(["bash", "-lc", cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _log(f"bridge locks: lock_j4={lock_j4} lock_j6=true")


def find_dummy_cdc() -> str:
    env = os.environ.get("PORT")
    if env and Path(env).exists():
        return env
    ports = sorted(Path("/dev").glob("ttyACM*"))
    if ports:
        return str(ports[-1])
    raise RuntimeError("no /dev/ttyACM* — Dummy USB CDC missing")


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
        "timeout -k 1 5 ros2 service call /servo_node/pause_servo "
        "std_srvs/srv/SetBool '{data: true}' >/dev/null"
    )
    subprocess.call(["bash", "-lc", cmd], env=env)


def _unpause_servo() -> None:
    env = os.environ.copy()
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "timeout -k 1 5 ros2 service call /servo_node/pause_servo "
        "std_srvs/srv/SetBool '{data: false}' >/dev/null"
    )
    subprocess.call(["bash", "-lc", cmd], env=env)


def _bridge_trigger(srv: str, timeout_s: int = 8) -> bool:
    """Call a Trigger service. 2s was too short after goto/hold — start then
    left the CDC bridge ignoring Servo, so 中心+J4 looked frozen."""
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout {timeout_s} ros2 service call {srv} std_srvs/srv/Trigger"
    )
    try:
        out = subprocess.check_output(
            ["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT, timeout=timeout_s + 2
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        out = getattr(exc, "output", None) or str(exc)
        _log(f"WARN {srv} failed: {str(out)[-240:]}")
        return False
    ok = "success=True" in out or "success=true" in out
    if not ok:
        _log(f"WARN {srv} no success: {out.strip()[-240:]}")
    return ok


def _stop_follow(*, kill_bridge: bool = False) -> None:
    """Pause Servo, stop tracker, hold CDC so residual Servo cannot yank the arm."""
    _kill("aruco_servo_tracker")
    _kill("dummy_slider_gui.py")
    _pause_servo()
    if _bridge_alive():
        _bridge_trigger("/cdc_servo_bridge/hold_servo", timeout_s=8)
    if kill_bridge:
        _kill("cdc_servo_bridge.py")
    time.sleep(0.2)


def _bridge_alive() -> bool:
    return bool(_pids("cdc_servo_bridge.py"))


def _tf_ok(parent: str, child: str) -> bool:
    # Jazzy tf2_echo has no --once; it streams until timeout.
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout -k 1 3 ros2 run tf2_ros tf2_echo {parent} {child} -r 2"
    )
    try:
        out = subprocess.check_output(
            ["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT, timeout=6
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        out = getattr(exc, "output", None) or ""
    return "Translation" in (out or "")


def _ensure_handeye_tf() -> str:
    """Publish link5→camera if missing. Prefer easy_handeye2 calib file."""
    if _tf_ok("link5_1_1", "camera_link"):
        src = "easy_handeye" if CALIB_FILE.is_file() else "tf-already"
        _log(f"handeye TF link5_1_1→camera_link OK ({src})")
        return "easy" if CALIB_FILE.is_file() else "existing"

    use_easy = "true" if CALIB_FILE.is_file() else "false"
    if use_easy == "true":
        _log(f"publishing easy_handeye2 calib: {CALIB_FILE}")
    else:
        _log("WARNING: no dummy_eih_calib.calib — using static handeye_static.yaml")

    if _pids("eye_in_hand_publish") or _pids("handeye_static_tf") or _pids("easy_handeye"):
        _log("handeye publisher already running but TF missing — check frames")
    bash = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "exec ros2 launch dummy_vision eye_in_hand_publish.launch.py "
        f"use_easy_handeye:={use_easy} publish_optical_stub:=false"
    )
    _spawn("handeye_pub", ["bash", "-lc", bash], WS)
    for _ in range(20):
        time.sleep(0.25)
        if _tf_ok("link5_1_1", "camera_link"):
            _log("handeye TF ready")
            return "easy" if use_easy == "true" else "static"
    _log("WARNING: handeye TF still missing after publish launch")
    return "missing"


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
        [
            sys.executable,
            str(HOME / "cdc_home_seven.py"),
            "--port",
            port,
            "--preset",
            preset,
            "--steps",
            "0",
            "--speed",
            "40",
        ]
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


def _params_files() -> str:
    parts = [f"--params-file {FOLLOW_YAML}"]
    if SESSION_YAML.is_file():
        parts.append(f"--params-file {SESSION_YAML}")
    return " ".join(parts)


def _ros2_run_if_missing(needle: str, executable: str, extra: str = "") -> None:
    if _pids(needle):
        return
    bash = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"exec ros2 run dummy_vision {executable} --ros-args {extra}"
    )
    _spawn(executable, ["bash", "-lc", bash], WS)
    _log(f"started {executable}")


def _ensure_tracking_nodes(*, restart: bool = False) -> None:
    if restart:
        _kill("tracking_target_mux")
        _kill("nut_detector_node")
        _kill("aruco_detector_node")
        time.sleep(0.35)
    _ros2_run_if_missing("aruco_detector_node", "aruco_detector_node")
    _ros2_run_if_missing(
        "tracking_target_mux",
        "tracking_target_mux_node",
        f"-r __node:=tracking_target_mux {_params_files()}",
    )
    _ros2_run_if_missing(
        "nut_detector_node",
        "nut_detector_node",
        _params_files(),
    )


def _set_mux_source(src: str) -> None:
    env = os.environ.copy()
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout 5 ros2 param set /tracking_target_mux follow_source {src}"
    )
    subprocess.call(
        ["bash", "-lc", cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


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


def pause_follow() -> None:
    """Stop tracker / hold Servo; keep pose and keep bridge (for jog panel)."""
    _stop_follow(kill_bridge=False)
    if not _bridge_alive():
        bash = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"exec python3 -u {HOME}/cdc_servo_bridge.py {find_dummy_cdc()} --ros-args "
            f"{_bridge_ros_args()}"
        )
        _spawn("cdc_bridge", ["bash", "-lc", bash], REPO)
        time.sleep(1.2)
        _log("started CDC bridge for jog")
    _log("follow paused — arm held; bridge ready for /cdc_servo_bridge/jog_fw")


def start_follow() -> None:
    """Start / keep continuous LIVE follow from the current pose (no auto-home)."""
    _kill("aruco_servo_tracker")
    _kill("dummy_slider_gui.py")
    handeye_src = _ensure_handeye_tf()
    if not _bridge_alive():
        bash = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"exec python3 -u {HOME}/cdc_servo_bridge.py {find_dummy_cdc()} --ros-args "
            f"{_bridge_ros_args()}"
        )
        _spawn("cdc_bridge", ["bash", "-lc", bash], REPO)
        time.sleep(1.2)
    else:
        _apply_bridge_locks()
    if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
        time.sleep(0.4)
        if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
            raise RuntimeError("CDC 仍在 hold：Servo 指令被丢掉，臂不会动")

    if not FOLLOW_YAML.is_file():
        raise RuntimeError(f"missing follow params: {FOLLOW_YAML}")

    _ensure_tracking_nodes(restart=False)
    src = _follow_source()
    _set_mux_source(src)

    ori = "ori+J4" if _follow_orientation_enabled() else "pos-only"
    tr = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "exec ros2 run dummy_vision aruco_servo_tracker_node --ros-args "
        "-r __node:=aruco_servo_tracker "
        f"{_params_files()}"
    )
    _spawn("tracker", ["bash", "-lc", tr], WS)
    _unpause_servo()
    _log(
        f"LIVE follow: optical 20cm | src={src} | {ori} | "
        f"handeye={handeye_src} | params={FOLLOW_YAML.name}+session"
    )


def hub_vision() -> None:
    """Bring up detector / mux / nut + handeye TF. Do not start LIVE tracker."""
    src = _ensure_handeye_tf()
    _ensure_tracking_nodes(restart=False)
    _log(f"hub vision ready handeye={src} (click 开始跟随 in Tracking Hub)")


def go_handeye() -> None:
    _stop_follow()
    _home("handeye")
    _log("at handeye, follow stopped")


def go_stow() -> None:
    _stop_follow()
    _home("stow")
    _log("stowed, follow stopped")


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in {
        "start",
        "handeye",
        "stop",
        "pause",
        "hub",
    }:
        print(__doc__)
        return 2
    try:
        {
            "start": start_follow,
            "handeye": go_handeye,
            "stop": go_stow,
            "pause": pause_follow,
            "hub": hub_vision,
        }[sys.argv[1]]()
    except Exception as exc:
        _log(f"ERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
