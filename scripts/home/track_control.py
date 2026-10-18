#!/usr/bin/env python3
"""Start / handeye / stow / pause helpers for the tracking HUD and CLI.

  python3 scripts/home/track_control.py start     # LIVE follow (optical, camera 20 cm)
  python3 scripts/home/track_control.py handeye   # cancel follow, go handeye
  python3 scripts/home/track_control.py stop      # cancel follow, fold to stow
  python3 scripts/home/track_control.py pause     # cancel follow, hold pose (for jog)

  python3 scripts/home/track_control.py hub       # vision nodes + handeye TF, no LIVE
  python3 scripts/home/track_control.py source vlm      # board|charger|vlm (OK mid-LIVE)

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
from follow_session import save as save_follow_session  # noqa: E402


def _follow_source() -> str:
    src = str(load_follow_session()["follow_source"])
    if src == "cup":
        return "charger"
    return src if src in ("board", "charger", "vlm") else "board"


# FW deg start poses (must match cdc_home_seven / bridge constants).
_START_POSE_FW = {
    "board": [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
    "charger": [0.0, -40.0, 120.0, 0.0, 70.0, 0.0],
    # Held pose 2026-10-18 19:33, rounded from [2.97, 0.4, 90.0, -0.04, 74.91, -0.04].
    "vlm": [3.0, 0.0, 90.0, 0.0, 75.0, 0.0],
}


def _start_preset(src: str | None = None) -> str:
    """Bridge/home preset name for the active follow source."""
    s = src or _follow_source()
    if s == "vlm":
        return "vlm"
    return "charger" if s == "charger" else "handeye"


def _bridge_ros_args() -> str:
    """CDC locks for WDLS Follow: J4 free (posture), J6 locked (CDC Quiet)."""
    return "-p joint_soft_half_range_deg:=0.0 -p lock_j4:=false -p lock_j6:=true"


def _apply_bridge_locks() -> None:
    """Push lock flags to a running bridge (ros2 param set)."""
    if not _bridge_alive():
        return
    env = os.environ.copy()
    for name, val in (("lock_j4", "false"), ("lock_j6", "true")):
        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"timeout -k 1 4 ros2 param set /cdc_servo_bridge {name} {val}"
        )
        try:
            subprocess.call(
                ["bash", "-lc", cmd],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=6,
            )
        except subprocess.TimeoutExpired:
            _log(f"WARN: param set {name} timed out")
    _log("bridge locks: lock_j4=false lock_j6=true (WDLS Follow)")


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
    """Pause Servo, stop Follow motion nodes, hold CDC so residual Servo cannot yank the arm."""
    _kill("wdls_follow_resolver")
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
        "charger": "/cdc_servo_bridge/goto_charger",
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


def _jog_fw(fw: list[float]) -> None:
    """Absolute FW move through the running bridge (no second serial writer)."""
    data = ",".join(str(float(x)) for x in fw)
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "ros2 topic pub --once /cdc_servo_bridge/jog_fw "
        f"std_msgs/msg/Float64MultiArray '{{data: [{data}]}}'"
    )
    subprocess.check_call(["bash", "-lc", cmd])


def _clear_vlm_query() -> None:
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "ros2 topic pub --once /tracking/vlm_query std_msgs/msg/String \"{data: ''}\""
    )
    subprocess.call(["bash", "-lc", cmd])


def _wait_near(src: str, timeout_s: float = 30.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _near_start_pose(src, tol_deg=12.0):
            return True
        time.sleep(0.4)
    return _near_start_pose(src, tol_deg=12.0)


def _home(preset: str) -> None:
    if preset == "vlm":
        if not _bridge_alive():
            _log("no CDC bridge — exclusive home (Servo is not the writer)")
            _home_exclusive(preset)
            return
        _log(f"jog to vlm start {_START_POSE_FW['vlm']}")
        _jog_fw(_START_POSE_FW["vlm"])
        if not _wait_near("vlm", 35.0):
            raise RuntimeError("未回到 VLM 桌面初始位")
        return
    if _bridge_alive():
        _goto_via_bridge(preset)
        return
    _log("no CDC bridge — exclusive home (Servo is not the writer)")
    _home_exclusive(preset)


def return_vlm_start() -> None:
    """Jog back to the VLM start pose. Keeps the phrase and leaves follow running."""
    _home("vlm")
    if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
        _log("WARN: release_servo after vlm return failed")
    _log("back at vlm start")


def release_arm() -> None:
    """Let Servo move again after a held return to the VLM start pose."""
    if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
        raise RuntimeError("release_servo failed")
    _log("servo released")


def rehome_vlm() -> None:
    """Drop the phrase, jog to the desk start, and keep Servo held.

    The page publishes the next phrase and waits for a box before release,
    so the first look happens at the start pose.
    """
    _clear_vlm_query()
    time.sleep(0.3)
    _home("vlm")
    if not _bridge_trigger("/cdc_servo_bridge/hold_servo", timeout_s=8):
        _log("WARN: hold_servo after vlm rehome failed")
    _log("vlm at start, servo held until the new phrase locks")


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
        _kill("charger_detector_node")
        _kill("vlm_locator_node")
        _kill("cup_detector_node")
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
        "charger_detector_node",
        "charger_detector_node",
        _params_files(),
    )
    # ROS node always OK; GPU sidecar is separate (start_locateanything_server.sh).
    _ros2_run_if_missing(
        "vlm_locator_node",
        "vlm_locator_node",
        _params_files(),
    )


def _set_mux_source(src: str) -> None:
    """Push follow_source to mux + WDLS resolver (LIVE mid-switch OK)."""
    if src == "cup":
        src = "charger"
    if src not in ("board", "charger", "vlm"):
        src = "board"
    env = os.environ.copy()
    for node in ("/tracking_target_mux", "/wdls_follow_resolver"):
        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"timeout 5 ros2 param set {node} follow_source {src}"
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


def _fw_from_joint_states(timeout_s: float = 5.0) -> list[float] | None:
    """Read one /joint_states and convert to FW degrees. None if unavailable."""
    import math
    import re

    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"timeout -k 1 {max(2, int(timeout_s))} ros2 topic echo /joint_states --once"
    )
    try:
        out = subprocess.check_output(
            ["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT, timeout=timeout_s + 5
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    # Parse "position:\\n- 0.1\\n- ..." (YAML-ish from ros2 topic echo).
    m = re.search(r"position:\s*\n((?:\s*-\s*[-\d.eE+]+\s*\n)+)", out)
    if not m:
        return None
    rad = [float(x) for x in re.findall(r"-\s*([-\d.eE+]+)", m.group(1))]
    if len(rad) < 6:
        return None
    rad_volumn = [0.0, 0.0, 1.57079, 0.0, 0.0, 0.0]
    rad_direct = [1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
    fw = [
        math.degrees((rad[i] + rad_volumn[i]) * rad_direct[i]) for i in range(6)
    ]
    return fw


def _near_start_pose(src: str | None = None, tol_deg: float = 20.0) -> bool:
    """True if current FW pose is close to the start pose for this follow source."""
    s = src or _follow_source()
    preset = _start_preset(s)
    target = _START_POSE_FW["charger" if preset == "charger" else "board"]
    if s == "vlm":
        target = _START_POSE_FW["vlm"]
    fw = _fw_from_joint_states()
    if fw is None:
        _log(f"WARN: could not read /joint_states — will go {preset} before follow")
        return False
    err = max(abs(a - b) for a, b in zip(fw, target))
    _log(
        f"pose vs {preset} max|Δ|={err:.1f}° now={[round(x, 1) for x in fw]} "
        f"tgt={target} (tol {tol_deg:.0f}°)"
    )
    return err <= tol_deg


def start_follow() -> None:
    """Start LIVE Follow via WDLS. Auto-homes to source start pose if far."""
    _kill("wdls_follow_resolver")
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

    src = _follow_source()
    preset = _start_preset(src)
    # Wrong camera aim (folded / side-looking) cannot Spray-Aim — home first.
    if not _near_start_pose(src, tol_deg=20.0):
        _log(f"not near {preset} — going {preset} before LIVE follow")
        _home(preset)
        time.sleep(0.3)

    if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
        time.sleep(0.4)
        if not _bridge_trigger("/cdc_servo_bridge/release_servo", timeout_s=10):
            raise RuntimeError("CDC 仍在 hold：Servo 指令被丢掉，臂不会动")

    if not FOLLOW_YAML.is_file():
        raise RuntimeError(f"missing follow params: {FOLLOW_YAML}")

    _ensure_tracking_nodes(restart=False)
    _set_mux_source(src)

    tr = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {WS}/install/setup.bash && "
        "exec ros2 run dummy_vision wdls_follow_resolver_node --ros-args "
        "-r __node:=wdls_follow_resolver "
        f"{_params_files()}"
    )
    _spawn("wdls_resolver", ["bash", "-lc", tr], WS)
    _unpause_servo()
    standoff = {"charger": "10cm", "vlm": "center-only, one lock"}.get(src, "20cm")
    _log(
        f"LIVE follow: WDLS resolver | optical {standoff} | src={src} | "
        f"start={preset} | J4 free / J6 lock | handeye={handeye_src} | "
        f"params={FOLLOW_YAML.name}+session"
    )


def hub_vision() -> None:
    """Bring up detector / mux / charger + handeye TF. Do not start LIVE tracker."""
    src = _ensure_handeye_tf()
    _ensure_tracking_nodes(restart=False)
    _set_mux_source(_follow_source())
    _log(f"hub vision ready handeye={src} (click 开始跟随 in Tracking Hub)")


def set_follow_source(src: str) -> None:
    """HUD / CLI: persist session + push mux & WDLS (works mid-LIVE)."""
    if src == "cup":
        src = "charger"
    if src not in ("board", "charger", "vlm"):
        src = "board"
    save_follow_session(follow_source=src)
    _set_mux_source(src)
    _log(f"follow_source={src} (mux+resolver)")


def go_handeye() -> None:
    """HUD「回初始」: board → handeye; charger → desk look-down start."""
    _stop_follow()
    preset = _start_preset()
    _home(preset)
    _log(f"at {preset} start, follow stopped")


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
        "source",
        "rehome",
        "vlm_return",
        "release",
    }:
        print(__doc__)
        return 2
    try:
        cmd = sys.argv[1]
        if cmd == "source":
            set_follow_source(sys.argv[2] if len(sys.argv) > 2 else "board")
        else:
            {
                "start": start_follow,
                "handeye": go_handeye,
                "stop": go_stow,
                "pause": pause_follow,
                "hub": hub_vision,
                "rehome": rehome_vlm,
                "vlm_return": return_vlm_start,
                "release": release_arm,
            }[cmd]()
    except Exception as exc:
        _log(f"ERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
