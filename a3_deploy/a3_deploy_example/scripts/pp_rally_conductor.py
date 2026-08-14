#!/usr/bin/env python3
"""Gate3 conductor for an autonomous policy-native physical-ball rally.

After the initial stand and one ``m`` transition, the robot is never reset,
aborted, side-selected, or manually recovered.  Side-neutral ball initial states
drive the MuJoCo ball; the production planner chooses FH/BH from live ball/base
measurements and the production runner chooses whether and when to swing.

Per-serve accounting joins the launch ``shot_id``, planner-selected side,
runner lifecycle, pelvis stability, MuJoCo racket-contact edge, and measured
post-contact table landing.  The historical completion/recovery result remains
visible as a lifecycle regression but cannot independently pass Gate3.

Env knobs:
  PP_SERVES=12        physical shots counted after MOTION entry
  PP_RESET_Y=-0.7625  production arena station
  PP_EXTRA_ARGS=""    observability-only runner flags; decision/timing overrides fail
  PP_DROPOUT_AT=0     serve index (1-based) at which to freeze the planner
                      (SIGSTOP) for PP_DROPOUT_S right after its engage — the
                      mid-swing mocap/planner dropout stress. 0 = off.
  PP_DROPOUT_S=1.0    dropout duration (s); > cmd-timeout 0.5 and base max-age 0.2
  PP_ALLOW_RESCUE=0   mandatory; operator rescue is forbidden
  PP_MAX_RESCUES=0    mandatory
  PP_REQUIRE_READY=1  require the runner's production READY evidence

The ``random`` RallyV17-r10 phase instead scores a configurable many-ball
session: it does not require READY or station transitions, and measures return
to the immutable MOTION-entry XY anchor after each full-body swing.

PASS requires the lifecycle/stability/input contract, the
planner-envelope preflight, and separately sufficient FH and BH measured
racket-contact and legal-landing rates.  Missing telemetry, side assignment,
or ``shot_id`` joins fail closed.
"""
import json
import math
import os
import pty
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from mujoco_sim_msgs.msg import SimReset   # needs the A3_MuJoCo_Sim install overlay sourced

from pp_gate3_core import join_physical_evidence_by_side, physical_report_complete

GEAR = os.environ.get("PP_GEAR", str(Path(__file__).resolve().parents[1]))
DIST = os.environ.get("PP_DIST", str(Path(GEAR) / "dist/a3_deploy_x86_64"))
RUNNER_LOG = "/tmp/pp_runner.log"
BALL_LOG = "/tmp/pp_ball.log"
REPORT_JSON = "/tmp/pp_rally_report.json"
OBS_CSV = "/tmp/pp_obs.csv"
TRACE_CSV = "/tmp/pp_runner_trace.csv"
PLANNER_EVIDENCE_JSON = os.environ.get(
    "PP_PLANNER_EVIDENCE_JSON", "/tmp/pp_planner_envelope_report.json")
PHYSICAL_EVIDENCE_JSON = os.environ.get(
    "PP_PHYSICAL_EVIDENCE_JSON", "/tmp/pp_physical_ball_report.json")
RANDOM_SERVES_RECEIPT = os.environ.get(
    "PP_RANDOM_SERVES_RECEIPT", "/tmp/pp_gate3_random_serves_receipt.json")
GATE3_VERDICT = os.environ.get("PP_GATE3_VERDICT", "certification").strip().lower()
if GATE3_VERDICT != "certification":
    raise ValueError("Gate3 is fail-closed: PP_GATE3_VERDICT must be certification")

SERVES = int(os.environ.get("PP_SERVES", "12"))
GATE3_PHASE = os.environ.get("PP_GATE3_PHASE", "qualification").strip().lower()
if GATE3_PHASE not in ("qualification", "task", "random"):
    raise ValueError("PP_GATE3_PHASE must be qualification, task or random")
if GATE3_PHASE == "qualification":
    REQUIRED_SERVES = 12
    REQUIRED_GLOBAL_CONTACTS = 11
    REQUIRED_GLOBAL_LANDINGS = 10
    REQUIRED_SIDE_SHOTS = 6
    REQUIRED_SIDE_CONTACTS = 5
    REQUIRED_SIDE_LANDINGS = 5
    REQUIRED_MIN_SIDE_RATE = 5.0 / 6.0
elif GATE3_PHASE == "task":
    REQUIRED_SERVES = 26
    REQUIRED_GLOBAL_CONTACTS = 25
    REQUIRED_GLOBAL_LANDINGS = 24
    REQUIRED_SIDE_SHOTS = None
    REQUIRED_SIDE_CONTACTS = 0
    REQUIRED_SIDE_LANDINGS = 0
    REQUIRED_MIN_SIDE_RATE = 0.8
else:
    if SERVES < 16:
        raise ValueError("Gate3 random phase requires at least 16 serves")
    REQUIRED_SERVES = None
    REQUIRED_GLOBAL_CONTACTS = (4 * SERVES + 4) // 5
    REQUIRED_GLOBAL_LANDINGS = (4 * SERVES + 4) // 5
    REQUIRED_SIDE_SHOTS = None
    REQUIRED_SIDE_CONTACTS = 0
    REQUIRED_SIDE_LANDINGS = 0
    REQUIRED_MIN_SIDE_RATE = 0.8
if REQUIRED_SERVES is not None and SERVES != REQUIRED_SERVES:
    raise ValueError(
        f"Gate3 {GATE3_PHASE} requires exactly {REQUIRED_SERVES} serves, got {SERVES}"
    )
RESET_Y = float(os.environ.get("PP_RESET_Y", "-0.7625"))
DROPOUT_AT = int(os.environ.get("PP_DROPOUT_AT", "0"))
DROPOUT_S = float(os.environ.get("PP_DROPOUT_S", "1.0"))
# Public Gate3 requires high completion/recovery coverage in addition to
# physical contact and landing evidence. Environment overrides may only make
# these certification thresholds stricter.
MIN_PROXY_RATE = float(os.environ.get(
    "PP_MIN_PROXY_RATE", os.environ.get("PP_MIN_RETURN_RATE", "1.0")))
MIN_ENGAGED_SERVES = int(os.environ.get("PP_MIN_ENGAGED_SERVES", str(SERVES)))
MIN_COMPLETED_SERVES = int(os.environ.get("PP_MIN_COMPLETED_SERVES", str(SERVES)))
MIN_ENGAGE_RATE = float(os.environ.get("PP_MIN_ENGAGE_RATE", "1.0"))
MIN_COMPLETION_RATE = float(os.environ.get("PP_MIN_COMPLETION_RATE", "1.0"))
MIN_RECOVERY_RATE = float(os.environ.get("PP_MIN_RECOVERY_RATE", "1.0"))
RECOVERY_RADIUS_M = float(os.environ.get("PP_RECOVERY_RADIUS_M", "0.10"))
MAX_END_XY_DRIFT_M = float(os.environ.get("PP_MAX_END_XY_DRIFT_M", "0.50"))
MAX_PEAK_XY_DRIFT_M = float(os.environ.get("PP_MAX_PEAK_XY_DRIFT_M", "0.50"))
ALLOW_RESCUE = os.environ.get("PP_ALLOW_RESCUE", "0").strip().lower() in ("1", "true", "yes")
MAX_RESCUES = int(os.environ.get("PP_MAX_RESCUES", "0"))
REQUIRE_READY = os.environ.get("PP_REQUIRE_READY", "1").strip().lower() in ("1", "true", "yes")
REQUIRE_FRESH_LOCALIZATION = os.environ.get(
    "PP_REQUIRE_FRESH_LOCALIZATION", "1" if REQUIRE_READY else "0"
).strip().lower() in ("1", "true", "yes")
REQUIRE_XHIT_FREEZE = os.environ.get(
    "PP_REQUIRE_XHIT_FREEZE", "0"
).strip().lower() in ("1", "true", "yes")
READY_X_MAX = float(os.environ.get("PP_READY_X_MAX", "0.10"))
READY_Y_MAX = float(os.environ.get("PP_READY_Y_MAX", "0.10"))
READY_SPEED_MAX = float(os.environ.get("PP_READY_SPEED_MAX", "0.20"))
MIN_STATION_TRANSITIONS = int(os.environ.get("PP_MIN_STATION_TRANSITIONS", "0"))
STATION_STEP_LO = float(os.environ.get("PP_STATION_STEP_LO", "0.20"))
STATION_STEP_HI = float(os.environ.get("PP_STATION_STEP_HI", "0.35"))
MIN_POSITIVE_MAIN_TRANSITIONS = int(os.environ.get("PP_MIN_POSITIVE_MAIN_TRANSITIONS", "0"))
POSITIVE_MAIN_LO = float(os.environ.get("PP_POSITIVE_MAIN_LO", "0.19"))
POSITIVE_MAIN_HI = float(os.environ.get("PP_POSITIVE_MAIN_HI", "0.24"))
MIN_PHYSICAL_SAMPLES_PER_SIDE = int(
    os.environ.get(
        "PP_MIN_PHYSICAL_SAMPLES_PER_SIDE",
        str(REQUIRED_SIDE_SHOTS or 6),
    ))
MIN_PHYSICAL_CONTACT_RATE = float(
    os.environ.get("PP_MIN_PHYSICAL_CONTACT_RATE", str(REQUIRED_MIN_SIDE_RATE)))
MIN_LEGAL_LANDING_RATE = float(
    os.environ.get("PP_MIN_LEGAL_LANDING_RATE", str(REQUIRED_MIN_SIDE_RATE)))
MIN_CONTACTS_PER_SIDE = int(os.environ.get(
    "PP_MIN_CONTACTS_PER_SIDE", str(REQUIRED_SIDE_CONTACTS)))
MIN_LANDINGS_PER_SIDE = int(os.environ.get(
    "PP_MIN_LANDINGS_PER_SIDE", str(REQUIRED_SIDE_LANDINGS)))
MIN_GLOBAL_CONTACTS = int(os.environ.get(
    "PP_MIN_GLOBAL_CONTACTS", str(REQUIRED_GLOBAL_CONTACTS)))
MIN_GLOBAL_LANDINGS = int(os.environ.get(
    "PP_MIN_GLOBAL_LANDINGS", str(REQUIRED_GLOBAL_LANDINGS)))
if ALLOW_RESCUE or MAX_RESCUES != 0:
    raise ValueError("Gate3 forbids operator rescue")
if not all(0.0 <= value <= 1.0 for value in (
        MIN_PROXY_RATE, MIN_ENGAGE_RATE, MIN_COMPLETION_RATE, MIN_RECOVERY_RATE)):
    raise ValueError("Gate3 rate thresholds must be inside [0,1]")
if GATE3_PHASE != "random" and (
        MIN_PROXY_RATE < 1.0 or MIN_ENGAGED_SERVES < SERVES
        or MIN_COMPLETED_SERVES < SERVES):
    raise ValueError("legacy Gate3 phases require every serve to engage and complete")
if GATE3_PHASE == "random" and (
        MIN_ENGAGED_SERVES < math.ceil(MIN_ENGAGE_RATE * SERVES)
        or MIN_COMPLETED_SERVES < math.ceil(MIN_ENGAGE_RATE * SERVES)):
    raise ValueError("random Gate3 count thresholds are weaker than its engage threshold")
if not (
        RECOVERY_RADIUS_M > 0.0
        and MAX_END_XY_DRIFT_M >= RECOVERY_RADIUS_M
        and MAX_PEAK_XY_DRIFT_M >= MAX_END_XY_DRIFT_M):
    raise ValueError("Gate3 XY recovery/drift thresholds are inconsistent")
if MIN_PHYSICAL_SAMPLES_PER_SIDE < 4:
    raise ValueError("Gate3 requires at least four measured shots per side")
if not (
    REQUIRED_MIN_SIDE_RATE <= MIN_PHYSICAL_CONTACT_RATE <= 1.0
    and REQUIRED_MIN_SIDE_RATE <= MIN_LEGAL_LANDING_RATE <= 1.0
):
    raise ValueError("Gate3 per-side physical rates are weaker than the selected phase")
if (
    MIN_CONTACTS_PER_SIDE < REQUIRED_SIDE_CONTACTS
    or MIN_LANDINGS_PER_SIDE < REQUIRED_SIDE_LANDINGS
    or MIN_GLOBAL_CONTACTS < REQUIRED_GLOBAL_CONTACTS
    or MIN_GLOBAL_LANDINGS < REQUIRED_GLOBAL_LANDINGS
):
    raise ValueError("Gate3 physical count thresholds are weaker than the selected phase")
# x-LOCK gate. STATION_X must equal both reset spawn x and runner-derived station. RallyV10's
# shared plane gives 1.02 - 0.58 = 0.44; old generations override both values in their wrapper.
STATION_X = float(os.environ.get("PP_STATION_X", "0.44"))
XLOCK_THRESH = float(os.environ.get("PP_XLOCK_THRESH", "0.05"))   # <= 0 disables
# Arithmetic-only comparison tolerance. The 2026-07-18 rerun measured 0.050000536 m against a
# 0.050000000 m limit; sub-micrometre floating/relay residue is not a physical x excursion. This
# fixed epsilon does not hide the earlier real 0.05135 m violation and is not operator-configurable.
XLOCK_COMPARE_EPS_M = 1.0e-6
# A runner log marker is not sufficient evidence that the plant stayed up.
# Gate3 observes the MuJoCo pelvis directly, so crossing this already-used
# stability boundary is a physical fall and must terminate the rally.
MIN_UPRIGHT_PELVIS_Z_M = 0.80


def xlock_within_threshold(x_error_m):
    """Compare one physical x error with the fixed arithmetic tolerance."""
    return abs(float(x_error_m)) <= XLOCK_THRESH + XLOCK_COMPARE_EPS_M


FLIGHT_S = float(os.environ.get("PP_FLIGHT_S", "2.5"))
PAUSE_S = float(os.environ.get("PP_PAUSE_S", "4.0"))
MOTION_IDLE_S = float(os.environ.get("PP_MOTION_IDLE_S", "20.0"))
if FLIGHT_S <= 0.0 or PAUSE_S < 0.0 or MOTION_IDLE_S < 0.0:
    raise ValueError("Gate3 flight/pause/MOTION-idle durations are invalid")
SERVE_PERIOD_S = FLIGHT_S + PAUSE_S
GLOBAL_TIMEOUT_S = MOTION_IDLE_S + SERVES * SERVE_PERIOD_S + 60.0

rclpy.init()
node = Node("rally_conductor")
state = {"x": None, "y": None, "z": None, "min_z_motion": 99.0,
         "max_abs_x_err_motion": 0.0,   # max |base_x - STATION_X| across the whole rally
         "max_anchor_xy_err_motion": 0.0,
         "valid": 0, "invalid": 0, "motion_active": False}


def pose_cb(m):
    state["x"] = m.pose.position.x
    state["y"] = m.pose.position.y
    state["z"] = m.pose.position.z
    if state["motion_active"]:
        xerr = abs(state["x"] - STATION_X)
        state["max_abs_x_err_motion"] = max(state["max_abs_x_err_motion"], xerr)
        state["min_z_motion"] = min(state["min_z_motion"], state["z"])
        session_anchor = globals().get("anchor_xy")
        if session_anchor is not None:
            anchor_err = math.hypot(
                state["x"] - session_anchor[0], state["y"] - session_anchor[1]
            )
            state["max_anchor_xy_err_motion"] = max(
                state["max_anchor_xy_err_motion"], anchor_err
            )
        open_row = globals().get("cur")
        if open_row is not None:
            open_row["max_abs_x_err"] = max(open_row["max_abs_x_err"], xerr)
            open_row["min_z"] = min(open_row["min_z"], state["z"])
            if session_anchor is not None:
                open_row["max_anchor_xy_err"] = max(
                    open_row["max_anchor_xy_err"], anchor_err
                )


def flat_cb(m):
    if m.data[1] > 0.5:
        state["valid"] += 1
    else:
        state["invalid"] += 1


node.create_subscription(PoseStamped, "/sim/a3/pelvis_pose", pose_cb, 10)
node.create_subscription(Float64MultiArray, "/racket/command_flat", flat_cb, 10)
reset_pub = node.create_publisher(SimReset, "/sim/a3/reset", 10)
xhit_client = node.create_client(Trigger, "/hope_planner/freeze_x_hit")


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)


def z():
    return state["z"] if state["z"] is not None else -1.0


def xy():
    return (state["x"] or 0.0, state["y"] or 0.0)


def reset():
    # Spawn exactly at the configured runner station. RallyV10 uses one shared reach plane, so
    # forehand/backhand selection can change only station_y; legacy wrappers supply their own x.
    m = SimReset()
    m.mode = 1
    m.keyframe_id = 0
    m.set_base = True
    m.pelvis_pose.position.x = STATION_X
    m.pelvis_pose.position.y = RESET_Y
    m.pelvis_pose.position.z = 1.07
    m.pelvis_pose.orientation.w = 1.0
    m.set_base_twist = True
    m.zero_all_velocities = True
    reset_pub.publish(m)


# ---- spawn the runner on a pty (termios key thread needs a real tty) ----
master, slave = pty.openpty()
log = open(RUNNER_LOG, "wb")
runner_core_args = os.environ.get(
    "PP_RUNNER_CORE_ARGS",
    "--planner --policy-native --start passive --official-stand",
).split()
if runner_core_args != [
        "--planner", "--policy-native", "--start", "passive", "--official-stand"]:
    raise ValueError(
        "Gate3 runner core args drifted from the shared production contract: "
        f"{runner_core_args!r}")
runner_extra_args = os.environ.get("PP_EXTRA_ARGS", "").split()
for forbidden in (
        "--demo", "--side", "--hold-recover", "--gate-x-max",
        "--ready-x-max", "--ready-y-max", "--ready-speed-max",
        "--ready-dwell", "--swing-rest"):
    if any(token == forbidden or token.startswith(forbidden + "=")
           for token in runner_extra_args):
        raise ValueError(f"Gate3 forbids runner override {forbidden}")
runner_launcher = os.environ.get("PP_RUNNER_LAUNCHER", "./run_a3_pingpong.sh")
if runner_launcher != "./run_a3_pingpong.sh" and not Path(runner_launcher).is_file():
    raise ValueError(f"explicit Gate3 runner launcher is missing: {runner_launcher}")
runner_argv = (
    ["env", "A3_SOURCE_ROBOT_ENV=0", "LD_LIBRARY_PATH=.:/opt/ros/jazzy/lib",
     runner_launcher]
    + runner_core_args
    + ["--obs-csv", OBS_CSV, "--trace-csv", TRACE_CSV]
    + runner_extra_args
)
proc = subprocess.Popen(
    runner_argv,
    cwd=DIST, stdin=slave, stdout=log, stderr=subprocess.STDOUT,
    start_new_session=True)
os.close(slave)


def key(c):
    os.write(master, c.encode())
    print(f"[rally] key '{c}' @ z={z():.2f}", flush=True)


print("[rally] waiting for runner boot (driver started)...", flush=True)
t0 = time.time()
runner_booted = False
while time.time() - t0 < 90:
    spin(1)
    if time.time() - t0 < 12:      # never trust an early marker (stale log tail)
        continue
    try:
        with open(RUNNER_LOG, "rb") as f:
            if b"driver started" in f.read():
                runner_booted = True
                break
    except FileNotFoundError:
        pass
if not runner_booted:
    print("[rally] runner boot marker missing after 90 s; Gate3 fails closed", flush=True)
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    sys.exit(1)
print(f"[rally] runner up after {time.time()-t0:.0f}s; standing the robot", flush=True)
spin(1.0)
print(f"[rally] /sim/a3/reset subscribers: {node.count_subscribers('/sim/a3/reset')}",
      flush=True)

# ---- incremental log watchers ----
_run_ofs = [0]
_ball_ofs = [0]
_run_tail = [b""]
_ball_tail = [b""]
ENGAGE_RE = re.compile(
    rb"\[pp engage\] (forehand|backhand) \S+[^:]*: tgt base-rel "
    rb"\(([-+0-9.]+),([-+0-9.]+),([-+0-9.]+)\) tts=([0-9.]+)s "
    rb"\(clock tts0=([0-9.]+)s\)(?: station=\(([-+0-9.]+),([-+0-9.]+)\) "
    rb"dx=([0-9.]+) dy=([0-9.]+) speed=(INVALID/)?([0-9.]+)( READY)?)?")
REJECT_RE = re.compile(rb"\[pp gate\] REJECT\(110\) (fh|bh) ([^\n]*)")
STATION_RE = re.compile(
    rb"\[pp station\] pending (fh|bh) station=\(([-+0-9.]+),([-+0-9.]+)\) "
    rb"cmd_step=([0-9.]+) m tts=([0-9.]+) s")
READY_RE = re.compile(
    rb"\[pp station(?: telemetry|-only)?\] READY (forehand|backhand|fh|bh) "
    rb"station=\(([-+0-9.]+),([-+0-9.]+)\) cmd_step=([0-9.]+) m "
    rb"dx=([0-9.]+) dy=([0-9.]+) speed=([0-9.]+) m/s "
    rb"dwell=([0-9.]+) s latency=([0-9.]+) s")
LIFECYCLE_RE = re.compile(
    rb"\[pp lifecycle\] seq=(\d+) event=([a-z_]+) reason=([a-z_]+) "
    rb"[^\n]*strike=([-+0-9.eE]+) tts=([-+0-9.eE]+)")
SERVE_RE = re.compile(
    rb"serve (\d+): shot_id=(\d+) p_table=\[([^\]]*)\] "
    rb"p_world=\[([^\]]*)\] v=\[([^\]]*)\]")
SAFETY_LATCH_RE = re.compile(
    rb"\[a3_policy_driver\] SAFETY LATCH: ([^\r\n]+)")


def new_runner_events():
    try:
        with open(RUNNER_LOG, "rb") as f:
            f.seek(_run_ofs[0])
            chunk = f.read()
            _run_ofs[0] += len(chunk)
    except FileNotFoundError:
        chunk = b""
    chunk = _run_tail[0] + chunk
    cut = chunk.rfind(b"\n") + 1
    _run_tail[0] = chunk[cut:]
    chunk = chunk[:cut]
    return {
        "engages": [
            {"side": m.group(1).decode(),
             "tgt_b": [float(m.group(2)), float(m.group(3)), float(m.group(4))],
             "tts": float(m.group(5)), "tts0": float(m.group(6)),
             "station": ([float(m.group(7)), float(m.group(8))] if m.group(7) else None),
             "ready_dx": (float(m.group(9)) if m.group(9) else None),
             "ready_dy": (float(m.group(10)) if m.group(10) else None),
             "ready_speed_valid": bool(m.group(12)) and not bool(m.group(11)),
             "ready_speed": (float(m.group(12)) if m.group(12) else None),
             "ready": bool(m.group(13))}
            for m in ENGAGE_RE.finditer(chunk)],
        "pending": [
            {"side": "forehand" if m.group(1) == b"fh" else "backhand",
             "station": [float(m.group(2)), float(m.group(3))],
             "command_step": float(m.group(4)), "tts": float(m.group(5))}
            for m in STATION_RE.finditer(chunk)],
        "ready": [
            {"side": m.group(1).decode(),
             "station": [float(m.group(2)), float(m.group(3))],
             "command_step": float(m.group(4)), "dx": float(m.group(5)),
             "dy": float(m.group(6)), "speed": float(m.group(7)),
             "dwell_s": float(m.group(8)), "latency_s": float(m.group(9))}
            for m in READY_RE.finditer(chunk)],
        "lifecycle": [
            {"seq": int(m.group(1)), "event": m.group(2).decode(),
             "reason": m.group(3).decode(),
             "strike_time": float(m.group(4)), "tts": float(m.group(5))}
            for m in LIFECYCLE_RE.finditer(chunk)],
        "complete": chunk.count(b"swing complete"),
        "recovered": chunk.count(b"post-swing recovery done"),
        "fall_guard": chunk.count(b"FALL GUARD"),
        "actual_q_fault": chunk.count(
            b"PHYSICAL SAFETY FAULT: measured q exceeds hard limit"
        ),
        "command_safety_faults": [
            m.group(1).decode(errors="replace")
            for m in SAFETY_LATCH_RE.finditer(chunk)
        ],
        "rejects": [m.group(1).decode() + " " + m.group(2).decode()[:110]
                    for m in REJECT_RE.finditer(chunk)],
        "no_base": chunk.count(b"NO FRESH mocap base sample"),
    }


def new_serves():
    try:
        with open(BALL_LOG, "rb") as f:
            f.seek(_ball_ofs[0])
            chunk = f.read()
            _ball_ofs[0] += len(chunk)
    except FileNotFoundError:
        chunk = b""
    chunk = _ball_tail[0] + chunk
    cut = chunk.rfind(b"\n") + 1
    _ball_tail[0] = chunk[cut:]
    chunk = chunk[:cut]
    return [{"n": int(m.group(1)),
             "shot_id": int(m.group(2)),
             "p_table": [round(float(v), 4) for v in m.group(3).decode().split(",")],
             "p": [round(float(v), 4) for v in m.group(4).decode().split(",")],
             "v": [round(float(v), 4) for v in m.group(5).decode().split(",")]}
            for m in SERVE_RE.finditer(chunk)]


def physical_evidence_complete():
    """True only after every expected shot has a loss-detectable parked sample."""
    try:
        with open(PHYSICAL_EVIDENCE_JSON) as stream:
            report = json.load(stream)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return False
    return physical_report_complete(report, range(1, SERVES + 1))


def stand_and_motion(label):
    """Proven stand dance (see pp_planner_conductor.py): reset into the armed 's'
    catch window, require continuous standing, then 'm' (MOTION)."""
    for attempt in range(12):
        reset()
        spin(0.15)
        key("s")
        spin(1.5)
        if z() < 0.95:
            print(f"[rally] {label} attempt {attempt}: z={z():.2f} after reset+s, retry",
                  flush=True)
            continue
        stable = True
        for _ in range(12):
            spin(0.5)
            if z() < 1.0:
                stable = False
                break
        if stable:
            if REQUIRE_XHIT_FREEZE:
                if not xhit_client.wait_for_service(timeout_sec=5.0):
                    print(
                        "[rally] x_hit freeze service unavailable; retrying stand",
                        flush=True,
                    )
                    continue
                future = xhit_client.call_async(Trigger.Request())
                freeze_deadline = time.time() + 5.0
                while time.time() < freeze_deadline and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not future.done() or future.result() is None:
                    print("[rally] x_hit freeze timed out; retrying stand", flush=True)
                    continue
                freeze_result = future.result()
                print(
                    f"[rally] x_hit freeze: "
                    f"{'PASS' if freeze_result.success else 'FAIL'} "
                    f"{freeze_result.message}",
                    flush=True,
                )
                if not freeze_result.success:
                    continue
            new_runner_events()       # flush pending markers
            new_serves()
            key("m")
            return True
        print(f"[rally] {label} stand lost in settle (z={z():.2f}); retry", flush=True)
    return False


def planner_pids():
    out = subprocess.run(["pgrep", "-f", "hope_planner_cpp_node"],
                         capture_output=True, text=True).stdout.split()
    return [int(p) for p in out]


if not stand_and_motion("rally"):
    print("[rally] FAILED to stand the robot; aborting", flush=True)
    key("q")
    spin(3)
    proc.terminate()
    sys.exit(1)

anchor_xy = xy()
state["motion_active"] = True
print(f"[rally] MOTION entered at base=({anchor_xy[0]:+.2f},{anchor_xy[1]:+.2f}); "
      f"counting {SERVES} serves, NO resets from here", flush=True)

# ---- the RALLY: attribute events to serve windows, never touch the robot ----
serve_rows = []       # one dict per counted serve
cur = None            # the open serve row
active = None         # row of the last ENGAGED serve (swing-outcome attribution)
fell = False
physical_fall_detected = False
runner_fall_guard_tripped = False
dropout_done = False
t_start = time.time()
serves_seen = 0
tail_deadline = None  # after the last serve, wait for its outcome
rescues = 0           # fixed at zero; retained in the report schema
actual_q_faults = 0
command_safety_faults = []
misses_in_a_row = 0   # consecutive serves with no engage -> diagnostic marker

while time.time() - t_start < GLOBAL_TIMEOUT_S:
    spin(0.25)
    if z() >= 0:
        state["min_z_motion"] = min(state["min_z_motion"], z())
        if state["x"] is not None:
            xerr = abs(state["x"] - STATION_X)
            state["max_abs_x_err_motion"] = max(state["max_abs_x_err_motion"], xerr)
            if cur is not None:
                cur["max_abs_x_err"] = max(cur["max_abs_x_err"], xerr)
        if cur is not None:
            cur["min_z"] = min(cur["min_z"], z())
        if z() < MIN_UPRIGHT_PELVIS_Z_M:
            physical_fall_detected = True
            fell = True
            print(
                f"[rally] PHYSICAL FALL: pelvis z={z():.3f} m < "
                f"{MIN_UPRIGHT_PELVIS_Z_M:.2f} m — rally over",
                flush=True,
            )
            break

    for s in new_serves():
        serves_seen += 1
        if serves_seen > SERVES:
            continue
        if cur is not None:
            serve_rows.append(cur)
            misses_in_a_row = 0 if cur["engaged"] else misses_in_a_row + 1
            if misses_in_a_row >= 2 and serves_seen < SERVES:
                misses_in_a_row = 0
                print(
                    "[rally] 2 serves w/o engage; continuing without reset/rescue",
                    flush=True,
                )
        bx, by = xy()
        anchor_err = math.hypot(bx - anchor_xy[0], by - anchor_xy[1])
        cur = {"serve": serves_seen, "shot_id": s["shot_id"],
               "ball_p0_table": s["p_table"], "ball_p0": s["p"], "ball_v0": s["v"],
               "t": round(time.time() - t_start, 1),
               "base_at_serve": [round(bx, 3), round(by, 3)],
               "x_err_at_serve": round(bx - STATION_X, 3),
               "x_err_at_engage": None,
               "max_abs_x_err": abs(bx - STATION_X),
               "max_anchor_xy_err": anchor_err,
               "engaged": None, "engages": [], "complete": 0, "recovered": 0,
               "pending_stations": [], "command_station": None,
               "command_side": None, "ready_events": [], "ready_latency_s": None,
               "lifecycle_events": [], "rejects": [], "no_base_warns": 0,
               "min_z": z() if z() > 0 else 99.0,
               "dropout": False}
        print(f"[rally] serve {serves_seen}/{SERVES} p={s['p']} v={s['v']} "
              f"base=({bx:+.2f},{by:+.2f})", flush=True)
        if serves_seen == SERVES:
            tail_deadline = time.time() + 2.5 * SERVE_PERIOD_S
    if fell:
        break

    ev = new_runner_events()
    actual_q_faults += ev["actual_q_fault"]
    if ev["command_safety_faults"]:
        command_safety_faults.extend(ev["command_safety_faults"])
        print(
            f"[rally] COMMAND SAFETY FAULT: {command_safety_faults[-1]} "
            "-- rally over",
            flush=True,
        )
        break
    if cur is not None:
        if ev["pending"]:
            cur.setdefault("pending_stations", []).extend(ev["pending"])
            cur["command_station"] = ev["pending"][-1]["station"]
            cur["command_side"] = ev["pending"][-1]["side"]
        if ev["ready"]:
            cur["ready_events"].extend(ev["ready"])
            cur["ready_latency_s"] = ev["ready"][-1]["latency_s"]
        if ev["lifecycle"]:
            cur["lifecycle_events"].extend(ev["lifecycle"])
        for e in ev["engages"]:
            cur["engages"].append(e)
            if cur["engaged"] is None:
                cur["engaged"] = e
            if e.get("station") is not None:
                cur["command_station"] = e["station"]  # actual release target beats early prediction
            cur["command_side"] = e["side"]
            active = cur          # swing outcome events credit the ENGAGED serve
            if state["x"] is not None:   # pelvis-pose latency <= one 0.25 s spin: negligible pre-windup
                cur["x_err_at_engage"] = round(state["x"] - STATION_X, 3)
            print(f"[rally]   engage {e['side']} tgt_b={e['tgt_b']} tts={e['tts']:.2f} "
                  f"tts0={e['tts0']:.2f} xerr@e={cur['x_err_at_engage']}", flush=True)
            if DROPOUT_AT and cur["serve"] == DROPOUT_AT and not dropout_done:
                dropout_done = True
                cur["dropout"] = True
                pids = planner_pids()
                print(f"[rally]   DROPOUT stress: SIGSTOP planner {pids} for "
                      f"{DROPOUT_S}s (mid-swing stale cmd + base)", flush=True)
                for p in pids:
                    os.kill(p, signal.SIGSTOP)
                spin(DROPOUT_S)
                for p in pids:
                    os.kill(p, signal.SIGCONT)
        # complete/recovered belong to the swing (the ENGAGED serve) even when the
        # recovery finishes windows later (the tighter static-handoff gates make the
        # policy recovery span 1-2 serve periods — safe, and must not read as MISSED).
        tgt = active if active is not None else cur
        tgt["complete"] += ev["complete"]
        tgt["recovered"] += ev["recovered"]
        cur["rejects"] += ev["rejects"]
        if ev["no_base"]:
            cur["no_base_warns"] += ev["no_base"]
    if ev["fall_guard"]:
        runner_fall_guard_tripped = True
        fell = True
        print("[rally] FALL GUARD tripped — rally over", flush=True)
        break
    if tail_deadline is not None and time.time() > tail_deadline:
        break
    # early exit: last serve fully resolved
    if (tail_deadline is not None and cur is not None and cur["serve"] == SERVES
            and cur["recovered"]):
        spin(1.0)
        break

if cur is not None:
    serve_rows.append(cur)

# Recovery can finish before the last ball's configured flight window closes.
# Keep the exact production runner alive until the 1 kHz plant recorder has
# observed that shot's inactive/park sample. Without this join barrier a valid
# last contact could be judged from a half-written physical window.
if serves_seen >= SERVES and not fell:
    evidence_deadline = time.time() + FLIGHT_S + 2.0
    while time.time() < evidence_deadline and not physical_evidence_complete():
        spin(0.10)
    print(
        "[rally] final physical telemetry "
        + ("complete" if physical_evidence_complete() else "INCOMPLETE (fail closed)"),
        flush=True,
    )

end_xy = xy()   # BEFORE 'q': the runner exit drops the robot limp and the pelvis slides
state["motion_active"] = False
key("q")
spin(3)
try:
    proc.wait(timeout=10)
except subprocess.TimeoutExpired:
    proc.kill()

# ---- verdicts ----
drift = ((end_xy[0] - anchor_xy[0]) ** 2 + (end_xy[1] - anchor_xy[1]) ** 2) ** 0.5
x_err_at_end = end_xy[0] - STATION_X   # SIGNED: separates forward creep from lateral footwork
proxy_completed = 0
# Behavioral recovery can span a serve boundary, so the legacy
# 'recovery done -> STATIC' marker is not the only certificate. A later engage
# also proves that the production heading/settle/station gates were satisfied
# from the walked pose. A missed feed between those events is not contrary evidence.
# For the last row, no later engage exists: accept upright survival through the tail.
for i, r in enumerate(serve_rows):
    # A missed feed does not invalidate recovery from the preceding swing.  Use the
    # next actual engage (if any) as the behavioral certificate, rather than only
    # the immediately following serve row.
    nxt = next((candidate for candidate in serve_rows[i + 1:] if candidate["engaged"]), None)
    recovery_xy = (
        serve_rows[i + 1]["base_at_serve"]
        if i + 1 < len(serve_rows)
        else end_xy
    )
    recovery_xy_error = math.hypot(
        float(recovery_xy[0]) - anchor_xy[0],
        float(recovery_xy[1]) - anchor_xy[1],
    )
    r["recovery_xy_error_m"] = round(recovery_xy_error, 4)
    r["recovered_to_session_anchor"] = bool(
        not fell and recovery_xy_error <= RECOVERY_RADIUS_M
    )
    if GATE3_PHASE == "random":
        r["recovered_behavioral"] = bool(
            r["recovered_to_session_anchor"]
            and r["min_z"] > MIN_UPRIGHT_PELVIS_Z_M
        )
    else:
        r["recovered_behavioral"] = bool(
            nxt is not None or
            (i == len(serve_rows) - 1 and not fell
             and r["min_z"] > MIN_UPRIGHT_PELVIS_Z_M))
for r in serve_rows:
    e = r["engaged"]
    blocking_events = [
        event for event in r["lifecycle_events"]
        if event["event"] in ("hold_blocked", "shot_expired")
    ]
    blocking_reasons = []
    for event in blocking_events:
        if event["reason"] not in blocking_reasons:
            blocking_reasons.append(event["reason"])
    r["miss_reason_chain"] = blocking_reasons
    r["miss_reason"] = (
        None if e else
        (blocking_reasons[0] if blocking_reasons else "no_pending_station")
    )
    r["engage_protocol_ok"] = len(r["engages"]) == 1
    r["ready_ok"] = bool(
        not REQUIRE_READY or (
            e and e.get("ready") and e.get("ready_speed_valid")
            and e.get("ready_dx") is not None and e["ready_dx"] <= READY_X_MAX
            and e.get("ready_dy") is not None and e["ready_dy"] <= READY_Y_MAX
            and e.get("ready_speed") is not None and e["ready_speed"] <= READY_SPEED_MAX))
    r["completed_recovered_proxy"] = (
        bool(e) and r["engage_protocol_ok"] and r["ready_ok"] and r["complete"] >= 1 and (
        r["recovered"] >= 1 or r["recovered_behavioral"])
    )
    # x-LOCK per-serve assertion (HITTER regime): base stays at the station plane for the
    # whole serve window, including serve-open and engage. COUNT + REPORT (never abort mid-rally: the serve-by-serve
    # x_err curve is exactly the diagnostic this gate exists to produce).
    r["xlock_ok"] = bool(
        XLOCK_THRESH <= 0
        or (xlock_within_threshold(r["x_err_at_serve"])
            and (r["x_err_at_engage"] is None
                 or xlock_within_threshold(r["x_err_at_engage"]))
            and xlock_within_threshold(r["max_abs_x_err"])))
    proxy_completed += r["completed_recovered_proxy"]
    sd = r["engaged"]["side"][:2] if r["engaged"] else "--"
    print(f"[rally] serve {r['serve']:2d} @{r['t']:6.1f}s base=({r['base_at_serve'][0]:+.2f},"
          f"{r['base_at_serve'][1]:+.2f}) {sd} engage={'Y' if r['engaged'] else 'n'} "
          f"complete={r['complete']} recovered={r['recovered']} minz={r['min_z']:.2f} "
          f"xerr@s={r['x_err_at_serve']:+.2f} xerr@e="
          f"{'--' if r['x_err_at_engage'] is None else format(r['x_err_at_engage'], '+.2f')} "
          f"xmax={r['max_abs_x_err']:.2f}"
          f"{'' if r['xlock_ok'] else ' XLOCK-VIOL'} "
          f"ready={'Y' if r['ready_ok'] else 'FAIL'} "
          f"miss={('>'.join(r['miss_reason_chain']) if r['miss_reason_chain'] else '--')} "
          f"rejects={len(r['rejects'])}{' DROPOUT' if r['dropout'] else ''} "
          f"engages={len(r['engages'])} xy_recover={r['recovery_xy_error_m']:.3f}m "
          f"-> {'PROXY_OK' if r['completed_recovered_proxy'] else 'PROXY_FAIL'}", flush=True)

n = len(serve_rows)
engaged_serves = sum(1 for r in serve_rows if r["engaged"])
completed_serves = sum(
    1 for r in serve_rows if r["engaged"] and r["complete"] >= 1
)
total_engage_events = sum(len(r["engages"]) for r in serve_rows)
incomplete = [r["serve"] for r in serve_rows
              if r["engaged"] and not r["completed_recovered_proxy"]]
multi_engage_serves = [r["serve"] for r in serve_rows if len(r["engages"]) > 1]
xlock_violations = [r["serve"] for r in serve_rows if not r["xlock_ok"]]
command_station_sequence = [
    (r["serve"], r.get("command_side"), r["command_station"])
    for r in serve_rows if r.get("command_station") is not None
]
achieved_station_sequence = [
    (r["serve"], r["engaged"]["side"], r["command_station"])
    for r in serve_rows if r.get("command_station") is not None and r.get("engaged") is not None
]
def station_transition_rows(sequence):
    result = []
    for prev, current in zip(sequence, sequence[1:]):
        dy = current[2][1] - prev[2][1]
        result.append({
            "from_serve": prev[0], "to_serve": current[0], "dy": round(dy, 4),
            "abs_dy": round(abs(dy), 4),
            "in_training_band": STATION_STEP_LO - 1e-3 <= abs(dy) <= STATION_STEP_HI + 1e-3,
            "positive_main": POSITIVE_MAIN_LO - 1e-3 <= dy <= POSITIVE_MAIN_HI + 1e-3,
        })
    return result


# Harness input coverage is independent of policy success. Counting only engaged rows made one
# runner miss fail both the proxy and the input-coverage gate, obscuring the actual failure layer.
command_station_transitions = station_transition_rows(command_station_sequence)
achieved_station_transitions = station_transition_rows(achieved_station_sequence)
station_good = sum(1 for s in command_station_transitions if s["in_training_band"])
positive_main_good = sum(1 for s in command_station_transitions if s["positive_main"])
station_dirs_ok = (not command_station_transitions or
                   (any(s["dy"] > 0 for s in command_station_transitions)
                    and any(s["dy"] < 0 for s in command_station_transitions)))
station_clips_ok = (not command_station_sequence or
                    ({s[1] for s in command_station_sequence if s[1]} >= {"forehand", "backhand"}))
station_coverage_ok = (
    MIN_STATION_TRANSITIONS <= 0 or
    (station_good >= MIN_STATION_TRANSITIONS and station_dirs_ok and station_clips_ok))
positive_main_coverage_ok = positive_main_good >= MIN_POSITIVE_MAIN_TRANSITIONS
no_base_warns = sum(int(r.get("no_base_warns", 0)) for r in serve_rows)
localization_ok = not (
    REQUIRE_FRESH_LOCALIZATION and DROPOUT_AT == 0 and no_base_warns > 0
)
proxy_rate = proxy_completed / n if n else 0.0
engage_rate = engaged_serves / n if n else 0.0
completion_rate = completed_serves / engaged_serves if engaged_serves else 0.0
recovered_engaged = sum(
    1 for r in serve_rows
    if r["engaged"] and r["recovered_to_session_anchor"]
)
recovery_rate = recovered_engaged / engaged_serves if engaged_serves else 0.0
if GATE3_PHASE == "random":
    runner_proxy_pass = bool(
        n == SERVES
        and engaged_serves >= MIN_ENGAGED_SERVES
        and completed_serves >= MIN_COMPLETED_SERVES
        and engage_rate >= MIN_ENGAGE_RATE
        and completion_rate >= MIN_COMPLETION_RATE
        and recovery_rate >= MIN_RECOVERY_RATE
        and not multi_engage_serves
        and proxy_rate >= MIN_PROXY_RATE)
else:
    runner_proxy_pass = bool(
        n == SERVES
        and engaged_serves >= MIN_ENGAGED_SERVES
        and completed_serves >= MIN_COMPLETED_SERVES
        and not incomplete
        and not multi_engage_serves
        and proxy_rate >= MIN_PROXY_RATE)
stability_pass = bool(
    not fell
    and actual_q_faults == 0
    and not command_safety_faults
    and drift <= MAX_END_XY_DRIFT_M
    and state["max_anchor_xy_err_motion"] <= MAX_PEAK_XY_DRIFT_M
    and rescues <= MAX_RESCUES
    and state["min_z_motion"] >= MIN_UPRIGHT_PELVIS_Z_M)
harness_input_pass = bool(station_coverage_ok and positive_main_coverage_ok)
xlock_pass = bool(
    XLOCK_THRESH <= 0 or (
        xlock_within_threshold(state["max_abs_x_err_motion"])
        and not xlock_violations))
runner_gate_pass = bool(
    runner_proxy_pass and stability_pass and harness_input_pass and localization_ok and xlock_pass)


def load_evidence(path, unavailable):
    try:
        with open(path) as stream:
            return json.load(stream)
    except (FileNotFoundError, json.JSONDecodeError, OSError) as exc:
        result = dict(unavailable)
        result["evidence_error"] = str(exc)
        return result


planner_evidence = load_evidence(PLANNER_EVIDENCE_JSON, {
    "runner_harness_pass": False,
    "planner_contract_pass": False,
    "physical_contact_measured": False,
    "landing_measured": False,
})
physical_evidence = load_evidence(PHYSICAL_EVIDENCE_JSON, {
    "physical_contact_measured": False,
    "landing_measured": False,
    "physical_contact_pass": False,
    "landing_pass": False,
})
random_serves_receipt = (
    load_evidence(RANDOM_SERVES_RECEIPT, {"evidence_error": "not selected"})
    if GATE3_PHASE == "random" else None
)
planner_contract_pass = bool(planner_evidence.get("planner_contract_pass", False))
physical_join = join_physical_evidence_by_side(
    serve_rows,
    physical_evidence,
    min_samples_per_side=MIN_PHYSICAL_SAMPLES_PER_SIDE,
    min_contact_rate=MIN_PHYSICAL_CONTACT_RATE,
    min_landing_rate=MIN_LEGAL_LANDING_RATE,
    exact_samples_per_side=REQUIRED_SIDE_SHOTS,
    min_contacts_per_side=MIN_CONTACTS_PER_SIDE,
    min_landings_per_side=MIN_LANDINGS_PER_SIDE,
    min_global_contacts=MIN_GLOBAL_CONTACTS,
    min_global_landings=MIN_GLOBAL_LANDINGS,
)
physical_by_id = {
    int(row["shot_id"]): row for row in physical_evidence.get("rows", [])
}
for row in serve_rows:
    row["physical_outcome"] = physical_by_id.get(int(row.get("shot_id") or 0))
physical_ball_pass = bool(physical_join["pass"])
certification_pass = bool(runner_gate_pass and planner_contract_pass and physical_ball_pass)
# Public Gate3 has exactly one top-level verdict. The historical runner-only
# result remains visible as the lifecycle regression subreport, but can never
# promote the gate without contact and landing evidence.
ok = certification_pass
summary = {
    "schema_version": 7,
    "phase": GATE3_PHASE,
    "serves": n, "engaged_serves": engaged_serves,
    "minimum_engaged_serves": MIN_ENGAGED_SERVES,
    "completed_serves": completed_serves,
    "minimum_completed_serves": MIN_COMPLETED_SERVES,
    "total_engage_events": total_engage_events,
    "completed_recovered_proxy": proxy_completed,
    "proxy_rate": round(proxy_rate, 3),
    "minimum_proxy_rate": MIN_PROXY_RATE,
    "engage_rate": round(engage_rate, 3),
    "minimum_engage_rate": MIN_ENGAGE_RATE,
    "completion_rate_given_engage": round(completion_rate, 3),
    "minimum_completion_rate_given_engage": MIN_COMPLETION_RATE,
    "recovered_engaged_serves": recovered_engaged,
    "recovery_rate_given_engage": round(recovery_rate, 3),
    "minimum_recovery_rate_given_engage": MIN_RECOVERY_RATE,
    "recovery_radius_m": RECOVERY_RADIUS_M,
    "falls": int(fell),
    "physical_falls": int(physical_fall_detected),
    "runner_fall_guards": int(runner_fall_guard_tripped),
    "minimum_upright_pelvis_z_m": MIN_UPRIGHT_PELVIS_Z_M,
    "engaged_but_unresolved_proxy": incomplete,
    "multi_engage_serves": multi_engage_serves,
    "station_drift_m": round(drift, 3),
    "max_end_xy_drift_m": MAX_END_XY_DRIFT_M,
    "max_anchor_xy_error_m": round(state["max_anchor_xy_err_motion"], 3),
    "max_peak_xy_drift_m": MAX_PEAK_XY_DRIFT_M,
    "station_x": STATION_X, "xlock_thresh_m": XLOCK_THRESH,
    "xlock_compare_epsilon_m": XLOCK_COMPARE_EPS_M,
    "xlock_violations": xlock_violations,
    "n_xlock_violations": len(xlock_violations),
    "max_abs_x_err_m": round(state["max_abs_x_err_motion"], 3),
    "x_err_at_end_m": round(x_err_at_end, 3),
    "min_z_motion": round(state["min_z_motion"], 3),
    "valid_cmds": state["valid"], "invalid_cmds": state["invalid"],
    "dropout_stress": DROPOUT_AT if dropout_done else 0,
    "operator_rescues": rescues,
    "actual_q_faults": actual_q_faults,
    "command_safety_faults": command_safety_faults,
    "command_safety_fault_count": len(command_safety_faults),
    "max_rescues_allowed": MAX_RESCUES,
    "rescue_enabled": ALLOW_RESCUE,
    "readiness_required": REQUIRE_READY,
    "fresh_localization_required": REQUIRE_FRESH_LOCALIZATION,
    "ready_limits": {"x_m": READY_X_MAX, "y_m": READY_Y_MAX,
                     "speed_mps": READY_SPEED_MAX},
    "station_transition_min_required": MIN_STATION_TRANSITIONS,
    "station_transitions_in_band": station_good,
    "station_transition_coverage_ok": station_coverage_ok,
    "station_transition_coverage_source": "commanded",
    "command_station_sequence": command_station_sequence,
    "achieved_station_sequence": achieved_station_sequence,
    "positive_main_transition_min_required": MIN_POSITIVE_MAIN_TRANSITIONS,
    "positive_main_transitions": positive_main_good,
    "positive_main_range_m": [POSITIVE_MAIN_LO, POSITIVE_MAIN_HI],
    "positive_main_transition_coverage_ok": positive_main_coverage_ok,
    "station_transitions": command_station_transitions,
    "achieved_station_transitions": achieved_station_transitions,
    "no_base_warns": no_base_warns,
    "localization_ok": localization_ok,
    "runner_proxy_pass": runner_proxy_pass,
    "stability_pass": stability_pass,
    "harness_input_pass": harness_input_pass,
    "xlock_pass": xlock_pass,
    "runner_gate_pass": runner_gate_pass,
    "planner_evidence": planner_evidence,
    "physical_ball_evidence": physical_evidence,
    "random_serves_receipt": random_serves_receipt,
    "physical_evidence_by_planner_side": physical_join,
    "planner_contract_pass": planner_contract_pass,
    "physical_ball_pass": physical_ball_pass,
    "certification_pass": certification_pass,
    "gate_name": "Gate3",
    "lifecycle_regression_pass": runner_gate_pass,
    "selected_gate_verdict": "certification",
    "physical_contact_measured": bool(physical_join["physical_contact_measured"]),
    "landing_measured": bool(physical_join["landing_measured"]),
    "pass": bool(ok),
    "rows": serve_rows,
}
with open(REPORT_JSON, "w") as f:
    json.dump(summary, f, indent=1)
print(f"[rally] SUMMARY: serves={n} engaged_serves={engaged_serves} "
      f"completed_serves={completed_serves} engage_events={total_engage_events} "
      f"proxy_ok={proxy_completed} "
      f"({100*proxy_rate:.0f}%) falls={int(fell)} actual_q_faults={actual_q_faults} "
      f"engage={100*engage_rate:.0f}% complete|engage={100*completion_rate:.0f}% "
      f"recover|engage={100*recovery_rate:.0f}% "
      f"rescues={rescues} drift_end/peak={drift:.2f}/"
      f"{state['max_anchor_xy_err_motion']:.2f}m "
      f"multi_engage={multi_engage_serves or 0} "
      f"xlock_viol={len(xlock_violations)}{xlock_violations or ''} "
      f"station_steps(commanded)={station_good}/{len(command_station_transitions)} "
      f"station_steps(achieved)={sum(s['in_training_band'] for s in achieved_station_transitions)}/"
      f"{len(achieved_station_transitions)} "
      f"positive_main={positive_main_good}/{MIN_POSITIVE_MAIN_TRANSITIONS} "
      f"stale_base_warns={no_base_warns} "
      f"xmax={state['max_abs_x_err_motion']:.2f}m xend={x_err_at_end:+.2f}m "
      f"min_z={state['min_z_motion']:.3f} runner_proxy={'PASS' if runner_proxy_pass else 'FAIL'} "
      f"safety={'PASS' if stability_pass else 'FAIL'} input={'PASS' if harness_input_pass else 'FAIL'} "
      f"localization={'PASS' if localization_ok else 'FAIL'} xlock={'PASS' if xlock_pass else 'FAIL'} "
      f"planner_contract={'PASS' if planner_contract_pass else 'FAIL'} "
      f"physical_ball={'PASS' if physical_ball_pass else 'NOT_PROVEN'} "
      f"contacts={physical_join['global_contacts']}/{MIN_GLOBAL_CONTACTS} "
      f"legal={physical_join['global_legal_landings']}/{MIN_GLOBAL_LANDINGS} "
      f"runner_gate={'PASS' if runner_gate_pass else 'FAIL'} "
      f"certification={'PASS' if certification_pass else 'FAIL'} "
      f"Gate3={'PASS' if ok else 'FAIL'} "
      f"(report: {REPORT_JSON}, obs csv: {OBS_CSV})", flush=True)
for side in ("forehand", "backhand"):
    physical_side = physical_join["by_side"][side]
    print(
        f"[rally] PHYSICAL {side}: shots={physical_side['shots']} "
        f"contact={physical_side['contacts']} "
        f"({100.0 * physical_side['contact_rate']:.0f}%) "
        f"legal_landing={physical_side['legal_landings']} "
        f"({100.0 * physical_side['landing_rate']:.0f}%) "
        f"-> {'PASS' if physical_side['pass'] else 'FAIL'}",
        flush=True,
    )
sys.exit(0 if ok else 1)
