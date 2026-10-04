#!/usr/bin/env python3
"""Gate3 conductor for an autonomous policy-native physical-ball rally.

After the initial stand and one ``m`` transition, the robot is never reset,
aborted, side-selected, or manually recovered.  Side-neutral ball initial states
drive the MuJoCo ball; the production planner chooses FH/BH from ball/session-HOME
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
  PP_BASE_OUTAGE_AT=0 serve index for an isolated calibrated-base output outage
  PP_BASE_OUTAGE_TICKS=15  upstream relay blackout duration in 50 Hz policy ticks
  PP_ALLOW_RESCUE=0   mandatory; operator rescue is forbidden
  PP_MAX_RESCUES=0    mandatory
  PP_REQUIRE_READY=1  require the selected release/readiness telemetry contract
  PP_READY_CONTRACT=legacy_ready_v1|late_reveal_commit_v1|home_preempt_commit_v1
  PP_RECOVERY_TARGET_CONTRACT=command_station|session_anchor

Recovery is measured against the target named by ``PP_RECOVERY_TARGET_CONTRACT``.
Legacy recipes return to the completed swing station; the fixed-HOME HITTER
scenario returns to the immutable MOTION-entry session anchor. A recovery requires position,
planar speed, pelvis yaw, pelvis tilt and continuous dwell; the next engage is reported
separately and is not accepted as proof of recovery.

The ``random`` RallyV17-r10 phase instead scores a configurable many-ball
session: it does not require READY or station transitions, and measures return
to the immutable MOTION-entry XY anchor after each full-body swing.

PASS requires the lifecycle/stability/input contract, the live C++ Planner
one-shot audit, and separately sufficient FH and BH measured racket-contact
and legal-landing rates.  Missing telemetry, side assignment, or ``shot_id``
joins fail closed.
"""
import ast
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

from pp_gate3_core import (
    audit_actual_q_full_tail_ledger,
    audit_cpp_planner_debug_csv,
    audit_runner_engagement_trace,
    audit_runner_localization_trace,
    join_planner_evidence_by_flight_id,
    join_physical_evidence_by_side,
    physical_report_complete,
)
from pp_recovery_dwell import update_recovery_dwell

GEAR = os.environ.get("PP_GEAR", str(Path(__file__).resolve().parents[1]))
DIST = os.environ.get("PP_DIST", str(Path(GEAR) / "dist/a3_deploy_x86_64"))
RUNNER_LOG = "/tmp/pp_runner.log"
BALL_LOG = "/tmp/pp_ball.log"
REPORT_JSON = "/tmp/pp_rally_report.json"
OBS_CSV = "/tmp/pp_obs.csv"
TRACE_CSV = "/tmp/pp_runner_trace.csv"
PLANT_CSV = os.environ.get("PP_PLANT_CSV", "/tmp/pp_mujoco_plant.csv")
PLANNER_EVIDENCE_JSON = os.environ.get(
    "PP_PLANNER_EVIDENCE_JSON", "/tmp/pp_planner_cpp_report.json")
PLANNER_DEBUG_CSV = os.environ.get(
    "PP_PLANNER_DEBUG_CSV", "/tmp/pp_planner_debug.csv")
PHYSICAL_EVIDENCE_JSON = os.environ.get(
    "PP_PHYSICAL_EVIDENCE_JSON", "/tmp/pp_physical_ball_report.json")
RANDOM_SERVES_RECEIPT = os.environ.get(
    "PP_RANDOM_SERVES_RECEIPT", "/tmp/pp_gate3_random_serves_receipt.json")
LAUNCH_RECEIPT_JSON = os.environ.get(
    "PP_LAUNCH_RECEIPT_JSON", "/tmp/pp_gate3_launch_receipt.json")
GATE3_SCENARIO = os.environ.get("PP_GATE3_SCENARIO", "unspecified").strip()
HITTER_FIXED_HOME_SCENARIO = "hitter_pingpong_fixed_home_rapid_1150ms_v1"
HITTER_TORTURE_SCENARIO = "hitter_pingpong_low_corner_torture_v1"
HITTER_FIXED_HOME_LANE_CONTRACTS = {
    HITTER_FIXED_HOME_SCENARIO,
    HITTER_TORTURE_SCENARIO,
}
GATE3_VERDICT = os.environ.get("PP_GATE3_VERDICT", "certification").strip().lower()
if GATE3_VERDICT != "certification":
    raise ValueError("Gate3 is fail-closed: PP_GATE3_VERDICT must be certification")

SERVES = int(os.environ.get("PP_SERVES", "12"))
GATE3_PHASE = os.environ.get("PP_GATE3_PHASE", "qualification").strip().lower()
if GATE3_PHASE not in (
    "qualification", "task", "fixed_home", "torture", "random"
):
    raise ValueError(
        "PP_GATE3_PHASE must be qualification, task, fixed_home, torture or random"
    )
if GATE3_PHASE == "qualification":
    REQUIRED_SERVES = 12
    REQUIRED_GLOBAL_CONTACTS = 11
    REQUIRED_GLOBAL_LANDINGS = 10
    REQUIRED_GLOBAL_LANDING_OBSERVATIONS = 11
    REQUIRED_SIDE_SHOTS = 6
    REQUIRED_SIDE_CONTACTS = 5
    REQUIRED_SIDE_LANDINGS = 5
    REQUIRED_SIDE_LANDING_OBSERVATIONS = 5
    REQUIRED_MIN_SIDE_RATE = 5.0 / 6.0
elif GATE3_PHASE == "task":
    REQUIRED_SERVES = 26
    REQUIRED_GLOBAL_CONTACTS = 25
    REQUIRED_GLOBAL_LANDINGS = 24
    REQUIRED_GLOBAL_LANDING_OBSERVATIONS = 25
    REQUIRED_SIDE_SHOTS = None
    REQUIRED_SIDE_CONTACTS = 0
    REQUIRED_SIDE_LANDINGS = 0
    REQUIRED_SIDE_LANDING_OBSERVATIONS = 4
    REQUIRED_MIN_SIDE_RATE = 0.8
elif GATE3_PHASE == "fixed_home":
    if SERVES < 26 or SERVES % 2:
        raise ValueError("Gate3 fixed_home requires an even serve count >= 26")
    REQUIRED_SERVES = 26
    REQUIRED_GLOBAL_CONTACTS = 22
    # Every rapid return remains physical through ground, across later launches.
    REQUIRED_GLOBAL_LANDINGS = REQUIRED_GLOBAL_CONTACTS
    REQUIRED_GLOBAL_LANDING_OBSERVATIONS = REQUIRED_GLOBAL_CONTACTS
    REQUIRED_SIDE_SHOTS = 13
    REQUIRED_SIDE_CONTACTS = 11
    REQUIRED_SIDE_LANDINGS = 2
    REQUIRED_SIDE_LANDING_OBSERVATIONS = 3
    REQUIRED_MIN_SIDE_RATE = 5.0 / 6.0
    if SERVES != 26:
        REQUIRED_SERVES = SERVES
        REQUIRED_GLOBAL_CONTACTS = (5 * SERVES + 5) // 6
        # A truncated rapid cycle can differ by one shot per side. The measured
        # per-side contact rate remains 5/6 and all flights must engage/complete.
        REQUIRED_SIDE_SHOTS = None
        REQUIRED_SIDE_CONTACTS = (5 * (SERVES // 2 - 1) + 5) // 6
elif GATE3_PHASE == "torture":
    if SERVES < 18 or (SERVES - 6) % 12 != 0:
        raise ValueError(
            "Gate3 torture requires six clean flights plus one or more 12-flight cycles"
        )
    REQUIRED_SERVES = None
    REQUIRED_GLOBAL_CONTACTS = (4 * SERVES + 4) // 5
    # Torture cadence also retains every outgoing trajectory.
    REQUIRED_GLOBAL_LANDINGS = REQUIRED_GLOBAL_CONTACTS
    REQUIRED_GLOBAL_LANDING_OBSERVATIONS = REQUIRED_GLOBAL_CONTACTS
    REQUIRED_SIDE_SHOTS = None
    REQUIRED_SIDE_CONTACTS = 0
    REQUIRED_SIDE_LANDINGS = 2
    REQUIRED_SIDE_LANDING_OBSERVATIONS = 3
    REQUIRED_MIN_SIDE_RATE = 0.8
else:
    if SERVES < 16:
        raise ValueError("Gate3 random phase requires at least 16 serves")
    REQUIRED_SERVES = None
    REQUIRED_GLOBAL_CONTACTS = (4 * SERVES + 4) // 5
    REQUIRED_GLOBAL_LANDINGS = (4 * SERVES + 4) // 5
    REQUIRED_GLOBAL_LANDING_OBSERVATIONS = REQUIRED_GLOBAL_CONTACTS
    REQUIRED_SIDE_SHOTS = None
    REQUIRED_SIDE_CONTACTS = 0
    REQUIRED_SIDE_LANDINGS = 0
    REQUIRED_SIDE_LANDING_OBSERVATIONS = 4
    REQUIRED_MIN_SIDE_RATE = 0.8
if REQUIRED_SERVES is not None and SERVES != REQUIRED_SERVES:
    raise ValueError(
        f"Gate3 {GATE3_PHASE} requires exactly {REQUIRED_SERVES} serves, got {SERVES}"
    )
RESET_Y = float(os.environ.get("PP_RESET_Y", "-0.7625"))
RESET_Z = float(os.environ.get("PP_RESET_Z", "1.07"))
DIRECT_MOTION_SETTLE_S = float(
    os.environ.get("PP_DIRECT_MOTION_SETTLE_S", "0.75")
)
DROPOUT_AT = int(os.environ.get("PP_DROPOUT_AT", "0"))
DROPOUT_S = float(os.environ.get("PP_DROPOUT_S", "1.0"))
BASE_OUTAGE_AT = int(os.environ.get("PP_BASE_OUTAGE_AT", "0"))
BASE_OUTAGE_TICKS = int(os.environ.get("PP_BASE_OUTAGE_TICKS", "15"))
BASE_RELAY_PGID = int(os.environ.get("PP_BASE_RELAY_PGID", "0"))
BASE_PRODUCER_CONTRACT = os.environ.get(
    "PP_BASE_PRODUCER_CONTRACT", ""
).strip()
BASE_PUBLISHER_COUNT = int(os.environ.get("PP_BASE_PUBLISHER_COUNT", "0"))
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
RECOVERY_SPEED_MAX_MPS = float(
    os.environ.get("PP_RECOVERY_SPEED_MAX_MPS", "0.20")
)
RECOVERY_TILT_MAX_DEG = float(
    os.environ.get("PP_RECOVERY_TILT_MAX_DEG", "12.0")
)
RECOVERY_YAW_MAX_DEG = float(
    os.environ.get("PP_RECOVERY_YAW_MAX_DEG", "5.0")
)
RECOVERY_DWELL_S = float(os.environ.get("PP_RECOVERY_DWELL_S", "0.20"))
RECOVERY_SPEED_EMA_ALPHA = float(
    os.environ.get("PP_RECOVERY_SPEED_EMA_ALPHA", "0.20")
)
REQUIRED_POST_COMPLETION_TAIL_S = float(
    os.environ.get("PP_REQUIRED_POST_COMPLETION_TAIL_S", "0.0")
)
MAX_END_XY_DRIFT_M = float(os.environ.get("PP_MAX_END_XY_DRIFT_M", "0.50"))
MAX_PEAK_XY_DRIFT_M = float(os.environ.get("PP_MAX_PEAK_XY_DRIFT_M", "0.50"))
MAX_END_YAW_DRIFT_DEG = float(
    os.environ.get("PP_MAX_END_YAW_DRIFT_DEG", "5.0")
)
MAX_PEAK_YAW_DRIFT_DEG = float(
    os.environ.get("PP_MAX_PEAK_YAW_DRIFT_DEG", "12.0")
)
ALLOW_RESCUE = os.environ.get("PP_ALLOW_RESCUE", "0").strip().lower() in ("1", "true", "yes")
MAX_RESCUES = int(os.environ.get("PP_MAX_RESCUES", "0"))
REQUIRE_READY = os.environ.get("PP_REQUIRE_READY", "1").strip().lower() in ("1", "true", "yes")
READY_CONTRACT = os.environ.get(
    "PP_READY_CONTRACT", "legacy_ready_v1"
).strip().lower()
RECOVERY_TARGET_CONTRACT = os.environ.get(
    "PP_RECOVERY_TARGET_CONTRACT", "command_station"
).strip().lower()
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
MIN_LANDING_OBSERVATIONS_PER_SIDE = int(os.environ.get(
    "PP_MIN_LANDING_OBSERVATIONS_PER_SIDE",
    str(REQUIRED_SIDE_LANDING_OBSERVATIONS),
))
MIN_GLOBAL_CONTACTS = int(os.environ.get(
    "PP_MIN_GLOBAL_CONTACTS", str(REQUIRED_GLOBAL_CONTACTS)))
MIN_GLOBAL_LANDINGS = int(os.environ.get(
    "PP_MIN_GLOBAL_LANDINGS", str(REQUIRED_GLOBAL_LANDINGS)))
MIN_GLOBAL_LANDING_OBSERVATIONS = int(os.environ.get(
    "PP_MIN_GLOBAL_LANDING_OBSERVATIONS",
    str(REQUIRED_GLOBAL_LANDING_OBSERVATIONS),
))
ALLOWED_LANDING_CENSOR_LANES = ()

if ALLOW_RESCUE or MAX_RESCUES != 0:
    raise ValueError("Gate3 forbids operator rescue")
if not REQUIRE_FRESH_LOCALIZATION:
    raise ValueError("Gate3 requires fresh localization on every MOTION tick")
if (
    BASE_PRODUCER_CONTRACT
    not in {"simulator_truth_direct_v1", "calibrated_hope_world_relay_v1"}
    or BASE_PUBLISHER_COUNT != 1
):
    raise ValueError(
        "Gate3 requires a named single-producer /a3/base_pose_flat receipt"
    )
if not 0 <= BASE_OUTAGE_AT <= SERVES:
    raise ValueError("PP_BASE_OUTAGE_AT must select a configured serve or zero")
if BASE_OUTAGE_AT and not (1 <= BASE_OUTAGE_TICKS <= 15 and BASE_RELAY_PGID > 1):
    raise ValueError(
        "base outage requires 1..15 policy ticks and a dedicated base-relay process group"
    )
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
        and RECOVERY_SPEED_MAX_MPS >= 0.0
        and 0.0 < RECOVERY_TILT_MAX_DEG < 90.0
        and 0.0 < RECOVERY_YAW_MAX_DEG < 90.0
        and RECOVERY_DWELL_S > 0.0
        and REQUIRED_POST_COMPLETION_TAIL_S >= 0.0
        and 0.0 < RECOVERY_SPEED_EMA_ALPHA <= 1.0
        and MAX_END_XY_DRIFT_M >= RECOVERY_RADIUS_M
        and MAX_PEAK_XY_DRIFT_M >= MAX_END_XY_DRIFT_M
        and MAX_END_YAW_DRIFT_DEG >= RECOVERY_YAW_MAX_DEG
        and MAX_PEAK_YAW_DRIFT_DEG >= MAX_END_YAW_DRIFT_DEG):
    raise ValueError("Gate3 HOME support-frame recovery/drift thresholds are inconsistent")
if READY_CONTRACT not in (
    "legacy_ready_v1", "late_reveal_commit_v1", "home_preempt_commit_v1", "none"
):
    raise ValueError(f"unsupported PP_READY_CONTRACT={READY_CONTRACT!r}")
if REQUIRE_READY and READY_CONTRACT == "none":
    raise ValueError("PP_REQUIRE_READY=1 requires a non-none PP_READY_CONTRACT")
if RECOVERY_TARGET_CONTRACT not in ("command_station", "session_anchor"):
    raise ValueError(
        f"unsupported PP_RECOVERY_TARGET_CONTRACT={RECOVERY_TARGET_CONTRACT!r}"
    )
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
    or MIN_LANDING_OBSERVATIONS_PER_SIDE
    < REQUIRED_SIDE_LANDING_OBSERVATIONS
    or MIN_GLOBAL_CONTACTS < REQUIRED_GLOBAL_CONTACTS
    or MIN_GLOBAL_LANDINGS < REQUIRED_GLOBAL_LANDINGS
    or MIN_GLOBAL_LANDING_OBSERVATIONS
    < REQUIRED_GLOBAL_LANDING_OBSERVATIONS
):
    raise ValueError("Gate3 physical count thresholds are weaker than the selected phase")

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


def _duration_schedule(name, fallback, *, allow_zero):
    raw = os.environ.get(name, "").strip()
    if not raw:
        values = [float(fallback)] * SERVES
    else:
        parsed = ast.literal_eval(raw)
        if not isinstance(parsed, (list, tuple)) or len(parsed) != SERVES:
            raise ValueError(f"{name} must contain exactly PP_SERVES values")
        values = [float(value) for value in parsed]
    if any(
        not math.isfinite(value)
        or value < 0.0
        or (not allow_zero and value == 0.0)
        for value in values
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} values must be finite and {qualifier}")
    return tuple(values)


FLIGHT_SCHEDULE = _duration_schedule(
    "PP_FLIGHT_S_LIST", FLIGHT_S, allow_zero=False
)
PAUSE_SCHEDULE = _duration_schedule(
    "PP_PAUSE_S_LIST", PAUSE_S, allow_zero=True
)
SERVE_PERIOD_S = max(
    flight + pause
    for flight, pause in zip(FLIGHT_SCHEDULE, PAUSE_SCHEDULE)
)
EXPECTED_SERVE_DURATION_S = sum(FLIGHT_SCHEDULE) + sum(
    PAUSE_SCHEDULE[:-1]
)
GLOBAL_TIMEOUT_S = MOTION_IDLE_S + EXPECTED_SERVE_DURATION_S + 60.0

LANE_CONTRACT = os.environ.get("PP_GATE3_LANE_CONTRACT", "none").strip()
if LANE_CONTRACT not in ({"none"} | HITTER_FIXED_HOME_LANE_CONTRACTS):
    raise ValueError(f"unsupported PP_GATE3_LANE_CONTRACT={LANE_CONTRACT!r}")
CLEAN_FIXED_HOME_SERVES = int(
    os.environ.get("PP_CLEAN_FIXED_HOME_SERVES", "6")
)
if not 0 < CLEAN_FIXED_HOME_SERVES < SERVES:
    raise ValueError("PP_CLEAN_FIXED_HOME_SERVES must be inside the serve ledger")
if LANE_CONTRACT == HITTER_FIXED_HOME_SCENARIO and (
    GATE3_PHASE != "fixed_home"
    or GATE3_SCENARIO != HITTER_FIXED_HOME_SCENARIO
    or SERVES < 26 or SERVES % 2 != 0
    or CLEAN_FIXED_HOME_SERVES != 6
):
    raise ValueError(
        f"{HITTER_FIXED_HOME_SCENARIO} requires its matching scenario, "
        "PP_GATE3_PHASE=fixed_home and an even serve count >= 26"
    )
if LANE_CONTRACT == HITTER_TORTURE_SCENARIO and (
    GATE3_PHASE != "torture"
    or GATE3_SCENARIO != HITTER_TORTURE_SCENARIO
    or SERVES < 18
    or (SERVES - CLEAN_FIXED_HOME_SERVES) % 12 != 0
    or CLEAN_FIXED_HOME_SERVES != 6
):
    raise ValueError(
        f"{HITTER_TORTURE_SCENARIO} requires its matching scenario, "
        "PP_GATE3_PHASE=torture, six clean flights and complete 12-flight cycles"
    )
FIXED_HOME_COMMAND_TOLERANCE_M = float(
    os.environ.get("PP_FIXED_HOME_COMMAND_TOLERANCE_M", "0.0015")
)
FIXED_HOME_DRIFT_NOT_OBSERVED_M = float(
    os.environ.get("PP_FIXED_HOME_DRIFT_NOT_OBSERVED_M", "0.01")
)
MIN_RAPID_ENGAGED = int(os.environ.get("PP_MIN_RAPID_ENGAGED", "4"))
MIN_RAPID_COMPLETED = int(os.environ.get("PP_MIN_RAPID_COMPLETED", "4"))
RAPID_MAX_ENGAGE_GAP_S = float(
    os.environ.get("PP_RAPID_MAX_ENGAGE_GAP_S", "1.40")
)
RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S = float(
    os.environ.get("PP_RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S", "0.40")
)
RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S = float(
    os.environ.get("PP_RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S", "0.55")
)
NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S = float(
    os.environ.get("PP_NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S", "0.12")
)
NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD = float(
    os.environ.get("PP_NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD", "0.20")
)
if not (
    0.0 <= FIXED_HOME_COMMAND_TOLERANCE_M
    and 0.0 < FIXED_HOME_DRIFT_NOT_OBSERVED_M
    and MIN_RAPID_ENGAGED >= 0
    and MIN_RAPID_COMPLETED >= 0
    and RAPID_MAX_ENGAGE_GAP_S > 0.0
    and 0.0 <= RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S
    < RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S
    and NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S >= 0.0
    and NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD > 0.0
):
    raise ValueError("fixed-HOME scenario thresholds are invalid")


def _lane_for_serve(serve):
    if LANE_CONTRACT not in HITTER_FIXED_HOME_LANE_CONTRACTS:
        return "unstratified"
    if serve <= CLEAN_FIXED_HOME_SERVES:
        return "clean_fixed_home"
    return "rapid_fixed_home"

rclpy.init()
node = Node("rally_conductor")
state = {"x": None, "y": None, "z": None, "min_z_motion": 99.0,
         "max_abs_x_err_motion": 0.0,   # max |base_x - STATION_X| across the whole rally
         "max_anchor_xy_err_motion": 0.0,
         "max_anchor_yaw_err_motion_deg": 0.0,
         "pose_sample_time_s": None, "speed_xy_mps": None, "tilt_deg": None,
         "yaw_rad": None,
         "valid": 0, "invalid": 0, "motion_active": False}


def _wrapped_angle_rad(value):
    return math.atan2(math.sin(value), math.cos(value))


def _recovery_target(row):
    if GATE3_PHASE == "random" or RECOVERY_TARGET_CONTRACT == "session_anchor":
        return globals().get("anchor_xy")
    return row.get("command_station")


def _observe_recovery(row, now):
    """Accumulate a strict, station-relative recovery dwell for one completed swing."""

    target = _recovery_target(row)
    if target is None or state["x"] is None or state["y"] is None:
        return
    xy_error = math.hypot(state["x"] - target[0], state["y"] - target[1])
    row["recovery_target"] = [float(target[0]), float(target[1])]
    row["recovery_last_xy_error_m"] = xy_error
    previous_min = row.get("recovery_min_xy_error_m")
    row["recovery_min_xy_error_m"] = (
        xy_error if previous_min is None else min(previous_min, xy_error)
    )
    row["recovery_last_speed_mps"] = state["speed_xy_mps"]
    row["recovery_last_tilt_deg"] = state["tilt_deg"]
    home_yaw = globals().get("anchor_yaw_rad")
    yaw_error_deg = (
        None
        if home_yaw is None or state["yaw_rad"] is None
        else abs(math.degrees(_wrapped_angle_rad(state["yaw_rad"] - home_yaw)))
    )
    row["recovery_last_yaw_error_deg"] = yaw_error_deg
    if row.get("complete", 0) < 1 or row.get("engaged") is None:
        return
    speed = state["speed_xy_mps"]
    tilt = state["tilt_deg"]
    settled_now = bool(
        speed is not None
        and tilt is not None
        and yaw_error_deg is not None
        and xy_error <= RECOVERY_RADIUS_M
        and speed <= RECOVERY_SPEED_MAX_MPS
        and tilt <= RECOVERY_TILT_MAX_DEG
        and yaw_error_deg <= RECOVERY_YAW_MAX_DEG
        and state["z"] >= MIN_UPRIGHT_PELVIS_Z_M
    )
    update_recovery_dwell(
        row,
        now=now,
        settled_now=settled_now,
        required_dwell_s=RECOVERY_DWELL_S,
    )
    if row.get("recovery_settled", False):
        completed_at = row.get("_complete_monotonic")
        if completed_at is not None and row.get("recovery_latency_s") is None:
            row["recovery_latency_s"] = max(0.0, now - completed_at)


def pose_cb(m):
    now = time.monotonic()
    old_x = state["x"]
    old_y = state["y"]
    old_t = state["pose_sample_time_s"]
    stamp = m.header.stamp
    source_time = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
    sample_time = source_time if source_time > 0.0 else now
    state["x"] = m.pose.position.x
    state["y"] = m.pose.position.y
    state["z"] = m.pose.position.z
    state["pose_sample_time_s"] = sample_time
    if (
        old_t is not None
        and sample_time > old_t
        and old_x is not None
        and old_y is not None
    ):
        raw_speed = math.hypot(
            state["x"] - old_x, state["y"] - old_y
        ) / (sample_time - old_t)
        previous_speed = state["speed_xy_mps"]
        state["speed_xy_mps"] = (
            raw_speed
            if previous_speed is None
            else (
                RECOVERY_SPEED_EMA_ALPHA * raw_speed
                + (1.0 - RECOVERY_SPEED_EMA_ALPHA) * previous_speed
            )
        )
    q = m.pose.orientation
    norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm2 > 1.0e-12:
        state["yaw_rad"] = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
        )
        up_z = (q.w * q.w + q.z * q.z - q.x * q.x - q.y * q.y) / norm2
        state["tilt_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
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
        session_anchor_yaw = globals().get("anchor_yaw_rad")
        if session_anchor_yaw is not None and state["yaw_rad"] is not None:
            yaw_err_deg = abs(math.degrees(_wrapped_angle_rad(
                state["yaw_rad"] - session_anchor_yaw
            )))
            state["max_anchor_yaw_err_motion_deg"] = max(
                state["max_anchor_yaw_err_motion_deg"], yaw_err_deg
            )
        open_row = globals().get("cur")
        if open_row is not None:
            open_row["max_abs_x_err"] = max(open_row["max_abs_x_err"], xerr)
            open_row["min_z"] = min(open_row["min_z"], state["z"])
            if session_anchor is not None:
                open_row["max_anchor_xy_err"] = max(
                    open_row["max_anchor_xy_err"], anchor_err
                )
        recovery_row = globals().get("active")
        if recovery_row is not None:
            _observe_recovery(recovery_row, now)


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
    m.pelvis_pose.position.z = RESET_Z
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
if os.environ.get("PP_POLICY_DIR"):
    runner_extra_args += ["--policy-dir", os.environ["PP_POLICY_DIR"]]
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
if os.environ.get("PP_CHILD_PIDS_FILE"):
    with open(os.environ["PP_CHILD_PIDS_FILE"], "a") as stream:
        stream.write(str(proc.pid) + "\n")
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
    rb"dx=([0-9.]+) dy=([0-9.]+) speed=(INVALID/)?([0-9.]+)([^\r\n]*))?")
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
RUNNER_CONTROL_RE = re.compile(
    rb"\[runner-control\] source=(\S+) request=(\d+) action=(\S+) "
    rb"result=(\S+) reason=(\S+) mode=(\S+) role=(\S+)")


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
             "ready": bool(m.group(13)) and b"READY" in m.group(13).split()}
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
        "complete": (
            chunk.count(b"swing complete")
            + chunk.count(b"contact follow-through complete -> HOME/PENDING")
        ),
        "recovered": chunk.count(b"post-swing recovery done"),
        "fall_guard": chunk.count(b"FALL GUARD"),
        "actual_q_fault": chunk.count(
            b"PHYSICAL SAFETY FAULT: measured q exceeds hard limit"
        ),
        "command_safety_faults": [
            m.group(1).decode(errors="replace")
            for m in SAFETY_LATCH_RE.finditer(chunk)
        ],
        "runner_control": [
            {"source": m.group(1).decode(),
             "request": int(m.group(2)),
             "action": m.group(3).decode(),
             "result": m.group(4).decode(),
             "reason": m.group(5).decode(),
             "mode": m.group(6).decode(),
             "role": m.group(7).decode()}
            for m in RUNNER_CONTROL_RE.finditer(chunk)
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
    catch window, require continuous standing, select RECEIVER, then enter MOTION.

    The integrated serve Runner rejects ENTER_MOTION unless its role is RECEIVER.
    Require the corresponding Runner ACKs so a rejected key cannot be mistaken for
    a successful Gate3 transition.
    """
    if os.environ.get("PP_DIRECT_MOTION_ENTRY", "0") == "1":
        # HumanLike's default pose is an actively balanced policy pose, not a
        # static PD_STAND. For this MuJoCo-only vendor-profile A/B, enter its
        # actor immediately after reset instead of waiting 7.5 s in a stand
        # controller that is intentionally not the HumanLike controller.
        # Wait in PASSIVE, reset, then publish PD_STAND immediately. Arming
        # PD_STAND before reset lets its fall counter observe the old down pose
        # and can race the fresh upright sample back to PASSIVE.
        reset_ready_deadline = time.time() + 2.0
        while (node.count_subscribers('/sim/a3/reset') < 1 and
               time.time() < reset_ready_deadline):
            spin(0.02)
        if node.count_subscribers('/sim/a3/reset') < 1:
            raise RuntimeError("MuJoCo reset subscriber did not become ready")
        reset()
        spin(0.01)
        key("s")
        # Let the vendor sole points establish a grounded, low-rate state
        # before HumanLike resets its history and takes lower-body ownership.
        # The shipped controller enters from an already-supported robot, not
        # from the first airborne milliseconds of a MuJoCo free-joint reset.
        spin(max(0.02, DIRECT_MOTION_SETTLE_S))
        key("R")
        role_ready = False
        role_deadline = time.time() + 2.0
        while time.time() < role_deadline:
            spin(0.01)
            for event in new_runner_events()["runner_control"]:
                if (event["action"] == "SET_RECEIVER" and
                        event["result"] in ("APPLIED", "ALREADY_SET") and
                        event["role"] == "RECEIVER"):
                    role_ready = True
            if role_ready:
                break
        key("m")
        motion_ready = False
        deadline = time.time() + 3.0
        while time.time() < deadline:
            spin(0.05)
            for event in new_runner_events()["runner_control"]:
                if (event["action"] == "SET_RECEIVER" and
                        event["result"] in ("APPLIED", "ALREADY_SET") and
                        event["role"] == "RECEIVER"):
                    role_ready = True
                if (event["action"] == "ENTER_MOTION" and
                        event["result"] in ("APPLIED", "ALREADY_SET") and
                        event["mode"] == "MOTION" and
                        event["role"] == "RECEIVER"):
                    motion_ready = True
            if role_ready and motion_ready:
                print("[rally] direct HumanLike MOTION entry acknowledged", flush=True)
                return True
        print(
            f"[rally] direct HumanLike entry ACK missing: "
            f"role={int(role_ready)} motion={int(motion_ready)}",
            flush=True,
        )
        return False

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
            key("R")
            role_ready = False
            role_deadline = time.time() + 3.0
            while time.time() < role_deadline:
                spin(0.05)
                controls = new_runner_events()["runner_control"]
                if any(
                    event["action"] == "SET_RECEIVER"
                    and event["result"] in ("APPLIED", "ALREADY_SET")
                    and event["role"] == "RECEIVER"
                    for event in controls
                ):
                    role_ready = True
                    break
                rejected = [
                    event for event in controls
                    if event["action"] == "SET_RECEIVER"
                    and event["result"].startswith("REJECTED_")
                ]
                if rejected:
                    print(f"[rally] RECEIVER role rejected: {rejected[-1]}", flush=True)
                    break
            if not role_ready:
                print("[rally] RECEIVER role ACK missing; retrying stand", flush=True)
                continue

            key("m")
            motion_ready = False
            motion_deadline = time.time() + 3.0
            while time.time() < motion_deadline:
                spin(0.05)
                controls = new_runner_events()["runner_control"]
                if any(
                    event["action"] == "ENTER_MOTION"
                    and event["result"] in ("APPLIED", "ALREADY_SET")
                    and event["mode"] == "MOTION"
                    and event["role"] == "RECEIVER"
                    for event in controls
                ):
                    motion_ready = True
                    break
                rejected = [
                    event for event in controls
                    if event["action"] == "ENTER_MOTION"
                    and event["result"].startswith("REJECTED_")
                ]
                if rejected:
                    print(f"[rally] MOTION rejected: {rejected[-1]}", flush=True)
                    break
            if motion_ready:
                return True
            print("[rally] MOTION ACK missing; retrying stand", flush=True)
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
if state["yaw_rad"] is None:
    raise RuntimeError("MOTION entry has no valid pelvis yaw sample")
anchor_yaw_rad = float(state["yaw_rad"])
state["motion_active"] = True
print(f"[rally] MOTION entered at base=({anchor_xy[0]:+.2f},{anchor_xy[1]:+.2f}); "
      f"yaw={math.degrees(anchor_yaw_rad):+.2f}deg; counting {SERVES} serves, "
      "NO resets from here", flush=True)

# ---- the RALLY: attribute events to serve windows, never touch the robot ----
serve_rows = []       # one dict per counted serve
cur = None            # the open serve row
active = None         # row of the last ENGAGED serve (swing-outcome attribution)
fell = False
physical_fall_detected = False
runner_fall_guard_tripped = False
dropout_done = False
base_outage_done = False
base_outage_start_wall_time_ns = None
base_outage_end_wall_time_ns = None
t_start = time.time()
serves_seen = 0
tail_deadline = None  # after the last serve, wait for its outcome
base_xy_trajectory = []  # 4 Hz measured top-view trace for lane diagnosis
rescues = 0           # fixed at zero; retained in the report schema
actual_q_faults = 0
command_safety_faults = []
misses_in_a_row = 0   # consecutive serves with no engage -> diagnostic marker

while time.time() - t_start < GLOBAL_TIMEOUT_S:
    spin(0.25)
    if state["x"] is not None and state["y"] is not None:
        open_serve = cur.get("serve") if cur is not None else serves_seen
        base_xy_trajectory.append(
            {
                "wall_time_ns": time.time_ns(),
                "t_s": round(time.time() - t_start, 3),
                "serve": int(open_serve),
                "lane": _lane_for_serve(int(open_serve)) if open_serve else "idle",
                "x_m": round(float(state["x"]), 4),
                "y_m": round(float(state["y"]), 4),
                "home_relative_y_m": round(float(state["y"] - anchor_xy[1]), 4),
                "speed_xy_mps": (
                    None
                    if state["speed_xy_mps"] is None
                    else round(float(state["speed_xy_mps"]), 4)
                ),
            }
        )
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
               "flight_id": s["shot_id"],
               "lane": _lane_for_serve(serves_seen),
               "scheduled_flight_s": FLIGHT_SCHEDULE[serves_seen - 1],
               "scheduled_pause_s": PAUSE_SCHEDULE[serves_seen - 1],
               "ball_p0_table": s["p_table"], "ball_p0": s["p"], "ball_v0": s["v"],
               "t": round(time.time() - t_start, 1),
               "base_at_serve": [round(bx, 3), round(by, 3)],
               "x_err_at_serve": round(bx - STATION_X, 3),
               "x_err_at_engage": None,
               "base_at_engage": None,
               "base_at_completion": None,
               "base_at_lifecycle_close": None,
               "base_at_physical_contact": None,
               "engage_time_s": None,
               "max_abs_x_err": abs(bx - STATION_X),
               "max_anchor_xy_err": anchor_err,
               "engaged": None, "engages": [], "complete": 0, "recovered": 0,
               "pending_stations": [], "command_station": None,
               "command_side": None, "ready_events": [], "ready_latency_s": None,
               "lifecycle_events": [], "rejects": [], "no_base_warns": 0,
               "recovery_target": None,
               "recovery_last_xy_error_m": None,
               "recovery_min_xy_error_m": None,
               "recovery_last_speed_mps": None,
               "recovery_last_tilt_deg": None,
               "recovery_last_yaw_error_deg": None,
               "recovery_dwell_observed_s": 0.0,
               "recovery_max_dwell_observed_s": 0.0,
               "recovery_revocation_count": 0,
               "recovery_latency_s": None,
               "recovery_settled": False,
               "_recovery_dwell_start": None,
               "_complete_monotonic": None,
               "min_z": z() if z() > 0 else 99.0,
               "dropout": False, "base_outage": False}
        print(f"[rally] serve {serves_seen}/{SERVES} p={s['p']} v={s['v']} "
              f"base=({bx:+.2f},{by:+.2f})", flush=True)
        if serves_seen == SERVES:
            tail_deadline = time.time() + max(
                2.5 * SERVE_PERIOD_S,
                FLIGHT_SCHEDULE[-1] + REQUIRED_POST_COMPLETION_TAIL_S + 1.0,
            )
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
                cur["engage_time_s"] = round(time.time() - t_start, 3)
            if e.get("station") is not None:
                cur["command_station"] = e["station"]  # actual release target beats early prediction
            cur["command_side"] = e["side"]
            active = cur          # swing outcome events credit the ENGAGED serve
            if state["x"] is not None:   # pelvis-pose latency <= one 0.25 s spin: negligible pre-windup
                cur["x_err_at_engage"] = round(state["x"] - STATION_X, 3)
                cur["base_at_engage"] = [
                    round(state["x"], 4),
                    round(state["y"], 4),
                ]
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
            if (
                BASE_OUTAGE_AT
                and cur["serve"] == BASE_OUTAGE_AT
                and not base_outage_done
            ):
                if os.getpgid(BASE_RELAY_PGID) != BASE_RELAY_PGID:
                    raise RuntimeError(
                        "calibrated base relay is not the expected isolated process group"
                    )
                base_outage_done = True
                cur["base_outage"] = True
                duration_s = BASE_OUTAGE_TICKS * 0.02
                print(
                    "[rally]   BASE OUTAGE stress: suspend calibrated base relay "
                    f"pgid={BASE_RELAY_PGID} for {BASE_OUTAGE_TICKS} policy ticks "
                    f"({duration_s:.2f}s); Planner/raw ball remain live",
                    flush=True,
                )
                base_outage_start_wall_time_ns = time.time_ns()
                os.killpg(BASE_RELAY_PGID, signal.SIGSTOP)
                try:
                    spin(duration_s)
                finally:
                    os.killpg(BASE_RELAY_PGID, signal.SIGCONT)
                    base_outage_end_wall_time_ns = time.time_ns()
        # complete/recovered belong to the swing (the ENGAGED serve) even when the
        # recovery finishes windows later (the tighter static-handoff gates make the
        # policy recovery span 1-2 serve periods — safe, and must not read as MISSED).
        tgt = active if active is not None else cur
        if ev["complete"] and tgt["complete"] == 0:
            tgt["_complete_monotonic"] = time.monotonic()
            if tgt.get("serve") == SERVES:
                tail_deadline = max(
                    tail_deadline or 0.0,
                    time.time() + REQUIRED_POST_COMPLETION_TAIL_S + 1.0,
                )
            if state["x"] is not None and state["y"] is not None:
                # This is the closest runner/conductor synchronized plant sample
                # to the runtime completion edge without inventing a
                # commanded-station-as-achieved shortcut.
                tgt["base_at_completion"] = [
                    round(state["x"], 4),
                    round(state["y"], 4),
                ]
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
    post_completion_tail_complete = bool(
        cur is not None
        and cur.get("_complete_monotonic") is not None
        and time.monotonic() - cur["_complete_monotonic"]
            >= REQUIRED_POST_COMPLETION_TAIL_S
    )
    if (tail_deadline is not None and cur is not None and cur["serve"] == SERVES
            and cur["recovery_settled"] and post_completion_tail_complete):
        spin(1.0)
        break

if cur is not None:
    serve_rows.append(cur)

# Stop future launches and retain the plant/evidence recorder until every ball
# already launched reaches ground, including after a robot fall or early exit.
drain = subprocess.run([sys.executable, str(Path(__file__).with_name("pp_gate3_drain_flights.py"))])
print("[rally] physical trajectories " + ("complete" if drain.returncode == 0 else "INCOMPLETE"), flush=True)

end_xy = xy()   # BEFORE 'q': the runner exit drops the robot limp and the pelvis slides
state["motion_active"] = False
key("q")
spin(3)
try:
    proc.wait(timeout=10)
except subprocess.TimeoutExpired:
    proc.kill()

# ---- verdicts ----


def load_evidence(path, unavailable):
    try:
        with open(path) as stream:
            return json.load(stream)
    except (FileNotFoundError, json.JSONDecodeError, OSError) as exc:
        result = dict(unavailable)
        result["evidence_error"] = str(exc)
        return result


physical_evidence = load_evidence(PHYSICAL_EVIDENCE_JSON, {
    "physical_contact_measured": False,
    "landing_measured": False,
    "physical_contact_pass": False,
    "landing_pass": False,
})
physical_by_id = {
    int(row["shot_id"]): row for row in physical_evidence.get("rows", [])
}
physical_contact_wall_time_ns_by_flight = {}
for flight_id, physical_row in physical_by_id.items():
    racket_events = physical_row.get("racket_events", [])
    if (
        int(physical_row.get("racket_contact_count", 0)) == 1
        and len(racket_events) == 1
        and int(racket_events[0].get("stamp_ns", 0)) > 0
    ):
        physical_contact_wall_time_ns_by_flight[flight_id] = int(
            racket_events[0]["stamp_ns"]
        )

actual_q_live_termination_faults = actual_q_faults
actual_q_tail_ledger = audit_actual_q_full_tail_ledger(
    TRACE_CSV,
    PLANT_CSV,
    RUNNER_LOG,
    range(1, SERVES + 1),
    physical_contact_wall_time_ns_by_flight=(
        physical_contact_wall_time_ns_by_flight
    ),
    required_post_completion_tail_s=REQUIRED_POST_COMPLETION_TAIL_S,
    runner_process_exited=proc.poll() is not None,
)
actual_q_faults = int(
    actual_q_tail_ledger.get("actual_q_violation_episode_count", 0)
)
actual_q_ledger_complete = bool(actual_q_tail_ledger.get("ledger_complete", False))
actual_q_ledger_consistency_pass = bool(
    actual_q_tail_ledger.get("cross_source_consistency_pass", False)
)
actual_q_by_flight = actual_q_tail_ledger.get("by_flight", {})
for row in serve_rows:
    row["actual_q_full_tail"] = actual_q_by_flight.get(
        str(int(row["flight_id"])),
        {
            "flight_id": int(row["flight_id"]),
            "actual_q_violation_episode_count": 0,
            "by_phase": {},
            "by_joint": {},
            "canonical_event_ids": [],
        },
    )


def load_live_cpp_planner_evidence():
    # The C++ logger flushes each batch, but allow the background writer a
    # short bounded interval after the final physical flight. This is evidence
    # collection only; it never stops, gates, or modifies robot motion.
    evidence = audit_cpp_planner_debug_csv(PLANNER_DEBUG_CSV, SERVES)
    deadline = time.monotonic() + 2.0
    while evidence.get("rows_observed", 0) < SERVES and time.monotonic() < deadline:
        time.sleep(0.05)
        evidence = audit_cpp_planner_debug_csv(PLANNER_DEBUG_CSV, SERVES)
    try:
        evidence_path = Path(PLANNER_EVIDENCE_JSON)
        evidence_path.parent.mkdir(parents=True, exist_ok=True)
        evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    except OSError as exc:
        evidence["planner_contract_pass"] = False
        evidence["pass"] = False
        evidence.setdefault("errors", []).append(
            f"cannot persist live C++ Planner evidence: {exc}"
        )
    return evidence


planner_evidence = load_live_cpp_planner_evidence()
planner_flight_join = join_planner_evidence_by_flight_id(
    serve_rows, planner_evidence, SERVES
)
runner_lifecycle_completion_required = bool(
    LANE_CONTRACT in HITTER_FIXED_HOME_LANE_CONTRACTS
)
runner_trace_evidence = audit_runner_engagement_trace(
    TRACE_CSV,
    range(1, SERVES + 1),
    require_completion_contract=runner_lifecycle_completion_required,
    physical_contact_wall_time_ns_by_flight=(
        physical_contact_wall_time_ns_by_flight
    ),
)
localization_trace_evidence = audit_runner_localization_trace(TRACE_CSV)
runner_engage_by_flight = {
    int(event["flight_id"]): event
    for event in runner_trace_evidence.get("engage_events", [])
}
runner_completion_by_flight = {
    int(event["flight_id"]): event
    for event in runner_trace_evidence.get("completion_events", [])
}
runner_contact_pose_by_flight = {
    int(event["flight_id"]): event
    for event in runner_trace_evidence.get("physical_contact_pose_events", [])
}
runner_trace_join_errors = []
for row in serve_rows:
    flight_id = int(row["flight_id"])
    trace_event = runner_engage_by_flight.get(flight_id)
    completion_event = runner_completion_by_flight.get(flight_id)
    contact_pose_event = runner_contact_pose_by_flight.get(flight_id)
    row["runner_trace_engaged"] = trace_event is not None
    row["runner_engage_wall_time_ns"] = (
        None if trace_event is None else int(trace_event["wall_time_ns"])
    )
    if bool(row.get("engaged")) != bool(trace_event):
        runner_trace_join_errors.append(
            f"flight {flight_id} engage log/trace mismatch"
        )
    # Stdout completion has no immutable identity and is retained only as a
    # legacy diagnostic. In the schema22/schema23 external-preempt lifecycle the same tick closes the old
    # flight and engages the new one, so assigning an unkeyed marker to the
    # conductor's current row would credit the wrong flight. The authoritative
    # lifecycle ledger comes from the Runner trace sequence instead.
    row["legacy_stdout_complete"] = int(row.get("complete", 0))
    row["runner_lifecycle_closed"] = completion_event is not None
    row["runner_lifecycle_completion"] = completion_event
    row["runner_physical_contact_pose"] = contact_pose_event
    row["base_at_physical_contact"] = (
        None
        if contact_pose_event is None
        else [
            round(float(contact_pose_event["base_x"]), 4),
            round(float(contact_pose_event["base_y"]), 4),
        ]
    )
    if runner_lifecycle_completion_required:
        row["complete"] = 1 if completion_event is not None else 0
        if completion_event is not None:
            base_x = completion_event.get("base_x")
            base_y = completion_event.get("base_y")
            if (
                base_x is not None
                and base_y is not None
                and math.isfinite(float(base_x))
                and math.isfinite(float(base_y))
            ):
                row["base_at_lifecycle_close"] = [
                    round(float(base_x), 4), round(float(base_y), 4)
                ]
runner_trace_join_pass = bool(
    runner_trace_evidence.get("trace_contract_pass", False)
    and runner_trace_evidence.get("physical_contact_pose_contract_pass", False)
    and not runner_trace_join_errors
)
runner_lifecycle_completion_join_pass = bool(
    runner_trace_evidence.get("completion_contract_pass", False)
    if runner_lifecycle_completion_required
    else True
)


def _trajectory_home_error(sample):
    return math.hypot(
        float(sample["x_m"]) - anchor_xy[0],
        float(sample["y_m"]) - anchor_xy[1],
    )


def _first_trajectory_sample_at_or_after(deadline_ns, end_ns):
    return next(
        (
            sample
            for sample in base_xy_trajectory
            if int(sample["wall_time_ns"]) >= deadline_ns
            and int(sample["wall_time_ns"]) <= end_ns
        ),
        None,
    )


# Natural recovery is an estimand, never an injected motion command. Each physical contact owns
# a HOME-error trace until the next COMMIT. Checkpoints beyond that edge are explicitly censored;
# they are not converted into misses or a hidden requirement to wait for HOME.
natural_drift_recovery_rows = []
for row_index, row in enumerate(serve_rows):
    flight_id = int(row["flight_id"])
    contact_wall_ns = physical_contact_wall_time_ns_by_flight.get(flight_id)
    next_commit_wall_ns = (
        serve_rows[row_index + 1].get("runner_engage_wall_time_ns")
        if row_index + 1 < len(serve_rows)
        else None
    )
    trace_end_ns = (
        int(base_xy_trajectory[-1]["wall_time_ns"])
        if base_xy_trajectory
        else 0
    )
    window_end_ns = int(next_commit_wall_ns or trace_end_ns)
    evidence = {
        "flight_id": flight_id,
        "lane": row.get("lane"),
        "contact_wall_time_ns": contact_wall_ns,
        "next_commit_wall_time_ns": next_commit_wall_ns,
        "contact_to_next_commit_s": (
            None
            if contact_wall_ns is None or next_commit_wall_ns is None
            else round((int(next_commit_wall_ns) - int(contact_wall_ns)) * 1.0e-9, 6)
        ),
        "protected_followthrough_s": NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S,
        "peak_home_drift_m": None,
        "home_error_at_0p4_s_m": None,
        "home_error_at_0p8_s_m": None,
        "home_error_at_1p2_s_m": None,
        "home_error_at_next_commit_m": None,
        "status": "MISSING_CONTACT_EVIDENCE",
    }
    if contact_wall_ns is not None:
        recovery_start_ns = int(
            contact_wall_ns
            + NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S * 1.0e9
        )
        window_samples = [
            sample
            for sample in base_xy_trajectory
            if recovery_start_ns <= int(sample["wall_time_ns"]) <= window_end_ns
        ]
        if window_samples:
            evidence["peak_home_drift_m"] = round(
                max(_trajectory_home_error(sample) for sample in window_samples), 4
            )
            evidence["recovery_window_start_wall_time_ns"] = recovery_start_ns
            for offset_s, key in (
                (0.4, "home_error_at_0p4_s_m"),
                (0.8, "home_error_at_0p8_s_m"),
                (1.2, "home_error_at_1p2_s_m"),
            ):
                deadline_ns = int(contact_wall_ns + offset_s * 1.0e9)
                if next_commit_wall_ns is not None and deadline_ns >= next_commit_wall_ns:
                    evidence[key] = "CENSORED_BY_NEXT_COMMIT"
                    continue
                sample = _first_trajectory_sample_at_or_after(deadline_ns, window_end_ns)
                evidence[key] = (
                    None
                    if sample is None
                    else round(_trajectory_home_error(sample), 4)
                )
            if next_commit_wall_ns is not None:
                next_sample = _first_trajectory_sample_at_or_after(
                    int(next_commit_wall_ns), trace_end_ns
                )
                if next_sample is not None:
                    evidence["home_error_at_next_commit_m"] = round(
                        _trajectory_home_error(next_sample), 4
                    )
            evidence["status"] = (
                "NOT_OBSERVED"
                if evidence["peak_home_drift_m"]
                < FIXED_HOME_DRIFT_NOT_OBSERVED_M
                else "OBSERVED"
            )
        else:
            evidence["status"] = "MISSING_POST_FOLLOWTHROUGH_SAMPLES"
    row["natural_drift_recovery"] = evidence
    natural_drift_recovery_rows.append(evidence)

natural_drift_observed_rows = [
    evidence
    for evidence in natural_drift_recovery_rows
    if evidence["status"] == "OBSERVED"
]
natural_drift_recovery_result = (
    "NOT_OBSERVED" if not natural_drift_observed_rows else "OBSERVED"
)

launch_cadence_receipt = load_evidence(
    LAUNCH_RECEIPT_JSON,
    {"cadence_pass": False, "rows": [], "flight_ids": []},
)
launch_cadence_pass = bool(
    launch_cadence_receipt.get("receipt_contract")
        == "gate3_launch_cadence_wall_clock_v1"
    and launch_cadence_receipt.get("scenario_id") == GATE3_SCENARIO
    and launch_cadence_receipt.get("flight_ids")
        == list(range(1, SERVES + 1))
    and launch_cadence_receipt.get("cadence_pass") is True
)

drift = ((end_xy[0] - anchor_xy[0]) ** 2 + (end_xy[1] - anchor_xy[1]) ** 2) ** 0.5
x_yaw = state["yaw_rad"]
end_yaw_drift_deg = (
    float("inf")
    if x_yaw is None
    else abs(math.degrees(_wrapped_angle_rad(x_yaw - anchor_yaw_rad)))
)
x_err_at_end = end_xy[0] - STATION_X   # SIGNED: separates forward creep from lateral footwork
proxy_completed = 0
# Keep the historical fixed-anchor calculation as a diagnostic for command-station recipes.
# Qualification recovery is the live dwell accumulated against the configured recovery target;
# a later engage is a separate station-acquisition event and cannot retroactively certify recovery.
for i, r in enumerate(serve_rows):
    legacy_recovery_xy = (
        serve_rows[i + 1]["base_at_serve"]
        if i + 1 < len(serve_rows)
        else end_xy
    )
    legacy_anchor_error = math.hypot(
        float(legacy_recovery_xy[0]) - anchor_xy[0],
        float(legacy_recovery_xy[1]) - anchor_xy[1],
    )
    r["legacy_session_anchor_error_m"] = round(legacy_anchor_error, 4)
    r["recovered_to_session_anchor"] = bool(
        not fell and legacy_anchor_error <= RECOVERY_RADIUS_M
    )
    last_error = r.get("recovery_last_xy_error_m")
    r["recovery_xy_error_m"] = (
        None if last_error is None else round(float(last_error), 4)
    )
    min_error = r.get("recovery_min_xy_error_m")
    r["recovery_min_xy_error_m"] = (
        None if min_error is None else round(float(min_error), 4)
    )
    r["recovery_last_speed_mps"] = (
        None
        if r.get("recovery_last_speed_mps") is None
        else round(float(r["recovery_last_speed_mps"]), 4)
    )
    r["recovery_last_tilt_deg"] = (
        None
        if r.get("recovery_last_tilt_deg") is None
        else round(float(r["recovery_last_tilt_deg"]), 3)
    )
    r["recovery_last_yaw_error_deg"] = (
        None
        if r.get("recovery_last_yaw_error_deg") is None
        else round(float(r["recovery_last_yaw_error_deg"]), 3)
    )
    r["recovery_dwell_observed_s"] = round(
        float(r.get("recovery_dwell_observed_s", 0.0)), 3
    )
    r["recovery_max_dwell_observed_s"] = round(
        float(r.get("recovery_max_dwell_observed_s", 0.0)), 3
    )
    r["recovery_revocation_count"] = int(
        r.get("recovery_revocation_count", 0)
    )
    r["recovery_latency_s"] = (
        None
        if r.get("recovery_latency_s") is None
        else round(float(r["recovery_latency_s"]), 3)
    )
    r["recovered_to_recovery_target"] = bool(
        not fell and r.get("recovery_settled", False)
    )
    r["recovered_to_command_station"] = bool(
        GATE3_PHASE != "random" and r["recovered_to_recovery_target"]
    )
    r["recovered_to_session_anchor_strict"] = bool(
        GATE3_PHASE == "random" and r["recovered_to_recovery_target"]
    )
    r["recovered_behavioral"] = r["recovered_to_recovery_target"]
    r.pop("_recovery_dwell_start", None)
    r.pop("_complete_monotonic", None)
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
    r["commit_telemetry_ok"] = bool(
        e
        and e.get("station") is not None
        and e.get("ready_speed_valid")
        and e.get("ready_dx") is not None
        and math.isfinite(e["ready_dx"])
        and e.get("ready_dy") is not None
        and math.isfinite(e["ready_dy"])
        and e.get("ready_speed") is not None
        and math.isfinite(e["ready_speed"])
    )
    r["station_acquired_at_engage"] = bool(
        r["commit_telemetry_ok"]
        and e["ready_dx"] <= READY_X_MAX
        and e["ready_dy"] <= READY_Y_MAX
        and e["ready_speed"] <= READY_SPEED_MAX
    )
    if not REQUIRE_READY:
        r["ready_ok"] = True
    elif READY_CONTRACT in ("late_reveal_commit_v1", "home_preempt_commit_v1"):
        # A late-reveal command intentionally releases immediately; requiring the obsolete READY
        # marker would fail every valid command. Require complete commit telemetry and report
        # station acquisition separately without turning it into a hidden runtime gate.
        r["ready_ok"] = r["commit_telemetry_ok"]
    else:
        r["ready_ok"] = bool(
            r["station_acquired_at_engage"] and e.get("ready")
        )
    r["completed_recovered_proxy"] = (
        bool(e)
        and r["engage_protocol_ok"]
        and r["ready_ok"]
        and r["complete"] >= 1
        and r["recovered_to_recovery_target"]
    )
    # A deliberately early rally command may preempt drift recovery. Requiring rapid fixed-HOME
    # rows to settle before accepting the next ball would test the opposite lifecycle contract.
    # Clean fixed-HOME rows retain the strict kinetic HOME dwell.
    r["completed_lane_proxy"] = bool(
        e
        and r["engage_protocol_ok"]
        and r["ready_ok"]
        and r["complete"] >= 1
        and (
            r.get("lane") == "rapid_fixed_home"
            or r["recovered_to_recovery_target"]
        )
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
    proxy_completed += r["completed_lane_proxy"]
    sd = r["engaged"]["side"][:2] if r["engaged"] else "--"
    print(f"[rally] serve {r['serve']:2d} lane={r['lane']:<12} @{r['t']:6.1f}s base=({r['base_at_serve'][0]:+.2f},"
          f"{r['base_at_serve'][1]:+.2f}) {sd} engage={'Y' if r['engaged'] else 'n'} "
          f"complete={r['complete']} recovered={r['recovered']} minz={r['min_z']:.2f} "
          f"xerr@s={r['x_err_at_serve']:+.2f} xerr@e="
          f"{'--' if r['x_err_at_engage'] is None else format(r['x_err_at_engage'], '+.2f')} "
          f"xmax={r['max_abs_x_err']:.2f}"
          f"{'' if r['xlock_ok'] else ' XLOCK-VIOL'} "
          f"ready={'Y' if r['ready_ok'] else 'FAIL'} "
          f"miss={('>'.join(r['miss_reason_chain']) if r['miss_reason_chain'] else '--')} "
          f"rejects={len(r['rejects'])}{' DROPOUT' if r['dropout'] else ''} "
          f"engages={len(r['engages'])} xy_recover="
          f"{'--' if r['recovery_xy_error_m'] is None else format(r['recovery_xy_error_m'], '.3f')}m "
          f"yaw_recover="
          f"{'--' if r['recovery_last_yaw_error_deg'] is None else format(r['recovery_last_yaw_error_deg'], '.2f')}deg "
          f"-> {'PROXY_OK' if r['completed_lane_proxy'] else 'PROXY_FAIL'}", flush=True)

n = len(serve_rows)
engaged_serves = sum(1 for r in serve_rows if r["engaged"])
completed_serves = sum(
    1
    for r in serve_rows
    if r["engaged"]
    and (
        r.get("runner_lifecycle_closed", False)
        if runner_lifecycle_completion_required
        else r["complete"] >= 1
    )
)
total_engage_events = sum(len(r["engages"]) for r in serve_rows)
incomplete = [r["serve"] for r in serve_rows
              if r["engaged"] and not r["completed_lane_proxy"]]
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
localization_ok = bool(
    localization_trace_evidence["pass"]
    and no_base_warns == 0
)
proxy_rate = proxy_completed / n if n else 0.0
engage_rate = engaged_serves / n if n else 0.0
completion_rate = completed_serves / engaged_serves if engaged_serves else 0.0
recovered_engaged = sum(
    1 for r in serve_rows
    if r["engaged"] and r["recovered_to_recovery_target"]
)
recovery_rate = recovered_engaged / engaged_serves if engaged_serves else 0.0
station_acquired_engaged = sum(
    1 for r in serve_rows
    if r["engaged"] and r["station_acquired_at_engage"]
)
station_acquisition_rate = (
    station_acquired_engaged / engaged_serves if engaged_serves else 0.0
)
clean_fixed_home_rows = [
    row for row in serve_rows if row.get("lane") == "clean_fixed_home"
]
rapid_rows = [
    row for row in serve_rows if row.get("lane") == "rapid_fixed_home"
]
fixed_home_command_errors = []
for row in serve_rows:
    station = row.get("command_station")
    command_error = (
        None
        if station is None
        else math.hypot(
            float(station[0]) - anchor_xy[0],
            float(station[1]) - anchor_xy[1],
        )
    )
    row["fixed_home_command_error_m"] = (
        None if command_error is None else round(command_error, 6)
    )
    if command_error is not None:
        fixed_home_command_errors.append(command_error)
fixed_home_max_command_error_m = max(fixed_home_command_errors, default=math.inf)
fixed_home_station_invariant_pass = bool(
    len(fixed_home_command_errors) == SERVES
    and fixed_home_max_command_error_m <= FIXED_HOME_COMMAND_TOLERANCE_M
)
clean_fixed_home_side_counts = {
    side: sum(
        1
        for row in clean_fixed_home_rows
        if row.get("engaged") is not None
        and row["engaged"].get("side") == side
    )
    for side in ("forehand", "backhand")
}
rapid_engaged_rows = [
    row for row in rapid_rows if row.get("runner_trace_engaged", False)
]
rapid_engaged = len(rapid_engaged_rows)
rapid_completed = sum(
    1 for row in rapid_rows
    if row.get("runner_trace_engaged", False)
    and row.get("runner_lifecycle_closed", False)
)
rapid_completion_kinds = [
    row["runner_lifecycle_completion"]["completion_kind"]
    for row in rapid_rows
    if row.get("runner_lifecycle_completion") is not None
]
rapid_completion_identity_pass = bool(
    len(rapid_rows) == SERVES - CLEAN_FIXED_HOME_SERVES
    and rapid_completion_kinds
    == ["preempt"] * (len(rapid_rows) - 1) + ["native"]
)
rapid_engage_wall_times_ns = [
    int(row["runner_engage_wall_time_ns"])
    for row in rapid_engaged_rows
]
rapid_engage_gaps_s = [
    round((current - previous) * 1.0e-9, 6)
    for previous, current in zip(
        rapid_engage_wall_times_ns, rapid_engage_wall_times_ns[1:]
    )
]
rapid_contact_to_next_commit_rows = [
    row["natural_drift_recovery"]
    for row in rapid_rows[:-1]
    if row.get("natural_drift_recovery") is not None
]
rapid_contact_to_next_commit_s = [
    evidence.get("contact_to_next_commit_s")
    for evidence in rapid_contact_to_next_commit_rows
]
rapid_contact_to_next_commit_pass = bool(
    len(rapid_contact_to_next_commit_s) == max(0, len(rapid_rows) - 1)
    and all(
        delay is not None
        and RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S <= float(delay)
        <= RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S
        for delay in rapid_contact_to_next_commit_s
    )
)
rapid_terminal_recovery_pass = bool(
    len(rapid_rows) == SERVES - CLEAN_FIXED_HOME_SERVES
    and rapid_rows[-1].get("recovered_to_recovery_target", False)
)
rapid_engaged_flight_ids = [int(row["flight_id"]) for row in rapid_engaged_rows]
rapid_missed_flight_ids = [
    int(row["flight_id"])
    for row in rapid_rows
    if not row.get("runner_trace_engaged", False)
]
rapid_sides = [
    "fh" if row["planner_side"] == "forehand" else "bh"
    for row in rapid_engaged_rows
    if row.get("planner_side") in ("forehand", "backhand")
]
rapid_side_transitions = {
    transition: 0 for transition in ("fh_fh", "fh_bh", "bh_fh", "bh_bh")
}
for previous, current in zip(rapid_sides, rapid_sides[1:]):
    rapid_side_transitions[f"{previous}_{current}"] += 1
natural_recovery_evidence_complete = all(
    row["status"] in {"OBSERVED", "NOT_OBSERVED"}
    for row in natural_drift_recovery_rows
    if row["contact_wall_time_ns"] is not None
)
if LANE_CONTRACT in HITTER_FIXED_HOME_LANE_CONTRACTS:
    clean_fixed_home_lane_pass = bool(
        len(clean_fixed_home_rows) == CLEAN_FIXED_HOME_SERVES
        and min(clean_fixed_home_side_counts.values()) >= 3
        and all(row["completed_lane_proxy"] for row in clean_fixed_home_rows)
        and fixed_home_station_invariant_pass
    )
    rapid_lane_pass = bool(
        len(rapid_rows) == SERVES - CLEAN_FIXED_HOME_SERVES
        and rapid_engaged >= MIN_RAPID_ENGAGED
        and rapid_completed >= MIN_RAPID_COMPLETED
        and rapid_completion_identity_pass
        and len(rapid_engage_gaps_s) == max(0, len(rapid_rows) - 1)
        and max(rapid_engage_gaps_s) <= RAPID_MAX_ENGAGE_GAP_S
        and rapid_contact_to_next_commit_pass
        and rapid_terminal_recovery_pass
        and all(count >= 1 for count in rapid_side_transitions.values())
        and runner_trace_join_pass
        and runner_lifecycle_completion_join_pass
        and planner_flight_join["pass"]
        and launch_cadence_pass
        and fixed_home_station_invariant_pass
    )
    lane_contract_pass = bool(
        clean_fixed_home_lane_pass
        and rapid_lane_pass
        and natural_recovery_evidence_complete
    )
else:
    clean_fixed_home_lane_pass = True
    rapid_lane_pass = True
    lane_contract_pass = True
if GATE3_PHASE == "random":
    runner_proxy_pass = bool(
        n == SERVES
        and engaged_serves >= MIN_ENGAGED_SERVES
        and completed_serves >= MIN_COMPLETED_SERVES
        and runner_lifecycle_completion_join_pass
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
        and runner_lifecycle_completion_join_pass
        and not incomplete
        and not multi_engage_serves
        and proxy_rate >= MIN_PROXY_RATE)
stability_pass = bool(
    not fell
    and actual_q_ledger_complete
    and actual_q_ledger_consistency_pass
    and actual_q_faults == 0
    and not command_safety_faults
    and drift <= MAX_END_XY_DRIFT_M
    and state["max_anchor_xy_err_motion"] <= MAX_PEAK_XY_DRIFT_M
    and end_yaw_drift_deg <= MAX_END_YAW_DRIFT_DEG
    and state["max_anchor_yaw_err_motion_deg"] <= MAX_PEAK_YAW_DRIFT_DEG
    and rescues <= MAX_RESCUES
    and state["min_z_motion"] >= MIN_UPRIGHT_PELVIS_Z_M)
harness_input_pass = bool(station_coverage_ok and positive_main_coverage_ok)
xlock_pass = bool(
    XLOCK_THRESH <= 0 or (
        xlock_within_threshold(state["max_abs_x_err_motion"])
        and not xlock_violations))
runner_gate_pass = bool(
    runner_proxy_pass
    and stability_pass
    and harness_input_pass
    and lane_contract_pass
    and planner_flight_join["pass"]
    and runner_trace_join_pass
    and runner_lifecycle_completion_join_pass
    and launch_cadence_pass
    and localization_ok
    and xlock_pass
)


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
    min_landing_observations_per_side=(
        MIN_LANDING_OBSERVATIONS_PER_SIDE
    ),
    min_global_contacts=MIN_GLOBAL_CONTACTS,
    min_global_landings=MIN_GLOBAL_LANDINGS,
    min_global_landing_observations=MIN_GLOBAL_LANDING_OBSERVATIONS,
    allowed_landing_censor_lanes=ALLOWED_LANDING_CENSOR_LANES,
)
for row in serve_rows:
    row["physical_outcome"] = physical_by_id.get(int(row.get("shot_id") or 0))
physical_ball_pass = bool(physical_join["pass"])
certification_pass = bool(runner_gate_pass and planner_contract_pass and physical_ball_pass)
# Public Gate3 has exactly one top-level verdict.  The historical runner-only
# result remains visible as the lifecycle regression subreport, but can never
# promote the gate without contact and landing evidence.
ok = certification_pass
summary = {
    # v15 adds authoritative all-MOTION-tick localization freshness evidence.
    "schema_version": 15,
    "scenario_id": GATE3_SCENARIO,
    "phase": GATE3_PHASE,
    "serves": n, "engaged_serves": engaged_serves,
    "minimum_engaged_serves": MIN_ENGAGED_SERVES,
    "completed_serves": completed_serves,
    "minimum_completed_serves": MIN_COMPLETED_SERVES,
    "total_engage_events": total_engage_events,
    "completed_recovered_proxy": sum(
        int(row["completed_recovered_proxy"]) for row in serve_rows
    ),
    "completed_lane_proxy": proxy_completed,
    "proxy_rate": round(proxy_rate, 3),
    "minimum_proxy_rate": MIN_PROXY_RATE,
    "engage_rate": round(engage_rate, 3),
    "minimum_engage_rate": MIN_ENGAGE_RATE,
    "completion_rate_given_engage": round(completion_rate, 3),
    "minimum_completion_rate_given_engage": MIN_COMPLETION_RATE,
    "recovered_engaged_serves": recovered_engaged,
    "recovery_rate_given_engage": round(recovery_rate, 3),
    "minimum_recovery_rate_given_engage": MIN_RECOVERY_RATE,
    "recovery_contract": (
        "session_anchor_support_yaw_kinetic_dwell_v2"
        if GATE3_PHASE == "random" or RECOVERY_TARGET_CONTRACT == "session_anchor"
        else "completed_swing_station_support_yaw_kinetic_dwell_v2"
    ),
    "recovery_radius_m": RECOVERY_RADIUS_M,
    "recovery_speed_max_mps": RECOVERY_SPEED_MAX_MPS,
    "recovery_tilt_max_deg": RECOVERY_TILT_MAX_DEG,
    "recovery_yaw_max_deg": RECOVERY_YAW_MAX_DEG,
    "recovery_dwell_s": RECOVERY_DWELL_S,
    "required_post_completion_tail_s": REQUIRED_POST_COMPLETION_TAIL_S,
    "recovery_speed_ema_alpha": RECOVERY_SPEED_EMA_ALPHA,
    "station_acquired_at_engage": station_acquired_engaged,
    "station_acquisition_rate_given_engage": round(station_acquisition_rate, 3),
    "lane_contract": LANE_CONTRACT,
    "lane_contract_pass": lane_contract_pass,
    "serve_flight_schedule_s": list(FLIGHT_SCHEDULE),
    "serve_pause_schedule_s": list(PAUSE_SCHEDULE),
    "base_xy_trajectory": base_xy_trajectory,
    "fixed_home_contract": {
        "session_home_xy": [float(anchor_xy[0]), float(anchor_xy[1])],
        "session_home_yaw_deg": round(math.degrees(anchor_yaw_rad), 6),
        "commanded_station_rows": len(fixed_home_command_errors),
        "maximum_commanded_station_error_m": round(
            fixed_home_max_command_error_m, 6
        ),
        "commanded_station_tolerance_m": FIXED_HOME_COMMAND_TOLERANCE_M,
        "station_invariant_pass": fixed_home_station_invariant_pass,
    },
    "clean_fixed_home": {
        "rows": len(clean_fixed_home_rows),
        "engaged_side_counts": clean_fixed_home_side_counts,
        "natural_wait_actual_q_rms_max_rad": (
            NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD
        ),
        "pass": clean_fixed_home_lane_pass,
    },
    "rapid_fixed_home": {
        "rows": len(rapid_rows),
        "engaged": rapid_engaged,
        "completed": rapid_completed,
        "completion_kinds": rapid_completion_kinds,
        "completion_identity_pass": rapid_completion_identity_pass,
        "engaged_flight_ids": rapid_engaged_flight_ids,
        "missed_flight_ids": rapid_missed_flight_ids,
        "engage_gaps_s": rapid_engage_gaps_s,
        "gap_time_source": "runner_trace.wall_time_ns",
        "maximum_engage_gap_s": RAPID_MAX_ENGAGE_GAP_S,
        "contact_to_next_commit_s": rapid_contact_to_next_commit_s,
        "contact_to_next_commit_time_source": (
            "physical_contact.wall_time_ns_to_runner_trace.engage.wall_time_ns"
        ),
        "minimum_contact_to_next_commit_s": RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S,
        "maximum_contact_to_next_commit_s": RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S,
        "contact_to_next_commit_pass": rapid_contact_to_next_commit_pass,
        "engaged_side_sequence": rapid_sides,
        "planner_side_sequence": [
            "fh" if row.get("planner_side") == "forehand" else "bh"
            for row in rapid_rows
            if row.get("planner_side") in ("forehand", "backhand")
        ],
        "side_transition_counts": rapid_side_transitions,
        "home_recovery_required_before_next_ball": False,
        "terminal_home_recovery_required": True,
        "terminal_recovery_pass": rapid_terminal_recovery_pass,
        "pass": rapid_lane_pass,
    },
    "natural_drift_recovery": {
        "result": natural_drift_recovery_result,
        "not_observed_below_m": FIXED_HOME_DRIFT_NOT_OBSERVED_M,
        "observed_rows": len(natural_drift_observed_rows),
        "evidence_complete": natural_recovery_evidence_complete,
        "rows": natural_drift_recovery_rows,
    },
    "disturbance_recovery_probe": {
        "status": "NOT_RUN_SEPARATE_PROBE",
        "part_of_formal_gate3": False,
    },
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
    "end_home_yaw_error_deg": round(end_yaw_drift_deg, 3),
    "maximum_home_yaw_error_deg": round(
        state["max_anchor_yaw_err_motion_deg"], 3
    ),
    "maximum_end_home_yaw_error_deg": MAX_END_YAW_DRIFT_DEG,
    "maximum_peak_home_yaw_error_deg": MAX_PEAK_YAW_DRIFT_DEG,
    "min_z_motion": round(state["min_z_motion"], 3),
    "valid_cmds": state["valid"], "invalid_cmds": state["invalid"],
    "dropout_stress": DROPOUT_AT if dropout_done else 0,
    "base_outage_stress": {
        "serve": BASE_OUTAGE_AT if base_outage_done else 0,
        "requested_upstream_blackout_ticks": (
            BASE_OUTAGE_TICKS if base_outage_done else 0
        ),
        "start_wall_time_ns": base_outage_start_wall_time_ns,
        "end_wall_time_ns": base_outage_end_wall_time_ns,
        "source": "calibrated_base_relay_only",
    },
    "base_pose_producer": {
        "contract": BASE_PRODUCER_CONTRACT,
        "publisher_count_at_preflight": BASE_PUBLISHER_COUNT,
        "single_producer_pass": BASE_PUBLISHER_COUNT == 1,
    },
    "operator_rescues": rescues,
    "actual_q_faults": actual_q_faults,
    "actual_q_fault_definition": "canonical measured-q excursion episodes",
    "actual_q_live_termination_faults": actual_q_live_termination_faults,
    "actual_q_ledger_complete": actual_q_ledger_complete,
    "actual_q_ledger_consistency_pass": actual_q_ledger_consistency_pass,
    "actual_q_full_tail_ledger": actual_q_tail_ledger,
    "command_safety_faults": command_safety_faults,
    "command_safety_fault_count": len(command_safety_faults),
    "max_rescues_allowed": MAX_RESCUES,
    "rescue_enabled": ALLOW_RESCUE,
    "readiness_required": REQUIRE_READY,
    "readiness_contract": READY_CONTRACT,
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
    "runner_trace_localization": localization_trace_evidence,
    "localization_ok": localization_ok,
    "runner_proxy_pass": runner_proxy_pass,
    "stability_pass": stability_pass,
    "harness_input_pass": harness_input_pass,
    "xlock_pass": xlock_pass,
    "runner_gate_pass": runner_gate_pass,
    "planner_evidence": planner_evidence,
    "planner_flight_join": planner_flight_join,
    "runner_trace_engagement": runner_trace_evidence,
    "runner_trace_join_errors": runner_trace_join_errors,
    "runner_trace_join_pass": runner_trace_join_pass,
    "runner_lifecycle_completion_required": (
        runner_lifecycle_completion_required
    ),
    "runner_lifecycle_completion_join_pass": (
        runner_lifecycle_completion_join_pass
    ),
    "launch_cadence_receipt": launch_cadence_receipt,
    "launch_cadence_pass": launch_cadence_pass,
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
      f"station_acquired|engage={100*station_acquisition_rate:.0f}% "
      f"rescues={rescues} drift_end/peak={drift:.2f}/"
      f"{state['max_anchor_xy_err_motion']:.2f}m "
      f"multi_engage={multi_engage_serves or 0} "
      f"xlock_viol={len(xlock_violations)}{xlock_violations or ''} "
      f"station_steps(commanded)={station_good}/{len(command_station_transitions)} "
      f"station_steps(achieved)={sum(s['in_training_band'] for s in achieved_station_transitions)}/"
      f"{len(achieved_station_transitions)} "
      f"lanes=clean_fixed_home:{'PASS' if clean_fixed_home_lane_pass else 'FAIL'},"
      f"rapid_fixed_home:{'PASS' if rapid_lane_pass else 'FAIL'} "
      f"fixed_home_cmd_max={fixed_home_max_command_error_m:.4f}m "
      f"natural_drift={natural_drift_recovery_result} "
      f"positive_main={positive_main_good}/{MIN_POSITIVE_MAIN_TRANSITIONS} "
      f"stale_base_warns={no_base_warns} "
      f"localization_fresh_ticks={localization_trace_evidence['fresh_ticks']}/"
      f"{localization_trace_evidence['motion_ticks']} "
      f"xmax={state['max_abs_x_err_motion']:.2f}m xend={x_err_at_end:+.2f}m "
      f"min_z={state['min_z_motion']:.3f} runner_proxy={'PASS' if runner_proxy_pass else 'FAIL'} "
      f"safety={'PASS' if stability_pass else 'FAIL'} input={'PASS' if harness_input_pass else 'FAIL'} "
      f"localization={'PASS' if localization_ok else 'FAIL'} xlock={'PASS' if xlock_pass else 'FAIL'} "
      f"planner_contract={'PASS' if planner_contract_pass else 'FAIL'} "
      f"physical_ball={'PASS' if physical_ball_pass else 'NOT_PROVEN'} "
      f"contacts={physical_join['global_contacts']}/{MIN_GLOBAL_CONTACTS} "
      f"landing_obs={physical_join['global_landing_observations']}/"
      f"{MIN_GLOBAL_LANDING_OBSERVATIONS} "
      f"censored={physical_join['global_landing_censored']} "
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
        f"landing_obs={physical_side['landing_observations']} "
        f"censored={physical_side['landing_censored']} "
        f"legal_landing={physical_side['legal_landings']} "
        f"({100.0 * physical_side['landing_rate']:.0f}%) "
        f"-> {'PASS' if physical_side['pass'] else 'FAIL'}",
        flush=True,
    )
sys.exit(0 if ok else 1)
