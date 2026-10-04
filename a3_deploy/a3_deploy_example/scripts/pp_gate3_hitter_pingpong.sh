#!/usr/bin/env bash
# Sole formal model_21800 Gate3 entry. Runtime geometry/control remains the
# build_1 rally_v14 runtime-v2 contract.
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$GEAR" || exit 2
export PP_GEAR="$GEAR"

while (($#)); do
  case "$1" in
    --preflight-only)
      export PP_GATE3_PREFLIGHT_ONLY=1
      ;;
    -h|--help)
      echo "Usage: scripts/pp_gate3_hitter_pingpong.sh [--preflight-only]"
      echo "  --preflight-only validate the selected environment without starting processes"
      exit 0
      ;;
    *)
      echo "[hitter-pingpong] unknown argument: $1" >&2
      exit 64
      ;;
  esac
  shift
done

HITTER_RUNNER_EXTRA_ARGS="${PP_EXTRA_ARGS:-}"
if [ -n "${PP_SCENE_EXPORTS:-}" ]; then
export PP_GATE3_PROFILE=hitter_pingpong_shared_plane_v1
export PP_GATE3_SCENARIO=hitter_pingpong_fixed_home_rapid_1150ms_v1
export PP_GATE3_PHASE=fixed_home
HITTER_GATE3_MODE="${PP_HITTER_GATE3_MODE:-certification}"
case "$HITTER_GATE3_MODE" in
  certification) ;;
  torture)
    export PP_GATE3_SCENARIO=hitter_pingpong_low_corner_torture_v1
    export PP_GATE3_PHASE=torture
    ;;
  *)
    echo "[hitter-pingpong] PP_HITTER_GATE3_MODE must be certification or torture"
    exit 2
    ;;
esac
# Gate3 measures policy capability.  The production L2/L3 commit supervisor is
# intentionally disabled here so it cannot turn an OOD policy failure into a
# cosmetically clean result.  Refuse a contradictory caller override.
if [ -n "${PP_ENVELOPE_MODE:-}" ] && [ "$PP_ENVELOPE_MODE" != "capability" ]; then
  echo "[hitter-pingpong] Gate3 requires PP_ENVELOPE_MODE=capability; got $PP_ENVELOPE_MODE"
  exit 2
fi
export PP_ENVELOPE_MODE=capability
export PP_SERVES="${PP_SERVES:-26}"
export PP_XHIT=0.08
export PP_XHIT_BH_DELTA=0.0
export PP_FIXED_PLANE_X=0.58
export PP_STATION_X=-0.50
export PP_XLOCK_THRESH=0.05
export PP_READY_X_MAX=0.10
export PP_READY_Y_MAX=0.10
export PP_READY_SPEED_MAX=0.20
export PP_SPLIT_Y=-0.25 PP_SPLIT_HYST=0.04
export PP_SWING_SIDE_REFERENCE_MODE=session_home_v1
export PP_LAND_X=2.055 PP_LAND_Y_FH=-0.7625 PP_LAND_Y_BH=-0.7625
export PP_DTF_FH=0.50 PP_DTF_BH=0.50
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pp_gate3_physical_common.sh"
if [ "$HITTER_GATE3_MODE" = "torture" ]; then
  gate3_apply_hitter_low_corner_torture_v1_contract || exit $?
  # Fall-stress runs retain all lifecycle and plant checks, but physical
  # contact is an 80%-coverage diagnostic rather than the reason to suppress a
  # hard low/corner question. Every ball retains its complete physical flight.
  export PP_MIN_GLOBAL_CONTACTS=$(( (4 * PP_SERVES + 4) / 5 ))
  export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=$(( PP_SERVES / 2 ))
  export PP_MIN_CONTACTS_PER_SIDE=$(( (2 * PP_SERVES + 4) / 5 ))
  export PP_MIN_RAPID_ENGAGED=$(( PP_SERVES - PP_CLEAN_FIXED_HOME_SERVES ))
  export PP_MIN_RAPID_COMPLETED="$PP_MIN_RAPID_ENGAGED"
  export PP_OPTIONAL_REACH_EXPECTED_FLIGHTS="$PP_SERVES"
else
  gate3_apply_hitter_fixed_home_rapid_1150ms_v1_contract || exit $?
  export PP_MIN_GLOBAL_CONTACTS=22
  # Incoming cadence and outgoing-flight lifetime are independent.
  export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=13
  export PP_MIN_CONTACTS_PER_SIDE=11
  export PP_MIN_RAPID_ENGAGED=20
  export PP_MIN_RAPID_COMPLETED=20
  if [ "$PP_SERVES" -ne 26 ]; then
    export PP_MIN_GLOBAL_CONTACTS=$(( (5 * PP_SERVES + 5) / 6 ))
    export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=$(( PP_SERVES / 2 - 1 ))
    export PP_MIN_CONTACTS_PER_SIDE=$(( (5 * (PP_SERVES / 2 - 1) + 5) / 6 ))
    export PP_MIN_RAPID_ENGAGED=$(( PP_SERVES - 6 ))
    export PP_MIN_RAPID_COMPLETED="$PP_MIN_RAPID_ENGAGED"
  fi
fi
export PP_MIN_PHYSICAL_CONTACT_RATE=0.8333333333333334
export PP_MIN_LEGAL_LANDING_RATE=0.8333333333333334
export PP_ALLOWED_LANDING_CENSOR_LANES=""
export PP_MIN_GLOBAL_LANDING_OBSERVATIONS="$PP_MIN_GLOBAL_CONTACTS"
export PP_MIN_GLOBAL_LANDINGS="$PP_MIN_GLOBAL_CONTACTS"
export PP_MIN_LANDING_OBSERVATIONS_PER_SIDE="$PP_MIN_CONTACTS_PER_SIDE"
export PP_MIN_LANDINGS_PER_SIDE="$PP_MIN_CONTACTS_PER_SIDE"
export PP_MIN_POST_CONTACT_LANDING_OBSERVATION_S=0.75
export PP_MIN_STATION_TRANSITIONS=0
export PP_MIN_POSITIVE_MAIN_TRANSITIONS=0
export PP_FIXED_HOME_COMMAND_TOLERANCE_M=0.0015
export PP_FIXED_HOME_DRIFT_NOT_OBSERVED_M=0.01
export PP_RAPID_MAX_ENGAGE_GAP_S=1.40
export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S=0.40
export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S=0.55
export PP_NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S=0.12
export PP_NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD=0.20
# Qualification is one no-reset phase (26 shots by default): every serve must engage
# and complete.  Keep this aligned with pp_rally_conductor.py instead of
# inheriting the obsolete 0.8 proxy default from the older V14 wrapper.
export PP_MIN_PROXY_RATE=1.0
export PP_MIN_ENGAGED_SERVES="$PP_SERVES"
export PP_MIN_COMPLETED_SERVES="$PP_SERVES"
export PP_ALLOW_RESCUE=0
export PP_MAX_RESCUES=0
export PP_REQUIRE_READY=1
# The hardware Planner reveals and commits the final target in one event. It reports station
# error/speed at release but does not emit the obsolete pre-release READY token.
# This scenario only requires the common late-reveal commit telemetry.  Whether
# a policy can admit the next flight during recovery is measured by the rapid
# lane itself, not selected here through a recipe-specific readiness label.
export PP_READY_CONTRACT=late_reveal_commit_v1
export PP_RECOVERY_TARGET_CONTRACT=session_anchor
export PP_RECOVERY_RADIUS_M=0.05
export PP_RECOVERY_SPEED_MAX_MPS=0.12
export PP_RECOVERY_DWELL_S=0.30
export PP_REQUIRE_PLANT_TRACE=1
export PP_REQUIRE_IDLE_SMOOTHNESS=1
export PP_REQUIRE_HOME_SUPPORT_FRAME=1
export PP_REQUIRE_TORSO_HOLD=1
export PP_REQUIRE_OPTIONAL_REACH="${PP_REQUIRE_OPTIONAL_REACH:-auto}"
export PP_REQUIRE_COLLISION_LEDGER=1
export PP_TORSO_HOLD_MAX_PITCH_MEDIAN_DEG=3.0
export PP_TORSO_HOLD_MAX_PITCH_P95_DEG=5.0
export PP_TORSO_HOLD_MAX_PITCH_PEAK_DEG=8.0
export PP_HOME_MAX_SUPPORT_MIDPOINT_END_M=0.03
export PP_HOME_MAX_FOOT_ANCHOR_END_M=0.03
export PP_HOME_MIN_SIGNED_WIDTH_M=0.26
export PP_HOME_MAX_SIGNED_WIDTH_M=0.31
export PP_RECOVERY_YAW_MAX_DEG="${PP_RECOVERY_YAW_MAX_DEG:-5.0}"
export PP_MAX_END_YAW_DRIFT_DEG="${PP_MAX_END_YAW_DRIFT_DEG:-5.0}"
export PP_MAX_PEAK_YAW_DRIFT_DEG="${PP_MAX_PEAK_YAW_DRIFT_DEG:-12.0}"
export PP_MOTION_IDLE_S="${PP_MOTION_IDLE_S:-20.0}"
export PP_MIN_MOTION_IDLE_S="${PP_MIN_MOTION_IDLE_S:-15.0}"
# Keep the final serve in MOTION long enough to observe the complete native
# post-contact recovery tail selected later by the recipe-aware report.
export PP_REQUIRED_POST_COMPLETION_TAIL_S=3.0
# Measured burst/dropout robustness is staged on a separate axis and must not
# be silently mixed into the fixed-HOME qualification.  Nominal Gate3 remains
# outage-free, while an explicitly selected serve enables the already isolated
# calibrated-base-relay stress lane.
export PP_BASE_OUTAGE_AT="${PP_BASE_OUTAGE_AT:-0}"
export PP_BASE_OUTAGE_TICKS="${PP_BASE_OUTAGE_TICKS:-15}"
# Metric windows are selected from the validated model recipe, independently
# from the immutable fixed-HOME launch scenario above.
export PP_RALLY_REPORT_MODE=auto
export PP_PLANNER_EVIDENCE_JSON=/tmp/pp_planner_cpp_report.json
export PP_PHYSICAL_EVIDENCE_JSON=/tmp/pp_physical_ball_report.json
export PP_GATE3_VERDICT=certification
# Gate3 is an x86 policy-behavior audit.  It adds no supervisor clamp beyond the
# action contract declared by the artifact: V11 remains affine+safe-clamp and
# Schema32/V12 keeps its deterministic slew layer active. This Gate3 capability
# probe also commits every fresh/finite/identity-valid positive-TTS flight: the
# Runner may not hide a fall by dropping a rapid command at a temporal support
# boundary. Audit counters never alter actor q_des.
export PP_EXTRA_ARGS="${HITTER_RUNNER_EXTRA_ARGS:+$HITTER_RUNNER_EXTRA_ARGS }--gate3-qdes-audit-only --gate3-force-every-flight"

  source "$PP_SCENE_EXPORTS" || exit 2
else
export PP_GATE3_PROFILE=rally_v14
export PP_GATE3_PHASE=qualification
export PP_SERVES=12
export PP_XHIT=0.08
export PP_XHIT_BH_DELTA=0.0
export PP_FIXED_PLANE_X=0.58
export PP_STATION_X=-0.50
export PP_XLOCK_THRESH=0.05
export PP_READY_X_MAX=0.10
export PP_READY_Y_MAX=0.10
export PP_READY_SPEED_MAX=0.20
export PP_SPLIT_Y=-0.25 PP_SPLIT_HYST=0.04
export PP_LAND_X=2.055 PP_LAND_Y_FH=-0.7625 PP_LAND_Y_BH=-0.7625
export PP_DTF_FH=0.50 PP_DTF_BH=0.50
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pp_gate3_physical_common.sh"
gate3_apply_physical_arena_contract
export PP_MIN_GLOBAL_CONTACTS=11
export PP_MIN_GLOBAL_LANDINGS=10
export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=6
export PP_MIN_CONTACTS_PER_SIDE=5
export PP_MIN_LANDINGS_PER_SIDE=5
export PP_MIN_PHYSICAL_CONTACT_RATE=0.8333333333333334
export PP_MIN_LEGAL_LANDING_RATE=0.8333333333333334
export PP_STATION_STEP_LO=0.12
export PP_STATION_STEP_HI=0.35
export PP_MIN_STATION_TRANSITIONS=4
export PP_POSITIVE_MAIN_LO=0.19
export PP_POSITIVE_MAIN_HI=0.24
export PP_MIN_POSITIVE_MAIN_TRANSITIONS=2
# Qualification is the strict 12-shot phase: every physical serve must engage
# and complete.  Keep this aligned with pp_rally_conductor.py instead of
# inheriting the obsolete 0.8 proxy default from the older V14 wrapper.
export PP_MIN_PROXY_RATE=1.0
export PP_MIN_ENGAGED_SERVES="$PP_SERVES"
export PP_MIN_COMPLETED_SERVES="$PP_SERVES"
export PP_ALLOW_RESCUE=0
export PP_MAX_RESCUES=0
export PP_REQUIRE_READY=1
export PP_REQUIRE_PLANT_TRACE=1
export PP_REQUIRE_IDLE_SMOOTHNESS=1
export PP_MOTION_IDLE_S="${PP_MOTION_IDLE_S:-20.0}"
export PP_MIN_MOTION_IDLE_S="${PP_MIN_MOTION_IDLE_S:-15.0}"
export PP_RALLY_REPORT_MODE=rally_v14
export PP_PLANNER_EVIDENCE_JSON=/tmp/pp_planner_envelope_report.json
export PP_PHYSICAL_EVIDENCE_JSON=/tmp/pp_physical_ball_report.json
export PP_GATE3_VERDICT=certification
# Gate3 is an x86 policy-behavior audit.  Let all finite actor commands reach
# MuJoCo unchanged and record safe/hard-limit exceedances instead of stopping
# the runner at the first one.  The final report may still mark them NO-GO.
export PP_EXTRA_ARGS="${PP_EXTRA_ARGS:+$PP_EXTRA_ARGS }--gate3-qdes-audit-only"

fi

PUBLIC_PACKAGE_DIST="$GEAR/../../agibot/code_deployment/a3_deploy_example/dist/a3_deploy_x86_64"
if [ -z "${PP_DIST:-}" ]; then
  if [ -x "$GEAR/dist/a3_deploy_x86_64/run_a3_pingpong.sh" ]; then
    PP_DIST="$GEAR/dist/a3_deploy_x86_64"
  else
    PP_DIST="$PUBLIC_PACKAGE_DIST"
  fi
fi
export PP_DIST
export A3_PINGPONG_RUNTIME_CFG="${A3_PINGPONG_RUNTIME_CFG:-$PP_DIST/config/a3_runtime_config.pingpong.hitter_pingpong.yaml}"
if [ ! -f "$A3_PINGPONG_RUNTIME_CFG" ]; then
  echo "[hitter-pingpong] runtime config missing: $A3_PINGPONG_RUNTIME_CFG"
  echo "[hitter-pingpong] build the public x86 package documented in docs/MODEL_21800.md"
  exit 2
fi
export PP_SIM_INSTALL="${PP_SIM_INSTALL:-$GEAR/../A3_MuJoCo_Sim/aimrt_mujoco_sim/cmake-build-model21800-gate3/install}"
if [ ! -x "$PP_SIM_INSTALL/bin/aimrt_main" ]; then
  echo "[hitter-pingpong] instrumented MuJoCo install missing: $PP_SIM_INSTALL"
  exit 2
fi
rm -f "$PP_PLANNER_EVIDENCE_JSON" "$PP_PHYSICAL_EVIDENCE_JSON"
if [ -z "${PP_SCENE_EXPORTS:-}" ]; then
python3 "$SCRIPT_DIR/pp_planner_envelope_audit.py" \
  --gate3-script "$SCRIPT_DIR/pp_gate3_hitter_pingpong.sh" --serves-list "$PP_SERVES_LIST" \
  --contract rally_v14 --verdict planner_contract \
  --json-out "$PP_PLANNER_EVIDENCE_JSON" || {
  echo "[hitter-pingpong] planner envelope preflight failed"; exit 2;
}

fi

# Let the supervisor wait for flight drain and recorder cleanup on signals.
exec bash "$SCRIPT_DIR/pp_gate3_rally.sh"
