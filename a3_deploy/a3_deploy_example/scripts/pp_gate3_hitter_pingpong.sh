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
python3 "$SCRIPT_DIR/pp_planner_envelope_audit.py" \
  --gate3-script "$SCRIPT_DIR/pp_gate3_hitter_pingpong.sh" --serves-list "$PP_SERVES_LIST" \
  --contract rally_v14 --verdict planner_contract \
  --json-out "$PP_PLANNER_EVIDENCE_JSON" || {
  echo "[hitter-pingpong] planner envelope preflight failed"; exit 2;
}

bash "$SCRIPT_DIR/pp_gate3_rally.sh"
rc=$?
echo "G3_HITTER_PINGPONG_RC=$rc"
exit "$rc"
