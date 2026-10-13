#!/usr/bin/env bash
# Non-certifying, x86-only MuJoCo replay for the exact V17 recipe-v1
# model_16600 ONNX. A configurable number of identical side-neutral balls
# select the same backhand arm-reach lane; the runner rejects any requested
# station step.
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HOPE_ROOT="$(cd -- "$GEAR/../.." && pwd)"
DIST="${PP_STATIONARY_DIST:-$GEAR/dist/a3_deploy_x86_64_stationary_replay}"
WS="$HOPE_ROOT/hope_ws"
SIM_INSTALL="${PP_SIM_INSTALL:-$HOPE_ROOT/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/build_stationary_ballfix_jazzy/install}"
RUNNER="$DIST/a3_deploy_onnx_ref_pingpong_stationary_replay"
RUNTIME_CFG="$GEAR/src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.v17_r1_stationary_replay.yaml"
AIMRT_CFG="$DIST/config/a3_aimrt_config.pingpong_ros2body.yaml"
MODEL="$GEAR/assets/a3_runtime/models/model_v17_candidate.onnx"
EXPECTED_MODEL_SHA="019fc86c8d045d95ac0054054e0fc91b845ce712feadfeb450f9a04b04556eaa"
EXPECTED_SIM_PLUGIN_SHA="86764f2e21ebae7e9eef5728241bc4bcee0b71d310b810cfe0e10212de5d3d12"
EXPECTED_SIM_XML_SHA="4887b5301915830a298bee781c897da9448d45cf41ac91da6ce7440a7c3bbd22"
SIM_PLUGIN="$SIM_INSTALL/lib/libmujoco_sim_pkg.so"
SIM_XML="$SIM_INSTALL/bin/cfg/model/a3_pingpong/a3_pingpong.xml"
STAMP="$(date +%Y%m%d_%H%M%S)"
EXPECTED_SHOTS="${PP_STATIONARY_EXPECTED_SHOTS:-3}"
ARTIFACT_DIR="${PP_STATIONARY_ARTIFACT_DIR:-$HOPE_ROOT/artifacts/v17_r1_stationary_${EXPECTED_SHOTS}ball_${STAMP}}"
TMP_PREFIX="/tmp/pp_v17r1_stationary_${$}"

RUNNER_LOG="${TMP_PREFIX}_runner.log"
BALL_LOG="${TMP_PREFIX}_ball.log"
PHYSICAL_JSON="${TMP_PREFIX}_physical.json"
SESSION_JSON="${TMP_PREFIX}_session.json"
OBS_CSV="${TMP_PREFIX}_obs.csv"
TRACE_CSV="${TMP_PREFIX}_trace.csv"
PLANT_CSV="${TMP_PREFIX}_plant.csv"
PLANT_JSON="${TMP_PREFIX}_plant.json"
FINAL_JSON="${TMP_PREFIX}_report.json"

SIM_LOG="${TMP_PREFIX}_sim.log"
RAW_MOCAP_LOG="${TMP_PREFIX}_raw_mocap.log"
MOCAP_RELAY_LOG="${TMP_PREFIX}_mocap_relay.log"
BASE_RELAY_LOG="${TMP_PREFIX}_base_relay.log"
PLANNER_LOG="${TMP_PREFIX}_planner.log"
CONTACT_LOG="${TMP_PREFIX}_contact.log"
RESET_LOG="${TMP_PREFIX}_reset.log"
ROUDI_LOG="${TMP_PREFIX}_roudi.log"
CONDUCTOR_LOG="${TMP_PREFIX}_conductor.log"
PLANT_REPORT_LOG="${TMP_PREFIX}_plant_report.log"
RALLY_REPORT_LOG="${TMP_PREFIX}_rally_report.log"
FINAL_REPORT_LOG="${TMP_PREFIX}_final_report.log"
PLANNER_DEBUG_CSV="${TMP_PREFIX}_planner_debug.csv"

FLIGHT_S="${PP_STATIONARY_FLIGHT_S:-2.5}"
PAUSE_S="${PP_STATIONARY_PAUSE_S:-4.0}"
MOTION_IDLE_S="${PP_STATIONARY_MOTION_IDLE_S:-3.0}"
VERDICT_MODE="${PP_STATIONARY_VERDICT_MODE:-replay}"
RECOVERY_MODE="${PP_STATIONARY_RECOVERY_MODE:-fixed}"
CONTACT_AB_JSON="${PP_STATIONARY_CONTACT_AB_JSON:-}"
# Old Gate3 serve 2: it selected backhand while preserving station
# (-0.50,-0.7625). Repeating one side-neutral physical state is the deliberate
# cherry-pick; no side label or runner side override exists.
SERVE_TUPLE='2.4,-0.8525,0.49,-3.0,0.0,2.2'
MOVING_SERVE_A_BH='2.6,-0.8525,0.65,-3.0,0.0,2.2'
MOVING_SERVE_B_BH='2.6,-0.6925,0.65,-3.0,0.0,2.2'
MOVING_SERVE_A_FH='2.6,-1.2425,0.65,-3.0,0.0,2.2'
SERVES_LIST=""

PIDS=()
ROUDI_STARTED=""
CONTACT_AB_REPORT_ARGS=()

fail() {
  echo "[stationary-3ball] PRECHECK FAIL: $*" >&2
  exit 2
}

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  if [[ -n "$ROUDI_STARTED" ]] && kill -0 "$ROUDI_STARTED" 2>/dev/null; then
    kill -TERM "$ROUDI_STARTED" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

[[ "$(uname -m)" == "x86_64" ]] ||
  fail "this replay target is x86_64-only and is intentionally absent on MDU"
[[ -x "$RUNNER" ]] || fail "isolated replay binary missing: $RUNNER"
[[ -f "$RUNTIME_CFG" ]] || fail "runtime config missing: $RUNTIME_CFG"
[[ -f "$AIMRT_CFG" ]] || fail "planner AimRT config missing: $AIMRT_CFG"
[[ -f "$MODEL" ]] || fail "legacy ONNX missing: $MODEL"
[[ -x "$SIM_INSTALL/bin/aimrt_main" ]] ||
  fail "instrumented MuJoCo install missing: $SIM_INSTALL"
[[ -f "$SIM_PLUGIN" ]] || fail "MuJoCo plugin missing: $SIM_PLUGIN"
[[ -f "$SIM_XML" ]] || fail "MuJoCo robot XML missing: $SIM_XML"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS Jazzy is unavailable"
[[ -r "$WS/install/local_setup.bash" ]] ||
  fail "HOPE ROS workspace is not built: $WS/install/local_setup.bash"
[[ "$EXPECTED_SHOTS" =~ ^[1-9][0-9]*$ ]] ||
  fail "PP_STATIONARY_EXPECTED_SHOTS must be an integer in [1,100]"
((EXPECTED_SHOTS <= 100)) ||
  fail "PP_STATIONARY_EXPECTED_SHOTS must be an integer in [1,100]"
[[ "$VERDICT_MODE" == "replay" || "$VERDICT_MODE" == "stability" ||
   "$VERDICT_MODE" == "lateral" || "$VERDICT_MODE" == "homing" ||
   "$VERDICT_MODE" == "moving" ]] ||
  fail "PP_STATIONARY_VERDICT_MODE must be replay, stability, lateral, homing, or moving"
[[ "$RECOVERY_MODE" == "fixed" || "$RECOVERY_MODE" == "fixed_homing" ||
   "$RECOVERY_MODE" == "moving_recovery" ]] ||
  fail "PP_STATIONARY_RECOVERY_MODE must be fixed, fixed_homing, or moving_recovery"
SERVES_LIST="["
for ((serve_index = 0; serve_index < EXPECTED_SHOTS; serve_index++)); do
  [[ "$serve_index" -gt 0 ]] && SERVES_LIST+=","
  if [[ "$RECOVERY_MODE" == "fixed" || "$RECOVERY_MODE" == "fixed_homing" ]]; then
    SERVES_LIST+="$SERVE_TUPLE"
  elif [[ "$serve_index" -eq 0 ]]; then
    SERVES_LIST+="$MOVING_SERVE_A_BH"
  elif ((serve_index % 2 == 1)); then
    SERVES_LIST+="$MOVING_SERVE_B_BH"
  else
    SERVES_LIST+="$MOVING_SERVE_A_FH"
  fi
done
SERVES_LIST+="]"
[[ -n "${DISPLAY:-}" ]] ||
  fail "DISPLAY is unset; enter the container with desktop forwarding or pass DISPLAY/XAUTHORITY"
if command -v xdpyinfo >/dev/null 2>&1; then
  timeout 3 xdpyinfo >/dev/null 2>&1 ||
    fail "DISPLAY='$DISPLAY' is not reachable; MuJoCo GLFW cannot start"
fi

ACTUAL_MODEL_SHA="$(sha256sum "$MODEL" | awk '{print $1}')"
[[ "$ACTUAL_MODEL_SHA" == "$EXPECTED_MODEL_SHA" ]] ||
  fail "legacy ONNX SHA mismatch: $ACTUAL_MODEL_SHA"
ACTUAL_SIM_PLUGIN_SHA="$(sha256sum "$SIM_PLUGIN" | awk '{print $1}')"
[[ "$ACTUAL_SIM_PLUGIN_SHA" == "$EXPECTED_SIM_PLUGIN_SHA" ]] ||
  fail "isolated MuJoCo plugin SHA mismatch: $ACTUAL_SIM_PLUGIN_SHA"
ACTUAL_SIM_XML_SHA="$(sha256sum "$SIM_XML" | awk '{print $1}')"
[[ "$ACTUAL_SIM_XML_SHA" == "$EXPECTED_SIM_XML_SHA" ]] ||
  fail "isolated MuJoCo XML SHA mismatch: $ACTUAL_SIM_XML_SHA"
if [[ -n "$CONTACT_AB_JSON" ]]; then
  [[ -f "$CONTACT_AB_JSON" ]] ||
    fail "scripted contact A/B evidence missing: $CONTACT_AB_JSON"
  python3 - "$CONTACT_AB_JSON" <<'PY' ||
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
if report.get("schema") != "pp-scripted-racket-contact-ab-v1":
    raise SystemExit("unexpected scripted contact A/B schema")
if report.get("explicit_contact_energy_pass") is not True:
    raise SystemExit("scripted contact A/B explicit energy gate did not pass")
PY
    fail "scripted contact A/B evidence did not pass"
  CONTACT_AB_REPORT_ARGS=(--contact-ab "$CONTACT_AB_JSON")
fi

for pattern in \
  '[a]imrt_main.*a3_pingpong' \
  '[a]3_deploy_onnx_ref_pingpong' \
  '[h]ope_planner_cpp_node' \
  '[p]p_gate3_sim_mocap.py' \
  '[p]p_gate3_ball_launcher.py' \
  '[p]p_gate3_ball_evidence.py'; do
  if pgrep -af "$pattern" >/dev/null; then
    pgrep -af "$pattern" >&2
    fail "another ping-pong replay/sim process is active; not killing it"
  fi
done

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash"
# shellcheck disable=SC1090
source "$WS/install/local_setup.bash"
set -u
export PYTHONPATH="$GEAR/scripts:$SIM_INSTALL/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export AMENT_PREFIX_PATH="$SIM_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DIST/config/fastrtps_profile.xml"

python3 -c \
  'import onnxruntime, rclpy; from mujoco_sim_msgs.msg import Gate3BallCommand, Gate3BallState, SimReset; from motion_capture_tracking_interfaces.msg import NamedPoseArray' ||
  fail "required Python runtime modules or ROS messages are not importable"

mkdir -p "$ARTIFACT_DIR"
source "$SCRIPT_DIR/pp_ball_physics.sh"
pp_load_ball_physics "$WS/src/hope_planner_cpp/config/model21800_hardware.yaml" "" \
  "$ARTIFACT_DIR/ball_physics.json" || exit 2
ROUDI_BEFORE="$(pgrep -x iox-roudi | head -n 1 || true)"

echo "[stationary-${EXPECTED_SHOTS}ball] sim up; model_sha=$ACTUAL_MODEL_SHA sim_plugin_sha=$ACTUAL_SIM_PLUGIN_SHA"
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  MUJOCO_GL=egl \
  A3_SIM_INSTALL='$SIM_INSTALL' \
  A3_IOX_ROUDI_LOG='$ROUDI_LOG' \
  A3_MUJOCO_DEBUG_CSV='$PLANT_CSV' \
  A3_MUJOCO_DEBUG_STRIDE=5 \
  A3_MUJOCO_PD_MODE=explicit \
  A3_GATE3_BALL_DRAG_K=$PP_BALL_DRAG_K \
  A3_GATE3_BALL_RESTITUTION_H=$PP_BALL_RESTITUTION_H \
  A3_GATE3_BALL_RESTITUTION_V=$PP_BALL_RESTITUTION_V \
  A3_SIM_FLAVOR=source A3_SIM_CFG=a3_pingpong_iceoryx_cfg.yaml \
  '$GEAR/scripts/run_sim.sh'" >"$SIM_LOG" 2>&1 &
SIM_PID="$!"
PIDS+=("$SIM_PID")
for _ in $(seq 1 40); do
  grep -qi "could not initialize GLFW" "$SIM_LOG" 2>/dev/null &&
    fail "MuJoCo GLFW initialization failed; see $SIM_LOG"
  kill -0 "$SIM_PID" 2>/dev/null ||
    fail "MuJoCo exited during startup; see $SIM_LOG"
  grep -qiE "will wait for shutdown|Sim Start|GuiLoop" "$SIM_LOG" 2>/dev/null && break
  sleep 1
done
grep -qiE "will wait for shutdown|Sim Start|GuiLoop" "$SIM_LOG" 2>/dev/null ||
  fail "MuJoCo failed to start; see $SIM_LOG"
sleep 2
grep -qi "could not initialize GLFW" "$SIM_LOG" 2>/dev/null &&
  fail "MuJoCo GLFW initialization failed after core startup; see $SIM_LOG"
kill -0 "$SIM_PID" 2>/dev/null ||
  fail "MuJoCo exited after core startup; see $SIM_LOG"
if [[ -z "$ROUDI_BEFORE" ]]; then
  ROUDI_STARTED="$(pgrep -x iox-roudi | head -n 1 || true)"
fi

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null
  source '$WS/install/local_setup.bash' 2>/dev/null
  exec python3 '$SCRIPT_DIR/pp_gate3_sim_mocap.py' --table-height-m 0.760" \
  >"$RAW_MOCAP_LOG" 2>&1 &
PIDS+=("$!")

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  source '$WS/install/local_setup.bash' 2>/dev/null
  exec ros2 run hope_bringup optitrack_mct_relay --ros-args \
    --params-file '$WS/src/hope_bringup/config/optitrack_relay.yaml' \
    -p publish_tf:=false" >"$MOCAP_RELAY_LOG" 2>&1 &
PIDS+=("$!")

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  source '$WS/install/local_setup.bash' 2>/dev/null
  exec '$WS/install/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node' --ros-args \
    --params-file '$WS/src/hope_planner_cpp/config/model21800_hardware.yaml' \
    -p base_pose_flat_input_topic:='' -p robot_pose_topic:=/UCB_P1/pose \
    -p publish_base_flat:=true \
    -p policy_z_offset:=0.760 -p fit_window:=26 \
    -p drag_k:=$PP_BALL_DRAG_K -p restitution_h:=$PP_BALL_RESTITUTION_H -p restitution_v:=$PP_BALL_RESTITUTION_V \
    -p post_net_future_bounce_tangential_gain:=$PP_BALL_FUTURE_BOUNCE_GAIN \
  -p post_net_future_bounce_restitution_v:=$PP_BALL_FUTURE_BOUNCE_RESTITUTION_V \
    -p x_hit_follow_robot:=false -p x_hit:=0.08 -p x_hit_bh_delta:=0.0 \
    -p swing_side_split_y:=-0.25 -p swing_side_hysteresis_y:=0.04 \
    -p target_land_x:=2.055 -p target_land_y_fh:=-0.7625 \
    -p target_land_y_bh:=-0.7625 -p delta_t_flight_fh:=0.50 \
    -p delta_t_flight_bh:=0.50 -p max_predict_time:=2.6 \
    -p solve_period_s:=0.033 -p debug_csv_path:='$PLANNER_DEBUG_CSV' \
    -p debug_session_id:=v17_r1_${RECOVERY_MODE}_${EXPECTED_SHOTS}ball_noncert" \
  >"$PLANNER_LOG" 2>&1 &
PIDS+=("$!")

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null
  PP_SERVES='$EXPECTED_SHOTS' PP_PHYSICAL_EVIDENCE_JSON='$PHYSICAL_JSON' \
  exec python3 '$SCRIPT_DIR/pp_gate3_ball_evidence.py'" \
  >"$CONTACT_LOG" 2>&1 &
PIDS+=("$!")

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null
  PP_SERVES_LIST='$SERVES_LIST' PP_SERVES='$EXPECTED_SHOTS' \
  PP_FLIGHT_S='$FLIGHT_S' PP_PAUSE_S='$PAUSE_S' \
  PP_MOTION_IDLE_S='$MOTION_IDLE_S' \
  exec python3 '$SCRIPT_DIR/pp_gate3_ball_launcher.py' \
    --wait-log '$RUNNER_LOG' \
    --wait-marker 'action=ENTER_MOTION result=APPLIED reason=MODE_CHANGED mode=MOTION'" \
  >"$BALL_LOG" 2>&1 &
PIDS+=("$!")

sleep 4
timeout 12 "$GEAR/scripts/reset_sim.sh" >"$RESET_LOG" 2>&1 ||
  fail "MuJoCo reset did not complete within 12 s; see $RESET_LOG"

echo "[stationary-${EXPECTED_SHOTS}ball] conductor: stand -> MOTION once -> ${EXPECTED_SHOTS} balls mode=${RECOVERY_MODE}"
set +e
PP_REPLAY_EXPECTED_SHOTS="$EXPECTED_SHOTS" \
PP_REPLAY_RECOVERY_MODE="$RECOVERY_MODE" \
PP_REPLAY_DIST="$DIST" \
PP_REPLAY_RUNNER="$RUNNER" \
PP_REPLAY_RUNTIME_CFG="$RUNTIME_CFG" \
PP_REPLAY_AIMRT_CFG="$AIMRT_CFG" \
PP_REPLAY_RUNNER_LOG="$RUNNER_LOG" \
PP_REPLAY_BALL_LOG="$BALL_LOG" \
PP_REPLAY_PHYSICAL_JSON="$PHYSICAL_JSON" \
PP_REPLAY_SESSION_JSON="$SESSION_JSON" \
PP_REPLAY_OBS_CSV="$OBS_CSV" \
PP_REPLAY_TRACE_CSV="$TRACE_CSV" \
python3 "$SCRIPT_DIR/pp_stationary_3ball_conductor.py" \
  2>&1 | tee "$CONDUCTOR_LOG"
CONDUCTOR_RC=${PIPESTATUS[0]}
set -e

cleanup
PIDS=()
ROUDI_STARTED=""
trap - EXIT INT TERM

set +e
python3 "$SCRIPT_DIR/pp_mujoco_plant_report.py" \
  --plant "$PLANT_CSV" --runner-trace "$TRACE_CSV" \
  --ball-log "$BALL_LOG" --json "$PLANT_JSON" \
  --min-idle-s 2.0 --idle-trim-s 0.5 \
  2>&1 | tee "$PLANT_REPORT_LOG"
PLANT_REPORT_RC=${PIPESTATUS[0]}

python3 "$SCRIPT_DIR/pp_rally_report.py" "$OBS_CSV" \
  --mode rally_v17 --runner-log "$RUNNER_LOG" --runner-cwd "$DIST" \
  --runner-trace "$TRACE_CSV" \
  >"$RALLY_REPORT_LOG" 2>&1
RALLY_REPORT_RC=$?

python3 "$SCRIPT_DIR/pp_stationary_3ball_report.py" \
  --session "$SESSION_JSON" --physical "$PHYSICAL_JSON" \
  --obs "$OBS_CSV" --runner-trace "$TRACE_CSV" --plant "$PLANT_CSV" \
  --plant-report "$PLANT_JSON" \
  --onnx "$MODEL" --sim-plugin "$SIM_PLUGIN" --sim-xml "$SIM_XML" \
  --expected-shots "$EXPECTED_SHOTS" --verdict-mode "$VERDICT_MODE" \
  "${CONTACT_AB_REPORT_ARGS[@]}" \
  --output "$FINAL_JSON" \
  2>&1 | tee "$FINAL_REPORT_LOG"
FINAL_RC=${PIPESTATUS[0]}
set -e

for path in \
  "$RUNNER_LOG" "$BALL_LOG" "$PHYSICAL_JSON" "$SESSION_JSON" \
  "$OBS_CSV" "$TRACE_CSV" "$PLANT_CSV" "$PLANT_JSON" "$FINAL_JSON" \
  "$SIM_LOG" "$RAW_MOCAP_LOG" "$MOCAP_RELAY_LOG" "$BASE_RELAY_LOG" \
  "$PLANNER_LOG" "$CONTACT_LOG" "$RESET_LOG" "$CONDUCTOR_LOG" \
  "$ROUDI_LOG" \
  "$PLANT_REPORT_LOG" "$RALLY_REPORT_LOG" "$FINAL_REPORT_LOG" \
  "$PLANNER_DEBUG_CSV"; do
  [[ -e "$path" ]] && cp -a "$path" "$ARTIFACT_DIR/"
done

[[ -e "$FINAL_JSON" ]] &&
  cp -a "$FINAL_JSON" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_report.json"
[[ -e "$SESSION_JSON" ]] &&
  cp -a "$SESSION_JSON" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_session.json"
[[ -e "$PHYSICAL_JSON" ]] &&
  cp -a "$PHYSICAL_JSON" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_physical.json"
[[ -e "$OBS_CSV" ]] &&
  cp -a "$OBS_CSV" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_obs.csv"
[[ -e "$TRACE_CSV" ]] &&
  cp -a "$TRACE_CSV" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_trace.csv"
[[ -e "$PLANT_CSV" ]] &&
  cp -a "$PLANT_CSV" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_plant.csv"
[[ -e "$PLANT_JSON" ]] &&
  cp -a "$PLANT_JSON" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_plant.json"
[[ -e "$RUNNER_LOG" ]] &&
  cp -a "$RUNNER_LOG" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_runner.log"
[[ -e "$BALL_LOG" ]] &&
  cp -a "$BALL_LOG" "$ARTIFACT_DIR/pp_stationary_${EXPECTED_SHOTS}ball_ball.log"
[[ -n "$CONTACT_AB_JSON" && -e "$CONTACT_AB_JSON" ]] &&
  cp -a "$CONTACT_AB_JSON" "$ARTIFACT_DIR/scripted_racket_contact_ab.json"

echo "================ V17-r1 stationary ${EXPECTED_SHOTS}-ball (NON-CERTIFYING) ================"
echo "model_sha=$ACTUAL_MODEL_SHA"
echo "sim_plugin_sha=$ACTUAL_SIM_PLUGIN_SHA sim_xml_sha=$ACTUAL_SIM_XML_SHA"
echo "conductor_rc=$CONDUCTOR_RC plant_report_rc=$PLANT_REPORT_RC diagnostic_rally_report_rc=$RALLY_REPORT_RC final_rc=$FINAL_RC"
echo "certification=false hardware_authorized=false"
echo "artifact_dir=$ARTIFACT_DIR"
[[ -f "$FINAL_JSON" ]] && python3 - "$FINAL_JSON" "$EXPECTED_SHOTS" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
expected = int(sys.argv[2])
print(
    "stationary_endurance_pass={}; lateral_lock_replay_pass={}; "
    "fixed_y_homing_replay_pass={}; moving_recovery_replay_pass={}; "
    "no_fall_observed={}; "
    "replay_pass={}; contacts={}/{}; "
    "legal={}/{}; next={}".format(
        report["stationary_endurance_pass"],
        report["lateral_lock_replay_pass"],
        report["fixed_y_homing_replay_pass"],
        report["moving_recovery_replay_pass"],
        report["no_fall_observed"],
        report["replay_pass"],
        report["physical_summary"]["contacts"],
        expected,
        report["physical_summary"]["legal_landings"],
        expected,
        report["next_step_recommendation"],
    )
)
PY
echo "============================================================================="
exit "$FINAL_RC"
