#!/usr/bin/env bash
# Gate3 — autonomous end-to-end policy-native certification.
#
#   side-neutral physical ball command -> MuJoCo ball/table/net/racket contacts
#   -> raw OptiTrack NamedPoseArray boundary -> production mocap relay + planner
#   -> production runner argv -> MuJoCo plant -> contact/landing/recovery evidence
#
# The runner enters MOTION once and is never reset or manually steered between
# shots.  Its historical completion/recovery check remains an internal
# lifecycle regression; only the joined physical certification may PASS Gate3.
# Verdicts: per-serve table + /tmp/pp_rally_report.json; per-tick actor obs in
# /tmp/pp_obs.csv (analyze: scripts/pp_rally_report.py). PP_VIEWER=1 to watch.
# Knobs: PP_SERVES (12) PP_PAUSE_S (4.0) PP_RESET_Y (-0.7625)
#        PP_DROPOUT_AT (0 = off)
#        PP_MIN_PROXY_RATE / PP_ALLOW_RESCUE / PP_MAX_RESCUES / PP_REQUIRE_READY /
#        PP_MIN_STATION_TRANSITIONS. PP_RALLY_REPORT_MODE defaults to auto: it reads the exact
#        validated FinalV3 recipe marker from the runner, then falls back to the loaded ONNX's
#        hitter_pure_training_recipe metadata. Metadata-less packages must explicitly set
#        PP_RALLY_REPORT_MODE=legacy (or rally_final_v3); unknown/contradictory modes fail closed.
#        RallyFinal certification uses 0.8 / 0 / 0 / 1 / 8 so the
#        harness cannot pass by silently resetting or counting unreached pending stations.
set +e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR="${PP_GEAR:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
HOPE_ROOT="${PP_HOPE_ROOT:-$(cd -- "$GEAR/../.." && pwd)}"
DIST="${PP_DIST:-$GEAR/dist/a3_deploy_x86_64}"
WS="${PP_HOPE_WS:-$HOPE_ROOT/hope_ws}"
SIM_INSTALL="${PP_SIM_INSTALL:-$HOPE_ROOT/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/cmake-build-model21800-gate3/install}"
PLANNER_DEBUG_CSV="${PP_PLANNER_DEBUG_CSV:-/tmp/pp_planner_debug.csv}"
export PP_PLANNER_DEBUG_CSV="$PLANNER_DEBUG_CSV"
PACKETIZER_DEBUG_CSV="${PP_PACKETIZER_DEBUG_CSV:-/tmp/pp_flight_packets.csv}"
# The real MDU launcher and Gate3 consume the same policy-native argv contract.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pp_gate3_runner_common.sh"
export PP_RUNNER_CORE_ARGS="$RALLY_POLICY_NATIVE_CORE_ARGS"
# One source of truth for both run_sim.sh and the conductor's reset-message overlay.
# Previously PP_SIM_INSTALL changed only the latter while run_sim.sh silently kept using
# A3_SIM_INSTALL/default, allowing the simulator binary and ROS message package to drift.
export A3_SIM_INSTALL="${A3_SIM_INSTALL:-$SIM_INSTALL}"
if [ ! -r /opt/ros/jazzy/setup.bash ]; then
  echo "[g3r] ENV FAIL: ROS Jazzy is unavailable."
  echo "[g3r] Run Gate3 inside: distrobox enter hope"
  exit 2
fi
if [ ! -r "$WS/install/local_setup.bash" ]; then
  echo "[g3r] ENV FAIL: HOPE ROS workspace is not built: $WS/install/local_setup.bash"
  exit 2
fi
WORLD_CONFIG_SOURCE="$WS/src/hope_bringup/config/hope_world_frame.yaml"
WORLD_CONFIG_INSTALL="$WS/install/hope_bringup/share/hope_bringup/config/hope_world_frame.yaml"
WORLD_LAUNCH_SOURCE="$WS/src/hope_bringup/launch/hope_world.launch.py"
WORLD_LAUNCH_INSTALL="$WS/install/hope_bringup/share/hope_bringup/launch/hope_world.launch.py"
P1_CALIBRATION_RECEIPT="${PP_P1_CALIBRATION_RECEIPT:-/tmp/hope-gate3-sim-calibration.json}"
if [ ! -r "$WORLD_CONFIG_SOURCE" ] || [ ! -r "$WORLD_CONFIG_INSTALL" ] || \
   ! cmp -s "$WORLD_CONFIG_SOURCE" "$WORLD_CONFIG_INSTALL" || \
   [ ! -r "$WORLD_LAUNCH_SOURCE" ] || [ ! -r "$WORLD_LAUNCH_INSTALL" ] || \
   ! cmp -s "$WORLD_LAUNCH_SOURCE" "$WORLD_LAUNCH_INSTALL"; then
  echo "[g3r] ENV FAIL: installed hope_world launch/config is stale or missing."
  echo "[g3r] Rebuild hope_ws before Gate3; sim and relay must consume identical calibration bytes."
  exit 2
fi
if [ -z "${PP_P1_CALIBRATION_RECEIPT:-}" ]; then
  python3 "$SCRIPT_DIR/pp_gate3_sim_calibration.py" "$P1_CALIBRATION_RECEIPT" || exit 2
fi
if [ ! -r "$P1_CALIBRATION_RECEIPT" ]; then
  echo "[g3r] simulation calibration missing: $P1_CALIBRATION_RECEIPT"
  exit 2
fi
if [ ! -r "$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash" ] || \
   [ ! -x "$SIM_INSTALL/bin/aimrt_main" ] || \
   [ ! -f "$SIM_INSTALL/bin/cfg/a3_pingpong_iceoryx_cfg.yaml" ]; then
  echo "[g3r] ENV FAIL: the Gate3-instrumented MuJoCo install is incomplete: $SIM_INSTALL"
  exit 2
fi
if [ -n "${PP_RUNNER_LAUNCHER:-}" ]; then
  if [ ! -x "$PP_RUNNER_LAUNCHER" ]; then
    echo "[g3r] ENV FAIL: explicit runner launcher is missing: $PP_RUNNER_LAUNCHER"
    exit 2
  fi
elif [ ! -x "$DIST/run_a3_pingpong.sh" ] || \
     [ ! -x "$DIST/a3_deploy_onnx_ref_pingpong" ]; then
  echo "[g3r] ENV FAIL: packaged ping-pong runner is incomplete: $DIST"
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash"
# shellcheck disable=SC1090
source "$WS/install/local_setup.bash"
PLANNER_CPP_PREFIX="$(ros2 pkg prefix hope_planner_cpp 2>/dev/null)" || {
  echo "[g3r] ENV FAIL: hope_planner_cpp is not installed in the selected overlay"
  exit 2
}
PLANNER_CPP_EXE="$PLANNER_CPP_PREFIX/lib/hope_planner_cpp/hope_planner_cpp_node"
PACKETIZER_EXE="$PLANNER_CPP_PREFIX/lib/hope_planner_cpp/hope_ball_flight_packetizer"
PLANNER_CPP_CONFIG="${PP_PLANNER_CPP_CONFIG:-$WS/src/hope_planner_cpp/config/model21800_hardware.yaml}"
PACKETIZER_CONFIG="${PP_PACKETIZER_CONFIG:-$WS/src/hope_planner_cpp/config/model21800_flight_packetizer.yaml}"
for required in \
  "$PLANNER_CPP_EXE" \
  "$PACKETIZER_EXE" \
  "$PLANNER_CPP_CONFIG" \
  "$PACKETIZER_CONFIG"; do
  if [ ! -r "$required" ]; then
    echo "[g3r] ENV FAIL: C++ Planner artifact missing: $required"
    exit 2
  fi
done
if [ ! -x "$PLANNER_CPP_EXE" ] || [ ! -x "$PACKETIZER_EXE" ]; then
  echo "[g3r] ENV FAIL: C++ Planner executables are not executable; rebuild hope_planner_cpp"
  exit 2
fi
if ! python3 -c \
  'import rclpy; from mujoco_sim_msgs.msg import Gate3BallCommand, Gate3BallState; from motion_capture_tracking_interfaces.msg import NamedPoseArray; from std_srvs.srv import Trigger'; then
  echo "[g3r] ENV FAIL: Gate3 ROS Python messages are not importable"
  exit 2
fi
MOTION_IDLE_S="${PP_MOTION_IDLE_S:-2.0}"
if ! [[ "$MOTION_IDLE_S" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "[g3r] PROFILE FAIL: PP_MOTION_IDLE_S must be a non-negative number"
  exit 2
fi

# Internal engine only. Geometry, scoring knobs, and the exact serve table are owned by a
# versioned wrapper. A bare launch used to silently select whichever generation happened to be
# encoded in these defaults; that made a V10 model run with the V8/V9 intercept contract.
# Stable profiles use pp_gate3_rally_v8.sh or pp_gate3_rally_v10.sh; V11--V17 use explicitly
# candidate-labelled wrappers.
PP_GATE3_SCENARIO="${PP_GATE3_SCENARIO:-$PP_GATE3_PROFILE}"
export PP_GATE3_SCENARIO
export PP_LAUNCH_RECEIPT_JSON="${PP_LAUNCH_RECEIPT_JSON:-/tmp/pp_gate3_launch_receipt.json}"
required_profile_vars=(
  PP_GATE3_PROFILE PP_GATE3_SCENARIO PP_XHIT PP_XHIT_BH_DELTA PP_FIXED_PLANE_X PP_STATION_X
  PP_XLOCK_THRESH PP_READY_X_MAX PP_SERVES PP_SERVES_LIST
)
for name in "${required_profile_vars[@]}"; do
  if [ -z "${!name+x}" ] || [ -z "${!name}" ]; then
    echo "[g3r] PROFILE FAIL: missing $name; use a versioned pp_gate3_rally_v*.sh wrapper"
    exit 2
  fi
done
PP_EXTRA_ARGS="${PP_EXTRA_ARGS:-}"
export PP_EXTRA_ARGS
export PP_ALLOW_RESCUE=0
export PP_MAX_RESCUES=0
export PP_GATE3_VERDICT=certification
export PP_TABLE_HEIGHT_M=0.760
export PP_POLICY_Z_OFFSET=0.760
export PP_RESET_Y="${PP_RESET_Y:--0.7625}"
export PP_FLIGHT_S="${PP_FLIGHT_S:-2.5}"
export PP_PAUSE_S="${PP_PAUSE_S:-4.0}"
export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE="${PP_MIN_PHYSICAL_SAMPLES_PER_SIDE:-4}"
export PP_MIN_PHYSICAL_CONTACT_RATE="${PP_MIN_PHYSICAL_CONTACT_RATE:-0.8}"
export PP_MIN_LEGAL_LANDING_RATE="${PP_MIN_LEGAL_LANDING_RATE:-0.8}"
awk -v n="$PP_MIN_PHYSICAL_SAMPLES_PER_SIDE" \
    -v c="$PP_MIN_PHYSICAL_CONTACT_RATE" \
    -v l="$PP_MIN_LEGAL_LANDING_RATE" \
    'BEGIN {exit !(n >= 4 && c >= 0.8 && c <= 1.0 && l >= 0.8 && l <= 1.0)}' || {
  echo "[g3r] PROFILE FAIL: Gate3 requires >=4 shots/side and >=0.8 contact/landing rates"
  exit 2
}

# Gate3 is autonomous and fail-closed.  The conductor may start the runner once
# and stop it once, but it may not choose a side, substitute a velocity, or
# alter recovery/readiness timing relative to the real policy-native command.
if [[ "${PP_ALLOW_RESCUE:-0}" != "0" || "${PP_MAX_RESCUES:-0}" != "0" ]]; then
  echo "[g3r] PROFILE FAIL: Gate3 forbids operator rescue"
  exit 2
fi
if [[ "${PP_GATE3_VERDICT:-certification}" != "certification" ]]; then
  echo "[g3r] PROFILE FAIL: Gate3 verdict is always certification"
  exit 2
fi
if [[ -n "${PP_EXTRA_ARGS}" ]]; then
  # shellcheck disable=SC2206
  gate3_extra_argv=(${PP_EXTRA_ARGS})
  rally_assert_policy_native_runner_args \
    --planner --policy-native --start passive --official-stand \
    "${gate3_extra_argv[@]}"
else
  rally_assert_policy_native_runner_args \
    --planner --policy-native --start passive --official-stand
fi
if [ "$PP_GATE3_PROFILE" = rally_v17_r10 ]; then
  awk -v split_y="${PP_SPLIT_Y:-}" -v hyst="${PP_SPLIT_HYST:-0.04}" \
      'BEGIN {exit !(split_y == -0.265 && hyst == 0.04)}' || {
    echo "[g3r] PROFILE FAIL: R10 requires exported-box midpoint split=-0.265 hysteresis=0.04"
    exit 2
  }
else
  expected_hyst=0.04
  if [ "${PP_GATE3_PROFILE:-}" = "hitter_pingpong_shared_plane_v1" ] &&
     { [ "${PP_REQUIRE_OPTIONAL_REACH:-0}" = "1" ] ||
       [ "${PP_REQUIRE_QUESTION_BANK_PROVENANCE:-0}" = "1" ]; }; then
    expected_hyst=0.0
  fi
  awk -v split_y="${PP_SPLIT_Y:--0.25}" -v hyst="${PP_SPLIT_HYST:-0.04}" \
      -v expected_hyst="$expected_hyst" \
      'BEGIN {exit !(split_y == -0.25 && hyst == expected_hyst)}' || {
    echo "[g3r] PROFILE FAIL: Gate3 side split/hysteresis disagrees with its artifact-bound Planner profile"
    exit 2
  }
fi
awk -v y="$PP_RESET_Y" -v z="$PP_POLICY_Z_OFFSET" -v h="$PP_TABLE_HEIGHT_M" \
    'BEGIN {exit !(y == -0.7625 && z == 0.760 && h == 0.760)}' || {
  echo "[g3r] PROFILE FAIL: Gate3 must use the production arena/table frame"
  exit 2
}
case "$PP_GATE3_PROFILE" in
  rally_v8_v13)
    awk -v x="$PP_XHIT" -v d="$PP_XHIT_BH_DELTA" -v p="$PP_FIXED_PLANE_X" \
        -v s="$PP_STATION_X" 'BEGIN {exit !(x==0.15 && d==-0.15 && p==0.65 && s==-0.50)}' || {
      echo "[g3r] PROFILE FAIL: rally_v8_v13 geometry drifted"; exit 2;
    }
    ;;
  rally_v10)
    awk -v x="$PP_XHIT" -v d="$PP_XHIT_BH_DELTA" -v p="$PP_FIXED_PLANE_X" \
        -v s="$PP_STATION_X" -v l="$PP_XLOCK_THRESH" \
        'BEGIN {exit !(x==0.08 && d==0.0 && p==0.58 && s==-0.50 && l==0.05)}' || {
      echo "[g3r] PROFILE FAIL: rally_v10 shared-plane geometry drifted"; exit 2;
    }
    ;;
  rally_v11|rally_v12|rally_v13|rally_v14|hitter_pingpong_recovery_tts_v2|hitter_pingpong_recovery_tts_v3|hitter_pingpong_continuous_rally_v2|hitter_pingpong_shared_plane_v1|rally_v15|rally_v17)
    awk -v x="$PP_XHIT" -v d="$PP_XHIT_BH_DELTA" -v p="$PP_FIXED_PLANE_X" \
        -v s="$PP_STATION_X" -v l="$PP_XLOCK_THRESH" \
        -v rx="$PP_READY_X_MAX" -v ry="${PP_READY_Y_MAX:-}" \
        -v rv="${PP_READY_SPEED_MAX:-}" \
        'BEGIN {exit !(x==0.08 && d==0.0 && p==0.58 && s==-0.50 && l==0.05 && rx==0.10 && ry==0.10 && rv==0.20)}' || {
      echo "[g3r] PROFILE FAIL: $PP_GATE3_PROFILE shared-plane/READY contract drifted"; exit 2;
    }
    ;;
  rally_v17_r10)
    awk -v x="$PP_XHIT" -v d="$PP_XHIT_BH_DELTA" -v p="$PP_FIXED_PLANE_X" \
        -v s="$PP_STATION_X" -v l="$PP_XLOCK_THRESH" \
        -v rx="$PP_READY_X_MAX" -v ry="${PP_READY_Y_MAX:-}" \
        -v rv="${PP_READY_SPEED_MAX:-}" \
        'BEGIN {exit !(x==0.08 && d==0.0 && p==0.58 && s==-0.50 && l==0.0 && rx==0.10 && ry==0.10 && rv==0.20)}' || {
      echo "[g3r] PROFILE FAIL: rally_v17_r10 fixed-station recovery geometry drifted"; exit 2;
    }
    ;;
  *)
    echo "[g3r] PROFILE FAIL: unsupported PP_GATE3_PROFILE='$PP_GATE3_PROFILE'"
    exit 2
    ;;
esac

if ! python3 - "$PP_SERVES_LIST" "$PP_SERVES" "$SCRIPT_DIR" <<'PY'
import sys

sys.path.insert(0, sys.argv[3])
from pp_gate3_core import parse_serves_list

serves = parse_serves_list(sys.argv[1])
if len(serves) != int(sys.argv[2]):
    raise SystemExit(
        f"Gate3 scenario count mismatch: parsed={len(serves)} PP_SERVES={sys.argv[2]}"
    )
PY
then
  echo "[g3r] PROFILE FAIL: side-neutral physical scenario is invalid"
  exit 2
fi

if [ "${PP_REQUIRE_IDLE_SMOOTHNESS:-0}" = "1" ]; then
  case " $PP_RUNNER_CORE_ARGS " in
    *" --policy-native "*) ;;
    *)
      echo "[g3r] PROFILE FAIL: idle smoothness gate requires --policy-native"
      exit 2
      ;;
  esac
  awk -v wait_s="$MOTION_IDLE_S" -v min_s="${PP_MIN_MOTION_IDLE_S:-15.0}" \
      'BEGIN {exit !(wait_s + 0.0 >= min_s + 0.0 && min_s + 0.0 > 0.0)}' || {
    echo "[g3r] PROFILE FAIL: PP_MOTION_IDLE_S=${PP_MOTION_IDLE_S:-2.0} must be >= PP_MIN_MOTION_IDLE_S=${PP_MIN_MOTION_IDLE_S:-15.0}"
    exit 2
  }
fi

if [ "${PP_GATE3_PREFLIGHT_ONLY:-0}" = "1" ]; then
  echo "[g3r] PREFLIGHT PASS: model_21800 ${PP_GATE3_PHASE} environment is complete"
  echo "[g3r] runner=$DIST/a3_deploy_onnx_ref_pingpong"
  echo "[g3r] sim=$SIM_INSTALL/bin/aimrt_main"
  echo "[g3r] planner=$PLANNER_CPP_EXE"
  echo "[g3r] flight_packetizer=$PACKETIZER_EXE"
  exit 0
fi

echo "[g3r] cleanup"
if [ "${PP_MANAGED_RUN:-0}" != "1" ]; then
if [ "${PP_REUSE_SIM:-0}" != "1" ]; then
  pkill -9 -f "aimrt_main.*a3_pingpong" 2>/dev/null
  pkill -9 -x iox-roudi 2>/dev/null
  rm -f /dev/shm/iox1_0_* 2>/dev/null
fi
pkill -9 -f "a3_deploy_onnx_ref_pingpong" 2>/dev/null
pkill -9 -f "hope_planner_cpp_node" 2>/dev/null
pkill -9 -f "hope_ball_flight_packetizer" 2>/dev/null
pkill -9 -f "hope_base_pose_flat_relay" 2>/dev/null
pkill -9 -f "ros2 launch hope_bringup hope_world.launch.py" 2>/dev/null
pkill -9 -f "optitrack_mct_relay" 2>/dev/null
pkill -9 -f "pp_gate3_sim_mocap.py" 2>/dev/null
pkill -9 -f "pp_gate3_ball_launcher.py" 2>/dev/null
pkill -9 -f "pp_gate3_ball_evidence.py" 2>/dev/null
fi
rm -f /tmp/pp_obs.csv /tmp/pp_runner_trace.csv \
  "${PP_STOP_LAUNCHES_FILE:-/tmp/pp_gate3_stop_launches}" \
  /tmp/pp_ball_trajectories.csv \
  "${PP_DRAIN_REPORT_JSON:-/tmp/pp_gate3_drain_report.json}" \
  /tmp/pp_mujoco_plant_report.json \
  /tmp/pp_rally_report.json /tmp/pp_runner.log /tmp/pp_ball.log \
  /tmp/pp_contact.log /tmp/pp_raw_mocap.log /tmp/pp_mocap_relay.log \
  /tmp/pp_base_relay.log /tmp/pp_packetizer.log "$PLANNER_DEBUG_CSV" \
  "$PP_LAUNCH_RECEIPT_JSON" \
  "$PLANNER_DEBUG_CSV.flight_packets.csv" "$PACKETIZER_DEBUG_CSV" \
  "${PP_PLANNER_EVIDENCE_JSON:-/tmp/pp_planner_cpp_report.json}" \
  "${PP_QUESTION_PROVENANCE_JSON:-/tmp/pp_gate3_question_provenance_report.json}" \
  "${PP_PHYSICAL_EVIDENCE_JSON:-/tmp/pp_physical_ball_report.json}" \
  2>/dev/null
if [ "${PP_REUSE_SIM:-0}" != "1" ]; then
  rm -f /tmp/pp_mujoco_plant.csv 2>/dev/null
fi
sleep 1

gate3_cleanup() {
  # Keep the physical simulator and recorder alive through the final flight,
  # even when a policy falls or the caller interrupts the run.
  local drain_rc=0
  python3 "$SCRIPT_DIR/pp_gate3_drain_flights.py" || drain_rc=$?
  if [ "$drain_rc" -ne 0 ]; then
    echo "[g3r] flight evidence incomplete (rc=$drain_rc); cleaning up failed run"
    python3 "$SCRIPT_DIR/pp_gate3_drain_flights.py" --finalize-report /tmp/pp_rally_report.json
  fi
  if [ "${PP_MANAGED_RUN:-0}" = "1" ]; then
    # Only process groups recorded by this run; never pkill another session.
    if [ -f "$PP_CHILD_PIDS_FILE" ]; then
      while read -r child; do
        [[ "$child" =~ ^[0-9]+$ ]] && kill -TERM -- "-$child" 2>/dev/null
      done < "$PP_CHILD_PIDS_FILE"
      sleep 1
      while read -r child; do
        [[ "$child" =~ ^[0-9]+$ ]] && kill -KILL -- "-$child" 2>/dev/null
      done < "$PP_CHILD_PIDS_FILE"
    fi
    if [ -f "${A3_IOX_ROUDI_PID_FILE:-}" ]; then
      read -r child < "$A3_IOX_ROUDI_PID_FILE"
      [[ "$child" =~ ^[0-9]+$ ]] && kill -TERM -- "-$child" 2>/dev/null
    fi
    return "$drain_rc"
  fi
  pkill -9 -f "a3_deploy_onnx_ref_pingpong" 2>/dev/null
  pkill -9 -f "hope_planner_cpp_node" 2>/dev/null
  pkill -TERM -f "hope_ball_flight_packetizer" 2>/dev/null
  pkill -TERM -f "ros2 launch hope_bringup hope_world.launch.py" 2>/dev/null
  pkill -TERM -f "hope_base_pose_flat_relay" 2>/dev/null
  pkill -INT -f "pp_gate3_ball_evidence.py" 2>/dev/null
  sleep 1
  pkill -TERM -f "pp_gate3_ball_launcher.py" 2>/dev/null
  pkill -TERM -f "pp_gate3_sim_mocap.py" 2>/dev/null
  pkill -TERM -f "optitrack_mct_relay" 2>/dev/null
  pkill -9 -f "ros2 topic echo" 2>/dev/null
  if [ "${PP_REUSE_SIM:-0}" != "1" ]; then
    pkill -9 -f "aimrt_main.*a3_pingpong" 2>/dev/null
    pkill -9 -x iox-roudi 2>/dev/null
    rm -f /dev/shm/iox1_0_* 2>/dev/null
  fi
  return "$drain_rc"
}
gate3_on_exit() {
  local original_rc=$? cleanup_rc=0
  trap - EXIT
  trap '' INT TERM
  gate3_cleanup || cleanup_rc=$?
  if [ "$original_rc" -eq 0 ]; then original_rc=$cleanup_rc; fi
  exit "$original_rc"
}
trap gate3_on_exit EXIT
trap 'exit 130' INT TERM

echo "[g3r] sim up (iceoryx body-drive + ros2 /sim/a3/pelvis_pose)"
cd "$GEAR"
GL_ENV="MUJOCO_GL=egl"
[ "${PP_VIEWER:-0}" = "1" ] && GL_ENV=""
if [ "${PP_REUSE_SIM:-0}" != "1" ]; then
  setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; $GL_ENV \
    A3_MUJOCO_DEBUG_CSV=/tmp/pp_mujoco_plant.csv \
    A3_MUJOCO_DEBUG_STRIDE=${PP_MUJOCO_DEBUG_STRIDE:-5} \
    A3_MUJOCO_PD_MODE=${PP_MUJOCO_PD_MODE:-explicit} \
    A3_GATE3_BALL_DRAG_K=0.1220 \
    A3_GATE3_BALL_RESTITUTION_H=0.64 \
    A3_GATE3_BALL_RESTITUTION_V=0.9215 \
    A3_SIM_FLAVOR=auto A3_SIM_CFG=a3_pingpong_iceoryx_cfg.yaml ./scripts/run_sim.sh" >/tmp/pp_sim.log 2>&1 &
  export PP_SIM_PID=$!
    if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi
  for i in $(seq 1 40); do
    grep -qiE "will wait for shutdown|Sim Start|gui" /tmp/pp_sim.log 2>/dev/null && break
    sleep 1
  done
  grep -qiE "will wait for shutdown|Sim Start|gui" /tmp/pp_sim.log 2>/dev/null || { echo "[g3r] SIM FAIL"; tail -8 /tmp/pp_sim.log; exit 1; }
else
  echo "[g3r] reusing attended MuJoCo process"
fi

echo "[g3r] raw simulated OptiTrack boundary (/sim plant -> /optitrack/poses)"
SIM_MOCAP_EXTRA_ARG=""
if [ "${PP_SKIP_P1_CALIBRATION_SHA:-0}" = "1" ]; then
  SIM_MOCAP_EXTRA_ARG="--allow-uncalibrated-p1-sim"
  if [ "${PP_BASE_OUTAGE_AT:-0}" != "0" ]; then
    echo "[g3r] ENV FAIL: quick simulator-truth base has no production relay to suspend"
    exit 2
  fi
fi
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null; \
  source '$WS/install/local_setup.bash' 2>/dev/null; \
  python3 '$SCRIPT_DIR/pp_gate3_sim_mocap.py' \
  --table-height-m '$PP_TABLE_HEIGHT_M' \
  --world-config '$WS/src/hope_bringup/config/hope_world_frame.yaml' \
  $SIM_MOCAP_EXTRA_ARG" \
  >/tmp/pp_raw_mocap.log 2>&1 &
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

echo "[g3r] production OptiTrack relay (/optitrack/poses -> /poses + /UCB_P1/pose)"
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
  source '$WS/install/local_setup.bash' 2>/dev/null; \
  ros2 run hope_bringup optitrack_mct_relay --ros-args \
  --params-file '$WS/src/hope_bringup/config/optitrack_relay.yaml' \
  -p publish_tf:=false" >/tmp/pp_mocap_relay.log 2>&1 &
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

if [ "${PP_SKIP_P1_CALIBRATION_SHA:-0}" = "1" ]; then
  # pp_gate3_sim_mocap.py owns /a3/base_pose_flat directly in this explicitly
  # uncalibrated simulator-truth mode. Starting hope_world here would create a
  # second producer whose invalid samples race and overwrite the valid mailbox.
  echo "[g3r] quick simulator-truth base direct; production hope_world relay disabled"
  export PP_BASE_RELAY_PGID=0
  export PP_BASE_PRODUCER_CONTRACT=simulator_truth_direct_v1
else
  echo "[g3r] calibrated hope_world relay (/UCB_P1/pose -> pelvis schema-2 base)"
  # Localization is a fail-closed safety input. Keep the calibrated relay in its
  # own process so Planner compute cannot delay the schema-2 base stream. The
  # launch consumes the same full translation, quaternion and calibration
  # receipts as the real OptiTrack path.
  setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; source $WS/install/local_setup.bash 2>/dev/null; \
    ros2 launch hope_bringup hope_world.launch.py \
    ucb_calibration_file:='$P1_CALIBRATION_RECEIPT'" \
    >/tmp/pp_base_relay.log 2>&1 &
    if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi
  # `setsid` makes this launch PID its process-group leader. The conductor can
  # suspend only this calibrated base-output path for a bounded outage stress.
  export PP_BASE_RELAY_PGID=$!
  export PP_BASE_PRODUCER_CONTRACT=calibrated_hope_world_relay_v1
fi

echo "[g3r] production C++ flight packetizer (/poses -> /ball/flight_packet)"
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; source $WS/install/local_setup.bash 2>/dev/null; \
  '$PACKETIZER_EXE' --ros-args \
  --params-file '$PACKETIZER_CONFIG' \
  -p session_id:=${PP_GATE3_SCENARIO}_gate3 \
  -p debug_csv_path:='$PACKETIZER_DEBUG_CSV'" \
  >/tmp/pp_packetizer.log 2>&1 &
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

echo "[g3r] production hope_planner_cpp (immutable flight packet -> schema-2 command)"
# The model_21800-compatible C++ Planner retains the production post-net one-shot contract.
echo "      hitter_pure profile: world x_hit=$PP_XHIT, shared reach plane=$PP_FIXED_PLANE_X, station_x=$PP_STATION_X"
echo "      backhand x_hit delta=$PP_XHIT_BH_DELTA (V10--V15 require 0.0 for y-only footwork)"
echo "      gate profile=$PP_GATE3_PROFILE (versioned wrapper; bare launch is rejected)"
echo "      landing x=${PP_LAND_X:-2.055} y(fh/bh)=${PP_LAND_Y_FH:--0.7625}/${PP_LAND_Y_BH:--0.7625}"
echo "      dtf(fh/bh)=${PP_DTF_FH:-0.50}/${PP_DTF_BH:-0.50}; versioned wrapper values are authoritative."
PLANNER_PROFILE_ARG=""
if [ -n "${PP_PLANNER_PROFILE:-}" ]; then
  if [ ! -f "$PP_PLANNER_PROFILE" ]; then
    echo "[g3r] ENV FAIL: planner profile missing: $PP_PLANNER_PROFILE"
    exit 2
  fi
  PLANNER_PROFILE_ARG="--params-file $PP_PLANNER_PROFILE"
fi
QUESTION_FIXTURE_ARGS=""
if [ -n "${PP_QUESTION_FIXTURE_CSV:-}" ]; then
  if [ ! -r "$PP_QUESTION_FIXTURE_CSV" ]; then
    echo "[g3r] ENV FAIL: question fixture CSV missing: $PP_QUESTION_FIXTURE_CSV"
    exit 2
  fi
  case "$PP_QUESTION_FIXTURE_CSV" in
    *[!A-Za-z0-9_./-]*)
      echo "[g3r] ENV FAIL: question fixture path contains unsupported shell characters"
      exit 2
      ;;
  esac
  if [ "${PP_QUESTION_FIXTURE_CONTRACT:-}" != "schema31_gate3_v5_l0_fixture_v1" ] || \
     [ "${PP_QUESTION_FIXTURE_BANK_SHA256:-}" != "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e" ] || \
     [ "${PP_QUESTION_FIXTURE_RECEIPT_SHA256:-}" != "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493" ] || \
     [ "${PP_QUESTION_FIXTURE_EXPECTED_FLIGHTS:-}" != "26" ] || \
     [ "${PP_SERVES:-}" != "26" ] || \
     [ "${PP_QUESTION_FIXTURE_MAX_POSITION_ERROR_M:-}" != "0.03" ] || \
     [ "${PP_QUESTION_FIXTURE_MAX_VELOCITY_ERROR_MPS:-}" != "0.10" ] || \
     [ "${PP_QUESTION_FIXTURE_MAX_TTS_ERROR_S:-}" != "0.03" ]; then
    echo "[g3r] PROFILE FAIL: Schema31 v5 fixture binding/tolerances drifted"
    exit 2
  fi
  QUESTION_FIXTURE_ARGS="-p question_fixture_csv:=$PP_QUESTION_FIXTURE_CSV \
    -p question_fixture_simulator_only:=true \
    -p question_fixture_contract:=$PP_QUESTION_FIXTURE_CONTRACT \
    -p question_fixture_bank_sha256:=$PP_QUESTION_FIXTURE_BANK_SHA256 \
    -p question_fixture_receipt_sha256:=$PP_QUESTION_FIXTURE_RECEIPT_SHA256 \
    -p question_fixture_expected_flights:=$PP_QUESTION_FIXTURE_EXPECTED_FLIGHTS \
    -p question_fixture_max_position_error_m:=$PP_QUESTION_FIXTURE_MAX_POSITION_ERROR_M \
    -p question_fixture_max_velocity_error_mps:=$PP_QUESTION_FIXTURE_MAX_VELOCITY_ERROR_MPS \
    -p question_fixture_max_tts_error_s:=$PP_QUESTION_FIXTURE_MAX_TTS_ERROR_S"
  echo "      Schema31 v5 exact-row fixture=$PP_QUESTION_FIXTURE_CSV (simulator-only, fail-closed physical residual)"
fi
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; source $WS/install/local_setup.bash 2>/dev/null; \
  exec '$PLANNER_CPP_EXE' --ros-args \
  --params-file '$PLANNER_CPP_CONFIG' \
  $PLANNER_PROFILE_ARG \
  -p flight_packet_input_enabled:=true -p flight_packet_topic:=/ball/flight_packet \
  -p base_pose_flat_input_topic:=/a3/base_pose_flat -p publish_base_flat:=false \
  -p policy_z_offset:=$PP_POLICY_Z_OFFSET -p fit_window:=26 \
  -p drag_k:=0.1220 -p restitution_h:=0.64 -p restitution_v:=0.9215 \
  -p x_hit_follow_robot:=false -p x_hit:=$PP_XHIT \
  -p x_hit_freeze_session_home:=${PP_XHIT_FREEZE_SESSION_HOME:-false} \
  -p x_hit_bh_delta:=$PP_XHIT_BH_DELTA \
  -p swing_side_reference_mode:=${PP_SWING_SIDE_REFERENCE_MODE:-live_base_v1} \
  -p swing_side_split_y:=${PP_SPLIT_Y:--0.25} -p swing_side_hysteresis_y:=${PP_SPLIT_HYST:-0.04} \
  -p target_land_x:=${PP_LAND_X:-2.055} \
  -p target_land_y_fh:=${PP_LAND_Y_FH:--0.7625} -p target_land_y_bh:=${PP_LAND_Y_BH:--0.7625} \
  -p delta_t_flight_fh:=${PP_DTF_FH:-0.50} -p delta_t_flight_bh:=${PP_DTF_BH:-0.50} \
  -p max_predict_time:=2.6 -p solve_period_s:=0.033 \
  $QUESTION_FIXTURE_ARGS \
  -p debug_csv_path:=$PLANNER_DEBUG_CSV \
  -p debug_session_id:=${PP_GATE3_SCENARIO}_gate3" >/tmp/pp_planner.log 2>&1 &
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

echo "[g3r] 1 kHz MuJoCo contact edges + 250 Hz loss-detectable landing telemetry"
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null; \
  PP_PHYSICAL_EVIDENCE_JSON='${PP_PHYSICAL_EVIDENCE_JSON:-/tmp/pp_physical_ball_report.json}' \
  PP_SERVES='${PP_SERVES:-12}' \
  python3 '$SCRIPT_DIR/pp_gate3_ball_evidence.py'" >/tmp/pp_contact.log 2>&1 &
  export PP_EVIDENCE_PID=$!
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

echo "[g3r] side-neutral physical scenario=$PP_GATE3_SCENARIO geometry_profile=$PP_GATE3_PROFILE ${PP_SERVES}-shot sweep"
setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
  source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null; \
  PP_SERVES_LIST='$PP_SERVES_LIST' PP_SERVES='${PP_SERVES:-12}' \
  PP_FLIGHT_S='$PP_FLIGHT_S' PP_PAUSE_S='${PP_PAUSE_S:-4.0}' \
  PP_MOTION_IDLE_S='$MOTION_IDLE_S' \
  python3 '$SCRIPT_DIR/pp_gate3_ball_launcher.py'" >/tmp/pp_ball.log 2>&1 &
  if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

setsid bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic echo --field pose.position.z /sim/a3/pelvis_pose" >/tmp/pp_pelvisz.log 2>&1 &
if [ -n "${PP_CHILD_PIDS_FILE:-}" ]; then printf '%s\n' "$!" >> "$PP_CHILD_PIDS_FILE"; fi

echo "[g3r] flat topics alive?"
sleep 4
# Both quick simulator-truth and calibrated production modes have exactly one
# authoritative /a3/base_pose_flat producer. Fail before MOTION if an unrelated
# or accidentally duplicated publisher is present on the ROS graph.
BASE_PUBLISHER_COUNT=0
for _ in $(seq 1 10); do
  BASE_PUBLISHER_COUNT="$(timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic info --no-daemon --spin-time 1 /a3/base_pose_flat 2>/dev/null" | awk '/Publisher count:/ {print $3; exit}')"
  [ "${BASE_PUBLISHER_COUNT:-0}" -eq 1 ] 2>/dev/null && break
  sleep 1
done
if [ "${BASE_PUBLISHER_COUNT:-0}" -ne 1 ] 2>/dev/null; then
  echo "[g3r] BASE PRODUCER FAIL: /a3/base_pose_flat publisher count=${BASE_PUBLISHER_COUNT:-0}, expected exactly 1"
  exit 1
fi
export PP_BASE_PUBLISHER_COUNT="$BASE_PUBLISHER_COUNT"
# No flight packet exists before the first ball is released. Verify graph discovery here;
# waiting for a sample would turn the correct pre-serve idle state into a false transport fault.
timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic info /ball/flight_packet 2>&1" | tail -2
timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic hz /racket/command_flat 2>&1 | head -2" | tail -1
timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic hz /a3/base_pose_flat 2>&1 | head -2" | tail -1
timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic hz /UCB_P1/pose 2>&1 | head -2" | tail -1
timeout 5 bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; source '$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash' 2>/dev/null; ros2 topic hz /sim/gate3/ball_state 2>&1 | head -2" | tail -1

echo "[g3r] prewarm ros2 pub discovery (straggler-reset-in-MOTION trap)"
RESET_PREWARM_OK=0
: >/tmp/pp_reset_prewarm.log
for _ in $(seq 1 3); do
  if timeout 10 "$GEAR/scripts/reset_sim.sh" >>/tmp/pp_reset_prewarm.log 2>&1; then
    RESET_PREWARM_OK=1
    break
  fi
  sleep 1
done
if [ "$RESET_PREWARM_OK" -ne 1 ]; then
  echo "[g3r] RESET DISCOVERY FAIL: /sim/a3/reset has no live subscriber"
  tail -4 /tmp/pp_reset_prewarm.log
  exit 1
fi

echo "[g3r] RALLY conductor: stand -> m ONCE, then ${PP_SERVES:-12} serves, no resets"
source "$SIM_INSTALL/share/mujoco_sim_msgs/local_setup.bash" 2>/dev/null
export AMENT_PREFIX_PATH="$SIM_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$SIM_INSTALL/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
cd "$DIST"
python3 "$GEAR/scripts/pp_rally_conductor.py"
RC=$?

echo "[g3r] stopping"
gate3_cleanup
CLEANUP_RC=$?
if [ "$CLEANUP_RC" -ne 0 ]; then RC=$CLEANUP_RC; fi
trap - EXIT INT TERM

echo "========================= RESULTS (rally) ========================="
echo "--- per-serve verdicts + SUMMARY are above ([rally] lines); JSON: /tmp/pp_rally_report.json ---"
echo "--- engage/complete/recovery event stream ---"
grep -aE "\[pp engage\]|swing complete|recovery done" /tmp/pp_runner.log | head -40
echo "--- FALL GUARD / mode flips (want: exactly one -> MOTION, no FALL GUARD) ---"
grep -anE "FALL GUARD|-> MOTION|-> PD_STAND|-> PASSIVE" /tmp/pp_runner.log | head -12
echo "--- gate rejections (first 8; z_w=0.00 late-in-flight is a dead ball = normal) ---"
grep -aE "\[pp gate\] REJECT" /tmp/pp_runner.log | head -8
echo "--- planner status distribution ---"
grep -aoE "PLANNER: [a-z_]+\]" /tmp/pp_runner.log | sort | uniq -c
if [ -n "${PP_REAL_TAIL_FIXTURE:-}" ]; then
echo "--- real-command distribution tail coverage (independent of functional Gate3) ---"
REAL_TAIL_FIXTURE="$PP_REAL_TAIL_FIXTURE"
REAL_TAIL_REPORT="${PP_REAL_TAIL_REPORT_JSON:-/tmp/pp_gate3_real_tail_report.json}"
python3 "$GEAR/scripts/pp_gate3_real_tail_audit.py" \
  --planner-csv "$PLANNER_DEBUG_CSV" \
  --fixture "$REAL_TAIL_FIXTURE" \
  --json "$REAL_TAIL_REPORT" \
  --policy-z-offset-m "$PP_POLICY_Z_OFFSET"
REAL_TAIL_RC=$?
if [ "$REAL_TAIL_RC" -ne 0 ]; then
  echo "[g3r] real-tail distribution NOT_COVERED (functional/physical Gate3 verdict unchanged)"
fi
if [ "${PP_REQUIRE_REAL_TAIL_COVERAGE:-0}" = "1" ] && [ "$REAL_TAIL_RC" -ne 0 ]; then
  echo "[g3r] required real-tail lane FAILED (rc=$REAL_TAIL_RC)"
  RC=$REAL_TAIL_RC
fi
elif [ "${PP_REQUIRE_REAL_TAIL_COVERAGE:-0}" = "1" ]; then
  echo "[g3r] required real-tail fixture missing: set PP_REAL_TAIL_FIXTURE"
  RC=2
fi
if [ "${PP_REQUIRE_QUESTION_BANK_PROVENANCE:-0}" = "1" ]; then
  echo "--- Schema3 joint question-bank row provenance (axis box alone cannot qualify) ---"
  python3 "$GEAR/scripts/pp_gate3_question_provenance_report.py" \
    --planner-csv "$PLANNER_DEBUG_CSV" \
    --bank "${PP_QUESTION_BANK_PATH:?Schema3 question bank path is required}" \
    --receipt "${PP_QUESTION_BANK_RECEIPT_PATH:?Schema3 question bank receipt is required}" \
    --json "${PP_QUESTION_PROVENANCE_JSON:-/tmp/pp_gate3_question_provenance_report.json}" \
    --minimum-flights "${PP_SERVES:-26}"
  QUESTION_PROVENANCE_RC=$?
  if [ "$QUESTION_PROVENANCE_RC" -ne 0 ]; then
    echo "[g3r] Schema3 formal qualification FAIL/NOT_PROVEN: exact joint-row provenance missing (rc=$QUESTION_PROVENANCE_RC)"
    RC=$QUESTION_PROVENANCE_RC
  fi
fi
echo "--- localization health (formal fresh-localization runs require zero stale MOTION ticks) ---"
grep -acE "NO FRESH mocap base sample" /tmp/pp_runner.log | xargs -I{} echo "stale-base warns: {}"
tail -1 /tmp/pp_base_relay.log 2>/dev/null || true
echo "--- measured MuJoCo racket contact and legal landing evidence ---"
if [ -s "${PP_PHYSICAL_EVIDENCE_JSON:-/tmp/pp_physical_ball_report.json}" ]; then
  python3 - "${PP_PHYSICAL_EVIDENCE_JSON:-/tmp/pp_physical_ball_report.json}" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
rows = report.get("rows", [])
print(
    "shots={shots}; telemetry_complete={complete}; contacts={contacts}; "
    "landing_observations={observations}; landing_censored={censored}; "
    "legal_landings={landings}; physical_contact_measured={physical}; "
    "landing_measured={landing}".format(
        shots=len(rows),
        complete=sum(bool(row.get("telemetry_complete")) for row in rows),
        contacts=sum(bool(row.get("contact_pass")) for row in rows),
        observations=sum(bool(row.get("landing_observed")) for row in rows),
        censored=sum(bool(row.get("landing_censored")) for row in rows),
        landings=sum(bool(row.get("landing_pass")) for row in rows),
        physical=report.get("physical_contact_measured", False),
        landing=report.get("landing_measured", False),
    )
)
PY
else
  echo "physical evidence report missing; log: /tmp/pp_contact.log"
fi
echo "--- pelvis z (min/max over the run) ---"
grep -aoE "^-?[0-9]+\.[0-9]+" /tmp/pp_pelvisz.log | awk 'NR==1{m=$1;mx=$1}{if($1<m)m=$1;if($1>mx)mx=$1;l=$1}END{printf "min=%.3f max=%.3f last=%.3f n=%d\n",m,mx,l,NR}'
echo "--- MuJoCo plant + runner command trace ---"
PLANT_REPORT_ARGS=(
  --plant /tmp/pp_mujoco_plant.csv --runner-trace /tmp/pp_runner_trace.csv
  --ball-log /tmp/pp_ball.log --json /tmp/pp_mujoco_plant_report.json
  --physical-ball-report /tmp/pp_physical_ball_report.json
  --optional-reach-distribution-contract "${PP_OPTIONAL_REACH_DISTRIBUTION_CONTRACT:-stress_balanced_10_8_8_v1}"
  --optional-reach-trajectory-qualification "${PP_OPTIONAL_REACH_TRAJECTORY_QUALIFICATION:-NOT_PROVEN}"
  --optional-reach-expected-flights "${PP_OPTIONAL_REACH_EXPECTED_FLIGHTS:-26}"
)
if [ "${PP_REQUIRE_IDLE_SMOOTHNESS:-0}" = "1" ]; then
  PLANT_REPORT_ARGS+=(
    --require-idle-smoothness
    --min-idle-s "${PP_MIN_MOTION_IDLE_S:-15.0}"
    --idle-trim-s "${PP_IDLE_TRIM_S:-1.0}"
    --max-qdes-step-peak-rad "${PP_IDLE_MAX_QDES_STEP_PEAK_RAD:-0.08}"
    --max-qdes-step-rms-rad "${PP_IDLE_MAX_QDES_STEP_RMS_RAD:-0.005}"
    --max-qdes-reversals-hz "${PP_IDLE_MAX_QDES_REVERSALS_HZ:-8.0}"
    # Diagnostic reference only: low-gain PD needs static q_des-q to generate balance torque.
    --max-tracking-error-rms-rad "${PP_IDLE_MAX_TRACKING_ERROR_RMS_RAD:-0.15}"
    --max-qd-rms-radps "${PP_IDLE_MAX_QD_RMS_RADPS:-0.50}"
    --max-ctrl-step-rms-ratio "${PP_IDLE_MAX_CTRL_STEP_RMS_RATIO:-0.03}"
    --max-ctrl-step-peak-ratio "${PP_IDLE_MAX_CTRL_STEP_PEAK_RATIO:-0.20}"
    --max-ctrl-saturation-fraction "${PP_IDLE_MAX_CTRL_SATURATION_FRACTION:-0.0}"
  )
fi
if [ "${PP_REQUIRE_HOME_SUPPORT_FRAME:-0}" = "1" ]; then
  PLANT_REPORT_ARGS+=(
    --require-home-support-frame
    --max-home-base-yaw-peak-deg "${PP_HOME_MAX_BASE_YAW_PEAK_DEG:-12.0}"
    --max-home-base-yaw-end-deg "${PP_HOME_MAX_BASE_YAW_END_DEG:-5.0}"
    --max-home-support-yaw-peak-deg "${PP_HOME_MAX_SUPPORT_YAW_PEAK_DEG:-12.0}"
    --max-home-support-yaw-end-deg "${PP_HOME_MAX_SUPPORT_YAW_END_DEG:-5.0}"
    --max-home-support-midpoint-peak-m "${PP_HOME_MAX_SUPPORT_MIDPOINT_PEAK_M:-0.10}"
    --max-home-support-midpoint-end-m "${PP_HOME_MAX_SUPPORT_MIDPOINT_END_M:-0.03}"
    --max-home-foot-anchor-peak-m "${PP_HOME_MAX_FOOT_ANCHOR_PEAK_M:-0.12}"
    --max-home-foot-anchor-end-m "${PP_HOME_MAX_FOOT_ANCHOR_END_M:-0.03}"
    --max-home-foot-heading-peak-deg "${PP_HOME_MAX_FOOT_HEADING_PEAK_DEG:-15.0}"
    --max-home-foot-heading-end-deg "${PP_HOME_MAX_FOOT_HEADING_END_DEG:-3.0}"
    --max-home-foot-tilt-peak-deg "${PP_HOME_MAX_FOOT_TILT_PEAK_DEG:-12.0}"
    --max-home-foot-tilt-end-deg "${PP_HOME_MAX_FOOT_TILT_END_DEG:-5.0}"
    --min-home-signed-width-m "${PP_HOME_MIN_SIGNED_WIDTH_M:-0.26}"
    --max-home-signed-width-m "${PP_HOME_MAX_SIGNED_WIDTH_M:-0.32}"
    --max-home-level2-width-allowance-m "${PP_HOME_MAX_LEVEL2_WIDTH_ALLOWANCE_M:-0.08}"
    --max-home-fore-aft-stagger-m "${PP_HOME_MAX_FORE_AFT_STAGGER_M:-0.10}"
  )
fi
if [ "${PP_REQUIRE_TORSO_HOLD:-0}" = "1" ]; then
  PLANT_REPORT_ARGS+=(
    --require-torso-hold
    --min-torso-hold-window-s "${PP_TORSO_HOLD_MIN_WINDOW_S:-0.20}"
    --torso-hold-settle-trim-s "${PP_TORSO_HOLD_SETTLE_TRIM_S:-0.12}"
    --max-torso-hold-pitch-median-deg "${PP_TORSO_HOLD_MAX_PITCH_MEDIAN_DEG:-3.0}"
    --max-torso-hold-pitch-p95-deg "${PP_TORSO_HOLD_MAX_PITCH_P95_DEG:-5.0}"
    --max-torso-hold-pitch-peak-deg "${PP_TORSO_HOLD_MAX_PITCH_PEAK_DEG:-8.0}"
  )
  if [ "${PP_REQUIRE_TORSO_HOLD_THREE_AXIS:-0}" = "1" ]; then
    PLANT_REPORT_ARGS+=(
      --require-torso-hold-three-axis
      --max-torso-hold-roll-median-deg "${PP_TORSO_HOLD_MAX_ROLL_MEDIAN_DEG:-3.0}"
      --max-torso-hold-roll-p95-deg "${PP_TORSO_HOLD_MAX_ROLL_P95_DEG:-5.0}"
      --max-torso-hold-roll-peak-deg "${PP_TORSO_HOLD_MAX_ROLL_PEAK_DEG:-8.0}"
      --max-torso-hold-yaw-median-deg "${PP_TORSO_HOLD_MAX_YAW_MEDIAN_DEG:-3.0}"
      --max-torso-hold-yaw-p95-deg "${PP_TORSO_HOLD_MAX_YAW_P95_DEG:-5.0}"
      --max-torso-hold-yaw-peak-deg "${PP_TORSO_HOLD_MAX_YAW_PEAK_DEG:-8.0}"
    )
  fi
fi
if [ "${PP_REQUIRE_OPTIONAL_REACH:-0}" = "auto" ]; then
  PP_REQUIRE_OPTIONAL_REACH="$(python3 - /tmp/pp_runner_trace.csv <<'PY'
import csv
import sys
try:
    with open(sys.argv[1], newline='') as stream:
        rows = csv.DictReader(stream)
        enabled = 'reach_level' in (rows.fieldnames or ()) and any(
            abs(float(row.get('reach_level', 0.0))) > 0.5 for row in rows
        )
    print(1 if enabled else 0)
except (OSError, TypeError, ValueError):
    print(0)
PY
)"
fi
if [ "${PP_REQUIRE_OPTIONAL_REACH:-0}" = "1" ]; then
  PLANT_REPORT_ARGS+=(--require-optional-reach)
fi
if [ "${PP_REQUIRE_COLLISION_LEDGER:-0}" = "1" ]; then
  PLANT_REPORT_ARGS+=(--require-collision-ledger)
fi
python3 "$GEAR/scripts/pp_mujoco_plant_report.py" "${PLANT_REPORT_ARGS[@]}"
PLANT_REPORT_RC=$?
if [ "$PLANT_REPORT_RC" -ne 0 ] && \
   { [ "${PP_REQUIRE_PLANT_TRACE:-0}" = "1" ] || \
     [ "${PP_REQUIRE_IDLE_SMOOTHNESS:-0}" = "1" ] || \
     [ "${PP_REQUIRE_HOME_SUPPORT_FRAME:-0}" = "1" ] || \
     [ "${PP_REQUIRE_TORSO_HOLD:-0}" = "1" ] || \
     [ "${PP_REQUIRE_OPTIONAL_REACH:-0}" = "1" ] || \
     [ "${PP_REQUIRE_COLLISION_LEDGER:-0}" = "1" ]; }; then
  echo "[g3r] MuJoCo plant trace FAILED (rc=$PLANT_REPORT_RC)"
  RC=$PLANT_REPORT_RC
fi
echo "--- Final obs/phase gate ---"
REPORT_MODE="${PP_RALLY_REPORT_MODE:-auto}"
case "$REPORT_MODE" in
  auto|legacy|rally_final_v3|rally_v8|rally_v9|rally_v10|rally_v11|rally_v12|rally_v13|rally_v14|hitter_pingpong_recovery_tts_v2|hitter_pingpong_recovery_tts_v3|hitter_pingpong_continuous_rally_v2|hitter_pingpong_build2_minimal_rootfix_v1|hitter_pingpong_build2_rapid_preempt_v1|hitter_pingpong_build2_fixed_home_feasible_v1|hitter_pingpong_build2_fixed_home_phase_recovery_v1|hitter_pingpong_build2_home_support_frame_v1|hitter_pingpong_build2_torso_optional_reach_v2|schema28|schema29|schema31|schema32|schema33|schema34|rally_v15|rally_v17|rally_v17_r10) ;;
  *)
    echo "[g3r] invalid PP_RALLY_REPORT_MODE='$REPORT_MODE'"
    REPORT_MODE="__invalid__"
    ;;
esac
if [ "$REPORT_MODE" = "__invalid__" ]; then
  REPORT_RC=2
else
  python3 "$GEAR/scripts/pp_rally_report.py" \
    /tmp/pp_obs.csv /tmp/pp_rally_report.json \
    --mode "$REPORT_MODE" --runner-log /tmp/pp_runner.log --runner-cwd "$DIST" \
    --runner-trace /tmp/pp_runner_trace.csv
  REPORT_RC=$?
fi
if [ "$REPORT_RC" -eq 2 ]; then
  echo "[g3r] pp_rally_report CONFIG/RECIPE FAIL (rc=$REPORT_RC; never advisory)"
  RC=$REPORT_RC
elif [ "$REPORT_RC" -ne 0 ] && \
     { [ "${PP_REQUIRE_READY:-0}" = "1" ] || [ "${PP_REQUIRE_RALLY_REPORT:-0}" = "1" ]; }; then
  echo "[g3r] pp_rally_report FAILED (rc=$REPORT_RC)"
  RC=$REPORT_RC
elif [ "$REPORT_RC" -ne 0 ]; then
  echo "[g3r] pp_rally_report advisory FAIL (set PP_REQUIRE_READY=1 for Final certification)"
fi
# Cleanup evidence remains authoritative even if a later report was provisional.
python3 "$SCRIPT_DIR/pp_gate3_drain_flights.py" --finalize-report /tmp/pp_rally_report.json
DRAIN_REPORT_RC=$?
if [ "$DRAIN_REPORT_RC" -ne 0 ]; then RC=$DRAIN_REPORT_RC; fi
echo "==================================================================="
exit $RC
