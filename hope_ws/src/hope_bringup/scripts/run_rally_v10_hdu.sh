#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_rally_v10_hdu.sh --mocap-ip HOST [options]

Required:
  --mocap-ip HOST            Mocap server address on the group-control Wi-Fi:
                             the CMTracker/VRPN PC (avatar_pro backend) or the
                             Motive PC (optitrack backend).
                             --vrpn-ip HOST is a back-compat alias.

Options:
  --mocap-backend NAME       avatar_pro (default, ChingMu/VRPN) or optitrack
                             (Motive/NatNet via the vendored
                             motion_capture_tracking driver).
  --mocap-interface-ip IPV4  HDU/laptop local IPv4 on the Motive network.
                             Required for optitrack multicast; not the Motive IP.
  --x-hit METRES             Already-confirmed fixed world plane; bypasses the runtime freeze.
  --x-hit-offset METRES      Settled base-X to fixed-plane offset (default 0.58).
  --side MODE                backhand (default), forehand, or both.
  --marker-to-base X,Y,Z     Legacy frame-probe translation only (default 0,0,0).
                             Runtime full-pose calibration comes exclusively from
                             hope_world_frame.yaml and requires both receipts.
  --position-scale SCALE     0.001 for millimetres, 1.0 for metres.
                             Default per backend: avatar_pro 0.001 (venue VRPN
                             streams mm), optitrack 1.0 (Motive streams metres).
  --port PORT                VRPN port (avatar_pro only; default 3883). The
                             NatNet command port is fixed in optitrack_mct.yaml.
  --update-freq HZ           Mocap camera rate. avatar_pro: VRPN poll rate
                             (default 300.0). optitrack: expected Motive rate
                             (default 300.0) — probe threshold only; Motive
                             owns the actual rate.
  --fit-window N             Deprecated compatibility option. Converts N mocap
                             samples into the C++ packetizer flight window N/HZ.
  --solve-period SEC         Planner solve period (default 0.033, about 30 Hz).
  --workspace PATH           HDU ROS workspace (default $HOME/hope_ws).
  --session-id ID            Shared HDU/MDU log session ID. Default: UTC timestamp.
  --session-root PATH        Session root (default /tmp/hope_real).
  --frame-preflight          Require the static Ball/P1 frame probe.
  --skip-frame-preflight     Disable it (OptiTrack enables it by default).
  --preflight-only           Check host/network/packages without starting ROS nodes.
  --print-cmd                Print the three managed ROS commands and exit.
  -h, --help                 Show this help.

This entrypoint is HDU-only. It supervises the mocap bridge/relay, the C++
flight packetizer, and the model_21800 C++ planner as one foreground process
group. Without
--x-hit, racket commands are blocked until the robot is in PD_STAND and the
operator presses c in this HDU terminal to freeze x_hit from a stable base-X
window. The c control uses local /tmp files, not a new ROS/DDS CLI participant.
DDS is bound to HDU Ethernet 10.42.10.10; the vendor mocap transport (VRPN TCP
or NatNet UDP) remains direct on the group-control Wi-Fi. It never starts the
MDU runner.
EOF
}

die() {
  echo "[rally-v10-hdu] ERROR: $*" >&2
  exit 1
}

is_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}

print_shell_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

MOCAP_BACKEND="avatar_pro"
MOCAP_IP=""
MOCAP_INTERFACE_IP="${HOPE_MOTIVE_INTERFACE_IP:-}"
X_HIT=""
X_HIT_OFFSET="0.58"
SIDE="backhand"
# This value is deliberately NOT forwarded to the runtime base-pose relay. It
# exists only for the legacy one-shot frame probe/evidence JSON.
MARKER_TO_BASE="0.0,0.0,0.0"
# Empty = backend-derived default (avatar_pro: 0.001 / 3883 / 300.0;
# optitrack: 1.0 / no port / 300.0). Explicit flags always win.
POSITION_SCALE=""
PORT=""
UPDATE_FREQ=""
FIT_WINDOW=""
SOLVE_PERIOD="0.033"
WORKSPACE="${HOME}/hope_ws"
SESSION_ROOT="/tmp/hope_real"
SESSION_ID="${HOPE_REAL_SESSION_ID:-}"
FRAME_PREFLIGHT_MODE="auto"
PREFLIGHT_ONLY=0
PRINT_CMD=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mocap-backend)
      MOCAP_BACKEND="${2:-}"
      shift 2
      ;;
    --mocap-ip|--vrpn-ip)
      MOCAP_IP="${2:-}"
      shift 2
      ;;
    --mocap-interface-ip)
      MOCAP_INTERFACE_IP="${2:-}"
      shift 2
      ;;
    --x-hit)
      X_HIT="${2:-}"
      shift 2
      ;;
    --x-hit-offset)
      X_HIT_OFFSET="${2:-}"
      shift 2
      ;;
    --side)
      SIDE="${2:-}"
      shift 2
      ;;
    --marker-to-base)
      MARKER_TO_BASE="${2:-}"
      shift 2
      ;;
    --position-scale)
      POSITION_SCALE="${2:-}"
      shift 2
      ;;
    --port)
      PORT="${2:-}"
      shift 2
      ;;
    --update-freq)
      UPDATE_FREQ="${2:-}"
      shift 2
      ;;
    --fit-window)
      FIT_WINDOW="${2:-}"
      shift 2
      ;;
    --solve-period)
      SOLVE_PERIOD="${2:-}"
      shift 2
      ;;
    --workspace)
      WORKSPACE="${2:-}"
      shift 2
      ;;
    --session-id)
      SESSION_ID="${2:-}"
      shift 2
      ;;
    --session-root)
      SESSION_ROOT="${2:-}"
      shift 2
      ;;
    --frame-preflight)
      FRAME_PREFLIGHT_MODE="on"
      shift
      ;;
    --skip-frame-preflight)
      FRAME_PREFLIGHT_MODE="off"
      shift
      ;;
    --preflight-only)
      PREFLIGHT_ONLY=1
      shift
      ;;
    --print-cmd)
      PRINT_CMD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 64
      ;;
  esac
done

case "${MOCAP_BACKEND}" in
  avatar_pro)
    [[ -z "${MOCAP_INTERFACE_IP}" ]] ||
      die "--mocap-interface-ip/HOPE_MOTIVE_INTERFACE_IP is optitrack-only"
    [[ -n "${POSITION_SCALE}" ]] || POSITION_SCALE="0.001"
    [[ -n "${PORT}" ]] || PORT="3883"
    [[ -n "${UPDATE_FREQ}" ]] || UPDATE_FREQ="300.0"
    ;;
  optitrack)
    [[ -n "${MOCAP_INTERFACE_IP}" ]] ||
      die "--mocap-interface-ip (or HOPE_MOTIVE_INTERFACE_IP) is required for OptiTrack multicast"
    [[ "${MOCAP_INTERFACE_IP}" =~ ^([0-9]{1,3}[.]){3}[0-9]{1,3}$ ]] ||
      die "--mocap-interface-ip must be an IPv4 address"
    [[ -n "${POSITION_SCALE}" ]] || POSITION_SCALE="1.0"
    [[ -z "${PORT}" ]] ||
      die "--port is a VRPN (avatar_pro) flag; the NatNet command port is fixed in optitrack_mct.yaml"
    [[ -n "${UPDATE_FREQ}" ]] || UPDATE_FREQ="300.0"
    ;;
  *)
    die "--mocap-backend must be one of: avatar_pro, optitrack"
    ;;
esac

if [[ "${FRAME_PREFLIGHT_MODE}" == "auto" ]]; then
  [[ "${MOCAP_BACKEND}" == "optitrack" ]] && FRAME_PREFLIGHT=1 || FRAME_PREFLIGHT=0
elif [[ "${FRAME_PREFLIGHT_MODE}" == "on" ]]; then
  FRAME_PREFLIGHT=1
else
  FRAME_PREFLIGHT=0
fi
[[ -n "${SESSION_ID}" ]] || SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)"
[[ "${SESSION_ID}" =~ ^[A-Za-z0-9._-]+$ ]] ||
  die "--session-id may contain only letters, digits, dot, underscore and dash"
[[ -n "${SESSION_ROOT}" && "${SESSION_ROOT}" == /* ]] ||
  die "--session-root must be a non-empty absolute path"
SESSION_DIR="${SESSION_ROOT%/}/${SESSION_ID}/hdu"

[[ -n "${MOCAP_IP}" ]] || die "--mocap-ip (or --vrpn-ip) is required"
is_number "${X_HIT_OFFSET}" || die "--x-hit-offset must be numeric"
awk -v value="${X_HIT_OFFSET}" 'BEGIN {exit !(value > 0)}' || die "--x-hit-offset must be > 0"
if [[ -n "${X_HIT}" ]]; then
  is_number "${X_HIT}" || die "--x-hit must be numeric; got '${X_HIT}'"
  PLANNER_X_HIT="${X_HIT}"
  REQUIRE_CALIBRATION="false"
  CALIBRATION_LABEL="pre-confirmed x_hit=${X_HIT} (runtime freeze bypassed)"
else
  PLANNER_X_HIT="0.0"
  REQUIRE_CALIBRATION="true"
  CALIBRATION_LABEL="operator freeze required after PD_STAND"
fi
is_number "${POSITION_SCALE}" || die "--position-scale must be numeric"
is_number "${UPDATE_FREQ}" || die "--update-freq must be numeric"
is_number "${SOLVE_PERIOD}" || die "--solve-period must be numeric"
awk -v value="${POSITION_SCALE}" 'BEGIN {exit !(value > 0)}' || die "--position-scale must be > 0"
awk -v value="${UPDATE_FREQ}" 'BEGIN {exit !(value > 0)}' || die "--update-freq must be > 0"
awk -v value="${SOLVE_PERIOD}" 'BEGIN {exit !(value > 0)}' || die "--solve-period must be > 0"
if [[ "${MOCAP_BACKEND}" == "avatar_pro" ]]; then
  [[ "${PORT}" =~ ^[1-9][0-9]{0,4}$ ]] || die "--port must be a positive integer"
  ((10#${PORT} <= 65535)) || die "--port must be <= 65535"
fi
# Backward-compatible conversion of the former sample-count option. The active
# estimator window belongs to the C++ packetizer and is expressed in seconds.
if [[ -n "${FIT_WINDOW}" ]]; then
  [[ "${FIT_WINDOW}" =~ ^[1-9][0-9]*$ ]] || die "--fit-window must be a positive integer"
  FLIGHT_WINDOW_S="$(awk -v samples="${FIT_WINDOW}" -v rate="${UPDATE_FREQ}" 'BEGIN {printf "%.9g", samples / rate}')"
else
  FLIGHT_WINDOW_S="0.18"
fi

IFS=',' read -r MARKER_X MARKER_Y MARKER_Z MARKER_EXTRA <<<"${MARKER_TO_BASE}"
[[ -n "${MARKER_X:-}" && -n "${MARKER_Y:-}" && -n "${MARKER_Z:-}" && -z "${MARKER_EXTRA:-}" ]] ||
  die "--marker-to-base must contain exactly X,Y,Z"
is_number "${MARKER_X}" && is_number "${MARKER_Y}" && is_number "${MARKER_Z}" ||
  die "--marker-to-base values must be numeric"
MARKER_ROS="[${MARKER_X},${MARKER_Y},${MARKER_Z}]"

case "${SIDE}" in
  backhand)
    SIDE_SPLIT="-10.0"
    SIDE_HYSTERESIS="0.0"
    EXPECTED_SIGN="-1.0"
    ;;
  forehand)
    SIDE_SPLIT="10.0"
    SIDE_HYSTERESIS="0.0"
    EXPECTED_SIGN="+1.0"
    ;;
  both)
    SIDE_SPLIT="-0.25"
    SIDE_HYSTERESIS="0.04"
    EXPECTED_SIGN="+1.0 or -1.0"
    ;;
  *)
    die "--side must be one of: backhand, forehand, both"
    ;;
esac

PLANNER_CONFIG_DIR="${WORKSPACE}/src/hope_planner_cpp/config"
X_HIT_REQUEST_FILE="/tmp/hope_rally_v10_x_hit.request"
X_HIT_STATUS_FILE="/tmp/hope_rally_v10_x_hit.status"
if [[ "${MOCAP_BACKEND}" == "optitrack" ]]; then
  BRIDGE_CMD=(
    ros2 launch hope_bringup optitrack_hope_bridge.launch.py
    "motive_hostname:=${MOCAP_IP}"
    "motive_interface_ip:=${MOCAP_INTERFACE_IP}"
    "position_scale:=${POSITION_SCALE}"
    "debug_csv_path:=${SESSION_DIR}/mocap_raw.csv"
    "debug_session_id:=${SESSION_ID}"
    start_flight_packetizer:=false
  )
else
  BRIDGE_CMD=(
    ros2 run hope_bringup pose_to_posearray --ros-args
    -p 'input_topics:=[/vrpn_mocap/Ball/pose_id_0]'
    -p trigger_index:=0
  )
fi
PACKETIZER_CMD=(
  ros2 run hope_planner_cpp hope_ball_flight_packetizer --ros-args
  --params-file "${PLANNER_CONFIG_DIR}/model21800_flight_packetizer.yaml"
  -p "flight_window_s:=${FLIGHT_WINDOW_S}"
  -p "session_id:=${SESSION_ID}"
  -p "debug_csv_path:=${SESSION_DIR}/flight_packetizer.csv"
)
PLANNER_CMD=(
  ros2 run hope_planner_cpp hope_planner_cpp_node --ros-args
  --params-file "${PLANNER_CONFIG_DIR}/model21800_hardware.yaml"
  -p flight_packet_input_enabled:=true
  -p flight_packet_topic:=/ball/flight_packet
  -p base_pose_flat_input_topic:=/a3/base_pose_flat
  -p publish_base_flat:=false
  -p x_hit_follow_robot:=false
  -p "x_hit:=${PLANNER_X_HIT}"
  -p "require_x_hit_calibration:=${REQUIRE_CALIBRATION}"
  -p "x_hit_calibration_offset:=${X_HIT_OFFSET}"
  -p x_hit_bh_delta:=0.0
  -p "swing_side_split_y:=${SIDE_SPLIT}"
  -p "swing_side_hysteresis_y:=${SIDE_HYSTERESIS}"
  -p "solve_period_s:=${SOLVE_PERIOD}"
  -p "x_hit_calibration_request_file:=${X_HIT_REQUEST_FILE}"
  -p "x_hit_calibration_status_file:=${X_HIT_STATUS_FILE}"
  -p "debug_csv_path:=${SESSION_DIR}/planner.csv"
  -p "debug_session_id:=${SESSION_ID}"
)

if [[ "${PRINT_CMD}" -eq 1 ]]; then
  echo "[rally-v10-hdu] bridge:"
  print_shell_command "${BRIDGE_CMD[@]}"
  echo "[rally-v10-hdu] flight packetizer:"
  print_shell_command "${PACKETIZER_CMD[@]}"
  echo "[rally-v10-hdu] planner (${SIDE}, expected swing_sign ${EXPECTED_SIGN}):"
  print_shell_command "${PLANNER_CMD[@]}"
  echo "[rally-v10-hdu] session_id=${SESSION_ID} session_dir=${SESSION_DIR} frame_preflight=${FRAME_PREFLIGHT}"
  exit 0
fi

machine="$(uname -m)"
[[ "${machine}" == "aarch64" || "${machine}" == "arm64" ]] ||
  die "HDU must be arm64/aarch64; uname -m returned '${machine}'"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "/opt/ros/jazzy/setup.bash is missing"
[[ -f "${WORKSPACE}/install/local_setup.bash" ]] ||
  die "${WORKSPACE}/install/local_setup.bash is missing; deploy/build the HDU workspace first"
[[ -f "${PLANNER_CONFIG_DIR}/model21800_hardware.yaml" ]] || die "C++ planner config is missing"
[[ -f "${PLANNER_CONFIG_DIR}/model21800_flight_packetizer.yaml" ]] || die "C++ packetizer config is missing"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${WORKSPACE}/install/local_setup.bash"
set -u

command -v ros2 >/dev/null || die "ros2 is unavailable after sourcing Jazzy"
command -v python3 >/dev/null || die "python3 is required for the VRPN TCP preflight"
command -v tee >/dev/null || die "tee is required for the field session log"
command -v awk >/dev/null || die "awk is required for numeric/interface checks"
command -v ip >/dev/null || die "iproute2 is required to verify the HDU internal interface"
# NOTE: driver now lives in the standalone NatNet2ROS2/ (or VRPN2ROS2/) workspace;
# build and source that workspace into the HDU overlay so these packages resolve.
if [[ "${MOCAP_BACKEND}" == "optitrack" ]]; then
  ros2 pkg prefix motion_capture_tracking >/dev/null ||
    die "motion_capture_tracking is not installed in the HDU overlay (deploy without --skip-optitrack)"
  ros2 pkg prefix motion_capture_tracking_interfaces >/dev/null ||
    die "motion_capture_tracking_interfaces is not installed in the HDU overlay"
else
  ros2 pkg prefix vrpn_mocap >/dev/null || die "vrpn_mocap is not installed in the HDU overlay"
fi
ros2 pkg prefix hope_bringup >/dev/null || die "hope_bringup is not installed in the HDU overlay"
ros2 pkg prefix hope_planner_cpp >/dev/null || die "hope_planner_cpp is not installed in the HDU overlay"
DDS_PROFILE="$(ros2 pkg prefix hope_bringup)/share/hope_bringup/config/fastdds_hdu_internal.xml"
[[ -f "${DDS_PROFILE}" ]] || die "HDU Fast DDS profile is missing: ${DDS_PROFILE}; rebuild hope_bringup"
ip -o -4 addr show | awk '$4 ~ /^10[.]42[.]10[.]10\// {found=1} END {exit !found}' ||
  die "HDU internal DDS address 10.42.10.10 is not present"

export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
rm -f -- "${X_HIT_REQUEST_FILE}" "${X_HIT_STATUS_FILE}"

if [[ "${MOCAP_BACKEND}" == "optitrack" ]]; then
  route_line="$(ip -o route get "${MOCAP_IP}" 2>/dev/null | head -n 1)"
  [[ -n "${route_line}" ]] || die "no route to Motive PC ${MOCAP_IP}"
  route_src="$(awk '{for (i=1; i<=NF; ++i) if ($i == "src") {print $(i+1); exit}}' <<<"${route_line}")"
  [[ "${route_src}" == "${MOCAP_INTERFACE_IP}" ]] ||
    die "Motive route uses source ${route_src:-NONE}, not --mocap-interface-ip ${MOCAP_INTERFACE_IP}"
  ros2 run hope_bringup natnet_preflight.py \
    --hostname "${MOCAP_IP}" --interface-ip "${MOCAP_INTERFACE_IP}" ||
    die "NatNet 4.5 multicast preflight failed"
else
  python3 - "${MOCAP_IP}" "${PORT}" <<'PY'
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
try:
    with socket.create_connection((host, port), timeout=3.0):
        pass
except OSError as exc:
    raise SystemExit(f"VRPN TCP preflight failed for {host}:{port}: {exc}")
print(f"[rally-v10-hdu] VRPN TCP reachable: {host}:{port}")
PY
fi

# Stale processes from EITHER backend poison a run (both publish /poses).
for process_pattern in '[a]vatar_pro_hope_bridge.launch.py' '[a]vatar_pro_vrpn_relay' \
    '[o]ptitrack_hope_bridge.launch.py' '[o]ptitrack_mct_relay' \
    '[m]otion_capture_tracking_node' '[h]ope_planner_cpp_node' \
    '[h]ope_ball_flight_packetizer' \
    '[h]ope_base_pose_flat_relay'; do
  if pgrep -f "${process_pattern}" >/dev/null; then
    die "an existing process matches ${process_pattern}; stop it explicitly before starting this supervisor"
  fi
done

echo "[rally-v10-hdu] preflight PASS: backend=${MOCAP_BACKEND} domain=232 workspace=${WORKSPACE} side=${SIDE}"
echo "[rally-v10-hdu] DDS bound to 10.42.10.10 via ${DDS_PROFILE}; solve_period=${SOLVE_PERIOD}s"
echo "[rally-v10-hdu] ${CALIBRATION_LABEL}; frame_probe_marker_to_base=${MARKER_ROS} position_scale=${POSITION_SCALE}"
if [[ "${PREFLIGHT_ONLY}" -eq 1 ]]; then
  exit 0
fi

mkdir -p -- "${SESSION_DIR}"
python3 - "${SESSION_DIR}/session.json" "${SESSION_ID}" "$$" "${MOCAP_BACKEND}" \
    "${MOCAP_IP}" "${MOCAP_INTERFACE_IP}" "${POSITION_SCALE}" \
    "${UPDATE_FREQ}" "${FLIGHT_WINDOW_S}" \
    "${SOLVE_PERIOD}" "${MARKER_TO_BASE}" "${PLANNER_X_HIT}" "${X_HIT_OFFSET}" \
    "${WORKSPACE}" <<'PY'
import hashlib
import json
import pathlib
import socket
import sys
import time

(output, session_id, launcher_pid, backend, mocap_ip, mocap_interface_ip,
 scale, rate, flight_window_s, solve_period,
 marker_to_base, x_hit, x_hit_offset, workspace) = sys.argv[1:]
workspace_path = pathlib.Path(workspace)
sources = {
    "planner_node": workspace_path / "src/hope_planner_cpp/src/planner_node.cpp",
    "planner_yaml": workspace_path / "src/hope_planner_cpp/config/model21800_hardware.yaml",
    "flight_packetizer": workspace_path / "src/hope_planner_cpp/src/flight_packetizer_node.cpp",
    "flight_packetizer_yaml": workspace_path / "src/hope_planner_cpp/config/model21800_flight_packetizer.yaml",
    "optitrack_relay": workspace_path / "src/hope_bringup/scripts/optitrack_mct_relay",
    "world_frame_yaml": workspace_path / "src/hope_bringup/config/hope_world_frame.yaml",
}
hashes = {}
for name, path in sources.items():
    if path.is_file():
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
report = {
    "schema_version": 2,
    "session_id": session_id,
    "component": "hdu",
    "hostname": socket.gethostname(),
    "launcher_pid": int(launcher_pid),
    "start_wall_time_ns": time.time_ns(),
    "mocap_backend": backend,
    "mocap_ip": mocap_ip,
    "mocap_interface_ip": mocap_interface_ip or None,
    "position_scale": float(scale),
    "update_frequency_hz": float(rate),
    "flight_window_s": float(flight_window_s),
    "solve_period_s": float(solve_period),
    "frame_probe_marker_to_base_xyz": [
        float(value) for value in marker_to_base.split(",")
    ],
    "runtime_base_pose_contract": "hope_world_frame_yaml_receipts_schema2",
    "policy_z_offset_m": 0.76,
    "x_hit_m": float(x_hit),
    "x_hit_calibration_offset_m": float(x_hit_offset),
    "source_sha256": hashes,
}
pathlib.Path(output).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
PY
exec > >(tee -a "${SESSION_DIR}/hdu.log") 2>&1
echo "[rally-v10-hdu] session_id=${SESSION_ID} logs=${SESSION_DIR}"

BRIDGE_PID=""
PACKETIZER_PID=""
PLANNER_PID=""
CONTROL_PID=""

stop_children() {
  local pid
  if [[ -n "${CONTROL_PID}" ]] && kill -0 "${CONTROL_PID}" 2>/dev/null; then
    kill -TERM "${CONTROL_PID}" 2>/dev/null || true
  fi
  for pid in "${PLANNER_PID}" "${PACKETIZER_PID}" "${BRIDGE_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${PLANNER_PID}" "${PACKETIZER_PID}" "${BRIDGE_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
  if [[ -n "${CONTROL_PID}" ]]; then
    wait "${CONTROL_PID}" 2>/dev/null || true
  fi
}

on_signal() {
  trap - INT TERM
  stop_children
  exit 130
}

trap stop_children EXIT
trap on_signal INT TERM

calibration_key_loop() {
  local key request_id status matched
  while IFS= read -r -s -n1 key; do
    if [[ "${key}" != "c" && "${key}" != "C" ]]; then
      continue
    fi
    echo
    echo "[rally-v10-hdu] c pressed: checking the settled base-X window"
    request_id="$(date +%s%N)"
    printf '%s\n' "${request_id}" >"${X_HIT_REQUEST_FILE}"
    matched=0
    for _ in $(seq 1 50); do
      sleep 0.1
      if [[ -f "${X_HIT_STATUS_FILE}" ]] &&
          grep -Fqx "request=${request_id}" "${X_HIT_STATUS_FILE}"; then
        matched=1
        break
      fi
    done
    if [[ "${matched}" -eq 0 ]]; then
      echo "[rally-v10-hdu] X-HIT request timed out; planner did not answer the local control file" >&2
      continue
    fi
    status="$(cat "${X_HIT_STATUS_FILE}")"
    echo "${status}"
    if grep -Fqx 'success=1' "${X_HIT_STATUS_FILE}"; then
      echo "[rally-v10-hdu] X-HIT FROZEN. Watch the in-process HDU HEALTH lines; no ros2 topic echo/hz is needed."
      echo "[rally-v10-hdu] MOTION READY: return to the MDU runner and press m when the field team is ready"
    else
      echo "[rally-v10-hdu] X-HIT STILL BLOCKED: keep the robot untouched, wait 2 s, then press c again" >&2
    fi
  done
}

echo "[rally-v10-hdu] starting bridge + relay"
"${BRIDGE_CMD[@]}" &
BRIDGE_PID=$!
sleep 2
kill -0 "${BRIDGE_PID}" 2>/dev/null || die "bridge exited during startup"

if [[ "${MOCAP_BACKEND}" == "optitrack" ]]; then
  # NatNet liveness gate (one-shot probe node, the sanctioned pattern — NOT a
  # long-running ros2 topic echo/hz): the robot rigid body streams whenever
  # Motive is truly up, so /P1/pose is the hard requirement. The ball may
  # legitimately be out of the volume -> soft warning only.
  P1_MIN_HZ="$(awk -v rate="${UPDATE_FREQ}" 'BEGIN {printf "%.1f", rate * 0.5}')"
  ros2 run hope_bringup mocap_rate_probe.py \
      --topic /P1/pose --min-hz "${P1_MIN_HZ}" --window 5 --discover-timeout 15 || {
    kill -INT "${BRIDGE_PID}" 2>/dev/null || true
    die "no /P1/pose stream from the OptiTrack bridge (Motive not streaming, P1 asset missing/misnamed, or NatNet blocked)"
  }
  ros2 run hope_bringup mocap_rate_probe.py \
      --topic /ball/point --min-hz 1 --window 3 --discover-timeout 5 ||
    echo "[rally-v10-hdu] WARN: no /ball/point data yet (ball out of the volume, or the mct 'Ball' tracker has not locked)"
fi

if [[ "${FRAME_PREFLIGHT}" -eq 1 ]]; then
  echo "[rally-v10-hdu] FRAME PREFLIGHT: keep Ball motionless on the table while samples are collected"
  FRAME_PROBE_CMD=(
    ros2 run hope_bringup mocap_frame_probe.py
    --marker-to-base "${MARKER_TO_BASE}"
    --policy-z-offset 0.76
    --json "${SESSION_DIR}/frame_preflight.json"
  )
  "${FRAME_PROBE_CMD[@]}" || {
    kill -INT "${BRIDGE_PID}" 2>/dev/null || true
    die "mocap frame preflight failed; planner was not started (inspect ${SESSION_DIR}/frame_preflight.json)"
  }
  echo "[rally-v10-hdu] FRAME PREFLIGHT PASS"
fi

echo "[rally-v10-hdu] starting C++ flight packetizer + Planner"
"${PACKETIZER_CMD[@]}" &
PACKETIZER_PID=$!
sleep 1
kill -0 "${PACKETIZER_PID}" 2>/dev/null || die "flight packetizer exited during startup"

"${PLANNER_CMD[@]}" &
PLANNER_PID=$!
sleep 2
kill -0 "${PLANNER_PID}" 2>/dev/null || die "planner exited during startup"

echo "[rally-v10-hdu] READY: bridge_pid=${BRIDGE_PID} packetizer_pid=${PACKETIZER_PID} planner_pid=${PLANNER_PID}"
echo "[rally-v10-hdu] use the planner's 1 Hz HDU HEALTH line; do NOT run ros2 topic echo/hz on the HDU"
if [[ "${REQUIRE_CALIBRATION}" == "true" ]]; then
  [[ -t 0 ]] || die "runtime x_hit freeze requires an interactive TTY (use ssh -tt)"
  echo "[rally-v10-hdu] X-HIT BLOCKED: in the MDU runner press s, leave the robot untouched for 2 s,"
  echo "[rally-v10-hdu] then return HERE and press c once. Do not press m until success=True."
  calibration_key_loop </dev/tty &
  CONTROL_PID=$!
else
  echo "[rally-v10-hdu] X-HIT READY from explicit --x-hit=${PLANNER_X_HIT}"
fi
echo "[rally-v10-hdu] before MOTION, throw a ball and require /racket/command_flat data[2]=${EXPECTED_SIGN}"

set +e
wait -n "${BRIDGE_PID}" "${PACKETIZER_PID}" "${PLANNER_PID}"
child_status=$?
set -e
echo "[rally-v10-hdu] a managed process exited (status=${child_status}); stopping the remaining process" >&2
if [[ "${child_status}" -eq 0 ]]; then
  exit 1
fi
exit "${child_status}"
