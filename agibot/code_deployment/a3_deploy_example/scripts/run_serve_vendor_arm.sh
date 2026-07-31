#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/serve_vendor_arm_common.sh"

readonly SERVE_VENDOR_ARM_PROCESS_MANAGER_PORT="50080"
readonly SERVE_VENDOR_ARM_PROCESS_MANAGER_JSON="http://127.0.0.1:50080/json"
readonly SERVE_VENDOR_ARM_MOTION_PLAYER_PORT="56444"
readonly SERVE_VENDOR_ARM_MOTION_PLAYER_RPC="http://127.0.0.1:56444/rpc/aimdk.protocol.MotionCommandService"
readonly SERVE_VENDOR_ARM_ACTION_RPC="http://127.0.0.1:56322/rpc/aimdk.protocol.MotionControlActionService/GetAction"
readonly SERVE_VENDOR_ARM_STATE_TOPIC="/motion/control/arm_joint_state"
readonly SERVE_VENDOR_ARM_COMMAND_TOPIC="/motion/control/arm_joint_command"
readonly SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256="2a7de3f1c97a300069899c139c9eb96e94fd61d3419701d5e44ef37b2bf6641d"

SERVE_VENDOR_ARM_RUNNER_PID=""
SERVE_VENDOR_ARM_HANDOFF_DIR=""
SERVE_VENDOR_ARM_READY_FILE=""
SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
SERVE_VENDOR_ARM_CLEANUP_ACTIVE=0

serve_vendor_arm_usage() {
  cat <<'EOF'
Usage:
  run_serve_vendor_arm.sh
  run_serve_vendor_arm.sh --preflight-only
  run_serve_vendor_arm.sh --hold-only --confirm-real-commands
  run_serve_vendor_arm.sh --prepare-only --confirm-real-commands
  run_serve_vendor_arm.sh --serve-only --confirm-real-commands

Default and --preflight-only are read-only. They validate the package,
trajectory, robot action, arm topics, and motion_player state. They neither
stop motion_player nor create an arm command publisher.

A3_VENDOR_ARM_QUICK_DEPLOY=1 only shortens the default read-only preflight for
an isolated field directory. Real modes always use the lean runtime path; run
the no-argument preflight once after each deploy for package/ELF/topic audit.

--hold-only captures and holds the measured 14-joint arm state for three
seconds, exits, and restores motion_player.

--prepare-only moves both arms from their measured entry state to the
CSV-derived serve-ready pose. The left wrist-roll stays at its measured entry
value because the CSV value exceeds the high-level A3 arm limit. READY is held
until Ctrl-C; Ctrl-C stops the custom publisher before motion_player is
restored.

--serve-only moves both arms to the same serve-ready pose. After the runner
prints "READY HOLD", press Space at the physical ball-release instant
(physical t=0). The runner holds READY for 1.000 seconds after Space, then
plays the original CSV stroke at its previous 100 Hz command timing and
amplitude; the nominal strike is about physical t=+1.060 seconds.
EOF
}

serve_vendor_arm_temp_file() {
  mktemp "${TMPDIR:-/tmp}/a3-serve-vendor-arm.XXXXXX"
}

serve_vendor_arm_motion_rpc() {
  local method="$1"
  local output="$2"
  curl -fsS \
    --connect-timeout 2 \
    --max-time 5 \
    -H 'content-type:application/json' \
    -H 'timeout: 5000' \
    -X POST \
    "${SERVE_VENDOR_ARM_MOTION_PLAYER_RPC}/${method}" \
    --data '{}' \
    -o "${output}"
}

serve_vendor_arm_process_manager_request() {
  local operation="$1"
  local output="$2"
  curl -fsS \
    --connect-timeout 2 \
    --max-time 10 \
    -H 'content-type:application/json' \
    -X POST \
    "${SERVE_VENDOR_ARM_PROCESS_MANAGER_JSON}/${operation}" \
    --data '{"app_name":"motion_player"}' \
    -o "${output}"
}

serve_vendor_arm_parse_process_manager_success() {
  local path="$1"
  python3 - "${path}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        payload = json.load(stream)
except (OSError, json.JSONDecodeError) as exc:
    print(f"invalid process-manager JSON: {exc}", file=sys.stderr)
    raise SystemExit(1)

def reject_explicit_failure(value):
    if isinstance(value, dict):
        for raw_key, child in value.items():
            key = str(raw_key).lower()
            if key in ("success", "ok") and child is False:
                return f"{raw_key}=false"
            if key in ("code", "error_code", "status_code"):
                if isinstance(child, int) and child not in (0, 200):
                    return f"{raw_key}={child!r}"
                if isinstance(child, str):
                    lowered = child.lower()
                    if child.isdigit() and child not in ("0", "200"):
                        return f"{raw_key}={child!r}"
                    if "fail" in lowered or "error" in lowered:
                        return f"{raw_key}={child!r}"
            if key in ("result", "state", "status") and isinstance(child, str):
                lowered = child.lower()
                if "fail" in lowered or "error" in lowered:
                    return f"{raw_key}={child!r}"
            if key in ("error", "err", "message", "msg") and isinstance(child, str):
                lowered = child.lower()
                if "fail" in lowered or "error" in lowered:
                    return f"{raw_key}={child!r}"
            nested = reject_explicit_failure(child)
            if nested:
                return nested
    elif isinstance(value, list):
        for child in value:
            nested = reject_explicit_failure(child)
            if nested:
                return nested
    return None


failure = reject_explicit_failure(payload)
if failure:
    print(f"process-manager reported failure: {failure}", file=sys.stderr)
    raise SystemExit(1)
PY
}

serve_vendor_arm_listener_count() {
  local port="$1"
  local listeners=""
  command -v ss >/dev/null ||
    return 1
  listeners="$(
    LC_ALL=C LANG=C \
      ss -H -lnt "sport = :${port}" \
      2>/dev/null ||
      true
  )"
  awk 'NF {count += 1} END {print count + 0}' <<<"${listeners}"
}

serve_vendor_arm_require_control_endpoints() {
  local pm_listeners=""
  local player_listeners=""
  pm_listeners="$(
    serve_vendor_arm_listener_count \
      "${SERVE_VENDOR_ARM_PROCESS_MANAGER_PORT}"
  )" ||
    serve_script_die \
      "ss is required to inspect the process-manager endpoint"
  player_listeners="$(
    serve_vendor_arm_listener_count \
      "${SERVE_VENDOR_ARM_MOTION_PLAYER_PORT}"
  )" ||
    serve_script_die "cannot inspect the motion_player endpoint"
  [[ "${pm_listeners}" == "1" ]] ||
    serve_script_die \
      "process_manager must have exactly one listener on 50080; got ${pm_listeners}"
  [[ "${player_listeners}" == "1" ]] ||
    serve_script_die \
      "motion_player must have exactly one listener on 56444; got ${player_listeners}"
  echo \
    "[serve-vendor-arm] control endpoints PASS: process_manager=50080 motion_player=56444"
}

serve_vendor_arm_parse_motion_status() {
  local path="$1"
  python3 - "${path}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        payload = json.load(stream)
except (OSError, json.JSONDecodeError) as exc:
    print(f"invalid GetMotionStatus JSON: {exc}", file=sys.stderr)
    raise SystemExit(1)

header = payload.get("header")
if not isinstance(header, dict) or str(header.get("code")) != "0":
    print(
        f"GetMotionStatus header.code must be 0; got "
        f"{header.get('code') if isinstance(header, dict) else None!r}",
        file=sys.stderr,
    )
    raise SystemExit(1)
status = payload.get("status")
if status not in ("MotionCommandStatus_IDLE", "MotionCommandStatus_STOP"):
    print(f"unexpected motion_player status: {status!r}", file=sys.stderr)
    raise SystemExit(1)
print(status)
PY
}

serve_vendor_arm_get_motion_status() {
  local output=""
  local status=""
  output="$(serve_vendor_arm_temp_file)"
  if ! serve_vendor_arm_motion_rpc GetMotionStatus "${output}"; then
    rm -f -- "${output}"
    return 1
  fi
  if ! status="$(serve_vendor_arm_parse_motion_status "${output}")"; then
    rm -f -- "${output}"
    return 1
  fi
  rm -f -- "${output}"
  printf '%s\n' "${status}"
}

serve_vendor_arm_require_motion_player_idle() {
  local status=""
  status="$(serve_vendor_arm_get_motion_status)" ||
    serve_script_die "GetMotionStatus failed on 127.0.0.1:56444"
  [[ "${status}" == "MotionCommandStatus_IDLE" ]] ||
    serve_script_die \
      "motion_player must be IDLE before handoff; got ${status}"
  echo "[serve-vendor-arm] motion_player state PASS: IDLE"
}

serve_vendor_arm_motion_player_pids() {
  local proc_dir=""
  local command_line=""
  for proc_dir in /proc/[0-9]*; do
    [[ -d "${proc_dir}" ]] || continue
    command_line="$(tr '\0' ' ' <"${proc_dir}/cmdline" 2>/dev/null || true)"
    if [[ "${command_line}" != \
            *"/scripts/motion_player/start_motion_player.sh"* &&
          "${command_line}" != *"/config/motion_player/"* &&
          "${command_line}" != *"motion_player_a3_t2d0.yaml"* ]]; then
      continue
    fi
    printf '%s\n' "${proc_dir#/proc/}"
  done
}

serve_vendor_arm_motion_player_process_count() {
  local pids=""
  pids="$(serve_vendor_arm_motion_player_pids)"
  if [[ -z "${pids}" ]]; then
    printf '0\n'
  else
    wc -l <<<"${pids}" | tr -d ' '
  fi
}

serve_vendor_arm_require_motion_player_process() {
  local count=""
  count="$(serve_vendor_arm_motion_player_process_count)"
  [[ "${count}" =~ ^[1-9][0-9]*$ ]] ||
    serve_script_die \
      "motion_player process tree is missing; got ${count} matching processes"
  echo "[serve-vendor-arm] motion_player process tree PASS: processes=${count}"
}

serve_vendor_arm_motion_player_is_up() {
  local process_count=""
  local listener_count=""
  local publisher_count=""
  local status=""
  process_count="$(serve_vendor_arm_motion_player_process_count)"
  listener_count="$(
    serve_vendor_arm_listener_count \
      "${SERVE_VENDOR_ARM_MOTION_PLAYER_PORT}"
  )" || return 1
  publisher_count="$(
    serve_vendor_arm_publisher_count \
      "${SERVE_VENDOR_ARM_COMMAND_TOPIC}"
  )" || return 1
  status="$(serve_vendor_arm_get_motion_status 2>/dev/null || true)"
  [[ "${process_count}" =~ ^[1-9][0-9]*$ &&
     "${listener_count}" == "1" &&
     "${publisher_count}" == "1" &&
     "${status}" == "MotionCommandStatus_IDLE" ]]
}

serve_vendor_arm_motion_player_is_down() {
  local process_count=""
  local listener_count=""
  local publisher_count=""
  process_count="$(serve_vendor_arm_motion_player_process_count)"
  listener_count="$(
    serve_vendor_arm_listener_count \
      "${SERVE_VENDOR_ARM_MOTION_PLAYER_PORT}"
  )" || return 1
  publisher_count="$(
    serve_vendor_arm_publisher_count \
      "${SERVE_VENDOR_ARM_COMMAND_TOPIC}"
  )" || return 1
  [[ "${process_count}" == "0" &&
     "${listener_count}" == "0" &&
     "${publisher_count}" == "0" ]]
}

serve_vendor_arm_motion_player_process_and_listener_are_down() {
  local process_count=""
  local listener_count=""
  process_count="$(serve_vendor_arm_motion_player_process_count)"
  listener_count="$(
    serve_vendor_arm_listener_count \
      "${SERVE_VENDOR_ARM_MOTION_PLAYER_PORT}"
  )" || return 1
  [[ "${process_count}" == "0" && "${listener_count}" == "0" ]]
}

serve_vendor_arm_wait_fast_process_release() {
  local attempt=""
  for ((attempt = 1; attempt <= 50; ++attempt)); do
    serve_vendor_arm_motion_player_process_and_listener_are_down &&
      return 0
    sleep 0.02
  done
  echo \
    "motion_player process/listener did not stop within one second" \
    >&2
  return 1
}

serve_vendor_arm_wait_motion_player_ownership() {
  local expected="$1"
  local attempts="${2:-100}"
  local attempt=""
  for ((attempt = 1; attempt <= attempts; ++attempt)); do
    case "${expected}" in
      up)
        serve_vendor_arm_motion_player_is_up && return 0
        ;;
      down)
        serve_vendor_arm_motion_player_is_down && return 0
        ;;
      *)
        echo "invalid internal ownership state: ${expected}" >&2
        return 64
        ;;
    esac
    sleep 0.1
  done
  echo "motion_player ownership did not become ${expected}" >&2
  return 1
}

serve_vendor_arm_stop_motion_player_app() {
  local output=""
  output="$(serve_vendor_arm_temp_file)"

  # Set this before the request: a timed-out request may still have taken
  # effect, so cleanup must inspect and restore the app.
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=1
  if ! serve_vendor_arm_process_manager_request stop_app "${output}"; then
    rm -f -- "${output}"
    echo "[serve-vendor-arm] process-manager stop_app transport failed" >&2
    return 1
  fi
  if ! serve_vendor_arm_parse_process_manager_success "${output}"; then
    rm -f -- "${output}"
    return 1
  fi
  rm -f -- "${output}"
  # Do not invoke a fresh ros2 CLI node here.  On this A3 firmware the arm
  # state stream stops together with motion_player, so the prearmed runner
  # must be released with minimum latency.  It independently requires the
  # vendor publisher graph count to reach zero before creating its publisher.
  serve_vendor_arm_wait_fast_process_release ||
    return 1
  echo \
    "[serve-vendor-arm] motion_player process release PASS: process=0 listener=0; runner is confirming publisher=0"
}

serve_vendor_arm_start_motion_player_app() {
  local output=""
  if serve_vendor_arm_motion_player_is_up; then
    SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
    echo \
      "[serve-vendor-arm] motion_player was already restored: process=1 listener=1 publisher=1"
    return 0
  fi
  if ! serve_vendor_arm_wait_motion_player_ownership down 100; then
    echo \
      "[serve-vendor-arm] CRITICAL: refusing start_app from mixed ownership state" \
      >&2
    return 1
  fi

  output="$(serve_vendor_arm_temp_file)"
  if ! serve_vendor_arm_process_manager_request start_app "${output}"; then
    rm -f -- "${output}"
    echo "[serve-vendor-arm] CRITICAL: process-manager start_app transport failed" >&2
    return 1
  fi
  if ! serve_vendor_arm_parse_process_manager_success "${output}"; then
    rm -f -- "${output}"
    echo "[serve-vendor-arm] CRITICAL: process-manager start_app reported failure" >&2
    return 1
  fi
  rm -f -- "${output}"
  serve_vendor_arm_wait_motion_player_ownership up 200 ||
    return 1
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
  echo \
    "[serve-vendor-arm] motion_player restore PASS: process=1 listener=1 publisher=1"
}

serve_vendor_arm_verify_action() {
  local output=""
  output="$(serve_vendor_arm_temp_file)"
  if ! curl -fsS \
      --connect-timeout 2 \
      --max-time 5 \
      -H 'content-type:application/json' \
      -H 'timeout: 5000' \
      -X POST \
      "${SERVE_VENDOR_ARM_ACTION_RPC}" \
      --data '{}' \
      -o "${output}"; then
    rm -f -- "${output}"
    serve_script_die "GetAction RPC is unavailable on 127.0.0.1:56322"
  fi
  if ! python3 - "${output}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        payload = json.load(stream)
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit(f"invalid GetAction JSON: {exc}")
header = payload.get("header")
info = payload.get("info")
if not isinstance(header, dict) or str(header.get("code")) != "0":
    raise SystemExit("GetAction header.code is not 0")
if not isinstance(info, dict):
    raise SystemExit("GetAction response has no info object")
if info.get("current_action") != "MotionControlAction_MOTION":
    raise SystemExit(
        f"required action MOTION; got {info.get('current_action')!r}"
    )
if info.get("status") != "MotionControlActionStatus_RUNNING":
    raise SystemExit(
        f"required action status RUNNING; got {info.get('status')!r}"
    )
PY
  then
    rm -f -- "${output}"
    serve_script_die "robot must remain MotionControlAction_MOTION/RUNNING"
  fi
  rm -f -- "${output}"
  echo "[serve-vendor-arm] action PASS: MOTION/RUNNING"
}

serve_vendor_arm_require_vendor_stack() {
  local owner_state=""
  local model=""
  command -v systemctl >/dev/null ||
    serve_script_die "systemctl is unavailable"
  owner_state="$(systemctl is-active agibot_pm 2>/dev/null || true)"
  [[ "${owner_state}" == "active" ]] ||
    serve_script_die \
      "agibot_pm must remain exactly active; got ${owner_state:-unknown}"
  [[ -f /agibot/data/info/model ]] ||
    serve_script_die "robot model file is missing"
  model="$(tr -d '[:space:]' </agibot/data/info/model)"
  [[ "${model}" == "A3_P1D0" ]] ||
    serve_script_die \
      "this commissioning wrapper is bound to A3_P1D0; got ${model:-unknown}"
  echo "[serve-vendor-arm] vendor stack PASS: agibot_pm=active model=A3_P1D0"
}

serve_vendor_arm_topic_info() {
  local topic="$1"
  LC_ALL=C LANG=C ros2 topic info --verbose "${topic}"
}

serve_vendor_arm_publisher_count() {
  local topic="$1"
  local output=""
  local count=""
  output="$(serve_vendor_arm_topic_info "${topic}" 2>/dev/null)" ||
    return 1
  count="$(
    awk -F: '
      /^Publisher count:[[:space:]]*[0-9]+[[:space:]]*$/ {
        gsub(/[[:space:]]/, "", $2)
        print $2
        exit
      }
    ' <<<"${output}"
  )"
  [[ "${count}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${count}"
}

serve_vendor_arm_wait_publisher_count() {
  local topic="$1"
  local expected="$2"
  local attempts="${3:-50}"
  local attempt=""
  local actual=""
  for ((attempt = 1; attempt <= attempts; ++attempt)); do
    actual="$(serve_vendor_arm_publisher_count "${topic}" || true)"
    if [[ "${actual}" == "${expected}" ]]; then
      return 0
    fi
    sleep 0.1
  done
  echo \
    "${topic} publisher count must be ${expected}; got ${actual:-unknown}" \
    >&2
  return 1
}

serve_vendor_arm_require_vendor_topics() {
  local state_publishers=""
  local command_publishers=""
  local command_info=""

  state_publishers="$(
    serve_vendor_arm_publisher_count "${SERVE_VENDOR_ARM_STATE_TOPIC}"
  )" ||
    serve_script_die \
      "cannot inspect ${SERVE_VENDOR_ARM_STATE_TOPIC}"
  [[ "${state_publishers}" == "1" ]] ||
    serve_script_die \
      "${SERVE_VENDOR_ARM_STATE_TOPIC} requires exactly one publisher; got ${state_publishers}"

  command_info="$(
    serve_vendor_arm_topic_info "${SERVE_VENDOR_ARM_COMMAND_TOPIC}"
  )" ||
    serve_script_die \
      "cannot inspect ${SERVE_VENDOR_ARM_COMMAND_TOPIC}"
  command_publishers="$(
    awk -F: '
      /^Publisher count:[[:space:]]*[0-9]+[[:space:]]*$/ {
        gsub(/[[:space:]]/, "", $2)
        print $2
        exit
      }
    ' <<<"${command_info}"
  )"
  [[ "${command_publishers}" == "1" ]] ||
    serve_script_die \
      "${SERVE_VENDOR_ARM_COMMAND_TOPIC} requires exactly one vendor publisher before handoff; got ${command_publishers:-unknown}"
  grep -Fq 'Node name: aimrt_motion_control_node' <<<"${command_info}" ||
    serve_script_die \
      "aimrt_motion_control_node is not subscribed to the arm command topic"
  grep -Fq 'Reliability: BEST_EFFORT' <<<"${command_info}" ||
    serve_script_die \
      "motion-control arm subscriber BEST_EFFORT contract was not observed"
  grep -Fq 'Durability: VOLATILE' <<<"${command_info}" ||
    serve_script_die \
      "motion-control arm subscriber VOLATILE contract was not observed"
  echo \
    "[serve-vendor-arm] topic ownership PASS: state_publishers=1 command_publishers=1"
}

serve_vendor_arm_wait_for_state_sample() {
  timeout 15s ros2 topic echo \
    "${SERVE_VENDOR_ARM_STATE_TOPIC}" \
    sensor_msgs/msg/JointState \
    --once \
    --qos-reliability best_effort \
    --qos-durability volatile \
    >/dev/null
}

serve_vendor_arm_offline_validate() {
  "${SERVE_VENDOR_ARM_RUNNER}" \
    --motion-csv "${SERVE_SCRIPT_MOTION}" \
    --offline-validate
}

serve_vendor_arm_prepare_deploy() {
  local machine=""
  local active_runner_pids=""
  local elf_description=""
  local dependency_report=""
  if [[ "${A3_VENDOR_ARM_QUICK_DEPLOY:-0}" != "1" ]]; then
    serve_script_prepare_mdu
    return
  fi

  machine="$(uname -m)"
  [[ "${machine}" == "aarch64" || "${machine}" == "arm64" ]] ||
    serve_script_die \
      "quick vendor-arm deploy requires aarch64; got ${machine}"
  [[ -f /agibot/software/v0/entry/env/env.sh ]] ||
    serve_script_die "vendor ROS environment is missing"
  [[ -x "${SERVE_VENDOR_ARM_RUNNER}" ]] ||
    serve_script_die "vendor-arm runner is missing or not executable"
  [[ -f "${SERVE_SCRIPT_MOTION}" ]] ||
    serve_script_die "serve CSV is missing: ${SERVE_SCRIPT_MOTION}"
  command -v file >/dev/null ||
    serve_script_die "file is required for quick-deploy ELF verification"
  command -v ldd >/dev/null ||
    serve_script_die "ldd is required for quick-deploy dependency verification"
  command -v taskset >/dev/null ||
    serve_script_die "taskset is unavailable"

  active_runner_pids="$(serve_script_active_runner_pids)"
  [[ -z "${active_runner_pids}" ]] ||
    serve_script_die \
      "another serve runner is active; pids=${active_runner_pids//$'\n'/,}"

  set +u
  # shellcheck disable=SC1091
  source /agibot/software/v0/entry/env/env.sh
  set -u
  export LD_LIBRARY_PATH="${SERVE_SCRIPT_DEPLOY_DIR}:${LD_LIBRARY_PATH:-}"

  elf_description="$(LC_ALL=C LANG=C file "${SERVE_VENDOR_ARM_RUNNER}")"
  grep -Eq 'ELF 64-bit.*(ARM aarch64|aarch64)' \
    <<<"${elf_description}" ||
    serve_script_die \
      "quick-deploy runner is not an AArch64 ELF: ${elf_description}"
  dependency_report="$(
    LC_ALL=C LANG=C ldd "${SERVE_VENDOR_ARM_RUNNER}" 2>&1
  )" ||
    serve_script_die "ldd could not inspect the quick-deploy runner"
  ! grep -Fq 'not found' <<<"${dependency_report}" ||
    serve_script_die "quick-deploy runner has unresolved dependencies"

  cd "${SERVE_SCRIPT_DEPLOY_DIR}"
  echo \
    "[serve-vendor-arm] QUICK DEPLOY PASS: runner_sha=$(serve_script_sha256 "${SERVE_VENDOR_ARM_RUNNER}") csv_sha=$(serve_script_sha256 "${SERVE_SCRIPT_MOTION}")"
}

serve_vendor_arm_prepare_runtime() {
  local active_runner_pids=""
  local motion_sha=""

  [[ -f /agibot/software/v0/entry/env/env.sh ]] ||
    serve_script_die "vendor ROS environment is missing"
  [[ -x "${SERVE_VENDOR_ARM_RUNNER}" ]] ||
    serve_script_die "vendor-arm runner is missing or not executable"
  [[ -f "${SERVE_SCRIPT_MOTION}" ]] ||
    serve_script_die "serve CSV is missing: ${SERVE_SCRIPT_MOTION}"
  motion_sha="$(serve_script_sha256 "${SERVE_SCRIPT_MOTION}")"
  [[ "${motion_sha}" == "${SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256}" ]] ||
    serve_script_die \
      "serve CSV identity mismatch: expected=${SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256} actual=${motion_sha}"

  active_runner_pids="$(serve_script_active_runner_pids)"
  [[ -z "${active_runner_pids}" ]] ||
    serve_script_die \
      "another serve runner is active; pids=${active_runner_pids//$'\n'/,}"

  set +u
  # shellcheck disable=SC1091
  source /agibot/software/v0/entry/env/env.sh
  set -u
  export LD_LIBRARY_PATH="${SERVE_SCRIPT_DEPLOY_DIR}:${LD_LIBRARY_PATH:-}"
  cd "${SERVE_SCRIPT_DEPLOY_DIR}"
  echo \
    "[serve-vendor-arm] LEAN RUNTIME: motion_sha=${motion_sha}; package audit is delegated to preflight/deploy"
}

serve_vendor_arm_require_real_tty() {
  [[ -t 0 && -t 1 ]] ||
    serve_script_die \
      "real arm commands require an interactive TTY; connect with ssh -tt"
  [[ -r /dev/tty && -w /dev/tty ]] ||
    serve_script_die \
      "real arm commands require a readable and writable controlling /dev/tty"
}

serve_vendor_arm_launch_runner() {
  local mode="$1"
  [[ -z "${SERVE_VENDOR_ARM_HANDOFF_DIR}" ]] ||
    serve_script_die "internal handoff directory already exists"
  SERVE_VENDOR_ARM_HANDOFF_DIR="$(
    mktemp -d "${TMPDIR:-/tmp}/a3-serve-vendor-arm-handoff.XXXXXX"
  )"
  SERVE_VENDOR_ARM_READY_FILE="${SERVE_VENDOR_ARM_HANDOFF_DIR}/ready"
  local -a command=(
    chrt --fifo 20
    taskset -c 4-7
    "${SERVE_VENDOR_ARM_RUNNER}"
    --motion-csv "${SERVE_SCRIPT_MOTION}"
    --allow-publish
    --confirm-real-commands
    --handoff-ready-file "${SERVE_VENDOR_ARM_READY_FILE}"
    --mode "${mode}"
  )
  if [[ "${mode}" == "hold-only" ]]; then
    command+=(--hold-seconds 3)
  fi
  if [[ "${mode}" == "serve-only" ]]; then
    # Space is a functional serve input, so preserve the controlling terminal
    # for this mode. Hold/prepare do not read stdin.
    "${command[@]}" </dev/tty &
  else
    "${command[@]}" &
  fi
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}

serve_vendor_arm_wait_runner_ready() {
  local attempt=""
  local marker=""
  for ((attempt = 1; attempt <= 200; ++attempt)); do
    if ! kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null; then
      wait "${SERVE_VENDOR_ARM_RUNNER_PID}" || true
      SERVE_VENDOR_ARM_RUNNER_PID=""
      echo "vendor arm runner exited before prearm completed" >&2
      return 1
    fi
    if [[ -f "${SERVE_VENDOR_ARM_READY_FILE}" ]]; then
      marker="$(<"${SERVE_VENDOR_ARM_READY_FILE}")"
      if [[ "${marker}" == "READY" ]]; then
        echo \
          "[serve-vendor-arm] runner PREARMED: fresh state cached, publisher not created"
        return 0
      fi
    fi
    sleep 0.05
  done
  echo "vendor arm runner did not prearm within ten seconds" >&2
  return 1
}

serve_vendor_arm_release_runner() {
  [[ -n "${SERVE_VENDOR_ARM_RUNNER_PID}" ]] ||
    return 1
  kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null ||
    return 1
  kill -USR1 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null ||
    return 1
  echo "[serve-vendor-arm] sent SIGUSR1 handoff release to prearmed runner"
}

serve_vendor_arm_wait_runner_started() {
  local attempt=""
  local marker=""
  for ((attempt = 1; attempt <= 500; ++attempt)); do
    if ! kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null; then
      wait "${SERVE_VENDOR_ARM_RUNNER_PID}" || true
      SERVE_VENDOR_ARM_RUNNER_PID=""
      echo \
        "vendor arm runner exited before publisher ownership was established" \
        >&2
      return 1
    fi
    if [[ -f "${SERVE_VENDOR_ARM_READY_FILE}" ]]; then
      marker="$(<"${SERVE_VENDOR_ARM_READY_FILE}")"
    fi
    if [[ "${marker}" == "RUNNING" ]]; then
      echo \
        "[serve-vendor-arm] runner takeover PASS: publisher opened and arm state resumed"
      return 0
    fi
    sleep 0.01
  done
  echo \
    "vendor arm runner did not confirm publisher ownership and live feedback" \
    >&2
  return 1
}

serve_vendor_arm_cleanup_handoff_files() {
  if [[ -n "${SERVE_VENDOR_ARM_READY_FILE}" ]]; then
    case "${SERVE_VENDOR_ARM_READY_FILE}" in
      */a3-serve-vendor-arm-handoff.*/ready)
        rm -f -- "${SERVE_VENDOR_ARM_READY_FILE}"
        ;;
      *)
        echo \
          "[serve-vendor-arm] refusing unexpected ready-file cleanup path" \
          >&2
        ;;
    esac
    SERVE_VENDOR_ARM_READY_FILE=""
  fi
  if [[ -n "${SERVE_VENDOR_ARM_HANDOFF_DIR}" ]]; then
    case "${SERVE_VENDOR_ARM_HANDOFF_DIR}" in
      */a3-serve-vendor-arm-handoff.*)
        rmdir -- "${SERVE_VENDOR_ARM_HANDOFF_DIR}" 2>/dev/null || true
        ;;
      *)
        echo \
          "[serve-vendor-arm] refusing unexpected handoff-dir cleanup path" \
          >&2
        ;;
    esac
    SERVE_VENDOR_ARM_HANDOFF_DIR=""
  fi
}

serve_vendor_arm_stop_runner() {
  local attempt=""
  if [[ -n "${SERVE_VENDOR_ARM_RUNNER_PID}" ]]; then
    if kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null; then
      kill -TERM "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
      for ((attempt = 1; attempt <= 20; ++attempt)); do
        kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || break
        sleep 0.05
      done
      if kill -0 "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null; then
        echo \
          "[serve-vendor-arm] runner ignored SIGTERM; forcing it down before restore" \
          >&2
        kill -KILL "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
      fi
    fi
    wait "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    SERVE_VENDOR_ARM_RUNNER_PID=""
  fi
  serve_vendor_arm_cleanup_handoff_files
}

serve_vendor_arm_restore() {
  [[ "${SERVE_VENDOR_ARM_RESTORE_REQUIRED}" -eq 1 ]] || return 0

  if ! serve_vendor_arm_start_motion_player_app; then
    echo \
      "[serve-vendor-arm] CRITICAL: automatic motion_player restore failed" \
      >&2
    return 1
  fi
}

serve_vendor_arm_cleanup() {
  local original_status="$?"
  local final_status="${original_status}"
  [[ "${SERVE_VENDOR_ARM_CLEANUP_ACTIVE}" -eq 0 ]] || return
  SERVE_VENDOR_ARM_CLEANUP_ACTIVE=1
  trap - EXIT
  trap '' HUP INT TERM

  serve_vendor_arm_stop_runner
  if ! serve_vendor_arm_restore; then
    final_status=1
  fi
  exit "${final_status}"
}

serve_vendor_arm_signal() {
  local status="$1"
  exit "${status}"
}

serve_vendor_arm_main() {
  local mode="preflight-only"
  local mode_seen=0
  local confirm_real=0
  local argument=""
  local runner_status=0

  while [[ $# -gt 0 ]]; do
    argument="$1"
    case "${argument}" in
      -h|--help)
        serve_vendor_arm_usage
        return 0
        ;;
      --preflight-only)
        [[ "${mode_seen}" -eq 0 ]] ||
          serve_script_die "choose exactly one operating mode"
        mode="preflight-only"
        mode_seen=1
        ;;
      --hold-only)
        [[ "${mode_seen}" -eq 0 ]] ||
          serve_script_die "choose exactly one operating mode"
        mode="hold-only"
        mode_seen=1
        ;;
      --prepare-only)
        [[ "${mode_seen}" -eq 0 ]] ||
          serve_script_die "choose exactly one operating mode"
        mode="prepare-only"
        mode_seen=1
        ;;
      --serve-only)
        [[ "${mode_seen}" -eq 0 ]] ||
          serve_script_die "choose exactly one operating mode"
        mode="serve-only"
        mode_seen=1
        ;;
      --confirm-real-commands)
        confirm_real=1
        ;;
      *)
        echo "unknown argument: ${argument}" >&2
        serve_vendor_arm_usage >&2
        return 64
        ;;
    esac
    shift
  done

  if [[ "${mode}" == "preflight-only" && "${confirm_real}" -eq 1 ]]; then
    serve_script_die \
      "--confirm-real-commands is valid only with --hold-only, --prepare-only, or --serve-only"
  fi
  if [[ "${mode}" != "preflight-only" && "${confirm_real}" -ne 1 ]]; then
    serve_script_die \
      "${mode} requires the literal --confirm-real-commands argument"
  fi

  trap serve_vendor_arm_cleanup EXIT
  trap 'serve_vendor_arm_signal 129' HUP
  trap 'serve_vendor_arm_signal 130' INT
  trap 'serve_vendor_arm_signal 143' TERM

  if [[ "${mode}" == "preflight-only" ]]; then
    serve_vendor_arm_prepare_deploy
    serve_vendor_arm_offline_validate
    serve_vendor_arm_require_vendor_stack
    serve_vendor_arm_verify_action
    serve_vendor_arm_require_control_endpoints
    serve_vendor_arm_require_motion_player_process
    serve_vendor_arm_require_motion_player_idle
    serve_vendor_arm_require_vendor_topics
    serve_vendor_arm_wait_for_state_sample ||
      serve_script_die \
        "no fresh arm_joint_state sample arrived within fifteen seconds"
    echo \
      "[serve-vendor-arm] PREFLIGHT PASS; motion_player and publishers unchanged"
    return 0
  fi

  # Real execution stays lean: the runner repeats CSV validation before
  # creating ROS entities and performs the exact-name state/publisher prearm.
  # Expensive package, ldd, topic-QoS and ros2-echo audits remain available in
  # the default read-only preflight, not in the command path.
  serve_vendor_arm_prepare_runtime
  serve_vendor_arm_require_vendor_stack
  serve_vendor_arm_verify_action
  serve_vendor_arm_require_motion_player_idle
  if [[ "${mode}" == "serve-only" ]]; then
    serve_vendor_arm_require_real_tty
  fi
  echo \
    "[serve-vendor-arm] REAL ${mode}: agibot_pm, motion_control, and HAL stay active."
  echo \
    "[serve-vendor-arm] Keep the robot supported and keep E-stop reachable."
  if [[ "${mode}" == "hold-only" ]]; then
    echo \
      "[serve-vendor-arm] The measured 14-arm pose will be held for 3 seconds."
  elif [[ "${mode}" == "prepare-only" ]]; then
    echo \
      "[serve-vendor-arm] Both arms move to the CSV-derived serve-ready pose; left wrist-roll stays measured."
    echo \
      "[serve-vendor-arm] READY is held until Ctrl-C; Ctrl-C restores motion_player."
  else
    echo \
      "[serve-vendor-arm] Both arms move to the CSV-derived serve-ready pose; left wrist-roll stays measured."
    echo \
      "[serve-vendor-arm] After the runner prints READY HOLD, press Space at the physical ball-release instant (physical t=0)."
    echo \
      "[serve-vendor-arm] READY holds for 1.000 seconds; the original-timing CSV stroke reaches nominal strike at about physical t=+1.060 seconds."
  fi

  serve_vendor_arm_launch_runner "${mode}"
  serve_vendor_arm_wait_runner_ready ||
    serve_script_die "custom arm runner failed to prearm"

  # There must be no blocking prompt after this point.  The runner already has
  # a fresh exact-name state sample but has not created a command publisher.
  serve_vendor_arm_stop_motion_player_app ||
    serve_script_die "failed to stop motion_player safely"
  serve_vendor_arm_release_runner ||
    serve_script_die "failed to release the prearmed arm runner"
  serve_vendor_arm_wait_runner_started ||
    serve_script_die "custom arm publisher failed to take ownership"
  echo \
    "[serve-vendor-arm] custom arm takeover PASS: PREARM handoff and live feedback completed"

  set +e
  wait "${SERVE_VENDOR_ARM_RUNNER_PID}"
  runner_status=$?
  set -e
  SERVE_VENDOR_ARM_RUNNER_PID=""
  serve_vendor_arm_cleanup_handoff_files
  if [[ "${runner_status}" -ne 0 ]]; then
    echo \
      "[serve-vendor-arm] vendor arm runner exited with status ${runner_status}" \
      >&2
    return "${runner_status}"
  fi
  echo \
    "[serve-vendor-arm] vendor arm runner completed; restoring motion_player"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  serve_vendor_arm_main "$@"
fi
