#!/usr/bin/env bash
# Narrow, fail-closed TTY supervisor for the already validated model21800 runner.
# This script never changes the runner, its gains, or its state machine.

set -euo pipefail
umask 077
export PATH="/usr/sbin:/usr/bin:/sbin:/bin"
export LC_ALL=C

readonly DEPLOY_DIR="/agibot/a3_deploy_model21800"
readonly RUN_SCRIPT="${DEPLOY_DIR}/run_a3.sh"
readonly RUN_BINARY="${DEPLOY_DIR}/a3_deploy_onnx_ref"
readonly RUNTIME_CFG="${DEPLOY_DIR}/config/a3_runtime_config.yaml"
readonly AIMRT_CFG="${DEPLOY_DIR}/config/a3_aimrt_config.iceoryx.yaml"
readonly RUN_SCRIPT_SHA="6654f815aa5815593c15ea93d10797a2a6dbe38188c3738d96c0c5f360b98288"
readonly RUN_BINARY_SHA="e4d8d7701017a96ff856717940efc39baff336889df260ef774180e78803e44c"
readonly RUNTIME_CFG_SHA="1377ea901126e6d4b0c8e8fafbce08c0b9cd62a6a5d96ba603ca54d1a8497515"
readonly AIMRT_CFG_SHA="c0a1af244d803ff17a76bc0b5016be9e28bf216179c5462efd6b94af2ebb1087"

readonly TMUX_BIN="/usr/bin/tmux"
readonly TMUX_SOCKET="hope-a3-control"
readonly TMUX_SESSION="model21800"
readonly TMUX_PANE="${TMUX_SESSION}:0.0"
readonly CONTROL_DIR="${DEPLOY_DIR}/.hope-control"
readonly LOG_DIR="${CONTROL_DIR}/logs"
readonly STATE_FILE="${CONTROL_DIR}/state"
readonly LOCK_FILE="${CONTROL_DIR}/lock"
readonly ESTOP_LATCH="${CONTROL_DIR}/estop-latched"
readonly MAX_EXACT_SEQUENCE=4503599627370496

RUN_ID="none"
PID=0
START_TICKS=0
BOOT_ID="none"
LOG_FILE="none"
STATE="STOPPED"
STARTED_NS=0
REQUEST_SEQ=0
REQUEST_CODE="none"
APPLIED_SEQ=0
RESULT=0
FAULT=0
MODE_OFFSET=0
PREPARE_OFFSET=0
LAST_PD_TICKS=0
REASON="NONE"

tmux_ctl() {
  "${TMUX_BIN}" -L "${TMUX_SOCKET}" "$@"
}

emit_status() {
  printf 'A3CTL_V1 state=%s run_id=%s pid=%s start_ticks=%s pd_ticks=%s request_seq=%s applied_seq=%s result=%s fault=%s reason=%s\n' \
    "${STATE}" "${RUN_ID}" "${PID}" "${START_TICKS}" \
    "${LAST_PD_TICKS}" "${REQUEST_SEQ}" "${APPLIED_SEQ}" \
    "${RESULT}" "${FAULT}" "${REASON}"
}

valid_uint() {
  [[ "${1:-}" =~ ^(0|[1-9][0-9]*)$ ]]
}

valid_sequence() {
  valid_uint "${1:-}" && (( ${#1} <= 16 )) &&
    (( 10#${1} <= MAX_EXACT_SEQUENCE ))
}

ensure_private_dirs() {
  if [[ "$(id -un)" != "agi" ]]; then
    STATE="UNKNOWN"
    FAULT=1
    RESULT=-3
    REASON="WRONG_USER"
    emit_status
    exit 1
  fi
  mkdir -p -- "${CONTROL_DIR}" "${LOG_DIR}"
  chmod 0700 -- "${CONTROL_DIR}" "${LOG_DIR}"
}

load_state() {
  RUN_ID="none"
  PID=0
  START_TICKS=0
  BOOT_ID="none"
  LOG_FILE="none"
  STATE="STOPPED"
  STARTED_NS=0
  REQUEST_SEQ=0
  REQUEST_CODE="none"
  APPLIED_SEQ=0
  RESULT=0
  FAULT=0
  MODE_OFFSET=0
  PREPARE_OFFSET=0
  LAST_PD_TICKS=0
  REASON="NONE"
  [[ -f "${STATE_FILE}" ]] || return 0

  local key value
  while IFS='=' read -r key value; do
    case "${key}" in
      run_id) [[ "${value}" =~ ^(none|[0-9a-f]{32})$ ]] && RUN_ID="${value}" ;;
      pid) valid_uint "${value}" && PID="${value}" ;;
      start_ticks) valid_uint "${value}" && START_TICKS="${value}" ;;
      boot_id) [[ "${value}" =~ ^(none|[0-9a-f-]{36})$ ]] && BOOT_ID="${value}" ;;
      log_file)
        if [[ "${value}" == none ||
              ( "${RUN_ID}" != none && "${value}" == "${LOG_DIR}/runner-${RUN_ID}.log" ) ]]; then
          LOG_FILE="${value}"
        fi
        ;;
      state)
        case "${value}" in
          STOPPED|STARTING|IDLE|PASSIVE|PD_RAMP|PD_READY|MOTION|FAILED|UNKNOWN)
            STATE="${value}"
            ;;
        esac
        ;;
      started_ns) valid_uint "${value}" && STARTED_NS="${value}" ;;
      request_seq) valid_sequence "${value}" && REQUEST_SEQ="${value}" ;;
      request_code)
        case "${value}" in none|passive|prepare|policy) REQUEST_CODE="${value}" ;; esac
        ;;
      applied_seq) valid_sequence "${value}" && APPLIED_SEQ="${value}" ;;
      result) [[ "${value}" =~ ^-?[0-3]$ ]] && RESULT="${value}" ;;
      fault) [[ "${value}" == 0 || "${value}" == 1 ]] && FAULT="${value}" ;;
      mode_offset) valid_uint "${value}" && MODE_OFFSET="${value}" ;;
      prepare_offset) valid_uint "${value}" && PREPARE_OFFSET="${value}" ;;
      last_pd_ticks) valid_uint "${value}" && LAST_PD_TICKS="${value}" ;;
      reason) [[ "${value}" =~ ^[A-Z0-9_]+$ ]] && REASON="${value}" ;;
    esac
  done < "${STATE_FILE}"
}

write_state() {
  local tmp
  tmp="$(mktemp "${CONTROL_DIR}/state.tmp.XXXXXX")"
  {
    printf 'run_id=%s\n' "${RUN_ID}"
    printf 'pid=%s\n' "${PID}"
    printf 'start_ticks=%s\n' "${START_TICKS}"
    printf 'boot_id=%s\n' "${BOOT_ID}"
    printf 'log_file=%s\n' "${LOG_FILE}"
    printf 'state=%s\n' "${STATE}"
    printf 'started_ns=%s\n' "${STARTED_NS}"
    printf 'request_seq=%s\n' "${REQUEST_SEQ}"
    printf 'request_code=%s\n' "${REQUEST_CODE}"
    printf 'applied_seq=%s\n' "${APPLIED_SEQ}"
    printf 'result=%s\n' "${RESULT}"
    printf 'fault=%s\n' "${FAULT}"
    printf 'mode_offset=%s\n' "${MODE_OFFSET}"
    printf 'prepare_offset=%s\n' "${PREPARE_OFFSET}"
    printf 'last_pd_ticks=%s\n' "${LAST_PD_TICKS}"
    printf 'reason=%s\n' "${REASON}"
  } > "${tmp}"
  chmod 0600 -- "${tmp}"
  mv -f -- "${tmp}" "${STATE_FILE}"
}

has_session() {
  tmux_ctl has-session -t "${TMUX_SESSION}" 2>/dev/null
}

process_start_ticks() {
  awk '{print $22}' "/proc/$1/stat" 2>/dev/null
}

process_is_live() {
  [[ -r "/proc/$1/stat" ]] && [[ "$(awk '{print $3}' "/proc/$1/stat" 2>/dev/null)" != Z ]]
}

validate_identity() {
  [[ "${PID}" != 0 && "${RUN_ID}" != none ]] || return 1
  [[ "${BOOT_ID}" == "$(tr -d '\n' < /proc/sys/kernel/random/boot_id)" ]] || return 1
  has_session || return 1
  local pane_pid pane_dead actual_exe actual_cwd actual_ticks
  pane_pid="$(tmux_ctl display-message -p -t "${TMUX_PANE}" '#{pane_pid}' 2>/dev/null)" || return 1
  pane_dead="$(tmux_ctl display-message -p -t "${TMUX_PANE}" '#{pane_dead}' 2>/dev/null)" || return 1
  [[ "${pane_dead}" == 0 && "${pane_pid}" == "${PID}" ]] || return 1
  [[ -r "/proc/${PID}/cmdline" ]] || return 1
  actual_exe="$(readlink -f "/proc/${PID}/exe" 2>/dev/null)" || return 1
  actual_cwd="$(readlink -f "/proc/${PID}/cwd" 2>/dev/null)" || return 1
  actual_ticks="$(process_start_ticks "${PID}")" || return 1
  [[ "${actual_exe}" == "${RUN_BINARY}" ]] || return 1
  [[ "${actual_cwd}" == "${DEPLOY_DIR}" ]] || return 1
  [[ "${actual_ticks}" == "${START_TICKS}" ]] || return 1

  local -a argv=()
  mapfile -d '' -t argv < "/proc/${PID}/cmdline"
  [[ "${#argv[@]}" == 4 ]] || return 1
  [[ "${argv[0]}" == "${RUN_BINARY}" ]] || return 1
  [[ "${argv[1]}" == "--runtime-cfg=${RUNTIME_CFG}" ]] || return 1
  [[ "${argv[2]}" == "--aimrt-cfg=${AIMRT_CFG}" ]] || return 1
  [[ "${argv[3]}" == "--frame-log-interval=25" ]] || return 1
}

verify_artifact() {
  local path="$1" expected="$2"
  [[ -f "${path}" && ! -L "${path}" ]] || return 1
  [[ "$(readlink -f "${path}")" == "${path}" ]] || return 1
  [[ "$(stat -c %U "${path}")" == "agi" ]] || return 1
  [[ "$(sha256sum "${path}" | awk '{print $1}')" == "${expected}" ]]
}

verify_artifacts() {
  verify_artifact "${RUN_SCRIPT}" "${RUN_SCRIPT_SHA}" &&
    verify_artifact "${RUN_BINARY}" "${RUN_BINARY_SHA}" &&
    verify_artifact "${RUNTIME_CFG}" "${RUNTIME_CFG_SHA}" &&
    verify_artifact "${AIMRT_CFG}" "${AIMRT_CFG_SHA}"
}

vendor_motion_control_running() {
  local comm
  for comm in /proc/[0-9]*/comm; do
    [[ -r "${comm}" ]] || continue
    [[ "$(tr -d '\n' < "${comm}")" == "motion_control" ]] && return 0
  done
  return 1
}

another_runner_running() {
  local exe actual
  for exe in /proc/[0-9]*/exe; do
    actual="$(readlink -f "${exe}" 2>/dev/null)" || continue
    case "${actual}" in
      */a3_deploy_onnx_ref|*/a3_deploy_onnx_ref_pingpong) return 0 ;;
    esac
  done
  return 1
}

log_has_fatal_since() {
  local offset="$1"
  tail -c "+$((offset + 1))" "${LOG_FILE}" 2>/dev/null |
    grep -Eq 'not all 6 topics ready|Backend::Start failed|A3PolicyDriver::StartDriver failed|SAFETY LATCH|safety latch|FATAL|Segmentation fault'
}

refresh_state() {
  load_state
  if [[ -f "${ESTOP_LATCH}" ]]; then
    FAULT=1
    REASON="ESTOP_LATCHED"
  fi

  if ! has_session; then
    if another_runner_running; then
      STATE="UNKNOWN"
      FAULT=1
      RESULT=-3
      REASON="UNMANAGED_RUNNER_PRESENT"
      write_state
      return 0
    fi
    if [[ "${STATE}" != STOPPED ]]; then
      STATE="FAILED"
      FAULT=1
      RESULT=-2
      REASON="RUNNER_EXITED"
    fi
    write_state
    return 0
  fi

  if ! validate_identity; then
    STATE="UNKNOWN"
    FAULT=1
    RESULT=-3
    REASON="IDENTITY_MISMATCH"
    write_state
    return 0
  fi
  if [[ "${LOG_FILE}" == none || ! -f "${LOG_FILE}" ]]; then
    STATE="UNKNOWN"
    FAULT=1
    RESULT=-3
    REASON="LOG_UNAVAILABLE"
    write_state
    return 0
  fi
  if log_has_fatal_since 0; then
    STATE="FAILED"
    FAULT=1
    [[ "${REQUEST_SEQ}" == "${APPLIED_SEQ}" ]] || RESULT=-2
    REASON="RUNNER_LOG_FAULT"
    write_state
    return 0
  fi

  if [[ "${STATE}" == STARTING ]]; then
    local ready_count now_ns
    ready_count="$(grep -Ec ' ready=yes +samples=' "${LOG_FILE}" || true)"
    if (( ready_count >= 6 )) &&
      grep -Fq '✓ backend started' "${LOG_FILE}" &&
      grep -Fq '✓ manual state machine: startup IDLE/no-output' "${LOG_FILE}" &&
      grep -Fq '✓ entering policy loop' "${LOG_FILE}"; then
      if grep -Eq '\[mode\] .* -> ' "${LOG_FILE}"; then
        STATE="UNKNOWN"
        FAULT=1
        RESULT=-3
        REASON="UNMANAGED_MODE_CHANGE"
      else
        STATE="IDLE"
        FAULT=0
        REASON="NONE"
        MODE_OFFSET="$(stat -c %s "${LOG_FILE}")"
      fi
    else
      now_ns="$(date +%s%N)"
      if (( STARTED_NS > 0 && now_ns - STARTED_NS > 15000000000 )); then
        STATE="FAILED"
        FAULT=1
        RESULT=-2
        REASON="STARTUP_TIMEOUT"
      fi
    fi
  elif [[ "${STATE}" == PD_RAMP || "${STATE}" == PD_READY ]]; then
    local latest_pd
    latest_pd="$(tail -c "+$((PREPARE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
      sed -nE 's/.*\[frames\].* mode=pd_stand pd=([0-9]+).*/\1/p' | tail -n 1)"
    if [[ -n "${latest_pd}" ]]; then
      if (( latest_pd < LAST_PD_TICKS )); then
        STATE="UNKNOWN"
        FAULT=1
        RESULT=-2
        REASON="PD_COUNTER_REGRESSED"
      else
        LAST_PD_TICKS="${latest_pd}"
        if (( latest_pd > 150 )); then
          STATE="PD_READY"
          REASON="NONE"
        fi
      fi
    fi
    if tail -c "+$((PREPARE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
      grep -Eq '\[frames\].* halts=[1-9][0-9]*'; then
      STATE="FAILED"
      FAULT=1
      RESULT=-2
      REASON="SAFE_HALT_OBSERVED"
    fi
  fi

  if [[ "${STATE}" == IDLE ]]; then
    if tail -c "+$((MODE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
      grep -Eq '\[mode\] .* -> |\[frames\].* mode='; then
      STATE="UNKNOWN"
      FAULT=1
      RESULT=-3
      REASON="UNMANAGED_MODE_CHANGE"
    fi
  elif [[ "${STATE}" == PASSIVE || "${STATE}" == PD_RAMP ||
          "${STATE}" == PD_READY || "${STATE}" == MOTION ]]; then
    local expected_mode latest_mode
    case "${STATE}" in
      PASSIVE) expected_mode="passive" ;;
      PD_RAMP|PD_READY) expected_mode="pd_stand" ;;
      MOTION) expected_mode="motion" ;;
    esac
    if tail -c "+$((MODE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
      grep -Eq '\[mode\] .* -> '; then
      STATE="UNKNOWN"
      FAULT=1
      RESULT=-3
      REASON="UNMANAGED_MODE_CHANGE"
    else
      latest_mode="$(tail -c "+$((MODE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
        sed -nE 's/.*\[frames\].* mode=([a-z_]+).*/\1/p' | tail -n 1)"
      if [[ -n "${latest_mode}" && "${latest_mode}" != "${expected_mode}" ]]; then
        STATE="UNKNOWN"
        FAULT=1
        RESULT=-3
        REASON="UNMANAGED_MODE_CHANGE"
      fi
    fi
    if [[ "${STATE}" != UNKNOWN ]] &&
      tail -c "+$((MODE_OFFSET + 1))" "${LOG_FILE}" 2>/dev/null |
      grep -Eq '\[frames\].* halts=[1-9][0-9]*'; then
      STATE="FAILED"
      FAULT=1
      RESULT=-2
      REASON="SAFE_HALT_OBSERVED"
    fi
  fi

  if [[ "${STATE}" == PASSIVE || "${STATE}" == PD_RAMP ||
        "${STATE}" == PD_READY || "${STATE}" == MOTION ]]; then
    local now_s mtime_s
    now_s="$(date +%s)"
    mtime_s="$(stat -c %Y "${LOG_FILE}")"
    if (( now_s - mtime_s > 3 )); then
      STATE="UNKNOWN"
      FAULT=1
      RESULT=-2
      REASON="ACTIVE_LOG_STALE"
    fi
  fi
  write_state
}

start_runner() {
  local sequence="$1"
  load_state
  if [[ "${REQUEST_SEQ}" != "${sequence}" || "${REQUEST_CODE}" != prepare ]]; then
    REASON="SUPERSEDED"
    emit_status
    return 0
  fi
  if [[ -f "${ESTOP_LATCH}" ]]; then
    RESULT=-3
    FAULT=1
    REASON="ESTOP_LATCHED"
    write_state
    emit_status
    return 0
  fi
  if has_session; then
    refresh_state
    emit_status
    return 0
  fi
  if ! verify_artifacts; then
    STATE="FAILED"
    RESULT=-3
    FAULT=1
    REASON="ARTIFACT_MISMATCH"
    write_state
    emit_status
    return 0
  fi
  if vendor_motion_control_running; then
    STATE="STOPPED"
    RESULT=-1
    FAULT=0
    REASON="VENDOR_CONTROL_CONFLICT"
    write_state
    emit_status
    return 0
  fi
  if another_runner_running; then
    STATE="STOPPED"
    RESULT=-1
    FAULT=0
    REASON="RUNNER_CONFLICT"
    write_state
    emit_status
    return 0
  fi
  if [[ -f "${ESTOP_LATCH}" ]]; then
    STATE="STOPPED"
    RESULT=-3
    FAULT=1
    REASON="ESTOP_LATCHED"
    write_state
    emit_status
    return 0
  fi

  RUN_ID="$(tr -d '-' < /proc/sys/kernel/random/uuid)"
  LOG_FILE="${LOG_DIR}/runner-${RUN_ID}.log"
  local gate_file="${CONTROL_DIR}/start-${RUN_ID}"
  : > "${LOG_FILE}"
  chmod 0600 -- "${LOG_FILE}"
  rm -f -- "${gate_file}"

  local launch_command
  launch_command="exec /bin/bash -lc 'while [[ ! -f ${gate_file} ]]; do /bin/sleep 0.02; done; cd ${DEPLOY_DIR}; exec /usr/bin/env -u A3_ROBOT_ENV -u A3_SOURCE_ROBOT_ENV -u A3_AIMRT_CFG -u A3_RUNTIME_CFG A3_TRANSPORT=iceoryx ./run_a3.sh --frame-log-interval=25'"
  tmux_ctl new-session -d -s "${TMUX_SESSION}" -n runner -x 240 -y 80 "${launch_command}"
  tmux_ctl set-option -t "${TMUX_SESSION}" remain-on-exit on
  tmux_ctl set-option -t "${TMUX_SESSION}" history-limit 100000
  tmux_ctl pipe-pane -o -t "${TMUX_PANE}" "exec /bin/sh -c 'cat >> ${LOG_FILE}'"
  : > "${gate_file}"

  PID="$(tmux_ctl display-message -p -t "${TMUX_PANE}" '#{pane_pid}')"
  local tries=0 actual_exe=""
  while (( tries < 40 )); do
    actual_exe="$(readlink -f "/proc/${PID}/exe" 2>/dev/null || true)"
    [[ "${actual_exe}" == "${RUN_BINARY}" ]] && break
    /bin/sleep 0.05
    ((tries += 1))
  done
  rm -f -- "${gate_file}"
  if [[ "${actual_exe}" != "${RUN_BINARY}" ]]; then
    tmux_ctl kill-session -t "${TMUX_SESSION}" 2>/dev/null || true
    STATE="FAILED"
    PID=0
    START_TICKS=0
    RESULT=-2
    FAULT=1
    REASON="EXEC_START_FAILED"
    write_state
    emit_status
    return 0
  fi

  START_TICKS="$(process_start_ticks "${PID}")"
  BOOT_ID="$(tr -d '\n' < /proc/sys/kernel/random/boot_id)"
  STATE="STARTING"
  STARTED_NS="$(date +%s%N)"
  RESULT=0
  FAULT=0
  MODE_OFFSET=0
  PREPARE_OFFSET=0
  LAST_PD_TICKS=0
  REASON="STARTING"
  write_state
  if ! validate_identity; then
    STATE="UNKNOWN"
    RESULT=-3
    FAULT=1
    REASON="IDENTITY_MISMATCH"
    write_state
  fi
  emit_status
}

claim_command() {
  local sequence="$1" code="$2"
  refresh_state
  if ! valid_sequence "${sequence}" || (( sequence == 0 )); then
    RESULT=-1
    REASON="INVALID_SEQUENCE"
    emit_status
    return 0
  fi
  case "${code}" in passive|prepare|policy) ;; *) RESULT=-1; REASON="INVALID_COMMAND"; emit_status; return 0 ;; esac
  if (( sequence <= REQUEST_SEQ )); then
    REASON="REPLAY_OR_SUPERSEDED"
    emit_status
    return 0
  fi
  REQUEST_SEQ="${sequence}"
  REQUEST_CODE="${code}"
  RESULT=0
  REASON="PENDING"
  if [[ -f "${ESTOP_LATCH}" ]]; then
    RESULT=-3
    FAULT=1
    REASON="ESTOP_LATCHED"
  elif [[ "${code}" == passive && "${STATE}" == STOPPED ]] && ! has_session; then
    APPLIED_SEQ="${sequence}"
    RESULT=1
    REASON="ALREADY_STOPPED"
  fi
  write_state
  emit_status
}

send_key() {
  local action="$1" sequence="$2" key expected_transition expected_frame=""
  refresh_state
  if [[ "${REQUEST_SEQ}" != "${sequence}" || "${REQUEST_CODE}" != "${action}" ]]; then
    REASON="SUPERSEDED"
    emit_status
    return 0
  fi
  if [[ -f "${ESTOP_LATCH}" || "${FAULT}" == 1 ]]; then
    RESULT=-3
    REASON="CONTROL_BLOCKED"
    write_state
    emit_status
    return 0
  fi
  case "${action}" in
    passive)
      key="p"
      expected_transition='\[mode\] (idle|passive|pd_stand|motion|teleop) -> passive'
      expected_frame='\[frames\].* mode=passive'
      ;;
    prepare)
      key="s"
      expected_transition='\[mode\] (idle|passive|pd_stand|motion|teleop) -> pd_stand'
      ;;
    policy)
      if [[ "${STATE}" != PD_READY ]]; then
        RESULT=-1
        REASON="PD_NOT_READY"
        write_state
        emit_status
        return 0
      fi
      key="m"
      expected_transition='\[mode\] pd_stand -> motion'
      expected_frame='\[frames\].* mode=motion'
      ;;
    *)
      RESULT=-1
      REASON="INVALID_COMMAND"
      write_state
      emit_status
      return 0
      ;;
  esac
  validate_identity || {
    STATE="UNKNOWN"
    RESULT=-3
    FAULT=1
    REASON="IDENTITY_MISMATCH"
    write_state
    emit_status
    return 0
  }

  local offset appended tries=0 transition_seen=0 frame_seen=0
  offset="$(stat -c %s "${LOG_FILE}")"
  tmux_ctl send-keys -t "${TMUX_PANE}" -l "${key}"
  while (( tries < 40 )); do
    if [[ -f "${ESTOP_LATCH}" ]]; then
      STATE="UNKNOWN"
      RESULT=-3
      FAULT=1
      REASON="ESTOP_LATCHED"
      write_state
      emit_status
      return 0
    fi
    appended="$(tail -c "+$((offset + 1))" "${LOG_FILE}" 2>/dev/null || true)"
    if grep -Eq 'not all 6 topics ready|Backend::Start failed|A3PolicyDriver::StartDriver failed|SAFETY LATCH|safety latch|FATAL|Segmentation fault|\[frames\].* halts=[1-9][0-9]*' <<< "${appended}"; then
      STATE="FAILED"
      RESULT=-2
      FAULT=1
      REASON="RUNNER_LOG_FAULT"
      write_state
      emit_status
      return 0
    fi
    grep -Eq "${expected_transition}" <<< "${appended}" && transition_seen=1
    if [[ -z "${expected_frame}" ]] || grep -Eq "${expected_frame}" <<< "${appended}"; then
      frame_seen=1
    fi
    if (( transition_seen == 1 && frame_seen == 1 )); then
      break
    fi
    if grep -Eq '\[mode\] ignored' <<< "${appended}"; then
      RESULT=-1
      REASON="RUNNER_REJECTED"
      write_state
      emit_status
      return 0
    fi
    /bin/sleep 0.05
    ((tries += 1))
  done
  if (( transition_seen != 1 || frame_seen != 1 )); then
    STATE="UNKNOWN"
    RESULT=-2
    FAULT=1
    REASON="ACK_TIMEOUT"
    write_state
    emit_status
    return 0
  fi

  MODE_OFFSET="$(stat -c %s "${LOG_FILE}")"
  APPLIED_SEQ="${sequence}"
  RESULT=1
  FAULT=0
  REASON="NONE"
  case "${action}" in
    passive) STATE="PASSIVE" ;;
    prepare)
      STATE="PD_RAMP"
      PREPARE_OFFSET="${offset}"
      LAST_PD_TICKS=0
      ;;
    policy) STATE="MOTION" ;;
  esac
  write_state
  emit_status
}

stop_for_estop() {
  load_state
  : > "${ESTOP_LATCH}"
  chmod 0600 -- "${ESTOP_LATCH}"
  FAULT=1
  RESULT=-3
  REASON="ESTOP_LATCHED"
  write_state

  if ! has_session; then
    if another_runner_running; then
      STATE="UNKNOWN"
      REASON="UNMANAGED_RUNNER_PRESENT"
      write_state
      emit_status
      return 1
    fi
    STATE="STOPPED"
    PID=0
    START_TICKS=0
    REASON="ALREADY_STOPPED"
    write_state
    emit_status
    return 0
  fi
  if ! validate_identity; then
    STATE="UNKNOWN"
    REASON="IDENTITY_MISMATCH"
    write_state
    emit_status
    return 1
  fi

  local exact_pid="${PID}" exact_ticks="${START_TICKS}" tries=0
  tmux_ctl send-keys -t "${TMUX_PANE}" -l q 2>/dev/null || true
  while (( tries < 10 )) && process_is_live "${exact_pid}"; do
    /bin/sleep 0.05
    ((tries += 1))
  done
  if process_is_live "${exact_pid}"; then
    if [[ "$(process_start_ticks "${exact_pid}")" != "${exact_ticks}" ]] ||
       [[ "$(readlink -f "/proc/${exact_pid}/exe" 2>/dev/null || true)" != "${RUN_BINARY}" ]]; then
      STATE="UNKNOWN"
      REASON="IDENTITY_CHANGED_DURING_STOP"
      write_state
      emit_status
      return 1
    fi
    kill -TERM "${exact_pid}" 2>/dev/null || true
    tries=0
    while (( tries < 15 )) && process_is_live "${exact_pid}"; do
      /bin/sleep 0.05
      ((tries += 1))
    done
  fi
  if process_is_live "${exact_pid}"; then
    if [[ "$(process_start_ticks "${exact_pid}")" != "${exact_ticks}" ]] ||
       [[ "$(readlink -f "/proc/${exact_pid}/exe" 2>/dev/null || true)" != "${RUN_BINARY}" ]]; then
      STATE="UNKNOWN"
      REASON="IDENTITY_CHANGED_DURING_STOP"
      write_state
      emit_status
      return 1
    fi
    kill -KILL "${exact_pid}" 2>/dev/null || true
    tries=0
    while (( tries < 10 )) && process_is_live "${exact_pid}"; do
      /bin/sleep 0.05
      ((tries += 1))
    done
    if process_is_live "${exact_pid}"; then
      STATE="UNKNOWN"
      REASON="STOP_TIMEOUT"
      write_state
      emit_status
      return 1
    fi
  fi

  tmux_ctl kill-session -t "${TMUX_SESSION}" 2>/dev/null || true
  STATE="STOPPED"
  PID=0
  START_TICKS=0
  LAST_PD_TICKS=0
  REASON="ESTOP_RUNNER_STOPPED"
  write_state
  emit_status
}

main() {
  ensure_private_dirs
  # Make the safety latch visible before waiting for an in-flight ordinary
  # operation's lock. That operation will fail closed and release the lock.
  if [[ "${1:-}" == stop && "$#" == 1 ]]; then
    : > "${ESTOP_LATCH}"
    chmod 0600 -- "${ESTOP_LATCH}"
  fi
  exec 9> "${LOCK_FILE}"
  if ! flock -w 2 9; then
    load_state
    RESULT=-2
    FAULT=1
    REASON="CONTROL_BUSY"
    emit_status
    exit 1
  fi

  case "${1:-}" in
    status)
      [[ "$#" == 1 ]] || { load_state; RESULT=-1; REASON="BAD_ARGUMENTS"; emit_status; exit 1; }
      refresh_state
      emit_status
      ;;
    claim)
      [[ "$#" == 3 ]] || { load_state; RESULT=-1; REASON="BAD_ARGUMENTS"; emit_status; exit 1; }
      claim_command "$2" "$3"
      ;;
    start)
      [[ "$#" == 2 ]] || { load_state; RESULT=-1; REASON="BAD_ARGUMENTS"; emit_status; exit 1; }
      valid_sequence "$2" || { load_state; RESULT=-1; REASON="INVALID_SEQUENCE"; emit_status; exit 1; }
      start_runner "$2"
      ;;
    key)
      [[ "$#" == 3 ]] || { load_state; RESULT=-1; REASON="BAD_ARGUMENTS"; emit_status; exit 1; }
      valid_sequence "$3" || { load_state; RESULT=-1; REASON="INVALID_SEQUENCE"; emit_status; exit 1; }
      case "$2" in passive|prepare|policy) ;; *) load_state; RESULT=-1; REASON="INVALID_COMMAND"; emit_status; exit 1 ;; esac
      send_key "$2" "$3"
      ;;
    stop)
      [[ "$#" == 1 ]] || { load_state; RESULT=-1; REASON="BAD_ARGUMENTS"; emit_status; exit 1; }
      stop_for_estop
      ;;
    *)
      load_state
      RESULT=-1
      REASON="BAD_ARGUMENTS"
      emit_status
      exit 1
      ;;
  esac
}

main "$@"
