#!/bin/bash

# Reviewed replacement for the vendor A3 time-distribution launcher.
# The system clock is owned by chrony. This script preserves the active
# ptp4l E2E and CLOCK_REALTIME-to-eth_hdu phc2sys behavior.

set -u -o pipefail
export LC_ALL=C

cd -- "$(dirname -- "$0")" || exit 1

LOG_DIR="/agibot/log/tsync"
MAX_LOG_FILES=20
device_name="eth_hdu"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
NTP_LOG="$LOG_DIR/ntp_${TIMESTAMP}.log"
PTP4L_LOG="$LOG_DIR/ptp4l_${TIMESTAMP}.log"
PHC2SYS_LOG="$LOG_DIR/phc2sys_${TIMESTAMP}.log"
ACTIVE_SOURCES="/etc/chrony/sources.d/hope-approved.sources"
REGION_ROOT="/etc/agibot-time/regions"
POLICY_FILE="/etc/default/agibot-timesync"
AGIBOT_ALLOW_UNQUALIFIED_TIME=0

mkdir -p "$LOG_DIR"

log() {
    local log_file=$1
    shift
    printf '%s\n' "$*" | tee -a "$log_file"
}

rotate_logs() {
    local prefix=$1
    local max_count=$2
    ls -t "${LOG_DIR}/${prefix}"_*.log 2>/dev/null |
        sed -e "1,${max_count}d" |
        xargs -r rm
}

validate_root_file() {
    local file=$1
    local perm
    [[ -f "$file" ]] || return 1
    [[ $(stat -Lc %u "$file") -eq 0 ]] || return 1
    perm=$(stat -Lc %a "$file")
    (( (8#$perm & 8#022) == 0 ))
}

validate_region_profile() {
    local target
    local source_count
    local source_files

    [[ -L "$ACTIVE_SOURCES" ]] || {
        log "$NTP_LOG" "ERROR: active regional source link is missing: $ACTIVE_SOURCES"
        return 1
    }
    target=$(readlink -f "$ACTIVE_SOURCES") || return 1
    case "$target" in
        "$REGION_ROOT"/china/chrony.sources|\
        "$REGION_ROOT"/us/chrony.sources|\
        "$REGION_ROOT"/europe/chrony.sources) ;;
        *)
            log "$NTP_LOG" "ERROR: unapproved regional source target: $target"
            return 1
            ;;
    esac
    validate_root_file "$target" || {
        log "$NTP_LOG" "ERROR: regional source file ownership or mode is unsafe: $target"
        return 1
    }
    grep -Eq 'site\.example|REPLACE_ME' "$target" && {
        log "$NTP_LOG" "ERROR: placeholder NTP source remains in $target"
        return 1
    }
    source_count=$(grep -Ec '^[[:space:]]*server[[:space:]]+' "$target" || true)
    [[ $source_count -eq 3 ]] || {
        log "$NTP_LOG" "ERROR: regional profile must contain exactly three servers: $target"
        return 1
    }
    mapfile -t source_files < <(compgen -G '/etc/chrony/sources.d/*.sources' || true)
    [[ ${#source_files[@]} -eq 1 && "${source_files[0]}" == "$ACTIVE_SOURCES" ]] || {
        log "$NTP_LOG" "ERROR: /etc/chrony/sources.d must contain only hope-approved.sources"
        return 1
    }
    log "$NTP_LOG" "Approved regional source profile: $target"
}

chrony_preflight() {
    local tracking

    validate_region_profile || return 1
    systemctl is-active --quiet chrony.service || {
        log "$NTP_LOG" "ERROR: chrony.service is not active"
        return 1
    }
    tracking=$(chronyc tracking 2>&1) || {
        log "$NTP_LOG" "ERROR: chronyc tracking failed: $tracking"
        return 1
    }
    if grep -q '^Leap status[[:space:]]*:[[:space:]]*Normal' <<<"$tracking" &&
       chronyc -n sources 2>/dev/null | grep -q '^\^\*'; then
        log "$NTP_LOG" "chrony is active and synchronized to a selected NTP source"
        return 0
    fi
    if [[ "$AGIBOT_ALLOW_UNQUALIFIED_TIME" == "1" ]]; then
        log "$NTP_LOG" "WARNING: continuing in explicitly approved UTC-unqualified holdover mode"
        return 0
    fi
    log "$NTP_LOG" "ERROR: UTC is not qualified; set AGIBOT_ALLOW_UNQUALIFIED_TIME=1 only under an approved holdover procedure"
    return 1
}

chrony_runtime_status() {
    local tracking

    if ! systemctl is-active --quiet chrony.service; then
        log "$NTP_LOG" "WARNING: chrony is inactive; continuing offline-capable A3 internal time distribution"
        return 0
    fi
    tracking=$(chronyc tracking 2>&1) || {
        log "$NTP_LOG" "WARNING: chrony status is unavailable; continuing offline-capable A3 internal time distribution: $tracking"
        return 0
    }
    if grep -q '^Leap status[[:space:]]*:[[:space:]]*Normal' <<<"$tracking" &&
       chronyc -n sources 2>/dev/null | grep -q '^\^\*'; then
        log "$NTP_LOG" "chrony is synchronized; external UTC coordination may run after strict preflight"
    else
        log "$NTP_LOG" "WARNING: UTC is unqualified; normal A3 operation continues, but external mocap coordination is prohibited"
    fi
    return 0
}

ensure_killed() {
    local process_name=$1
    local log_file=$2
    while pgrep -x "$process_name" >/dev/null; do
        log "$log_file" "Stopping existing $process_name process..."
        pkill -x "$process_name" || true
        sleep 2
        pgrep -x "$process_name" >/dev/null && pkill -9 -x "$process_name" || true
        sleep 1
    done
    log "$log_file" "$process_name process is stopped."
}

rotate_logs "ntp" "$MAX_LOG_FILES"
rotate_logs "ptp4l" "$MAX_LOG_FILES"
rotate_logs "phc2sys" "$MAX_LOG_FILES"

if [[ -e "$POLICY_FILE" ]]; then
    validate_root_file "$POLICY_FILE" || {
        log "$NTP_LOG" "ERROR: unsafe policy file: $POLICY_FILE"
        exit 1
    }
    # shellcheck source=/etc/default/agibot-timesync
    source "$POLICY_FILE"
fi

if [[ "${1:-}" == "--preflight" ]]; then
    chrony_preflight
    exit $?
fi
if [[ "${1:-}" == "--runtime-status" ]]; then
    chrony_runtime_status
    exit 0
fi

# Stop conflicting vendor services, but never stop or disable chrony.
SERVICES=("ptp4l" "phc2sys" "agibot_timesync" "ntpd" "ntpsec")
for SERVICE in "${SERVICES[@]}"; do
    if systemctl is-active --quiet "$SERVICE"; then
        log "$PTP4L_LOG" "Detected active $SERVICE; stopping and disabling it..."
        systemctl disable --now "$SERVICE"
        log "$PTP4L_LOG" "$SERVICE is stopped and disabled."
    else
        log "$PTP4L_LOG" "$SERVICE is not active."
    fi
done

while true; do
    if ip link show "$device_name" 2>/dev/null | grep -q "LOWER_UP"; then
        TIMESTAMPING_INFO=$(ethtool -T "$device_name" 2>&1) || {
            log "$PTP4L_LOG" "ERROR: ethtool could not query $device_name timestamping: $TIMESTAMPING_INFO"
            exit 1
        }
        grep -Eq 'PTP Hardware Clock:[[:space:]]*[0-9]+' <<<"$TIMESTAMPING_INFO" || {
            log "$PTP4L_LOG" "ERROR: $device_name does not expose a PTP hardware clock"
            exit 1
        }
        for CAPABILITY in hardware-transmit hardware-receive hardware-raw-clock; do
            grep -q "$CAPABILITY" <<<"$TIMESTAMPING_INFO" || {
                log "$PTP4L_LOG" "ERROR: $device_name lacks $CAPABILITY timestamping capability"
                exit 1
            }
        done
        log "$PTP4L_LOG" "Interface $device_name is Link Up with hardware PTP timestamping."
        break
    fi
    log "$PTP4L_LOG" "ERROR: $device_name is not ready; waiting..."
    sleep 2
done

# External UTC is not required for the robot's existing internal time domain.
# Strict qualification is an explicit gate for external mocap workflows only.
chrony_runtime_status

if pgrep -x ptp4l >/dev/null; then
    ensure_killed "ptp4l" "$PTP4L_LOG"
fi
log "$PTP4L_LOG" "Starting ptp4l master on $device_name in preserved E2E mode..."
nohup ptp4l -i "$device_name" -2 -E -m \
    2>&1 | awk '{print strftime("[%F %T]"), $0}' >> "$PTP4L_LOG" &

if pgrep -x phc2sys >/dev/null; then
    ensure_killed "phc2sys" "$PHC2SYS_LOG"
fi
log "$PHC2SYS_LOG" "Starting phc2sys (CLOCK_REALTIME -> $device_name PHC)..."
nohup phc2sys -s CLOCK_REALTIME -c "$device_name" -w -O 0 -S 10 -m \
    2>&1 | awk '{print strftime("[%F %T]"), $0}' >> "$PHC2SYS_LOG" &
