#!/bin/bash

# Chrony owns HDU CLOCK_REALTIME. Dedicated systemd units own ptp4l and
# phc2sys; this vendor launch hook only reports and validates their state.

set -u -o pipefail
export LC_ALL=C

LOG_DIR="/agibot/log/tsync"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
NTP_LOG="$LOG_DIR/ntp_${TIMESTAMP}.log"
ACTIVE_SOURCES="/etc/chrony/sources.d/hope-approved.sources"
REGION_ROOT="/etc/agibot-time/regions"
POLICY_FILE="/etc/default/agibot-timesync"
AGIBOT_ALLOW_UNQUALIFIED_TIME=0
HDU_CLOCK_SERVICES=(agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service)

mkdir -p "$LOG_DIR"

log() {
    printf '%s\n' "$*" | tee -a "$NTP_LOG"
}

validate_root_file() {
    local file=$1 perm
    [[ -f "$file" ]] || return 1
    [[ $(stat -Lc %u "$file") -eq 0 ]] || return 1
    perm=$(stat -Lc %a "$file")
    (( (8#$perm & 8#022) == 0 ))
}

validate_region_profile() {
    local target source_count
    local -a source_files

    [[ -L "$ACTIVE_SOURCES" ]] || {
        log "ERROR: active regional source link is missing: $ACTIVE_SOURCES"
        return 1
    }
    target=$(readlink -f "$ACTIVE_SOURCES") || return 1
    case "$target" in
        "$REGION_ROOT"/china/chrony.sources|\
        "$REGION_ROOT"/us/chrony.sources|\
        "$REGION_ROOT"/europe/chrony.sources) ;;
        *)
            log "ERROR: unapproved regional source target: $target"
            return 1
            ;;
    esac
    validate_root_file "$target" || {
        log "ERROR: regional source file ownership or mode is unsafe: $target"
        return 1
    }
    grep -Eq 'site\.example|REPLACE_ME' "$target" && {
        log "ERROR: placeholder NTP source remains in $target"
        return 1
    }
    source_count=$(grep -Ec '^[[:space:]]*server[[:space:]]+' "$target" || true)
    [[ $source_count -eq 3 ]] || {
        log "ERROR: regional profile must contain exactly three servers: $target"
        return 1
    }
    mapfile -t source_files < <(compgen -G '/etc/chrony/sources.d/*.sources' || true)
    [[ ${#source_files[@]} -eq 1 && "${source_files[0]}" == "$ACTIVE_SOURCES" ]] || {
        log "ERROR: /etc/chrony/sources.d must contain only hope-approved.sources"
        return 1
    }
    log "Approved regional source profile: $target"
}

clock_distribution_ready() {
    local service
    for service in "${HDU_CLOCK_SERVICES[@]}"; do
        systemctl is-active --quiet "$service" || {
            log "ERROR: $service is not active"
            return 1
        }
    done
    return 0
}

clock_distribution_status() {
    local service
    for service in "${HDU_CLOCK_SERVICES[@]}"; do
        if systemctl is-active --quiet "$service"; then
            log "$service is active and supervised by systemd"
        else
            log "WARNING: $service is not active; systemd will continue recovery attempts"
        fi
    done
    return 0
}

chrony_preflight() {
    local tracking

    validate_region_profile || return 1
    systemctl is-active --quiet chrony.service || {
        log "ERROR: chrony.service is not active"
        return 1
    }
    tracking=$(chronyc tracking 2>&1) || {
        log "ERROR: chronyc tracking failed: $tracking"
        return 1
    }
    if grep -q '^Leap status[[:space:]]*:[[:space:]]*Normal' <<<"$tracking" &&
       chronyc -n sources 2>/dev/null | grep -q '^\^\*'; then
        log "chrony is active and synchronized to a selected NTP source"
        return 0
    fi
    if [[ "$AGIBOT_ALLOW_UNQUALIFIED_TIME" == "1" ]]; then
        log "WARNING: continuing in explicitly approved UTC-unqualified holdover mode"
        return 0
    fi
    log "ERROR: UTC is not qualified; external mocap coordination is prohibited"
    return 1
}

chrony_runtime_status() {
    local tracking

    if ! systemctl is-active --quiet chrony.service; then
        log "WARNING: chrony is inactive; A3 internal time distribution remains offline-capable"
        return 0
    fi
    tracking=$(chronyc tracking 2>&1) || {
        log "WARNING: chrony status is unavailable; A3 internal time distribution remains available: $tracking"
        return 0
    }
    if grep -q '^Leap status[[:space:]]*:[[:space:]]*Normal' <<<"$tracking" &&
       chronyc -n sources 2>/dev/null | grep -q '^\^\*'; then
        log "chrony is synchronized; external UTC coordination may run after strict preflight"
    else
        log "WARNING: UTC is unqualified; normal A3 operation continues, but external mocap coordination is prohibited"
    fi
    return 0
}

if [[ -e "$POLICY_FILE" ]]; then
    validate_root_file "$POLICY_FILE" || {
        log "ERROR: unsafe policy file: $POLICY_FILE"
        exit 1
    }
    # shellcheck source=/etc/default/agibot-timesync
    source "$POLICY_FILE"
fi

case "${1:-}" in
    --preflight)
        chrony_preflight && clock_distribution_ready
        exit $?
        ;;
    --runtime-status|"")
        chrony_runtime_status
        clock_distribution_status
        exit 0
        ;;
    *)
        log "ERROR: unsupported argument: $1"
        exit 2
        ;;
esac
