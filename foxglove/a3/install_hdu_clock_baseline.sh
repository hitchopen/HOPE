#!/bin/bash
# Install the pinned, vendor-owned HDU clock baseline on a fresh A3 image.
# The install path is attended, backs up every replaced target, and restores
# the original files and service state if any post-stop check fails.

set -Eeuo pipefail
umask 022
export LC_ALL=C
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

readonly EXPECTED_MANIFEST_SHA256="92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168"
readonly EXPECTED_TIMESYNC_SHA256="80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b"
readonly EXPECTED_README_SHA256="9e6caeee749773437f36c43054594a3a4cb45e3963f88eadb81d79d77189c47a"
readonly MODE="${1:-}"
readonly PACKAGE_ARGUMENT="${2:-}"

die() {
    printf 'STOP: %s\n' "$*" >&2
    exit 1
}

usage() {
    cat >&2 <<'EOF'
usage:
  install_hdu_clock_baseline.sh --verify-package-only PACKAGE_ROOT
  sudo install_hdu_clock_baseline.sh --install PACKAGE_ROOT
  sudo install_hdu_clock_baseline.sh --install-for-paired-reboot PACKAGE_ROOT
EOF
    exit 2
}

[[ "$MODE" == "--verify-package-only" || "$MODE" == "--install" ||
   "$MODE" == "--install-for-paired-reboot" ]] || usage
[[ -n "$PACKAGE_ARGUMENT" ]] || usage
PKG_ROOT="$(readlink -f -- "$PACKAGE_ARGUMENT")"
readonly PKG_ROOT

verify_package() {
    [[ -d "$PKG_ROOT" ]] || die "clock package directory is missing: $PKG_ROOT"
    [[ -f "$PKG_ROOT/MANIFEST.sha256" ]] || die "clock package manifest is missing"
    [[ "$(sha256sum "$PKG_ROOT/MANIFEST.sha256" | awk '{print $1}')" == \
        "$EXPECTED_MANIFEST_SHA256" ]] || die "clock package manifest identity mismatch"
    (
        cd "$PKG_ROOT"
        sha256sum -c MANIFEST.sha256
    )
    [[ "$(sha256sum "$PKG_ROOT/README.md" | awk '{print $1}')" == \
        "$EXPECTED_README_SHA256" ]] || die "clock package README identity mismatch"
    [[ "$(sha256sum "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" | awk '{print $1}')" == \
        "$EXPECTED_TIMESYNC_SHA256" ]] || die "clock package timesync identity mismatch"
    bash -n "$PKG_ROOT/opt/agibot/entry/system/timesync.sh"
    bash -n "$PKG_ROOT/usr/local/sbin/agibot-clock-bootstrap"
    bash -n "$PKG_ROOT/usr/local/sbin/agibot-time-region"
    grep -F -- '--tx_timestamp_timeout "$PTP4L_TX_TIMESTAMP_TIMEOUT_MS"' \
        "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" >/dev/null
    grep -Fx 'After=chrony.service' \
        "$PKG_ROOT/etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf" >/dev/null
    ! grep -Eq '^[[:space:]]*(makestep|allow|local|sourcedir[[:space:]]+/run/chrony-dhcp)' \
        "$PKG_ROOT/etc/chrony/chrony.conf"
    [[ ! -e "$PKG_ROOT/etc/systemd/system/agibot-hdu-ptp4l.service" ]]
    [[ ! -e "$PKG_ROOT/etc/systemd/system/agibot-hdu-phc2sys.service" ]]
}

verify_package
if [[ "$MODE" == "--verify-package-only" ]]; then
    echo "HDU_CLOCK_PACKAGE_VERIFY_OK"
    exit 0
fi

[[ $EUID -eq 0 ]] || die "run --install through sudo"
[[ "$(id -un)" == root ]] || die "installer did not obtain root identity"
[[ "$(hostname -s)" == hdu ]] || die "target hostname is not hdu"
[[ "$(uname -m)" == aarch64 ]] || die "target architecture is not aarch64"
[[ -e /opt/agibot/entry/system/timesync.sh ]] || die "vendor timesync.sh is missing"
# New BSPs gate master startup on MDU uptime. A hot PM restart cannot restore
# that topology; fail before mutation rather than roll back after stopping it.
if grep -Fq 'system/timesync_guard.py' /opt/agibot/entry/run_agibot.sh; then
    [[ "$MODE" == "--install-for-paired-reboot" ]] ||
        die "guarded BSP requires --install-for-paired-reboot; see new_robot_commissioning.md"
elif [[ "$MODE" == "--install-for-paired-reboot" ]]; then
    die "paired-reboot mode requires the guarded BSP startup"
fi
for TOOL in apt-get awk date dpkg-query find grep install ip pgrep pkill \
    readlink rm seq sha256sum sleep stat systemctl tar; do
    command -v "$TOOL" >/dev/null || die "required installer tool is missing: $TOOL"
done
if [[ "$MODE" == "--install-for-paired-reboot" ]]; then
    # Paired installation changes the HDU clock while the downstream board is
    # offline for maintenance. Unknown/reachable-but-running is not sufficient.
    /usr/sbin/runuser -u agi -- /usr/bin/ssh \
        -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=yes \
        agi@10.42.10.12 'bash -s' <<'MDU_STOPPED'
set -euo pipefail
[[ "$(hostname -s)" == mdu ]]
for unit in agibot_pm agibot_ui agibot_top agibot_roudi; do
    state=$(systemctl show "$unit" -p ActiveState --value)
    [[ "$state" == inactive || "$state" == failed ]]
done
! pgrep -f '[a]3_deploy_onnx_ref_pingpong|[a]imrt_main_hal'
! pgrep -x ptp4l
! pgrep -x phc2sys
MDU_STOPPED
fi
systemctl cat agibot_pm.service >/dev/null
systemctl is-active --quiet agibot_pm.service || die "agibot_pm.service must be active before maintenance"
ip -4 -o address show dev eth_hdu | grep -F '10.42.10.10/24' >/dev/null

for SERVICE in \
    hope-lifecycle-supervisor.service \
    hope-command-proxy.service \
    hope-time-calibration.service; do
    if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
        die "$SERVICE is active; stop the HOPE lifecycle before clock installation"
    fi
done

CURRENT_TIMESYNC_SHA256="$(sha256sum /opt/agibot/entry/system/timesync.sh | awk '{print $1}')"
if [[ "$CURRENT_TIMESYNC_SHA256" != "$EXPECTED_TIMESYNC_SHA256" ]]; then
    for TOKEN in ntp2rtc.py ptp4l phc2sys; do
        grep -q "$TOKEN" /opt/agibot/entry/system/timesync.sh || \
            die "unrecognized vendor timesync.sh: missing $TOKEN"
    done
fi

MISSING_PACKAGES=()
for PACKAGE in chrony ethtool linuxptp; do
    dpkg-query -W -f='${db:Status-Status}\n' "$PACKAGE" 2>/dev/null | \
        grep -qx installed || MISSING_PACKAGES+=("$PACKAGE")
done
printf 'Missing dependency packages: %s\n' "${MISSING_PACKAGES[*]:-none}"

printf '%s\n' \
    'This operation stops A3 application/time processes, installs the pinned clock baseline,' \
    'then restores vendor services, or leaves them stopped in paired-reboot mode.' \
    'The robot must be physically supported,' \
    'motion/external commands must be disabled, and the physical E-stop must be reachable.'
read -r -p 'Type INSTALL_HDU_CLOCK_BASELINE to continue: ' CONFIRMATION
[[ "$CONFIRMATION" == INSTALL_HDU_CLOCK_BASELINE ]] || die "operator confirmation not received"

readonly BACKUP_DIR="/var/backups/hope-hdu-clock-baseline-$(date -u +%Y%m%dT%H%M%SZ)"
install -d -o root -g root -m 0700 "$BACKUP_DIR"

ACTIVE_SERVICES=()
for SERVICE in agibot_roudi agibot_top agibot_ui agibot_pm; do
    systemctl is-active --quiet "$SERVICE" && ACTIVE_SERVICES+=("$SERVICE")
done
printf '%s\n' "${ACTIVE_SERVICES[@]}" > "$BACKUP_DIR/active-services.before"
printf '%s\n' "$CURRENT_TIMESYNC_SHA256" > "$BACKUP_DIR/timesync.sha256.before"
printf '%s\n' "${MISSING_PACKAGES[@]}" > "$BACKUP_DIR/missing-packages.before"

CHRONY_ENABLED_STATE="$(systemctl is-enabled chrony.service 2>/dev/null || true)"
CHRONY_WAS_ACTIVE=0
systemctl is-active --quiet chrony.service && CHRONY_WAS_ACTIVE=1
printf 'enabled_state=%s\nactive=%s\n' \
    "$CHRONY_ENABLED_STATE" "$CHRONY_WAS_ACTIVE" \
    > "$BACKUP_DIR/chrony-state.before"

MUTATION_STARTED=0
INSTALL_SUCCEEDED=0
FILES_BACKED_UP=0
PACKAGE_TARGETS=()

restore_chrony_state() {
    local rc=0
    case "$CHRONY_ENABLED_STATE" in
        masked)
            systemctl mask chrony.service >/dev/null 2>&1 || rc=1
            ;;
        enabled)
            systemctl unmask chrony.service >/dev/null 2>&1 || rc=1
            systemctl enable chrony.service >/dev/null 2>&1 || rc=1
            ;;
        disabled)
            systemctl unmask chrony.service >/dev/null 2>&1 || rc=1
            systemctl disable chrony.service >/dev/null 2>&1 || rc=1
            ;;
        not-found|'')
            systemctl unmask chrony.service >/dev/null 2>&1 || rc=1
            if systemctl cat chrony.service >/dev/null 2>&1; then
                systemctl disable chrony.service >/dev/null 2>&1 || rc=1
            fi
            ;;
        *)
            systemctl unmask chrony.service >/dev/null 2>&1 || rc=1
            ;;
    esac
    if ((CHRONY_WAS_ACTIVE)); then
        systemctl start chrony.service >/dev/null 2>&1 || rc=1
    elif systemctl cat chrony.service >/dev/null 2>&1; then
        systemctl stop chrony.service >/dev/null 2>&1 || rc=1
    fi
    return "$rc"
}

restore_vendor_services() {
    local service rc=0
    for service in agibot_roudi agibot_top agibot_ui agibot_pm; do
        if printf '%s\n' "${ACTIVE_SERVICES[@]}" | grep -qx "$service"; then
            systemctl start "$service" >/dev/null 2>&1 || rc=1
        fi
    done
    return "$rc"
}

rollback_install() {
    local rollback_rc=0
    set +e
    printf 'ROLLBACK: restoring pre-install files and service state from %s\n' \
        "$BACKUP_DIR" >&2
    for SERVICE in agibot_pm agibot_ui agibot_top agibot_roudi; do
        if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
            systemctl stop "$SERVICE" >/dev/null 2>&1 || rollback_rc=1
        fi
    done
    if systemctl is-active --quiet chrony.service 2>/dev/null; then
        systemctl stop chrony.service >/dev/null 2>&1 || rollback_rc=1
    fi
    pkill -TERM -x phc2sys >/dev/null 2>&1 || true
    pkill -TERM -x ptp4l >/dev/null 2>&1 || true
    if ((FILES_BACKED_UP)); then
        while IFS= read -r REL; do
            if [[ -n "$REL" ]]; then
                rm -f -- "/$REL" || rollback_rc=1
            fi
        done < "$BACKUP_DIR/absent-targets.before"
        tar -C / -xpf "$BACKUP_DIR/rootfs.tar" || rollback_rc=1
    fi
    systemctl daemon-reload >/dev/null 2>&1 || rollback_rc=1
    restore_chrony_state || rollback_rc=1
    restore_vendor_services || rollback_rc=1
    if ((rollback_rc)); then
        echo "HDU_CLOCK_BASELINE_ROLLBACK_INCOMPLETE" >&2
    else
        echo "HDU_CLOCK_BASELINE_ROLLED_BACK" >&2
    fi
}

on_exit() {
    local rc=$?
    trap - EXIT INT TERM
    if ((MUTATION_STARTED)) && ((INSTALL_SUCCEEDED == 0)); then
        rollback_install
    fi
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT TERM

MUTATION_STARTED=1
for SERVICE in agibot_pm agibot_ui agibot_top agibot_roudi; do
    if systemctl is-active --quiet "$SERVICE"; then
        systemctl stop "$SERVICE"
    fi
done
systemctl mask --now chrony.service >/dev/null 2>&1 || true
pkill -TERM -f '/ntp2rtc.py' >/dev/null 2>&1 || true
pkill -TERM -x phc2sys >/dev/null 2>&1 || true
pkill -TERM -x ptp4l >/dev/null 2>&1 || true
sleep 2
if pgrep -a -f '[n]tp2rtc.py|[c]hronyd|[p]tp4l|[p]hc2sys'; then
    die "a time process remains after entering maintenance"
fi

if ((${#MISSING_PACKAGES[@]})); then
    apt-get update
    apt-get install --no-install-recommends --no-upgrade -y \
        "${MISSING_PACKAGES[@]}"
fi
for TOOL in chronyd chronyc ethtool ptp4l phc2sys systemd-analyze; do
    command -v "$TOOL" >/dev/null || die "required clock tool is missing: $TOOL"
done

TIMESTAMPING_INFO="$(ethtool -T eth_hdu 2>&1)" || \
    die "cannot query eth_hdu timestamping: $TIMESTAMPING_INFO"
grep -Eq 'PTP Hardware Clock:[[:space:]]*[0-9]+' <<<"$TIMESTAMPING_INFO" || \
    die "eth_hdu does not expose a PTP hardware clock"
for CAPABILITY in hardware-transmit hardware-receive hardware-raw-clock; do
    grep -q "$CAPABILITY" <<<"$TIMESTAMPING_INFO" || \
        die "eth_hdu lacks $CAPABILITY timestamping"
done

mapfile -t EXTRA_SOURCES < <(
    find /etc/chrony/sources.d -maxdepth 1 -type f -name '*.sources' \
        ! -name hope-approved.sources -print 2>/dev/null
)
((${#EXTRA_SOURCES[@]} == 0)) || \
    die "unexpected chrony .sources file: ${EXTRA_SOURCES[*]}"
if [[ -e /etc/chrony/sources.d/hope-approved.sources || \
      -L /etc/chrony/sources.d/hope-approved.sources ]]; then
    [[ -L /etc/chrony/sources.d/hope-approved.sources ]] || \
        die "hope-approved.sources exists but is not a symlink"
    [[ "$(readlink /etc/chrony/sources.d/hope-approved.sources)" == \
        /etc/agibot-time/current/chrony.sources ]] || \
        die "hope-approved.sources points to an unexpected target"
fi

chronyd -p -f "$PKG_ROOT/etc/chrony/chrony.conf" >/dev/null
chronyd -p -f "$PKG_ROOT/etc/chrony/chrony-bootstrap.conf" >/dev/null

mapfile -t PACKAGE_TARGETS < <(
    awk '$2 != "README.md" {print $2}' "$PKG_ROOT/MANIFEST.sha256"
)
ROLLBACK_TARGETS=(
    "${PACKAGE_TARGETS[@]}"
    etc/agibot-time/current
    etc/chrony/sources.d/hope-approved.sources
)
EXISTING_TARGETS=()
ABSENT_TARGETS=()
for REL in "${ROLLBACK_TARGETS[@]}"; do
    [[ "$REL" != /* && "$REL" != *'..'* ]] || die "unsafe package target: $REL"
    if [[ -e "/$REL" || -L "/$REL" ]]; then
        EXISTING_TARGETS+=("$REL")
    else
        ABSENT_TARGETS+=("$REL")
    fi
done
printf '%s\n' "${ABSENT_TARGETS[@]}" > "$BACKUP_DIR/absent-targets.before"
if ((${#EXISTING_TARGETS[@]})); then
    tar -C / -cpf "$BACKUP_DIR/rootfs.tar" "${EXISTING_TARGETS[@]}"
    (
        cd /
        sha256sum "${EXISTING_TARGETS[@]}" 2>/dev/null || true
    ) > "$BACKUP_DIR/existing-targets.sha256.before"
else
    tar -C / -cpf "$BACKUP_DIR/rootfs.tar" --files-from /dev/null
fi
FILES_BACKED_UP=1

for REGION in china us europe; do
    install -D -o root -g root -m 0644 \
        "$PKG_ROOT/etc/agibot-time/regions/$REGION/chrony.sources" \
        "/etc/agibot-time/regions/$REGION/chrony.sources"
done
for REL in \
    etc/chrony/chrony.conf \
    etc/chrony/chrony-bootstrap.conf \
    etc/default/agibot-timesync \
    etc/default/agibot-clock-bootstrap \
    etc/systemd/system/agibot-clock-bootstrap.service \
    etc/systemd/system/chrony.service.d/agibot-controls.conf \
    etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf; do
    install -D -o root -g root -m 0644 "$PKG_ROOT/$REL" "/$REL"
done
for REL in usr/local/sbin/agibot-time-region usr/local/sbin/agibot-clock-bootstrap; do
    install -D -o root -g root -m 0755 "$PKG_ROOT/$REL" "/$REL"
done

VENDOR_UID="$(stat -Lc %u /opt/agibot/entry/system/timesync.sh)"
VENDOR_GID="$(stat -Lc %g /opt/agibot/entry/system/timesync.sh)"
VENDOR_MODE="$(stat -Lc %a /opt/agibot/entry/system/timesync.sh)"
install -D -o "$VENDOR_UID" -g "$VENDOR_GID" -m "$VENDOR_MODE" \
    "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" \
    /opt/agibot/entry/system/timesync.sh

bash -n /opt/agibot/entry/system/timesync.sh
bash -n /usr/local/sbin/agibot-clock-bootstrap
bash -n /usr/local/sbin/agibot-time-region
chronyd -p -f /etc/chrony/chrony.conf >/dev/null
chronyd -p -f /etc/chrony/chrony-bootstrap.conf >/dev/null
systemctl daemon-reload
# Verify the unit only after its absolute ExecStart target has been installed.
# systemd-analyze resolves ExecStart against the live root even when the unit
# file itself is passed from the staged package.
systemd-analyze verify \
    /etc/systemd/system/agibot-clock-bootstrap.service
grep -Fx 'After=chrony.service' \
    /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf >/dev/null
if systemctl cat agibot_pm.service | grep -q 'timesync.sh --preflight'; then
    die "normal A3 startup is incorrectly gated on NTP preflight"
fi

/usr/local/sbin/agibot-time-region china
systemctl disable --now agibot-clock-bootstrap.service
systemctl unmask chrony.service
systemctl enable --now chrony.service
systemctl is-enabled --quiet chrony.service
systemctl is-active --quiet chrony.service
[[ "$(systemctl is-enabled agibot-clock-bootstrap.service 2>/dev/null || true)" == \
    disabled ]]

if [[ "$MODE" == "--install-for-paired-reboot" ]]; then
    # Leave time consumers stopped for an attended bootstrap and paired reboot.
    # This branch stages the baseline; it does not claim runtime acceptance.
    for SERVICE in "${ACTIVE_SERVICES[@]}"; do
        ! systemctl is-active --quiet "$SERVICE" || die "vendor consumer unexpectedly active"
    done
    ! pgrep -x ptp4l || die "PTP master unexpectedly active"
    ! pgrep -x phc2sys || die "PHC consumer unexpectedly active"
else
restore_vendor_services
for SERVICE in "${ACTIVE_SERVICES[@]}"; do
    systemctl is-active --quiet "$SERVICE" || \
        die "originally active service did not restart: $SERVICE"
done
for _ in $(seq 1 30); do
    if [[ "$(pgrep -xc ptp4l || true)" == 1 && \
          "$(pgrep -xc phc2sys || true)" == 1 ]]; then
        break
    fi
    sleep 1
done
[[ "$(pgrep -xc ptp4l || true)" == 1 ]] || die "expected one HDU ptp4l process"
[[ "$(pgrep -xc phc2sys || true)" == 1 ]] || die "expected one HDU phc2sys process"
pgrep -a -x ptp4l | \
    grep -F -- '-i eth_hdu -2 -E -m --tx_timestamp_timeout 100' >/dev/null
pgrep -a -x phc2sys | \
    grep -F -- '-s CLOCK_REALTIME -c eth_hdu' >/dev/null
for PID in $(pgrep -x ptp4l) $(pgrep -x phc2sys); do
    grep -Eq '^[^:]*:[^:]*:/system[.]slice/agibot_pm[.]service(/.*)?$' \
        "/proc/$PID/cgroup" || die "clock pid $PID is not owned by agibot_pm.service"
done

fi

for REL in "${PACKAGE_TARGETS[@]}"; do
    EXPECTED_HASH="$(awk -v rel="$REL" '$2 == rel {print $1}' "$PKG_ROOT/MANIFEST.sha256")"
    [[ "$(sha256sum "/$REL" | awk '{print $1}')" == "$EXPECTED_HASH" ]] || \
        die "installed clock asset hash mismatch: /$REL"
done
[[ "$(readlink -f /etc/chrony/sources.d/hope-approved.sources)" == \
    /etc/agibot-time/regions/china/chrony.sources ]]

chronyc tracking || true
chronyc -n sources -v || true
/opt/agibot/entry/system/timesync.sh --runtime-status || true

INSTALL_SUCCEEDED=1
printf 'HDU_CLOCK_BASELINE_BACKUP=%s\n' "$BACKUP_DIR"
if [[ "$MODE" == "--install-for-paired-reboot" ]]; then
    echo "HDU_CLOCK_BASELINE_STAGED_PENDING_PAIRED_REBOOT"
else
    echo "HDU_CLOCK_BASELINE_INSTALLED_OK"
fi
