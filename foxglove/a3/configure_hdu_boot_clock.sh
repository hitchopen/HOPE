#!/bin/bash
# Stage the HOPE boot policy over the pinned, offline-capable clock baseline.
# Does not start bootstrap, stop consumers, step clocks, or reboot either board.
set -Eeuo pipefail
export LC_ALL=C
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

die() { printf 'STOP: %s\n' "$*" >&2; exit 1; }
MODE=${1:-}
[[ $# == 1 && "$MODE" == --enable-next-boot ]] ||
    die 'usage: sudo configure_hdu_boot_clock.sh --enable-next-boot'
[[ $EUID == 0 && $(hostname -s) == hdu ]] || die 'requires root on HDU'
SOURCE="$(dirname "$(readlink -f "$0")")/30-hope-clock-boot-ordering.conf"
TARGET=/etc/systemd/system/agibot-clock-bootstrap.service.d/30-hope-ordering.conf
[[ -f "$SOURCE" ]] || die 'stage the boot-ordering asset beside this script'

# Reuse the reviewed 30-second bootstrap, never turn runtime chrony into a
# stepper. The independent baseline installer intentionally remains unchanged.
sha256sum --check --status <<'HASHES' || die 'installed bootstrap baseline differs'
7d664c2f9125bc0f6d27a59a5b629192702d55e028a235078ed25d1891071506  /usr/local/sbin/agibot-clock-bootstrap
5011495605f36439189e5edf359a7771b610ae2cbad8b9bd7db6c2a2a7bfd3c0  /etc/chrony/chrony-bootstrap.conf
93cba5c4d8aa153e8be62b05dc0a310994133c208d3bb4b3a2f87c1bdd199328  /etc/default/agibot-clock-bootstrap
HASHES
systemctl cat agibot-clock-bootstrap.service >/dev/null
systemctl cat chrony.service >/dev/null
systemctl show agibot-clock-bootstrap.service -p Before --value |
    grep -qw chrony.service || die 'missing bootstrap-before-chrony ordering'
for consumer in agibot_roudi.service agibot_pm.service agibot_top.service agibot_ui.service; do
    systemctl show agibot-clock-bootstrap.service -p Before --value |
        grep -qw "$consumer" || die "missing bootstrap-before-$consumer ordering"
done
if grep -Eq '^[[:space:]]*makestep([[:space:]]|$)' /etc/chrony/chrony.conf; then
    die 'runtime chrony must remain slew-only'
fi
STATE=$(systemctl is-enabled agibot-clock-bootstrap.service 2>/dev/null || true)
[[ "$STATE" == enabled || "$STATE" == disabled ]] || die "unexpected bootstrap state: $STATE"

BACKUP=/var/backups/hope-boot-clock-$(date -u +%Y%m%dT%H%M%SZ)-$$
install -d -m 0700 "$BACKUP"
printf '%s\n' "$STATE" > "$BACKUP/bootstrap-enabled.before"
systemctl cat agibot-clock-bootstrap.service chrony.service agibot_pm.service > "$BACKUP/units.before"
if [[ -e "$TARGET" ]]; then
    cp -a "$TARGET" "$BACKUP/30-hope-ordering.conf"
else
    touch "$BACKUP/dropin-was-absent"
fi
rollback() {
    local rc=$?
    trap - EXIT
    if ((rc)); then
        if [[ -e "$BACKUP/dropin-was-absent" ]]; then
            rm -f -- "$TARGET"
        else
            cp -a "$BACKUP/30-hope-ordering.conf" "$TARGET"
        fi
        systemctl daemon-reload
        if [[ "$STATE" == disabled ]]; then
            systemctl disable agibot-clock-bootstrap.service
        fi
        printf 'BOOT_CLOCK_SETUP_FAILED; backup=%s\n' "$BACKUP" >&2
    fi
    exit "$rc"
}
trap rollback EXIT
install -D -o root -g root -m 0644 "$SOURCE" "$TARGET"
systemctl daemon-reload
systemd-analyze verify /etc/systemd/system/agibot-clock-bootstrap.service
systemctl enable agibot-clock-bootstrap.service
systemctl is-enabled --quiet agibot-clock-bootstrap.service
cmp "$SOURCE" "$TARGET"
printf 'HDU_BOOT_CLOCK_STAGED_NEXT_BOOT backup=%s\n' "$BACKUP"
printf '%s\n' 'No clock operation ran. Validate an attended paired reboot and fresh NTP/PTP before use.'
