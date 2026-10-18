#!/bin/bash
# Install from a checkout; never reconnect Wi-Fi or START the robot.
set -euo pipefail
umask 077
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
[[ $EUID == 0 && $(hostname -s) == hdu ]] || { echo 'Run as root on HDU'; exit 1; }
wifi_only=false
case "${1:-}" in
    '') [[ $# == 0 ]] ;;
    --wifi-only) [[ $# == 1 ]]; wifi_only=true ;;
    *) echo "Usage: $0 [--wifi-only]"; exit 2 ;;
esac
package=$(cd -- "$(dirname -- "$0")/.." && pwd)
target=/usr/local/libexec/hope-lifecycle
[[ -f $target && -e /proc/net/rtl8852be/wifi_hdu/scan_deny ]]
backup=$(mktemp -d /var/backups/hope-wifi-policy-XXXXXXXX)
echo "BACKUP=$backup"
files=(/etc/NetworkManager/conf.d/90-hope-wifi-powersave.conf
       /usr/local/libexec/hope_wifi_policy.py /etc/systemd/system/hope-wifi-policy.service "$target")
units=(hope-wifi-policy.service)
if $wifi_only; then
    # Never opt into this profile on an unassociated robot.
    iw dev wifi_hdu link | grep -q '^Connected to '
    files+=(/usr/local/libexec/hope_catch_background.py
            /etc/polkit-1/rules.d/49-hope-wifi-only.rules /etc/hope-foxglove/wifi-only)
    units+=(zy_cellular.service)
fi
printf '%s\n' "${files[@]}" > "$backup/files.txt"
for path in "${files[@]}"; do
    if [[ -e $path ]]; then
        mkdir -p "$backup$(dirname "$path")"
        cp -a "$path" "$backup$path"
    fi
done
for unit in "${units[@]}"; do
    enabled=$(systemctl is-enabled "$unit" 2>/dev/null || true)
    active=$(systemctl is-active "$unit" 2>/dev/null || true)
    case $enabled in enabled|disabled|not-found|'') ;; *) echo "Unsupported prior unit state: $unit $enabled"; exit 1 ;; esac
    case $active in active|inactive|unknown|failed) ;; *) echo "Unit is transitioning: $unit $active"; exit 1 ;; esac
    printf '%s %s %s\n' "$unit" "${enabled:-not-found}" "$active" >> "$backup/units.txt"
done
iw dev wifi_hdu get power_save > "$backup/power-save.txt"
cat /proc/net/rtl8852be/wifi_hdu/scan_deny > "$backup/scan-deny.txt"
# Record profile overrides before mutation. Backups are root-only.
nmcli -t -f UUID,TYPE connection show > "$backup/connection-types.txt"
while IFS=: read -r uuid type; do
    if [[ $type == 802-11-wireless ]]; then
        value=$(nmcli -g 802-11-wireless.powersave connection show uuid "$uuid")
        printf '%s\n' "${value%% *}" > "$backup/$uuid.powersave"
    fi
done < "$backup/connection-types.txt"
# Preserve robot-specific lifecycle changes: insert only the admission check.
python3 - "$target" "$backup/lifecycle.new" <<'PY'
import pathlib, sys
source = pathlib.Path(sys.argv[1]).read_text()
anchor = 'preflight_hdu() {\n  local planner_prefix messages_prefix static_peers\n  require_user "$ROBOT_USER" PREFLIGHT\n'
hook = '  python3 "$(dirname "$SELF")/hope_wifi_policy.py" check || \\\n    die PREFLIGHT WIFI_POLICY_NOT_READY\n'
if hook not in source:
    if source.count(anchor) != 1:
        raise SystemExit('Unknown lifecycle layout; no files installed')
    source = source.replace(anchor, anchor + hook, 1)
pathlib.Path(sys.argv[2]).write_text(source)
PY
bash -n "$backup/lifecycle.new"
install -m 0700 "$package/a3/restore_hdu_wifi_policy.sh" "$backup/restore.sh"
rollback() {
    trap - ERR
    echo "Install failed; restoring $backup" >&2
    bash "$backup/restore.sh" || echo "RESTORE FAILED: inspect $backup before continuing" >&2
    exit 1
}
trap rollback ERR
install -m 0755 "$package/helpers/hope_wifi_policy.py" /usr/local/libexec/hope_wifi_policy.py
install -m 0644 "$package/a3/hope-wifi-policy.service" /etc/systemd/system/hope-wifi-policy.service
install -m 0644 "$package/a3/90-hope-wifi-powersave.conf" /etc/NetworkManager/conf.d/90-hope-wifi-powersave.conf
install -m 0755 "$backup/lifecycle.new" "$target"
while IFS=: read -r uuid type; do
    if [[ $type == 802-11-wireless ]]; then
        nmcli connection modify uuid "$uuid" 802-11-wireless.powersave 2
    fi
done < "$backup/connection-types.txt"
if $wifi_only; then
    install -m 0755 "$package/helpers/hope_catch_background.py" /usr/local/libexec/hope_catch_background.py
    install -d -m 0755 /etc/hope-foxglove
    install -m 0644 "$package/a3/49-hope-wifi-only.rules" /etc/polkit-1/rules.d/49-hope-wifi-only.rules
    printf 'wifi-only-v1\n' > /etc/hope-foxglove/wifi-only
    chmod 0644 /etc/hope-foxglove/wifi-only
    # Prove the same narrow permission used by the hardened START supervisor.
    # polkit automatically reloads rules; allow a brief bounded reload window.
    stopped=false
    for attempt in 1 2 3 4 5; do
        if runuser -u agi -- setpriv --no-new-privs systemctl --no-ask-password stop zy_cellular.service; then
            stopped=true; break
        fi
        sleep 1
    done
    $stopped
    systemctl disable zy_cellular.service
    [[ $(systemctl is-active zy_cellular.service || true) == inactive ]]
    [[ $(systemctl is-enabled zy_cellular.service || true) == disabled ]]
fi
nmcli general reload conf
systemctl daemon-reload
systemctl enable hope-wifi-policy.service
systemctl restart hope-wifi-policy.service
python3 /usr/local/libexec/hope_wifi_policy.py check
trap - ERR
echo 'HDU_WIFI_POLICY_INSTALLED (no Wi-Fi reconnect or robot START performed)'
echo "Rollback: sudo bash $backup/restore.sh"
