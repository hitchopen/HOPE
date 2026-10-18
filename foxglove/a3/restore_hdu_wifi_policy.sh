#!/bin/bash
# Installed inside a root-owned backup by install_hdu_wifi_policy.sh.
set -euo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
[[ $EUID == 0 && $(hostname -s) == hdu ]] || exit 1
backup=$(cd -- "$(dirname -- "$0")" && pwd)
[[ $backup == /var/backups/hope-wifi-policy-* && -f $backup/units.txt && -f $backup/files.txt ]]
# Restores the pre-install snapshot. Run while stopped; later edits are overwritten.
systemctl stop hope-wifi-policy.service || true
systemctl disable hope-wifi-policy.service || true
while IFS= read -r path; do
    if [[ -e $backup$path ]]; then
        cp -a "$backup$path" "$path"
    else
        rm -f -- "$path"
    fi
done < "$backup/files.txt"
while IFS=: read -r uuid type; do
    if [[ $type == 802-11-wireless ]]; then
        nmcli connection modify uuid "$uuid" 802-11-wireless.powersave "$(< "$backup/$uuid.powersave")"
    fi
done < "$backup/connection-types.txt"
nmcli general reload conf
systemctl daemon-reload
case "$(< "$backup/power-save.txt")" in
    'Power save: on') iw dev wifi_hdu set power_save on ;;
    'Power save: off') iw dev wifi_hdu set power_save off ;;
esac
if [[ -e /proc/net/rtl8852be/wifi_hdu/scan_deny ]]; then
    if grep -q '^scan_deny set by /proc is enable$' "$backup/scan-deny.txt"; then
        echo 1 > /proc/net/rtl8852be/wifi_hdu/scan_deny
    else
        echo 0 > /proc/net/rtl8852be/wifi_hdu/scan_deny
    fi
fi
while read -r unit enabled active; do
    case $enabled in
        enabled) systemctl enable "$unit" ;;
        disabled) systemctl disable "$unit" ;;
    esac
    if [[ $active == active ]]; then
        systemctl restart "$unit"
    elif [[ $enabled != not-found ]]; then
        systemctl stop "$unit"
    fi
done < "$backup/units.txt"
echo "Restored $backup (no robot START performed)"
