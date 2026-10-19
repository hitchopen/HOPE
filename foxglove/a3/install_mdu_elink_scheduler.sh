#!/usr/bin/env bash
# Install the guarded scheduling fix without restarting HAL or Runner.
set -euo pipefail
umask 077
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
[[ $EUID == 0 ]] || {
  echo 'Run as root on the audited MDU' >&2; exit 1;
}
package=$(cd -- "$(dirname -- "$0")/.." && pwd)
target=/usr/local/libexec/hope-lifecycle
helper=/usr/local/libexec/hope_elink_scheduler.py
[[ -f $target && ! -L $target && ! -L $helper ]]
# Use the candidate scheduler's audit, not the potentially older installed one.
# Validate model, binary hashes, config paths and the installed lifecycle before
# creating backups or changing any installed file. P1/Ultra layouts are rejected.
python3 "$package/helpers/hope_elink_scheduler.py" check-hardware --lifecycle "$target"
backup=$(mktemp -d /var/backups/hope-elink-scheduler-XXXXXXXX)
echo "BACKUP=$backup"
cp -a "$target" "$backup/hope-lifecycle"
if [[ -e $helper ]]; then cp -a "$helper" "$backup/hope_elink_scheduler.py"; fi
# Preserve other local lifecycle changes. Add the read-only hardware preflight
# and scheduling hook, and reject any unrecognized launch layout.
python3 - "$target" "$backup/lifecycle.new" <<'PY'
from pathlib import Path
import sys
source = Path(sys.argv[1]).read_text()
anchor = '  check_t3_elink_ready HAL\n'
hook = '  sudo -n python3 "$(dirname "$SELF")/hope_elink_scheduler.py" apply --config "$elink_config" || \\\n    die HAL ELINK_SCHEDULER_NOT_READY\n'
if hook not in source:
    if source.count(anchor) != 1:
        raise SystemExit('Unknown lifecycle layout; no files installed')
    source = source.replace(anchor, anchor + hook, 1)
preflight_anchor = '  render_t3_elink_config >/dev/null || die "$step" T3_ELINK_GRIPPER_CONFIG_INVALID\n'
preflight = '  python3 "$(dirname "$SELF")/hope_elink_scheduler.py" check-hardware \\\n    --lifecycle "$SELF" >/dev/null || die "$step" ELINK_HARDWARE_PROFILE_NOT_AUDITED\n'
if preflight not in source:
    if source.count(preflight_anchor) != 1:
        raise SystemExit('Unknown hardware preflight layout; no files installed')
    source = source.replace(preflight_anchor, preflight + preflight_anchor, 1)
Path(sys.argv[2]).write_text(source)
PY
bash -n "$backup/lifecycle.new"
python3 - "$package/helpers/hope_elink_scheduler.py" <<'PY'
from pathlib import Path
import sys
compile(Path(sys.argv[1]).read_text(), sys.argv[1], 'exec')
PY
install -o root -g root -m 0755 "$package/helpers/hope_elink_scheduler.py" "$helper"
install -o root -g root -m 0755 "$backup/lifecycle.new" "$target"
echo 'MDU_ELINK_SCHEDULER_INSTALLED (apply to the current session separately; no robot restart)'
