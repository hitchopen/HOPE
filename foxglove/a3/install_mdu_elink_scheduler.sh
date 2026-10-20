#!/usr/bin/env bash
# Install the guarded scheduling fix and its sudo policy without restarting HAL.
set -euo pipefail
umask 077
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
[[ $EUID == 0 ]] || { echo 'Run as root on the audited MDU' >&2; exit 1; }
robot_user=agi
if [[ $# == 2 && $1 == --robot-user ]]; then
  robot_user=$2
elif [[ $# != 0 ]]; then
  echo "usage: $0 [--robot-user USER]" >&2; exit 1
fi
[[ $robot_user =~ ^[a-z_][a-z0-9_-]*\$?$ ]] || {
  echo 'Invalid robot account name' >&2; exit 1;
}
robot_uid=$(id -u "$robot_user")
[[ $robot_uid != 0 ]] || {
  echo 'Expected an existing non-root robot account' >&2; exit 1;
}
for command in /usr/bin/python3 /usr/bin/sudo /usr/sbin/visudo /usr/sbin/runuser; do
  [[ -x $command ]] || { echo "Missing required command: $command" >&2; exit 1; }
done
package=$(cd -- "$(dirname -- "$0")/.." && pwd)
target=/usr/local/libexec/hope-lifecycle
helper=/usr/local/libexec/hope_elink_scheduler.py
sudoers=/etc/sudoers.d/hope-elink-scheduler
[[ -f $target && ! -L $target && ! -L $helper && ! -L $sudoers ]]
# Use the candidate scheduler's audit, not the potentially older installed one.
/usr/bin/python3 -I "$package/helpers/hope_elink_scheduler.py" check-hardware --lifecycle "$target"
# Refuse writable/non-root install paths before granting privileges. The helper
# repeats this check when invoked through sudo, and its -I shebang ignores user
# Python paths and startup modules.
/usr/bin/python3 -I - "$target" "$helper" "$sudoers" <<'PY_PATHS'
from pathlib import Path
import stat
import sys
for argument in sys.argv[1:]:
    file = Path(argument)
    for path in (file, *file.parents):
        if path == file and not path.exists():
            continue
        info = path.lstat()
        expected = stat.S_ISREG if path == file else stat.S_ISDIR
        if not expected(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
            raise SystemExit(f'Expected root-owned install path without group/other write access: {path}')
PY_PATHS
/usr/sbin/visudo -c >/dev/null
backup=$(mktemp -d /var/backups/hope-elink-scheduler-XXXXXXXX)
echo "BACKUP=$backup"
cp -a "$target" "$backup/hope-lifecycle"
if [[ -e $helper ]]; then cp -a "$helper" "$backup/hope_elink_scheduler.py"; fi
if [[ -e $sudoers ]]; then cp -a "$sudoers" "$backup/sudoers"; fi
# Preserve field-local changes while upgrading both old and current hook forms.
/usr/bin/python3 -I - "$target" "$backup/lifecycle.new" <<'PY'
from pathlib import Path
import sys
source = Path(sys.argv[1]).read_text()
anchor = '  check_t3_elink_ready HAL\n'
legacy_hook = '  sudo -n python3 "$(dirname "$SELF")/hope_elink_scheduler.py" apply --config "$elink_config" || \\\n    die HAL ELINK_SCHEDULER_NOT_READY\n'
hook = legacy_hook.replace('sudo -n python3 ', 'sudo -n ')
if hook not in source:
    if source.count(legacy_hook) == 1:
        source = source.replace(legacy_hook, hook, 1)
    elif source.count(anchor) == 1 and 'ELINK_SCHEDULER_NOT_READY' not in source:
        source = source.replace(anchor, anchor + hook, 1)
    else:
        raise SystemExit('Unknown lifecycle layout; no files installed')
preflight_anchor = '  render_t3_elink_config >/dev/null || die "$step" T3_ELINK_GRIPPER_CONFIG_INVALID\n'
old_preflight = '  python3 "$(dirname "$SELF")/hope_elink_scheduler.py" check-hardware \\\n    --lifecycle "$SELF" >/dev/null || die "$step" ELINK_HARDWARE_PROFILE_NOT_AUDITED\n'
preflight = old_preflight + '''  sudo -n "$(dirname "$SELF")/hope_elink_scheduler.py" check-privileges >/dev/null || \\
    die "$step" ELINK_SCHEDULER_SUDO_NOT_READY
  sudo -n -l -- "$(dirname "$SELF")/hope_elink_scheduler.py" apply \\
    --config "/tmp/hope_real/$SESSION_ID/mdu/system/hal_elink_a3_gripper.yaml" >/dev/null || \\
    die "$step" ELINK_SCHEDULER_APPLY_NOT_PERMITTED
'''
if preflight not in source:
    if source.count(old_preflight) == 1:
        source = source.replace(old_preflight, preflight, 1)
    elif source.count(preflight_anchor) == 1 and 'ELINK_HARDWARE_PROFILE_NOT_AUDITED' not in source:
        source = source.replace(preflight_anchor, preflight + preflight_anchor, 1)
    else:
        raise SystemExit('Unknown hardware preflight layout; no files installed')
Path(sys.argv[2]).write_text(source)
PY
bash -n "$backup/lifecycle.new"
# The wildcard is limited further by the helper's exact argument and session-path
# validation. Never authorize the Python interpreter, arbitrary files or commands.
printf '%s ALL=(root) NOPASSWD: NOSETENV: %s check-privileges, %s apply --config /tmp/hope_real/*/mdu/system/hal_elink_a3_gripper.yaml\n' \
  "$robot_user" "$helper" "$helper" > "$backup/sudoers.new"
chmod 0440 "$backup/sudoers.new"
/usr/sbin/visudo -cf "$backup/sudoers.new" >/dev/null
# Roll back the helper, lifecycle and dedicated rule together if installation or
# the real unprivileged-account probe fails. No vendor service is touched.
committed=false
rollback() {
  if [[ $committed != true ]]; then
    cp -a "$backup/hope-lifecycle" "$target"
    if [[ -e $backup/hope_elink_scheduler.py ]]; then
      cp -a "$backup/hope_elink_scheduler.py" "$helper"
    else
      rm -f -- "$helper"
    fi
    if [[ -e $backup/sudoers ]]; then
      cp -a "$backup/sudoers" "$sudoers"
    else
      rm -f -- "$sudoers"
    fi
    echo "Installation failed; restored files from $backup" >&2
  fi
}
trap rollback EXIT
install -o root -g root -m 0755 "$package/helpers/hope_elink_scheduler.py" "$helper"
install -o root -g root -m 0755 "$backup/lifecycle.new" "$target"
install -o root -g root -m 0440 "$backup/sudoers.new" "$sudoers"
/usr/sbin/visudo -c >/dev/null
/usr/sbin/runuser -u "$robot_user" -- /usr/bin/sudo -n -- "$helper" check-privileges
/usr/sbin/runuser -u "$robot_user" -- /usr/bin/sudo -n -l -- "$helper" apply \
  --config /tmp/hope_real/install_probe/mdu/system/hal_elink_a3_gripper.yaml >/dev/null
committed=true
printf 'MDU_ELINK_SCHEDULER_INSTALLED user=%s sudoers=%s (no robot restart)\n' "$robot_user" "$sudoers"
