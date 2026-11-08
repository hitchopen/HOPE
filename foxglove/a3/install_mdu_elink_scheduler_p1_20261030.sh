#!/usr/bin/env bash
# Install only the audited 2026-10-30 P1 BSP profile; preserve the legacy installer.
set -euo pipefail
umask 077
[[ $EUID == 0 ]] || { echo 'Run as root on the audited P1 MDU' >&2; exit 1; }
package=$(cd -- "$(dirname -- "$0")/.." && pwd)
staging=$(mktemp -d)
trap 'rm -rf -- "$staging"' EXIT
mkdir -p "$staging/a3" "$staging/helpers"
cp "$package/a3/install_mdu_elink_scheduler.sh" "$staging/a3/"
cp "$package/helpers/hope_elink_scheduler_p1_20261030.py" "$staging/helpers/hope_elink_scheduler.py"
bash "$staging/a3/install_mdu_elink_scheduler.sh" "$@"
