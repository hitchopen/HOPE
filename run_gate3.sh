#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The installed simulation stack lives in hope on this workstation. Preserve
# argv exactly, including paths containing spaces; never source robot env.sh.
if [[ ! -r /opt/ros/jazzy/setup.bash && -z "${CONTAINER_ID:-}" ]] && command -v distrobox >/dev/null; then
  exec distrobox enter hope -- bash "$ROOT/run_gate3.sh" "$@"
fi
exec python3 "$ROOT/a3_deploy/a3_deploy_example/scripts/run_gate3.py" "$@"
