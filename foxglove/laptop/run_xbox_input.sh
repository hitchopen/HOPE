#!/usr/bin/env bash
# Run inside the hope ROS container: LT velocity, fixed A/B/X/Y Runner
# service requests, and LB+RB through the existing E-stop service.
set -e
source /opt/ros/jazzy/setup.bash
hope_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
xbox_runtime="$HOME/.local/share/hope-xbox"
mkdir -p "$xbox_runtime"
exec 9>"$xbox_runtime/input.lock"
flock -n 9 || { echo 'Xbox input already running' >&2; exit 1; }
exec "$hope_root/hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh" \
  --domain-id 232 --peer "${HOPE_XBOX_HDU_IP:-10.42.20.10}" -- \
  "$xbox_runtime/venv/bin/python" "$hope_root/foxglove/laptop/hope_xbox_preview.py" --ros --control
