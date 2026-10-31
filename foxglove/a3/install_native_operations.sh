#!/usr/bin/env bash
# Build and stage only. Activation is a separate attended, stopped operation.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_user=agi
robot_group=agi
while (($#)); do
  case "$1" in
    --robot-user) robot_user="$2"; shift 2 ;;
    --robot-group) robot_group="$2"; shift 2 ;;
    *) printf 'Unsupported option: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ "$robot_user" =~ ^[a-z_][a-z0-9_-]*$ && "$robot_group" =~ ^[a-z_][a-z0-9_-]*$ ]] || exit 2
id "$robot_user" >/dev/null
getent group "$robot_group" >/dev/null
robot_home="$(getent passwd "$robot_user" | cut -d: -f6)"
[[ "$robot_home" =~ ^/[a-zA-Z0-9_./-]+$ ]] || exit 2
# Reuse the same calibration validator and CAD table as the installed relay;
# do not fork its implementation or move it away from its data directory.
bringup_dir="$robot_home/hope_ws/install/hope_bringup/lib/hope_bringup"
for module in p1_calibration.py p1_marker_layout.py base_pose_contract.py with_fastdds_unicast.sh; do
  if [[ ! -r "$bringup_dir/$module" ]]; then
    printf 'Missing matching hope_bringup dependency: %s\n' "$bringup_dir/$module" >&2
    exit 1
  fi
done
build_dir="$(mktemp -d /tmp/hope-operations-build.XXXXXX)"
cmake -S "$root/native" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$build_dir" --target hope-operations -j2
sed -e "s/^User=agi$/User=$robot_user/" -e "s/^Group=agi$/Group=$robot_group/" \
  -e "s|/home/agi/|$robot_home/|g" \
  "$root/hope-lifecycle-supervisor.service" > "$build_dir/hope-lifecycle-supervisor.service"
sudo /usr/bin/python3 "$root/hope_operations_deploy.py" stage --source "$root" \
  --binary "$build_dir/hope-operations" --unit "$build_dir/hope-lifecycle-supervisor.service"
printf 'Build evidence: %s\n' "$build_dir"
printf 'Activation requires a separate attended command; see native/README.md.\n'
