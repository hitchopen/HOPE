#!/usr/bin/env bash
# Use a fresh build/install and keep all observed failures, including inherited lint.
set -eo pipefail
repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
validation_root=${1:?Usage: verify.sh /absolute/empty/validation-output-directory}
if [[ "$validation_root" != /* || -e "$validation_root" ]]; then
  echo "Validation output must be a new absolute path: $validation_root" >&2
  exit 2
fi
mkdir -p "$validation_root"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-187}
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
if git -C "$repo" rev-parse HEAD > "$validation_root/code-sha.txt" 2>/dev/null; then
  git -C "$repo" status --porcelain > "$validation_root/worktree-status.txt"
fi
cd "$repo/hope_ws"
set +e
MAKEFLAGS='-j2 -l2' colcon --log-base "$validation_root/log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$validation_root/build" --install-base "$validation_root/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup motion_capture_tracking_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  2>&1 | tee "$validation_root/build.log"
build_status=${PIPESTATUS[0]}
printf '%s\n' "$build_status" > "$validation_root/build.exit"
set -e
if (( build_status != 0 )); then exit "$build_status"; fi
source "$validation_root/install/setup.bash"
set +e
colcon --log-base "$validation_root/test-log" test \
  --build-base "$validation_root/build" --install-base "$validation_root/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup --executor sequential \
  --event-handlers console_direct+ 2>&1 | tee "$validation_root/test.log"
test_status=${PIPESTATUS[0]}
printf '%s\n' "$test_status" > "$validation_root/test.exit"
colcon test-result --test-result-base "$validation_root/build" --verbose \
  > "$validation_root/test-result.log" 2>&1
result_status=$?
printf '%s\n' "$result_status" > "$validation_root/test-result.exit"
cat "$validation_root/test-result.log"
if (( test_status != 0 )); then exit "$test_status"; fi
exit "$result_status"
