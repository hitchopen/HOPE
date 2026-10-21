#!/usr/bin/env bash
# Run inside distrobox enter hope; all ROS tests use shadow outputs.
set -eo pipefail
repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
evidence=${1:?Usage: verify_integrated.sh /new/absolute/evidence-directory}
if [[ "$evidence" != /* || -e "$evidence" ]]; then
  echo "Evidence must be a new absolute directory" >&2
  exit 2
fi
mkdir -p "$evidence"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
git -C "$repo" rev-parse HEAD > "$evidence/code-sha.txt"
git -C "$repo" status --porcelain > "$evidence/worktree-status.txt"
cmake -S "$repo/hope_ws/src/hope_planner_cpp/test/native" -B "$evidence/native" \
  -DCMAKE_BUILD_TYPE=Release > "$evidence/native-build.log" 2>&1
cmake --build "$evidence/native" -j2 >> "$evidence/native-build.log" 2>&1
ctest --test-dir "$evidence/native" --output-on-failure > "$evidence/native-test.log" 2>&1
python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/integrated_numerical_gate.py" \
  --executable "$evidence/native/hope_integrated_return_sweep" --output "$evidence/outgoing-reference.json"
cd "$repo/hope_ws"
MAKEFLAGS='-j2' colcon --log-base "$evidence/log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$evidence/build" --install-base "$evidence/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup motion_capture_tracking_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  > "$evidence/ros-build.log" 2>&1
source "$evidence/install/setup.bash"
set +e
colcon --log-base "$evidence/test-log" test --build-base "$evidence/build" \
  --install-base "$evidence/install" --packages-select hope_msgs hope_planner_cpp hope_bringup \
  --executor sequential --event-handlers console_direct+ > "$evidence/ros-test.log" 2>&1
colcon test-result --test-result-base "$evidence/build" --verbose > "$evidence/ros-test-result.log" 2>&1
result=$?
cat "$evidence/ros-test-result.log"
echo "$result" > "$evidence/test-result.exit"
exit "$result"
