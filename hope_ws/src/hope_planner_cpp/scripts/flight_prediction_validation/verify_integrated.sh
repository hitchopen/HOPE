#!/usr/bin/env bash
# Run inside distrobox enter hope; all ROS tests use shadow outputs.
set -eo pipefail
default_repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
repo=${HOPE_VERIFICATION_SOURCE_ROOT:-$default_repo}
evidence=${1:?Usage: verify_integrated.sh /new/absolute/evidence-directory}
if [[ "$evidence" != /* || -e "$evidence" ]]; then
  echo "Evidence must be a new absolute directory" >&2
  exit 2
fi
mkdir -p "$evidence"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
if [[ -n "${HOPE_VERIFICATION_SOURCE_SHA:-}" ]]; then
  echo "$HOPE_VERIFICATION_SOURCE_SHA" > "$evidence/code-sha.txt"
  echo 'exported source; provenance supplied by wrapper' > "$evidence/worktree-status.txt"
else
  git -C "$repo" rev-parse HEAD > "$evidence/code-sha.txt"
  git -C "$repo" status --porcelain > "$evidence/worktree-status.txt"
fi
# Continue independent gates after a failure; retain every exit status.
aggregate=0
run_gate() {
  local label=$1
  shift
  set +e
  "$@" > "$evidence/$label.log" 2>&1
  local status=$?
  set -e
  echo "$status" > "$evidence/$label.exit"
  if (( status != 0 )); then aggregate=1; fi
}
if [[ -e "$repo/.git" && -f "$repo/hope_ws/src/hope_planner_cpp/test/reacquisition_baseline_probe.py" ]]; then
  run_gate failing-before python3 "$repo/hope_ws/src/hope_planner_cpp/test/reacquisition_baseline_probe.py" \
    --repo "$repo" --output "$evidence/failing-before"
fi
run_gate native-configure cmake -S "$repo/hope_ws/src/hope_planner_cpp/test/native" -B "$evidence/native" -DCMAKE_BUILD_TYPE=Release
run_gate native-build cmake --build "$evidence/native" -j2
run_gate native-test ctest --test-dir "$evidence/native" --output-on-failure
run_gate review-cases "$evidence/native/test_v2_pr64_review_regressions" \
  --gtest_output="xml:$evidence/review-cases.xml"
if [[ -x "$evidence/native/test_reacquisition_spin" ]]; then
  run_gate reacquisition-cases "$evidence/native/test_reacquisition_spin" \
    --gtest_output="xml:$evidence/reacquisition-cases.xml"
  run_gate counterexample-cases "$evidence/native/test_reacquisition_counterexamples" \
    --gtest_output="xml:$evidence/counterexample-cases.xml"
  run_gate base-cases "$evidence/native/test_base_pose_contract" \
    --gtest_output="xml:$evidence/base-cases.xml"
  run_gate base-producer python3 "$repo/hope_ws/src/hope_planner_cpp/test/base_pose_producer_contract.py" \
    --probe "$evidence/native/hope_base_pose_contract_probe" --output "$evidence/base-producer.json"
  run_gate policy-contract env PYTHONPATH="${HOPE_POLICY_PYTHONPATH:-}" python3 \
    "$repo/hope_ws/src/hope_planner_cpp/test/review_policy_contract.py" --repo "$repo" \
    --output "$evidence/policy-contract.json"
fi
run_gate sanitized-configure cmake -S "$repo/hope_ws/src/hope_planner_cpp/test/native" -B "$evidence/sanitized" \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer -O1 -g1'
run_gate sanitized-build cmake --build "$evidence/sanitized" -j2
run_gate sanitized-test env ASAN_OPTIONS=detect_leaks=1 ctest --test-dir "$evidence/sanitized" --output-on-failure
run_gate outgoing-reference python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/integrated_numerical_gate.py" \
  --executable "$evidence/native/hope_integrated_return_sweep" --output "$evidence/outgoing-reference.json"
if [[ -e "$repo/.git" ]]; then
  run_gate legacy-parity python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/production_parity.py" \
    --repo "$repo" --output "$evidence/legacy-parity.json"
fi
cd "$repo/hope_ws"
run_gate ros-build env MAKEFLAGS=-j2 colcon --log-base "$evidence/log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$evidence/build" --install-base "$evidence/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup motion_capture_tracking_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release

if [[ -f "$evidence/install/setup.bash" ]]; then source "$evidence/install/setup.bash"; fi
run_gate ros-test colcon --log-base "$evidence/test-log" test --build-base "$evidence/build" \
  --install-base "$evidence/install" --packages-select hope_msgs hope_planner_cpp hope_bringup \
  --executor sequential --event-handlers console_direct+
run_gate ros-test-result colcon test-result --test-result-base "$evidence/build" --verbose
cat "$evidence/ros-test-result.log"
echo "$aggregate" > "$evidence/aggregate.exit"
exit "$aggregate"
