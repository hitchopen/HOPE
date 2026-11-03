#!/usr/bin/env bash
# Run inside distrobox enter hope; all ROS tests use shadow outputs.
set -eo pipefail
default_repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
repo=${HOPE_VERIFICATION_SOURCE_ROOT:-$default_repo}
evidence=${1:?Usage: verify_integrated.sh /new/absolute/evidence-directory}
build_jobs=${HOPE_VERIFICATION_BUILD_JOBS:-2}
if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'HOPE_VERIFICATION_BUILD_JOBS must be a positive integer' >&2
  exit 2
fi
if [[ "$evidence" != /* || -e "$evidence" ]]; then
  echo "Evidence must be a new absolute directory" >&2
  exit 2
fi
mkdir -p "$evidence"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ -n "${HOPE_VERIFICATION_SOURCE_SHA:-}" ]]; then
  echo "$HOPE_VERIFICATION_SOURCE_SHA" > "$evidence/code-sha.txt"
  echo 'exported source; provenance supplied by wrapper' > "$evidence/worktree-status.txt"
else
  git -C "$repo" rev-parse HEAD > "$evidence/code-sha.txt"
  git -C "$repo" status --porcelain > "$evidence/worktree-status.txt"
  if [[ -s "$evidence/worktree-status.txt" ]]; then
    echo 'Final verification requires a clean committed worktree' >&2
    exit 2
  fi
fi
# Continue independent gates after a failure; retain every exit status.
aggregate=0
run_gate() {
  local label=$1
  shift
  printf '%q ' "$@" > "$evidence/$label.command"
  printf '\n' >> "$evidence/$label.command"
  echo "START $label"
  set +e
  "$@" > "$evidence/$label.log" 2>&1
  local status=$?
  set -e
  echo "$status" > "$evidence/$label.exit"
  echo "RESULT $label exit=$status"
  if (( status != 0 )); then aggregate=1; fi
}
run_after() {
  local label=$1 prerequisites=$2 prerequisite
  shift 2
  local -a dependencies
  IFS=, read -r -a dependencies <<< "$prerequisites"
  for prerequisite in "${dependencies[@]}"; do
    if [[ ! -f "$evidence/$prerequisite.exit" ]] || [[ $(cat "$evidence/$prerequisite.exit") != 0 ]]; then
      echo "NOT_RUN: prerequisite $prerequisite failed or is missing" > "$evidence/$label.log"
      echo 77 > "$evidence/$label.exit"
      aggregate=1
      return
    fi
  done
  run_gate "$label" "$@"
}
if [[ -e "$repo/.git" && -f "$repo/hope_ws/src/hope_planner_cpp/test/reacquisition_baseline_probe.py" ]]; then
  run_gate failing-before python3 "$repo/hope_ws/src/hope_planner_cpp/test/reacquisition_baseline_probe.py" \
    --repo "$repo" --output "$evidence/failing-before"
fi
run_gate native-configure cmake -S "$repo/hope_ws/src/hope_planner_cpp/test/native" -B "$evidence/native" -DCMAKE_BUILD_TYPE=Release
run_after native-build native-configure cmake --build "$evidence/native" -j"$build_jobs"
run_after native-test native-build ctest --test-dir "$evidence/native" --output-on-failure
run_after review-cases native-build "$evidence/native/test_v2_pr64_review_regressions" \
  --gtest_output="xml:$evidence/review-cases.xml"
if [[ -f "$repo/hope_ws/src/hope_planner_cpp/test/test_reacquisition_spin.cpp" ]]; then
  run_after reacquisition-cases native-build "$evidence/native/test_reacquisition_spin" \
    --gtest_output="xml:$evidence/reacquisition-cases.xml"
  run_after counterexample-cases native-build "$evidence/native/test_reacquisition_counterexamples" \
    --gtest_output="xml:$evidence/counterexample-cases.xml"
  run_after base-cases native-build "$evidence/native/test_base_pose_contract" \
    --gtest_output="xml:$evidence/base-cases.xml"
  run_after base-producer native-build python3 "$repo/hope_ws/src/hope_planner_cpp/test/base_pose_producer_contract.py" \
    --probe "$evidence/native/hope_base_pose_contract_probe" --output "$evidence/base-producer.json"
  run_gate policy-contract env PYTHONPATH="${HOPE_POLICY_PYTHONPATH:-}" python3 \
    "$repo/hope_ws/src/hope_planner_cpp/test/review_policy_contract.py" --repo "$repo" \
    --output "$evidence/policy-contract.json"
fi
run_gate sanitized-configure cmake -S "$repo/hope_ws/src/hope_planner_cpp/test/native" -B "$evidence/sanitized" \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG=-g1 \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer -O1'
run_after sanitized-build sanitized-configure cmake --build "$evidence/sanitized" -j"$build_jobs"
run_after sanitized-test sanitized-build env ASAN_OPTIONS=detect_leaks=1 ctest --test-dir "$evidence/sanitized" --output-on-failure
run_after outgoing-reference native-build python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/integrated_numerical_gate.py" \
  --executable "$evidence/native/hope_integrated_return_sweep" --output "$evidence/outgoing-reference.json"
if [[ -e "$repo/.git" ]]; then
  run_gate legacy-parity python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/production_parity.py" \
    --repo "$repo" --output "$evidence/legacy-parity.json"
  run_gate historical-legacy-parity python3 "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/production_parity.py" \
    --repo "$repo" --base 8d599e7d21e196275cf426488555e2648e8fd789 \
    --candidate-ref 47296db5653c6dda7d397d7a82153f69fa4ed4fb \
    --output "$evidence/historical-legacy-parity.json"
  run_after premerge-v2-parity native-build python3 "$repo/hope_ws/src/hope_planner_cpp/test/pr85_cutoff_parity.py" \
    --repo "$repo" --replay "$evidence/native/hope_flight_prediction_v2_replay" \
    --output "$evidence/premerge-v2-parity.json"
fi
cd "$repo/hope_ws"
run_gate ros-build env MAKEFLAGS=-j"$build_jobs" colcon --log-base "$evidence/log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$evidence/build" --install-base "$evidence/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup motion_capture_tracking_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release

if [[ $(cat "$evidence/ros-build.exit") == 0 ]]; then source "$evidence/install/setup.bash"; fi
run_after incoming-conversion native-build,ros-build python3 \
  "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/incoming_conversion_simulation.py" \
  --probe "$evidence/native/hope_incoming_conversion_probe" --output "$evidence/incoming-conversion"
run_after mujoco-conversion incoming-conversion env \
  PYTHONPATH="${HOPE_SIMULATION_PYTHONPATH:-}${PYTHONPATH:+:$PYTHONPATH}" python3 \
  "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/mujoco_conversion_replay.py" \
  --repo "$repo" --conversion "$evidence/incoming-conversion" --output "$evidence/mujoco-conversion"
run_after ros-test ros-build colcon --log-base "$evidence/test-log" test --build-base "$evidence/build" \
  --install-base "$evidence/install" --packages-select hope_msgs hope_planner_cpp hope_bringup \
  --executor sequential --event-handlers console_direct+
run_after ros-test-result ros-test colcon test-result --test-result-base "$evidence/build" --verbose
run_after ros-result-audit ros-test python3 \
  "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/qualification_result_audit.py" \
  --build "$evidence/build" --baseline-build "${HOPE_SELECTED_UPSTREAM_BUILD:-$evidence/missing-selected-upstream-baseline}" \
  --output "$evidence/ros-result-audit.json"
run_after diagnostic-comparison ros-build python3 \
  "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/qualification_campaign.py" \
  --repo "$repo" --build "$evidence/build/hope_planner_cpp" --output "$evidence/diagnostic-comparison" \
  --diagnostic-comparison
run_after repeated-campaign ros-build python3 \
  "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/qualification_campaign.py" \
  --repo "$repo" --build "$evidence/build/hope_planner_cpp" --output "$evidence/repeated-campaign"
if [[ -f "$repo/hope_ws/src/hope_planner_cpp/test/v2_incoming_producer_chain.py" ]]; then
  run_after incoming-campaign ros-build python3 \
    "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/v2_incoming_campaign.py" \
    --repo "$repo" --build "$evidence/build/hope_planner_cpp" --output "$evidence/incoming-campaign"
  run_gate runner-revocation bash \
    "$repo/hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify_runner_revocation.sh" \
    "$evidence/runner-revocation" "${HOPE_RUNNER_BASELINE_REPO:-}"
fi
if [[ -e "$repo/.git" ]]; then
  git -C "$repo" rev-parse HEAD > "$evidence/final-code-sha.txt"
  git -C "$repo" status --porcelain > "$evidence/final-worktree-status.txt"
  run_gate source-commit-unchanged cmp "$evidence/code-sha.txt" "$evidence/final-code-sha.txt"
  run_gate source-worktree-clean test ! -s "$evidence/final-worktree-status.txt"
fi
echo "$aggregate" > "$evidence/aggregate.exit"
exit "$aggregate"
