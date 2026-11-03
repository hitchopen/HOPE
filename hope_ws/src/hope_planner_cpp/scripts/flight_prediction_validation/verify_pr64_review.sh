#!/usr/bin/env bash
# Full baseline/corrected reproduction, including expected failing-before RVs.
# Run with a ROS Jazzy + Eigen/GTest/SciPy toolchain and a new evidence path.
set -eo pipefail
repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
evidence=${1:?Usage: verify_pr64_review.sh /new/absolute/evidence-directory}
if [[ "$evidence" != /* || -e "$evidence" ]]; then exit 2; fi
mkdir -p "$evidence/pinned-source"
base=3346e03002b47611e3531d7147392faec9e9268f
git -C "$repo" archive "$base" | tar -x -C "$evidence/pinned-source"
planner=hope_ws/src/hope_planner_cpp
cp "$repo/$planner/test/test_v2_pr64_review_regressions.cpp" "$evidence/pinned-source/$planner/test/"
cat >> "$evidence/pinned-source/$planner/test/native/CMakeLists.txt" <<'CMAKE'
add_executable(test_v2_pr64_review_regressions ${PLANNER}/test/test_v2_pr64_review_regressions.cpp)
target_link_libraries(test_v2_pr64_review_regressions PRIVATE hope_integrated_native GTest::gtest_main)
target_compile_definitions(test_v2_pr64_review_regressions PRIVATE HOPE_PR64_BASELINE=1)
add_test(NAME v2_pr64_review_regressions COMMAND test_v2_pr64_review_regressions)
CMAKE
entry="$repo/$planner/scripts/flight_prediction_validation/verify_integrated.sh"
set +e
HOPE_VERIFICATION_SOURCE_ROOT="$evidence/pinned-source" HOPE_VERIFICATION_SOURCE_SHA="$base" \
  bash "$entry" "$evidence/baseline" > "$evidence/baseline-console.log" 2>&1
baseline_status=$?
bash "$entry" "$evidence/corrected" > "$evidence/corrected-console.log" 2>&1
corrected_status=$?
set -e
printf '%s\n' "$baseline_status" > "$evidence/baseline.exit"
printf '%s\n' "$corrected_status" > "$evidence/corrected.exit"
# Expected baseline failures remain visible. Corrected aggregate preserves
# inherited bringup lint as a nonzero exit, never a hidden exception.
exit "$corrected_status"
