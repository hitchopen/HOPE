#!/usr/bin/env bash
# Offline native tests only: no HAL, ROS publishers or hardware commands.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
native_dir="${script_dir}/../src/a3/a3_deploy_onnx_ref"
test_dir="$(mktemp -d)"
trap 'rm -f "$test_dir/test_serve"; rmdir "$test_dir"' EXIT
sources=()
for test in pp_serve_controller pp_command_transition pp_runner_control pp_serve_segments; do
  sources+=("${native_dir}/unit_tests/test_${test}.cpp")
done
for source in pp_serve_controller pp_serve025_fullbody_timeline pp_gripper_worker; do
  sources+=("${native_dir}/src/a3_deploy/${source}.cpp")
done
"${CXX:-g++}" -std=c++20 -O2 -pthread \
  -I"${native_dir}/include" -I/usr/include/eigen3 \
  "${sources[@]}" -lgtest -lgtest_main -lyaml-cpp -o "${test_dir}/test_serve"
"${test_dir}/test_serve"
