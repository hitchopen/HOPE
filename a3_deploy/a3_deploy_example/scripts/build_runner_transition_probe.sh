#!/usr/bin/env bash
# Build only the local simulation ABI; never connects to HAL or DDS.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source_root="$root/src/a3/a3_deploy_onnx_ref"
ort_root="${ORT_ROOT:-$root/thirdparty/onnxruntime/onnxruntime-linux-x64-1.19.2}"
output="${1:-/tmp/hope_runner_transition_probe.so}"
mkdir -p "$(dirname -- "$output")"
"${CXX:-g++}" -std=c++20 -O2 -shared -fPIC -pthread \
  -I/usr/include/eigen3 -I"$ort_root/include" -I"$source_root/include" \
  "$root/scripts/runner_transition_probe.cpp" \
  "$source_root/src/a3_deploy/pp_humanlike_policy.cpp" \
  "$source_root/src/a3_deploy/pp_hybrid_lower_policy.cpp" \
  "$source_root/src/a3_deploy/pp_serve_controller.cpp" \
  "$source_root/src/a3_deploy/pp_serve025_fullbody_timeline.cpp" \
  "$source_root/src/a3_deploy/pp_gripper_worker.cpp" \
  "$ort_root/lib/libonnxruntime.so" -Wl,-rpath,"$ort_root/lib" \
  -lyaml-cpp -o "$output"
printf 'Built local simulation probe: %s\n' "$output"
