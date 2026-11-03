#!/usr/bin/env bash
# Production PpPolicy + real ONNX, entirely in memory; no HAL or DDS.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../../../.." && pwd)
output=${1:?Usage: verify_runner_revocation.sh NEW_OUTPUT_DIRECTORY [BASELINE_REPO]}
baseline=${2:-}
ort=${ORT_ROOT:?Set ORT_ROOT to the ONNX Runtime C++ distribution}
if [[ -e "$output" ]]; then echo 'Output must be a new directory' >&2; exit 2; fi
mkdir -p "$output"
runner=src/a3/a3_deploy_onnx_ref
deploy=a3_deploy/a3_deploy_example
policy="$repo/hope_training/whole_body_tracking/checkpoints"
probe="$repo/hope_ws/src/hope_planner_cpp/test/runner_revocation_probe.cpp"
sha256sum "$probe" "$policy/policy.onnx" "$policy/deploy.yaml" \
  "$ort/lib/libonnxruntime.so" > "$output/input-sha256.txt"
build() {
  local source=$1 label=$2
  shift 2
  "${CXX:-g++}" -std=c++20 -O1 -pthread "$@" -I/usr/include/eigen3 \
    -I"$ort/include" -I"$source/$deploy/$runner/include" "$probe" \
    "$source/$deploy/$runner/src/a3_deploy/pp_gripper_worker.cpp" \
    "$ort/lib/libonnxruntime.so" -Wl,-rpath,"$ort/lib" -lyaml-cpp \
    -o "$output/$label" > "$output/$label-build.log" 2>&1
}
build "$repo" candidate
if [[ -n "$baseline" ]]; then build "$baseline" baseline -DHOPE_RUNNER_BASELINE; fi
for mode in disabled enabled baseline; do
  if [[ "$mode" == baseline && -z "$baseline" ]]; then continue; fi
  binary=candidate
  if [[ "$mode" == baseline ]]; then binary=baseline; fi
  for schema in 4 5; do
    for lifecycle in 1 2 3 4; do
      name="$mode-schema-$schema-case-$lifecycle"
      set +e
      "$output/$binary" "$policy/policy.onnx" "$policy/deploy.yaml" \
        "$lifecycle" "$schema" "$mode" > "$output/$name.json" 2> "$output/$name.log"
      status=$?
      set -e
      echo "$status" > "$output/$name.exit"
    done
  done
done
python3 - "$output" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
rows = []
for path in sorted(root.glob('*-schema-*-case-*.json')):
    row = json.loads(path.read_text())
    enabled = path.name.startswith('enabled-')
    pending_case = row['case'] in (2, 4)
    expected_engagement = row['case'] != 1 and not (enabled and pending_case)
    assert row['engaged'] == expected_engagement, (path, row)
    assert row['active_after_revocation'] == (row['case'] == 3), (path, row)
    if pending_case:
        assert row['policy_observed_before'], (path, row)
    expected_exit = 1 if pending_case and not enabled else 0
    assert int(path.with_suffix('.exit').read_text()) == expected_exit, path
    rows.append(dict(file=path.name, result=row))
assert len(rows) in (16, 24), len(rows)
result = dict(status='PASS', cases=rows, after_engagement_cancellation=False,
              scope='production policy software lifecycle; no HAL or physical actuation')
(root / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result))
PY
