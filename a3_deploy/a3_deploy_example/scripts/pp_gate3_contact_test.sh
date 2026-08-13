#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  pp_gate3_contact_test.sh [--sim-install PATH] [--output PATH]

Compile and run the isolated MuJoCo racket-contact A/B test used by the
model_21800 Gate3 qualification.  The command does not start ROS, AimRT,
Planner, Runner, or the full simulator.

Defaults:
  --sim-install  a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/
                 cmake-build-model21800-gate3/install
  --output       /tmp/model21800_gate3_contact_ab.json
USAGE
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
SIM_ROOT="${REPO_ROOT}/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim"
SIM_INSTALL="${PP_SIM_INSTALL:-${SIM_ROOT}/cmake-build-model21800-gate3/install}"
OUTPUT="${PP_GATE3_CONTACT_OUTPUT:-/tmp/model21800_gate3_contact_ab.json}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim-install)
      SIM_INSTALL="${2:-}"
      shift 2
      ;;
    --output)
      OUTPUT="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 64
      ;;
  esac
done

SOURCE="${SIM_ROOT}/scripts/pp_scripted_racket_contact_ab.cc"
CONTACT_HEADER="${SIM_ROOT}/src/module/mujoco_sim_module/common/gate3_ball_contact_model.h"
MUJOCO_HEADER="${SIM_INSTALL}/include/mujoco/mujoco.h"
MUJOCO_LIBRARY="${SIM_INSTALL}/lib/libmujoco.so"
for required in "${SOURCE}" "${CONTACT_HEADER}" "${MUJOCO_HEADER}" "${MUJOCO_LIBRARY}"; do
  if [[ ! -e "${required}" ]]; then
    echo "missing Gate3 contact-test dependency: ${required}" >&2
    exit 66
  fi
done

compiler="${CXX:-c++}"
if ! command -v "${compiler}" >/dev/null 2>&1; then
  echo "C++ compiler not found: ${compiler}" >&2
  exit 69
fi

binary="$(mktemp "${TMPDIR:-/tmp}/model21800-gate3-contact.XXXXXX")"
report_tmp="$(mktemp "${TMPDIR:-/tmp}/model21800-gate3-contact-report.XXXXXX")"
cleanup() {
  rm -f -- "${binary}" "${report_tmp}"
}
trap cleanup EXIT

"${compiler}" \
  -std=c++20 \
  -O2 \
  -Wall \
  -Wextra \
  -I"${SIM_ROOT}/src/module/mujoco_sim_module" \
  -I"${SIM_INSTALL}/include" \
  "${SOURCE}" \
  -L"${SIM_INSTALL}/lib" \
  -Wl,-rpath,"${SIM_INSTALL}/lib" \
  -lmujoco \
  -pthread \
  -ldl \
  -o "${binary}"

LD_LIBRARY_PATH="${SIM_INSTALL}/lib:${LD_LIBRARY_PATH:-}" \
  "${binary}" > "${report_tmp}"

python3 - "${report_tmp}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
report = json.loads(path.read_text(encoding="utf-8"))
if report.get("schema") != "pp-scripted-racket-contact-ab-v1":
    raise SystemExit("unexpected scripted contact report schema")
if report.get("explicit_contact_energy_pass") is not True:
    raise SystemExit("explicit racket-contact model did not pass")
explicit = [row for row in report.get("results", []) if row.get("mode") == "explicit_map"]
if not explicit:
    raise SystemExit("scripted contact report has no explicit-map cases")
for row in explicit:
    if row.get("contact_rising_edges") != 1:
        raise SystemExit(f"invalid contact edge count: {row}")
    if row.get("finite") is not True or row.get("map_match") is not True:
        raise SystemExit(f"invalid explicit-map result: {row}")
print(
    "model_21800 Gate3 contact A/B: PASS "
    f"({len(explicit)} explicit cases, "
    f"max_map_error={report['explicit_max_map_error_mps']:.3g} m/s)"
)
PY

install -D -m 0644 "${report_tmp}" "${OUTPUT}"
echo "report: ${OUTPUT}"
