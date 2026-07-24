#!/usr/bin/env bash
# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
#
# Launch the clean-room G1 reference runner against the in-process MuJoCo sim.
#
# NOTE: a G1 ping-pong MJCF is not shipped yet (the MuJoCo real-ball path is the deferred
# follow-up). Point --model-xml at your own G1 MJCF, or set simulation.model_xml_path in the
# runtime YAML. Requires:
#     pip install numpy pyyaml onnxruntime mujoco
# and an exported policy at config/../models/hope_pingpong_g1.onnx (or pass --onnx).
#
# Examples:
#   ./run_pingpong_sim.sh --view --realtime --model-xml /path/g1_pingpong.xml
#   ./run_pingpong_sim.sh --duration 20 --model-xml /path/g1_pingpong.xml
#   ./run_pingpong_sim.sh --onnx /path/hope_pingpong_g1.onnx --idle --model-xml /path/g1_pingpong.xml
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd "${HERE}/.." && pwd)"
REF_DIR="${EXAMPLE_DIR}/reference"

export PYTHONPATH="${REF_DIR}:${PYTHONPATH:-}"

exec python3 -m g1_deploy_onnx_ref_pingpong \
  --config "${EXAMPLE_DIR}/config/hope_pingpong_runtime.yaml" \
  --backend mujoco \
  "$@"
