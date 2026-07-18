#!/usr/bin/env bash
# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
#
# Launch the clean-room reference runner against the in-process MuJoCo sim.
#
# This drives the SAME a3_pingpong MJCF that the AimRT MuJoCo sim wraps, stepping
# MuJoCo in-process -- no AimRT/iceoryx/ROS2 stack required. Requires:
#     pip install numpy pyyaml onnxruntime mujoco
# and an exported policy at config/../models/hope_pingpong.onnx (or pass --onnx).
#
# Examples:
#   ./run_pingpong_sim.sh --view --realtime            # windowed, wall-clock 50 Hz
#   ./run_pingpong_sim.sh --duration 20                # headless, 20 s
#   ./run_pingpong_sim.sh --onnx /path/hope_pingpong.onnx --idle
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd "${HERE}/.." && pwd)"
REF_DIR="${EXAMPLE_DIR}/reference"

export PYTHONPATH="${REF_DIR}:${PYTHONPATH:-}"

exec python3 -m a3_deploy_onnx_ref_pingpong \
  --config "${EXAMPLE_DIR}/config/hope_pingpong_runtime.yaml" \
  --backend mujoco \
  "$@"
