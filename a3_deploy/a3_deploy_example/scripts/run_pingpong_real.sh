#!/usr/bin/env bash
# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
#
# =============================================================================
#  DOCUMENTED TEMPLATE -- NOT RUNNABLE AS-IS. DO NOT EXECUTE ON A REAL ROBOT
#  WITHOUT YOUR OWN LICENSED AGIBOT VENDOR DEPLOY PACKAGE.
# =============================================================================
#
# This repository ships a CLEAN-ROOM reference runner and the Mulan-licensed
# MuJoCo sim only. It does NOT ship the proprietary AgiBot A3 C++ deploy runner,
# the vendor real-time backend, or the tuned control parameters. Real-robot
# execution is the responsibility of the operator using their OWN licensed vendor
# package, on hardware, with all vendor safety systems (motor protection, hard
# limits, e-stop) active.
#
# The public contract this reference implements (111-D observation, 31-D action,
# RacketCommand, 50 Hz, joint order in reference/.../joint_order.py) is exactly
# what a vendor backend must satisfy. The steps below are the INTEGRATION OUTLINE;
# every line that would touch hardware is commented out on purpose.
#
# See README.md ("Running on real hardware") and docs/RUN_ON_AGIBOT.md.

set -euo pipefail

echo "run_pingpong_real.sh is a documented template, not an executable script." >&2
echo "It requires your own licensed AgiBot vendor deploy package and hardware." >&2
echo "Open the file and follow the commented integration outline." >&2
exit 1

# --------------------------------------------------------------------------- #
# INTEGRATION OUTLINE (uncomment and adapt inside YOUR vendor deploy package)  #
# --------------------------------------------------------------------------- #
#
# 0) Provide your licensed vendor package and point this script at it:
#    VENDOR_DEPLOY_ROOT="/opt/agibot/a3_deploy"      # YOUR package, not shipped here
#    HOPE_ONNX="/path/to/hope_pingpong.onnx"          # your exported policy
#    ACTION_ADAPTER="../config/action_adapter.yaml"   # the shared adapter config
#
# 1) Bring up the vendor real-time backend and its safety systems per the vendor
#    documentation (motor enable, IMU, joint state, e-stop). This repo provides
#    none of that.
#
# 2) Wire the public contract into the vendor backend:
#      - build the 111-D observation each 50 Hz tick (see observation.py);
#      - run hope_pingpong.onnx: obs[1,111] -> raw_action[1,31];
#      - feed raw_action back as next-tick last_action;
#      - map raw_action -> 31 joint targets with the shared ActionAdapter
#        (action_adapter.yaml), holding head_yaw/head_pitch passive;
#      - consume RacketCommand from your planner (task_id / task_revision /
#        swing_side / position / velocity / time_to_strike).
#    The reference runner's modules document each step executably.
#
# 3) Command the vendor backend with the 31 joint targets at 50 Hz. Keep the
#    vendor's PD gains / limits / e-stop authoritative -- the public code does not
#    set, probe, or bypass them.
#
#    e.g. (pseudocode, inside the vendor package):
#    "${VENDOR_DEPLOY_ROOT}/bin/a3_deploy" \
#        --policy "${HOPE_ONNX}" \
#        --action-adapter "${ACTION_ADAPTER}" \
#        --obs-contract hope_pingpong \
#        --rate-hz 50
