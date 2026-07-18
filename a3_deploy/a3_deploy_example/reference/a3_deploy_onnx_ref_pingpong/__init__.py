# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""Clean-room reference runner for the HOPE 111-D deploy contract.

This package is authored from scratch against the public observation / action /
RacketCommand contract. It is NOT the vendor deploy runner and contains none of
the vendor's source, tuned constants, or gates. It exists to document the contract
executably and to run the exported policy against the shipped MuJoCo sim.

Public modules:
  joint_order      -- the 31-DOF Agibot A3 joint order
  observation      -- the 111-D observation builder
  action_adapter   -- the shared ActionAdapter (raw_action -> joint targets)
  racket_command   -- RacketCommand + command sources
  ros_command_source -- ROS 2 planner bridge (hope_msgs/RacketCommand -> source)
  lifecycle        -- ready -> swing -> follow-through -> recovery state machine
  onnx_policy      -- onnxruntime actor wrapper (obs[1,111] -> raw_action[1,31])
  sim_bridge       -- MuJoCo (default) and AimRT-process (seam) bridges
  config           -- runtime config loader
  runner           -- the 50 Hz control loop
"""

from __future__ import annotations

from .action_adapter import ActionAdapter
from .config import RuntimeConfig
from .joint_order import JOINT_NAMES, NUM_JOINTS
from .lifecycle import LifecycleConfig, Phase, SwingLifecycle
from .observation import OBS_DIM, ObsTarget, RobotState, build_observation
from .racket_command import (
    BACKHAND,
    FOREHAND,
    ExampleCommandFeed,
    QueueRacketCommandSource,
    RacketCommand,
    RacketCommandSource,
)
from .ros_command_source import RosRacketCommandSource, racket_command_from_msg

__all__ = [
    "ActionAdapter",
    "RuntimeConfig",
    "JOINT_NAMES",
    "NUM_JOINTS",
    "LifecycleConfig",
    "Phase",
    "SwingLifecycle",
    "OBS_DIM",
    "ObsTarget",
    "RobotState",
    "build_observation",
    "BACKHAND",
    "FOREHAND",
    "ExampleCommandFeed",
    "QueueRacketCommandSource",
    "RacketCommand",
    "RacketCommandSource",
    "RosRacketCommandSource",
    "racket_command_from_msg",
]
