# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""Shared ActionAdapter: raw policy output -> joint-position targets.

The adapter is a pure, deterministic numeric transform (a joint-position residual
plus a clamp). It is NOT a rejection filter and emits no failure status:

    q_des = default_q + raw_action * action_scale
    q_des = clip(q_des, clamp_lower, clamp_upper)

The SAME configuration file drives training and this reference runner, so a user
tunes the mapping in exactly one place. The shipped constants are neutral EXAMPLE
values (see ``config/action_adapter.yaml``) and must be tuned for a real robot.

Vendor hard limits, motor protection, and e-stop remain the responsibility of the
robot backend; this transform neither probes nor bypasses them.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .joint_order import JOINT_NAMES, NUM_JOINTS


@dataclass
class ActionAdapter:
    default_q: np.ndarray     # (31,) neutral/upright joint positions, rad
    action_scale: np.ndarray  # (31,) per-column residual scale (uniform in the example)
    clamp_lower: np.ndarray   # (31,) lower joint-position clamp, rad
    clamp_upper: np.ndarray   # (31,) upper joint-position clamp, rad

    def __post_init__(self) -> None:
        for field in ("default_q", "action_scale", "clamp_lower", "clamp_upper"):
            v = np.asarray(getattr(self, field), dtype=np.float64).reshape(-1)
            if v.shape[0] != NUM_JOINTS:
                raise ValueError(f"{field} must be length {NUM_JOINTS}, got {v.shape[0]}")
            setattr(self, field, v)
        if np.any(self.clamp_lower > self.clamp_upper):
            raise ValueError("action_adapter clamp_lower must be <= clamp_upper for every joint")

    def decode(self, raw_action: np.ndarray) -> np.ndarray:
        """Map ``raw_action[31]`` to 31 clamped joint-position targets."""
        raw = np.asarray(raw_action, dtype=np.float64).reshape(-1)
        if raw.shape[0] != NUM_JOINTS:
            raise ValueError(f"raw_action must be length {NUM_JOINTS}, got {raw.shape[0]}")
        q_des = self.default_q + raw * self.action_scale
        return np.clip(q_des, self.clamp_lower, self.clamp_upper)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ActionAdapter":
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)

        default_q = _resolve_per_joint(doc["default_q"], "default_q")

        scale_spec = doc["action_scale"]
        if isinstance(scale_spec, (int, float)):
            action_scale = np.full(NUM_JOINTS, float(scale_spec), dtype=np.float64)
        else:
            action_scale = _resolve_per_joint(scale_spec, "action_scale")

        clamp = doc["joint_position_clamp"]
        clamp_lower = _resolve_per_joint(clamp["lower"], "joint_position_clamp.lower")
        clamp_upper = _resolve_per_joint(clamp["upper"], "joint_position_clamp.upper")
        return cls(default_q, action_scale, clamp_lower, clamp_upper)


def _resolve_per_joint(spec, field_name: str) -> np.ndarray:
    """Accept either an ordered length-31 list or a ``{joint_name: value}`` map."""
    if isinstance(spec, dict):
        missing = [n for n in JOINT_NAMES if n not in spec]
        if missing:
            raise ValueError(f"{field_name} is missing joints: {missing[:3]}...")
        return np.array([float(spec[n]) for n in JOINT_NAMES], dtype=np.float64)
    arr = np.asarray(spec, dtype=np.float64).reshape(-1)
    if arr.shape[0] != NUM_JOINTS:
        raise ValueError(f"{field_name} must be length {NUM_JOINTS}, got {arr.shape[0]}")
    return arr
