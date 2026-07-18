"""Loader for the SHARED ActionAdapter configuration (training side).

The raw-action -> joint-target mapping is defined in exactly ONE file, read by both
the training package and the reference deploy runner:

    a3_deploy/a3_deploy_example/config/action_adapter.yaml

This module resolves that file (walking up from here to the repo root), parses it
against the canonical 31-joint order, and exposes the resulting ``default_q``,
``action_scale`` and joint-position clamp arrays so
:class:`~whole_body_tracking.tasks.tracking.config.agibot_a3.hope_env_cfg.HOPEPingPongEnvCfg`
can wire them into the articulation default pose and the action term. The deploy
runner applies the identical transform through its own ``ActionAdapter``:

    q_des = default_q + raw_action * action_scale
    q_des = clip(q_des, clamp_lower, clamp_upper)

Pure NumPy + PyYAML — importable without Isaac / torch (so the parity test in
``tests/test_action_adapter_parity.py`` can compare this loader against the deploy
``ActionAdapter`` directly).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml

# Repo-root-relative locations of the two shared contract files.
SHARED_ADAPTER_RELPATH = os.path.join(
    "a3_deploy", "a3_deploy_example", "config", "action_adapter.yaml"
)
JOINT_ORDER_RELPATH = os.path.join("hope_training", "config", "joint_order_agibot_a3.yaml")


def _find_upward(relpath: str, start: str | None = None) -> str:
    """Walk up from ``start`` (default: this file) until ``relpath`` exists."""
    d = os.path.dirname(os.path.abspath(start or __file__))
    for _ in range(16):
        cand = os.path.join(d, relpath)
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    raise FileNotFoundError(
        f"shared config not found walking up from {start or __file__}: {relpath}"
    )


def find_shared_action_adapter_config() -> str:
    """Absolute path of the canonical shared ``action_adapter.yaml``."""
    return _find_upward(SHARED_ADAPTER_RELPATH)


def load_joint_order(path: str | None = None) -> tuple[str, ...]:
    """The canonical 31-joint order from ``hope_training/config/joint_order_agibot_a3.yaml``."""
    path = path or _find_upward(JOINT_ORDER_RELPATH)
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    order = tuple(str(n) for n in doc["joint_order"])
    if len(order) != 31 or len(set(order)) != 31:
        raise ValueError(f"joint order must list 31 unique joints, got {len(order)}")
    return order


@dataclass(frozen=True)
class ActionAdapterConfig:
    """The shared adapter constants, resolved into canonical joint order."""

    joint_names: tuple[str, ...]
    default_q: np.ndarray     # (31,) rad
    action_scale: np.ndarray  # (31,) residual scale
    clamp_lower: np.ndarray   # (31,) rad
    clamp_upper: np.ndarray   # (31,) rad

    def decode(self, raw_action: np.ndarray) -> np.ndarray:
        """Reference transform (identical to the deploy ``ActionAdapter.decode``)."""
        raw = np.asarray(raw_action, dtype=np.float64).reshape(-1)
        if raw.shape[0] != len(self.joint_names):
            raise ValueError(f"raw_action must be length {len(self.joint_names)}")
        q_des = self.default_q + raw * self.action_scale
        return np.clip(q_des, self.clamp_lower, self.clamp_upper)

    # -- dict views used to build the Isaac Lab cfg (exact joint names, no regex) --
    def default_q_by_name(self) -> dict[str, float]:
        return {n: float(v) for n, v in zip(self.joint_names, self.default_q)}

    def action_scale_by_name(self) -> dict[str, float]:
        return {n: float(v) for n, v in zip(self.joint_names, self.action_scale)}

    def position_clamp_by_name(self) -> dict[str, tuple[float, float]]:
        return {
            n: (float(lo), float(hi))
            for n, lo, hi in zip(self.joint_names, self.clamp_lower, self.clamp_upper)
        }


def _resolve_per_joint(spec, joint_names: tuple[str, ...], field_name: str) -> np.ndarray:
    """Accept an ordered length-31 list or a ``{joint_name: value}`` map (same as deploy)."""
    n = len(joint_names)
    if isinstance(spec, dict):
        missing = [name for name in joint_names if name not in spec]
        if missing:
            raise ValueError(f"{field_name} is missing joints: {missing[:3]}...")
        return np.array([float(spec[name]) for name in joint_names], dtype=np.float64)
    arr = np.asarray(spec, dtype=np.float64).reshape(-1)
    if arr.shape[0] != n:
        raise ValueError(f"{field_name} must be length {n}, got {arr.shape[0]}")
    return arr


def load_action_adapter_config(
    path: str | None = None, joint_names: tuple[str, ...] | None = None
) -> ActionAdapterConfig:
    """Parse the shared adapter YAML into canonical-order arrays."""
    path = path or find_shared_action_adapter_config()
    joint_names = joint_names or load_joint_order()
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)

    default_q = _resolve_per_joint(doc["default_q"], joint_names, "default_q")

    scale_spec = doc["action_scale"]
    if isinstance(scale_spec, (int, float)):
        action_scale = np.full(len(joint_names), float(scale_spec), dtype=np.float64)
    else:
        action_scale = _resolve_per_joint(scale_spec, joint_names, "action_scale")

    clamp = doc["joint_position_clamp"]
    clamp_lower = _resolve_per_joint(clamp["lower"], joint_names, "joint_position_clamp.lower")
    clamp_upper = _resolve_per_joint(clamp["upper"], joint_names, "joint_position_clamp.upper")
    if np.any(clamp_lower > clamp_upper):
        raise ValueError("joint_position_clamp lower must be <= upper for every joint")

    return ActionAdapterConfig(
        joint_names=joint_names,
        default_q=default_q,
        action_scale=action_scale,
        clamp_lower=clamp_lower,
        clamp_upper=clamp_upper,
    )
