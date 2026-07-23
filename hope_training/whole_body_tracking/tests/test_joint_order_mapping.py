"""Pure tests for the canonical <-> Isaac articulation joint-order mapping."""

from __future__ import annotations

import importlib.util
import os
import sys

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UTILS = os.path.join(_ROOT, "source", "whole_body_tracking", "whole_body_tracking", "utils")


def _load_action_adapter_config():
    path = os.path.join(_UTILS, "action_adapter_config.py")
    spec = importlib.util.spec_from_file_location("hope_action_adapter_config_mapping", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


cfg = _load_action_adapter_config()


ISAAC_A3_JOINT_ORDER = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "head_yaw_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "head_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]


def test_identity_mapping():
    canonical = cfg.load_joint_order()
    mapping = cfg.resolve_joint_order_mapping(canonical, canonical_joint_names=canonical)
    assert mapping.is_identity
    assert mapping.canonical_to_articulation == tuple(range(31))
    assert mapping.articulation_to_canonical == tuple(range(31))


def test_known_isaac_a3_mapping_direction():
    canonical = cfg.load_joint_order()
    mapping = cfg.resolve_joint_order_mapping(ISAAC_A3_JOINT_ORDER, canonical_joint_names=canonical)
    assert not mapping.is_identity
    assert mapping.canonical_to_articulation == (
        2, 5, 8, 11, 16, 12, 17, 21, 23, 25, 27, 29, 13, 18, 22, 24,
        26, 28, 30, 0, 3, 6, 9, 14, 19, 1, 4, 7, 10, 15, 20,
    )
    reordered = [ISAAC_A3_JOINT_ORDER[i] for i in mapping.canonical_to_articulation]
    assert tuple(reordered) == canonical


def test_mapping_rejects_missing_or_extra_joint():
    canonical = cfg.load_joint_order()
    bad = list(ISAAC_A3_JOINT_ORDER)
    bad[-1] = "not_a_real_joint"
    with pytest.raises(ValueError, match="missing_from_articulation"):
        cfg.resolve_joint_order_mapping(bad, canonical_joint_names=canonical)


def test_mapping_rejects_duplicate_articulation_names():
    bad = list(ISAAC_A3_JOINT_ORDER)
    bad[-1] = bad[0]
    with pytest.raises(ValueError, match="unique"):
        cfg.resolve_joint_order_mapping(bad, canonical_joint_names=cfg.load_joint_order())
