"""Unitree G1 contract parity + consistency (Isaac-free).

The G1 target is added in parallel to the A3. This test pins the G1 invariants that make training,
ONNX export, and the deploy runner agree, WITHOUT Isaac / torch:

  * the canonical G1 joint order is 29 unique names, identical across
    ``joint_order_unitree_g1.yaml``, the deploy ``joint_order.JOINT_NAMES``, and the shared
    ``action_adapter.yaml`` keys;
  * the training-side ``load_g1_action_adapter_config`` and the deploy ``ActionAdapter`` read the
    SAME shared YAML and decode identically (residual + clamp);
  * the deploy observation dim is 105 = 18 + 3*29, the action dim 29, and there are no passive
    (head) columns;
  * the exporter's DOF-derived helpers and the actor observation contract agree at 105-D.

Run:  python tests/test_g1_contract.py   (or pytest)
"""

from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np
import yaml

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_SRC = os.path.join(_ROOT, "source", "whole_body_tracking", "whole_body_tracking")
_UTILS = os.path.join(_SRC, "utils")
_REFERENCE_DIR = os.path.join(_REPO, "g1_deploy", "g1_deploy_example", "reference")
_G1_YAML = os.path.join(_REPO, "g1_deploy", "g1_deploy_example", "config", "action_adapter.yaml")
_G1_JOINT_ORDER_YAML = os.path.join(_REPO, "hope_training", "config", "joint_order_unitree_g1.yaml")


def _load_by_path(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# Training-side loaders (pure modules, loaded by file path so Isaac is never imported).
train_side = _load_by_path("hope_action_adapter_config_g1", os.path.join(_UTILS, "action_adapter_config.py"))
exporter = _load_by_path("hope_exporter_g1", os.path.join(_UTILS, "exporter.py"))
contract_mod = _load_by_path(
    "hope_actor_obs_contract_g1",
    os.path.join(_SRC, "tasks", "tracking", "actor_observation_contract.py"),
)

# Deploy-side G1 reference package (import-light: numpy + yaml; onnx/mujoco/rclpy are lazy).
sys.path.insert(0, _REFERENCE_DIR)
from g1_deploy_onnx_ref_pingpong.action_adapter import ActionAdapter  # noqa: E402
from g1_deploy_onnx_ref_pingpong.joint_order import (  # noqa: E402
    HEAD_INDICES,
    JOINT_NAMES,
    NUM_JOINTS,
)
from g1_deploy_onnx_ref_pingpong.observation import OBS_DIM  # noqa: E402


def _both():
    deploy = ActionAdapter.from_yaml(_G1_YAML)
    training = train_side.load_g1_action_adapter_config()
    return deploy, training


def test_g1_joint_order_29_unique_and_consistent():
    assert NUM_JOINTS == 29
    assert len(JOINT_NAMES) == 29 and len(set(JOINT_NAMES)) == 29
    with open(_G1_JOINT_ORDER_YAML) as fh:
        yorder = tuple(yaml.safe_load(fh)["joint_order"])
    assert yorder == tuple(JOINT_NAMES), "joint_order_unitree_g1.yaml != deploy JOINT_NAMES"
    with open(_G1_YAML) as fh:
        doc = yaml.safe_load(fh)
    assert set(doc["default_q"]) == set(JOINT_NAMES)
    assert set(doc["joint_position_clamp"]["lower"]) == set(JOINT_NAMES)
    assert set(doc["joint_position_clamp"]["upper"]) == set(JOINT_NAMES)


def test_g1_has_no_passive_head():
    assert HEAD_INDICES == ()


def test_g1_shared_config_and_joint_order_match():
    training = train_side.load_g1_action_adapter_config()
    assert tuple(training.joint_names) == tuple(JOINT_NAMES)
    # The DOF-based selector resolves the G1 file for 29 DOF.
    order, src = train_side.load_joint_order_for_dof(29)
    assert tuple(order) == tuple(JOINT_NAMES) and "unitree_g1" in src


def test_g1_constants_identical():
    deploy, training = _both()
    np.testing.assert_array_equal(training.default_q, deploy.default_q)
    np.testing.assert_array_equal(training.action_scale, deploy.action_scale)
    np.testing.assert_array_equal(training.clamp_lower, deploy.clamp_lower)
    np.testing.assert_array_equal(training.clamp_upper, deploy.clamp_upper)


def test_g1_decode_parity():
    deploy, training = _both()
    rng = np.random.default_rng(0)
    cases = [
        np.zeros(NUM_JOINTS),
        np.ones(NUM_JOINTS),
        -np.ones(NUM_JOINTS),
        rng.uniform(-3.0, 3.0, NUM_JOINTS),
        np.full(NUM_JOINTS, 50.0),   # saturates the upper clamp
        np.full(NUM_JOINTS, -50.0),  # saturates the lower clamp
    ]
    for raw in cases:
        np.testing.assert_allclose(training.decode(raw), deploy.decode(raw), atol=1e-12)


def test_g1_observation_and_action_dims():
    assert OBS_DIM == 105 == 18 + 3 * NUM_JOINTS
    assert exporter.obs_dim_for(29) == 105
    assert exporter.contract_name_for(29) == "hope_pingpong_g1"
    layout = exporter.observation_layout_for(29)
    assert layout[-1]["slice"][1] == 105  # contiguous, ends at 105
    assert "unitree_g1" in exporter.joint_order_source_for(29)


def test_g1_actor_observation_contract_105():
    c = contract_mod.resolve_actor_observation_contract("hope_pingpong_g1")
    assert c.total_dim == 105
    dims = dict(c.layout)
    assert dims["joint_pos"] == 29 and dims["joint_vel"] == 29 and dims["last_action"] == 29


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK {name}")
    print("ALL G1 CONTRACT TESTS PASSED")
