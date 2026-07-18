"""Training <-> deployment action-adapter parity (finding: unify the adapter contract).

Both sides must read the SAME shared configuration
(``a3_deploy/a3_deploy_example/config/action_adapter.yaml``) and map an identical
raw action to identical joint-position targets:

    q_des = default_q + raw_action * action_scale;  clip to the shared per-joint clamp

This test loads the deploy ``ActionAdapter`` and the training-side
``action_adapter_config`` loader from the one canonical YAML and asserts byte-identical
constants plus equivalent decode outputs for representative raw actions (zero, +/-1,
random, and clamp-saturating extremes).

Scope note: the decode-parity leg exercises the deploy adapter against the training
LOADER's reference ``decode`` (the same residual+clamp formula), not the live Isaac
action term — that term needs torch/Isaac and applies the identical constants via
``hope_env_cfg`` (position_clamp / scale / default_q wiring), which these tests pin at
the constants level. Pure NumPy + PyYAML — runs without Isaac / torch / onnxruntime.

Run:  python tests/test_action_adapter_parity.py   (or pytest)
"""

from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_UTILS = os.path.join(_ROOT, "source", "whole_body_tracking", "whole_body_tracking", "utils")
_REFERENCE_DIR = os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference")
_SHARED_YAML = os.path.join(
    _REPO, "a3_deploy", "a3_deploy_example", "config", "action_adapter.yaml"
)


def _load_by_path(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# Training-side loader (pure module, loaded by file path so the Isaac Lab package
# __init__ is never imported).
train_side = _load_by_path(
    "hope_action_adapter_config", os.path.join(_UTILS, "action_adapter_config.py")
)

# Deploy-side ActionAdapter (the reference runner package is import-light).
sys.path.insert(0, _REFERENCE_DIR)
from a3_deploy_onnx_ref_pingpong.action_adapter import ActionAdapter  # noqa: E402
from a3_deploy_onnx_ref_pingpong.joint_order import JOINT_NAMES, NUM_JOINTS  # noqa: E402


def _both():
    deploy = ActionAdapter.from_yaml(_SHARED_YAML)
    training = train_side.load_action_adapter_config(path=_SHARED_YAML)
    return deploy, training


def test_shared_config_exists():
    assert os.path.isfile(_SHARED_YAML), _SHARED_YAML
    # The training loader auto-discovers the same canonical file.
    assert os.path.samefile(train_side.find_shared_action_adapter_config(), _SHARED_YAML)


def test_joint_order_matches_deploy():
    training = train_side.load_action_adapter_config(path=_SHARED_YAML)
    assert tuple(training.joint_names) == tuple(JOINT_NAMES)


def test_constants_identical():
    deploy, training = _both()
    np.testing.assert_array_equal(training.default_q, deploy.default_q)
    np.testing.assert_array_equal(training.action_scale, deploy.action_scale)
    np.testing.assert_array_equal(training.clamp_lower, deploy.clamp_lower)
    np.testing.assert_array_equal(training.clamp_upper, deploy.clamp_upper)


def test_decode_parity_on_representative_actions():
    deploy, training = _both()
    rng = np.random.default_rng(0)
    cases = [
        np.zeros(NUM_JOINTS),
        np.ones(NUM_JOINTS),
        -np.ones(NUM_JOINTS),
        rng.uniform(-3.0, 3.0, NUM_JOINTS),
        rng.standard_normal(NUM_JOINTS) * 0.5,
        np.full(NUM_JOINTS, 50.0),    # saturates the upper clamp on every joint
        np.full(NUM_JOINTS, -50.0),   # saturates the lower clamp on every joint
    ]
    for raw in cases:
        np.testing.assert_allclose(training.decode(raw), deploy.decode(raw), rtol=0, atol=0)


def test_zero_action_returns_default_pose_within_clamp():
    deploy, training = _both()
    np.testing.assert_array_equal(deploy.decode(np.zeros(NUM_JOINTS)), deploy.default_q)
    assert np.all(training.default_q >= training.clamp_lower)
    assert np.all(training.default_q <= training.clamp_upper)


def test_cfg_dict_views_round_trip():
    """The dict views hope_env_cfg feeds into Isaac Lab reproduce the same arrays."""
    _deploy, training = _both()
    dq = training.default_q_by_name()
    sc = training.action_scale_by_name()
    cl = training.position_clamp_by_name()
    assert list(dq.keys()) == list(JOINT_NAMES)
    np.testing.assert_array_equal(
        np.array([dq[n] for n in JOINT_NAMES]), training.default_q
    )
    np.testing.assert_array_equal(
        np.array([sc[n] for n in JOINT_NAMES]), training.action_scale
    )
    lo = np.array([cl[n][0] for n in JOINT_NAMES])
    hi = np.array([cl[n][1] for n in JOINT_NAMES])
    np.testing.assert_array_equal(lo, training.clamp_lower)
    np.testing.assert_array_equal(hi, training.clamp_upper)


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"[ok] {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {fn.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} action-adapter parity tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
