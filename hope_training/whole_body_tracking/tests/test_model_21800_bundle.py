"""Compatibility checks for the published build_1 ``model_21800`` bundle.

These are host-only NumPy/PyYAML tests.  Actual ONNX Runtime execution is covered
by the documented MuJoCo command; this file pins the two pieces that differ from
the compact public exporter: training-order joint columns and the reference clock.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import yaml

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_EXAMPLE = os.path.join(_REPO, "a3_deploy", "a3_deploy_example")
_REFERENCE = os.path.join(_EXAMPLE, "reference")
_RUNTIME = os.path.join(_EXAMPLE, "config", "hope_pingpong_runtime.yaml")
_DEPLOY = os.path.join(
    _EXAMPLE, "models", "model_21800", "policy", "params", "deploy.yaml"
)

sys.path.insert(0, _REFERENCE)

from a3_deploy_onnx_ref_pingpong.config import RuntimeConfig  # noqa: E402
from a3_deploy_onnx_ref_pingpong.joint_order import JOINT_NAMES, NUM_JOINTS  # noqa: E402
from a3_deploy_onnx_ref_pingpong.onnx_policy import OnnxPolicy  # noqa: E402


def test_published_runtime_resolves_complete_model_bundle():
    cfg = RuntimeConfig.load(_RUNTIME)
    assert cfg.onnx_path.is_file()
    assert cfg.onnx_path.name == "policy.onnx"
    assert cfg.onnx_path.parent.parent.parent.name == "model_21800"
    assert os.path.isfile(_DEPLOY)

    with open(_DEPLOY, "r", encoding="utf-8") as fh:
        deploy = yaml.safe_load(fh)
    sdk_names = tuple(deploy["joint_sdk_names"])
    by_name_kp = dict(zip(sdk_names, deploy["stiffness"]))
    by_name_kd = dict(zip(sdk_names, deploy["damping"]))
    np.testing.assert_array_equal(cfg.sim_kp, [by_name_kp[name] for name in JOINT_NAMES])
    np.testing.assert_array_equal(cfg.sim_kd, [by_name_kd[name] for name in JOINT_NAMES])


def test_build1_reference_clock_matches_export_metadata():
    policy = OnnxPolicy.__new__(OnnxPolicy)
    policy._time_step_name = "time_step"
    policy._clip_lengths = (107, 109)
    policy._strike_phases = (0.3868, 0.4444)

    assert policy.reference_time_step(0.40, +1.0, 0.02) == 21
    assert policy.reference_time_step(0.40, -1.0, 0.02) == 135
    assert policy.reference_time_step(99.0, +1.0, 0.02) == 0
    assert policy.reference_time_step(-99.0, -1.0, 0.02) == 215


class _FakeSession:
    def __init__(self) -> None:
        self.feeds = None

    def run(self, output_names, feeds):
        assert output_names == ["actions"]
        self.feeds = feeds
        return [np.arange(NUM_JOINTS, dtype=np.float32).reshape(1, NUM_JOINTS)]


def test_build1_joint_columns_round_trip_between_sdk_and_policy_order():
    policy = OnnxPolicy.__new__(OnnxPolicy)
    policy._sess = _FakeSession()
    policy._input_name = "obs"
    policy._time_step_name = "time_step"
    policy._output_name = "actions"
    policy._policy_to_sdk = np.arange(NUM_JOINTS - 1, -1, -1, dtype=np.int64)

    obs = np.arange(110, dtype=np.float32)
    action = policy.infer(obs, time_step=42)

    np.testing.assert_array_equal(policy._sess.feeds["obs"][0, 3:34], obs[3:34][::-1])
    np.testing.assert_array_equal(policy._sess.feeds["obs"][0, 34:65], obs[34:65][::-1])
    np.testing.assert_array_equal(policy._sess.feeds["obs"][0, 65:96], obs[65:96][::-1])
    np.testing.assert_array_equal(policy._sess.feeds["time_step"], [[42.0]])
    np.testing.assert_array_equal(action, np.arange(NUM_JOINTS, dtype=np.float32)[::-1])
