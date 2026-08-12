"""Observation-contract test for the passive head joints (applied-action feedback).

Training (``mdp/hope_actions.py``) binds the passive columns FROM THE ACTION TERM by
joint name (``cfg.passive_joint_names = A3_PASSIVE_HEAD_JOINT_NAMES``), zeroes them in
``applied_raw_actions``, and exposes exactly that buffer as the policy's ``actions``
observation (``mdp.applied_last_action``); the processed targets hold the passive
joints at their defaults. The deploy runner and the MuJoCo evaluator must reproduce
this — the actor must never see nonzero values in observation columns that were
always zero during training.

Host-side (no torch / Isaac / MuJoCo / onnxruntime) this asserts BOTH ends:

  * the training-side declaration: ``A3_PASSIVE_HEAD_JOINT_NAMES`` (read from
    ``hope_actions.py`` as a literal) names head_yaw/head_pitch, which occupy
    indices 3 and 4 of the canonical joint order — the same ``HEAD_INDICES`` the
    reference runner zeroes;
  * the deploy-side behavior: the REAL ``PingPongReferenceRunner`` tick loop (fake
    bridge + fake policy) keeps the head columns of ``last_action`` zero even though
    the policy emits ones, feeds the zeroed applied action back in the observation's
    actions slice ([65:96] of the 110-D contract), and writes the default head pose.

Run:  python tests/test_passive_head_feedback.py   (or pytest)
"""

from __future__ import annotations

import ast
import importlib.util
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_REFERENCE_DIR = os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference")
_RUNTIME_YAML = os.path.join(
    _REPO, "a3_deploy", "a3_deploy_example", "config", "hope_pingpong_runtime.yaml"
)
_HOPE_ACTIONS_PY = os.path.join(
    _ROOT,
    "source", "whole_body_tracking", "whole_body_tracking",
    "tasks", "tracking", "mdp", "hope_actions.py",
)
_ADAPTER_CONFIG_PY = os.path.join(
    _ROOT, "source", "whole_body_tracking", "whole_body_tracking", "utils",
    "action_adapter_config.py",
)

sys.path.insert(0, _REFERENCE_DIR)

from a3_deploy_onnx_ref_pingpong.config import RuntimeConfig  # noqa: E402
from a3_deploy_onnx_ref_pingpong.joint_order import HEAD_INDICES, JOINT_NAMES, NUM_JOINTS  # noqa: E402
from a3_deploy_onnx_ref_pingpong.observation import RobotState  # noqa: E402
from a3_deploy_onnx_ref_pingpong.racket_command import QueueRacketCommandSource  # noqa: E402
from a3_deploy_onnx_ref_pingpong.runner import PingPongReferenceRunner  # noqa: E402
from a3_deploy_onnx_ref_pingpong.sim_bridge import SimBridge  # noqa: E402

_LAST_ACTION_SLICE = slice(65, 96)  # 110-D hitter_pure contract: actions (applied) columns
_HEAD = list(HEAD_INDICES)
_ACTUATED = [i for i in range(NUM_JOINTS) if i not in _HEAD]


def _training_passive_head_names() -> tuple[str, ...]:
    """Read the A3_PASSIVE_HEAD_JOINT_NAMES literal from hope_actions.py (no torch import)."""
    with open(_HOPE_ACTIONS_PY, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=_HOPE_ACTIONS_PY)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "A3_PASSIVE_HEAD_JOINT_NAMES":
                    value = ast.literal_eval(node.value)
                    return tuple(str(v) for v in value)
    raise AssertionError("A3_PASSIVE_HEAD_JOINT_NAMES not found in hope_actions.py")


def _load_by_path(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_training_passive_names_bind_to_head_columns_3_and_4():
    """The training-side declaration and the deploy HEAD_INDICES name the same columns."""
    passive = _training_passive_head_names()
    assert passive == ("head_yaw_joint", "head_pitch_joint")
    # Resolve the names against the canonical joint order (the action term resolves its
    # columns by name at runtime; the canonical order is what the exported ONNX uses).
    adapter_cfg = _load_by_path("hope_adapter_cfg_passive", _ADAPTER_CONFIG_PY)
    order = adapter_cfg.load_joint_order()
    assert tuple(order) == tuple(JOINT_NAMES)
    cols = tuple(order.index(name) for name in passive)
    assert cols == tuple(HEAD_INDICES) == (3, 4)


class _FakeBridge(SimBridge):
    """Static standing robot; records every written joint target."""

    def __init__(self, default_q: np.ndarray) -> None:
        self._q = default_q.copy()
        self.written_q_des: list[np.ndarray] = []

    def reset(self) -> None:
        pass

    def read_state(self) -> RobotState:
        return RobotState(
            base_pos_w=np.array([0.0, 0.0, 1.0]),
            base_quat_w=np.array([1.0, 0.0, 0.0, 0.0]),
            base_ang_vel_b=np.zeros(3),
            q=self._q.copy(),
            qd=np.zeros(NUM_JOINTS),
        )

    def write_targets(self, q_des, kp, kd) -> None:
        self.written_q_des.append(np.asarray(q_des, dtype=np.float64).copy())

    def step(self) -> None:
        pass


class _OnesPolicy:
    """Emits all-ones raw actions and records every observation it saw."""

    def __init__(self) -> None:
        self.seen_obs: list[np.ndarray] = []

    def infer(self, obs: np.ndarray) -> np.ndarray:
        self.seen_obs.append(np.asarray(obs, dtype=np.float64).copy())
        return np.ones(NUM_JOINTS, dtype=np.float32)


def _run_ticks(n: int = 3):
    cfg = RuntimeConfig.load(_RUNTIME_YAML)
    assert cfg.passive_neck, "shipped runtime config must keep the neck passive"
    bridge = _FakeBridge(cfg.action_adapter.default_q)
    policy = _OnesPolicy()
    runner = PingPongReferenceRunner(cfg, bridge, QueueRacketCommandSource(), policy=policy)
    runner.run(max_ticks=n, status_every=0)
    return cfg, bridge, policy, runner


def test_last_action_head_columns_zeroed():
    _cfg, _bridge, policy, runner = _run_ticks(3)
    assert runner.last_action.shape == (NUM_JOINTS,)
    assert np.all(runner.last_action[_HEAD] == 0.0)
    assert np.all(runner.last_action[_ACTUATED] == 1.0)


def test_observation_actions_slice_matches_training_contract():
    _cfg, _bridge, policy, _runner = _run_ticks(3)
    # Tick 0 sees the zero-initialized last_action; from tick 1 on it must be the
    # APPLIED action: ones in actuated columns, zeros in the passive head columns.
    first = policy.seen_obs[0][_LAST_ACTION_SLICE]
    assert policy.seen_obs[0].shape == (110,)
    assert np.all(first == 0.0)
    for obs in policy.seen_obs[1:]:
        la = obs[_LAST_ACTION_SLICE]
        assert np.all(la[_HEAD] == 0.0), "passive head columns must stay zero in the actions slice"
        assert np.all(la[_ACTUATED] == 1.0)


def test_head_targets_written_at_default():
    cfg, bridge, _policy, _runner = _run_ticks(2)
    for q_des in bridge.written_q_des:
        np.testing.assert_allclose(q_des[_HEAD], cfg.action_adapter.default_q[_HEAD])


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
    print(f"\n{len(tests) - failed}/{len(tests)} passive-head feedback tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
