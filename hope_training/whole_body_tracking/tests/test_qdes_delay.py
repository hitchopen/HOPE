"""CPU tests of production q_des FIFO, reset and Markov replay methods.

Only Isaac's affine JointPositionAction base and articulation storage are
replaced. The two repository action classes are compiled unchanged from their
AST so no Kit process is required; FIFO/replay math is never duplicated here.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")
SOURCE = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/hope_actions.py"


class IsaacAffineStorage:
    """Minimal upstream affine action/storage interface, without physics."""

    def __init__(self, cfg, env):
        self.cfg = cfg
        self.device = "cpu"
        self._asset = env.robot
        self._joint_names = env.robot.joint_names
        self._joint_ids = list(range(len(self._joint_names)))
        self.action_dim = len(self._joint_names)
        self._raw_actions = torch.zeros(env.num_envs, self.action_dim)
        self._processed_actions = self._raw_actions.clone()
        self._offset = env.robot.data.default_joint_pos.clone()
        self._scale = torch.full((self.action_dim,), 0.5)

    def process_actions(self, actions):
        self._raw_actions.copy_(actions)
        self._processed_actions = actions * self._scale + self._offset

    def reset(self, env_ids=None):
        rows = slice(None) if env_ids is None else env_ids
        self._raw_actions[rows] = 0.0
        self._processed_actions[rows] = 0.0

    @property
    def raw_actions(self):
        return self._raw_actions


@pytest.fixture(scope="module")
def action_class():
    names = {
        "resolve_action_columns_by_joint_name", "_action_joint_ids_in_column_order",
        "ClampedJointPositionAction", "V11SafeClampedJointPositionAction",
    }
    tree = ast.parse(SOURCE.read_text())
    definitions = [node for node in tree.body if getattr(node, "name", None) in names]
    namespace = {
        "torch": torch, "math": math, "JointPositionAction": IsaacAffineStorage,
        "V11_SAFE_QDES_ACTION_CONTRACT": "v11_affine_safe_qdes_v1",
    }
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["V11SafeClampedJointPositionAction"]


def make_action(action_class, num_envs=4, max_steps=1, nominal_fraction=0.5):
    limits = torch.tensor([-1.0, 1.0]).expand(num_envs, 3, 2).clone()
    robot = SimpleNamespace(
        joint_names=["left_ankle_pitch_joint", "head_yaw_joint", "head_pitch_joint"],
        data=SimpleNamespace(
            soft_joint_pos_limits=limits,
            joint_pos_limits=limits * 1.2,
            default_joint_pos=torch.zeros(num_envs, 3),
        ),
    )
    cfg = SimpleNamespace(
        passive_joint_names=("head_yaw_joint", "head_pitch_joint"),
        action_contract="v11_affine_safe_qdes_v1",
        actual_q_hard_tolerance_rad=0.002,
        actual_q_hard_audit_mode="telemetry",
        qdes_delay_min_steps=0,
        qdes_delay_max_steps=max_steps,
        qdes_delay_nominal_fraction=nominal_fraction,
    )
    return action_class(cfg, SimpleNamespace(robot=robot, num_envs=num_envs))


def command(term, active_value, passive_value=3.0):
    actions = torch.full_like(term.raw_actions, passive_value)
    actions[:, 0] = active_value
    return actions


def test_fifo_delays_clamped_target_but_preserves_raw_feedback(action_class):
    term = make_action(action_class)
    term._qdes_delay_steps[:] = torch.tensor([0, 1, 0, 1])
    term.process_actions(command(term, 0.2))
    # Normal reset bootstraps the queue with the first command, without a stale zero pulse.
    torch.testing.assert_close(term._qdes_executed[:, 0], torch.full((4,), 0.1))
    term.process_actions(command(term, 4.0))
    torch.testing.assert_close(term._qdes_executed[:, 0], torch.tensor([1.0, 0.1, 1.0, 0.1]))
    torch.testing.assert_close(term._unclamped_processed_actions[:, 0], torch.full((4,), 2.0))
    torch.testing.assert_close(term.applied_raw_actions[:, 0], torch.full((4,), 4.0))
    assert bool((term.applied_raw_actions[:, 1:] == 0.0).all())
    assert bool((term._qdes_delay_queue.abs() <= 1.0).all())
    term.process_actions(command(term, -0.8))
    torch.testing.assert_close(term._qdes_executed[:, 0], torch.tensor([-0.4, 1.0, -0.4, 1.0]))


def test_partial_reset_does_not_change_other_environments(action_class):
    term = make_action(action_class)
    term._qdes_delay_steps[:] = torch.tensor([0, 1, 0, 1])
    term.process_actions(command(term, 0.2))
    term.process_actions(command(term, 0.8))
    untouched = torch.tensor([0, 2, 3])
    before = term.capture_markov_replay_state(untouched)
    term.reset(torch.tensor([1]))
    after = term.capture_markov_replay_state(untouched)
    for key in before:
        torch.testing.assert_close(after[key], before[key], rtol=0, atol=0)
    assert bool((term._qdes_delay_queue[1] == 0.0).all())
    assert bool(term._qdes_delay_seed_pending[1])
    assert not bool(term._qdes_delay_seed_pending[untouched].any())


def test_same_environment_markov_replay_restores_next_fifo_transition(action_class):
    term = make_action(action_class)
    term._qdes_delay_steps[:] = torch.tensor([0, 1, 0, 1])
    term.process_actions(command(term, 0.2))
    term.process_actions(command(term, 1.6))
    replay_env = torch.tensor([1])
    snapshot = term.capture_markov_replay_state(replay_env)
    next_action = command(term, -0.6)
    term.process_actions(next_action)
    expected = term.capture_markov_replay_state(replay_env)
    term.reset(replay_env)
    term.restore_markov_replay_state(replay_env, snapshot)
    assert not bool(term._qdes_delay_seed_pending[1])
    term.process_actions(next_action)
    actual = term.capture_markov_replay_state(replay_env)
    for key in expected:
        torch.testing.assert_close(actual[key], expected[key], rtol=0, atol=0)
    # Executed q_des is the queued 0.8, while current raw action history is -0.6.
    assert actual["executed_qdes"][0, 0] == pytest.approx(0.8)
    assert actual["applied_raw"][0, 0] == pytest.approx(-0.6)


def test_nominal_half_plus_uniform_random_half_means_quarter_delayed(action_class):
    torch.manual_seed(112)
    term = make_action(action_class, num_envs=20000)
    assert bool((term._qdes_delay_steps[::2] == 0).all())
    delayed_fraction = (term._qdes_delay_steps == 1).float().mean().item()
    assert delayed_fraction == pytest.approx(0.25, abs=0.015)
    # Stable cohort membership survives arbitrary partial reset order.
    ids = torch.tensor([17, 4, 13, 0, 22])
    term.reset(ids)
    assert bool((term._qdes_delay_steps[ids[ids % 2 == 0]] == 0).all())


def test_all_nominal_retains_direct_execution_even_with_fifo_allocated(action_class):
    direct = make_action(action_class, max_steps=0, nominal_fraction=1.0)
    fifo = make_action(action_class, max_steps=1, nominal_fraction=1.0)
    for value in [0.3, 5.0, -0.7, -3.0]:
        direct.process_actions(command(direct, value))
        fifo.process_actions(command(fifo, value))
        torch.testing.assert_close(fifo._processed_actions, direct._processed_actions, rtol=0, atol=0)
        torch.testing.assert_close(fifo.applied_raw_actions, direct.applied_raw_actions, rtol=0, atol=0)


def test_cross_environment_nonzero_lag_cannot_contaminate_nominal_cohort(action_class):
    term = make_action(action_class)
    term._qdes_delay_steps[1] = 1
    term.process_actions(command(term, 0.2))
    snapshot = term.capture_markov_replay_state(torch.tensor([1]))
    # Current v3 replay selects same-environment samples. This protects against
    # a regression to v2's unrestricted cross-environment sampling.
    with pytest.raises(RuntimeError, match="non-zero q_des lag in the nominal cohort"):
        term.restore_markov_replay_state(torch.tensor([0]), snapshot)
