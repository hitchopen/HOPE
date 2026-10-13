"""Exercise the production reward against the real clamped/delayed action path."""
import ast
import math
from types import SimpleNamespace

import pytest
import torch

from test_qdes_delay import SOURCE, action_class, make_action, command


@pytest.fixture
def reward():
    tree = ast.parse(SOURCE.read_text())
    names = {"executed_qdes_difference_l2", "resolve_action_columns_by_joint_name"}
    nodes = [n for n in tree.body if getattr(n, "name", None) in names]
    scope = {"torch": torch, "math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope["executed_qdes_difference_l2"]


def environment(action, held, tts=1.0):
    racket = SimpleNamespace(time_to_strike=torch.full((action._num_envs,), tts), metrics={})
    motion = SimpleNamespace(in_hold=torch.tensor(held, dtype=torch.bool))
    terms = {"racket_target": racket, "motion": motion}
    return SimpleNamespace(action_manager=SimpleNamespace(get_term=lambda _: action),
                           command_manager=SimpleNamespace(get_term=terms.__getitem__))


def test_hold_is_four_times_live_cost_and_passive_head_is_ignored(action_class, reward):
    action = make_action(action_class, num_envs=2, max_steps=0)
    action.process_actions(command(action, .2, passive_value=99.))
    env = environment(action, [True, False])
    result = reward(env, "racket_target", hold_multiplier=4.)
    torch.testing.assert_close(result, torch.tensor([.16, .04]))


def test_strike_exemption_does_not_silence_a_frozen_hold_clock(action_class, reward):
    action = make_action(action_class, num_envs=2, max_steps=0)
    action.process_actions(command(action, .2))
    env = environment(action, [True, False], tts=0.)
    # Use the fixture's sole active joint to exercise the named exemption mask.
    result = reward(env, "racket_target", hold_multiplier=4.,
                    upper_joint_names=("left_ankle_pitch_joint",))
    torch.testing.assert_close(result, torch.tensor([.16, 0.]))


def test_clamp_and_delay_own_cost_not_raw_gaussian_samples(action_class, reward):
    action = make_action(action_class, num_envs=2)
    action._qdes_delay_steps[:] = torch.tensor([0, 1])
    action.process_actions(command(action, .2))
    action.process_actions(command(action, 20.))
    env = environment(action, [False, False])
    torch.testing.assert_close(reward(env, "racket_target"), torch.tensor([3.24, 0.]))
    action.process_actions(command(action, 40.))
    torch.testing.assert_close(reward(env, "racket_target"), torch.tensor([0., 3.24]))


def test_reversal_cost_and_partial_reset_have_no_stale_history(action_class, reward):
    action = make_action(action_class, num_envs=2, max_steps=0)
    env = environment(action, [True, True])
    action.process_actions(command(action, .2))
    first = reward(env, "racket_target", hold_multiplier=4.)
    action.process_actions(command(action, -.2))
    torch.testing.assert_close(reward(env, "racket_target", hold_multiplier=4.), 4*first)
    action.reset(torch.tensor([0]))
    action.process_actions(command(action, 0.))
    torch.testing.assert_close(reward(env, "racket_target", hold_multiplier=4.), torch.tensor([0., .16]))


@pytest.mark.parametrize("value", [.5, float("nan"), float("inf")])
def test_invalid_hold_weights_fail(action_class, reward, value):
    action = make_action(action_class, num_envs=1)
    with pytest.raises(ValueError, match="hold multiplier"):
        reward(environment(action, [True]), "racket_target", hold_multiplier=value)
