"""Exercise the production impulse event with an ordinary torch robot stand-in."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import torch


_PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/sim2real_events.py"
_SPEC = importlib.util.spec_from_file_location("hope_sim2real_events", _PATH)
events = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(events)


def _environment(count=12):
    original = torch.arange(count * 6, dtype=torch.float32).reshape(count, 6) / 10.0
    writes = []
    robot = SimpleNamespace(
        device="cpu",
        data=SimpleNamespace(root_vel_w=original.clone()),
        write_root_velocity_to_sim=lambda velocity, env_ids: writes.append((env_ids.clone(), velocity.clone())),
    )
    command = SimpleNamespace(time_to_strike=torch.ones(count))
    env = SimpleNamespace(
        num_envs=count,
        common_step_counter=96_000,
        scene={"robot": robot},
        command_manager=SimpleNamespace(get_term=lambda _: command),
        episode_length_buf=torch.full((count,), 50, dtype=torch.long),
        reset_buf=torch.zeros(count, dtype=torch.bool),
        termination_manager=SimpleNamespace(dones=torch.zeros(count, dtype=torch.bool)),
    )
    return env, robot, command, writes, original


def test_default_cohort_is_inert_without_resolving_robot_or_commands():
    events.bounded_planar_push(object(), None)


def test_impulse_preserves_existing_motion_and_nonplanar_velocity():
    env, robot, _, writes, original = _environment()
    torch.manual_seed(71)
    events.bounded_planar_push(env, None, cohort_fraction=1.0)
    ids, velocity = writes[0]
    delta = velocity - original[ids]
    assert torch.all(delta[:, :2].abs() <= 0.100001)
    assert torch.any(delta[:, :2].abs() > 0.001)
    torch.testing.assert_close(velocity[:, 2:], original[ids, 2:], rtol=0, atol=0)
    # The simulation write owns state changes; the cached observation buffer remains untouched.
    torch.testing.assert_close(robot.data.root_vel_w, original, rtol=0, atol=0)


def test_common_step_ramp_scales_impulse_and_zero_start_has_no_write():
    env, _, _, writes, original = _environment()
    env.common_step_counter = 0
    events.bounded_planar_push(env, None, cohort_fraction=1.0)
    assert writes == []
    env.common_step_counter = 48_000
    torch.manual_seed(9)
    events.bounded_planar_push(env, None, cohort_fraction=1.0)
    half = writes[-1][1] - original
    env.common_step_counter = 192_000
    torch.manual_seed(9)
    events.bounded_planar_push(env, None, cohort_fraction=1.0)
    full = writes[-1][1] - original
    torch.testing.assert_close(full, 2 * half, rtol=0, atol=1.0e-6)


def test_fixed_cohort_does_not_change_when_interval_ids_arrive_in_subsets():
    env, _, _, writes, _ = _environment()
    events.bounded_planar_push(env, torch.tensor([0, 3, 4, 7]), cohort_fraction=0.5)
    assert writes[0][0].tolist() == [3, 7]
    events.bounded_planar_push(env, torch.tensor([7, 0, 3, 4]), cohort_fraction=0.5)
    assert writes[1][0].tolist() == [7, 3]


def test_strike_and_reset_exclusions_do_not_delay_or_modify_commands():
    env, _, cmd, writes, _ = _environment()
    cmd.time_to_strike[:5] = torch.tensor([-0.15, 0.0, 0.15, float("nan"), float("inf")])
    env.reset_buf[5] = True
    env.termination_manager.dones[6] = True
    env.episode_length_buf[7] = 0
    env.episode_length_buf[8] = 1
    cmd.time_to_strike[9] = -0.30
    before = cmd.time_to_strike.clone()
    events.bounded_planar_push(env, None, cohort_fraction=1.0)
    assert writes[0][0].tolist() == [9, 10, 11]
    torch.testing.assert_close(cmd.time_to_strike, before, equal_nan=True)


def test_empty_interval_and_fully_excluded_cohort_are_noops():
    env, _, _, writes, _ = _environment()
    events.bounded_planar_push(env, torch.tensor([], dtype=torch.long), cohort_fraction=1.0)
    events.bounded_planar_push(env, torch.tensor([0, 2, 4]), cohort_fraction=0.5)
    assert writes == []
