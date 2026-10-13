"""Regression checks for the failure modes found in W&B run o8c40w3y."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch


@pytest.fixture
def rewards():
    path = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/hope_rewards.py"
    names = {"_deadband_huber", "_record_metric_snapshot", "_settled_refinement_gate",
             "_bounded_refinement_debt", "settled_leg_command_debt", "settled_waist_pitch_tracking_debt"}
    nodes = [n for n in ast.parse(path.read_text()).body if getattr(n, "name", None) in names]
    scope = {"torch": torch, "math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), scope)
    return scope


def fixture():
    names = ["waist_pitch_joint", "right_shoulder_pitch_joint", "head_yaw_joint"]
    names += [f"{side}_{joint}_joint" for side in ("right", "left")
              for joint in ("knee", "hip_roll", "hip_yaw", "hip_pitch", "ankle_roll", "ankle_pitch")]
    asset = SimpleNamespace(data=SimpleNamespace(joint_pos=torch.zeros(4, len(names))))
    action = SimpleNamespace(_joint_names=names, _asset=asset,
                             _select_action_joints=lambda x: x.flip(-1),
                             executed_qdes_delta=torch.full((4, len(names)), .03),
                             _qdes_executed=torch.full((4, len(names)), .4))
    motion = SimpleNamespace(in_hold=torch.tensor([True, False, False, True]))
    cmd = SimpleNamespace(time_to_strike=torch.tensor([0., .7, -.4, 0.]), metrics={})
    force = torch.zeros(4, 2, 3); force[:, :, 2] = 100.
    force[3, 0, 2] = 0.  # A step during HOLD is also exempt.
    sensor = SimpleNamespace(data=SimpleNamespace(net_forces_w=force))
    terms = {"motion": motion, "racket_target": cmd}
    env = SimpleNamespace(common_step_counter=576000,
                          command_manager=SimpleNamespace(get_term=terms.__getitem__),
                          action_manager=SimpleNamespace(get_term=lambda _: action),
                          scene=SimpleNamespace(sensors={"contact": sensor}))
    return env, action, cmd, motion, sensor, SimpleNamespace(name="contact", body_ids=[0, 1])


@pytest.mark.parametrize("step,scale", [(0, 0.), (448000, 0.), (512000, .5), (576000, 1.), (999999, 1.)])
def test_no_early_exploration_tax_and_linear_ramp(rewards, step, scale):
    env, _, cmd, _, _, cfg = fixture()
    for name in ("settled_leg_command_debt", "settled_waist_pitch_tracking_debt"):
        env.common_step_counter = 576000
        full = rewards[name](env, cfg)
        assert full[0] > 0
        env.common_step_counter = step
        torch.testing.assert_close(rewards[name](env, cfg), full * scale)
        assert cmd.metrics["settled_refinement_scale"].eq(scale).all()


def test_windup_followthrough_and_single_support_are_free(rewards):
    env, _, _, _, _, cfg = fixture()
    for name in ("settled_leg_command_debt", "settled_waist_pitch_tracking_debt"):
        value = rewards[name](env, cfg)
        assert value[0] > 0
        assert value[1:].eq(0).all()


def test_legs_only_no_max_joint_amplification_or_arm_tax(rewards):
    env, action, _, _, _, cfg = fixture()
    action.executed_qdes_delta.zero_()
    action.executed_qdes_delta[:, :3] = 100.
    assert rewards["settled_leg_command_debt"](env, cfg).eq(0).all()
    action.executed_qdes_delta[:, 3] = .02
    one = rewards["settled_leg_command_debt"](env, cfg)
    action.executed_qdes_delta[:, 3:] = -.02
    all_legs = rewards["settled_leg_command_debt"](env, cfg)
    torch.testing.assert_close(one * 12, all_legs)


def test_noise_tails_have_bounded_budget_and_monotone_cost(rewards):
    env, action, _, _, _, cfg = fixture()
    previous = 0.
    for amplitude in (.002, .01, .1, 1., 10000.):
        action.executed_qdes_delta.fill_(amplitude)
        action._qdes_executed.fill_(amplitude)
        leg = rewards["settled_leg_command_debt"](env, cfg)
        waist = rewards["settled_waist_pitch_tracking_debt"](env, cfg)
        budget = .05 * leg + .10 * waist
        assert torch.isfinite(budget).all() and (budget <= .150001).all()
        assert budget[0] >= previous
        previous = budget[0]


def test_tracking_uses_postphysics_order_and_allows_pd_preload(rewards):
    env, action, cmd, _, _, cfg = fixture()
    # Waist action column 0 is the last articulation column in this fixture.
    action._asset.data.joint_pos[:, -1] = .30
    assert rewards["settled_waist_pitch_tracking_debt"](env, cfg).eq(0).all()
    action._asset.data.joint_pos[:, -1] = .1
    positive = rewards["settled_waist_pitch_tracking_debt"](env, cfg)
    action._qdes_executed[:, 0] = -.2
    torch.testing.assert_close(rewards["settled_waist_pitch_tracking_debt"](env, cfg), positive)
    snapshot = cmd.metrics["settled_waist_pitch_tracking_abs_rad"].clone()
    action._qdes_executed.zero_()
    action._asset.data.joint_pos.zero_()
    torch.testing.assert_close(cmd.metrics["settled_waist_pitch_tracking_abs_rad"], snapshot)


@pytest.mark.parametrize("kwargs", [{"std": 0}, {"std": float("nan")}, {"margin": -1},
                                    {"ramp_steps": 0}, {"start_steps": -1}, {"ramp_steps": float("inf")}])
def test_invalid_bands_and_schedules_fail(rewards, kwargs):
    env, _, _, _, _, cfg = fixture()
    for name in ("settled_leg_command_debt", "settled_waist_pitch_tracking_debt"):
        with pytest.raises(ValueError):
            rewards[name](env, cfg, **kwargs)
