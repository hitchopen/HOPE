"""Actual hard-stop clearance, including a stationary waist supporting on a stop."""
import math
import sys
import types
from types import SimpleNamespace as NS

import pytest
import torch

from test_gaussian_action_projection import load_module, UTILS
from test_settled_policy_refinement import fake_environment

package = types.ModuleType("joint_limit_test_modules")
package.__path__ = [str(UTILS)]
sys.modules[package.__name__] = package
limits = load_module("joint_limit_test_modules.actual_joint_limit", UTILS/'actual_joint_limit.py')


def fixture():
    env, action, sensor, motion = fake_environment()
    action._asset.data.joint_pos_limits = torch.tensor([-1., 1.]).reshape(1, 1, 2).repeat(4, 31, 1)
    action._asset.data.joint_pos_limits[:, 14] = torch.tensor([-.418879, .418879])
    action._passive_action_cols = torch.tensor([15, 16])
    cmd = NS(metrics={}, time_to_strike=torch.tensor([1., 0., -.8, .65]))
    motion.in_hold[:] = torch.tensor([True, False, False, False])
    terms = {'racket_target': cmd, 'motion': motion}
    env.command_manager.get_term = terms.__getitem__
    return env, action, cmd


def test_buffer_and_hard_boundary_have_monotone_nonvanishing_overrun_gradient():
    q = torch.tensor([[0., .9, .95, 1., 1.05]], requires_grad=True)
    hard = torch.tensor([-1., 1.]).expand(1, 5, 2)
    debt, distance = limits.limit_clearance_cost(q, hard, torch.full_like(q, .1))
    torch.testing.assert_close(debt, torch.tensor([[0., 0., .25, 1., 2.]]), atol=1e-6, rtol=1e-5)
    debt.sum().backward()
    assert q.grad[0, 4] > 0
    assert q.grad[0, 4] == pytest.approx(float(q.grad[0, 3]))
    assert distance[0, 4] < 0
    reflected, _ = limits.limit_clearance_cost(-q.detach(), hard, torch.full_like(q, .1))
    torch.testing.assert_close(reflected, debt.detach())


def test_waist_on_stop_not_diluted_and_rest_penalty_is_stronger_from_step_zero():
    env, action, cmd = fixture()
    action._asset.data.joint_pos[:, 14] = .418879
    debt = limits.evaluate_actual_joint_limit(env)
    torch.testing.assert_close(debt, torch.tensor([3., 1., 3., 2.]))
    assert env.common_step_counter == 0
    assert cmd.metrics['actual_limit_waist_pitch_upper_clearance_rad'].eq(0).all()
    # Contact force and a matching command are irrelevant: q_des-q is not used.
    env.scene.sensors['contact_forces'].data.net_forces_w.zero_()
    action._qdes_executed = action._asset.data.joint_pos.clone()
    torch.testing.assert_close(limits.evaluate_actual_joint_limit(env), debt)


def test_model16100_like_overshoot_increases_cost_instead_of_flattening_at_stop():
    env, action, _ = fixture()
    action._asset.data.joint_pos[:, 14] = .418879
    on_stop = limits.evaluate_actual_joint_limit(env)
    action._asset.data.joint_pos[:, 14] = .422
    assert (limits.evaluate_actual_joint_limit(env) > on_stop).all()


def test_normal_pose_with_point_four_pd_bias_costs_nothing():
    env, action, _ = fixture()
    action._qdes_executed = action._asset.data.joint_pos+.4
    assert limits.evaluate_actual_joint_limit(env).eq(0).all()


def test_nonwaist_joint_also_penalized_and_passive_head_excluded():
    env, action, _ = fixture()
    action._asset.data.joint_pos[:, 15:17] = 1.1
    assert limits.evaluate_actual_joint_limit(env).eq(0).all()
    action._asset.data.joint_pos[:, 0] = 1.
    torch.testing.assert_close(limits.evaluate_actual_joint_limit(env), torch.tensor([1.5, .5, 1.5, 1.]))


def test_narrow_ranges_and_per_environment_hard_limits_use_physical_values():
    env, action, cmd = fixture()
    action._asset.data.joint_pos_limits[0, 14] = torch.tensor([-.05, .05])
    action._asset.data.joint_pos[0, 14] = .04
    debt = limits.evaluate_actual_joint_limit(env)
    assert float(cmd.metrics['actual_limit_waist_pitch_buffer_rad'][0]) == pytest.approx(.02)
    assert float(debt[0]) == pytest.approx(.75)
    assert debt[1:].eq(0).all()


def test_metric_reset_does_not_modify_plant_limits_or_state():
    env, action, cmd = fixture()
    action._asset.data.joint_pos[:, 14] = .39
    limits.evaluate_actual_joint_limit(env)
    before = action._asset.data.joint_pos.clone()
    for metric in cmd.metrics.values():
        metric.zero_()
    torch.testing.assert_close(action._asset.data.joint_pos, before)


@pytest.mark.parametrize('kwargs', [{'margin_rad': 0}, {'waist_margin_rad': float('nan')}, {'hold_recover_multiplier': .5}])
def test_invalid_configuration_rejected(kwargs):
    env, *_ = fixture()
    with pytest.raises(ValueError):
        limits.evaluate_actual_joint_limit(env, **kwargs)


def test_missing_hard_limits_never_falls_back_to_soft_limits():
    env, action, _ = fixture()
    del action._asset.data.joint_pos_limits
    with pytest.raises(AttributeError):
        limits.evaluate_actual_joint_limit(env)
