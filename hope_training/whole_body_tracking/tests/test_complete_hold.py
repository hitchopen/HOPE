"""v11: the entire ready state matters, but a new ball always releases it."""
import ast
import math
from types import SimpleNamespace as NS

import pytest
import torch

from test_full_body_hold import environment, full
from test_gaussian_action_projection import UTILS, load_module

complete = load_module("full_hold_test.complete_hold", UTILS / "complete_hold.py")


def fixture(n=4):
    env, cmd, motion, action = environment(n)
    cmd.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.]).repeat(n, 1)
    env.step_dt = .02
    env.episode_length_buf = torch.ones(n, dtype=torch.long)
    return env, cmd, motion, action


def reward(env):
    return full.full_body_hold(env, positive_kernel=True, completion_blend=.5)


def tick(env):
    env.common_step_counter += 1
    env.episode_length_buf += 1


def test_complete_pose_needs_contiguous_dwell_and_reset_cannot_inherit_success():
    env, cmd, _, _ = fixture()
    first = reward(env)
    for _ in range(14):
        tick(env); reward(env)
    assert cmd.metrics['complete_hold_settled'].eq(0).all()
    tick(env)
    torch.testing.assert_close(reward(env), torch.ones(4))
    assert (first < 1).all()
    assert cmd.metrics['complete_hold_settled'].eq(1).all()
    tick(env); env.episode_length_buf[0] = 1
    cmd.base_target_pos_w[1, 0] = .001  # even a new reachable station starts a new dwell
    reward(env)
    assert cmd.metrics['complete_hold_dwell_s'][:2].eq(0).all()
    assert cmd.metrics['complete_hold_settled'][2:].eq(1).all()


def test_single_bad_wrist_cannot_be_hidden_by_other_limbs_or_pd_bias():
    env, cmd, _, action = fixture()
    col = action._joint_names.index('right_wrist_pitch_joint')
    baseline = reward(env).clone()
    cmd.robot.data.joint_pos[:, col] += math.radians(15)
    bad = reward(env)
    assert (bad < baseline-.25).all()
    assert cmd.metrics['complete_hold_ready'].eq(0).all()
    action._qdes_executed += .5
    cmd.robot.data.applied_torque *= 3
    torch.testing.assert_close(reward(env), bad)
    ratios = torch.tensor([[3., .9, .9], [3., .1, .1]])
    assert complete.weakest_score(ratios)[0] == complete.weakest_score(ratios)[1]


def test_station_speed_support_and_body_geometry_are_joint_requirements():
    env, cmd, _, _ = fixture(5)
    cmd.base_target_pos_w[0, 0] = .08
    cmd.robot.data.root_lin_vel_w[1, 1] = .15
    env.scene.sensors['contact_forces'].data.net_forces_w[2, 0].zero_()
    cmd.robot.data.body_pos_w[3, 3, 2] += .04
    cmd.robot.data.joint_vel[4, 0] = 1.0
    reward(env)
    assert cmd.metrics['complete_hold_ready'].eq(0).all()
    assert (cmd.metrics['complete_hold_worst_ratio'] > 1).all()


def test_strike_interrupts_immediately_and_dwell_cannot_span_an_unobserved_gap():
    env, cmd, motion, _ = fixture()
    for _ in range(20):
        reward(env); tick(env)
    motion.in_hold.zero_(); cmd.time_to_strike.fill_(.7)
    assert reward(env).eq(0).all()
    assert cmd.metrics['complete_hold_dwell_s'].eq(0).all()
    tick(env); motion.in_hold.fill_(True)
    assert (reward(env) < 1).all()
    # Calling the function twice for one physics tick must not double the dwell.
    before = cmd.metrics['complete_hold_dwell_s'].clone()
    reward(env)
    torch.testing.assert_close(cmd.metrics['complete_hold_dwell_s'], before)
    env.common_step_counter += 10; env.episode_length_buf += 10
    reward(env)
    assert cmd.metrics['complete_hold_dwell_s'].eq(0).all()


def test_curriculum_does_not_change_the_early_teacher_or_make_large_errors_negative():
    env, cmd, _, action = fixture()
    env.common_step_counter = 448000
    torch.testing.assert_close(reward(env), full.full_body_hold(env, positive_kernel=True))
    env.common_step_counter = 576000
    cmd.robot.data.joint_pos += 1.
    value = reward(env)
    assert value.gt(0).all() and value.lt(1).all()


def test_anchor_teacher_releases_only_fixed_pose_owned_phases():
    path = UTILS.parent/'tasks/tracking/mdp/rewards.py'
    nodes = [n for n in ast.parse(path.read_text()).body
             if isinstance(n, ast.FunctionDef) and n.name == 'motion_global_anchor_orientation_error_exp']
    scope = {'torch': torch, 'ManagerBasedRLEnv': object, 'MotionCommand': object,
             'quat_error_magnitude': lambda a, b: (a-b).norm(dim=-1)}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)
    fn = scope['motion_global_anchor_orientation_error_exp']
    motion = NS(in_hold=torch.tensor([True, False, False, False]),
                anchor_quat_w=torch.zeros(4, 4), robot_anchor_quat_w=torch.ones(4, 4)*.01)
    racket = NS(time_to_strike=torch.tensor([.8, .7, -.5, -.8]))
    env = NS(command_manager=NS(get_term={'motion': motion, 'racket_target': racket}.__getitem__))
    old = fn(env, 'motion', .4)
    new = fn(env, 'motion', .4, exclude_recovery=True)
    assert new[[0, 3]].eq(0).all()
    torch.testing.assert_close(new[1:3], old[1:3])


def test_recipes_are_separable_and_preserve_control_and_incoming_ball_contracts():
    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf
    with initialize_config_dir(version_base=None, config_dir=str(UTILS.parents[3]/'cfg')):
        recipes = [OmegaConf.to_container(compose(config_name='train', overrides=[
            'task='+t, 'algo=ppo_balanced_hold']), resolve=True) for t in (
            'HOPEPingPongBalancedHold', 'HOPEPingPongHoldReference', 'HOPEPingPongCompleteHold')]
    old, anchor, new = recipes
    for cfg in (anchor, new):
        for k in old['task']:
            allowed = ('rewards', 'build_contract', 'racket') if cfg is new else ('rewards', 'build_contract')
            if k not in allowed:
                assert cfg['task'][k] == old['task'][k], k
        assert cfg['algo'] == old['algo']
        assert not cfg['algo']['algorithm']['settled_aux_enabled']
    a, b = old['task']['rewards'], anchor['task']['rewards']
    assert {k for k in a.keys() | b.keys() if a.get(k) != b.get(k)} == {'motion_anchor_exclude_recovery'}
    r = new['task']['rewards']
    assert r['full_body_hold_start_steps'] == r['station_recovery_start_steps'] == 6000*64
    assert r['full_body_hold_ramp_steps'] == 2000*64
    assert r['executed_qdes_rate_weight'] == a['executed_qdes_rate_weight']
    assert r['full_body_quiet_weight'] == a['full_body_quiet_weight']


def test_absent_hold_is_not_success_and_new_metrics_reach_wandb():
    env, cmd, motion, _ = fixture()
    motion.in_hold.zero_(); cmd.time_to_strike.fill_(.7)
    reward(env)
    summary = full.phase_summaries(cmd.metrics)
    assert torch.isnan(summary['complete_hold_ready_fraction'])
    assert torch.isnan(summary['complete_hold_settled_fraction'])
    logging = load_module('complete_hold_logging', UTILS/'wandb_logging.py')
    for name in (*summary, *cmd.metrics):
        if name.startswith('complete_hold_'):
            assert logging.wandb_metric_allowed('Live/racket_target/'+name, 'hitter_pingpong_v1')
