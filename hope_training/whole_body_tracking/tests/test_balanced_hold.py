"""v10 protects feedback, actual pose pressure and the original PD-offset contract."""
import math

import pytest
import torch

from test_full_body_hold import environment, full
from test_gaussian_action_projection import UTILS, load_module, policy_modules

contract = load_module("balanced_contract_test", UTILS / "balanced_hold_contract.py")


def test_positive_pose_score_keeps_large_error_pressure_and_ignores_pd_bias():
    env, cmd, _, action = environment()
    assert full.full_body_hold(env, positive_kernel=True).eq(1).all()
    scores = []
    for angle in (.1, .4, 1.0):
        cmd.robot.data.joint_pos[:, action._joint_names.index('left_knee_joint')] += angle
        score = full.full_body_hold(env, positive_kernel=True)
        assert score.gt(math.exp(-1.8)).all() and score.lt(1).all()
        scores.append(score)
    assert (scores[0] > scores[1]).all() and (scores[1] > scores[2]).all()
    action._qdes_executed += 1.0
    cmd.robot.data.applied_torque *= 4
    torch.testing.assert_close(full.full_body_hold(env, positive_kernel=True), scores[-1])
    # Full-body posture cannot evade its objective by moving away/unloading.
    cmd.base_target_pos_w[:, 0] = .3
    env.scene.sensors['contact_forces'].data.net_forces_w.zero_()
    assert (full.full_body_hold(env, positive_kernel=True) < scores[-1]).all()


def test_positive_pose_kernel_preserves_early_teacher_and_strike_release():
    env, cmd, motion, action = environment()
    cmd.robot.data.joint_pos[:, action._joint_names.index('right_elbow_joint')] += .4
    env.common_step_counter = 448000
    early = full.full_body_hold(env, positive_kernel=True)
    torch.testing.assert_close(early, full.full_body_hold(env))
    env.common_step_counter = 576000
    late = full.full_body_hold(env, positive_kernel=True)
    env.common_step_counter = 512000
    torch.testing.assert_close(full.full_body_hold(env, positive_kernel=True), (early+late)/2)
    env.common_step_counter = 576000
    motion.in_hold.zero_()
    for tts in (.1, -.5):
        cmd.time_to_strike.fill_(tts)
        assert full.full_body_hold(env, positive_kernel=True).eq(0).all()
    cmd.time_to_strike.fill_(-.8)
    assert full.full_body_hold(env, positive_kernel=True).gt(0).all()


def test_quieting_releases_recovery_without_bad_wrist_disabling_pose_or_quiet():
    env, cmd, motion, action = environment(5)
    cmd.robot.data.joint_vel.fill_(.3)
    cmd.robot.data.joint_pos[1, action._joint_names.index('right_wrist_pitch_joint')] += .5
    cmd.base_target_pos_w[2, 0] = .3
    cmd.robot.data.root_lin_vel_w[3, 0] = .3
    env.scene.sensors['contact_forces'].data.net_forces_w[4].zero_()
    quiet = full.full_body_quiet(env, pose_independent=True)
    assert quiet[0] > 0 and quiet[1] == quiet[0]
    assert quiet[2:].eq(0).all()  # travel, high speed and single support are released
    pose = full.full_body_hold(env, positive_kernel=True)
    assert pose[1] < pose[0] and pose[2] < pose[0]
    motion.in_hold.zero_(); cmd.time_to_strike.fill_(.1)
    assert full.full_body_quiet(env, pose_independent=True).eq(0).all()


def recipe():
    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf
    with initialize_config_dir(version_base=None, config_dir=str(UTILS.parents[3]/'cfg')):
        # Historical v9 is no longer a selectable task. Keep its comparison
        # fixture here without restoring the removed training entry point.
        old = compose(config_name='train', overrides=['task=HOPEPingPongFullBodyHold', 'algo=ppo_full_body'])
        OmegaConf.set_struct(old, False)
        old = OmegaConf.merge(old, {'task': {'rewards': {
            'full_body_hold_cost_scale': 3.6, 'full_body_quiet_state_independent': True,
            'full_body_quiet_weight': -.15, 'station_recovery_weight': -.75},
            'build_contract': {'reward_revision': 'actual_stable_hold_v9'}}})
        new = compose(config_name='train', overrides=['task=HOPEPingPongBalancedHold', 'algo=ppo_balanced_hold'])
    return OmegaConf.to_container(old, resolve=True), OmegaConf.to_container(new, resolve=True)


def test_v10_preserves_plant_observations_actions_ball_clock_and_joint_safety():
    old, new = recipe()
    for k in old['task']:
        if k not in ('rewards', 'build_contract'):
            assert old['task'][k] == new['task'][k], k
    a, b = old['task']['rewards'], new['task']['rewards']
    changed = {k for k in a.keys() | b.keys() if a.get(k) != b.get(k)}
    assert changed == {'full_body_hold_cost_scale', 'full_body_hold_positive_kernel',
                       'full_body_quiet_state_independent', 'full_body_quiet_pose_independent',
                       'full_body_quiet_weight', 'full_body_quiet_start_steps', 'station_recovery_weight'}
    assert b['actual_joint_limit_weight'] == a['actual_joint_limit_weight'] == -1
    assert b['full_body_hold_start_steps'] == 7000*64
    assert b['full_body_quiet_start_steps'] == 6000*64
    assert b['full_body_quiet_ramp_steps'] == 2000*64
    algorithm = new['algo']['algorithm']
    assert algorithm['projection_aux_enabled'] is True  # action boundary objective remains
    contract.validate_balanced_hold_algorithm('actual_balanced_hold_v10', algorithm)


@pytest.mark.parametrize('key,value', [('settled_aux_enabled', True),
    ('settled_aux_smooth_coef', .01), ('settled_aux_tracking_coef', .01),
    ('settled_aux_gradient_ratio_max', .05)])
@pytest.mark.parametrize('revision', ['actual_balanced_hold_v10', 'actual_hold_reference_v11', 'actual_complete_hold_v11', 'actual_complete_hold_v11_loaded_feet_v1'])
def test_v10_rejects_accidentally_reenabled_temporal_actor_objective(key, value, revision):
    _, new = recipe()
    algorithm = new['algo']['algorithm']; algorithm[key] = value
    with pytest.raises(ValueError, match='ppo_balanced_hold'):
        contract.validate_balanced_hold_algorithm(revision, algorithm)


def test_v10_real_ppo_update_has_no_temporal_gradient_but_still_learns(policy_modules):
    from test_settled_policy_refinement import build_algorithm
    torch.manual_seed(37)
    obs, policy, alg = build_algorithm(policy_modules, enabled=False)
    assert alg._settled_refinement is None
    before = [p.detach().clone() for p in policy.actor.parameters()]
    with torch.no_grad():
        for _ in range(4):
            action = alg.act(obs)
            obs = obs.clone(); obs['policy'] = torch.randn(4, 6)
            alg.process_env_step(obs, -action.square().mean(-1), torch.zeros(4, dtype=torch.bool), {})
        alg.compute_returns(obs)
    stats = alg.update()
    assert stats['settled_aux_enabled'] == 0 and stats['settled_mean_gradient_ratio'] == 0
    assert 'settled_mean_qdes_step_rad' not in stats  # disabled is not perfect smoothness
    assert all(math.isfinite(v) for v in stats.values())
    assert any(not torch.equal(a, b) for a, b in zip(before, policy.actor.parameters()))
