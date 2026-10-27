"""Stationary rally: immutable station, wide reach bank, slippery-floor prior."""
import ast
import hashlib
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from stationary_test_support import PKG, ROOT, load, loader, robust
from test_sim2real_export import fixture as export_fixture, module as exporter

TASK = 'HOPEPingPongStationaryRally'


def load_utils_package_module(name):
    """Load a utils module under a stub package so its relative imports resolve without Isaac."""
    import importlib, importlib.util, sys, types
    package = 'stationary_utils_stub'
    if package not in sys.modules:
        stub = types.ModuleType(package); stub.__path__ = [str(PKG/'utils')]
        sys.modules[package] = stub
    return importlib.import_module(f'{package}.{name}')


commands = load_utils_package_module('task_command_overrides')
overrides = load('stationary_reward_overrides', PKG/'utils/task_reward_overrides.py')


def recipe():
    with initialize_config_dir(config_dir=str(ROOT/'cfg'), version_base=None):
        return OmegaConf.to_container(compose(config_name='train', overrides=[f'task={TASK}', 'algo=ppo_balanced_hold']), resolve=True)


def racket_cfg(task):
    cfg = dict(task['racket'])
    cfg['racket_pos_range_per_clip'] = commands._clip_ranges(cfg['pos_range_per_clip'])
    return cfg


def test_recipe_never_moves_the_station_and_keeps_the_rally_abi():
    cfg = recipe()
    task, algo = cfg['task'], cfg['algo']
    robust.validate_training_contract(task, algo)
    r = task['racket']
    assert r['base_target_x_range'] == r['base_target_y_range'] == [0, 0]
    assert r['station_y_step_range'] is None and r['station_y_same_prob'] == 1.0
    assert r['venue_tuple_fixed_station'] and r['venue_tuple_robust_sampling']
    assert not r['venue_tuple_limit_reset_travel']
    assert r['venue_tuple_final_mix_prob'] == 1.0 and r['venue_tuple_mix_mode'] == 'per_swing_bank_v2'
    assert r['pos_range_per_clip']['forehand']['y'][1] <= r['pos_range_per_clip']['backhand']['y'][0]
    assert task['build_contract']['station_command_contract'] == 'fixed_session_anchor_v1'
    assert task['build_contract']['recovery_target_contract'] == 'fixed_session_anchor_v1'
    assert task['build_contract']['actor_observation_dim'] == 110 and task['build_contract']['action_dim'] == 31
    assert task['motion']['interruptible_recovery_enabled'] and not task['motion']['wrap_teleport']
    assert task['rewards']['foot_friction_cone_weight'] < 0
    assert task['friction_contract']['robot_static_friction_range'][0] <= 0.25
    assert not algo['algorithm']['settled_aux_enabled']


@pytest.mark.parametrize('mutation', [
    ('racket', 'venue_tuple_fixed_station', False),
    ('racket', 'base_target_y_range', [-0.1, 0.1]),
    ('racket', 'station_y_step_range', [0.12, 0.35]),
    ('build_contract', 'station_command_contract', 'current_command_station_v1'),
])
def test_contract_rejects_any_moving_station(mutation):
    cfg = recipe()
    block, key, value = mutation
    cfg['task'][block][key] = value
    with pytest.raises(ValueError, match='stationary_reach_v1'):
        robust.validate_training_contract(cfg['task'], cfg['algo'])


def test_contract_rejects_overlapping_side_bands():
    cfg = recipe()
    cfg['task']['racket']['pos_range_per_clip']['backhand']['y'][0] = -0.40
    with pytest.raises(ValueError, match='strictly on the negative side'):
        robust.validate_training_contract(cfg['task'], cfg['algo'])
    # Adjacent bands are the point: no dead zone for the planner between FH and BH.
    cfg = recipe()
    fh, bh = cfg['task']['racket']['pos_range_per_clip']['forehand'], cfg['task']['racket']['pos_range_per_clip']['backhand']
    assert fh['y'][1] == bh['y'][0]
    robust.validate_training_contract(cfg['task'], cfg['algo'])


def test_bank_commands_the_anchor_and_covers_both_reach_boxes():
    task = recipe()['task']
    r = task['racket']
    box = np.array(commands._clip_ranges(r['pos_range_per_clip']))
    vbox = np.array(commands._clip_ranges(r['vel_range_per_clip']))
    sets = []
    for split in ('train', 'dev', 'test'):
        path = (ROOT/r['venue_tuple_bank_path']).with_name(f'{split}.npz')
        receipt = json.loads(path.with_suffix('.receipt.json').read_text())
        assert hashlib.sha256(path.read_bytes()).hexdigest() == receipt['bank_sha256']
        assert receipt['contract']['distribution'] == 'stationary_reach_v1'
        assert receipt['contract']['recipe_revision'] == r['venue_tuple_bank_recipe_revision']
        if split == 'train':
            assert hashlib.sha256(path.read_bytes()).hexdigest() == r['venue_tuple_bank_sha256']
            assert hashlib.sha256(path.with_suffix('.receipt.json').read_bytes()).hexdigest() == r['venue_tuple_bank_receipt_sha256']
        with np.load(path) as a:
            sets.append(set(a['parent_id']))
            assert np.all(a['station_xy'] == 0)
            for side in (0, 1):
                mask = a['clip'] == side
                offsets = a['target_position_offset'][mask]
                assert (offsets >= box[side, :, 0] - 1e-6).all() and (offsets <= box[side, :, 1] + 1e-6).all()
                velocity = a['target_velocity'][mask]
                assert (velocity >= vbox[side, :, 0] - 1e-6).all() and (velocity <= vbox[side, :, 1] + 1e-6).all()
                kinds = a['source_kind'][mask]
                assert (kinds == 0).any() and (kinds == 1).any()
                if split == 'train':
                    assert mask.sum() >= r['venue_tuple_bank_min_rows_per_side']
                    # Every lateral quarter of the wide box is populated, both sides.
                    y = offsets[kinds == 1, 1]
                    edges = np.linspace(box[side, 1, 0], box[side, 1, 1], 5)
                    assert (np.histogram(y, edges)[0] >= 0.15 * len(y)).all()
                    # Core rows keep the original reach centers the warm-started actor knows.
                    core = offsets[kinds == 0]
                    np.testing.assert_allclose(core[:, 0], 0.58, atol=1e-6)
                    np.testing.assert_allclose(core[:, 1], -0.44 if side == 0 else -0.09, atol=1e-6)
    assert not (sets[0] & sets[1] or sets[0] & sets[2] or sets[1] & sets[2])


def test_unbounded_sampler_never_falls_back_and_keeps_the_requested_mixture():
    torch.manual_seed(5)
    obj = loader(racket_cfg(recipe()['task']))
    for bank in obj._venue_tuple_bank_by_clip.values():
        previous = torch.rand(2048, 2) - 0.5
        remaining = torch.full((2048,), 0.05)  # far below any travel budget
        row, requested, fallback = robust.sample_rows(bank, previous, remaining,
            speed=.6, brake_s=.2, wide_probability=.7, unbounded=True)
        assert not fallback.any()
        assert torch.equal(bank['source_kind'][row].bool(), requested)
        assert .65 < requested.float().mean() < .75
        with pytest.raises(RuntimeError, match='travel budget'):
            robust.sample_rows(bank, previous + 5.0, remaining, speed=.6, brake_s=.2, wide_probability=.7)


def test_production_loader_requires_matching_station_contract():
    cfg = racket_cfg(recipe()['task'])
    obj = loader(cfg)
    assert all(bank['station_xy'].eq(0).all() for bank in obj._venue_tuple_bank_by_clip.values())
    cfg['venue_tuple_fixed_station'] = False
    with pytest.raises(RuntimeError, match='venue_tuple_fixed_station'):
        loader(cfg)
    cfg = racket_cfg(recipe()['task'])
    cfg['racket_pos_range_per_clip'] = ((( .58, .58), (-.48, -.40), (.85, 1.30)), ((.58, .58), (-.13, -.05), (.85, 1.30)))
    with pytest.raises(RuntimeError, match='inside pos_range_per_clip'):
        loader(cfg)


def test_export_marks_fixed_session_anchor_for_the_two_clip_stationary_rally():
    env, task, _, motion, racket = export_fixture()
    task['build_contract'].update(station_command_contract='fixed_session_anchor_v1',
                                  recovery_target_contract='fixed_session_anchor_v1',
                                  training_distribution_revision='stationary_reach_v1')
    motion.step_reference_mode = 'station_error'
    racket.base_target_x_range = racket.base_target_y_range = (0., 0.)
    racket.station_y_step_range = None
    racket.venue_tuple_fixed_station = True
    racket.venue_tuple_mix_mode = 'per_swing_bank_v2'
    racket.venue_tuple_final_mix_prob = 1.0
    racket.venue_tuple_robust_sampling = True
    racket.venue_tuple_limit_reset_travel = False
    racket.racket_pos_range_per_clip = (((.48, .66), (-.64, -.30), (.85, 1.30)), ((.48, .66), (-.27, .20), (.85, 1.30)))
    md = exporter.sim2real_metadata(env, task)
    assert md['hitter_pure_station_contract'] == 'fixed_session_anchor_v1'
    # The deployed runner only accepts this value under the fixed-anchor contract.
    assert md['hitter_pure_target_sample_contract'] == 'four_clip_ball_clock_v1'
    assert md['hitter_pure_training_distribution_revision'] == 'stationary_reach_v1'
    assert 'hitter_pure_station_y_step_range' not in md
    racket.racket_pos_range_per_clip = (((.48, .66), (-.64, -.30), (.85, 1.30)), ((.48, .66), (-.35, .20), (.85, 1.30)))
    with pytest.raises(RuntimeError, match='stationary reach bank'):
        exporter.sim2real_metadata(env, task)
    racket.racket_pos_range_per_clip = (((.44, .70), (-.68, -.24), (.80, 1.35)), ((.44, .70), (-.24, .28), (.80, 1.35)))
    assert exporter.sim2real_metadata(env, task)['hitter_pure_station_contract'] == 'fixed_session_anchor_v1'
    racket.racket_pos_range_per_clip = (((.48, .66), (-.64, -.30), (.85, 1.30)), ((.48, .66), (-.27, .20), (.85, 1.30)))
    racket.base_target_y_range = (-.1, .1)
    with pytest.raises(RuntimeError, match='immutable station'):
        exporter.sim2real_metadata(env, task)


SOURCE = PKG/'tasks/tracking/mdp/hope_rewards.py'
NAMES = {'_cmd', '_record_metric_snapshot', '_deadband_huber', 'foot_friction_cone_debt'}
nodes = [n for n in ast.parse(SOURCE.read_text()).body if getattr(n, 'name', None) in NAMES]
scope = {'torch': torch, 'ManagerBasedRLEnv': object, 'RacketTargetCommand': object}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), scope)
friction_cone = scope['foot_friction_cone_debt']


def friction_fixture(n=4, bodies=3):
    sensor = NS(data=NS(net_forces_w=torch.zeros(n, 4, 3)))
    sensor.data.net_forces_w[:, [3, 1], 2] = 300.
    asset = NS(data=NS(body_com_lin_vel_w=torch.zeros(n, bodies, 3)),
               root_physx_view=NS(get_masses=lambda: torch.tensor([[10., 20., 30.]] * n)))
    cmd = NS(metrics={})
    class Scene(dict):
        sensors = {'contact_forces': sensor}
    env = NS(scene=Scene(robot=asset), command_manager=NS(get_term=lambda _: cmd),
             step_dt=0.02, episode_length_buf=torch.full((n,), 50))
    return env, cmd, asset, sensor, NS(name='robot'), NS(name='contact_forces', body_ids=[3, 1])


def test_friction_cone_charges_required_shear_from_com_acceleration():
    env, cmd, asset, sensor, asset_cfg, sensor_cfg = friction_fixture()
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).eq(0).all()  # first call: no history
    asset.data.body_com_lin_vel_w[:, :, 0] = 0.02  # 1 m/s^2 over 20 ms: ratio 0.10, inside margin
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).eq(0).all()
    torch.testing.assert_close(cmd.metrics['foot_friction_ratio'], torch.full((4,), 1 / 9.81))
    asset.data.body_com_lin_vel_w[:, :, 0] = 0.02 + 0.12  # +6 m/s^2: ratio 0.61, charged
    value = friction_cone(env, 'racket_target', asset_cfg, sensor_cfg)
    assert value.gt(0).all()
    # Mass weighting: the same net impulse carried by the heaviest body only is identical.
    asset.data.body_com_lin_vel_w[:, :, 0] = 0.14
    asset.data.body_com_lin_vel_w[:, 2, 0] = 0.14 + 0.12 * 60 / 30
    torch.testing.assert_close(friction_cone(env, 'racket_target', asset_cfg, sensor_cfg), value)
    # Unloading (vertical deceleration) raises the ratio for the same shear.
    asset.data.body_com_lin_vel_w[:, :, 0] = 0.26 + 0.12
    asset.data.body_com_lin_vel_w[:, :, 2] = -0.06
    assert (friction_cone(env, 'racket_target', asset_cfg, sensor_cfg) > value).all()


def test_friction_cone_is_free_in_flight_and_right_after_reset():
    env, cmd, asset, sensor, asset_cfg, sensor_cfg = friction_fixture()
    friction_cone(env, 'racket_target', asset_cfg, sensor_cfg)
    asset.data.body_com_lin_vel_w[:, :, 1] = 0.2
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).gt(0).all()
    asset.data.body_com_lin_vel_w[:, :, 1] = 0.4
    sensor.data.net_forces_w[:, [3, 1], 2] = 0.
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).eq(0).all()
    sensor.data.net_forces_w[:, [3, 1], 2] = 300.
    asset.data.body_com_lin_vel_w[:, :, 1] = 0.6
    env.episode_length_buf[:] = 1
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).eq(0).all()
    env.episode_length_buf[:] = 2
    asset.data.body_com_lin_vel_w[:, :, 1] = 0.8
    assert friction_cone(env, 'racket_target', asset_cfg, sensor_cfg).gt(0).all()
    with pytest.raises(RuntimeError, match='two resolved foot'):
        friction_cone(env, 'racket_target', asset_cfg, NS(name='contact_forces', body_ids=[1]))


def test_friction_cone_recipe_reaches_the_registered_term():
    term = NS(weight=0., params=dict(ratio_margin=.3, std=.15, force_threshold=10.))
    rewards = {k: v for k, v in recipe()['task']['rewards'].items() if k.startswith('foot_friction_cone_')}
    applied = []
    overrides.apply_reward_overrides(NS(foot_friction_cone=term), rewards, applied)
    assert term.weight == -0.15 and term.params['ratio_margin'] == 0.30 and term.params['std'] == 0.15
    assert term.params['force_threshold'] == 10.0


def test_swing_imitation_releases_the_racket_arm_but_keeps_torso_and_left_arm():
    rewards = recipe()['task']['rewards']
    names = rewards['swing_imitation_body_names']
    assert 'torso_Link' in names and not any('right_' in n for n in names)
    terms = {k: NS(weight=1.0, params=dict(body_names=['torso_Link', 'right_wrist_yaw_Link']))
             for k in ('motion_body_pos', 'motion_body_ori', 'motion_body_lin_vel', 'motion_body_ang_vel')}
    applied = []
    overrides.apply_reward_overrides(NS(**terms), {'swing_imitation_body_names': names}, applied)
    assert all(t.params['body_names'] == list(names) for t in terms.values())
    assert len(applied) == 4
    with pytest.raises(ValueError, match='distinct body names'):
        overrides.apply_reward_overrides(NS(**terms), {'swing_imitation_body_names': ['torso_Link', 'torso_Link']}, [])
    with pytest.raises(ValueError, match='list of body names'):
        overrides.apply_reward_overrides(NS(**terms), {'swing_imitation_body_names': 'torso_Link'}, [])
    assert rewards['racket_position_std'] == 0.22 and rewards['racket_exact_position_debt_weight'] == -4.0


def test_hold_reference_owns_reset_pose_and_action_zero():
    task = recipe()['task']
    motion = task['motion']
    assert motion['hold_reference_sets_action_default'] is True
    reference = load('stationary_pose_reference', PKG/'utils/task_pose_reference.py')
    pose = reference.load_reference(motion['hold_reference_path'], motion['hold_reference_sha256'])
    assert pose['candidate_key'] == 'B_32_5_K20' and len(pose['joint_order']) == 31
    robot = NS(init_state=NS(joint_pos={}, pos=(0., 0., 1.), rot=(1., 0., 0., 0.)))
    env_cfg = NS(commands=NS(motion=NS(hold_reference_path=None, hold_reference_sha256=None,
                                       hold_reference_sets_action_default=True, rally_pre_strike_s_range=(.35, .85)),
                             racket_target=NS()), actions=NS(joint_pos=NS()), scene=NS(robot=robot))
    applied = []
    commands.apply_command_overrides(env_cfg, {'motion': {k: motion[k] for k in (
        'hold_reference_path', 'hold_reference_sha256', 'hold_reference_sets_action_default', 'rally_pre_strike_s_range')}}, applied)
    assert robot.init_state.joint_pos['left_knee_joint'] == pytest.approx(pose['actual_q_reference_rad'][pose['joint_order'].index('left_knee_joint')])
    assert robot.init_state.pos[2] == pytest.approx(1.066, abs=1e-3)
    assert any('B_32_5_K20' in a for a in applied)
    assert env_cfg.commands.motion.rally_pre_strike_s_range == (0.35, 0.95)


def test_balance_weights_reach_registered_terms():
    rewards = recipe()['task']['rewards']
    terms = {k: NS(weight=0., params={}) for k in ('upright', 'pre_strike_tilt', 'dynamic_support')}
    terms['dynamic_support'].params = dict(std=.1, margin=.02, foot_radius=.06)
    applied = []
    overrides.apply_reward_overrides(NS(**terms), {k: v for k, v in rewards.items() if k in (
        'upright_weight', 'pre_strike_tilt_weight', 'dynamic_support_weight')}, applied)
    assert terms['upright'].weight == -1.5 and terms['pre_strike_tilt'].weight == -0.5 and terms['dynamic_support'].weight == -2.5


def test_v4_bank_covers_flat_strokes_centre_aiming_and_planner_traces():
    with np.load(ROOT/recipe()['task']['racket']['venue_tuple_bank_path']) as a:
        tv, off = a['target_velocity'], a['target_position_offset']
        assert (a['parent_id'] < 0).sum() >= 10000  # planner-trace parents on both sides
        for side in (0, 1):
            m = a['clip'] == side
            wide = m & (a['source_kind'] == 1) & (a['parent_id'] >= 0)
            assert np.quantile(tv[wide, 2], 0.05) < 0.45          # flat strokes exist
            assert np.corrcoef(off[wide, 1], tv[wide, 1])[0, 1] < -0.3  # lateral velocity leans back to centre
            assert off[m, 2].max() > 1.40                          # high contacts
            assert ((a['parent_id'] < 0) & m).sum() >= 8000         # planner-trace parents on this side
    receipt = json.loads((ROOT/recipe()['task']['racket']['venue_tuple_bank_receipt_path']).read_text())
    assert receipt['contract']['mutation']['vz_scale'][0] == 0.0


def test_momentum_terms_reach_registered_terms():
    rewards = recipe()['task']['rewards']
    terms = {'post_swing_base_quiet': NS(weight=0., params=dict(std=.25, margin=.08, t_lo=.2, t_hi=1.55, huber_tail=True)),
             'base_ang_vel_xy_rally': NS(weight=0., params=dict(strike_exclusion_s=.12))}
    applied = []
    overrides.apply_reward_overrides(NS(**terms), {k: v for k, v in rewards.items() if k.startswith(('post_swing_base_quiet', 'base_ang_vel_xy_rally'))}, applied)
    assert terms['post_swing_base_quiet'].weight == -0.5 and terms['base_ang_vel_xy_rally'].weight == -0.3
    assert terms['base_ang_vel_xy_rally'].params['strike_exclusion_s'] == 0.12
