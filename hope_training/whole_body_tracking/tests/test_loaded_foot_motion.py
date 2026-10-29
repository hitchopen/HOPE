"""Exercise the production contact-motion reward without starting Isaac/PPO."""
import ast
from types import SimpleNamespace as NS

import pytest
import torch

from test_complete_hold_robustness import PKG, recipe, load


SOURCE = PKG/'tasks/tracking/mdp/hope_rewards.py'
NAMES = {'_cmd', '_record_metric_snapshot', '_deadband_huber', 'settle_foot_slip_penalty'}
nodes = [n for n in ast.parse(SOURCE.read_text()).body if getattr(n, 'name', None) in NAMES]
scope = {'torch': torch, 'ManagerBasedRLEnv': object, 'RacketTargetCommand': object}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), scope)
penalty = scope['settle_foot_slip_penalty']


def fixture(n=6):
    # Different articulation/sensor indices exercise manager-resolved order.
    asset = NS(data=NS(body_lin_vel_w=torch.zeros(n, 4, 3), body_ang_vel_w=torch.zeros(n, 4, 3)))
    sensor = NS(data=NS(net_forces_w=torch.zeros(n, 4, 3)))
    sensor.data.net_forces_w[:, [3, 1], 2] = 200
    cmd = NS(metrics={}, base_pos_w=torch.zeros(n, 3), base_target_pos_w=torch.ones(n, 2),
             time_to_strike=torch.tensor([3., .1, 0., -.15, -.8, -2.5])[:n])
    class Scene(dict):
        sensors = {'contact_forces': sensor}
    env = NS(scene=Scene(robot=asset), command_manager=NS(get_term=lambda _: cmd))
    config_source = PKG/'tasks/tracking/config/agibot_a3/hope_env_cfg.py'
    cls = next(n for n in ast.parse(config_source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'HOPEPingPongHitterPureRallyV13RewardsCfg')
    term = next(n for n in cls.body if isinstance(n, ast.Assign)
                and any(getattr(t, 'id', '') == 'settle_foot_slip' for t in n.targets))
    params = next(k.value for k in term.value.keywords if k.arg == 'params')
    cfg = eval(compile(ast.Expression(params), str(config_source), 'eval'),
               {'SceneEntityCfg': lambda name, **_: NS(name=name, body_ids=[0, 2] if name == 'robot' else [3, 1]),
                'A3_FEET_BODIES': ['left', 'right']})
    term_cfg = NS(weight=-.1, params=cfg)
    overrides = load('loaded_foot_overrides', PKG/'utils/task_reward_overrides.py')
    overrides.apply_reward_overrides(NS(settle_foot_slip=term_cfg),
        {k: v for k, v in recipe()['task']['rewards'].items() if k.startswith('settle_foot_slip_')}, [])
    return env, cmd, asset, sensor, cfg


def test_loaded_translation_has_no_station_or_strike_recovery_gap():
    env, cmd, asset, sensor, cfg = fixture()
    asset.data.body_lin_vel_w[:, 0, 0] = .27
    value = penalty(env, **cfg)
    assert value.gt(0).all()
    torch.testing.assert_close(value, value[0].expand_as(value))
    # Neither a stationary loaded foot nor distance can dilute this foot's cost.
    sensor.data.net_forces_w[:, 1].zero_()
    cmd.base_target_pos_w.zero_()
    torch.testing.assert_close(penalty(env, **cfg), value)
    asset.data.body_lin_vel_w[:, 0, 0] = .6
    assert (penalty(env, **cfg) > value).all()


def test_pure_loaded_twist_is_visible_but_airborne_foot_motion_is_free():
    env, cmd, asset, sensor, cfg = fixture()
    asset.data.body_ang_vel_w[:, 0, 2] = torch.tensor([1., -1., 1., -1., 1., -1.])
    value = penalty(env, **cfg)
    assert value.gt(0).all()
    torch.testing.assert_close(value, value[0].expand_as(value))
    assert cmd.metrics['loaded_foot_slip_speed'].eq(0).all()
    assert cmd.metrics['loaded_foot_twist_rate'].eq(1).all()
    asset.data.body_lin_vel_w[:, 0, :2] = 2
    # Large sideways force alone is not upward support load.
    sensor.data.net_forces_w[:, 3] = torch.tensor([100., 0., 0.])
    assert penalty(env, **cfg).eq(0).all()


def test_deadbands_allow_small_adjustments_and_roll_pitch_are_not_twist():
    env, _, asset, _, cfg = fixture()
    asset.data.body_lin_vel_w[:, 0, 0] = .02
    asset.data.body_ang_vel_w[:, 0] = torch.tensor([2., 3., .10])
    assert penalty(env, **cfg).eq(0).all()


def test_legacy_mode_retains_arrival_gate_and_does_not_require_angular_state():
    env, cmd, asset, _, cfg = fixture()
    cfg.update(all_phases=False, twist_weight=0.)
    del asset.data.body_ang_vel_w
    asset.data.body_lin_vel_w[:, 0, 0] = .27
    value = penalty(env, **cfg)
    assert value[:4].eq(0).all() and value[4] > 0 and value[5] == 0
    cmd.base_target_pos_w.zero_()
    assert penalty(env, **cfg)[1:3].gt(0).all()


def test_metrics_do_not_alias_plant_state_and_are_uploaded():
    env, cmd, asset, _, cfg = fixture()
    asset.data.body_lin_vel_w[:, 0, 0] = .27
    asset.data.body_ang_vel_w[:, 0, 2] = .8
    penalty(env, **cfg)
    linear, angular = asset.data.body_lin_vel_w.clone(), asset.data.body_ang_vel_w.clone()
    logging = load('loaded_foot_logging', PKG/'utils/wandb_logging.py')
    for name, value in cmd.metrics.items():
        assert logging.wandb_metric_allowed('Live/racket_target/'+name, 'hitter_pingpong_v1')
        value.zero_()
    torch.testing.assert_close(asset.data.body_lin_vel_w, linear)
    torch.testing.assert_close(asset.data.body_ang_vel_w, angular)


@pytest.mark.parametrize('old_revision', ['actual_complete_hold_v11', 'actual_complete_hold_v11_loaded_feet_v1'])
def test_contract_retains_pose_and_balance_feedback_and_rejects_old_reward_resume(old_revision):
    cfg = recipe()
    assert cfg['task']['env']['episode_length_s'] == 12
    revision = cfg['task']['build_contract']['reward_revision']
    assert revision == 'actual_complete_hold_v11_strike_position_v2'
    balanced = load('loaded_foot_balanced', PKG/'utils/balanced_hold_contract.py')
    algorithm = dict(cfg['algo']['algorithm'], settled_aux_enabled=True)
    with pytest.raises(ValueError, match='ppo_balanced_hold'):
        balanced.validate_balanced_hold_algorithm(revision, algorithm)
    train = PKG.parents[2]/'scripts/train.py'
    node = next(n for n in ast.parse(train.read_text()).body
                if getattr(n, 'name', '') == '_validate_training_checkpoint')
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(train), 'exec'), namespace)
    with pytest.raises(ValueError, match='reward revision'):
        namespace[node.name]({'hope_exact_resume_state': {
            'training_distribution_revision': 'complete_hold_robust_v1',
            'reward_revision': old_revision}}, NS(), 'unused', revision, 'complete_hold_robust_v1')


def test_wrong_resolved_foot_count_fails_instead_of_silently_disabling_cost():
    env, _, _, _, cfg = fixture()
    cfg['sensor_cfg'].body_ids = [3]
    with pytest.raises(RuntimeError, match='exactly two'):
        penalty(env, **cfg)
