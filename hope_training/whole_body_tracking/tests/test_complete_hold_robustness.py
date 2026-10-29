"""Offline tests of the actual bank loader, sampler, curriculum and resume guard."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT/'source/whole_body_tracking/whole_body_tracking'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


robust = load('robust_question_test', PKG/'utils/robust_ball_distribution.py')
transport = load('robust_transport_test', PKG/'tasks/tracking/mdp/mocap_transport.py')


def recipe():
    with initialize_config_dir(config_dir=str(ROOT/'cfg'), version_base=None):
        return OmegaConf.to_container(compose(config_name='train', overrides=['task=HOPEPingPongCompleteHold', 'algo=ppo_balanced_hold']), resolve=True)


def initialize_production_velocity_boxes(racket):
    """Run the real YAML translator and constructor's velocity checks without Kit."""
    source = PKG/'tasks/tracking/mdp/hope_commands.py'
    tree = ast.parse(source.read_text())
    config = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommandCfg')
    defaults = {n.target.id: ast.literal_eval(n.value) for n in config.body
                if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                and n.target.id.startswith('racket_vel_')}
    obj = NS(cfg=NS(**defaults), device='cpu')
    translator = load('robust_command_overrides_test', PKG/'utils/task_command_overrides.py')
    env_cfg = NS(commands=NS(motion=NS(), racket_target=obj.cfg), actions=NS(joint_pos=NS()))
    translator.apply_command_overrides(env_cfg, {'racket': {
        k: v for k, v in racket.items() if k.startswith('vel_') and k in translator._RACKET_ALIASES}}, [])
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    def assignment_to(node, name):
        return isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Attribute) and t.attr == name for t in node.targets)
    start = next(i for i, n in enumerate(init.body) if assignment_to(n, '_vel_range_per_clip_t'))
    end = next(i for i, n in enumerate(init.body) if assignment_to(n, '_pos_range_per_clip_t'))
    exec(compile(ast.Module(body=init.body[start:end], type_ignores=[]), str(source), 'exec'),
         {'torch': torch, 'self': obj})
    return obj


def test_resolved_velocity_curriculum_passes_real_constructor_checks():
    obj = initialize_production_velocity_boxes(recipe()['task']['racket'])
    assert obj._vel_range_per_clip_t.shape == (2, 3, 2)
    assert obj._vel_start_range_per_clip_t[1, 2, 0] == 0
    torch.testing.assert_close(obj._vel_planner_range_per_clip_t[0, 2, 0], torch.tensor(.41))


@pytest.mark.parametrize('side,low,component', [('backhand', .55, 'start'), ('forehand', .50, 'planner')])
def test_previous_bank_only_envelope_reproduces_constructor_failure(side, low, component):
    racket = recipe()['task']['racket']
    racket['vel_range_per_clip'][side]['z'][0] = low
    with pytest.raises(ValueError, match=f'racket_vel_{component}_range_per_clip must be contained'):
        initialize_production_velocity_boxes(racket)


def test_parent_disjoint_splits_checksums_and_export_envelope():
    cfg = recipe()
    robust.validate_training_contract(cfg['task'], cfg['algo'])
    racket = cfg['task']['racket']
    sets = []
    for split in ('train', 'dev', 'test'):
        path = ROOT/f'assets/complete_hold_robust/{split}.npz'
        receipt = json.loads(path.with_suffix('.receipt.json').read_text())
        assert hashlib.sha256(path.read_bytes()).hexdigest() == receipt['bank_sha256']
        with np.load(path) as a:
            sets.append(set(a['parent_id']))
            for side, name in enumerate(('forehand', 'backhand')):
                mask = a['clip'] == side
                box = np.array([racket['vel_range_per_clip'][name][key] for key in 'xyz'])
                assert (a['target_velocity'][mask] >= box[:, 0]).all()
                assert (a['target_velocity'][mask] <= box[:, 1]).all()
                reach = np.array([.58, -.44 if side == 0 else -.09])
                np.testing.assert_allclose(a['target_position_offset'][mask, :2], np.broadcast_to(reach, (int(mask.sum()), 2)), atol=1e-6)
                if split == 'train':
                    assert mask.sum() >= racket['venue_tuple_bank_min_rows_per_side']
    assert not (sets[0]&sets[1] or sets[0]&sets[2] or sets[1]&sets[2])


def loader(cfg):
    source = PKG/'tasks/tracking/mdp/hope_commands.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_resolve_venue_tuple_bank_path', '_load_venue_tuple_bank')]
    import os
    namespace = dict(np=np, torch=torch, json=json, Path=Path, os=os)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), namespace)
    obj = NS(cfg=NS(**cfg), _venue_tuple_mix_mode='per_swing_bank_v2', device='cpu', _venue_tuple_bank_by_clip={})
    obj._resolve_venue_tuple_bank_path = lambda p: ROOT/p
    namespace['_load_venue_tuple_bank'](obj)
    return obj


def test_production_loader_and_sampler_keep_rows_intact_and_respect_travel():
    torch.manual_seed(17)
    obj = loader(recipe()['task']['racket'])
    for bank in obj._venue_tuple_bank_by_clip.values():
        previous = torch.rand(1024, 2)*torch.tensor([.20, 1.0])-torch.tensor([.10, .50])
        remaining = .35 + .5*torch.rand(1024)
        row, requested, fallback = robust.sample_rows(bank, previous, remaining,
            speed=.6, brake_s=.2, wide_probability=.5)
        distance = (bank['station_xy'][row]-previous).norm(dim=-1)
        assert (distance <= .6*(remaining-.2)+1e-6).all()
        assert .44 < requested.float().mean() < .56
        assert bank['target_velocity'][row].isfinite().all()
        assert fallback.dtype == torch.bool
    cfg = recipe()['task']['racket']
    cfg['venue_tuple_bank_path'] = 'assets/complete_hold_robust/test.npz'
    cfg['venue_tuple_bank_receipt_path'] = 'assets/complete_hold_robust/test.receipt.json'
    for key, path in [('venue_tuple_bank_sha256', cfg['venue_tuple_bank_path']),
                      ('venue_tuple_bank_receipt_sha256', cfg['venue_tuple_bank_receipt_path'])]:
        cfg[key] = hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    with pytest.raises(RuntimeError, match='train split'):
        loader(cfg)


def test_late_stress_boundary_and_time_fraction_are_not_packet_probability():
    assert robust.curriculum_scale(7000*64, 448000, 128000) == 0
    assert robust.curriculum_scale(8000*64, 448000, 128000) == .5
    assert robust.curriculum_scale(9000*64, 448000, 128000) == 1
    p = transport.outage_start_probability(.07, (.2, .2), .01, (1., 1.), .02)
    torch.manual_seed(29)
    remaining = torch.zeros(2048, dtype=torch.long)
    reset = torch.zeros(2048, dtype=torch.bool)
    active = 0
    total = 0
    for tick in range(1800):
        remaining, blocked, started = transport.advance_packet_outage(remaining, reset, dt=.02,
            scale=1., start_probability=p, short_range_s=(.2, .2), long_probability=.01, long_range_s=(1., 1.))
        if tick > 100:
            active += int(blocked.sum())
            total += len(blocked)
    assert abs(active/total-.07) < .003
    before = torch.random.get_rng_state()
    r, blocked, started = transport.advance_packet_outage(remaining, reset, dt=.02,
        scale=0., start_probability=p, short_range_s=(.2, .2), long_probability=.01, long_range_s=(1., 1.))
    assert not r.any() and not blocked.any() and not started.any()
    assert torch.equal(before, torch.random.get_rng_state())


def test_production_command_installs_intact_rows_using_previous_command_not_provisional_station():
    cfg = recipe()['task']['racket']
    obj = loader(cfg)
    source = PKG/'tasks/tracking/mdp/hope_commands.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_fixed_balanced_tuple_selection', '_apply_precomputed_venue_tuple', '_normal_from_target_velocity')]
    namespace = dict(torch=torch, curriculum_scale=robust.curriculum_scale, sample_robust_rows=robust.sample_rows)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), namespace)
    obj.num_envs = 8
    obj._env = NS(common_step_counter=9000*64)
    motion = NS(cfg=NS(rally_station_speed_mps=.6, rally_station_brake_s=.2))
    obj._motion = lambda: motion
    obj._fixed_balanced_tuple_selection = lambda *args: namespace['_fixed_balanced_tuple_selection'](obj, *args)
    obj._normal_from_target_velocity = namespace['_normal_from_target_velocity']
    obj._resample_is_wrap = True
    obj._venue_tuple_selected = torch.zeros(8, dtype=torch.bool)
    obj._question_wide_selected = torch.zeros(8, dtype=torch.bool)
    obj._venue_tuple_outcome_pending = torch.zeros(8, dtype=torch.bool)
    obj._venue_tuple_outcome_clip = torch.full((8,), -1, dtype=torch.long)
    obj.metrics = {key: torch.zeros(8) for key in (
        'venue_tuple_mix_effective', 'venue_tuple_accept_rate', 'venue_tuple_selected',
        'station_y_step_command', 'station_y_step_signed_command', 'station_y_step_class')}
    for key, width in (('base_target_pos_w', 2), ('racket_target_pos_w', 3), ('racket_target_vel_w', 3),
                       ('racket_target_normal_w', 3), ('_venue_planner_contact_normal_w', 3),
                       ('vb_vel_in_w', 3), ('vb_spin_in_w', 3), ('_venue_intended_landing_xy', 2),
                       ('_venue_outgoing_velocity_seed', 3)):
        setattr(obj, key, torch.zeros(8, width))
    obj.time_to_strike = torch.full((8,), .6)
    ids = torch.tensor([1, 3, 6, 7])
    clips = torch.tensor([0, 1, 0, 1])
    origins = torch.tensor([[10., 20., 0.]]).repeat(4, 1)
    obj._robust_previous_station_x = torch.full((8,), 10.)
    previous_y = torch.full((4,), 20.1)
    # A provisional draw is deliberately far away; selecting relative to this
    # would either fail or silently forgive debt instead of using the old command.
    obj.base_target_pos_w[ids] = origins[:, :2]+torch.tensor([100., -100.])
    namespace['_apply_precomputed_venue_tuple'](obj, ids, origins, clips, previous_y)
    previous = torch.stack((torch.full((4,), 10.), previous_y), dim=-1)
    assert ((obj.base_target_pos_w[ids]-previous).norm(dim=-1) <= .24+1e-5).all()
    expected_reach = torch.tensor([[.58, -.44], [.58, -.09], [.58, -.44], [.58, -.09]])
    torch.testing.assert_close(obj.racket_target_pos_w[ids, :2]-obj.base_target_pos_w[ids], expected_reach, atol=2e-6, rtol=0)
    assert obj._venue_tuple_outcome_pending[ids].all()
    assert torch.equal(obj._venue_tuple_outcome_clip[ids], clips)


def test_old_optimizer_resume_is_rejected_for_new_distribution():
    source = ROOT/'scripts/train.py'
    fn = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef)
              and n.name == '_validate_training_checkpoint')
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(source), 'exec'), namespace)
    state = dict(training_recipe_revision='continuous_rally_sim2real_v3', reward_revision='actual_complete_hold_v11',
                 policy_exploration_contract='same')
    check = namespace['_validate_training_checkpoint']
    with pytest.raises(ValueError, match='training distribution differs'):
        check({'hope_exact_resume_state': state}, NS(exploration_contract='same'),
              state['training_recipe_revision'], state['reward_revision'], 'complete_hold_robust_v1')
    state['training_distribution_revision'] = 'complete_hold_robust_v1'
    check({'hope_exact_resume_state': state}, NS(exploration_contract='same'),
          state['training_recipe_revision'], state['reward_revision'], 'complete_hold_robust_v1')
