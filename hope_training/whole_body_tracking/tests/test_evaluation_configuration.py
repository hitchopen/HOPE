"""Checkpoint evaluation restores the saved MDP and policy architecture before Isaac."""
import builtins
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from omegaconf import OmegaConf

from test_ppq004_training_default import config
from test_refitted_stationary_launcher import ROOT, bundle
from test_command_overrides import _env
import ppq004_training as prelaunch


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


evaluate = load('hope_evaluate_configuration', ROOT / 'scripts/evaluate.py')


def save_run(tmp_path, cfg):
    checkpoint = tmp_path / 'model_100.pt'
    checkpoint.touch()
    params = tmp_path / 'params'
    params.mkdir(exist_ok=True)
    OmegaConf.save(cfg, params / 'task_resolved.yaml')
    return checkpoint


def test_saved_stationary_recipe_keeps_hold_reward_bank_and_bounded_policy(tmp_path):
    cfg = config(['task=HOPEPingPongStationaryRally', 'algo=ppo_balanced_hold'])
    checkpoint = save_run(tmp_path, cfg)
    restored = evaluate._load_evaluation_config(checkpoint)
    assert OmegaConf.to_container(restored.task) == OmegaConf.to_container(cfg.task)
    assert OmegaConf.to_container(restored.algo) == OmegaConf.to_container(cfg.algo)
    assert restored.task.racket.venue_tuple_fixed_station
    assert restored.task.motion.hold_reference_path == cfg.task.motion.hold_reference_path
    assert not restored.algo.algorithm.settled_aux_enabled
    assert restored.algo.policy.min_noise_std == cfg.algo.policy.min_noise_std
    env = {'HOPE_BALL_PHYSICS_YAML': '/unrelated/default.yaml'}
    evaluate._bind_saved_physics(restored, environ=env)
    assert Path(env['HOPE_BALL_PHYSICS_YAML']).name == 'ball_physics_venue.yaml'


def test_missing_or_incomplete_metadata_never_falls_back_to_baseline(tmp_path, monkeypatch):
    checkpoint = tmp_path / 'model.pt'
    checkpoint.touch()
    monkeypatch.setattr(evaluate, 'parse_args', lambda: NS(
        checkpoint=str(checkpoint), resolved_config=None, task=None))
    real_import = builtins.__import__

    def guard(name, *args, **kwargs):
        assert not name.startswith('isaac'), 'metadata must be checked before simulator imports'
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guard)
    with pytest.raises(FileNotFoundError, match='--resolved-config'):
        evaluate.main()
    supplied = tmp_path / 'explicit.yaml'
    supplied.write_text('task: {gym_task: baseline}\n')
    with pytest.raises(ValueError, match='task and algo'):
        evaluate._load_evaluation_config(checkpoint, supplied)
    cfg = config(['task=HOPEPingPong', 'algo=ppo'])
    OmegaConf.save(cfg, supplied)
    restored = evaluate._load_evaluation_config(checkpoint, supplied)
    assert restored.algo.algorithm.settled_aux_enabled
    with pytest.raises(ValueError, match='differs from saved task'):
        evaluate._load_evaluation_config(checkpoint, supplied, task_id='another-task')


def test_full_checkpoint_cannot_silently_take_a_different_bank(bundle, tmp_path):
    cfg = config(['task=HOPEPingPongStationaryPPQ004General', f'task.ppq004_refit_manifest={json.dumps(str(bundle[0]))}'])
    prelaunch.configure_refitted_task(cfg, environ={})
    checkpoint = save_run(tmp_path, cfg)
    restored = evaluate._load_evaluation_config(checkpoint)
    environment = {}
    evaluate._bind_saved_physics(restored, environ=environment)
    assert environment['HOPE_BALL_PHYSICS_YAML'] == str(bundle[2])
    restored.task.racket.venue_tuple_bank_sha256 = '0' * 64
    with pytest.raises(ValueError, match='bank identity'):
        evaluate._bind_saved_physics(restored, environ={})
    restored = evaluate._load_evaluation_config(checkpoint)
    restored.task.ppq004_physics_sha256 = '0' * 64
    with pytest.raises(ValueError, match='physics profile'):
        evaluate._bind_saved_physics(restored, environ={})


def test_evaluation_applies_actual_trainer_overrides_and_saved_motion(tmp_path, monkeypatch):
    # Load the real pure override helpers without the Isaac-registering package initializer.
    utilities = ROOT / 'source/whole_body_tracking/whole_body_tracking/utils'
    for name in ('task_command_overrides', 'task_sim2real_overrides', 'task_reward_overrides'):
        module = load('evaluation_' + name, utilities / f'{name}.py')
        monkeypatch.setitem(sys.modules, 'whole_body_tracking.utils.' + name, module)
    trainer = load('evaluation_train_entrypoint', ROOT / 'scripts/train.py')
    monkeypatch.setitem(sys.modules, 'train', trainer)
    clip = tmp_path / 'saved_forehand.npz'
    clip.touch()
    cfg = OmegaConf.create({
        'seed': 42, 'motion_file': None, 'motion_file_2': None,
        'task': {'motion_file': str(clip), 'env': {'episode_length_s': 73},
                 'motion': {'hold_steps_range': [7, 11]},
                 'racket': {'venue_tuple_fixed_station': True,
                            'venue_tuple_bank_recipe_revision': 22,
                            'venue_tuple_bank_path': '/saved/train.npz'},
                 'rewards': {'full_body_hold_weight': 1.7},
                 'domain_rand': {'link_mass_range': None},
                 'observation_noise': {'enabled': False}}})
    env = _env()
    env.episode_length_s = 10
    env.rewards = NS(full_body_hold=NS(weight=0))
    env.events = NS(randomize_link_mass=NS(params={}))
    env.observations = NS(policy=NS(enable_corruption=True))
    evaluate._apply_evaluation_recipe(env, cfg)
    assert env.episode_length_s == 73 and env.seed == 42
    assert env.commands.motion.hold_steps_range == (7, 11)
    assert env.commands.motion.motion_file == str(clip)
    assert env.commands.racket_target.venue_tuple_fixed_station
    assert env.commands.racket_target.venue_tuple_bank_recipe_revision == 22
    assert env.commands.racket_target.venue_tuple_bank_path == '/saved/train.npz'
    assert env.rewards.full_body_hold.weight == 1.7
    assert env.events.randomize_link_mass is None
    assert not env.observations.policy.enable_corruption


def test_legacy_bank_builders_ignore_new_global_ppq004_default(monkeypatch):
    legacy = load('evaluation_legacy_bank_builder', ROOT / 'scripts/build_complete_hold_bank.py')
    stationary = load('evaluation_stationary_bank_builder', ROOT / 'scripts/build_stationary_reach_bank.py')
    monkeypatch.setenv('HOPE_BALL_PHYSICS_YAML', str(ROOT.parents[1] / 'configs/ball_physics_ppq004_training.yaml'))
    vb, _ = legacy.load_physics()
    params = legacy.legacy_physics_params(vb)
    assert params.return_model == 'legacy_venue'
    assert Path(params.source_path).name == 'ball_physics_venue.yaml'
    # The stationary builder delegates both model loading and profile selection to this helper.
    inherited, _ = stationary.load_legacy_builder()
    other = inherited.legacy_physics_params(vb)
    assert other == params


def test_evaluation_rejects_changed_reward_or_objective_checkpoint_identity():
    expected = {'objective_sha256': 'a' * 64, 'reward': {'reward_enabled': True}}
    evaluate._verify_placement_checkpoint({'hope_exact_resume_state': {
        'placement_reward_identity': expected}}, expected)
    evaluate._verify_placement_checkpoint({}, None)  # historical, non-placement policy
    with pytest.raises(ValueError, match='placement reward/objective'):
        evaluate._verify_placement_checkpoint({}, expected)
    with pytest.raises(ValueError, match='placement reward/objective'):
        evaluate._verify_placement_checkpoint({'hope_exact_resume_state': {
            'placement_reward_identity': dict(expected, reward={'reward_enabled': False})}}, expected)
