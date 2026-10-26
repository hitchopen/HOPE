"""The standard stationary task must load the shipped 3star bank and its physics."""
import hashlib
import json
import os

import pytest
from hydra import compose, initialize_config_dir

from stationary_test_support import ROOT, load

train = load('stationary_defaults_train', ROOT/'scripts/train.py')


def config(task='HOPEPingPongStationaryRally'):
    with initialize_config_dir(config_dir=str(ROOT/'cfg'), version_base=None):
        return compose(config_name='train', overrides=[f'task={task}'])


def test_task_alone_selects_published_bank_and_pinned_physics(monkeypatch, tmp_path):
    cfg = config()
    monkeypatch.delenv('HOPE_BALL_PHYSICS_YAML', raising=False)
    monkeypatch.chdir(tmp_path)
    profile = train._configure_ball_physics(cfg)
    assert profile == ROOT.parents[1]/'configs/ball_physics_ppq004_training.yaml'
    assert os.environ['HOPE_BALL_PHYSICS_YAML'] == str(profile)
    assert hashlib.sha256(profile.read_bytes()).hexdigest() == cfg.task.ball_physics.sha256
    bank = ROOT/cfg.task.racket.venue_tuple_bank_path
    assert bank.parent.name == 'stationary_reach_v4_ppq004_generalization_20261022'
    assert cfg.task.racket.venue_tuple_bank_recipe_revision == 21
    assert hashlib.sha256(bank.read_bytes()).hexdigest() == cfg.task.racket.venue_tuple_bank_sha256
    receipt=json.loads((ROOT/cfg.task.racket.venue_tuple_bank_receipt_path).read_text())
    assert receipt['source']['physics_yaml_sha256'] == cfg.task.ball_physics.sha256


def test_stale_shell_profile_cannot_override_the_task_default(monkeypatch):
    monkeypatch.setenv('HOPE_BALL_PHYSICS_YAML', '/stale/ball_physics.yaml')
    selected = train._configure_ball_physics(config())
    assert os.environ['HOPE_BALL_PHYSICS_YAML'] == str(selected)


def test_missing_or_modified_profile_fails_before_launch(monkeypatch, tmp_path):
    cfg=config()
    monkeypatch.setenv('HOPE_BALL_PHYSICS_YAML','unchanged')
    cfg.task.ball_physics.path=str(tmp_path/'missing.yaml')
    with pytest.raises(FileNotFoundError,match='not found'):
        train._configure_ball_physics(cfg)
    profile=tmp_path/'changed.yaml';profile.write_text('changed: true\n')
    cfg.task.ball_physics.path=str(profile)
    with pytest.raises(ValueError,match='checksum mismatch'):
        train._configure_ball_physics(cfg)
    assert os.environ['HOPE_BALL_PHYSICS_YAML']=='unchanged'


def test_legacy_tasks_preserve_explicit_environment_profile(monkeypatch):
    monkeypatch.setenv('HOPE_BALL_PHYSICS_YAML','legacy.yaml')
    assert train._configure_ball_physics(config('HOPEPingPong')) is None
    assert os.environ['HOPE_BALL_PHYSICS_YAML']=='legacy.yaml'


def test_standalone_configs_match_the_source_training_recipe():
    from omegaconf import OmegaConf
    with initialize_config_dir(config_dir=str(ROOT/'cfg'), version_base=None):
        cfg = compose(config_name='train', overrides=[
            'task=HOPEPingPongStationaryRally', 'algo=ppo_balanced_hold'])
    expected = {
        'task': '3dcea64afd8a292e141c1d6dedd7b2d6ecdca237347dd45e9045d488aba652de',
        'algo': '1fa13da751dfafa2d00be10776d0e0733a1e254db8778a48a6f4a6b16e15d9a7',
    }
    # Captured from the fully resolved main_improve_2@9c4d6366a recipe,
    # not regenerated from the standalone files under test.
    for key, checksum in expected.items():
        data = OmegaConf.to_container(cfg[key], resolve=True)
        payload = json.dumps(data, sort_keys=True, separators=(',', ':')).encode()
        assert hashlib.sha256(payload).hexdigest() == checksum
