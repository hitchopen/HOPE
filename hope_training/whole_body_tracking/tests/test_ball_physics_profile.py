"""The PPQ004 default references runtime YAML and binds every input file."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('hope_profile_test', ROOT / 'scripts/ball_physics_profile.py')
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)
DEFAULT = ROOT.parents[1] / profile.DEFAULT_PROFILE


def test_default_uses_runtime_profile_and_settings_without_duplicate_fits(monkeypatch):
    monkeypatch.delenv('HOPE_BALL_PHYSICS_YAML', raising=False)
    monkeypatch.delenv('HOPE_BALL_PHYSICS_CONFIG', raising=False)
    raw = yaml.safe_load(DEFAULT.read_text())
    assert 'flight' not in raw and 'contact' not in raw
    loaded = profile.load_profile()
    planner = yaml.safe_load((DEFAULT.parent / raw['planner_profile_yaml']).read_text())['hope_planner']['ros__parameters']
    settings = yaml.safe_load((DEFAULT.parent / raw['planner_settings_yaml']).read_text())['hope_planner']['ros__parameters']
    settings.update(yaml.safe_load((DEFAULT.parent / raw['return_physics_yaml']).read_text())['hope_planner']['ros__parameters'])
    assert loaded['flight']['k_d'] == loaded['drag']['k'] == planner['drag_k']
    assert loaded['flight']['k_m'] == planner['magnus_k']
    assert loaded['contact']['table']['e_eff'] == planner['restitution_v']
    assert loaded['return_model']['integrate_dt_s'] == settings['return_dt_integrate']
    assert loaded['return_model']['flight_time_s'] == settings['delta_t_flight']
    assert loaded['return_model']['restitution_racket'] == settings['restitution_racket']
    assert loaded['drag']['velocity_clip'] is None
    assert len(profile.profile_source_hashes()) == 6


def test_changed_reference_changes_effective_coefficients_and_provenance(tmp_path):
    wrapper = yaml.safe_load(DEFAULT.read_text())
    for key in ('extends', 'geometry_yaml', 'planner_profile_yaml', 'planner_settings_yaml', 'return_physics_yaml'):
        wrapper[key] = str((DEFAULT.parent / wrapper[key]).resolve())
    runtime = yaml.safe_load(Path(wrapper['planner_profile_yaml']).read_text())
    selected = tmp_path / 'runtime.yaml'
    selected.write_text(yaml.safe_dump(runtime))
    wrapper['planner_profile_yaml'] = str(selected)
    config = tmp_path / 'profile.yaml'
    config.write_text(yaml.safe_dump(wrapper))
    before = profile.profile_source_hashes(config)
    for node in ('hope_planner', 'hope_ball_flight_packetizer'):
        runtime[node]['ros__parameters']['drag_k'] *= 1.25
    selected.write_text(yaml.safe_dump(runtime))
    loaded = profile.load_profile(config)
    assert loaded['flight']['k_d'] == runtime['hope_planner']['ros__parameters']['drag_k']
    assert profile.profile_source_hashes(config)[str(selected)] != before[str(selected)]
    assert profile.profile_source_hashes(config)[str(config)] == before[str(config)]


def test_explicit_missing_profile_never_falls_back_to_constants(tmp_path, monkeypatch):
    monkeypatch.setenv('HOPE_BALL_PHYSICS_YAML', str(tmp_path / 'absent.yaml'))
    with pytest.raises(FileNotFoundError):
        profile.load_profile()


def test_cyclic_references_fail_clearly(tmp_path):
    path = tmp_path / 'cycle.yaml'
    path.write_text('extends: cycle.yaml\n')
    with pytest.raises(ValueError, match='Cyclic'):
        profile.load_profile(path)
