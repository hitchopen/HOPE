"""PPQ004 defaults bind verified local artifacts before any simulator import."""

import ast
import builtins
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from test_refitted_stationary_launcher import ROOT, bundle, digest
import ppq004_training as prelaunch


def config(overrides=(), name="train"):
    with initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base=None):
        return compose(config_name=name, overrides=list(overrides))


def test_default_training_selects_recipe23_sweet_spot_and_scratch_run():
    cfg = config()
    assert cfg.task.build_contract.training_distribution_revision == prelaunch.SWEET_SPOT_DISTRIBUTION
    assert cfg.task.racket.venue_tuple_bank_recipe_revision == 23
    assert cfg.task.ppq004_refit_manifest == "outputs/stationary_ppq004_sweet_spot/training_overrides.yaml"
    assert cfg.task.sweet_spot.reward_enabled is True
    assert cfg.task.racket.venue_tuple_bank_path is None
    assert cfg.algo.algorithm.settled_aux_enabled is False
    assert (cfg.num_envs, cfg.max_iterations, cfg.seed) == (6144, 30000, 0)
    assert cfg.checkpoint_path is None and cfg.warm_start_actor_checkpoint is None


def test_effective_default_records_matching_bank_and_physics_in_saved_configuration(bundle, tmp_path):
    manifest, bank, physics, receipt = bundle
    cfg = config(["task=HOPEPingPongStationaryPPQ004General", f"task.ppq004_refit_manifest={json.dumps(str(manifest))}"])
    environment = {"HOPE_BALL_PHYSICS_YAML": "/old/venue.yaml"}
    assert prelaunch.configure_refitted_task(cfg, environ=environment)
    assert environment["HOPE_BALL_PHYSICS_YAML"] == str(physics)
    saved = tmp_path / "task_resolved.yaml"
    OmegaConf.save(cfg, saved)
    restored = OmegaConf.load(saved)
    assert restored.task.racket.venue_tuple_bank_path == str(bank)
    assert restored.task.racket.venue_tuple_bank_sha256 == digest(bank)
    assert restored.task.racket.venue_tuple_bank_receipt_sha256 == digest(receipt)
    profile = prelaunch.load_profile(physics)
    assert restored.task.racket.vb_table_near_x == profile["training_frame"]["near_x"]
    assert restored.task.racket.vb_table_surface_z == profile["training_frame"]["surface_z"]
    assert restored.task.ppq004_physics_yaml == str(physics)
    assert restored.task.ppq004_physics_sha256 == digest(physics)
    assert dict(restored.task.ppq004_physics_source_hashes) == prelaunch.profile_source_hashes(physics)
    assert restored.task.build_contract.training_distribution_revision == prelaunch.DISTRIBUTION


def test_manifest_path_is_training_package_relative_independent_of_cwd(bundle, monkeypatch, tmp_path):
    monkeypatch.setattr(prelaunch, "TRAINING_ROOT", bundle[0].parent)
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    cfg = config(["task=HOPEPingPongStationaryPPQ004General", "task.ppq004_refit_manifest=training_overrides.yaml"])
    assert prelaunch.configure_refitted_task(cfg, environ={})
    assert cfg.task.ppq004_refit_manifest == str(bundle[0])


@pytest.mark.parametrize("task,algo", [
    ("HOPEPingPong", "ppo"),
    ("HOPEPingPongCompleteHold", "ppo_balanced_hold"),
    ("HOPEPingPongStationaryRally", "ppo_balanced_hold"),
])
def test_explicit_legacy_recipes_do_not_read_manifest_or_change_physics(task, algo):
    cfg = config([f"task={task}", f"algo={algo}"])
    original = OmegaConf.to_yaml(cfg)
    environment = {"HOPE_BALL_PHYSICS_YAML": "/explicit/venue.yaml"}
    assert prelaunch.configure_refitted_task(cfg, environ=environment) is False
    assert OmegaConf.to_yaml(cfg) == original
    assert environment == {"HOPE_BALL_PHYSICS_YAML": "/explicit/venue.yaml"}


@pytest.mark.parametrize("override", [
    "task.ppq004_refit_manifest=null",
    "~task.ppq004_refit_manifest",
    "task.build_contract.training_distribution_revision=stationary_reach_v1",
])
def test_ppq004_identity_cannot_silently_drop_manifest_binding(override):
    with pytest.raises(ValueError, match="PPQ004|ppq004_refit_manifest"):
        prelaunch.configure_refitted_task(config([override]), environ={})


def entrypoint_main(filename):
    tree = ast.parse((ROOT / "scripts" / filename).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    node.decorator_list = []
    namespace = {"OmegaConf": OmegaConf, "sys": SimpleNamespace(argv=[filename])}
    exec(compile(ast.Module(body=[node], type_ignores=[]), filename, "exec"), namespace)
    return namespace["main"]


@pytest.mark.parametrize("filename", ["train.py", "play.py"])
def test_missing_bundle_fails_before_isaac_import_with_regeneration_command(filename, tmp_path, monkeypatch):
    missing = tmp_path / "refit" / "training_overrides.yaml"
    cfg = config([f"task.ppq004_refit_manifest={json.dumps(str(missing))}"])
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        assert not name.startswith("isaac"), "simulator imported before checking the refit bundle"
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    with pytest.raises(FileNotFoundError, match="refit_stationary_reach_bank.py") as exc:
        entrypoint_main(filename)(cfg)
    assert "recipe-23" in str(exc.value)
    assert "--landing-objective" in str(exc.value)
    assert "assets/stationary_reach_v4_ppq004_generalization_20261022" in str(exc.value)
    assert str(missing.parent) in str(exc.value)


@pytest.mark.parametrize("filename", ["train.py", "play.py"])
def test_matching_physics_is_set_before_app_launcher(filename, bundle, monkeypatch):
    cfg = config(["task=HOPEPingPongStationaryPPQ004General", f"task.ppq004_refit_manifest={json.dumps(str(bundle[0]))}"])
    original_import = builtins.__import__
    monkeypatch.setenv("HOPE_BALL_PHYSICS_YAML", "/old/venue.yaml")

    def app_launcher(**kwargs):
        import os
        assert os.environ["HOPE_BALL_PHYSICS_YAML"] == str(bundle[2])
        assert cfg.task.racket.venue_tuple_bank_path == str(bundle[1])
        raise RuntimeError("verified before simulated AppLauncher")

    def fake_import(name, *args, **kwargs):
        if name == "isaaclab.app":
            return SimpleNamespace(AppLauncher=app_launcher)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    with pytest.raises(RuntimeError, match="verified before simulated AppLauncher"):
        entrypoint_main(filename)(cfg)


def test_play_defaults_still_use_baseline_without_manifest():
    cfg = config(name="play")
    assert cfg.task.build_contract.get("training_distribution_revision") != prelaunch.DISTRIBUTION
    assert "ppq004_refit_manifest" not in cfg.task
    assert cfg.algo.algorithm.settled_aux_enabled is True
    assert prelaunch.configure_refitted_task(cfg, environ={}) is False


@pytest.mark.parametrize("task,algo", [
    ("HOPEPingPong", "ppo"), ("HOPEPingPongCompleteHold", "ppo_balanced_hold"),
    ("HOPEPingPongStationaryRally", "ppo_balanced_hold"),
])
def test_legacy_recipe_without_explicit_profile_binds_its_venue_physics(task, algo):
    cfg = config([f"task={task}", f"algo={algo}"])
    environment = {}
    assert prelaunch.configure_refitted_task(cfg, environ=environment) is False
    assert environment["HOPE_BALL_PHYSICS_YAML"] == str(ROOT.parents[1] / "configs/ball_physics_venue.yaml")


def test_legacy_recipe_preserves_explicit_old_config_environment():
    cfg = config(["task=HOPEPingPong", "algo=ppo"])
    environment = {"HOPE_BALL_PHYSICS_CONFIG": "/explicit/profile.yaml"}
    assert prelaunch.configure_refitted_task(cfg, environ=environment) is False
    assert environment == {"HOPE_BALL_PHYSICS_CONFIG": "/explicit/profile.yaml"}
