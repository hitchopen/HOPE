"""The PPQ004 launcher must bind the bank, physics and resume identity together."""

import ast
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import ppq004_training as prelaunch
from ball_physics_profile import load_profile, profile_source_hashes
spec = importlib.util.spec_from_file_location(
    "refitted_stationary_launcher", ROOT / "scripts/train_refitted_stationary.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def bundle(tmp_path):
    # A small payload isolates the launcher's identity checks; the production
    # bank loader has separate array/schema/coverage regression tests.
    bank = tmp_path / "train bank.npz"
    bank.write_bytes(b"test bank payload")
    physics = tmp_path / "physics.yaml"
    expanded = load_profile(ROOT.parents[1] / "configs/ball_physics_ppq004_training.yaml")
    for key in ("extends", "geometry_yaml", "planner_profile_yaml", "planner_settings_yaml"):
        expanded.pop(key, None)
    base = tmp_path / "base_physics.yaml"
    base.write_text(yaml.safe_dump(expanded))
    physics.write_text(yaml.safe_dump({"extends": base.name}))
    receipt = tmp_path / "train.receipt.json"
    receipt.write_text(json.dumps({
        "schema_version": 2, "split": "train", "bank_sha256": digest(bank),
        "source": {"physics_yaml_sha256": digest(physics),
                   "physics_source_hashes": profile_source_hashes(physics),
                   "return_model_sha256": digest(ROOT / "scripts/full_ball_model.py"),
                   "profile_loader_sha256": digest(ROOT / "scripts/ball_physics_profile.py")},
        "contract": {"distribution": "stationary_reach_v1", "recipe_revision": 22,
                     "qualification": launcher.QUALIFICATION, "return_model": "full_ppq004_first_bounce_v1",
                     "training_frame": expanded["training_frame"],
                     "first_bounce_center_height_m": expanded["ball"]["radius"]},
    }))
    path = tmp_path / "training_overrides.yaml"
    path.write_text(yaml.safe_dump({
        "environment": {"HOPE_BALL_PHYSICS_YAML": str(physics)},
        "task": {"racket": {
            "venue_tuple_bank_path": str(bank),
            "venue_tuple_bank_receipt_path": str(receipt),
            "venue_tuple_bank_sha256": digest(bank),
            "venue_tuple_bank_receipt_sha256": digest(receipt),
            "venue_tuple_bank_recipe_revision": 22,
        }},
    }))
    return path, bank, physics, receipt


def compose_launch(command):
    with initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base=None):
        cfg = compose(config_name="train", overrides=command[2:])
    prelaunch.configure_refitted_task(cfg, environ={})
    return OmegaConf.to_container(cfg, resolve=True)


def test_verified_bundle_composes_matching_physics_bank_and_resume_identity(bundle, monkeypatch):
    path, bank, physics, _ = bundle
    monkeypatch.setenv("HOPE_BALL_PHYSICS_YAML", "/wrong/old/venue.yaml")
    command, env = launcher.prepare_launch(path, ["num_envs=16", "seed=7", "run_name=ppq004"])
    cfg = compose_launch(command)
    assert env["HOPE_BALL_PHYSICS_YAML"] == str(physics)
    assert cfg["task"]["racket"]["venue_tuple_bank_path"] == str(bank)
    assert cfg["task"]["racket"]["venue_tuple_bank_recipe_revision"] == 22
    assert cfg["task"]["build_contract"]["training_distribution_revision"] == launcher.DISTRIBUTION
    assert cfg["num_envs"] == 16 and cfg["seed"] == 7
    assert cfg["algo"]["algorithm"]["settled_aux_enabled"] is False
    assert cfg["task"]["racket"]["venue_tuple_fixed_station"] is True


@pytest.mark.parametrize("field", ["bank", "physics", "receipt"])
def test_changed_artifact_is_rejected(bundle, field):
    path, bank, physics, receipt = bundle
    selected = dict(bank=bank, physics=physics, receipt=receipt)[field]
    selected.write_bytes(selected.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="checksum"):
        launcher.prepare_launch(path)


@pytest.mark.parametrize("block,key,value", [
    (None, "split", "test"),
    (None, "schema_version", 1),
    ("contract", "recipe_revision", 19),
    ("contract", "recipe_revision", 20),
    ("contract", "recipe_revision", 21),
    ("contract", "qualification", "unqualified"),
    ("contract", "qualification", "runtime_planner_return_v1"),
    ("contract", "return_model", "runtime_planner_v1"),
    ("contract", "first_bounce_center_height_m", 0.0),
    ("contract", "training_frame", {"near_x": .6, "surface_z": .76, "y_center": 0.0}),
    ("contract", "return_model", "legacy_spin_return"),
    ("source", "return_model_sha256", "0" * 64),
    ("source", "profile_loader_sha256", "0" * 64),
    ("contract", "distribution", "complete_hold_robust_v1"),
    ("source", "physics_yaml_sha256", "0" * 64),
])
def test_wrong_refit_or_split_is_rejected_even_with_matching_receipt_hash(bundle, block, key, value):
    path, _, _, receipt = bundle
    data = json.loads(receipt.read_text())
    (data if block is None else data[block])[key] = value
    receipt.write_text(json.dumps(data))
    data = yaml.safe_load(path.read_text())
    data["task"]["racket"]["venue_tuple_bank_receipt_sha256"] = digest(receipt)
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        launcher.prepare_launch(path)


def test_other_physics_model_is_rejected(bundle):
    path, _, physics, _ = bundle
    physics.write_text("provenance: {model: venue}\n")
    with pytest.raises(ValueError, match="PPQ004_UNIFIED"):
        launcher.prepare_launch(path)


@pytest.mark.parametrize("override", [
    "task=HOPEPingPong", "algo=ppo", "task.racket.venue_tuple_bank_path=/tmp/old.npz",
    "task.build_contract.training_distribution_revision=stationary_reach_v1",
    "+hydra.job.env_set.HOPE_BALL_PHYSICS_YAML=/tmp/old.yaml",
    "task={racket:{venue_tuple_bank_recipe_revision:19}}", "~task", "seed=1,2", "++seed=3",
])
def test_run_options_cannot_replace_recipe_or_physics_identity(bundle, override):
    with pytest.raises(ValueError, match="ordinary run options"):
        launcher.prepare_launch(bundle[0], [override])


def test_dry_run_never_starts_isaac_and_launch_uses_argv_and_training_cwd(bundle, monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(launcher.subprocess, "call", lambda *a, **kw: calls.append((a, kw)) or 0)
    assert launcher.main(["--overrides", str(bundle[0]), "--dry-run", "num_envs=1"]) == 0
    assert not calls
    assert "HOPE_BALL_PHYSICS_YAML=" in capsys.readouterr().out
    assert launcher.main(["--overrides", str(bundle[0]), "num_envs=1"]) == 0
    assert len(calls) == 1
    args, kwargs = calls[0]
    assert isinstance(args[0], list) and kwargs["cwd"] == ROOT
    assert kwargs["env"]["HOPE_BALL_PHYSICS_YAML"] == str(bundle[2])


def test_refitted_revision_keeps_stationary_validation_and_export(bundle):
    from test_complete_hold_robustness import robust
    from test_sim2real_export import fixture, module as exporter

    cfg = compose_launch(launcher.prepare_launch(bundle[0])[0])
    robust.validate_training_contract(cfg["task"], cfg["algo"])
    cfg["task"]["racket"]["base_target_x_range"] = [-.1, .1]
    with pytest.raises(ValueError, match="fixed session-anchor"):
        robust.validate_training_contract(cfg["task"], cfg["algo"])
    env, task, _, motion, racket = fixture()
    task["build_contract"].update(
        station_command_contract="fixed_session_anchor_v1",
        recovery_target_contract="fixed_session_anchor_v1",
        training_distribution_revision=launcher.DISTRIBUTION)
    motion.step_reference_mode = "station_error"
    racket.base_target_x_range = racket.base_target_y_range = (0., 0.)
    racket.station_y_step_range = None
    racket.venue_tuple_fixed_station = racket.venue_tuple_robust_sampling = True
    racket.venue_tuple_mix_mode = "per_swing_bank_v2"
    racket.venue_tuple_final_mix_prob = 1.0
    racket.venue_tuple_limit_reset_travel = False
    racket.racket_pos_range_per_clip = (((.44, .70), (-.68, -.24), (.8, 1.45)),
                                      ((.44, .70), (-.24, .28), (.8, 1.45)))
    metadata = exporter.sim2real_metadata(env, task)
    assert metadata["hitter_pure_training_distribution_revision"] == launcher.DISTRIBUTION


def test_old_physics_checkpoint_cannot_resume_optimizer_into_refitted_distribution():
    tree = ast.parse((ROOT / "scripts/train.py").read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_validate_training_checkpoint")
    namespace = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "train.py", "exec"), namespace)
    for old in ("stationary_reach_v1", "stationary_reach_ppq004_v1", "stationary_reach_ppq004_runtime_v1"):
        checkpoint = {"hope_exact_resume_state": {"training_distribution_revision": old}}
        with pytest.raises(ValueError, match="training distribution differs"):
            namespace[fn.name](checkpoint, SimpleNamespace(), "recipe", "reward", launcher.DISTRIBUTION)


def test_referenced_profile_change_is_rejected_without_wrapper_change(bundle):
    path, _, physics, _ = bundle
    before = digest(physics)
    base = physics.parent / "base_physics.yaml"
    base.write_text(base.read_text() + "\n# changed referenced profile\n")
    assert digest(physics) == before
    with pytest.raises(ValueError, match="referenced physics profile checksums"):
        launcher.prepare_launch(path)


def test_identical_profile_and_bank_bundle_can_move_without_changing_source_hashes(bundle, tmp_path):
    import shutil
    manifest, bank, physics, receipt = bundle
    moved = tmp_path / "moved_checkout"
    moved.mkdir()
    for original in (bank, physics, receipt, physics.parent / "base_physics.yaml"):
        shutil.copy2(original, moved / original.name)
    data = yaml.safe_load(manifest.read_text())
    data["environment"]["HOPE_BALL_PHYSICS_YAML"] = str(moved / physics.name)
    data["task"]["racket"]["venue_tuple_bank_path"] = str(moved / bank.name)
    data["task"]["racket"]["venue_tuple_bank_receipt_path"] = str(moved / receipt.name)
    relocated = moved / manifest.name
    relocated.write_text(yaml.safe_dump(data))
    # Original receipt still contains source paths from the previous location.
    launcher.prepare_launch(relocated)
    base = moved / "base_physics.yaml"
    base.write_text(base.read_text() + "\n# changed dependency\n")
    with pytest.raises(ValueError, match="referenced physics profile checksums"):
        launcher.prepare_launch(relocated)


def test_source_identity_does_not_collapse_duplicate_dependency_names():
    with pytest.raises(ValueError, match="unique YAML basenames"):
        prelaunch._source_identities({"/first/profile.yaml": "0" * 64, "/second/profile.yaml": "0" * 64})
