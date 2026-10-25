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
    physics.write_text(yaml.safe_dump({"provenance": {"model": "PPQ004_UNIFIED"}}))
    receipt = tmp_path / "train.receipt.json"
    receipt.write_text(json.dumps({
        "schema_version": 2, "split": "train", "bank_sha256": digest(bank),
        "source": {"physics_yaml_sha256": digest(physics)},
        "contract": {"distribution": "stationary_reach_v1", "recipe_revision": 20,
                     "qualification": launcher.QUALIFICATION},
    }))
    path = tmp_path / "training_overrides.yaml"
    path.write_text(yaml.safe_dump({
        "environment": {"HOPE_BALL_PHYSICS_YAML": str(physics)},
        "task": {"racket": {
            "venue_tuple_bank_path": str(bank),
            "venue_tuple_bank_receipt_path": str(receipt),
            "venue_tuple_bank_sha256": digest(bank),
            "venue_tuple_bank_receipt_sha256": digest(receipt),
            "venue_tuple_bank_recipe_revision": 20,
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
    assert cfg["task"]["racket"]["venue_tuple_bank_recipe_revision"] == 20
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
    ("contract", "recipe_revision", 21),
    ("contract", "qualification", "unqualified"),
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
    checkpoint = {"hope_exact_resume_state": {"training_distribution_revision": "stationary_reach_v1"}}
    with pytest.raises(ValueError, match="training distribution differs"):
        namespace[fn.name](checkpoint, SimpleNamespace(), "recipe", "reward", launcher.DISTRIBUTION)
