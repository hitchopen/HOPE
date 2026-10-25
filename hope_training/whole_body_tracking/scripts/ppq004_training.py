"""Pure PPQ004 manifest validation and Hydra prelaunch configuration.

Do not import whole_body_tracking here: its package initializer registers Isaac
classes. Both train and play call this helper before importing Isaac or ball physics.
"""

import hashlib
import json
import os
from pathlib import Path
import shlex

import yaml
from omegaconf import OmegaConf, open_dict


TRAINING_ROOT = Path(__file__).resolve().parents[1]
DISTRIBUTION = "stationary_reach_ppq004_v1"
QUALIFICATION = "ppq004_at_strike_retarget_and_fine_requalification_v1"
BANK_KEYS = {
    "venue_tuple_bank_path", "venue_tuple_bank_receipt_path",
    "venue_tuple_bank_sha256", "venue_tuple_bank_receipt_sha256",
    "venue_tuple_bank_recipe_revision",
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _path(value, base):
    if not isinstance(value, str) or not value:
        raise ValueError("refit paths must be nonempty strings")
    path = Path(value).expanduser()
    return (path if path.is_absolute() else base / path).resolve(strict=True)


def missing_bundle_message(manifest):
    command = ["hope_isaac_py", str(TRAINING_ROOT / "scripts/refit_stationary_reach_bank.py"),
               "--source", str(TRAINING_ROOT / "assets/stationary_reach_v4"),
               "--output", str(manifest.parent),
               "--physics", str(TRAINING_ROOT.parents[1] / "configs/ball_physics_ppq004_training.yaml")]
    return (f"PPQ004 refit bundle is missing: {manifest}\n"
            "Generate the validated recipe-20 bank before training; there is no venue-physics fallback.\n"
            + shlex.join(command))


def load_refit_bundle(overrides_path):
    """Validate the refitter bundle; return resolved bank fields, physics and manifest."""
    manifest = Path(overrides_path).expanduser().resolve()
    if not manifest.is_file():
        raise FileNotFoundError(missing_bundle_message(manifest))
    bundle = _mapping(yaml.safe_load(manifest.read_text()), "refit bundle")
    environment = _mapping(bundle.get("environment"), "environment")
    task = _mapping(bundle.get("task"), "task")
    bank = _mapping(task.get("racket"), "task.racket")
    if (set(bundle) != {"environment", "task"}
            or set(environment) != {"HOPE_BALL_PHYSICS_YAML"}
            or set(task) != {"racket"} or set(bank) != BANK_KEYS):
        raise ValueError("expected the unmodified refitter training_overrides.yaml fields")

    physics = _path(environment["HOPE_BALL_PHYSICS_YAML"], manifest.parent)
    bank_path = _path(bank["venue_tuple_bank_path"], manifest.parent)
    receipt_path = _path(bank["venue_tuple_bank_receipt_path"], manifest.parent)
    profile = _mapping(yaml.safe_load(physics.read_text()), "physics profile")
    if _mapping(profile.get("provenance"), "physics provenance").get("model") != "PPQ004_UNIFIED":
        raise ValueError("physics profile must identify PPQ004_UNIFIED")
    receipt = _mapping(json.loads(receipt_path.read_text()), "bank receipt")
    contract = _mapping(receipt.get("contract"), "receipt contract")
    source = _mapping(receipt.get("source"), "receipt source")
    if (receipt.get("schema_version") != 2 or receipt.get("split") != "train"
            or contract.get("distribution") != "stationary_reach_v1"
            or contract.get("recipe_revision") != 20
            or bank["venue_tuple_bank_recipe_revision"] != 20
            or contract.get("qualification") != QUALIFICATION):
        raise ValueError("expected a qualified recipe-20 stationary train split")
    if sha256(physics) != source.get("physics_yaml_sha256"):
        raise ValueError("physics profile checksum differs from the bank's refit receipt")
    bank_digest = sha256(bank_path)
    if bank_digest != bank["venue_tuple_bank_sha256"] or bank_digest != receipt.get("bank_sha256"):
        raise ValueError("bank checksum differs from the refit bundle or receipt")
    if sha256(receipt_path) != bank["venue_tuple_bank_receipt_sha256"]:
        raise ValueError("bank receipt checksum differs from the refit bundle")

    resolved = dict(bank, venue_tuple_bank_path=str(bank_path),
                    venue_tuple_bank_receipt_path=str(receipt_path))
    return resolved, physics, manifest


def configure_refitted_task(cfg, *, environ=None):
    """Bind a selected PPQ004 task before simulator imports and config snapshots.

    Manifest paths in task YAML are relative to the training package, regardless
    of launch cwd. Other explicit task recipes retain their existing behavior.
    """
    revision = cfg.task.get("build_contract", {}).get("training_distribution_revision")
    if "ppq004_refit_manifest" not in cfg.task:
        if revision == DISTRIBUTION:
            raise ValueError("PPQ004 training requires task=HOPEPingPongStationaryPPQ004 and its refit manifest")
        return False
    if revision != DISTRIBUTION:
        raise ValueError("PPQ004 manifest requires the stationary_reach_ppq004_v1 training identity")
    selected = cfg.task.get("ppq004_refit_manifest")
    if not isinstance(selected, str) or not selected:
        raise ValueError("task.ppq004_refit_manifest must name the validated recipe-20 refit bundle")
    path = Path(selected).expanduser()
    manifest = path if path.is_absolute() else TRAINING_ROOT / path
    bank, physics, manifest = load_refit_bundle(manifest)
    with open_dict(cfg.task):
        for key, value in bank.items():
            OmegaConf.update(cfg.task, f"racket.{key}", value, merge=False)
        cfg.task.ppq004_refit_manifest = str(manifest)
        cfg.task.ppq004_physics_yaml = str(physics)
        cfg.task.ppq004_physics_sha256 = sha256(physics)
    target_environment = os.environ if environ is None else environ
    target_environment["HOPE_BALL_PHYSICS_YAML"] = str(physics)
    return True
