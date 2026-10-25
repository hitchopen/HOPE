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

from ball_physics_profile import load_profile, profile_source_hashes


TRAINING_ROOT = Path(__file__).resolve().parents[1]
DISTRIBUTION = "stationary_reach_ppq004_full_v1"
QUALIFICATION = "full_ppq004_first_bounce_v1"
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


def _source_identities(hashes):
    """Compare dependency content across checkout moves without weakening hashes.

    The selected profile has uniquely named YAML dependencies. Reject duplicate
    basenames instead of conflating two different files after relocation.
    """
    hashes = _mapping(hashes, "physics source hashes")
    identities = {}
    for path, digest in hashes.items():
        if (not isinstance(path, str) or not isinstance(digest, str)
                or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)):
            raise ValueError("physics source hashes must contain paths and SHA-256 digests")
        name = Path(path).name
        if not name or name in identities:
            raise ValueError("physics source identities require unique YAML basenames")
        identities[name] = digest
    return identities


def missing_bundle_message(manifest):
    command = ["hope_isaac_py", str(TRAINING_ROOT / "scripts/refit_stationary_reach_bank.py"),
               "--source", str(TRAINING_ROOT / "assets/stationary_reach_v4_ppq004_generalization_20261022"),
               "--output", str(manifest.parent),
               "--physics", str(TRAINING_ROOT.parents[1] / "configs/ball_physics_ppq004_training.yaml")]
    return (f"PPQ004 refit bundle is missing: {manifest}\n"
            "Generate the validated recipe-22 bank before training; there is no venue-physics fallback.\n"
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
    profile = load_profile(physics)
    if _mapping(profile.get("provenance"), "physics provenance").get("model") != "PPQ004_UNIFIED":
        raise ValueError("physics profile must identify PPQ004_UNIFIED")
    receipt = _mapping(json.loads(receipt_path.read_text()), "bank receipt")
    contract = _mapping(receipt.get("contract"), "receipt contract")
    source = _mapping(receipt.get("source"), "receipt source")
    if (receipt.get("schema_version") != 2 or receipt.get("split") != "train"
            or contract.get("distribution") != "stationary_reach_v1"
            or contract.get("recipe_revision") != 22
            or bank["venue_tuple_bank_recipe_revision"] != 22
            or contract.get("qualification") != QUALIFICATION
            or contract.get("return_model") != "full_ppq004_first_bounce_v1"):
        raise ValueError("expected a full-PPQ004 first-bounce recipe-22 stationary train split")
    if (contract.get("training_frame") != profile.get("training_frame")
            or contract.get("first_bounce_center_height_m") != profile.get("ball", {}).get("radius")):
        raise ValueError("bank first-bounce plane or table placement differs from the physics profile")
    if sha256(physics) != source.get("physics_yaml_sha256"):
        raise ValueError("physics profile checksum differs from the bank's refit receipt")
    if _source_identities(profile_source_hashes(physics)) != _source_identities(source.get("physics_source_hashes")):
        raise ValueError("referenced physics profile checksums differ from the bank receipt")
    if profile.get("return_model", {}).get("id") != "full_ppq004_first_bounce_v1":
        raise ValueError("physics profile must select full_ppq004_first_bounce_v1")
    if source.get("return_model_sha256") != sha256(Path(__file__).with_name("full_ball_model.py")):
        raise ValueError("return-model implementation checksum differs from the bank receipt")
    if source.get("profile_loader_sha256") != sha256(Path(__file__).with_name("ball_physics_profile.py")):
        raise ValueError("physics-profile loader checksum differs from the bank receipt")
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
    target_environment = os.environ if environ is None else environ
    revision = cfg.task.get("build_contract", {}).get("training_distribution_revision")
    if "ppq004_refit_manifest" not in cfg.task:
        if revision in (DISTRIBUTION, "stationary_reach_ppq004_v1", "stationary_reach_ppq004_runtime_v1"):
            raise ValueError("PPQ004 training requires task=HOPEPingPongStationaryPPQ004 and its refit manifest")
        if not (target_environment.get("HOPE_BALL_PHYSICS_YAML")
                or target_environment.get("HOPE_BALL_PHYSICS_CONFIG")):
            target_environment["HOPE_BALL_PHYSICS_YAML"] = str(
                TRAINING_ROOT.parents[1] / "configs/ball_physics_venue.yaml")
        return False
    if revision != DISTRIBUTION:
        raise ValueError("PPQ004 manifest requires the stationary_reach_ppq004_full_v1 training identity")
    selected = cfg.task.get("ppq004_refit_manifest")
    if not isinstance(selected, str) or not selected:
        raise ValueError("task.ppq004_refit_manifest must name the validated recipe-22 refit bundle")
    path = Path(selected).expanduser()
    manifest = path if path.is_absolute() else TRAINING_ROOT / path
    bank, physics, manifest = load_refit_bundle(manifest)
    frame = load_profile(physics)["training_frame"]
    if frame["y_center"] != 0.0:
        raise ValueError("fixed-station training requires table y_center=0")
    with open_dict(cfg.task):
        for key, value in bank.items():
            OmegaConf.update(cfg.task, f"racket.{key}", value, merge=False)
        cfg.task.racket.vb_table_near_x = frame["near_x"]
        cfg.task.racket.vb_table_surface_z = frame["surface_z"]
        cfg.task.ppq004_refit_manifest = str(manifest)
        cfg.task.ppq004_physics_yaml = str(physics)
        cfg.task.ppq004_physics_sha256 = sha256(physics)
        cfg.task.ppq004_physics_source_hashes = profile_source_hashes(physics)
    target_environment["HOPE_BALL_PHYSICS_YAML"] = str(physics)
    return True
