"""Pure PPQ004 manifest validation and Hydra prelaunch configuration.

Do not import whole_body_tracking here: its package initializer registers Isaac
classes. Both train and play call this helper before importing Isaac or ball physics.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import shlex

import yaml
from omegaconf import OmegaConf, open_dict

from ball_physics_profile import load_profile, profile_source_hashes


TRAINING_ROOT = Path(__file__).resolve().parents[1]
DISTRIBUTION = "stationary_reach_ppq004_full_v1"
SWEET_SPOT_DISTRIBUTION = "stationary_reach_ppq004_sweet_spot_v1"
SWEET_SPOT_REWARD = "actual_complete_hold_v11_stationary_sweet_spot_v1"
RECIPE_BY_DISTRIBUTION = {DISTRIBUTION: 22, SWEET_SPOT_DISTRIBUTION: 23}
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


def missing_bundle_message(manifest, *, recipe=23):
    command = ["hope_isaac_py", str(TRAINING_ROOT / "scripts/refit_stationary_reach_bank.py"),
               "--source", str(TRAINING_ROOT / "assets/stationary_reach_v4_ppq004_generalization_20261022"),
               "--output", str(manifest.parent),
               "--physics", str(TRAINING_ROOT.parents[1] / "configs/ball_physics_ppq004_training.yaml")]
    if recipe == 23:
        command.extend(["--landing-objective", str(TRAINING_ROOT.parents[1] / "configs/return_placement_sweet_spot.yaml")])
    return (f"PPQ004 refit bundle is missing: {manifest}\n"
            f"Generate the validated recipe-{recipe} bank before training; there is no venue-physics fallback.\n"
            + shlex.join(command))


def load_refit_bundle(overrides_path, *, expected_recipe=None):
    """Validate the refitter bundle; return resolved bank fields, physics and manifest."""
    manifest = Path(overrides_path).expanduser().resolve()
    if not manifest.is_file():
        raise FileNotFoundError(missing_bundle_message(manifest, recipe=expected_recipe or 23))
    bundle = _mapping(yaml.safe_load(manifest.read_text()), "refit bundle")
    environment = _mapping(bundle.get("environment"), "environment")
    task = _mapping(bundle.get("task"), "task")
    bank = _mapping(task.get("racket"), "task.racket")
    if (set(bundle) != {"environment", "task"}
            or set(environment) != {"HOPE_BALL_PHYSICS_YAML"}
            or set(task) not in ({"racket"}, {"racket", "landing_objective"}) or set(bank) != BANK_KEYS):
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
    recipe = bank["venue_tuple_bank_recipe_revision"]
    if (receipt.get("schema_version") != 2 or receipt.get("split") != "train"
            or contract.get("distribution") != "stationary_reach_v1"
            or recipe not in (22, 23) or contract.get("recipe_revision") != recipe
            or (expected_recipe is not None and recipe != expected_recipe)
            or contract.get("qualification") != QUALIFICATION
            or contract.get("return_model") != "full_ppq004_first_bounce_v1"):
        raise ValueError("expected a matching full-PPQ004 first-bounce recipe-22/23 stationary train split")
    if recipe == 23:
        objective = _validate_landing_objective(task.get("landing_objective"), profile, manifest.parent)
        if (contract.get("training_distribution_revision") != SWEET_SPOT_DISTRIBUTION
                or contract.get("landing_objective") != objective["descriptor"]
                or contract.get("landing_objective_sha256") != objective["sha256"]
                or source.get("landing_objective_yaml_sha256") != objective["sha256"]):
            raise ValueError("landing objective identity differs between manifest and receipt")
        from sweet_spot_profile import objective_in_training_frame
        target = objective_in_training_frame(objective["descriptor"], profile)["target_xy"]
        for axis, bounds_key in enumerate(("landing_x", "landing_y")):
            bounds = contract.get(bounds_key)
            if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in bounds)
                    or not bounds[0] < target[axis] < bounds[1]):
                raise ValueError("sweet-spot target lies outside the bank's qualified landing bounds")
    elif ("landing_objective" in task or "landing_objective" in contract
          or "landing_objective_sha256" in contract):
        raise ValueError("recipe-22 must not contain sweet-spot landing objective labels")
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


def _validate_landing_objective(value, profile, base):
    from sweet_spot_profile import load_objective, objective_in_training_frame, objective_sha256
    value = _mapping(value, "landing objective")
    if set(value) != {"path", "sha256", "descriptor"}:
        raise ValueError("landing objective must contain path, sha256 and descriptor")
    path = _path(value["path"], base)
    descriptor = load_objective(path)
    if objective_sha256(path) != value["sha256"] or descriptor != value["descriptor"]:
        raise ValueError("landing objective YAML checksum or descriptor differs from manifest")
    objective_in_training_frame(descriptor, profile)  # Reject illegal target/region geometry.
    return {"path": str(path), "sha256": value["sha256"], "descriptor": descriptor}


def _configure_sweet_spot(task, objective, profile):
    """Derive reward fields from canonical controls; reject conflicting duplicates."""
    from sweet_spot_profile import objective_in_training_frame
    settings = OmegaConf.to_container(task.get("sweet_spot"), resolve=True)
    if not isinstance(settings, dict) or set(settings) != {
            "reward_enabled", "distance_scale_m", "inside_bonus", "reward_weight"}:
        raise ValueError("task.sweet_spot requires reward_enabled, distance_scale_m, inside_bonus and reward_weight")
    if type(settings["reward_enabled"]) is not bool:
        raise ValueError("task.sweet_spot.reward_enabled must be an ordinary boolean")
    for key in ("distance_scale_m", "inside_bonus", "reward_weight"):
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"task.sweet_spot.{key} must be a finite number")
        if value < 0 or (key == "distance_scale_m" and value == 0):
            raise ValueError(f"task.sweet_spot.{key} is outside its allowed range")
    if settings["reward_enabled"] and settings["reward_weight"] == 0:
        raise ValueError("enabled sweet-spot reward requires reward_weight > 0; use reward_enabled=false to disable")
    region = objective_in_training_frame(objective["descriptor"], profile)
    fields = {"sweet_spot_reward_enabled": settings["reward_enabled"],
              "sweet_spot_x_range": region["x_range"], "sweet_spot_y_range": region["y_range"],
              "sweet_spot_distance_scale_m": settings["distance_scale_m"],
              "sweet_spot_inside_bonus": settings["inside_bonus"]}
    for key, value in fields.items():
        current = task.racket.get(key)
        if current is not None and current != value:
            raise ValueError(f"task.racket.{key} conflicts with canonical task.sweet_spot/objective")
        task.racket[key] = value
    weight = settings["reward_weight"] if settings["reward_enabled"] else 0.0
    current = task.rewards.get("sweet_spot_landing_weight")
    if current is not None and current != weight:
        raise ValueError("sweet_spot_landing_weight conflicts with canonical task.sweet_spot")
    task.rewards.sweet_spot_landing_weight = weight
    task.landing_objective = objective


def configure_refitted_task(cfg, *, environ=None):
    """Bind a selected PPQ004 task before simulator imports and config snapshots.

    Manifest paths in task YAML are relative to the training package, regardless
    of launch cwd. Other explicit task recipes retain their existing behavior.
    """
    target_environment = os.environ if environ is None else environ
    revision = cfg.task.get("build_contract", {}).get("training_distribution_revision")
    if "ppq004_refit_manifest" not in cfg.task:
        if revision in (*RECIPE_BY_DISTRIBUTION, "stationary_reach_ppq004_v1", "stationary_reach_ppq004_runtime_v1"):
            raise ValueError("PPQ004 training requires task=HOPEPingPongStationaryPPQ004 and its refit manifest")
        if not (target_environment.get("HOPE_BALL_PHYSICS_YAML")
                or target_environment.get("HOPE_BALL_PHYSICS_CONFIG")):
            target_environment["HOPE_BALL_PHYSICS_YAML"] = str(
                TRAINING_ROOT.parents[1] / "configs/ball_physics_venue.yaml")
        return False
    if revision not in RECIPE_BY_DISTRIBUTION:
        raise ValueError("PPQ004 manifest requires a matching full or sweet-spot training identity")
    recipe = RECIPE_BY_DISTRIBUTION[revision]
    selected = cfg.task.get("ppq004_refit_manifest")
    if not isinstance(selected, str) or not selected:
        raise ValueError(f"task.ppq004_refit_manifest must name the validated recipe-{recipe} refit bundle")
    path = Path(selected).expanduser()
    manifest = path if path.is_absolute() else TRAINING_ROOT / path
    bank, physics, manifest = load_refit_bundle(manifest, expected_recipe=recipe)
    profile = load_profile(physics)
    frame = profile["training_frame"]
    if frame["y_center"] != 0.0:
        raise ValueError("fixed-station training requires table y_center=0")
    with open_dict(cfg.task):
        if recipe == 23:
            bundle_task = yaml.safe_load(manifest.read_text())["task"]
            objective = _validate_landing_objective(bundle_task["landing_objective"], profile, manifest.parent)
            _configure_sweet_spot(cfg.task, objective, profile)
            if cfg.task.build_contract.get("reward_revision") != SWEET_SPOT_REWARD:
                raise ValueError("sweet-spot recipe requires its distinct reward revision")
        elif cfg.task.get("sweet_spot") is not None or cfg.task.get("landing_objective") is not None:
            raise ValueError("recipe-22 general-return baseline cannot carry sweet-spot reward settings")
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
