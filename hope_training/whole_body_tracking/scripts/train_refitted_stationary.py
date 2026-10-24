"""Launch StationaryRally with a verified PPQ004 refit and matching physics.

Run with the Isaac interpreter, from any directory:
  hope_isaac_py scripts/train_refitted_stationary.py --overrides \
      /path/to/refitted/training_overrides.yaml --dry-run num_envs=64

The refitter's YAML is a receipt bundle, not a Hydra config. This launcher
checks its identities before setting the physics environment and translating
the bank fields to Hydra arguments. It does not establish policy performance.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import yaml


TRAINING_ROOT = Path(__file__).resolve().parents[1]
DISTRIBUTION = "stationary_reach_ppq004_v1"
QUALIFICATION = "ppq004_at_strike_retarget_and_fine_requalification_v1"
BANK_KEYS = {
    "venue_tuple_bank_path", "venue_tuple_bank_receipt_path",
    "venue_tuple_bank_sha256", "venue_tuple_bank_receipt_sha256",
    "venue_tuple_bank_recipe_revision",
}
# Keep the verified recipe intact while allowing normal run/resume controls.
RUN_KEYS = {
    "headless", "device", "seed", "num_envs", "max_iterations", "run_name",
    "checkpoint_path", "warm_start_actor_checkpoint", "warm_start_critic_iterations",
    "warm_start_finetune_lr", "video", "video_length", "video_interval",
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


def prepare_launch(overrides_path, run_overrides=()):
    """Return argv and environment; importing this module never starts Isaac."""
    manifest = Path(overrides_path).expanduser().resolve(strict=True)
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

    # Reject config-group selection, deletion/addition syntax, sweeps and
    # environment overrides. subprocess receives argv directly, never a shell.
    from hydra.core.override_parser.overrides_parser import OverridesParser
    parser = OverridesParser.create()
    for argument in run_overrides:
        try:
            parsed = parser.parse_override(argument)
        except Exception as exc:
            raise ValueError(f"invalid Hydra run option: {argument}") from exc
        if (parsed.key_or_group not in RUN_KEYS or parsed.package is not None
                or parsed.is_add() or parsed.is_force_add() or parsed.is_delete()
                or parsed.is_sweep_override()):
            raise ValueError(f"only ordinary run options may override the refit launch: {argument}")

    resolved = dict(bank, venue_tuple_bank_path=str(bank_path),
                    venue_tuple_bank_receipt_path=str(receipt_path))
    command = [sys.executable, str(TRAINING_ROOT / "scripts/train.py"),
               "task=HOPEPingPongStationaryRally", "algo=ppo_balanced_hold",
               f"task.build_contract.training_distribution_revision={DISTRIBUTION}"]
    command.extend(f"task.racket.{key}={json.dumps(value)}" for key, value in sorted(resolved.items()))
    command.extend(run_overrides)
    env = os.environ.copy()
    env["HOPE_BALL_PHYSICS_YAML"] = str(physics)
    return command, env


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--overrides", type=Path, required=True,
                        help="training_overrides.yaml written by refit_stationary_reach_bank.py")
    parser.add_argument("--dry-run", action="store_true", help="verify and print without starting Isaac")
    parser.add_argument("run_overrides", nargs="*", help="ordinary Hydra run options, such as seed=1")
    args = parser.parse_args(argv)
    try:
        command, env = prepare_launch(args.overrides, args.run_overrides)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(shlex.join(["env", f"HOPE_BALL_PHYSICS_YAML={env['HOPE_BALL_PHYSICS_YAML']}", *command]), flush=True)
    if args.dry_run:
        return 0
    return subprocess.call(command, env=env, cwd=TRAINING_ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
