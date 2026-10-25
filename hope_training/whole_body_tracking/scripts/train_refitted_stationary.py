"""Launch StationaryRally with a verified PPQ004 refit and matching physics.

Run with the Isaac interpreter, from any directory:
  hope_isaac_py scripts/train_refitted_stationary.py --overrides \
      /path/to/refitted/training_overrides.yaml --dry-run num_envs=64

The refitter's YAML is a receipt bundle, not a Hydra config. This launcher
checks its identities before setting the physics environment and translating
the bank fields to Hydra arguments. It does not establish policy performance.
"""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

from ppq004_training import (DISTRIBUTION, QUALIFICATION, TRAINING_ROOT, load_refit_bundle)


# Keep the verified recipe intact while allowing normal run/resume controls.
RUN_KEYS = {
    "headless", "device", "seed", "num_envs", "max_iterations", "run_name",
    "checkpoint_path", "warm_start_actor_checkpoint", "warm_start_critic_iterations",
    "warm_start_finetune_lr", "video", "video_length", "video_interval",
}


def prepare_launch(overrides_path, run_overrides=()):
    """Return argv and environment; importing this module never starts Isaac."""
    _, physics, manifest = load_refit_bundle(overrides_path)

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

    command = [sys.executable, str(TRAINING_ROOT / "scripts/train.py"),
               "task=HOPEPingPongStationaryPPQ004", "algo=ppo_balanced_hold",
               f"task.ppq004_refit_manifest={json.dumps(str(manifest))}"]
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
