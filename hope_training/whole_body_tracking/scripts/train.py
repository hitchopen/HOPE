"""Hydra training entry for the HOPE Agibot A3 policy.

Single task, single algo. Build the ``HOPE-PingPong-AgibotA3-v0`` environment (111-D actor
observation, privileged critic, 50 Hz control, ``wrap_teleport: false``), a rsl_rl PPO runner, and
train. Checkpoints are written locally (periodic every ``save_interval`` and a final one). There is
no Weights & Biases, no external logging service, no gate / lineage / curriculum machinery.

Usage:
    python scripts/train.py task=HOPEPingPong algo=ppo headless=true

Override any field on the CLI, e.g.:
    python scripts/train.py task=HOPEPingPong num_envs=2048 max_iterations=20000 seed=1 \
        motion_file=/abs/hope_forehand.npz motion_file_2=/abs/hope_backhand.npz

Tune training by editing cfg/task/HOPEPingPong.yaml and cfg/algo/ppo.yaml.
"""

import csv
import os
import pathlib
import sys

import hydra
from omegaconf import OmegaConf
import yaml


def _repo_root() -> pathlib.Path:
    """Repo root = the directory that contains ``hope_training/`` (walk up from this file)."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    return here.parents[2]


def _resolve_motion_path(value: str) -> str:
    """Resolve a clip path: absolute / cwd-relative first, then repo-root-relative."""
    p = pathlib.Path(str(value))
    if p.is_file():
        return str(p.resolve())
    rooted = _repo_root() / value
    if rooted.is_file():
        return str(rooted.resolve())
    # Return the repo-root candidate so the error message points at a stable location.
    return str(rooted)


def _cfg_get(cfg, key: str, default=None):
    if hasattr(cfg, "get"):
        return cfg.get(key, default)
    return getattr(cfg, key, default)


def _motion_list(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    return [str(item) for item in value]


def _resolve_existing_file(value: str) -> pathlib.Path:
    p = pathlib.Path(str(value)).expanduser()
    if p.is_file():
        return p.resolve()
    rooted = _repo_root() / str(value)
    if rooted.is_file():
        return rooted.resolve()
    return rooted


def _float_or_none(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _range3_from_mapping(mapping: dict, prefix: str, *, source: str) -> tuple | None:
    """Parse prefix_{x,y,z}_{lo,hi} as a 3-axis target box."""
    ranges = []
    present = []
    for axis in ("x", "y", "z"):
        lo = _float_or_none(mapping.get(f"{prefix}_{axis}_lo"))
        hi = _float_or_none(mapping.get(f"{prefix}_{axis}_hi"))
        present.extend((lo is not None, hi is not None))
        ranges.append((lo, hi, axis))
    if not any(present):
        return None
    if not all(present):
        raise ValueError(f"{source}: incomplete {prefix}_{{x,y,z}}_{{lo,hi}} target box.")
    out = []
    for lo, hi, axis in ranges:
        if hi < lo:
            raise ValueError(f"{source}: {prefix}_{axis}_hi ({hi}) is smaller than {prefix}_{axis}_lo ({lo}).")
        out.append((float(lo), float(hi)))
    return tuple(out)


def _metadata_from_mapping(mapping: dict, *, source: str) -> dict:
    strike_phase = _float_or_none(mapping.get("strike_phase"))
    swing_side = _float_or_none(mapping.get("swing_side"))
    if strike_phase is None:
        strike_frame = _float_or_none(mapping.get("strike_frame"))
        frames = _float_or_none(mapping.get("frames") or mapping.get("frame_count"))
        if strike_frame is not None and frames is not None and frames > 1:
            strike_phase = float(strike_frame / (frames - 1))
    return {
        "source": source,
        "strike_phase": strike_phase,
        "swing_side": swing_side,
        "racket_pos_range": _range3_from_mapping(mapping, "racket_pos", source=source),
        "racket_vel_range": _range3_from_mapping(mapping, "racket_vel", source=source),
    }


def _metadata_from_sidecar(path: str) -> dict:
    sidecar = pathlib.Path(path).with_suffix(".yaml")
    if not sidecar.is_file():
        return {
            "source": str(sidecar),
            "strike_phase": None,
            "swing_side": None,
            "racket_pos_range": None,
            "racket_vel_range": None,
        }
    with sidecar.open("r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    return _metadata_from_mapping(doc, source=str(sidecar))


def _resolve_motion_plan(cfg) -> tuple[list[str], list[dict]]:
    """Return local clip paths and optional per-clip timing/side metadata."""
    manifest = _cfg_get(cfg, "motion_manifest")
    if manifest is not None:
        manifest_path = _resolve_existing_file(str(manifest))
        if not manifest_path.is_file():
            raise FileNotFoundError(f"motion_manifest not found: {manifest_path}")
        resolved: list[str] = []
        metadata: list[dict] = []
        with manifest_path.open("r", encoding="utf-8", newline="") as fh:
            for row in csv.DictReader(fh, delimiter="\t"):
                clip = row.get("output") or row.get("motion_file") or row.get("path") or row.get("file")
                if not clip:
                    continue
                path = _resolve_motion_path(clip)
                resolved.append(path)
                metadata.append(_metadata_from_mapping(row, source=str(manifest_path)))
        if not resolved:
            raise RuntimeError(f"motion_manifest has no clip rows: {manifest_path}")
    else:
        explicit = _motion_list(_cfg_get(cfg, "motion_files"))
        if explicit:
            clips = explicit
        else:
            primary = cfg.motion_file if cfg.motion_file is not None else cfg.task.get("motion_file")
            secondary = cfg.motion_file_2 if cfg.motion_file_2 is not None else cfg.task.get("motion_file_2")
            clips = [primary]
            if secondary is not None:
                clips.append(secondary)
        resolved = [_resolve_motion_path(c) for c in clips if c is not None]
        metadata = [_metadata_from_sidecar(path) for path in resolved]

    if not resolved:
        raise RuntimeError(
            "No motion clip configured. Set motion_manifest, motion_files, motion_file, or "
            "motion_file_2 on the CLI or in cfg/task/HOPEPingPong.yaml."
        )
    for clip in resolved:
        if not pathlib.Path(clip).is_file():
            raise FileNotFoundError(
                f"motion clip not found: {clip}\nProvide your own clips or the placeholder clips "
                "under hope_training/motions/preprocessed/ (see docs/REPLACE_MOTIONS.md)."
            )
    return resolved, metadata


def _resolve_motion_sources(cfg) -> list[str]:
    """Return local clip paths; kept for scripts/tests that only need the path list."""
    return _resolve_motion_plan(cfg)[0]


def _set_dotted(obj, dotted: str, value, applied: list, where: str) -> None:
    """Set ``obj.<a>.<b>... = value`` if the attribute chain exists; else warn and skip."""
    parts = dotted.split(".")
    node = obj
    for attr in parts[:-1]:
        if not hasattr(node, attr):
            print(f"[train.py] WARNING: {where}: '{dotted}' — no attribute '{attr}'; skipped.", flush=True)
            return
        node = getattr(node, attr)
    leaf = parts[-1]
    if not hasattr(node, leaf):
        print(f"[train.py] WARNING: {where}: '{dotted}' — no attribute '{leaf}'; skipped.", flush=True)
        return
    setattr(node, leaf, value)
    applied.append(f"{dotted} = {value}")


def _apply_domain_rand(env_cfg, dr, applied: list) -> None:
    """Apply the shared link-mass / PD-gain randomization knobs.

    The event terms are named ``events.link_mass`` and ``events.pd_gains`` in
    :class:`HOPEPingPongEnvCfg.EventCfg` — the override MUST target those exact fields
    (see ``tests/test_domain_rand_overrides.py``). Semantics per range knob:
      * absent          -> keep the env-cfg default;
      * ``null``        -> disable the event entirely (set the term to None);
      * ``[lo, hi]``    -> override the distribution parameters.
    """
    if dr is None:
        return
    events = getattr(env_cfg, "events", None)
    if events is None:
        return

    def _apply(range_key: str, event_name: str, param_keys: tuple[str, ...]) -> None:
        if range_key not in dr:
            return
        if not hasattr(events, event_name):
            print(
                f"[train.py] WARNING: domain_rand.{range_key}: events.{event_name} does not "
                "exist on this env cfg; skipped.",
                flush=True,
            )
            return
        rng = dr.get(range_key)
        if rng is None:
            if getattr(events, event_name) is not None:
                setattr(events, event_name, None)
                applied.append(f"events.{event_name} = None (disabled)")
            return
        term = getattr(events, event_name)
        if term is None:
            print(
                f"[train.py] WARNING: domain_rand.{range_key}: events.{event_name} is already "
                "disabled in the env cfg; range ignored.",
                flush=True,
            )
            return
        lo, hi = float(rng[0]), float(rng[1])
        for key in param_keys:
            term.params[key] = (lo, hi)
        applied.append(f"events.{event_name} = {(lo, hi)}")

    _apply("link_mass_range", "link_mass", ("mass_distribution_params",))
    _apply(
        "pd_gain_range",
        "pd_gains",
        ("stiffness_distribution_params", "damping_distribution_params"),
    )


def _expand_two_side_boxes(boxes, sides: list[float]) -> tuple | None:
    if boxes is None:
        return None
    box_list = tuple(boxes)
    if len(box_list) != 2 or len(sides) == 2:
        return boxes
    return tuple(box_list[0] if side >= 0.0 else box_list[1] for side in sides)


def _metadata_box_tuple(metadata: list[dict], key: str, label: str) -> tuple | None:
    boxes = [m.get(key) for m in metadata]
    has_box = [box is not None for box in boxes]
    if not any(has_box):
        return None
    if not all(has_box):
        missing = [str(m.get("source", f"clip {i}")) for i, ok in enumerate(has_box) if not ok]
        raise ValueError(f"Incomplete per-clip {label}: every motion row needs {label} fields; missing {missing}")
    return tuple(boxes)


def _apply_motion_metadata(env_cfg, motion_files: list[str], metadata: list[dict], applied: list) -> None:
    """Wire sidecar/manifest timing metadata into the racket-target command."""
    if len(metadata) != len(motion_files):
        return
    commands = getattr(env_cfg, "commands", None)
    racket = getattr(commands, "racket_target", None) if commands is not None else None
    if racket is None:
        return

    phases = [m.get("strike_phase") for m in metadata]
    if all(phase is not None for phase in phases):
        values = tuple(float(phase) for phase in phases)
        racket.strike_phase_per_clip = values
        applied.append(f"commands.racket_target.strike_phase_per_clip = {values}")

    sides = [m.get("swing_side") for m in metadata]
    if all(side is not None for side in sides):
        side_values = tuple(1.0 if float(side) >= 0.0 else -1.0 for side in sides)
        if hasattr(racket, "swing_side_per_clip"):
            racket.swing_side_per_clip = side_values
            applied.append(f"commands.racket_target.swing_side_per_clip = {side_values}")
        if hasattr(racket, "mount_normal_sign_per_clip"):
            racket.mount_normal_sign_per_clip = side_values
            applied.append(f"commands.racket_target.mount_normal_sign_per_clip = {side_values}")

        pos_boxes = _expand_two_side_boxes(getattr(racket, "racket_pos_range_per_clip", None), list(side_values))
        vel_boxes = _expand_two_side_boxes(getattr(racket, "racket_vel_range_per_clip", None), list(side_values))
        if pos_boxes is not None and pos_boxes is not getattr(racket, "racket_pos_range_per_clip", None):
            racket.racket_pos_range_per_clip = pos_boxes
            applied.append("commands.racket_target.racket_pos_range_per_clip = expanded from swing_side_per_clip")
        if vel_boxes is not None and vel_boxes is not getattr(racket, "racket_vel_range_per_clip", None):
            racket.racket_vel_range_per_clip = vel_boxes
            applied.append("commands.racket_target.racket_vel_range_per_clip = expanded from swing_side_per_clip")

    pos_boxes = _metadata_box_tuple(metadata, "racket_pos_range", "racket_pos_range")
    if pos_boxes is not None and hasattr(racket, "racket_pos_range_per_clip"):
        racket.racket_pos_range_per_clip = pos_boxes
        applied.append("commands.racket_target.racket_pos_range_per_clip = from motion metadata")

    vel_boxes = _metadata_box_tuple(metadata, "racket_vel_range", "racket_vel_range")
    if vel_boxes is not None and hasattr(racket, "racket_vel_range_per_clip"):
        racket.racket_vel_range_per_clip = vel_boxes
        applied.append("commands.racket_target.racket_vel_range_per_clip = from motion metadata")


def _apply_task_overrides(env_cfg, cfg, applied: list) -> None:
    """Apply the launcher-level knobs + generic dotted-path overrides from the task cfg."""
    task = cfg.task
    # episode length (top-level on ManagerBasedRLEnvCfg).
    env_block = task.get("env")
    if env_block is not None and env_block.get("episode_length_s") is not None:
        _set_dotted(env_cfg, "episode_length_s", float(env_block.get("episode_length_s")), applied, "env")
    # continuous multi-rally lifecycle: no teleport on clip wrap.
    motion_block = task.get("motion")
    if motion_block is not None and motion_block.get("wrap_teleport") is not None:
        _set_dotted(
            env_cfg, "commands.motion.wrap_teleport", bool(motion_block.get("wrap_teleport")), applied, "motion"
        )
    # domain randomization.
    _apply_domain_rand(env_cfg, task.get("domain_rand"), applied)
    # generic overrides map (dotted attribute paths -> value).
    overrides = task.get("overrides")
    if overrides:
        for dotted, value in OmegaConf.to_container(overrides, resolve=True).items():
            _set_dotted(env_cfg, str(dotted), value, applied, "overrides")


def _run(cfg):
    import gymnasium as gym
    import torch
    from datetime import datetime

    from isaaclab.utils.io import dump_yaml
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from isaaclab_tasks.utils import parse_env_cfg

    import whole_body_tracking.tasks  # noqa: F401  -- registers the gym task
    from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
    from whole_body_tracking.utils.ppo_cfg import runner_kwargs

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

    task_id = str(cfg.task.gym_task)
    num_envs = int(cfg.num_envs) if cfg.num_envs is not None else int(cfg.task.env.num_envs)

    # 1) environment cfg from the registered gym task + task-cfg overrides.
    env_cfg = parse_env_cfg(task_id, device=str(cfg.device), num_envs=num_envs)
    applied: list = []
    _apply_task_overrides(env_cfg, cfg, applied)
    env_cfg.seed = int(cfg.seed)
    env_cfg.sim.device = str(cfg.device)

    # 2) reference motion clips and optional per-clip timing metadata.
    motion_files, motion_metadata = _resolve_motion_plan(cfg)
    for i, mf in enumerate(motion_files):
        print(f"[train.py] motion clip {i}: {mf}", flush=True)
    env_cfg.commands.motion.motion_file = motion_files if len(motion_files) > 1 else motion_files[0]
    _apply_motion_metadata(env_cfg, motion_files, motion_metadata, applied)
    print(f"[train.py] task={task_id} num_envs={num_envs} — applied {len(applied)} task override(s):", flush=True)
    for line in applied:
        print(f"[train.py]     {line}", flush=True)

    # 3) PPO runner cfg from cfg/algo/ppo.yaml.
    algo = OmegaConf.to_container(cfg.algo, resolve=True)
    agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(algo, str(cfg.task.experiment_name)))
    agent_cfg.seed = int(cfg.seed)
    agent_cfg.device = str(cfg.device)
    if cfg.max_iterations is not None:
        agent_cfg.max_iterations = int(cfg.max_iterations)
    if cfg.run_name is not None:
        agent_cfg.run_name = str(cfg.run_name)

    # 4) local logging directory.
    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root, log_dir)
    print(f"[train.py] experiment={agent_cfg.experiment_name} | log_dir={log_dir}", flush=True)

    # 5) build env, (optionally) record video, wrap for rsl_rl.
    render_mode = "rgb_array" if cfg.video else None
    env = gym.make(task_id, cfg=env_cfg, render_mode=render_mode)

    # JOINT-ORDER GATE (train time): the actor/deploy contract stays canonical even if Isaac imports
    # the articulation in a different internal order. The env config and MotionCommand install the
    # canonical<->articulation permutation; this gate rejects only missing/extra joints.
    from whole_body_tracking.utils.action_adapter_config import load_joint_order, resolve_joint_order_mapping

    _joint_names = list(env.unwrapped.scene["robot"].data.joint_names)
    _expected_order = list(load_joint_order())
    try:
        _mapping = resolve_joint_order_mapping(_joint_names, canonical_joint_names=_expected_order)
    except ValueError as exc:
        raise RuntimeError(
            "Articulation joint set does not match the canonical deploy joint set "
            "(hope_training/config/joint_order_agibot_a3.yaml).\n"
            f"  articulation: {_joint_names}\n"
            f"  canonical:    {_expected_order}\n"
            "Fix your A3 URDF/USD or canonical config before training."
        ) from exc
    if _mapping.is_identity:
        print("[train.py] joint-order gate: articulation matches the canonical deploy order.", flush=True)
    else:
        print(
            "[train.py] joint-order gate: using canonical->articulation permutation "
            f"{list(_mapping.canonical_to_articulation)}",
            flush=True,
        )

    # Validate the 111-D actor observation contract when the task declares one (guarded import).
    expected_contract = cfg.task.get("actor_obs_contract")
    if expected_contract is not None:
        try:
            from whole_body_tracking.tasks.tracking.actor_observation_contract import (
                validate_actor_observation_contract,
            )

            contract = validate_actor_observation_contract(env.unwrapped, str(expected_contract))
            print(
                f"[train.py] actor observation contract validated: {contract.name} "
                f"({contract.total_dim}D)",
                flush=True,
            )
        except ImportError:
            print("[train.py] NOTE: actor_observation_contract validator not available; skipping.", flush=True)

    if cfg.video:
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=os.path.join(log_dir, "videos", "train"),
            step_trigger=lambda step: step % int(cfg.video_interval) == 0,
            video_length=int(cfg.video_length),
            disable_logger=True,
        )
    env = RslRlVecEnvWrapper(env)

    runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.add_git_repo_to_log(__file__)

    # 6) optional resume from a local checkpoint (strict: weights + optimizer + iteration counter).
    ckpt = getattr(cfg, "checkpoint_path", None)
    if ckpt is not None:
        ckpt = os.path.abspath(str(ckpt))
        if not os.path.isfile(ckpt):
            raise FileNotFoundError(f"[train.py] checkpoint_path does not exist: {ckpt}")
        load_optimizer = bool(getattr(cfg, "checkpoint_load_optimizer", True))
        runner.load(ckpt, load_optimizer=load_optimizer)
        print(
            f"[train.py] resumed from checkpoint: {ckpt} "
            f"(load_optimizer={load_optimizer})",
            flush=True,
        )

    # 7) dump the resolved configuration + train.
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    env.close()


@hydra.main(version_base=None, config_path="../cfg", config_name="train")
def main(cfg):
    OmegaConf.resolve(cfg)
    OmegaConf.set_struct(cfg, False)

    # Launch Isaac Sim BEFORE importing isaaclab modules. Clear argv so Kit does not try to parse
    # Hydra's task=.../algo=... overrides.
    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=bool(cfg.headless), device=str(cfg.device), enable_cameras=bool(cfg.video))
    simulation_app = app_launcher.app

    failed = False
    try:
        _run(cfg)
    except Exception:
        import traceback

        print("\n[train.py] ERROR during run:", flush=True)
        traceback.print_exc()
        failed = True
    finally:
        simulation_app.close()
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
