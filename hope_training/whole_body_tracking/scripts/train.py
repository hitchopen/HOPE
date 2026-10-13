"""Hydra training entry for the HOPE Agibot A3 policy.

Single task, single algo. Build the ``HOPE-HitterPingPong-AgibotA3-v0`` environment (110-D
``hitter_pure`` actor observation, privileged critic, 50 Hz control, ``wrap_teleport: false``), a
rsl_rl PPO runner, and train. Checkpoints are written locally (periodic every ``save_interval`` and
a final one). The PPO runner config selects TensorBoard or W&B logging; credentials
and the team destination are configured outside the repository.

Usage:
    python scripts/train.py task=HOPEPingPong algo=ppo headless=true

Override any field on the CLI, e.g.:
    python scripts/train.py task=HOPEPingPong num_envs=2048 max_iterations=20000 seed=1 \
        motion_file=/abs/hope_forehand.npz motion_file_2=/abs/hope_backhand.npz

Tune training by editing cfg/task/HOPEPingPong.yaml and cfg/algo/ppo.yaml.
"""

import os
import pathlib
import sys

import hydra
from omegaconf import OmegaConf


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
    repo_root = _repo_root()
    candidates = (
        repo_root / value,
        repo_root / "hope_training" / "whole_body_tracking" / value,
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    # Return the training-root candidate because task YAML paths are relative
    # to hope_training/whole_body_tracking, independent of the launch cwd.
    return str(candidates[-1].resolve())


def _resolve_motion_sources(cfg) -> list[str]:
    """Return the list of local clip paths [forehand, backhand] (CLI overrides the task cfg)."""
    primary = cfg.motion_file if cfg.motion_file is not None else cfg.task.get("motion_file")
    secondary = cfg.motion_file_2 if cfg.motion_file_2 is not None else cfg.task.get("motion_file_2")
    clips = [primary]
    if secondary is not None:
        clips.append(secondary)
    resolved = [_resolve_motion_path(c) for c in clips if c is not None]
    if not resolved:
        raise RuntimeError(
            "No motion clip configured. Set motion_file (and motion_file_2) on the CLI or in "
            "cfg/task/HOPEPingPong.yaml."
        )
    for clip in resolved:
        if not pathlib.Path(clip).is_file():
            raise FileNotFoundError(
                f"motion clip not found: {clip}\nUse the published clips under "
                "hope_training/motions/preprocessed/ or provide your own pair "
                "(see docs/REPLACE_MOTIONS.md)."
            )
    return resolved


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


def _validate_training_checkpoint(loaded, policy, recipe_revision, reward_revision=None):
    """Do not silently resume a legacy optimizer into a different training MDP."""
    state = loaded.get("hope_exact_resume_state", {})
    if state.get("reward_revision") != reward_revision:
        raise ValueError("checkpoint_path reward revision differs; use explicit actor warm-start")
    expected = getattr(policy, "exploration_contract", None)
    if state.get("policy_exploration_contract") != expected:
        raise ValueError(
            "checkpoint_path exploration contract differs from this recipe; train from "
            "scratch with checkpoint_path=null. Actor migration requires a separate explicit "
            "weights-only workflow, not optimizer resume."
        )
    if state.get("training_recipe_revision") != recipe_revision:
        raise ValueError(
            "checkpoint_path training recipe differs; use checkpoint_path=null for the "
            "repaired rally task instead of resuming the old optimizer/critic."
        )


def _apply_domain_rand(env_cfg, dr, applied: list) -> None:
    """Apply the shared link-mass / PD-gain randomization knobs.

    The event terms are named ``events.randomize_link_mass`` and
    ``events.randomize_pd_gains`` in :class:`HOPEEventCfg` — the override MUST target
    those exact fields (see ``tests/test_domain_rand_overrides.py``). Semantics per
    range knob:
      * absent          -> keep the env-cfg default;
      * ``null``        -> disable the event entirely (set the term to None);
      * ``[lo, hi]``    -> override the distribution parameters.

    PD gains have two modes:
      * ``pd_mode`` absent (legacy generic scale DR): ``pd_gain_range`` drives the
        stiffness/damping distribution params of ``events.randomize_pd_gains``;
      * ``pd_mode: a3_message_passive_nominal_cohort_v1`` (the shipped
        HitterPingPong recipe, ``mdp.randomize_a3_message_pd_gains``):
        ``pd_gain_range`` MUST be null (it only retires the generic scale DR — the
        a3-message term stays), and ``pd_alpha_range`` / ``pd_beta_range`` /
        ``pd_nominal_fraction`` override the term's alpha/beta/nominal params.
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

    _apply("link_mass_range", "randomize_link_mass", ("mass_distribution_params",))

    pd_mode = dr.get("pd_mode")
    if pd_mode is None:
        _apply(
            "pd_gain_range",
            "randomize_pd_gains",
            ("stiffness_distribution_params", "damping_distribution_params"),
        )
        return

    # a3-message PD cohort (the env cfg already installs randomize_a3_message_pd_gains
    # for this recipe): pd_gain_range null only documents that the generic scale DR is
    # off; the alpha/beta/nominal knobs refine the installed term.
    if dr.get("pd_gain_range") is not None:
        print(
            f"[train.py] WARNING: domain_rand.pd_mode={pd_mode!r} requires "
            "pd_gain_range: null (the a3-message term replaces the generic scale DR); "
            "pd_gain_range ignored.",
            flush=True,
        )
    term = getattr(events, "randomize_pd_gains", None)
    if term is None:
        print(
            f"[train.py] WARNING: domain_rand.pd_mode={pd_mode!r}: "
            "events.randomize_pd_gains is disabled in the env cfg; PD knobs ignored.",
            flush=True,
        )
        return
    for range_key, param_key in (
        ("pd_alpha_range", "alpha_range"),
        ("pd_beta_range", "beta_range"),
    ):
        rng = dr.get(range_key)
        if rng is not None:
            term.params[param_key] = (float(rng[0]), float(rng[1]))
            applied.append(f"events.randomize_pd_gains.{param_key} = {term.params[param_key]}")
    nominal = dr.get("pd_nominal_fraction")
    if nominal is not None:
        term.params["nominal_fraction"] = float(nominal)
        applied.append(f"events.randomize_pd_gains.nominal_fraction = {float(nominal)}")


def _apply_task_overrides(env_cfg, cfg, applied: list) -> None:
    """Apply the launcher-level knobs + generic dotted-path overrides from the task cfg."""
    from whole_body_tracking.utils.task_reward_overrides import apply_reward_overrides
    from whole_body_tracking.utils.task_command_overrides import apply_command_overrides
    from whole_body_tracking.utils.task_sim2real_overrides import apply_sim2real_overrides

    task = cfg.task
    # episode length (top-level on ManagerBasedRLEnvCfg).
    env_block = task.get("env")
    if env_block is not None and env_block.get("episode_length_s") is not None:
        _set_dotted(env_cfg, "episode_length_s", float(env_block.get("episode_length_s")), applied, "env")
    apply_command_overrides(env_cfg, OmegaConf.to_container(task, resolve=True), applied)
    apply_sim2real_overrides(env_cfg, OmegaConf.to_container(task, resolve=True), applied)
    # domain randomization.
    _apply_domain_rand(env_cfg, task.get("domain_rand"), applied)
    # Reward terms are registered by the EnvCfg, but Hydra's rewards mapping is
    # not applied by Isaac Lab automatically. Keep the YAML as the recipe source
    # of truth and fail loudly on any unmapped or misspelled field.
    apply_reward_overrides(env_cfg.rewards, task.get("rewards"), applied)
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

    import importlib

    importlib.import_module("whole_body_tracking.tasks")  # registers the gym task
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
    if bool(cfg.headless) and not bool(cfg.video):
        # No USD debug markers are needed for headless training without video.
        # These are presentation only; contact sensing and commands stay on.
        env_cfg.commands.motion.debug_vis = False
        env_cfg.scene.contact_forces.debug_vis = False
        applied.append("headless/no-video: motion/contact debug markers disabled")
    env_cfg.seed = int(cfg.seed)
    env_cfg.sim.device = str(cfg.device)
    print(f"[train.py] task={task_id} num_envs={num_envs} — applied {len(applied)} task override(s):", flush=True)
    for line in applied:
        print(f"[train.py]     {line}", flush=True)

    # 2) reference motion clips (local .npz; clip 0 = forehand, clip 1 = backhand).
    motion_files = _resolve_motion_sources(cfg)
    for i, mf in enumerate(motion_files):
        print(f"[train.py] motion clip {i}: {mf}", flush=True)
    env_cfg.commands.motion.motion_file = motion_files if len(motion_files) > 1 else motion_files[0]

    # 3) PPO runner cfg from cfg/algo/ppo.yaml.
    algo = OmegaConf.to_container(cfg.algo, resolve=True)
    if not bool(algo["algorithm"].get("settled_aux_enabled", False)):
        raise ValueError("HOPEPingPong requires algo=ppo")
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

    # Validate the actor observation contract when the task declares one (guarded import).
    # The name (e.g. hitter_pure, 110-D) is resolved generically through the contract registry.
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
    # W&B logging must not implicitly enable the legacy automatic ONNX export path.
    runner.checkpoint_export_onnx = bool(cfg.task.get("checkpoint_export_onnx", False))
    runner.add_git_repo_to_log(__file__)

    # 6) optional resume from a local checkpoint (strict: weights + optimizer + iteration counter).
    recipe_revision = cfg.task.get("build_contract", {}).get("training_recipe_revision")
    reward_revision = cfg.task.get("build_contract", {}).get("reward_revision")
    runner.checkpoint_resume_context = {"training_recipe_revision": recipe_revision, "reward_revision": reward_revision}
    ckpt = getattr(cfg, "checkpoint_path", None)
    warm_start = getattr(cfg, "warm_start_actor_checkpoint", None)
    if ckpt is not None and warm_start is not None:
        raise ValueError("checkpoint_path and warm_start_actor_checkpoint are mutually exclusive")
    if ckpt is not None:
        ckpt = os.path.abspath(str(ckpt))
        if not os.path.isfile(ckpt):
            raise FileNotFoundError(f"[train.py] checkpoint_path does not exist: {ckpt}")
        loaded = torch.load(ckpt, map_location="cpu", weights_only=False)
        _validate_training_checkpoint(loaded, runner.alg.policy, recipe_revision, reward_revision)
        warm_guard = loaded.get("hope_exact_resume_state", {}).get("actor_warm_start")
        del loaded
        runner.load(ckpt)
        if warm_guard:
            runner.v17_actor_freeze_until_iteration = warm_guard["critic_iterations"]
            runner.v17_actor_finetune_learning_rate = warm_guard["finetune_lr"]
            runner.checkpoint_resume_context["actor_warm_start"] = warm_guard
        print(f"[train.py] resumed from checkpoint: {ckpt}", flush=True)

    if warm_start is not None:
        import hashlib
        import json
        import math
        from whole_body_tracking.utils.actor_warm_start import initialize_actor
        warm_path = pathlib.Path(str(warm_start)).resolve()
        critic_iterations = int(cfg.warm_start_critic_iterations)
        finetune_lr = float(cfg.warm_start_finetune_lr)
        if critic_iterations < 1 or not math.isfinite(finetune_lr) or finetune_lr <= 0:
            raise ValueError("actor warm-start requires positive critic iterations and finite learning rate")
        if runner.alg.__class__.__name__ != "ProjectionPPO":
            raise ValueError("actor warm-start requires ProjectionPPO's critic-only update guard")
        source_cfg = OmegaConf.load(warm_path.parent / "params/task_resolved.yaml")
        source_task = OmegaConf.to_container(source_cfg.task, resolve=True)
        target_task = OmegaConf.to_container(cfg.task, resolve=True)
        loaded = torch.load(warm_path, map_location="cpu", weights_only=False)
        initialize_actor(runner.alg.policy, loaded, source_task, target_task)
        del loaded
        warm_guard = {"source": str(warm_path), "sha256": hashlib.sha256(warm_path.read_bytes()).hexdigest(),
                      "critic_iterations": critic_iterations, "finetune_lr": finetune_lr}
        runner.v17_actor_freeze_until_iteration = critic_iterations
        runner.v17_actor_finetune_learning_rate = finetune_lr
        runner.checkpoint_resume_context["actor_warm_start"] = warm_guard
        pathlib.Path(log_dir).mkdir(parents=True, exist_ok=True)
        pathlib.Path(log_dir, "actor_warm_start.json").write_text(json.dumps(warm_guard, indent=2) + "\n")
        print(f"[train.py] actor/std initialized from {warm_path}; fresh critic/optimizer; guard={warm_guard}", flush=True)

    # 7) dump the resolved configuration + train.
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    OmegaConf.save(cfg, os.path.join(log_dir, "params", "task_resolved.yaml"))
    # Dump before Kit shutdown, which may exit without running Python's atexit hooks.
    profile_path = os.environ.get("HOPE_TRAIN_PROFILE")
    if profile_path:
        import cProfile
        profile = cProfile.Profile()
        try:
            with profile:
                runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        finally:
            pathlib.Path(profile_path).parent.mkdir(parents=True, exist_ok=True)
            profile.dump_stats(profile_path)
    else:
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
        # Kit shutdown can exit before Python's atexit hooks run. Flush W&B
        # while its background service is still alive, including failed runs.
        try:
            wandb = sys.modules.get("wandb")
            if wandb is not None and wandb.run is not None:
                wandb.finish(exit_code=1 if failed else 0)
        finally:
            simulation_app.close()
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
