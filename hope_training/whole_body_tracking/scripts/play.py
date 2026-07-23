"""Play a trained HOPE policy in the Isaac Lab viewer.

Loads a LOCAL checkpoint and runs the policy in-sim. No Weights & Biases, and no export coupling —
exporting the ONNX policy is a separate step (scripts/export_onnx.py).

Usage:
    python scripts/play.py task=HOPEPingPong num_envs=4 \
        checkpoint=logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt
"""

import pathlib
import sys

import hydra
from omegaconf import OmegaConf


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    return here.parents[2]


def _resolve_motion_path(value: str) -> str:
    p = pathlib.Path(str(value))
    if p.is_file():
        return str(p.resolve())
    rooted = _repo_root() / value
    return str(rooted.resolve()) if rooted.is_file() else str(rooted)


def _resolve_motion_sources(cfg) -> list[str]:
    from train import _resolve_motion_plan

    return _resolve_motion_plan(cfg)


def _run(cfg, simulation_app):
    import os

    import gymnasium as gym
    import torch

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg

    import whole_body_tracking.tasks  # noqa: F401
    from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
    from whole_body_tracking.utils.ppo_cfg import runner_kwargs

    task_id = str(cfg.task.gym_task)
    num_envs = int(cfg.num_envs)

    env_cfg = parse_env_cfg(task_id, device=str(cfg.device), num_envs=num_envs)
    motion_files, motion_metadata = _resolve_motion_sources(cfg)
    if motion_files:
        env_cfg.commands.motion.motion_file = motion_files if len(motion_files) > 1 else motion_files[0]
        from train import _apply_motion_metadata

        _apply_motion_metadata(env_cfg, motion_files, motion_metadata, [])
    if cfg.task.get("motion") is not None and cfg.task.motion.get("wrap_teleport") is not None:
        env_cfg.commands.motion.wrap_teleport = bool(cfg.task.motion.wrap_teleport)

    # resolve the checkpoint: explicit path, else latest local checkpoint under logs/rsl_rl/<exp>/.
    experiment_name = str(cfg.task.experiment_name)
    if cfg.checkpoint is not None:
        resume_path = os.path.abspath(str(cfg.checkpoint))
    else:
        log_root = os.path.abspath(os.path.join("logs", "rsl_rl", experiment_name))
        resume_path = get_checkpoint_path(log_root, ".*", ".*")
    print(f"[play.py] loading checkpoint: {resume_path}", flush=True)

    render_mode = "rgb_array" if bool(cfg.video) else None
    env = gym.make(task_id, cfg=env_cfg, render_mode=render_mode)
    if bool(cfg.video):
        video_folder = os.path.abspath(
            str(cfg.video_folder) if cfg.video_folder is not None else os.path.join("outputs", "play_videos")
        )
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=video_folder,
            step_trigger=lambda step: step == 0,
            video_length=int(cfg.video_length),
            disable_logger=True,
            name_prefix="hope_play",
        )
    env = RslRlVecEnvWrapper(env)

    algo = OmegaConf.to_container(cfg.algo, resolve=True)
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg

    agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(algo, experiment_name))
    agent_cfg.device = str(cfg.device)
    runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.get_observations()
    steps = 0
    max_steps = int(cfg.video_length) if bool(cfg.video) else None
    try:
        while simulation_app.is_running():
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, _, _ = env.step(actions)
            steps += 1
            if max_steps is not None and steps >= max_steps:
                break
    finally:
        env.close()


@hydra.main(version_base=None, config_path="../cfg", config_name="play")
def main(cfg):
    OmegaConf.resolve(cfg)
    OmegaConf.set_struct(cfg, False)

    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(
        headless=bool(cfg.headless), device=str(cfg.device), enable_cameras=bool(cfg.video)
    )
    simulation_app = app_launcher.app
    try:
        _run(cfg, simulation_app)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
