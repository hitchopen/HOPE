"""Termination helpers for physical falls and reference tracking failures."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from whole_body_tracking.tasks.tracking.mdp.commands import MotionCommand
from whole_body_tracking.tasks.tracking.mdp.rewards import _get_body_indexes

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _past_grace_steps(env: ManagerBasedRLEnv, command: MotionCommand, min_steps: int) -> torch.Tensor:
    steps = getattr(command, "steps_since_resample", env.episode_length_buf)
    return steps >= int(min_steps)


def _past_episode_steps(env: ManagerBasedRLEnv, min_steps: int) -> torch.Tensor:
    return env.episode_length_buf >= int(min_steps)


def base_tilted(
    env: ManagerBasedRLEnv,
    threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    min_steps: int = 0,
) -> torch.Tensor:
    """True when the base is tilted past ``threshold`` (horizontal component of projected gravity)."""
    asset: Articulation = env.scene[asset_cfg.name]
    bad = torch.norm(asset.data.projected_gravity_b[:, :2], dim=-1) > threshold
    return bad & _past_episode_steps(env, min_steps)


def base_too_low(
    env: ManagerBasedRLEnv,
    min_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    min_steps: int = 0,
) -> torch.Tensor:
    """True when the base root height drops below ``min_height`` (the robot has fallen/collapsed)."""
    asset: Articulation = env.scene[asset_cfg.name]
    bad = asset.data.root_pos_w[:, 2] < min_height
    return bad & _past_episode_steps(env, min_steps)


def bad_anchor_pos_z_only(
    env: ManagerBasedRLEnv, command_name: str, threshold: float, min_steps: int = 0
) -> torch.Tensor:
    """True when the reference anchor and robot anchor differ too much in height."""
    command: MotionCommand = env.command_manager.get_term(command_name)
    bad = torch.abs(command.anchor_pos_w[:, -1] - command.robot_anchor_pos_w[:, -1]) > threshold
    return bad & _past_grace_steps(env, command, min_steps) & (~command.in_hold)


def bad_anchor_ori(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    command_name: str,
    threshold: float,
    min_steps: int = 0,
) -> torch.Tensor:
    """True when the robot anchor tilt diverges too far from the reference anchor tilt."""
    asset: Articulation = env.scene[asset_cfg.name]
    command: MotionCommand = env.command_manager.get_term(command_name)
    motion_projected_gravity_b = math_utils.quat_rotate_inverse(command.anchor_quat_w, asset.data.GRAVITY_VEC_W)
    robot_projected_gravity_b = math_utils.quat_rotate_inverse(
        command.robot_anchor_quat_w, asset.data.GRAVITY_VEC_W
    )
    bad = (motion_projected_gravity_b[:, 2] - robot_projected_gravity_b[:, 2]).abs() > threshold
    return bad & _past_grace_steps(env, command, min_steps) & (~command.in_hold)


def bad_motion_body_pos_z_only(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float,
    body_names: list[str] | None = None,
    min_steps: int = 0,
) -> torch.Tensor:
    """True when selected tracked bodies drift too far from the reference in vertical position."""
    command: MotionCommand = env.command_manager.get_term(command_name)
    idx = _get_body_indexes(command, body_names)
    error = torch.abs(command.body_pos_relative_w[:, idx, -1] - command.robot_body_pos_w[:, idx, -1])
    bad = torch.any(error > threshold, dim=-1)
    return bad & _past_grace_steps(env, command, min_steps) & (~command.in_hold)
