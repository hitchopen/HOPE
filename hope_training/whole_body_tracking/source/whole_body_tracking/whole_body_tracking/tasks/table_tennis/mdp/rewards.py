"""Reward terms for the table-tennis environment.

Small, ball-aware example terms; the generic robot rewards (alive, action-rate, ...) are reused from
``isaaclab.envs.mdp``. Add more match objectives (racket-to-ball tracking, net crossing) here as the
policy is developed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

from .. import geometry

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ball_above_surface(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """1.0 while the ball is above the table surface (world z > 0), else 0.0. Shape ``(N,)``.

    A "ball in play" signal demonstrating how to read ball state in the world frame (subtract the
    per-environment origin) for reward shaping."""
    ball: RigidObject = env.scene[asset_cfg.name]
    z_world = ball.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return (z_world > 0.0).float()


def ball_bounce_opponent_half(
    env: "ManagerBasedRLEnv",
    surface_band: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """1.0 while the ball is near the table surface over the opponent (P2) half, else 0.0. Shape ``(N,)``.

    A simplified, stateless proxy for the "ball bounces on the opponent half" return objective: the ball
    center is within ``[0, BALL_RADIUS + surface_band]`` of the surface (a bounce is near this height)
    **and** its (x, y) lies on the opponent half of the table (``NET_X < x <= TABLE_LENGTH``,
    ``-TABLE_WIDTH <= y <= 0``). Combine with racket-contact and net-crossing terms for a full return
    reward."""
    ball: RigidObject = env.scene[asset_cfg.name]
    p = ball.data.root_pos_w - env.scene.env_origins  # world-frame position
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    near_surface = (z > 0.0) & (z <= geometry.BALL_RADIUS + surface_band)
    on_opponent_half = (x > geometry.NET_X) & (x <= geometry.TABLE_LENGTH)
    within_width = (y >= -geometry.TABLE_WIDTH) & (y <= 0.0)
    return (near_surface & on_opponent_half & within_width).float()
