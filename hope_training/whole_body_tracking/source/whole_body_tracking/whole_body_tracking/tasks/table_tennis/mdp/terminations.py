"""Termination terms for the table-tennis environment (ball out of play, robot fallen)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ball_out_of_bounds(
    env: "ManagerBasedRLEnv",
    bounds: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """True when the ball leaves the axis-aligned play volume (``bounds`` in the HOPE frame). Shape ``(N,)``."""
    ball: RigidObject = env.scene[asset_cfg.name]
    p = ball.data.root_pos_w - env.scene.env_origins  # HOPE-frame position
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    bx, by, bz = bounds["x"], bounds["y"], bounds["z"]
    return (
        (x < bx[0]) | (x > bx[1]) | (y < by[0]) | (y > by[1]) | (z < bz[0]) | (z > bz[1])
    )


def robot_base_too_low(
    env: "ManagerBasedRLEnv",
    minimum_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """True when the robot base (pelvis) drops below ``minimum_height`` in the HOPE frame. Shape ``(N,)``."""
    robot: Articulation = env.scene[asset_cfg.name]
    z_hope = robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return z_hope < minimum_height
