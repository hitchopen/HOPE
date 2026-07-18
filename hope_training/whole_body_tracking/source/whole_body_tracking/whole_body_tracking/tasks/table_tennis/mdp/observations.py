"""Observation terms for the table-tennis environment (ball state in the robot base frame).

These ball terms are scene / critic (privileged) signals for training the returner policy; they are not
part of the deployed actor observation.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_rotate_inverse

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ball_position_b(
    env: "ManagerBasedRLEnv",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ball_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """Ball position relative to the robot base, expressed in the robot base frame. Shape ``(N, 3)``."""
    robot: Articulation = env.scene[robot_cfg.name]
    ball: RigidObject = env.scene[ball_cfg.name]
    rel_w = ball.data.root_pos_w - robot.data.root_pos_w
    return quat_rotate_inverse(robot.data.root_quat_w, rel_w)


def ball_velocity_b(
    env: "ManagerBasedRLEnv",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ball_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """Ball linear velocity expressed in the robot base frame. Shape ``(N, 3)``."""
    robot: Articulation = env.scene[robot_cfg.name]
    ball: RigidObject = env.scene[ball_cfg.name]
    return quat_rotate_inverse(robot.data.root_quat_w, ball.data.root_lin_vel_w)
