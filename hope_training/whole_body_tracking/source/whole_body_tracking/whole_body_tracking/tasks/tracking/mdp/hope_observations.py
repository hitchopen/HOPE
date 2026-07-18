"""HOPE observation terms.

The actor (policy) observation is exactly the ``hope_pingpong`` contract (111 dims). Most
terms are standard proprioception from ``isaaclab.envs.mdp`` (base_ang_vel, joint_pos_rel,
joint_vel_rel, projected_gravity); the goal/target terms below wrap :class:`RacketTargetCommand`, and
``last_action`` reads the action term's deploy-faithful applied action.

The privileged terms at the bottom (actual racket FK state, desired racket normal) are CRITIC-ONLY.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.utils.math import quat_rotate_inverse, yaw_quat

from whole_body_tracking.tasks.tracking.mdp.hope_commands import RacketTargetCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _cmd(env: ManagerBasedRLEnv, command_name: str) -> RacketTargetCommand:
    return env.command_manager.get_term(command_name)


# --- actor (policy) terms --------------------------------------------------------- #
def base_forward_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Base forward unit vector e_base,x, world XY (2)."""
    return _cmd(env, command_name).base_forward_xy()


def fixed_station_error_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Fixed startup station XY minus current base XY, world frame (2). In-place recentring feedback."""
    return _cmd(env, command_name).fixed_station_error_xy()


def racket_target_rel_base(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Target racket position minus base position, world frame (3)."""
    return _cmd(env, command_name).racket_target_rel_base_w()


def racket_target_vel_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Desired racket velocity, world frame (3)."""
    return _cmd(env, command_name).racket_target_vel_w


def time_to_strike(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Remaining time until the strike, s (1)."""
    return _cmd(env, command_name).time_to_strike.unsqueeze(-1)


def swing_side(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Forehand (+1) / backhand (-1), locked per swing (1)."""
    return _cmd(env, command_name).swing_sign.unsqueeze(-1)


def applied_last_action(env: ManagerBasedRLEnv, action_name: str = "joint_pos") -> torch.Tensor:
    """Previous tick's applied raw action (31), including the zeroed passive-head columns.

    Reads the action term's deploy-faithful ``applied_raw_actions`` rather than Isaac Lab's global
    ``last_action`` so the passive-head zeroing is reflected in the feedback the actor sees.
    """
    action_term = env.action_manager.get_term(action_name)
    applied = getattr(action_term, "applied_raw_actions", None)
    if applied is None:
        raise RuntimeError(f"Action term {action_name!r} does not expose applied_raw_actions")
    return applied


# --- privileged (critic-only) terms --------------------------------------------------------- #
def racket_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket position relative to the base (FK, yaw-heading frame). Privileged."""
    cmd = _cmd(env, command_name)
    return quat_rotate_inverse(yaw_quat(cmd.base_quat_w), cmd.racket_pos_w - cmd.base_pos_w)


def racket_lin_vel_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket linear velocity (FK), world frame. Privileged."""
    return _cmd(env, command_name).racket_lin_vel_w


def racket_normal_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket face normal (FK), world frame. Privileged."""
    return _cmd(env, command_name).racket_normal_w


def racket_target_normal_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Desired racket face normal, world frame. Privileged (reward target only)."""
    return _cmd(env, command_name).racket_target_normal_w


def episode_time_left(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Time remaining in the episode (s). Privileged critic input."""
    buf = getattr(env, "episode_length_buf", None)
    if buf is None:
        return torch.zeros(env.num_envs, 1, device=env.device)
    return ((env.max_episode_length - buf).float() * env.step_dt).unsqueeze(-1)
