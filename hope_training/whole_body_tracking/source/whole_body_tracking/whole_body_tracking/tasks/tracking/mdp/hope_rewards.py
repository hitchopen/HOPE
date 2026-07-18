"""HOPE reward terms.

The public task uses eleven reward terms. Eight are defined here; three (upright balance,
action smoothness, joint-limit regularization) are standard ``isaaclab.envs.mdp`` terms wired in the
env config. All example weights and kernel widths live in the env config / task YAML and are meant to
be tuned — they are illustrative, not performance-tuned values.

  1. upright / balance            -> mdp.flat_orientation_l2       (env config)
  2. forehand/backhand imitation  -> sample_imitation
  3. racket position              -> racket_position
  4. racket velocity              -> racket_velocity
  5. simplified blade direction   -> racket_blade_direction
  6. actual ball contact          -> ball_contact
  7. net crossing                 -> ball_net_cross
  8. opponent-half first bounce   -> ball_opponent_bounce
  9. in-place follow-through/recovery -> follow_through_recovery
 10. action smoothness            -> mdp.action_rate_l2            (env config)
 11. joint-limit regularization   -> mdp.joint_pos_limits          (env config)

The racket position/velocity/blade terms are active only in a short window around the strike; the
contact/net/bounce terms fire once at the exact strike frame; imitation is active during the swing
(not the frozen pre-swing hold); recovery is active through the follow-through and the hold.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from whole_body_tracking.tasks.tracking.mdp import rewards as _imitation
from whole_body_tracking.tasks.tracking.mdp.hope_commands import RacketTargetCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _cmd(env: ManagerBasedRLEnv, command_name: str) -> RacketTargetCommand:
    return env.command_manager.get_term(command_name)


# --- (2) forehand/backhand sample imitation ------------------------------------------------- #
def sample_imitation(
    env: ManagerBasedRLEnv,
    command_name: str,
    std_pos: float = 0.3,
    std_ori: float = 0.4,
    body_names: list[str] | None = None,
) -> torch.Tensor:
    """Track the imitated clip's (upper-)body pose during the swing.

    Combines the anchor-relative body position and orientation tracking kernels and gates them to the
    active swing (zero during the frozen pre-swing hold). ``body_names`` selects the tracked subset
    (the env config passes the upper-body bodies so the legs stay free to step)."""
    rp = _imitation.motion_relative_body_position_error_exp(env, command_name, std_pos, body_names)
    ro = _imitation.motion_relative_body_orientation_error_exp(env, command_name, std_ori, body_names)
    motion = env.command_manager.get_term(command_name)
    return (0.5 * rp + 0.5 * ro) * (~motion.in_hold).float()


# --- (3,4,5) racket goal tracking, active in the strike window ------------------------------ #
def racket_position(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track the racket center against the target's swing-through point near the strike."""
    cmd = _cmd(env, command_name)
    target_now = cmd.racket_target_pos_w - cmd.racket_target_vel_w * cmd.time_to_strike.unsqueeze(-1)
    error = torch.sum(torch.square(cmd.racket_pos_w - target_now), dim=-1)
    return torch.exp(-error / std**2) * cmd.strike_window.float()


def racket_velocity(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track the racket linear velocity against the desired velocity near the strike."""
    cmd = _cmd(env, command_name)
    error = torch.sum(torch.square(cmd.racket_lin_vel_w - cmd.racket_target_vel_w), dim=-1)
    return torch.exp(-error / std**2) * cmd.strike_window.float()


def racket_blade_direction(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Align the racket face normal with the desired blade direction near the strike (``std`` in rad)."""
    cmd = _cmd(env, command_name)
    cos_ang = torch.sum(cmd.racket_normal_w * cmd.racket_target_normal_w, dim=-1).clamp(-1.0, 1.0)
    angle = torch.acos(cos_ang)
    return torch.exp(-(angle**2) / std**2) * cmd.strike_window.float()


# --- (6,7,8) no-spin return outcome, one-shot at the strike --------------------------------- #
def ball_contact(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """+1 on the strike frame when the racket actually contacts the target ball (near + approaching)."""
    return _cmd(env, command_name).ball_contact.float()


def ball_net_cross(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """+1 on the strike frame when the (no-spin) outgoing ball clears the net."""
    return _cmd(env, command_name).ball_net_cross.float()


def ball_opponent_bounce(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """+1 on the strike frame when the outgoing ball's first bounce lands on the opponent half."""
    return _cmd(env, command_name).ball_on_opponent.float()


# --- (9) in-place follow-through / recovery ------------------------------------------------- #
def follow_through_recovery(
    env: ManagerBasedRLEnv, command_name: str, std: float = 0.5, station_std: float = 0.3
) -> torch.Tensor:
    """Reward settling calmly AT the fixed station through the follow-through and the pre-swing hold.

    ``exp(-(|v_base_xy|/std)^2) * exp(-(station_err/station_std)^2) * feet_contact_frac`` active in the
    follow-through ((~pre_strike) & (~strike_window)) and during the hold. This is in-place recentring
    and balance only — it never rewards walking, footstep planning, or leaving the station.
    """
    cmd = _cmd(env, command_name)
    v_xy = torch.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    calm = torch.exp(-torch.square(v_xy / std))
    station_err = torch.norm(cmd.base_pos_w[:, :2] - cmd.fixed_station_w, dim=-1)
    at_station = torch.exp(-torch.square(station_err / station_std))
    in_hold = cmd._motion().in_hold
    gate = ((~cmd.pre_strike) & (~cmd.strike_window)) | in_hold
    return calm * at_station * cmd.feet_contact_frac * gate.float()
