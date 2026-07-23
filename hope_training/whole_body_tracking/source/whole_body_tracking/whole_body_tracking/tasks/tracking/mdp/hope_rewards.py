"""HOPE reward terms.

The public task uses eleven reward terms. Eight are defined here; three (upright balance,
action smoothness, joint-limit regularization) are standard ``isaaclab.envs.mdp`` terms wired in the
env config. All example weights and kernel widths live in the env config / task YAML and are meant to
be tuned — they are illustrative, not performance-tuned values.

  1. upright / balance            -> mdp.flat_orientation_l2       (env config)
  2. forehand/backhand imitation  -> sample_imitation
  3. racket position              -> racket_position
  4. racket velocity              -> racket_velocity
  5. impact outgoing velocity     -> impact_outgoing_velocity
  6. simplified blade direction   -> racket_blade_direction
  7. soft contact proximity       -> soft_ball_contact
  8. actual ball contact          -> ball_contact
  9. net crossing                 -> ball_net_cross
 10. opponent-half first bounce   -> ball_opponent_bounce
 11. in-place follow-through/recovery -> follow_through_recovery / recovery_health
 12. action smoothness            -> mdp.action_rate_l2            (env config)
 13. joint-limit regularization   -> mdp.joint_pos_limits          (env config)

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


def wrist_motion_pos_release(
    env: ManagerBasedRLEnv,
    motion_command_name: str,
    racket_command_name: str,
    std: float,
    body_names: list[str] | None = None,
    release_window_s: float = 0.20,
) -> torch.Tensor:
    """Track the racket wrist from the clip except near impact, where the task target owns it."""
    value = _imitation.motion_relative_body_position_error_exp(env, motion_command_name, std, body_names)
    racket = _cmd(env, racket_command_name)
    release = racket.time_to_strike.abs() <= release_window_s
    return value * (~release).float()


def wrist_motion_ori_release(
    env: ManagerBasedRLEnv,
    motion_command_name: str,
    racket_command_name: str,
    std: float,
    body_names: list[str] | None = None,
    release_window_s: float = 0.20,
) -> torch.Tensor:
    """Track the racket wrist orientation from the clip except near impact."""
    value = _imitation.motion_relative_body_orientation_error_exp(env, motion_command_name, std, body_names)
    racket = _cmd(env, racket_command_name)
    release = racket.time_to_strike.abs() <= release_window_s
    return value * (~release).float()


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


def impact_outgoing_velocity(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track the predicted post-impact ball velocity against the desired outgoing velocity."""
    cmd = _cmd(env, command_name)
    error = torch.sum(torch.square(cmd.impact_ball_out_vel_w - cmd.racket_target_vel_w), dim=-1)
    return torch.exp(-error / std**2) * cmd.strike_window.float()


def racket_blade_direction(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Align the racket face normal with the desired blade direction near the strike (``std`` in rad)."""
    cmd = _cmd(env, command_name)
    cos_ang = torch.sum(cmd.racket_normal_w * cmd.racket_target_normal_w, dim=-1).clamp(-1.0, 1.0)
    angle = torch.acos(cos_ang)
    return torch.exp(-(angle**2) / std**2) * cmd.strike_window.float()


def soft_ball_contact(
    env: ManagerBasedRLEnv,
    command_name: str,
    pos_std: float = 0.18,
    approach_speed: float = 0.15,
    approach_std: float = 0.75,
    normal_speed: float = 0.0,
    normal_std: float = 0.75,
    window_s: float = 0.20,
) -> torch.Tensor:
    """Dense pre-contact shaping: be near the impact point and move into the incoming ball.

    The hard contact/net/bounce terms stay as the success metric. This term only creates a learnable
    slope around the sparse one-frame contact event, so a standing policy cannot get the contact reward
    without actually bringing the racket into the strike neighborhood.
    """
    cmd = _cmd(env, command_name)
    time_abs = cmd.time_to_strike.abs()
    gate = time_abs <= window_s
    timing = torch.exp(-torch.square(time_abs / max(window_s, 1e-6)))

    pos_err = torch.norm(cmd.racket_pos_w - cmd.racket_target_pos_w, dim=-1)
    proximity = torch.exp(-torch.square(pos_err / pos_std))

    if cmd.cfg.contact_approach_mode == "target_velocity":
        approach_dir = cmd.racket_target_vel_w / (torch.norm(cmd.racket_target_vel_w, dim=-1, keepdim=True) + 1e-6)
    else:
        to_target = cmd.racket_target_pos_w - cmd.racket_pos_w
        approach_dir = to_target / (torch.norm(to_target, dim=-1, keepdim=True) + 1e-6)
    approach = torch.sum(cmd.racket_lin_vel_w * approach_dir, dim=-1)
    approach_score = torch.sigmoid((approach - approach_speed) / approach_std)

    normal = cmd.racket_normal_w / (torch.norm(cmd.racket_normal_w, dim=-1, keepdim=True) + 1e-6)
    rel_in = cmd.incoming_ball_vel_w - cmd.racket_lin_vel_w
    closing = -torch.sum(rel_in * normal, dim=-1)
    normal_score = torch.sigmoid((closing - normal_speed) / normal_std)

    return proximity * approach_score * normal_score * timing * gate.float()


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


def recovery_health(
    env: ManagerBasedRLEnv,
    command_name: str,
    height_std: float = 0.12,
    upright_std: float = 0.35,
    lin_vel_std: float = 0.35,
    ang_vel_std: float = 1.0,
    station_std: float = 0.25,
) -> torch.Tensor:
    """Reward a deploy-ready stance after the strike and during the ready hold.

    This complements ``follow_through_recovery``: it explicitly scores base height, uprightness,
    residual base motion, station drift, and foot contact. It is gated off before the strike so it
    does not suppress the swing itself.
    """
    cmd = _cmd(env, command_name)
    data = cmd.robot.data
    in_hold = cmd._motion().in_hold
    gate = (((~cmd.pre_strike) & (~cmd.strike_window)) | in_hold).float()

    default_z = data.default_root_state[:, 2] + env.scene.env_origins[:, 2]
    height = torch.exp(-torch.square((data.root_pos_w[:, 2] - default_z) / height_std))
    upright_err = torch.norm(data.projected_gravity_b[:, :2], dim=-1)
    upright = torch.exp(-torch.square(upright_err / upright_std))
    lin = torch.exp(-torch.square(torch.norm(data.root_lin_vel_w[:, :2], dim=-1) / lin_vel_std))
    ang = torch.exp(-torch.square(torch.norm(data.root_ang_vel_w, dim=-1) / ang_vel_std))
    station_err = torch.norm(cmd.base_pos_w[:, :2] - cmd.fixed_station_w, dim=-1)
    station = torch.exp(-torch.square(station_err / station_std))
    feet = torch.clamp(cmd.feet_contact_frac, 0.0, 1.0)

    score = 0.25 * height + 0.25 * upright + 0.20 * lin + 0.15 * ang + 0.10 * station + 0.05 * feet
    return score * gate
