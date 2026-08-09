"""HOPE racket-target observation terms.

These wrap :class:`RacketTargetCommand`. The actor (policy) group should use only the *desired*
quantities the planner provides at deploy time (HITTER actor observation, Table I):

* :func:`racket_target_pos_b`  — desired racket position relative to base (3)
* :func:`racket_target_vel_w`  — desired racket velocity in world frame (3)
* :func:`time_to_strike`       — time remaining until strike (1)
* :func:`base_target_pos_b`    — desired base XY position relative to base (2)

The desired racket *normal* and the *actual* racket state are privileged/critic-only or used by
the reward; they are intentionally NOT in the HITTER actor observation (the racket is never sensed
on hardware). :func:`swing_type` is provided for a unified forehand+backhand policy variant; the
HOPE default trains separate policies and does not need it.
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


# --- actor (policy) observations: desired targets only ------------------------------------ #
def racket_target_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Desired racket pos rel-base (yaw frame). PRIVILEGED — uses world base position (`full` mode)."""
    return _cmd(env, command_name).racket_target_pos_b()


def racket_target_pos_rel_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Desired racket pos relative to the CURRENT racket (FK), yaw frame. DEPLOY-HONEST (no world
    base position; see :meth:`RacketTargetCommand.racket_target_pos_b_rel`). Used by the deploy-parity
    actor contract (legacy task name: `real_sensor_only`). A1: reads the ACTOR-visible target view
    (delayed/jittered when target latency is on; the live tensor otherwise)."""
    return _cmd(env, command_name).racket_target_pos_b_rel()


def racket_target_vel_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Desired racket velocity, world frame. ACTOR term — A1: reads the ACTOR-visible view
    (delayed/jittered when target latency is on; the live tensor otherwise, byte-identical).
    The critic uses :func:`racket_target_vel_w_live`."""
    return _cmd(env, command_name).actor_racket_target_vel_w()


def time_to_strike(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Time remaining until the strike (s). NOT delayed by A1 target latency ON PURPOSE: the swing
    clock is generated robot-side by the deploy runner, not by the mocap link, so it carries no
    mocap/planner transport latency."""
    return _cmd(env, command_name).time_to_strike.unsqueeze(-1)


def base_target_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    return _cmd(env, command_name).base_target_pos_b()


def swing_type(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Forehand (+1) / backhand (-1). Only needed for a unified (single) policy. A1: delayed with
    the target when latency is on (the flag rides the same planner->runner message as the target)."""
    return _cmd(env, command_name).actor_swing_sign().unsqueeze(-1)


def applied_last_action(env: ManagerBasedRLEnv, action_name: str = "joint_pos") -> torch.Tensor:
    """Last raw-domain action actually applied, including passive-joint overrides.

    For FinalV3 this remains a 31-D term, but the two head slots are exactly zero because the A3
    deploy runner holds the neck at its default pose.  Refuse to fall back to Isaac Lab's global
    ``last_action``: doing so would silently restore the dropped-channel feedback integrator.
    """
    action_term = env.action_manager.get_term(action_name)
    applied = getattr(action_term, "applied_raw_actions", None)
    if applied is None:
        raise RuntimeError(
            f"Action term {action_name!r} does not expose deploy-faithful applied_raw_actions"
        )
    if applied.ndim != 2 or applied.shape[-1] != action_term.action_dim:
        raise RuntimeError(
            f"Action term {action_name!r} returned invalid applied-action shape "
            f"{tuple(applied.shape)}; expected (num_envs, {action_term.action_dim})"
        )
    return applied


def executed_qdes_feedback(
    env: ManagerBasedRLEnv, action_name: str = "joint_pos"
) -> torch.Tensor:
    """Normalized q_des actually executed on the previous tick (V15 action feedback)."""
    action_term = env.action_manager.get_term(action_name)
    feedback = getattr(action_term, "executed_qdes_feedback", None)
    if feedback is None:
        raise RuntimeError(
            f"Action term {action_name!r} does not expose executed_qdes_feedback"
        )
    if feedback.ndim != 2 or feedback.shape[-1] != action_term.action_dim:
        raise RuntimeError(
            f"Action term {action_name!r} returned invalid executed-q_des feedback shape "
            f"{tuple(feedback.shape)}; expected (num_envs, {action_term.action_dim})"
        )
    return feedback


# --- HITTER Table-I exact actor terms (hitter_pure contract, 2026-07-07) ------------------- #
# World-frame vectors + the explicit base forward vector e_base,x, exactly as the paper's actor
# observation. NOT pre-rotated into the yaw-heading frame: the policy learns the rotation itself
# (this is what lets it correct its facing toward the table — the heading-frame formulation loses
# the yaw error entirely once the reference-orientation term is removed from the actor).
def base_forward_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Base forward unit vector e_base,x (world xy, 2). Deploy: IMU + yaw-align-at-engage."""
    return _cmd(env, command_name).base_forward_xy()


def base_target_delta_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Target base position p̂_base,xy − p_base,xy (world frame, 2). Deploy: planner station −
    mocap base position (same world frame; no rotation)."""
    return _cmd(env, command_name).base_target_delta_xy_w()


def racket_target_rel_base(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Target racket position relative to the base (world frame, 3; HITTER §V-B-1). Deploy:
    planner racket target − mocap base position. A1: actor-visible (delayed/jittered) view."""
    return _cmd(env, command_name).racket_target_rel_base_w()


def base_target_delta_xy_mocap(
    env: ManagerBasedRLEnv, command_name: str
) -> torch.Tensor:
    """V15 actor station delta using the held/noisy/delayed mocap base estimate."""
    return _cmd(env, command_name).actor_base_target_delta_xy_w()


def racket_target_rel_base_mocap(
    env: ManagerBasedRLEnv, command_name: str
) -> torch.Tensor:
    """V15 actor racket target relative to the mocap-estimated base position."""
    return _cmd(env, command_name).actor_racket_target_rel_base_w()


def projected_gravity_mocap(
    env: ManagerBasedRLEnv, command_name: str
) -> torch.Tensor:
    """V17 gravity from the mocap-anchored full-pose actor view."""
    return _cmd(env, command_name).actor_projected_gravity_b()


def base_forward_xy_mocap(
    env: ManagerBasedRLEnv, command_name: str
) -> torch.Tensor:
    """V17 base forward from the same mocap quaternion as deploy."""
    return _cmd(env, command_name).actor_base_forward_xy()


def base_velocity_xy_mocap(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Filtered XY velocity differentiated from received mocap base positions."""
    return _cmd(env, command_name).actor_base_velocity_xy_w()


def base_localization_age(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Local receipt age normalized by the configured mocap stale threshold."""
    return _cmd(env, command_name).actor_base_localization_age().unsqueeze(-1)


def desired_lateral_velocity(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """HUGWBC lateral velocity command derived once from the new HITTER station (1)."""
    return _cmd(env, command_name).desired_lateral_velocity()


def gait_clock(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """HUGWBC left/right gait sine clocks (2); zero while standing."""
    return _cmd(env, command_name).gait_clock()


def locomotion_mode(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Finite locomotion mode (1=STEP, 0=latched STAND)."""
    return _cmd(env, command_name).locomotion_mode()


def upper_intervention_indicator(
    env: ManagerBasedRLEnv, action_name: str = "joint_pos"
) -> torch.Tensor:
    """Training-only HUGWBC action-replacement flag; always zero at deployment."""
    action_term = env.action_manager.get_term(action_name)
    indicator = getattr(action_term, "upper_intervention_indicator", None)
    if indicator is None:
        raise RuntimeError(
            f"Action term {action_name!r} does not expose upper_intervention_indicator"
        )
    return indicator


# --- privileged (critic) observations: desired normal + actual racket state --------------- #
def racket_target_vel_w_live(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """TRUE live desired racket velocity (world). CRITIC/privileged term: the asymmetric critic
    keeps the undegraded target even when the actor's view is delayed/jittered (A1). Identical to
    :func:`racket_target_vel_w` when the A1 knobs are off."""
    return _cmd(env, command_name).racket_target_vel_w


def racket_target_normal_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    return _cmd(env, command_name).racket_target_normal_w


def racket_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket position relative to base (FK). Privileged — not sensed on hardware."""
    cmd = _cmd(env, command_name)
    return quat_rotate_inverse(yaw_quat(cmd.base_quat_w), cmd.racket_pos_w - cmd.base_pos_w)


def racket_lin_vel_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket linear velocity (FK), world frame. Privileged."""
    return _cmd(env, command_name).racket_lin_vel_w


def racket_normal_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Actual racket face normal (FK), world frame. Privileged."""
    return _cmd(env, command_name).racket_normal_w


def episode_time_left(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Time remaining in the episode (seconds). HITTER critic privileged input."""
    # IsaacLab 2.1 calls observation terms once during ObservationManager._prepare_terms (dimension
    # probe) BEFORE ManagerBasedRLEnv allocates episode_length_buf — fall back to zeros there.
    buf = getattr(env, "episode_length_buf", None)
    if buf is None:
        return torch.zeros(env.num_envs, 1, device=env.device)
    left = (env.max_episode_length - buf).float() * env.step_dt
    return left.unsqueeze(-1)
