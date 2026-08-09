"""HOPE goal-tracking reward terms (HITTER r_goal).

These implement the racket/base target tracking rewards on top of the BeyondMimic imitation
reward (``r_imitation``, the ``motion_*`` terms already in ``rewards.py``) and the regularization
reward (``r_regularization``, ``action_rate_l2`` / ``joint_torques_l2`` / contact penalties).

Activation timing follows HITTER: the base-position reward is active **before** the strike; the
racket position/velocity/normal rewards are active only in a **short window around** the strike.
Because a ``RewardTermCfg`` weight is constant, the time gating is applied *inside* each term by
multiplying the exponential kernel by the command's ``pre_strike`` / ``strike_window`` mask.

The exponential kernel form (``exp(-error/std**2)``) mirrors the BeyondMimic motion-tracking
rewards. HITTER does not publish reward weights or kernel forms, so the weights in the env config
are HOPE choices to be tuned, not paper-sourced values.
"""

from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

from isaaclab.utils.math import quat_apply, quat_rotate_inverse, yaw_quat

from whole_body_tracking.tasks.tracking.mdp.hope_commands import RacketTargetCommand
from whole_body_tracking.tasks.tracking.mdp.recovery_safe_set import (
    actual_q_stopping_violation,
    aggregate_recovery_violations,
    normalized_upper_violation,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _cmd(env: ManagerBasedRLEnv, command_name: str) -> RacketTargetCommand:
    return env.command_manager.get_term(command_name)


def _record_metric_snapshot(metrics: dict, name: str, value: torch.Tensor) -> None:
    """Record telemetry without retaining a mutable view into plant/control state.

    IsaacLab zeros every command metric in-place during ``CommandTerm.reset``. If a metric
    directly aliases ``robot.data``, reset corrupts the actor-visible cache while leaving
    PhysX unchanged. Reuse an independent metric buffer after its first allocation.
    """

    destination = metrics.get(name)
    if (
        not torch.is_tensor(destination)
        or destination.shape != value.shape
        or destination.device != value.device
        or destination.dtype != value.dtype
    ):
        metrics[name] = value.detach().clone()
    else:
        destination.copy_(value.detach())


def _dbg_log(cmd: RacketTargetCommand, name: str, raw: torch.Tensor, mask: torch.Tensor) -> None:
    """Log raw (pre-mask) and gated (post-mask) kernel values, held over the active mask.

    No-op unless ``cmd.cfg.debug_reward_logging`` is set. The held value lets the reset-mean report the
    in-window reward, and lets you see how much reward the time-gate is killing (gated vs raw) and whether
    the raw kernel still has any gradient at the current error scale (raw ~0 => std too tight).
    """
    if not cmd.cfg.debug_reward_logging:
        return
    cmd.metrics[f"dbg_{name}_raw"] = torch.where(mask, raw, cmd.metrics[f"dbg_{name}_raw"])
    cmd.metrics[f"dbg_{name}_gated"] = torch.where(mask, raw * mask.float(), cmd.metrics[f"dbg_{name}_gated"])


def _position_guidance_gate(cmd: RacketTargetCommand) -> torch.Tensor:
    """Return the task-local position gate, or the legacy strike gate when unset."""
    window_s = float(getattr(cmd.cfg, "position_guidance_window_s", 0.0))
    if window_s <= 0.0:
        return cmd.strike_window
    return cmd.time_to_strike.abs() <= window_s + 1.0e-6


def _moving_position_kernel(
    cmd: RacketTargetCommand, std: float
) -> torch.Tensor:
    """Moving-target Gaussian without a time gate or temporal scale."""
    target_pos_now = (
        cmd.racket_target_pos_w
        - cmd.racket_target_vel_w * cmd.time_to_strike.unsqueeze(-1)
    )
    error = torch.sum(torch.square(cmd.racket_pos_w - target_pos_now), dim=-1)
    return torch.exp(-error / std**2)


def racket_position_tracking_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track the local moving strike trajectory with task-configured timing and scale.

    The moving-target sign remains ``p_strike - v_target * tts``. RallyV15 narrows only this
    position-derived gate to ±0.04 s and applies a temporal scale of 2.0; legacy tasks without
    those command keys retain their full strike window and unit scale.
    """
    cmd = _cmd(env, command_name)
    raw = _moving_position_kernel(cmd, std)
    gate = _position_guidance_gate(cmd)
    temporal_scale = float(
        getattr(cmd.cfg, "position_guidance_temporal_scale", 1.0)
    )
    _dbg_log(cmd, "racket_pos", raw, gate)
    return temporal_scale * raw * gate.float()


def racket_position_tracking_static_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Ablation B: track the strike POINT itself (no swing-through), decoupling position from timing/velocity.

    Identical gating to ``racket_position_tracking_exp`` but compares against the bare ``racket_target_pos_w``
    instead of the moving swing-through point ``target - vel*t_to_strike``. Over a ±0.15 s window the
    swing-through point sweeps up to ~0.9 m at a 6 m/s target, so the standard term mostly rewards being on
    the moving line (timing/velocity); this variant gives a clean "get the paddle to the point" signal for
    early stable positioning. Select via ``rewards.racket_position_static: true`` in the task YAML.
    """
    cmd = _cmd(env, command_name)
    error = torch.sum(torch.square(cmd.racket_pos_w - cmd.racket_target_pos_w), dim=-1)
    raw = torch.exp(-error / std**2)
    _dbg_log(cmd, "racket_pos", raw, cmd.strike_window)
    return raw * cmd.strike_window.float()


def racket_velocity_tracking_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track racket linear velocity near the strike time (FK actual vs desired, world frame)."""
    cmd = _cmd(env, command_name)
    error = torch.sum(torch.square(cmd.racket_lin_vel_w - cmd.racket_target_vel_w), dim=-1)
    raw = torch.exp(-error / std**2)
    _dbg_log(cmd, "racket_vel", raw, cmd.strike_window)
    return raw * cmd.strike_window.float()


def racket_preimpact_velocity_huber_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    preimpact_s: float = 0.30,
    margin: float = 0.50,
    huber_scale: float = 0.50,
) -> torch.Tensor:
    """Non-saturating velocity teacher before contact.

    The desired velocity follows the clean per-phase clip velocity at the beginning of the
    window, then smoothly hands ownership to the sampled target at impact.  Error inside the
    exact-strike acceptance margin is free; outside it, smooth-L1 retains a linear tail where
    the exponential contact reward becomes weak.  Configure with a negative reward weight.
    """

    if preimpact_s <= 0.0 or margin < 0.0 or huber_scale <= 0.0:
        raise ValueError(
            "preimpact velocity debt requires preimpact_s>0, margin>=0 and "
            f"huber_scale>0; got {preimpact_s}/{margin}/{huber_scale}"
        )
    cmd = _cmd(env, command_name)
    phase = ((float(preimpact_s) - cmd.time_to_strike) / float(preimpact_s)).clamp(
        0.0, 1.0
    )
    blend = phase * phase * (3.0 - 2.0 * phase)
    reference_velocity = cmd.reference_racket_velocity_w()
    desired_velocity = torch.lerp(
        reference_velocity,
        cmd.racket_target_vel_w,
        blend.unsqueeze(-1),
    )
    error_vector = cmd.racket_lin_vel_w - desired_velocity
    error = torch.linalg.norm(error_vector, dim=-1)
    scaled = torch.clamp(error - float(margin), min=0.0) / float(huber_scale)
    debt = torch.where(
        scaled <= 1.0,
        0.5 * torch.square(scaled),
        scaled - 0.5,
    )
    motion = cmd._motion()
    gate = (
        (~motion.in_hold)
        & (cmd.time_to_strike >= -1.0e-6)
        & (cmd.time_to_strike <= float(preimpact_s) + 1.0e-6)
    )
    gate_f = gate.float()
    cmd.metrics["preimpact_velocity_active"] = gate_f
    cmd.metrics["preimpact_velocity_blend"] = blend * gate_f
    cmd.metrics["preimpact_velocity_error"] = error * gate_f
    cmd.metrics["preimpact_velocity_debt"] = debt * gate_f
    cmd.metrics["preimpact_reference_racket_speed"] = (
        torch.linalg.norm(reference_velocity, dim=-1) * gate_f
    )
    cmd.metrics["preimpact_desired_racket_speed"] = (
        torch.linalg.norm(desired_velocity, dim=-1) * gate_f
    )
    for axis_index, axis in enumerate(("x", "y", "z")):
        cmd.metrics[f"preimpact_velocity_error_{axis}"] = (
            error_vector[:, axis_index] * gate_f
        )
    return debt * gate_f


def racket_normal_tracking_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track racket face-normal orientation near the strike time. ``std`` is in radians."""
    cmd = _cmd(env, command_name)
    cos_ang = torch.sum(cmd.racket_normal_w * cmd.racket_target_normal_w, dim=-1).clamp(-1.0, 1.0)
    angle = torch.acos(cos_ang)
    raw = torch.exp(-(angle**2) / std**2)
    _dbg_log(cmd, "racket_normal", raw, cmd.strike_window)
    return raw * cmd.strike_window.float()


def base_position_tracking_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """Track desired base XY position before the strike (encourages repositioning footwork)."""
    cmd = _cmd(env, command_name)
    error = torch.sum(torch.square(cmd.base_pos_w[:, :2] - cmd.base_target_pos_w), dim=-1)
    raw = torch.exp(-error / std**2)
    _dbg_log(cmd, "base", raw, cmd.pre_strike)
    return raw * cmd.pre_strike.float()


# ============================================================================================== #
# V15 LOWER-BODY FOUNDATION — HUGWBC command/contact/intervention design adapted to HITTER.
# HITTER still supplies the world-frame station and the racket strike objective.  These terms only
# supervise the lower body from a finite velocity/gait command and keep its balance terms active
# under the full upper-body swing.  Only the displacement clock is pre-swing; it does not
# continuously servo residual station error and therefore cannot reward endless micro-steps.
# ============================================================================================== #
def hugwbc_lateral_velocity_tracking_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    locomotion_only: bool = False,
) -> torch.Tensor:
    """Track finite world-Y motion, then zero velocity through swing and recovery."""
    cmd = _cmd(env, command_name)
    desired = cmd.desired_lateral_velocity().squeeze(-1)
    error = torch.square(cmd.robot.data.root_lin_vel_w[:, 1] - desired)
    supervision = (
        cmd.locomotion_supervision()
        if locomotion_only
        else cmd.balance_supervision()
    )
    return torch.exp(-error / float(std) ** 2) * supervision.float()


def hugwbc_yaw_rate_tracking_exp(
    env: ManagerBasedRLEnv, command_name: str, std: float
) -> torch.Tensor:
    """Track zero base yaw rate through locomotion and upper-body motion, as in HUGWBC."""
    cmd = _cmd(env, command_name)
    error = torch.square(cmd.robot.data.root_ang_vel_b[:, 2])
    return torch.exp(-error / float(std) ** 2) * cmd.balance_supervision().float()


def hugwbc_finite_station_tracking_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    stand_deadband: float,
) -> torch.Tensor:
    """HITTER station score without post-cycle position-servo gradients.

    While STEP is active the smooth HITTER position score helps select the correct displacement.
    When the finite gait ends it becomes a binary terminal neighbourhood score: a residual error
    cannot generate another sequence of shrinking corrective actions.
    """
    cmd = _cmd(env, command_name)
    error = torch.linalg.norm(cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1)
    move = cmd.locomotion_mode().squeeze(-1) > 0.5
    supervision = cmd.locomotion_supervision()
    smooth = torch.exp(-torch.square(error / float(std)))
    # STAND income reads the LATCHED command error (plan-time for STAND, gait-completion for
    # STEP), never the live error: a live binary paid 1.0/step for shuffling back inside the
    # deadband — exactly the micro-step farming V15 forbids (audit 2026-07-23).
    stand_success = (cmd.finite_station_latched_error() <= float(stand_deadband)).float()
    cmd.metrics["finite_station_error"] = error
    cmd.metrics["finite_station_success"] = stand_success * supervision.float()
    return torch.where(move, smooth, stand_success) * supervision.float()


def hugwbc_contact_force_tracking(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg,
    force_sigma: float = 50.0,
    locomotion_only: bool = False,
) -> torch.Tensor:
    """HUGWBC shaped penalty for force on a foot commanded to swing."""
    cmd = _cmd(env, command_name)
    sensor = env.scene.sensors[sensor_cfg.name]
    force = torch.linalg.norm(
        sensor.data.net_forces_w[:, sensor_cfg.body_ids, :], dim=-1
    )
    desired = cmd.desired_contact_states()
    debt = (1.0 - desired) * (1.0 - torch.exp(-torch.square(force) / float(force_sigma)))
    supervision = (
        cmd.locomotion_supervision()
        if locomotion_only
        else cmd.balance_supervision()
    )
    return debt.mean(dim=-1) * supervision.float()


def hugwbc_contact_velocity_tracking(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    velocity_sigma: float = 5.0,
    locomotion_only: bool = False,
) -> torch.Tensor:
    """HUGWBC shaped penalty for motion of a foot commanded to be in stance."""
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    speed = torch.linalg.norm(
        asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :], dim=-1
    )
    desired = cmd.desired_contact_states()
    debt = desired * (1.0 - torch.exp(-torch.square(speed) / float(velocity_sigma)))
    supervision = (
        cmd.locomotion_supervision()
        if locomotion_only
        else cmd.balance_supervision()
    )
    return debt.mean(dim=-1) * supervision.float()


def hugwbc_standing_double_contact(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg,
    force_threshold: float = 10.0,
    true_stand_only: bool = False,
) -> torch.Tensor:
    """Positive double-support reward in STAND mode (released HUGWBC `standing`)."""
    cmd = _cmd(env, command_name)
    sensor = env.scene.sensors[sensor_cfg.name]
    force = torch.linalg.norm(
        sensor.data.net_forces_w[:, sensor_cfg.body_ids, :], dim=-1
    )
    both = torch.all(force > float(force_threshold), dim=-1)
    mode = cmd.locomotion_mode().squeeze(-1)
    stand_mode = mode.abs() < 0.5 if true_stand_only else mode < 0.5
    stand = cmd.balance_supervision() & stand_mode
    return (both & stand).float()


def hugwbc_standing_air(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg,
    force_threshold: float = 10.0,
    true_stand_only: bool = False,
) -> torch.Tensor:
    """Penalty when neither foot is loaded in STAND mode (released HUGWBC `standing_air`)."""
    cmd = _cmd(env, command_name)
    sensor = env.scene.sensors[sensor_cfg.name]
    force = torch.linalg.norm(
        sensor.data.net_forces_w[:, sensor_cfg.body_ids, :], dim=-1
    )
    airborne = torch.all(force <= float(force_threshold), dim=-1)
    mode = cmd.locomotion_mode().squeeze(-1)
    stand_mode = mode.abs() < 0.5 if true_stand_only else mode < 0.5
    stand = cmd.balance_supervision() & stand_mode
    return (airborne & stand).float()


def hugwbc_orientation_control(
    env: ManagerBasedRLEnv, command_name: str, swing_scale: float = 0.25
) -> torch.Tensor:
    """HUGWBC projected-gravity control, retained weakly through the racket swing."""
    cmd = _cmd(env, command_name)
    projected = cmd.robot.data.projected_gravity_b
    debt = torch.sum(torch.square(projected[:, :2]), dim=-1)
    hold_scale = cmd.locomotion_supervision().float()
    scale = torch.where(hold_scale > 0.5, torch.ones_like(debt), torch.full_like(debt, float(swing_scale)))
    return debt * scale


def hugwbc_waist_yaw_roll_control(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg, swing_supervised: bool = False
) -> torch.Tensor:
    """Prevent the V14 waist-twist locomotion shortcut.

    ``swing_supervised=False`` keeps the original hold-only mask (``locomotion_supervision``).

    ``swing_supervised=True`` uses ``balance_supervision`` instead, so the waist is also charged
    through the wind-up, the contact and the recovery. The 2026-07-25 audit found this term was
    the ONLY waist penalty in V15 and that it was switched off for exactly the phase in which
    ``waist_roll`` was being driven into its mechanical hard stop: 31.4% of control steps sat
    inside the near-limit band, 12.1% crossed the lower hard limit, 85.4% of all joint hard-limit
    violations in the run came from this one joint, and 12-13% of episodes terminated on the
    resulting post-physics audit. ``balance_supervision``'s own docstring already states the
    principle this restores — "balance must not disappear when the racket wind-up starts" — it
    simply had not been applied to the waist.
    """
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    default = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    mask = (
        cmd.balance_supervision() if swing_supervised else cmd.locomotion_supervision()
    )
    return torch.mean(torch.square(q - default), dim=-1) * mask.float()


def hugwbc_base_height_control(
    env: ManagerBasedRLEnv,
    command_name: str,
    target_height: float,
    stand_scale: float = 3.0,
) -> torch.Tensor:
    """HUGWBC root-height control through locomotion, swing and recovery."""
    cmd = _cmd(env, command_name)
    error = torch.square(cmd.robot.data.root_pos_w[:, 2] - float(target_height))
    balance = cmd.balance_supervision()
    # locomotion_mode is -1 through the swing; a plain `mode < 0.5` gate charged the x3 STAND
    # scale against the legitimate strike crouch (audit 2026-07-23).  Only true latched STAND
    # (mode == 0) earns the tightened scale; swing/recovery pay the base weight.
    stand = balance & (cmd.locomotion_mode().squeeze(-1).abs() < 0.5)
    scale = torch.where(stand, torch.full_like(error, float(stand_scale)), torch.ones_like(error))
    return error * scale * balance.float()


def hugwbc_stand_still_foot_placement(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    stance_width: float,
) -> torch.Tensor:
    """Released HUGWBC Raibert-style foot placement cost, only after finite STEP ends."""
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    feet_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    rel_w = feet_w - cmd.base_pos_w.unsqueeze(1)
    heading = yaw_quat(cmd.base_quat_w).unsqueeze(1).expand(-1, rel_w.shape[1], -1)
    feet_b = quat_rotate_inverse(heading.reshape(-1, 4), rel_w.reshape(-1, 3)).reshape_as(rel_w)
    desired_y = torch.tensor(
        [0.5 * float(stance_width), -0.5 * float(stance_width)],
        device=feet_b.device,
        dtype=feet_b.dtype,
    ).unsqueeze(0)
    debt = torch.sum(torch.square(feet_b[:, :, 1] - desired_y), dim=-1)
    # True STAND only (mode == 0): charging the +/-half-stance-width template through the swing
    # (mode -1) taxed the staggered strike stance (audit 2026-07-23).
    stand = cmd.balance_supervision() & (cmd.locomotion_mode().squeeze(-1).abs() < 0.5)
    return debt * stand.float()


def hugwbc_feet_clearance(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    target_height: float = 0.08,
) -> torch.Tensor:
    """HUGWBC swing-foot clearance shaping for the finite STEP (released feet_clearance form).

    The audit (2026-07-23) found v15 ported the contact-schedule rewards but omitted the paper's
    clearance term entirely, leaving nothing that rewards lifting the swing foot — and
    low-clearance shuffle-stepping is precisely the v14 hardware failure this redesign targets.
    Height is measured RELATIVE to the lower foot (both bodies are the ankle links, so the mount
    offset cancels and no ground-height calibration is needed); the (1 - desired_contact) gate
    charges only the commanded swing foot; active only while the finite STEP runs.
    """
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]
    reference = foot_z.min(dim=-1, keepdim=True).values
    clearance = foot_z - reference
    desired = cmd.desired_contact_states()
    debt = torch.sum(
        (1.0 - desired) * torch.square(clearance - float(target_height)), dim=-1
    )
    move = cmd.locomotion_mode().squeeze(-1) > 0.5
    cmd.metrics["feet_clearance_swing_height"] = torch.where(
        move, (clearance * (1.0 - desired)).max(dim=-1).values,
        torch.zeros_like(debt),
    )
    return debt * move.float()


def swing_arm_joint_imitation(
    env,
    command_name: str,
    action_name: str,
    std: float,
    joint_names,
    racket_command_name: str = "racket_target",
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """Joint-space racket-arm anchor through the SWING (audit 2026-07-23 twisted-arm finding).

    The Cartesian imitation terms average their error over 7 tracked bodies inside the exponent,
    so during the strike window the 14/14/5 racket goals could buy a contorted elbow/wrist that
    still presents the correct racket point.  This term scores the arm JOINT configuration
    against the clip reference at the matched phase, exactly zero during holds (the hold
    deviation term owns that regime), with intervention-owned channels masked per joint — the
    never-replaced wrists therefore stay supervised through the intervened follow-through too.
    Positive exp kernel; configure with a positive weight.

    STRIKE-WINDOW EXEMPTION (2026-07-23 aiming-budget fix): paying this income through the
    contact window pins the arm to the CLIP's strike point, and v15 samples targets OFF the clip
    manifold (x 0.58 vs natural 0.70 reach, 0.45 m z band) — the pinned arm then cannot adapt
    and pos_err floors at ~0.5 m.  Inside [-follow_through_free_s, +strike_free_pre_s] around
    impact this term pays exactly zero (same convention as executed_qdes_difference_l2), so the
    aiming gradient owns the contact while the swing body keeps its anti-twist anchor.
    """
    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("swing_arm strike/follow-through windows must be non-negative")
    cmd = env.command_manager.get_term(command_name)
    action_term = env.action_manager.get_term(action_name)
    resolved_action_names = list(getattr(action_term, "_joint_names", ()))
    requested = tuple(str(name) for name in joint_names)
    missing = [name for name in requested if name not in resolved_action_names]
    if not requested or missing:
        raise RuntimeError(
            "swing_arm_joint_imitation requires non-empty exact action joint names; "
            f"missing={missing}"
        )
    action_cols = torch.tensor(
        [resolved_action_names.index(name) for name in requested],
        dtype=torch.long,
        device=cmd.device,
    )
    action_joint_ids = getattr(action_term, "_action_joint_ids", None)
    if not torch.is_tensor(action_joint_ids):
        raise RuntimeError(
            f"Action term {action_name!r} does not expose articulation joint ids"
        )
    joint_ids = action_joint_ids.index_select(0, action_cols)
    ref = cmd.joint_pos.index_select(-1, joint_ids)
    q = cmd.robot.data.joint_pos.index_select(-1, joint_ids)
    squared_error = torch.square(q - ref)

    supervised = torch.ones_like(squared_error)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(action_term, "_upper_intervention_effective", None)
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        intervened_joint = (
            action_cols.unsqueeze(-1) == intervention_cols.unsqueeze(0)
        ).any(dim=-1)
        supervised = torch.where(
            intervention_effective.unsqueeze(-1) & intervened_joint.unsqueeze(0),
            torch.zeros_like(supervised),
            supervised,
        )

    err = (squared_error * supervised).sum(dim=-1) / supervised.sum(dim=-1).clamp_min(1.0)
    r = torch.exp(-err / float(std) ** 2)
    cmd.metrics["swing_arm_joint_error_rms"] = torch.sqrt(err)
    racket = env.command_manager.get_term(racket_command_name)
    in_contact_window = (
        (racket.time_to_strike <= float(strike_free_pre_s))
        & (racket.time_to_strike >= -float(follow_through_free_s))
    )
    live = ~cmd.in_hold & ~in_contact_window
    return torch.where(live, r, torch.zeros_like(r))


def swing_arm_joint_huber_debt(
    env,
    command_name: str,
    action_name: str,
    joint_names,
    racket_command_name: str = "racket_target",
    release_pre_s: float = 0.30,
    margin: float = 0.10,
    huber_scale: float = 0.35,
) -> torch.Tensor:
    """Active racket-arm smooth-L1 teacher during the early released swing.

    The existing Gaussian arm anchor remains the near-reference objective, but at the observed
    roughly one-radian error its gradient is effectively absent.  This debt keeps a linear tail
    until ``release_pre_s`` before impact, then turns off so the sampled Cartesian/velocity target
    owns the final acceleration.  Configure with a negative reward weight.
    """

    if release_pre_s < 0.0 or margin < 0.0 or huber_scale <= 0.0:
        raise ValueError(
            "swing-arm Huber debt requires release_pre_s>=0, margin>=0 and "
            f"huber_scale>0; got {release_pre_s}/{margin}/{huber_scale}"
        )
    cmd = env.command_manager.get_term(command_name)
    action_term = env.action_manager.get_term(action_name)
    resolved_action_names = list(getattr(action_term, "_joint_names", ()))
    requested = tuple(str(name) for name in joint_names)
    missing = [name for name in requested if name not in resolved_action_names]
    if not requested or missing:
        raise RuntimeError(
            "swing_arm_joint_huber_debt requires non-empty exact action joint "
            f"names; missing={missing}"
        )
    action_cols = torch.tensor(
        [resolved_action_names.index(name) for name in requested],
        dtype=torch.long,
        device=cmd.device,
    )
    action_joint_ids = getattr(action_term, "_action_joint_ids", None)
    if not torch.is_tensor(action_joint_ids):
        raise RuntimeError(
            f"Action term {action_name!r} does not expose articulation joint ids"
        )
    joint_ids = action_joint_ids.index_select(0, action_cols)
    reference = cmd.joint_pos.index_select(-1, joint_ids)
    actual = cmd.robot.data.joint_pos.index_select(-1, joint_ids)
    absolute_error = torch.abs(actual - reference)

    supervised = torch.ones_like(absolute_error)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(
        action_term, "_upper_intervention_effective", None
    )
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        intervened_joint = (
            action_cols.unsqueeze(-1) == intervention_cols.unsqueeze(0)
        ).any(dim=-1)
        supervised = torch.where(
            intervention_effective.unsqueeze(-1)
            & intervened_joint.unsqueeze(0),
            torch.zeros_like(supervised),
            supervised,
        )

    scaled = torch.clamp(absolute_error - float(margin), min=0.0) / float(
        huber_scale
    )
    per_joint = torch.where(
        scaled <= 1.0,
        0.5 * torch.square(scaled),
        scaled - 0.5,
    )
    denominator = supervised.sum(dim=-1).clamp_min(1.0)
    debt = (per_joint * supervised).sum(dim=-1) / denominator
    rms = torch.sqrt(
        (torch.square(actual - reference) * supervised).sum(dim=-1)
        / denominator
    )
    racket = env.command_manager.get_term(racket_command_name)
    gate = (~cmd.in_hold) & (
        racket.time_to_strike > float(release_pre_s) + 1.0e-6
    )
    gate_f = gate.float()
    racket.metrics["swing_arm_huber_active"] = gate_f
    racket.metrics["swing_arm_huber_debt"] = debt * gate_f
    racket.metrics["swing_arm_huber_error_rms"] = rms * gate_f
    return debt * gate_f


def post_strike_brake(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    """POSITIVE braking reward through the FOLLOW-THROUGH (2026-07-07 continuous-rally upgrade).

    Deploy P7 failure mode: the walk-and-strike lunge carries base momentum past the strike; with
    nothing positive active in the tts<0 segment (every goal term is pre_strike/strike_window gated)
    the policy has no incentive to arrest it, and over consecutive swings the displacement
    accumulates until a swing starts from an untrained stance and falls. This term pays
    ``exp(-(|v_base_xy|/std)^2)`` ONLY in the follow-through window::

        (~pre_strike) & (~strike_window)

    i.e. from strike-window EXIT (tts < -strike_window_s) to the clip wrap — it can never touch the
    strike itself (the swing's through-speed is strike_window-protected), and on the wrap step tts
    snaps positive for the next swing so the window closes exactly at the wrap. During a post-wrap
    HOLD, ``pre_strike`` is True (the hold freezes tts positive at the windup value), so braking
    there is ``hold_ready``'s job (stillness x planted feet), not this term's. The window length is
    clip-clocked (not policy-controllable), so the bounded positive income cannot be farmed by
    prolonging it. Deliberately NO position target here: pulling toward any station mid-follow-
    through fights the swing's natural momentum sink — position homing is ``base_position``'s job
    once the next station appears at the wrap.
    """
    cmd = _cmd(env, command_name)
    v_xy = torch.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    raw = torch.exp(-torch.square(v_xy / std))
    gate = (~cmd.pre_strike) & (~cmd.strike_window)
    _dbg_log(cmd, "post_strike_brake", raw, gate)
    return raw * gate.float()


def lower_body_plant_imitation(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg, std: float = 0.5, tts_std: float = 0.25
) -> torch.Tensor:
    """POSITIVE lower-body imitation, active ONLY near the strike, ramping in as tts→0 (2026-07-09).

    The paper (§V-A: B = above pelvis) deliberately does NOT imitate the lower body, so the legs are
    reward-free and — measured on model_21500 in the AGI-plant G3 — flail during the swing
    (leg-joint speed ~2.85 rad/s in the strike window vs ~1.5 during holds) while the base yaws and
    lunges ("下半身一直在飘"). But that lower-body freedom is only NEEDED during the APPROACH (y-footwork
    to a varying station); AT the strike the stance should be planted like the demo, whose retargeted
    legs ARE a stable, weight-transferred plant.

    This term tracks the reference lower-body JOINT pose (``exp(-‖q_legs − ref_legs‖² / std²)``, ref =
    the hold-aware clip pose ``cmd.joint_pos``, the same source foot_orientation reads) times a tts
    RAMP ``exp(-(tts / tts_std)²)`` that peaks at the strike (tts=0) and is ~zero beyond ~±2·tts_std s.
    So it is essentially OFF through the footwork approach (large +tts) and the holds — the y-footwork
    and the recovery holds stay FREE (this is deliberate: too-early/too-strong a plant reward would
    stop the robot stepping in y, the user's explicit worry) — and ramps to full only in the last
    ~2·tts_std s before contact, teaching a demo-matched planted stance exactly when the flailing
    hurts. It tracks JOINT angles (leg configuration ⇒ feet pose RELATIVE to the base), NOT global
    base xy — the base sits wherever the footwork put it; only the legs settle to the demo (which also
    curbs the lunge kinematically: planted demo legs + feet cannot translate the base far). Weight
    POSITIVE and SMALL — it must never out-vote the racket strike terms OR the pre-strike footwork;
    reference is hold-zeroed so it is a safe no-op on tasks without the clip machinery.
    """
    cmd = _cmd(env, command_name)  # racket_target: holds the tts clock (time_to_strike)
    asset = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    # The hold-aware reference joint pose lives on the MOTION command (same source foot_orientation
    # reads); the RacketTargetCommand exposes it via _motion() (as hold_ready reaches in_hold).
    ref = cmd._motion().joint_pos[:, asset_cfg.joint_ids]
    # MEAN (per-joint) squared error, NOT sum: the 12-joint SUM saturates exp(-err/std²) to a dead 0
    # (the footwork legs sit well off the demo's single-clip pose), which killed the gradient entirely
    # in the first run (2026-07-09_17-09-18: Episode_Reward flat 0.0). Mean makes std joint-count-
    # independent and gives a live gradient at the actual deviation (~0.5 reward, rising as legs settle).
    err = torch.mean(torch.square(q - ref), dim=1)
    ramp = torch.exp(-torch.square(cmd.time_to_strike / tts_std))
    return torch.exp(-err / std**2) * ramp


def pre_strike_foot_slip(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize horizontal foot speed WHILE the foot is in contact, BEFORE the strike only.

    The robot was sliding/leaning to reach far racket targets while the base reward pinned it near spawn
    (foot_slip_speed high, foot_contact_frac low). This term teaches it to plant its feet and stabilize
    during the approach. It is gated by ``pre_strike`` ONLY (not ``strike_window``), so the strike swing's
    footwork is untouched. ``foot_slip_in_contact`` (sum over feet of horizontal speed * in_contact) is
    precomputed by the RacketTargetCommand each step (0 if the feet/contact sensor cannot be resolved).
    Returns a positive magnitude; the RewTerm weight is negative.
    """
    cmd = _cmd(env, command_name)
    return cmd.foot_slip_in_contact * cmd.pre_strike.float()


# ============================================================================================== #
# Footwork-to-strike (BASE-FREE). The legs move because moving the body REDUCES the racket->target
# distance (racket_progress), not because they track a base target. Footwork is penalized for being
# BAD (slip / drag / violent / unstable at strike), NOT for stepping — the feet are free to move.
# ============================================================================================== #
def racket_progress(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """DENSE pre-strike reward for reducing the racket->target distance (prev - current, clamped). This
    is the base-free driver of whole-body footwork: the legs/waist/arms all get credit for moving the
    racket closer to the target, with NO base-position target. Gated to pre_strike (approach phase); the
    strike swing itself is scored by the racket pos/vel/normal terms. Positive when approaching; RewTerm
    weight is POSITIVE."""
    cmd = _cmd(env, command_name)
    position_window_s = float(
        getattr(cmd.cfg, "position_guidance_window_s", 0.0)
    )
    if position_window_s > 0.0:
        approach_gate = cmd.time_to_strike > position_window_s + 1.0e-6
    else:
        approach_gate = cmd.pre_strike
    return cmd.racket_progress * approach_gate.float()


def hold_ready(
    env: ManagerBasedRLEnv, command_name: str, std: float, reach: float = 0.65, reach_mode: str = "racket",
    include_ang_vel: bool = True, heading_gate: float = 0.0,
) -> torch.Tensor:
    """POSITIVE ready-stance reward during the pre-swing HOLD (the between-swing recovery phase).

    HITTER's balance recovery comes from a positive "prepare for the next target" signal (its pre-strike
    base-position reward), not from balance penalties. In the base-free deploy-parity design the hold
    phase (reference frozen at the next swing's first frame) already pulls the UPPER body to the ready
    pose via imitation, but the legs/base get zero positive signal — only penalties. This term fills that
    gap without a base-position target (deploy-honest: everything here is proprioceptive in spirit —
    stillness + planted feet): ``exp(-(|v_base|^2 + |w_base|^2)/std^2) * feet_contact_frac``, gated to
    the motion command's ``in_hold`` mask. Rewards arriving at the next windup calm, upright-by-stillness
    and with both feet planted — i.e. finishing the previous swing in a recoverable state.

    ``reach`` gate: stillness is only the CORRECT ready action when the robot is already where it can
    strike from. Without the gate this term pays ~weight/step for planted stillness, which out-earns the
    telescoping racket_progress for stepping during the hold — i.e. it would teach freeze-then-rush
    exactly when wide target boxes need footwork. Two gate modes (``reach_mode``):

    * ``"racket"`` (legacy default, base-free tasks): ``racket_target_distance < reach`` — the 3D
      FK-blade->target distance. CAVEAT (2026-07-05 footwork audit): this gate is NOT
      station-selective — the blade distance is arm-pose-controllable (arm imitation is swing-only,
      so reaching toward the target during the hold is reward-free), and for near-side targets it is
      SMALLER at the wrong station than at the correct one, inverting the settle income exactly where
      a step is required. Keep it only for base-free tasks that have no meaningful station.
    * ``"station"`` (HITTER footwork tasks): ``|base_xy − base_target_xy| < reach`` — the planar
      base->commanded-station error. Station-selective by construction and not arm-gameable: far
      station -> the term is silent (base_position/racket_progress drive the step, untaxed);
      arrived -> the stillness income switches on (move to the stance, THEN settle, then swing).

    Zero outside the hold (the swing itself is untouched) and a safe no-op if the motion command has
    no hold state. RewTerm weight is POSITIVE.
    """
    cmd = _cmd(env, command_name)
    in_hold = getattr(cmd._motion(), "in_hold", None)
    if in_hold is None:
        return torch.zeros(cmd.num_envs, device=cmd.device)
    data = cmd.robot.data
    # ``include_ang_vel=False`` drops the angular-velocity term so the settle income does NOT reward a
    # ZERO yaw-rate (2026-07-09): pairing hold_ready (base-position settle) with hold_heading (re-squaring
    # turn) otherwise conflicts — hold_ready would tax exactly the yaw motion hold_heading teaches. With it
    # off, hold_ready settles only the LINEAR base velocity (position/stillness), leaving the turn to
    # hold_heading. Default True keeps the legacy full stillness kernel for every existing task.
    motion_sq = torch.sum(torch.square(data.root_lin_vel_w), dim=-1)
    if include_ang_vel:
        motion_sq = motion_sq + torch.sum(torch.square(data.root_ang_vel_w), dim=-1)
    raw = torch.exp(-motion_sq / std**2) * cmd.feet_contact_frac
    if reach_mode == "station":
        station_err = torch.norm(cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1)
        near = (station_err < reach).float()
    elif reach_mode == "racket":
        near = (cmd.racket_target_distance < reach).float()
    else:
        raise ValueError(f"hold_ready: unknown reach_mode '{reach_mode}' (expected 'racket' or 'station')")
    # NEAR-SQUARE HEADING GATE (2026-07-11 RallyV8): pay the settle income only once |base yaw|
    # (vs the world +x training heading, same convention as hold_heading) is inside the gate.
    # This is the memory-prescribed third way for the idle-feet problem: include_ang_vel=True
    # keeps the full angular-stillness kernel (V5's include_ang_vel=False let the base SPIN at
    # 0.69 rad/s during holds), while the gate keeps that kernel from taxing the hold_heading
    # re-squaring turn — while still yawed the income is simply silent and hold_heading owns the
    # gradient; once square, full stillness (linear + angular + feet) is paid. 0.0 = off (legacy).
    if heading_gate > 0.0:
        q = cmd.base_quat_w
        fwd_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
        fwd_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
        yaw = torch.atan2(fwd_y, fwd_x)
        raw = raw * (yaw.abs() < heading_gate).float()
    return raw * near * in_hold.float()


def hold_heading(env: ManagerBasedRLEnv, command_name: str, std: float = 0.6) -> torch.Tensor:
    """POSITIVE heading-restoration reward during the pre-swing HOLD (rally-gate finding 2026-07-08).

    The deploy rally gate measured swing follow-throughs leaving the robot 30-55° off the world
    +x strike heading (execution over-rotation; the reference clips yaw <=±21° mid-swing and END
    at ~0-6°), and NOTHING teaches recovery: base_position is position-only, hold_ready is
    stillness x planted-feet, motion_global_anchor_ori's exp kernel is gradient-dead at 40°+,
    and — decisively — no trained state is ever yawed (RSI pose_range carries no yaw noise and
    stand starts are exactly square), so "yawed -> turn back" was never in the data. Deploy-side
    the C++ runner must gate engages on heading (`PLANNER: yawed`) and wait for an operator
    re-stand — the annoyance this term exists to remove.

    ``exp(-yaw^2/std^2) * in_hold`` with yaw = the base x-axis heading in the world XY plane
    (0 == world +x, the strike heading every swing starts from in training). Pair it with
    ``motion.stand_start_yaw_range`` so yawed hold states actually appear in training (the
    reward alone has nothing to learn from). std 0.6 rad ON PURPOSE (design review 2026-07-08):
    a tight 0.35 kernel is itself gradient-dead across the injected ±0.7 rad band (exp(-4)=2%
    income, slope 0.21/rad at the edge — the exact pathology it replaces), while 0.6 keeps
    r(40°)=0.26 with slope ~1.0/rad and peaks its pull at ~24° — right at the deploy engage
    gate. No reach gate: restoring heading is never the wrong action during a hold, and it
    composes with base_position's station pull (turn while walking home). Zero outside the
    hold (the swing's natural ±20° yaw excursion is untouched); weight POSITIVE; safe no-op
    if the motion command has no hold state.
    """
    cmd = _cmd(env, command_name)
    in_hold = getattr(cmd._motion(), "in_hold", None)
    if in_hold is None:
        return torch.zeros(cmd.num_envs, device=cmd.device)
    q = cmd.base_quat_w  # (w, x, y, z)
    # world-frame base forward = R(q) @ x_hat; heading = atan2(fwd_y, fwd_x)
    fwd_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    fwd_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw = torch.atan2(fwd_y, fwd_x)
    return torch.exp(-torch.square(yaw) / std**2) * in_hold.float()


def base_decel_tracking(
    env: ManagerBasedRLEnv, command_name: str, v_gain: float = 2.0, v_max: float = 1.6, std: float = 0.4
) -> torch.Tensor:
    """P2.4 PACE-style smooth-deceleration shaping: track a pseudo base-velocity command that decays
    with the remaining planar racket->target error (G08: the robot rushes far targets reactively, with
    no deceleration profile, and arrives too hot to strike).

    PACE's remedy is a velocity command proportional to the remaining position error, so the DESIRED
    speed goes to ~0 exactly at arrival. Deploy-parity constraint: the 175-D actor obs contract is
    FROZEN, so this is a REWARD-side term only — nothing new is observed; the kernel reuses the task's
    own error measure (the planar racket->target distance, frame-invariant, no world base position):

        v_des = clamp(v_gain * ||(racket_target_xy - racket_xy)||, 0, v_max)
        reward = exp(-(||v_base_xy|| - v_des)^2 / std^2)

    Far target -> v_des saturates at v_max and the term pays for MOVING (it cooperates with
    racket_progress instead of taxing the approach); as the strike stance is reached v_des -> 0 and the
    term pays for a CALM base — a smooth taper instead of the bang-bang rush-then-slam. Gated to
    ``pre_strike`` ONLY: the strike swing and the post-strike recovery are untouched (post-strike the
    distance to the OLD swung-through target would otherwise command a bogus speed-up). Base velocity
    is the WORLD planar root velocity (same source as hold_ready); v_gain [1/s] is the P-gain of the
    pseudo velocity command, v_max [m/s] its cap, std [m/s] the kernel width. RewTerm weight is
    POSITIVE; default weight 0.0 = OFF (flag-gated via task.rewards.base_decel_weight)."""
    cmd = _cmd(env, command_name)
    planar_err = torch.norm(cmd.racket_target_pos_w[:, :2] - cmd.racket_pos_w[:, :2], dim=-1)
    v_des = (v_gain * planar_err).clamp(0.0, v_max)
    v_base = torch.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    raw = torch.exp(-torch.square(v_base - v_des) / std**2)
    return raw * cmd.pre_strike.float()


def base_station_settle(
    env: ManagerBasedRLEnv, command_name: str, v_gain: float = 2.0, v_max: float = 1.2, std: float = 0.4
) -> torch.Tensor:
    """PACE-style base deceleration keyed to the base→STATION error (2026-07-09): teaches the robot to
    move to the y-station EARLY and SETTLE there BEFORE it strikes — the pre-strike gap the near-strike
    ``lower_body_plant_imitation`` term does NOT cover (that one only shapes the leg pose at contact).

    The HITTER ``base_position`` reward pins the base to the station, but at std 0.20 its kernel is
    gradient-DEAD at a fresh ±0.40 m station (``exp(-(0.40/0.20)²)≈0.018``), so a far y-station exerts
    almost no early pull — the policy learns a last-moment RUSH and arrives hot, then lunges. This term
    fixes BOTH failure modes with ONE velocity-command kernel (same PACE idea as ``base_decel_tracking``
    but keyed to the base→station error, NOT racket→target — the racket-keyed version commands the base
    to keep moving until the ARM reaches, ``v_des≈v_gain·0.51`` even AT the correct x-locked station, so
    it fights settling).

    ⚠ DIRECTIONAL (2026-07-09 fix): the desired velocity is a VECTOR pointing at the station, and the
    kernel matches the base velocity VECTOR to it — NOT just the speed magnitude. An earlier scalar
    version (``exp(-(‖v_base‖ − v_des)²/std²)``) rewarded merely MOVING at the right speed, so orbiting
    the station or moving AWAY at the right |v| scored full marks. Here::

        d      = station_xy − base_xy                       # vector TO the station
        v_des  = (d/‖d‖) · clamp(v_gain·‖d‖, 0, v_max)      # desired velocity: toward station, →0 at it
        reward = exp(−‖v_base_xy − v_des‖² / std²)          # match direction AND magnitude

    Far away this pays only for moving TOWARD the station at the decel speed (a live early-move gradient
    linear in ‖d‖, no dead far edge); moving sideways or away scores low; as the base arrives ``v_des→0``
    so it pays for a CALM, settled base — "move early, TOWARD the station, then settle, THEN strike". It
    never rewards freezing FAR away (‖d‖ large ⇒ v_des large ⇒ stillness scores low), so the y-footwork
    is directed, not suppressed. At the station the ill-defined unit direction is harmless (v_des→0 kills
    it). Gated to ``pre_strike & ~strike_window`` (2026-07-09 audit fix): pre_strike (tts>0) alone
    OVERLAPS the strike window for tts in (0, strike_window_s], and paying for a near-still base in the
    ~6 pre-contact steps is the post_strike_brake GAE channel pointed at the swing's weight transfer —
    excluding the window keeps the approach+hold shaping intact while the strike stays reward-untouched
    (calm-at-contact is enforced by the base_speed_at_strike GATE, not by a gradient on the swing). The
    post-strike follow-through is also excluded (the distance to the OLD station would command a bogus
    speed-up). ``v_base`` is the WORLD
    planar root velocity; ``v_gain`` [1/s] the P-gain, ``v_max`` [m/s] the cap, ``std`` [m/s] the kernel
    width. RewTerm weight POSITIVE; default 0.0 = OFF (flag-gated ``task.rewards.base_station_settle_weight``)."""
    cmd = _cmd(env, command_name)
    to_station = cmd.base_target_pos_w - cmd.base_pos_w[:, :2]          # vector base→station (world xy)
    dist = torch.norm(to_station, dim=-1)
    v_des_mag = (v_gain * dist).clamp(0.0, v_max)                        # desired SPEED (→0 at station)
    # desired velocity VECTOR = unit(to_station) · v_des_mag. clamp_min guards the div; at the station
    # v_des_mag→0 zeroes the (then ill-defined) direction, so there is no eps artifact.
    v_des_vec = (to_station / dist.clamp_min(1e-6).unsqueeze(-1)) * v_des_mag.unsqueeze(-1)
    v_base = cmd.robot.data.root_lin_vel_w[:, :2]
    err = torch.sum(torch.square(v_base - v_des_vec), dim=-1)            # VECTOR velocity error (dir+mag)
    raw = torch.exp(-err / std**2)
    return raw * (cmd.pre_strike & ~cmd.strike_window).float()


# ============================================================================================== #
# POST-SWING RECOVERY (2026-07-09) — kill the AGI-plant forward x-DRIFT at its SOURCE (the swing
# follow-through), so the base enters the between-swing hold already settled on the x-locked station.
# Diagnosis (memory hope-legplant-regresses-agi): the strike itself is clean, but the follow-through
# carries the base forward in world x; over consecutive serves it accumulates until a swing starts
# off-station and topples (model_21500 recovers to 0.11 m in AGI G3; the reverted demo leg-POSE plant
# made it DIVERGE — 1.4-2.0 m). These three terms are the POST-strike complement to the pre-strike
# base_station_settle: x-ONLY station pull + world-vx quieting + leg-VELOCITY quieting (NOT demo pose —
# pose re-imports the demo forward step-in). All gate to the FOLLOW-THROUGH window via
# _post_strike_window, so they never touch the strike's through-motion (strike_window-protected) or the
# pre-strike y-footwork approach (pre_strike-gated out); x/leg-only ⇒ lateral y footwork stays FREE.
#
# ⚠ GAE PRECISION RISK (the reason post_strike_brake stays 0): POSITIVE income in the follow-through is
# credited by GAE back onto the SWING action, so a policy can farm it by swinging SOFTER (less momentum
# to arrest / less drift to correct) — that channel collapsed single-swing composite 0.994→0.866 on
# model_18000/rally2 when the base was drift-free. The x-LOCK weakens it a lot (base barely drifts in
# Isaac now, so the follow-through gradient w.r.t. swing-softness is small), and the window excludes the
# strike, but it is NOT zero. GATE PROTOCOL: run G1 (single-swing det composite) FIRST every ~200 iters;
# if it drops below ~0.95, LOWER these weights (vx_quiet first — the speed term is the most brake-like)
# or push the window later. And note the drift is an AGI-plant phenomenon Isaac UNDER-shows, so in Isaac
# these can saturate (base already ~on-station ⇒ reward ~1, dead gradient) — the load-bearing test is
# AGI G3 (pp_gate3_rally.sh), never Isaac G1 alone.
def _post_strike_window(
    cmd: RacketTargetCommand, t_hi: float, t_lo: float = 0.0
) -> torch.Tensor:
    """Boolean mask for ``t_lo <= time-after-strike < t_hi`` in the follow-through.

    ``(~pre_strike) & (~strike_window)`` is the tts<0 follow-through (the same gate post_strike_brake
    uses — it can never touch the strike, and it closes at the clip wrap where tts snaps positive);
    ``time_to_strike > -t_hi`` caps it at ``t_hi`` s post-strike (the default 1.0-1.2 s sits inside the
    ~1.5-1.8 s fh/bh follow-through, before the wrap). All-False on tasks whose tts clock never goes
    negative (no strike machinery), so every caller is a safe no-op there."""
    return (
        (~cmd.pre_strike)
        & (~cmd.strike_window)
        & (cmd.time_to_strike > -t_hi)
        & (cmd.time_to_strike <= -t_lo)
    )


def post_strike_x_settle(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float = 0.08,
    t_lo: float = 0.0,
    t_hi: float = 1.2,
) -> torch.Tensor:
    """POSITIVE post-strike x-station settle: pull the base back onto the x-locked station AFTER the swing.

    x-ONLY (anisotropic) by design — ``exp(-(base_x − station_x)² / std²)`` in WORLD x, the locked axis
    (base_target_x_range [0,0] ⇒ station_x is the fixed spawn plane). It says NOTHING about y, so the
    lateral footwork toward the next ball is untaxed (the user's explicit constraint: never a tight xy
    norm). Active ONLY in the follow-through window (``_post_strike_window``). std 0.08 m is a TIGHT
    kernel (the x-lock target is ~1 cm) so it pays hard for returning to the plane and keeps a live
    gradient against a ~0.2 m follow-through excursion. Weight POSITIVE; default 0.0 = OFF. See the
    section header for the GAE precision caveat + gate protocol."""
    cmd = _cmd(env, command_name)
    x_err = cmd.base_pos_w[:, 0] - cmd.base_target_pos_w[:, 0]           # world base_x − station_x
    raw = torch.exp(-torch.square(x_err) / std**2)
    gate = _post_strike_window(cmd, t_hi, t_lo)
    _dbg_log(cmd, "post_strike_x_settle", raw, gate)
    return raw * gate.float()


def post_strike_vx_quiet(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float = 0.15,
    t_lo: float = 0.0,
    t_hi: float = 1.0,
) -> torch.Tensor:
    """POSITIVE post-strike forward-velocity quieting: reward a near-zero WORLD-x base velocity after the
    swing — arrests the follow-through momentum that carries the base off the x-station.

    ``exp(-(base_vx)² / std²)`` on the world-x root velocity (the locked / strike-heading axis). Only vx
    is quieted — vy is FREE, so the robot may still step laterally toward the next ball (unlike the
    full-|v_xy| ``post_strike_brake``, which taxes y motion too and is the reason brake was replaced).
    Velocity-space partner of ``post_strike_x_settle`` (position): together they decelerate-and-home the
    base in x. Active ONLY in the follow-through window. std 0.15 m/s; weight POSITIVE; default 0.0 = OFF.
    ⚠ This is the most brake-like of the three (rewards low speed) — the first to lower if G1 drops."""
    cmd = _cmd(env, command_name)
    vx = cmd.robot.data.root_lin_vel_w[:, 0]                            # world-x base velocity
    raw = torch.exp(-torch.square(vx) / std**2)
    gate = _post_strike_window(cmd, t_hi, t_lo)
    _dbg_log(cmd, "post_strike_vx_quiet", raw, gate)
    return raw * gate.float()


def post_strike_leg_quiet(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    t_lo: float = 0.0,
    t_hi: float = 1.2,
) -> torch.Tensor:
    """PENALTY on leg-joint SPEED in the follow-through — quiet the legs after the swing WITHOUT any demo
    pose (pose imitation re-imports the demo forward step-in; memory hope-legplant-regresses-agi).

    Returns ``mean(qd_leg²)`` over the leg joints (hip/knee/ankle), MEAN not sum (joint-count-independent
    — the lower_body_plant lesson: a 12-joint SUM silently scales the effective weight and saturated the
    kernel to a dead 0). RewTerm weight is NEGATIVE and SMALL (start −0.02): it biases the legs to stop
    thrashing during recovery but must not out-vote the swing or suppress the pre-strike y-steps (the
    follow-through gate already keeps it off the approach — it uses leg VELOCITY, not pose, so it has no
    forward-step-in bias). Active ONLY in the follow-through window; safe no-op off-window."""
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    qd = asset.data.joint_vel[:, asset_cfg.joint_ids]
    raw = torch.mean(torch.square(qd), dim=1)                          # mean per-joint leg speed²
    gate = _post_strike_window(cmd, t_hi, t_lo)
    return raw * gate.float()


def backhand_left_hand_clearance(
    env: ManagerBasedRLEnv, command_name: str, margin: float = 0.15, left_body_name: str = "left_wrist_yaw_Link"
) -> torch.Tensor:
    """PENALTY: keep the racket clear of the LEFT hand during the BACKHAND (2026-07-09).

    Isaac trains with ``enabled_self_collisions=False`` (agibot_a3.py) so the robot's own bodies pass
    through each other for FREE — nothing discourages the backhand windup from sweeping the paddle across
    to the left where the left hand sits. AGI MuJoCo HAS self-collision, so the racket visibly HITS the
    left hand there (the classic Isaac-hides / AGI-reveals gap, cf. [[hope-plant-alignment]]). This is a
    REWARD-side barrier that teaches the clearance without turning on global self-collision physics
    (which would fire on every limb pair and can destabilize the un-validated retargeted clips).

    Hinge barrier on the racket-center ↔ left-wrist distance:  ``(clamp(margin − dist, 0)/margin)²`` —
    exactly 0 when clear (dist ≥ margin, so it never perturbs a clean swing), rising to 1 at contact.
    ``racket_pos_w`` is the paddle center (FK); the left HAND link is fixed-merged in Isaac so the last
    actuated left-arm body ``left_wrist_yaw_Link`` is the resolvable proxy — ``margin`` (default 0.15 m)
    absorbs the blade radius (~0.07) + hand-past-wrist (~0.07). Gated to the BACKHAND clip
    (``clip_id == 1``) only — the forehand keeps the paddle on the right, away from the left hand, and
    should not be taxed. RewTerm weight NEGATIVE (start ~−2.0). Safe no-op if the left body cannot be
    resolved or the motion command has no clip_id."""
    cmd = _cmd(env, command_name)
    idx = getattr(cmd, "_left_hand_body_index", None)
    if idx is None:
        found = cmd.robot.find_bodies(left_body_name, preserve_order=True)[0]
        idx = int(found[0]) if len(found) else -1
        cmd._left_hand_body_index = idx
    clip_id = getattr(cmd._motion(), "clip_id", None)
    if idx < 0 or clip_id is None:
        return torch.zeros(cmd.num_envs, device=cmd.device)
    left_pos = cmd.robot.data.body_pos_w[:, idx]                       # (N,3) world left-wrist
    dist = torch.norm(cmd.racket_pos_w - left_pos, dim=-1)            # racket-center ↔ left-wrist
    barrier = (torch.clamp(margin - dist, min=0.0) / margin) ** 2      # 0 clear .. 1 at contact
    bh = (clip_id == 1).float()                                        # backhand clip only
    _dbg_log(cmd, "bh_left_hand_clearance", barrier, bh > 0.5)
    return barrier * bh


# ============================================================================================== #
# HITTER-PURE RALLY FINAL (2026-07-10): minimal, phase-gated corrections on the CLEAN HitterPure
# task.  These terms deliberately do not use a hold state, ball state, heading curriculum, or demo
# lower-body pose.  Every penalty is bounded and leaves early lateral approach steps unpenalized.
# ============================================================================================== #
def _deadband_huber(value: torch.Tensor, margin: float, std: float) -> torch.Tensor:
    """Smooth-L1 debt outside a physical deadband, with a non-saturating tail."""
    scaled = torch.clamp(value - float(margin), min=0.0) / max(float(std), 1e-6)
    return torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)


def windup_x_recovery_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    x_margin: float = 0.025,
    x_std: float = 0.05,
    vx_margin: float = 0.05,
    vx_std: float = 0.20,
    position_blend: float = 0.60,
    t_hi: float = 0.96,
) -> torch.Tensor:
    """Recover the commanded x station during ready/windup without taxing correction.

    The position channel is zero inside ``x_margin``. Outside that band the velocity channel
    charges only motion that increases ``|base_x-station_x|``; walking back toward the station is
    free. Inside the band it charges excessive ``|vx|`` so the base does not coast through the
    target. Both channels use smooth-L1 tails and the term ends before the strike window, avoiding
    the failure mode of an unconditional windup ``|vx|`` penalty that can freeze required recovery.
    The RewTerm weight must be negative.
    """
    if not 0.0 <= position_blend <= 1.0:
        raise ValueError(
            f"windup x recovery position_blend must be in [0,1], got {position_blend}"
        )
    cmd = _cmd(env, command_name)
    signed_x_error = cmd.base_pos_w[:, 0] - cmd.base_target_pos_w[:, 0]
    x_error = torch.abs(signed_x_error)
    position_debt = _deadband_huber(x_error, x_margin, x_std)

    vx = cmd.robot.data.root_lin_vel_w[:, 0]
    outward_vx = torch.sign(signed_x_error) * vx
    charged_vx = torch.where(x_error > x_margin, outward_vx, torch.abs(vx))
    velocity_debt = _deadband_huber(charged_vx, vx_margin, vx_std)

    raw = position_blend * position_debt + (1.0 - position_blend) * velocity_debt
    gate = (
        (cmd.time_to_strike > float(cmd.cfg.strike_window_s))
        & (cmd.time_to_strike < float(t_hi))
    )
    return raw * gate.float()


def strike_x_drift_penalty(
    env: ManagerBasedRLEnv, command_name: str, margin: float = 0.04, std: float = 0.08,
    t_pre: float = 0.45, t_post: float = 1.00, huber_tail: bool = False,
) -> torch.Tensor:
    """Bounded x-station drift penalty from final approach through early recovery.

    ``1-exp(-(relu(|base_x-station_x|-margin)/std)^2)`` is exactly zero inside the 4 cm deadband,
    saturates at one, and says nothing about y.  Thus it prevents the forward lunge without taxing
    the commanded lateral step.  RewTerm weight is negative.
    """
    cmd = _cmd(env, command_name)
    x_err = torch.abs(cmd.base_pos_w[:, 0] - cmd.base_target_pos_w[:, 0])
    if huber_tail:
        raw = _deadband_huber(x_err, margin, std)
    else:
        excess = torch.clamp(x_err - margin, min=0.0)
        raw = 1.0 - torch.exp(-torch.square(excess / std))
    gate = (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)
    return raw * gate.float()


def rally_strike_x_margin_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.015,
    std: float = 0.020,
    half_window_s: float = 0.040,
    forehand_scale: float = 1.0,
    backhand_scale: float = 1.0,
) -> torch.Tensor:
    """Contact-local x-lock debt with explicit margin below the Gate3 limit.

    The broad ``strike_x_drift_penalty`` remains responsible for approach and recovery.  This
    term only sharpens the final few 50-Hz samples around contact, where RallyV11 averaged about
    the same 3 cm error that Gate3 treats as a hard limit.  A 1.5 cm zero-debt band leaves useful
    balance motion while training a measurable margin to the 3 cm report threshold.
    """
    if (
        margin < 0.0 or std <= 0.0 or half_window_s <= 0.0
        or forehand_scale < 0.0 or backhand_scale < 0.0
    ):
        raise ValueError(
            "strike-x margin/std/half-window must be non-negative/positive/positive, "
            f"got {margin}/{std}/{half_window_s}"
        )
    cmd = _cmd(env, command_name)
    x_error = torch.abs(cmd.base_pos_w[:, 0] - cmd.base_target_pos_w[:, 0])
    debt = _deadband_huber(x_error, float(margin), float(std))
    gate = torch.abs(cmd.time_to_strike) <= float(half_window_s)
    side_scale = torch.where(
        cmd.swing_sign > 0.0,
        torch.full_like(debt, float(forehand_scale)),
        torch.full_like(debt, float(backhand_scale)),
    )
    cmd.metrics["strike_x_margin_error"] = x_error * gate.float()
    cmd.metrics["strike_x_margin_excess"] = (
        torch.clamp(x_error - float(margin), min=0.0) * gate.float()
    )
    return debt * side_scale * gate.float()


def rally_post_swing_xlock_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.030,
    std: float = 0.040,
    t_lo: float = 0.10,
    t_hi: float = 1.55,
    forehand_scale: float = 1.0,
    backhand_scale: float = 1.0,
) -> torch.Tensor:
    """Negative recovery-window x-lock debt for the conductor's whole-interval gate.

    The inherited positive exponential settle reward becomes nearly flat once recovery is merely
    reasonable. This deadband-Huber term keeps a non-saturating gradient beyond 3 cm, leaving a
    measurable 2 cm margin to Gate3's 5 cm whole-interval limit. Side scales let V13 charge the
    observed backhand tail without increasing forehand contact-x pressure.
    """
    if (
        margin < 0.0 or std <= 0.0 or t_lo < 0.0 or t_hi <= t_lo
        or forehand_scale < 0.0 or backhand_scale < 0.0
    ):
        raise ValueError(
            "invalid post-swing x-lock parameters: "
            f"margin/std={margin}/{std}, window={t_lo}/{t_hi}, "
            f"side={forehand_scale}/{backhand_scale}"
        )
    cmd = _cmd(env, command_name)
    x_error = torch.abs(cmd.base_pos_w[:, 0] - cmd.base_target_pos_w[:, 0])
    debt = _deadband_huber(x_error, float(margin), float(std))
    side_scale = torch.where(
        cmd.swing_sign > 0.0,
        torch.full_like(debt, float(forehand_scale)),
        torch.full_like(debt, float(backhand_scale)),
    )
    gate = (cmd.time_to_strike < -float(t_lo)) & (cmd.time_to_strike > -float(t_hi))
    cmd.metrics["post_swing_xlock_error"] = x_error * gate.float()
    cmd.metrics["post_swing_xlock_excess"] = (
        torch.clamp(x_error - float(margin), min=0.0) * gate.float()
    )
    return debt * side_scale * gate.float()


def strike_x_velocity_penalty(
    env: ManagerBasedRLEnv, command_name: str, margin: float = 0.05, std: float = 0.20,
    t_pre: float = 0.45, t_post: float = 1.00, huber_tail: bool = False,
) -> torch.Tensor:
    """Bounded world-|vx| penalty over the same x-lock window; lateral ``vy`` remains free."""
    cmd = _cmd(env, command_name)
    vx = torch.abs(cmd.robot.data.root_lin_vel_w[:, 0])
    if huber_tail:
        raw = _deadband_huber(vx, margin, std)
    else:
        excess = torch.clamp(vx - margin, min=0.0)
        raw = 1.0 - torch.exp(-torch.square(excess / std))
    gate = (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)
    return raw * gate.float()


def pre_strike_station_settle(
    env: ManagerBasedRLEnv, command_name: str, v_gain: float = 2.0, v_max: float = 0.8,
    std: float = 0.35, t_max: float = 1.0, t_min: float | None = None,
    velocity_margin: float = 0.0, debt_huber: bool = False,
) -> torch.Tensor:
    """Move toward the station early, then become still before the strike.

    Desired planar velocity is ``unit(station-base)*clamp(v_gain*distance, 0, v_max)``.  It is large
    and directional while far away and converges continuously to zero at arrival, so the same term
    expresses ``move -> settle`` without a policy-stretchable hold.  It is active only for
    ``strike_window_s < tts < t_max`` and therefore never damps the actual strike window.
    """
    cmd = _cmd(env, command_name)
    delta = cmd.base_target_pos_w - cmd.base_pos_w[:, :2]
    dist = torch.linalg.norm(delta, dim=-1)
    speed = (v_gain * dist).clamp(0.0, v_max)
    desired = delta / dist.clamp_min(1e-6).unsqueeze(-1) * speed.unsqueeze(-1)
    err = torch.linalg.norm(cmd.robot.data.root_lin_vel_w[:, :2] - desired, dim=-1)
    if debt_huber:
        raw = _deadband_huber(err, velocity_margin, std)
    else:
        raw = torch.exp(-torch.square(err) / std**2)
    gate = (cmd.time_to_strike > float(cmd.cfg.strike_window_s)) & (cmd.time_to_strike < t_max)
    if t_min is not None:
        gate &= cmd.time_to_strike > float(t_min)
    return raw * gate.float()


def post_swing_base_quiet(
    env: ManagerBasedRLEnv, command_name: str, std: float = 0.25,
    t_lo: float = 0.20, t_hi: float = 1.00, margin: float = 0.0,
    huber_tail: bool = False,
) -> torch.Tensor:
    """Bounded penalty on full planar base speed after a 0.20 s follow-through grace period."""
    cmd = _cmd(env, command_name)
    speed = torch.linalg.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    raw = (_deadband_huber(speed, margin, std) if huber_tail else
           1.0 - torch.exp(-torch.square(speed) / std**2))
    gate = (cmd.time_to_strike < -t_lo) & (cmd.time_to_strike > -t_hi)
    return raw * gate.float()


def post_swing_leg_quiet(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg, std: float = 0.8,
    t_lo: float = 0.20, t_hi: float = 1.00, margin: float = 0.0,
    huber_tail: bool = False,
) -> torch.Tensor:
    """Bounded leg-joint velocity penalty in recovery; no demo pose and no approach-step tax."""
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    qd_rms = torch.sqrt(
        torch.mean(torch.square(asset.data.joint_vel[:, asset_cfg.joint_ids]), dim=-1)
    )
    raw = (_deadband_huber(qd_rms, margin, std) if huber_tail else
           1.0 - torch.exp(-torch.square(qd_rms) / std**2))
    gate = (cmd.time_to_strike < -t_lo) & (cmd.time_to_strike > -t_hi)
    return raw * gate.float()


def settle_foot_slip_penalty(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg, sensor_cfg,
    std: float = 0.12, station_reach: float = 0.15, force_threshold: float = 10.0,
    pre_t_max: float = 0.45, strike_t_post: float = 0.10,
    post_t_lo: float = 0.20, post_t_hi: float = 1.00,
    margin: float = 0.0, huber_tail: bool = False,
) -> torch.Tensor:
    """Moderate contact-slip penalty only after arrival or during post-swing recovery.

    Early lateral footwork is free: the arrival branch opens only when the base is within
    ``station_reach``. It intentionally covers contact and the first ``strike_t_post`` seconds after
    contact so the task cannot solve the hit by sliding a loaded foot; the small bounded weight still
    permits legitimate weight transfer. The later post branch reopens after the follow-through grace
    period. Airborne swing-foot velocity is never penalized because the source metric contains only
    horizontal velocity of contacting feet.
    """
    cmd = _cmd(env, command_name)
    # Do not consume RacketTargetCommand's lazy stability cache here.  On the first env.step Isaac Lab
    # computes rewards before CommandManager.compute(), so that cache has not been populated yet.  The
    # RewardManager resolves these SceneEntityCfg objects during setup instead, making the two-foot
    # contract available on the very first reward step and keeping resolution failures fail-closed.
    foot_ids_robot = asset_cfg.body_ids
    foot_ids_contact = sensor_cfg.body_ids
    if len(foot_ids_robot) != 2 or len(foot_ids_contact) != 2:
        raise RuntimeError(
            "RallyFinal settle_foot_slip requires exactly two manager-resolved ankle-roll bodies "
            f"for robot/contact_forces, got {len(foot_ids_robot)}/{len(foot_ids_contact)}; "
            "refusing a fail-open safety reward"
        )
    asset = env.scene[asset_cfg.name]
    sensor = env.scene.sensors[sensor_cfg.name]
    foot_force = torch.linalg.norm(sensor.data.net_forces_w[:, foot_ids_contact, :], dim=-1)
    in_contact = (foot_force > force_threshold).float()
    foot_speed_xy = torch.linalg.norm(asset.data.body_lin_vel_w[:, foot_ids_robot, :2], dim=-1)
    # Match the command metric semantics: mean horizontal speed over contacting feet, zero airborne.
    slip = (foot_speed_xy * in_contact).sum(dim=-1) / in_contact.sum(dim=-1).clamp(min=1.0)
    raw = (_deadband_huber(slip, margin, std) if huber_tail else
           1.0 - torch.exp(-torch.square(slip / std)))
    station_err = torch.linalg.norm(cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1)
    arrived_strike = (
        (station_err < station_reach)
        & (cmd.time_to_strike > -strike_t_post)
        & (cmd.time_to_strike < pre_t_max)
    )
    post = (cmd.time_to_strike < -post_t_lo) & (cmd.time_to_strike > -post_t_hi)
    return raw * (arrived_strike | post).float()


def backhand_left_arm_clearance(
    env: ManagerBasedRLEnv, command_name: str, hand_margin: float = 0.15,
    forearm_margin: float = 0.12, t_pre: float = 0.70, t_post: float = 0.20,
    hand_body_name: str = "left_wrist_yaw_Link", elbow_body_name: str = "left_elbow_Link",
    forearm_end_body_name: str = "left_wrist_roll_Link",
) -> torch.Tensor:
    """Backhand danger-window barrier for both the left hand and the left forearm segment."""
    cmd = _cmd(env, command_name)
    missing = [
        name for name in (hand_body_name, elbow_body_name, forearm_end_body_name)
        if cmd._resolve_body_index(name) < 0
    ]
    if missing:
        raise RuntimeError(
            f"RallyFinal left-arm clearance bodies missing from articulation: {missing}"
        )
    hand_dist, forearm_dist = cmd.left_arm_clearance(
        hand_body_name=hand_body_name,
        elbow_body_name=elbow_body_name,
        forearm_end_body_name=forearm_end_body_name,
    )
    hand = torch.square(torch.clamp(hand_margin - hand_dist, min=0.0) / hand_margin)
    forearm = torch.square(torch.clamp(forearm_margin - forearm_dist, min=0.0) / forearm_margin)
    barrier = torch.maximum(hand, forearm)
    gate = (cmd.swing_sign < 0.0) & (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)
    return barrier * gate.float()


def strike_front_facing(
    env: ManagerBasedRLEnv, command_name: str, base_std: float = 0.35,
    torso_std: float = 0.30, torso_rate_std: float = 1.00,
    t_pre: float = 0.25, t_post: float = 0.20, torso_body_name: str = "torso_Link",
) -> torch.Tensor:
    """Face world +x with both pelvis and torso, and avoid torso yaw spin around contact.

    This is a short strike-window reward, not a heading curriculum.  It leaves the earlier lateral
    approach and natural arm motion free while rejecting large whole-body twist/turn solutions.  The
    three Gaussian scores are averaged, not multiplied: a large torso yaw rate must not zero the pelvis
    and torso heading gradients.  The arithmetic mean keeps the same [0, 1] range and maximum income.
    """
    cmd = _cmd(env, command_name)

    def _yaw(q):
        fx = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
        fy = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
        return torch.atan2(fy, fx)

    torso_idx = cmd._resolve_body_index(torso_body_name)
    if torso_idx < 0:
        raise RuntimeError(
            f"RallyFinal front-facing reward cannot resolve torso body {torso_body_name!r}"
        )
    base_yaw = _yaw(cmd.base_quat_w)
    torso_yaw = _yaw(cmd.robot.data.body_quat_w[:, torso_idx])
    torso_wz = cmd.robot.data.body_ang_vel_w[:, torso_idx, 2]
    base_score = torch.exp(-torch.square(base_yaw / base_std))
    torso_score = torch.exp(-torch.square(torso_yaw / torso_std))
    torso_rate_score = torch.exp(-torch.square(torso_wz / torso_rate_std))
    raw = (base_score + torso_score + torso_rate_score) / 3.0
    gate = (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)
    return raw * gate.float()


def rally_heading_debt(
    env: ManagerBasedRLEnv, command_name: str,
    yaw_margin: float = 0.12, yaw_std: float = 0.25,
    rate_margin: float = 0.15, rate_std: float = 0.50,
    heading_blend: float = 0.70,
    ready_t_lo: float = 0.45, ready_t_hi: float = 1.40,
    post_t_lo: float = 0.35, post_t_hi: float = 1.80,
    huber_tail: bool = False,
) -> torch.Tensor:
    """Bounded pelvis-heading debt in runner-ready and full post-swing recovery phases.

    ``strike_front_facing`` intentionally owns only the contact window.  The deploy runner, however,
    can clamp the policy at ``tts=1.30`` while waiting for the next ball, and the longest backhand clip
    continues for about 1.74 s after contact.  Without this term a policy can hit square, finish at a
    yawed rest pose, and be rejected forever by the runner's 20 degree engage safety gate.

    The yaw component has a deploy-safe deadband instead of paying income continuously at zero.  The
    rate component is directional outside that deadband: it penalizes angular velocity that increases
    ``|yaw|`` but leaves a corrective turn toward world +x free.  Once inside the deadband, any excessive
    yaw rate is charged so the base does not coast straight through the square pose.  This avoids the
    bad fixed point of a standalone ``|wz| -> 0`` penalty, which would happily freeze the robot at 23
    degrees.  Both channels are bounded in [0, 1]; the RewTerm weight must be negative.

    The ready gate covers the runner's frozen clock while leaving the final 0.45 s wind-up to the
    existing strike term. RallyFinalV2 configures the post gate after 0.20 s of free follow-through
    and through both clip tails. No hold state, ball state, torso pose, or lower-body demonstration is
    used.
    """
    cmd = _cmd(env, command_name)
    q = cmd.base_quat_w
    forward_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    forward_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw = torch.atan2(forward_y, forward_x)
    yaw_abs = torch.abs(yaw)

    if huber_tail:
        yaw_debt = _deadband_huber(yaw_abs, yaw_margin, yaw_std)
    else:
        yaw_excess = torch.clamp(yaw_abs - yaw_margin, min=0.0)
        yaw_debt = 1.0 - torch.exp(-torch.square(yaw_excess / yaw_std))

    root_wz = cmd.robot.data.root_ang_vel_w[:, 2]
    # Outside the safe heading band, only rotation AWAY from square is debt; corrective rotation is
    # free.  Inside it, quiet both directions to prevent momentum from carrying the pelvis back out.
    outward_rate = torch.sign(yaw) * root_wz
    charged_rate = torch.where(yaw_abs > yaw_margin, outward_rate, torch.abs(root_wz))
    if huber_tail:
        rate_debt = _deadband_huber(charged_rate, rate_margin, rate_std)
    else:
        rate_excess = torch.clamp(charged_rate - rate_margin, min=0.0)
        rate_debt = 1.0 - torch.exp(-torch.square(rate_excess / rate_std))

    raw = heading_blend * yaw_debt + (1.0 - heading_blend) * rate_debt
    ready = (cmd.time_to_strike > ready_t_lo) & (cmd.time_to_strike < ready_t_hi)
    post = (cmd.time_to_strike < -post_t_lo) & (cmd.time_to_strike > -post_t_hi)
    return raw * (ready | post).float()


def rally_heading_settle_debt(
    env: ManagerBasedRLEnv, command_name: str,
    yaw_margin: float = 0.12, yaw_std: float = 0.25,
    rate_margin: float = 0.05, rate_std: float = 0.25,
    heading_blend: float = 0.35,
    yaw_rate_gain: float = 2.0, yaw_rate_max: float = 0.15,
    ready_t_lo: float = 0.45, ready_t_hi: float = 1.00,
    post_t_lo: float = 0.20, post_t_hi: float = 1.20,
    huber_tail: bool = True,
    forehand_scale: float = 1.0,
    backhand_scale: float = 1.0,
) -> torch.Tensor:
    """First-order pelvis heading recovery that settles instead of snapping through square.

    RallyV9's directional rate channel deliberately made corrective rotation free while the
    pelvis was outside the heading deadband.  Gate3 showed the resulting loophole: the policy
    learned a late, fast correction that reduced heading error but still exceeded the absolute
    yaw-rate gate.  V10 defines a bounded first-order target instead::

        desired_wz = clamp(-yaw_rate_gain * signed_yaw_excess, +/-yaw_rate_max)

    where ``signed_yaw_excess`` is zero inside ``yaw_margin``.  The rate debt measures deviation
    from that target, so recovery remains rewarded while excessive corrective speed, outward
    rotation, and coasting through square are all charged.  ``yaw_rate_max + rate_margin`` is
    intentionally the deploy Gate3 limit (0.20 rad/s), so the
    zero-debt rate envelope cannot extend past the gate. The phase windows remain identical to V9.
    """
    if not 0.0 <= heading_blend <= 1.0:
        raise ValueError(f"heading_blend must be in [0,1], got {heading_blend}")
    if yaw_rate_gain < 0.0 or yaw_rate_max <= 0.0:
        raise ValueError(
            "yaw_rate_gain must be non-negative and yaw_rate_max positive, got "
            f"{yaw_rate_gain}/{yaw_rate_max}"
        )
    if forehand_scale < 0.0 or backhand_scale < 0.0:
        raise ValueError(
            f"heading side scales must be non-negative, got {forehand_scale}/{backhand_scale}"
        )

    cmd = _cmd(env, command_name)
    q = cmd.base_quat_w
    forward_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    forward_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw = torch.atan2(forward_y, forward_x)
    yaw_abs = torch.abs(yaw)

    if huber_tail:
        yaw_debt = _deadband_huber(yaw_abs, yaw_margin, yaw_std)
    else:
        yaw_excess = torch.clamp(yaw_abs - yaw_margin, min=0.0)
        yaw_debt = 1.0 - torch.exp(-torch.square(yaw_excess / yaw_std))

    signed_excess = torch.sign(yaw) * torch.clamp(yaw_abs - yaw_margin, min=0.0)
    desired_wz = torch.clamp(
        -float(yaw_rate_gain) * signed_excess,
        min=-float(yaw_rate_max),
        max=float(yaw_rate_max),
    )
    root_wz = cmd.robot.data.root_ang_vel_w[:, 2]
    rate_error = torch.abs(root_wz - desired_wz)
    if huber_tail:
        rate_debt = _deadband_huber(rate_error, rate_margin, rate_std)
    else:
        rate_excess = torch.clamp(rate_error - rate_margin, min=0.0)
        rate_debt = 1.0 - torch.exp(-torch.square(rate_excess / rate_std))

    raw = heading_blend * yaw_debt + (1.0 - heading_blend) * rate_debt
    ready = (cmd.time_to_strike > ready_t_lo) & (cmd.time_to_strike < ready_t_hi)
    post = (cmd.time_to_strike < -post_t_lo) & (cmd.time_to_strike > -post_t_hi)
    gate = ready | post
    side_scale = torch.where(
        cmd.swing_sign > 0.0,
        torch.full_like(raw, float(forehand_scale)),
        torch.full_like(raw, float(backhand_scale)),
    )
    cmd.metrics["yaw_settle_desired_rate_abs"] = torch.abs(desired_wz) * gate.float()
    cmd.metrics["yaw_settle_rate_error"] = rate_error * gate.float()
    return raw * side_scale * gate.float()


def rally_ready_root_height_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    motion_command_name: str,
    min_height: float = 1.02,
    std: float = 0.05,
    ready_t_lo: float = -0.10,
    ready_t_hi: float = 1.10,
    post_t_lo: float = 0.20,
    post_t_hi: float = 1.55,
) -> torch.Tensor:
    """Zero-income pelvis-height debt for the V7 ready/recovery mismatch.

    The approved V7 clips begin near the deploy stand height but drop the pelvis by roughly
    10--12 cm and finish in that crouch.  The lower body remains intentionally free: this term
    observes only root ``z`` and is exactly zero above ``min_height``.  A smooth-L1 tail avoids
    the flat gradient of a bounded exponential without ever paying income for standing tall.

    The gate covers the complete exogenous hold, the pre-strike ready/wind-up through contact,
    and recovery after the standard 0.20 s follow-through grace.  It has no station-distance gate or x/y component, so
    the 20--35 cm lateral step can happen concurrently and cannot delay the swing clock.
    """
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    deficit = torch.clamp(float(min_height) - cmd.robot.data.root_pos_w[:, 2], min=0.0)
    scaled = deficit / max(float(std), 1.0e-6)
    raw = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)

    in_hold = getattr(motion, "in_hold", None)
    if in_hold is None:
        raise RuntimeError(
            "RallyFinalV3 root-height debt requires MotionCommand.in_hold; refusing a "
            "fail-open ready/recovery reward"
        )
    ready = (cmd.time_to_strike > ready_t_lo) & (cmd.time_to_strike < ready_t_hi)
    post = (cmd.time_to_strike < -post_t_lo) & (cmd.time_to_strike > -post_t_hi)
    gate = in_hold | ready | post
    cmd.metrics["rally_ready_root_height_deficit"] = deficit * gate.float()
    return raw * gate.float()


# ============================================================================================== #
# HITTER-PURE RALLY FINAL V2 (2026-07-10): deploy-ready foot discipline.  These terms are separate
# from the legacy global ``foot_orientation`` stack: both are phase-gated and neither adds an
# observation or an arrival-controlled clock. One is bounded over only the two legal hip-yaw
# reference DOFs implicated by toe-in; the other owns PRE-CLAMP ankle-roll q_des with a robust Huber
# debt that the normal measured-position joint-limit reward cannot see.
# ============================================================================================== #
def rally_foot_orientation_discipline(
    env: ManagerBasedRLEnv, command_name: str, motion_command_name: str, asset_cfg,
    margin: float = 0.12, std: float = 0.50,
    t_pre: float = 1.40, t_post: float = 1.80,
    default_during_hold: bool = False,
) -> torch.Tensor:
    """Bounded reference-relative discipline for exactly two hip-yaw joints.

    The clip's hip-roll and ankle-roll references can exceed the A3 hard limits, so following those
    channels would directly conflict with deploy-safe q_des shaping. Only the legal left/right
    hip-yaw references are used here to discourage toe-in while preserving lateral footwork. Ankle
    roll is handled exclusively by ``rally_ankle_qdes_saturation_penalty``.

    Each joint has a ``margin`` deadband, then a saturating ``1-exp(-(excess/std)^2)`` debt.  Averaging
    the two channels keeps the result in [0, 1] independent of joint count. This is an anti-toe-in
    regularizer, not lower-body plant imitation.
    """
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 2:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(
            f"RallyFinalV2 hip-yaw discipline requires exactly two resolved joints, got {count}"
        )
    q = asset.data.joint_pos[:, joint_ids]
    ref = motion.joint_pos[:, joint_ids]
    if default_during_hold:
        in_hold = getattr(motion, "in_hold", None)
        if in_hold is None:
            raise RuntimeError(
                "RallyFinalV3 hip-yaw discipline requires MotionCommand.in_hold when "
                "default_during_hold=True"
            )
        # Make the V3 contract explicit instead of relying on MotionCommand.joint_pos's current
        # hold substitution: hold targets deploy-default hip yaw, while the released swing keeps
        # the approved clip reference.  This is two-DOF discipline, not lower-body imitation.
        default = asset.data.default_joint_pos[:, joint_ids]
        ref = torch.where(in_hold.unsqueeze(-1), default, ref)
    error = torch.abs(q - ref)
    excess = torch.clamp(error - margin, min=0.0)
    raw = torch.mean(1.0 - torch.exp(-torch.square(excess / std)), dim=-1)
    gate = (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)

    # Reward-owned command metrics are training diagnostics only.  Zero outside the gate so reset
    # means report the ready/strike/recovery distribution rather than stale values from an old swing.
    gated_error = error * gate.unsqueeze(-1).float()
    cmd.metrics["rally_foot_orientation_error_mean"] = torch.mean(gated_error, dim=-1)
    cmd.metrics["rally_foot_orientation_error_max"] = torch.max(gated_error, dim=-1).values
    cmd.metrics["rally_foot_orientation_excess_frac"] = (
        (error > margin).float().mean(dim=-1) * gate.float()
    )
    return raw * gate.float()


def rally_ankle_qdes_saturation_penalty(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg,
    action_name: str = "joint_pos", safe_abs: float = 0.20, std: float = 0.10,
    t_pre: float = 1.40, t_post: float = 1.00,
) -> torch.Tensor:
    """Penalize ankle-roll targets that ask the deploy safety clamp to do control work.

    ``joint_limit`` observes the simulated joint *after* q_des was clamped, so a policy can request a
    wildly illegal ankle target and receive the same feedback as a legal at-limit request.  The
    clamped action term retains its absolute pre-clamp target specifically for this reward. Debt starts
    outside an absolute ``[-safe_abs,+safe_abs]`` envelope (default +/-0.20 rad), materially inside both
    Isaac's soft limit and the C++ deploy hard limit. A smooth-L1/Huber kernel retains a useful gradient
    for the old 0.7-0.9 rad requests instead of saturating exponentially. Exact soft-clamp, hard-clamp,
    safe-envelope fractions and maximum hard-limit excess are logged separately.
    """
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 2:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(
            f"RallyFinalV2 ankle q_des reward requires exactly two ankle-roll joints, got {count}"
        )

    action_term = env.action_manager.get_term(action_name)
    raw_qdes = getattr(action_term, "unclamped_processed_actions", None)
    if raw_qdes is None:
        raise RuntimeError(
            "RallyFinalV2 ankle q_des reward requires ClampedJointPositionAction's "
            "pre-clamp target buffer; refusing to measure already-clamped targets"
        )

    # SceneEntityCfg joint_ids use articulation columns; the action tensor uses the configured action
    # term order.  Resolve explicitly instead of assuming those orders happen to be identical.
    action_joint_ids = getattr(action_term, "_joint_ids", None)
    if isinstance(action_joint_ids, slice):
        action_joint_ids = list(range(len(action_term._asset.joint_names)))[action_joint_ids]
    elif torch.is_tensor(action_joint_ids):
        action_joint_ids = action_joint_ids.detach().cpu().tolist()
    elif action_joint_ids is not None:
        action_joint_ids = list(action_joint_ids)
    if action_joint_ids is None:
        raise RuntimeError("RallyFinalV2 cannot resolve joint order of the joint_pos action term")
    action_col = {int(joint_id): col for col, joint_id in enumerate(action_joint_ids)}
    missing = [int(joint_id) for joint_id in joint_ids if int(joint_id) not in action_col]
    if missing:
        raise RuntimeError(f"RallyFinalV2 ankle joints missing from joint_pos action term: {missing}")
    cols = torch.tensor(
        [action_col[int(joint_id)] for joint_id in joint_ids],
        dtype=torch.long,
        device=raw_qdes.device,
    )
    ankle_qdes = raw_qdes.index_select(-1, cols)

    soft_limits = asset.data.soft_joint_pos_limits[:, joint_ids, :]
    hard_all = getattr(asset.data, "joint_pos_limits", None)
    hard_limits = hard_all[:, joint_ids, :] if hard_all is not None else soft_limits
    soft_lo, soft_hi = soft_limits[..., 0], soft_limits[..., 1]
    hard_lo, hard_hi = hard_limits[..., 0], hard_limits[..., 1]
    safe = max(float(safe_abs), 0.0)
    safe_lo = torch.maximum(hard_lo, torch.full_like(hard_lo, -safe))
    safe_hi = torch.minimum(hard_hi, torch.full_like(hard_hi, safe))
    safe_excess = torch.clamp(safe_lo - ankle_qdes, min=0.0) + torch.clamp(
        ankle_qdes - safe_hi, min=0.0
    )
    soft_excess = torch.clamp(soft_lo - ankle_qdes, min=0.0) + torch.clamp(
        ankle_qdes - soft_hi, min=0.0
    )
    hard_excess = torch.clamp(hard_lo - ankle_qdes, min=0.0) + torch.clamp(
        ankle_qdes - hard_hi, min=0.0
    )
    scaled = safe_excess / max(float(std), 1e-6)
    huber = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    raw = torch.mean(huber, dim=-1)
    gate = (cmd.time_to_strike < t_pre) & (cmd.time_to_strike > -t_post)
    gate_f = gate.float()
    cmd.metrics["raw_ankle_qdes_clamp_frac"] = (
        (hard_excess > 0.0).float().mean(dim=-1) * gate_f
    )
    cmd.metrics["raw_ankle_qdes_soft_clamp_frac"] = (
        (soft_excess > 0.0).float().mean(dim=-1) * gate_f
    )
    cmd.metrics["raw_ankle_qdes_safe_limit_frac"] = (
        (safe_excess > 0.0).float().mean(dim=-1) * gate_f
    )
    cmd.metrics["raw_ankle_qdes_clamp_excess_max"] = (
        torch.max(hard_excess, dim=-1).values * gate_f
    )
    cmd.metrics["raw_ankle_qdes_abs_max"] = torch.max(torch.abs(ankle_qdes), dim=-1).values * gate_f
    return raw * gate_f


def racket_normal_alignment_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.10,
    std: float = 0.35,
) -> torch.Tensor:
    """Non-dead strike-window paddle-normal debt for fresh HitterPure training.

    The precise positive Gaussian normal reward has effectively zero gradient at the failed
    FinalV3 forehand error (about 66 deg). This companion term uses the same deploy-honest target
    normal, derived from the planner's commanded racket velocity, and supplies a smooth-L1 angular
    tail until the policy enters the Gaussian's useful basin. It is exactly zero inside ``margin``
    and outside the strike window; it adds no observation, curriculum, reference normal, or ball
    state. Configure it with a negative reward weight.
    """
    if margin < 0.0:
        raise ValueError(f"racket normal debt margin must be non-negative, got {margin}")
    if std <= 0.0:
        raise ValueError(f"racket normal debt std must be positive, got {std}")
    cmd = _cmd(env, command_name)
    cos_error = torch.sum(
        cmd.racket_normal_w * cmd.racket_target_normal_w, dim=-1
    # Keep acos' derivative finite at exact alignment/anti-alignment.  Values clipped at the
    # positive endpoint are already well inside the zero-debt margin.
    ).clamp(-1.0 + 1.0e-6, 1.0 - 1.0e-6)
    angle = torch.acos(cos_error)
    excess = torch.clamp(angle - float(margin), min=0.0)
    scaled = excess / float(std)
    debt = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    return debt * cmd.strike_window.float()


def racket_position_alignment_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.075,
    std: float = 0.35,
) -> torch.Tensor:
    """Non-dead strike-position debt that hands off to the precise positive Gaussian.

    A fresh FinalV3 policy was 0.5--1.0 m from the commanded strike point.  With the existing
    0.15-m exponential kernel that makes both reward and gradient effectively zero, especially
    after wrist-position imitation was removed.  This smooth-L1 tail compares the physical racket
    center with the same planner swing-through trajectory used by the precise positive reward,
    remains informative at metre-scale error, and is exactly zero inside ``margin``. It is
    strike-window-only and adds no actor observation, ball state, target curriculum, or deployment
    dependency. Configure it with a negative weight.
    """
    if margin < 0.0:
        raise ValueError(f"racket position debt margin must be non-negative, got {margin}")
    if std <= 0.0:
        raise ValueError(f"racket position debt std must be positive, got {std}")
    cmd = _cmd(env, command_name)
    target_pos_now = (
        cmd.racket_target_pos_w
        - cmd.racket_target_vel_w * cmd.time_to_strike.unsqueeze(-1)
    )
    error = torch.linalg.norm(cmd.racket_pos_w - target_pos_now, dim=-1)
    excess = torch.clamp(error - float(margin), min=0.0)
    scaled = excess / float(std)
    debt = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    return debt * cmd.strike_window.float()


def racket_exact_position_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.075,
    huber_scale: float = 0.025,
    window_s: float = 0.02,
) -> torch.Tensor:
    """Exact-contact debt against the static strike point in a local ±window.

    This term is deliberately independent of the adaptive position sigma. It is exactly zero
    inside ``margin`` and never follows the moving trajectory. Configure it with a negative
    RewardTerm weight; RewardManager applies the policy-step dt after this function returns.
    """
    if margin < 0.0:
        raise ValueError(
            f"racket exact position debt margin must be non-negative, got {margin}"
        )
    if huber_scale <= 0.0:
        raise ValueError(
            "racket exact position debt huber_scale must be positive, got "
            f"{huber_scale}"
        )
    if window_s < 0.0:
        raise ValueError(
            f"racket exact position debt window_s must be non-negative, got {window_s}"
        )
    cmd = _cmd(env, command_name)
    error = torch.linalg.norm(
        cmd.racket_pos_w - cmd.racket_target_pos_w, dim=-1
    )
    excess = torch.clamp(error - float(margin), min=0.0)
    scaled = excess / float(huber_scale)
    debt = torch.where(
        scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5
    )
    gate = cmd.time_to_strike.abs() <= float(window_s) + 1.0e-6
    return debt * gate.float()


def rally_joint_qdes_saturation_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    action_name: str = "joint_pos",
    std: float = 0.20,
    max_blend: float = 0.25,
) -> torch.Tensor:
    """Charge only q_des requests discarded by the deploy-faithful safety clamp.

    Without a pre-clamp debt PPO can rail raw actions far beyond the legal range and use the clamp
    as a hidden bang-bang controller (the failed run reached raw action maxima near 90). Legal
    targets cost exactly zero. Passive head columns are excluded because their separate reward owns
    those deliberately unused neurons. The mean term disciplines broad saturation; ``max_blend``
    keeps one exploding joint from disappearing in a 29-DOF mean.
    """
    if std <= 0.0:
        raise ValueError(f"joint q_des saturation std must be positive, got {std}")
    if not 0.0 <= max_blend <= 1.0:
        raise ValueError(f"joint q_des saturation max_blend must be in [0,1], got {max_blend}")
    cmd = _cmd(env, command_name)
    action_term = env.action_manager.get_term(action_name)
    raw_qdes = getattr(action_term, "unclamped_processed_actions", None)
    if raw_qdes is None:
        raise RuntimeError(
            "FinalV3 whole-joint q_des debt requires ClampedJointPositionAction's pre-clamp buffer"
        )

    active_cols = getattr(action_term, "_rally_active_qdes_cols", None)
    active_joint_ids = getattr(action_term, "_rally_active_qdes_joint_ids", None)
    if active_cols is None or active_joint_ids is None:
        action_joint_ids = getattr(action_term, "_joint_ids", None)
        if isinstance(action_joint_ids, slice):
            action_joint_ids = list(range(len(action_term._asset.joint_names)))[action_joint_ids]
        elif torch.is_tensor(action_joint_ids):
            action_joint_ids = action_joint_ids.detach().cpu().tolist()
        elif action_joint_ids is not None:
            action_joint_ids = list(action_joint_ids)
        if action_joint_ids is None or len(action_joint_ids) != raw_qdes.shape[-1]:
            raise RuntimeError("FinalV3 cannot resolve q_des action columns in articulation order")
        passive_cols = getattr(action_term, "_passive_action_cols", None)
        passive = set() if passive_cols is None else set(passive_cols.detach().cpu().tolist())
        active_list = [col for col in range(raw_qdes.shape[-1]) if col not in passive]
        if not active_list:
            raise RuntimeError("FinalV3 whole-joint q_des debt resolved no active action columns")
        active_cols = torch.tensor(active_list, dtype=torch.long, device=raw_qdes.device)
        active_joint_ids = torch.tensor(
            [action_joint_ids[col] for col in active_list],
            dtype=torch.long,
            device=raw_qdes.device,
        )
        # Cache constant name/order resolution; avoid CPU/GPU synchronization every control step.
        action_term._rally_active_qdes_cols = active_cols
        action_term._rally_active_qdes_joint_ids = active_joint_ids

    active_raw_qdes = raw_qdes.index_select(-1, active_cols)
    soft = action_term._asset.data.soft_joint_pos_limits.index_select(1, active_joint_ids)
    active_excess = torch.clamp(soft[..., 0] - active_raw_qdes, min=0.0) + torch.clamp(
        active_raw_qdes - soft[..., 1], min=0.0
    )
    scaled = active_excess / float(std)
    huber = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    mean_debt = torch.mean(huber, dim=-1)
    max_debt = torch.max(huber, dim=-1).values
    debt = (1.0 - float(max_blend)) * mean_debt + float(max_blend) * max_debt

    cmd.metrics["raw_joint_qdes_soft_clamp_frac"] = (active_excess > 0.0).float().mean(dim=-1)
    cmd.metrics["raw_joint_qdes_soft_clamp_excess_max"] = torch.max(active_excess, dim=-1).values
    cmd.metrics["raw_joint_qdes_abs_max"] = torch.max(torch.abs(active_raw_qdes), dim=-1).values
    return debt


def rally_all_joint_qdes_barrier(
    env: ManagerBasedRLEnv,
    command_name: str,
    action_name: str = "joint_pos",
    safe_margin_fraction: float = 0.05,
    std_fraction: float = 0.03,
    topk: int = 4,
    topk_blend: float = 0.75,
) -> torch.Tensor:
    """Range-normalized deploy-limit barrier over every A3 q_des action column.

    The hard limits come from Isaac's imported official A3 URDF and match the official MuJoCo/C++
    deploy table. A fractional inset avoids giving PPO a zero-cost target exactly on the SDK clamp.
    Unlike V11/V12's single maximum, the mean of the worst ``topk`` channels gives simultaneous
    ankle/shoulder/waist violations simultaneous gradient. All 31 action columns are included;
    passive head actions are not silently exempted from a deploy contract.
    """
    if not 0.0 <= safe_margin_fraction < 0.5:
        raise ValueError(f"safe_margin_fraction must be in [0,0.5), got {safe_margin_fraction}")
    if std_fraction <= 0.0 or topk <= 0 or not 0.0 <= topk_blend <= 1.0:
        raise ValueError(
            f"invalid all-joint barrier std/topk/blend: {std_fraction}/{topk}/{topk_blend}"
        )
    cmd = _cmd(env, command_name)
    action_term = env.action_manager.get_term(action_name)
    raw_qdes = getattr(action_term, "unclamped_processed_actions", None)
    if raw_qdes is None:
        raise RuntimeError(
            "RallyV13 all-joint q_des barrier requires the pre-clamp action buffer"
        )
    action_joint_ids = getattr(action_term, "_joint_ids", None)
    if isinstance(action_joint_ids, slice):
        action_joint_ids = list(range(len(action_term._asset.joint_names)))[action_joint_ids]
    elif torch.is_tensor(action_joint_ids):
        action_joint_ids = action_joint_ids.detach().cpu().tolist()
    elif action_joint_ids is not None:
        action_joint_ids = list(action_joint_ids)
    if action_joint_ids is None or len(action_joint_ids) != raw_qdes.shape[-1]:
        raise RuntimeError("RallyV13 cannot resolve all joint_pos action columns")
    ids = torch.as_tensor(action_joint_ids, dtype=torch.long, device=raw_qdes.device)
    hard_all = getattr(action_term._asset.data, "joint_pos_limits", None)
    if hard_all is None:
        raise RuntimeError("RallyV13 requires official hard joint_pos_limits; refusing soft fallback")
    hard = hard_all.index_select(1, ids)
    span = (hard[..., 1] - hard[..., 0]).clamp_min(1.0e-6)
    safe_lo = hard[..., 0] + float(safe_margin_fraction) * span
    safe_hi = hard[..., 1] - float(safe_margin_fraction) * span
    safe_excess = torch.clamp(safe_lo - raw_qdes, min=0.0) + torch.clamp(
        raw_qdes - safe_hi, min=0.0
    )
    hard_excess = torch.clamp(hard[..., 0] - raw_qdes, min=0.0) + torch.clamp(
        raw_qdes - hard[..., 1], min=0.0
    )
    scaled = safe_excess / (float(std_fraction) * span)
    per_joint = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    k = min(int(topk), int(per_joint.shape[-1]))
    mean_debt = torch.mean(per_joint, dim=-1)
    topk_debt = torch.topk(per_joint, k=k, dim=-1).values.mean(dim=-1)
    debt = (1.0 - float(topk_blend)) * mean_debt + float(topk_blend) * topk_debt
    cmd.metrics["raw_all_joint_qdes_safe_excess_frac"] = (
        (safe_excess > 0.0).float().mean(dim=-1)
    )
    cmd.metrics["raw_all_joint_qdes_hard_clip_count"] = (
        (hard_excess > 0.0).float().sum(dim=-1)
    )
    cmd.metrics["raw_all_joint_qdes_hard_excess_max"] = hard_excess.max(dim=-1).values
    cmd.metrics["raw_all_joint_qdes_topk_debt"] = topk_debt
    return debt


def rally_waist_qdes_saturation_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    action_name: str = "joint_pos",
    std: float = 0.10,
    max_blend: float = 1.0,
) -> torch.Tensor:
    """Targeted pre-clamp debt for waist roll/pitch requests.

    RallyV11's whole-body max channel can be owned by a much larger task-wrist request on the
    same tick, leaving no marginal signal for the waist even though the downstream SDK clamp
    still discards it.  This term resolves exactly ``waist_roll_joint`` and
    ``waist_pitch_joint`` in action-column order and measures their excess over Isaac's
    conservative soft limits before clamping.  Legal requests remain exactly free.
    """
    if std <= 0.0 or not 0.0 <= max_blend <= 1.0:
        raise ValueError(f"invalid waist q_des std/max_blend: {std}/{max_blend}")
    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 2:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(f"RallyV12 waist q_des debt requires exactly two joints, got {count}")

    action_term = env.action_manager.get_term(action_name)
    raw_qdes = getattr(action_term, "unclamped_processed_actions", None)
    if raw_qdes is None:
        raise RuntimeError(
            "RallyV12 waist q_des debt requires ClampedJointPositionAction's pre-clamp buffer"
        )
    action_joint_ids = getattr(action_term, "_joint_ids", None)
    if isinstance(action_joint_ids, slice):
        action_joint_ids = list(range(len(action_term._asset.joint_names)))[action_joint_ids]
    elif torch.is_tensor(action_joint_ids):
        action_joint_ids = action_joint_ids.detach().cpu().tolist()
    elif action_joint_ids is not None:
        action_joint_ids = list(action_joint_ids)
    if action_joint_ids is None:
        raise RuntimeError("RallyV12 cannot resolve joint order of the joint_pos action term")
    action_col = {int(joint_id): col for col, joint_id in enumerate(action_joint_ids)}
    missing = [int(joint_id) for joint_id in joint_ids if int(joint_id) not in action_col]
    if missing:
        raise RuntimeError(f"RallyV12 waist joints missing from joint_pos action term: {missing}")
    cols = torch.tensor(
        [action_col[int(joint_id)] for joint_id in joint_ids],
        dtype=torch.long,
        device=raw_qdes.device,
    )
    waist_qdes = raw_qdes.index_select(-1, cols)
    soft_limits = asset.data.soft_joint_pos_limits[:, joint_ids, :]
    excess = torch.clamp(soft_limits[..., 0] - waist_qdes, min=0.0) + torch.clamp(
        waist_qdes - soft_limits[..., 1], min=0.0
    )
    scaled = excess / float(std)
    per_joint = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    debt = (
        (1.0 - float(max_blend)) * torch.mean(per_joint, dim=-1)
        + float(max_blend) * torch.max(per_joint, dim=-1).values
    )
    cmd.metrics["raw_waist_qdes_soft_clamp_frac"] = (excess > 0.0).float().mean(dim=-1)
    cmd.metrics["raw_waist_qdes_soft_clamp_excess_max"] = torch.max(excess, dim=-1).values
    return debt


def _post_swing_recovery_gate_and_scale(
    env: ManagerBasedRLEnv,
    cmd,
    *,
    t_lo: float,
    t_hi: float,
    motion_command_name: str | None,
    include_replay: bool,
    curriculum_scaled: bool,
) -> tuple[torch.Tensor, float]:
    """Resolve the ordinary post-strike window plus V17 replay-hold state."""

    gate = (cmd.time_to_strike < -float(t_lo)) & (
        cmd.time_to_strike > -float(t_hi)
    )
    scale = 1.0
    if include_replay or curriculum_scaled:
        if not motion_command_name:
            raise ValueError(
                "replay/curriculum-scaled recovery debt requires motion_command_name"
            )
        motion = env.command_manager.get_term(motion_command_name)
        if include_replay:
            replay_active = getattr(motion, "post_swing_replay_active", None)
            if replay_active is None:
                raise RuntimeError(
                    "replay-aware recovery debt requires post_swing_replay_active"
                )
            gate = gate | replay_active
        if curriculum_scaled:
            scale = float(getattr(motion, "recovery_curriculum_scale", 0.0))
            if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
                raise RuntimeError(
                    f"invalid live recovery curriculum scale: {scale}"
                )
    return gate, scale


def post_swing_tilt_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.10,
    std: float = 0.20,
    t_lo: float = 0.10,
    t_hi: float = 1.55,
    motion_command_name: str | None = None,
    include_replay: bool = False,
    curriculum_scaled: bool = False,
) -> torch.Tensor:
    """Non-dead pelvis-tilt recovery debt after a short free follow-through.

    The approach and contact are untouched. After ``t_lo`` seconds this asks the base to return
    inside a small projected-gravity deadband and retains a linear gradient toward the physical
    fall boundary. It catches a slowly tipping robot that already has low translational speed.
    """
    if margin < 0.0 or std <= 0.0 or t_lo < 0.0 or t_hi <= t_lo:
        raise ValueError(
            "post-swing tilt debt requires margin>=0, std>0 and 0<=t_lo<t_hi; "
            f"got margin={margin}, std={std}, t_lo={t_lo}, t_hi={t_hi}"
        )
    cmd = _cmd(env, command_name)
    projected = getattr(cmd.robot.data, "projected_gravity_b", None)
    if projected is None:
        raise RuntimeError("post-swing tilt debt requires projected_gravity_b")
    tilt_signal = torch.linalg.norm(projected[:, :2], dim=-1)
    excess = torch.clamp(tilt_signal - float(margin), min=0.0)
    scaled = excess / float(std)
    debt = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    gate, curriculum_scale = _post_swing_recovery_gate_and_scale(
        env,
        cmd,
        t_lo=t_lo,
        t_hi=t_hi,
        motion_command_name=motion_command_name,
        include_replay=include_replay,
        curriculum_scaled=curriculum_scaled,
    )
    tilt_deg = torch.rad2deg(torch.asin(tilt_signal.clamp(0.0, 1.0)))
    previous = cmd.metrics.get("post_swing_base_tilt_deg", torch.zeros_like(tilt_deg))
    cmd.metrics["post_swing_base_tilt_deg"] = torch.where(gate, tilt_deg, previous)
    return debt * gate.float() * curriculum_scale


def recovery_safe_set_cost(
    env: ManagerBasedRLEnv,
    command_name: str,
    action_name: str,
    *,
    motion_command_name: str,
    t_lo: float = 0.10,
    t_hi: float = 1.55,
    tilt_safe: float = 0.10,
    tilt_width: float = 0.20,
    base_speed_safe: float = 0.20,
    base_speed_width: float = 0.40,
    yaw_rate_safe: float = 0.20,
    yaw_rate_width: float = 0.60,
    joint_speed_safe: float = 0.35,
    joint_speed_width: float = 1.00,
    foot_slip_safe: float = 0.03,
    foot_slip_width: float = 0.12,
    station_safe: float = 0.10,
    station_width: float = 0.20,
    heading_safe: float = 0.15,
    heading_width: float = 0.30,
    qdes_step_safe: float = 0.08,
    qdes_step_width: float = 0.12,
    qdes_second_safe: float = 0.10,
    qdes_second_width: float = 0.15,
    hard_margin_fraction: float = 0.05,
    brake_accel_waist: float = 9.0,
    brake_accel_arm: float = 3.0,
    brake_accel_leg: float = 5.0,
    max_blend: float = 0.50,
    topk: int = 3,
    include_replay: bool = True,
    include_ready_hold: bool = False,
    include_qdes_dynamics: bool = True,
    curriculum_scaled: bool = True,
) -> torch.Tensor:
    """Bounded recovery safe-set objective for V17 recipe revision 3.

    Each channel is normalized independently to ``[0,1]``. The final cost is half the worst
    channel plus half the mean of the three worst channels, so a single joint racing toward a
    hard rail cannot disappear in a 31-joint mean. The contact/strike window is untouched. In
    revision 3, the same bounded cost can supervise a sampled strict-READY hold before release;
    ordinary post-impact and replay recovery activation remain explicit phase masks.
    """

    cmd = _cmd(env, command_name)
    gate, scale = _post_swing_recovery_gate_and_scale(
        env,
        cmd,
        t_lo=t_lo,
        t_hi=t_hi,
        motion_command_name=motion_command_name,
        include_replay=include_replay,
        curriculum_scaled=curriculum_scaled,
    )
    post_strike_gate = (
        (cmd.time_to_strike < -float(t_lo))
        & (cmd.time_to_strike > -float(t_hi))
    )
    motion = env.command_manager.get_term(motion_command_name)
    replay_active = getattr(motion, "post_swing_replay_active", None)
    replay_gate = (
        replay_active
        if include_replay and torch.is_tensor(replay_active)
        else torch.zeros_like(gate)
    )
    ready_hold_gate = torch.zeros_like(gate)
    if include_ready_hold:
        held_metric = getattr(motion, "metrics", {}).get("in_hold")
        held = (
            held_metric > 0.5
            if torch.is_tensor(held_metric)
            else torch.zeros_like(gate)
        )
        required = getattr(cmd, "_ready_release_required", None)
        wait_steps = getattr(cmd, "_ready_release_wait_steps", None)
        if not torch.is_tensor(required) or not torch.is_tensor(wait_steps):
            raise RuntimeError(
                "READY-hold safe-set supervision requires sampled release state"
            )
        station_error_for_gate = torch.linalg.norm(
            cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1
        )
        quat_for_gate = cmd.base_quat_w
        forward_x_for_gate = 1.0 - 2.0 * (
            quat_for_gate[:, 2].square() + quat_for_gate[:, 3].square()
        )
        forward_y_for_gate = 2.0 * (
            quat_for_gate[:, 1] * quat_for_gate[:, 2]
            + quat_for_gate[:, 0] * quat_for_gate[:, 3]
        )
        heading_for_gate = torch.abs(
            torch.atan2(forward_y_for_gate, forward_x_for_gate)
        )
        near_ready_envelope = (
            station_error_for_gate
            <= float(station_safe) + float(station_width)
        ) & (
            heading_for_gate
            <= float(heading_safe) + float(heading_width)
        )
        ready_hold_gate = (
            required
            & held
            & ((wait_steps > 0) | near_ready_envelope)
        )
        gate = gate | ready_hold_gate
    if curriculum_scaled:
        scale = float(getattr(cmd, "recovery_curriculum_scale", scale))
        if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
            raise RuntimeError(
                f"invalid READY/safe-set curriculum scale: {scale}"
            )
    action = env.action_manager.get_term(action_name)
    required_action_fields = (
        "_select_action_joints",
        "_hard_lo",
        "_hard_hi",
        "_qdes_delta",
        "_qdes_second_difference",
        "_joint_names",
    )
    missing = [
        name for name in required_action_fields if not hasattr(action, name)
    ]
    if missing:
        raise RuntimeError(
            "recovery_safe_set_cost requires the V11-safe action audit; "
            f"missing={missing}"
        )

    data = cmd.robot.data
    projected = getattr(data, "projected_gravity_b", None)
    if projected is None:
        raise RuntimeError(
            "recovery_safe_set_cost requires projected_gravity_b"
        )
    tilt = torch.linalg.norm(projected[:, :2], dim=-1)
    base_speed = torch.linalg.norm(data.root_lin_vel_w[:, :2], dim=-1)
    yaw_rate = torch.abs(data.root_ang_vel_b[:, 2])
    joint_speed = torch.sqrt(torch.mean(data.joint_vel.square(), dim=-1))
    foot_slip = cmd.metrics.get("foot_slip_speed")
    if not torch.is_tensor(foot_slip):
        raise RuntimeError(
            "recovery_safe_set_cost requires live foot_slip_speed telemetry"
        )
    station_error = torch.linalg.norm(
        cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1
    )
    quat = cmd.base_quat_w
    forward_x = 1.0 - 2.0 * (
        quat[:, 2].square() + quat[:, 3].square()
    )
    forward_y = 2.0 * (
        quat[:, 1] * quat[:, 2] + quat[:, 0] * quat[:, 3]
    )
    heading = torch.abs(torch.atan2(forward_y, forward_x))

    active_mask = getattr(action, "_recovery_safe_set_active_mask", None)
    brake_accel = getattr(action, "_recovery_safe_set_brake_accel", None)
    brake_signature = (
        float(brake_accel_waist),
        float(brake_accel_arm),
        float(brake_accel_leg),
    )
    if (
        not torch.is_tensor(active_mask)
        or not torch.is_tensor(brake_accel)
        or getattr(action, "_recovery_safe_set_brake_signature", None)
        != brake_signature
    ):
        active_mask = torch.ones(
            action.action_dim, dtype=torch.bool, device=cmd.device
        )
        passive_cols = getattr(action, "_passive_action_cols", None)
        passive_indices = set()
        if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
            active_mask[passive_cols] = False
            passive_indices = {
                int(value) for value in passive_cols.detach().cpu().tolist()
            }
        brake_values = []
        for index, name in enumerate(action._joint_names):
            if index in passive_indices:
                continue
            lowered = str(name).lower()
            if "waist" in lowered:
                brake_values.append(float(brake_accel_waist))
            elif any(
                token in lowered
                for token in ("shoulder", "elbow", "wrist", "arm")
            ):
                brake_values.append(float(brake_accel_arm))
            else:
                brake_values.append(float(brake_accel_leg))
        brake_accel = torch.tensor(
            brake_values, dtype=data.joint_pos.dtype, device=cmd.device
        ).unsqueeze(0)
        action._recovery_safe_set_active_mask = active_mask
        action._recovery_safe_set_brake_accel = brake_accel
        action._recovery_safe_set_brake_signature = brake_signature
    q = action._select_action_joints(data.joint_pos)[:, active_mask]
    qd = action._select_action_joints(data.joint_vel)[:, active_mask]
    hard_lo = action._hard_lo[:, active_mask]
    hard_hi = action._hard_hi[:, active_mask]
    stopping_violation, stopping_distance, available_distance = (
        actual_q_stopping_violation(
            q,
            qd,
            hard_lo,
            hard_hi,
            brake_accel,
            margin_fraction=float(hard_margin_fraction),
        )
    )
    qdes_step = torch.max(
        torch.abs(action._qdes_delta[:, active_mask]), dim=-1
    ).values
    qdes_second = torch.max(
        torch.abs(action._qdes_second_difference[:, active_mask]), dim=-1
    ).values

    qdes_step_violation = normalized_upper_violation(
        qdes_step, qdes_step_safe, qdes_step_width
    )
    qdes_second_violation = normalized_upper_violation(
        qdes_second, qdes_second_safe, qdes_second_width
    )
    channel_values = [
        normalized_upper_violation(tilt, tilt_safe, tilt_width),
        normalized_upper_violation(base_speed, base_speed_safe, base_speed_width),
        normalized_upper_violation(yaw_rate, yaw_rate_safe, yaw_rate_width),
        normalized_upper_violation(joint_speed, joint_speed_safe, joint_speed_width),
        normalized_upper_violation(foot_slip, foot_slip_safe, foot_slip_width),
        normalized_upper_violation(station_error, station_safe, station_width),
        normalized_upper_violation(heading, heading_safe, heading_width),
    ]
    if bool(include_qdes_dynamics):
        channel_values.extend((qdes_step_violation, qdes_second_violation))
    channel_values.append(stopping_violation.max(dim=-1).values)
    channels = torch.stack(channel_values, dim=-1)
    cost = aggregate_recovery_violations(
        channels, topk=int(topk), max_blend=float(max_blend)
    )
    active_cost = cost * gate.float() * float(scale)
    _record_metric_snapshot(
        cmd.metrics, "recovery_safe_set_cost", active_cost
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_raw_cost",
        cost,
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_raw_cost_active",
        cost * gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_post_strike_gate",
        post_strike_gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_replay_gate",
        replay_gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_ready_hold_gate",
        ready_hold_gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_qdes_step_violation",
        qdes_step_violation,
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_qdes_second_violation",
        qdes_second_violation,
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_qdes_step_active_weighted",
        qdes_step * gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_safe_set_qdes_second_active_weighted",
        qdes_second * gate.float(),
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_actual_q_stopping_distance_max",
        stopping_distance.max(dim=-1).values,
    )
    _record_metric_snapshot(
        cmd.metrics,
        "recovery_actual_q_available_distance_min",
        available_distance.min(dim=-1).values,
    )
    _record_metric_snapshot(
        cmd.metrics, "recovery_qdes_step_max", qdes_step
    )
    _record_metric_snapshot(
        cmd.metrics, "recovery_qdes_second_difference_max", qdes_second
    )
    return active_cost


def post_swing_heading_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    margin: float = 0.087,
    std: float = 0.25,
    t_lo: float = 0.10,
    t_hi: float = 1.55,
    motion_command_name: str | None = None,
    include_replay: bool = False,
    curriculum_scaled: bool = False,
) -> torch.Tensor:
    """Non-saturating world-heading recovery debt after a short free follow-through.

    The yaw sibling of ``post_swing_xlock`` (x) and ``post_swing_tilt_debt`` (tilt). V15 shipped
    those two but nothing for yaw, and the only heading term it had — ``hold_heading`` — is gated
    to ``in_hold``, whose docstring states outright that "the swing's natural ±20° yaw excursion
    is untouched". Meanwhile ``hugwbc_yaw_rate`` pays an always-on reward for ZERO yaw rate, i.e.
    it taxes the very rotation needed to square back up. The 2026-07-25 audit measured the result:
    ``ready_station_heading_error_deg`` climbed 2.9° → 14.4° inside one run (the ready monitor's
    own threshold is 15.0°), and ``post_swing_base_heading_error_deg`` 4.75° → 12.2°, which closed
    the ``ready`` gate and with it the whole velocity curriculum.

    Same shape as the tilt sibling: exactly zero inside ``margin`` (default 0.087 rad = 5°, so a
    normal follow-through pays nothing), then smooth-L1 in ``|yaw| / std`` so the gradient does not
    die at large excursions the way an exp kernel would. Gated to ``-t_hi < tts < -t_lo`` so the
    approach, the contact and the first ``t_lo`` seconds of follow-through are untouched.

    ``yaw`` is the base x-axis heading in the world XY plane, 0 == world +x — the same convention
    as ``hold_heading`` (:func:`hold_heading`) and as every station/velocity semantic in the task.
    """
    if margin < 0.0 or std <= 0.0 or t_lo < 0.0 or t_hi <= t_lo:
        raise ValueError(
            "post-swing heading debt requires margin>=0, std>0 and 0<=t_lo<t_hi; "
            f"got margin={margin}, std={std}, t_lo={t_lo}, t_hi={t_hi}"
        )
    cmd = _cmd(env, command_name)
    q = cmd.base_quat_w  # (w, x, y, z)
    fwd_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    fwd_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw_abs = torch.abs(torch.atan2(fwd_y, fwd_x))
    excess = torch.clamp(yaw_abs - float(margin), min=0.0)
    scaled = excess / float(std)
    debt = torch.where(scaled <= 1.0, 0.5 * torch.square(scaled), scaled - 0.5)
    gate, curriculum_scale = _post_swing_recovery_gate_and_scale(
        env,
        cmd,
        t_lo=t_lo,
        t_hi=t_hi,
        motion_command_name=motion_command_name,
        include_replay=include_replay,
        curriculum_scaled=curriculum_scaled,
    )
    yaw_deg = torch.rad2deg(yaw_abs)
    previous = cmd.metrics.get("post_swing_heading_debt_deg", torch.zeros_like(yaw_deg))
    cmd.metrics["post_swing_heading_debt_deg"] = torch.where(gate, yaw_deg, previous)
    return debt * gate.float() * curriculum_scale


# ============================================================================================== #
# PRE-STRIKE STABILITY (2026-07-10, HOPEPingPongHitterPurePreStrikeStable) — three MINIMAL terms that
# fix the single-swing pre-strike instability (base still translating / turning when the swing arms, so
# the robot enters the hit carrying base velocity/yaw-rate → foot slip + weird compensation). All are
# NARROWLY gated (a short pre-strike window / the strike itself) so the normal approach footwork and the
# swing's own dynamics are UNTOUCHED — deliberately NOT a full-episode base-velocity penalty. They read
# only IMU-measurable base state (world root lin/ang vel, base quat) so they are deploy-honest and change
# NO actor observation. NOT the rally/hold/arrival stack (V5/V6) — these are single-swing shaping only.
# ============================================================================================== #
def pre_strike_base_vel_quiet(
    env: ManagerBasedRLEnv, command_name: str, std: float = 0.20, t_min: float = 0.05, t_max: float = 0.30
) -> torch.Tensor:
    """PENALTY: quiet the base world-frame PLANAR velocity in a SHORT window just BEFORE the strike.

    Failure it targets: the base is still shifting laterally (finishing the approach step) when the swing
    triggers, so the robot strikes with residual base xy speed → the planted foot slips and the legs make
    an odd compensation. This pays a BOUNDED "un-stillness" penalty ``1 − exp(−‖v_base_xy‖² / std²)`` (0
    when the base is still, saturating toward 1 as it moves) gated to ``t_min < time_to_strike < t_max``
    AND ``~strike_window``. Bounded ON PURPOSE: with the small negative weight it is a gentle nudge that
    can never out-vote the racket strike terms (an unbounded quadratic on velocity is a footgun far from
    the optimum). Deliberately NOT gated over the whole approach — the robot MUST still move to the
    station earlier; this only asks it to have arrived and settled by the last ~0.3 s. The explicit
    ``~strike_window`` conjunct keeps it out of the scored strike window even when ``t_min <
    strike_window_s`` (the base_station_settle audit lesson: a base-velocity gradient inside the window
    taxes the swing's weight transfer) — so the EFFECTIVE window is ``max(t_min, strike_window_s) < tts
    < t_max`` (with this task's defaults 0.05/0.12/0.30 → 0.12–0.30 s; t_min only bites if raised above
    strike_window_s). The follow-through (tts<0) is untouched. ``std`` [m/s] sets the velocity scale;
    weight NEGATIVE. Safe no-op on tasks whose tts clock never enters the window (returns 0)."""
    cmd = _cmd(env, command_name)
    v_sq = torch.sum(torch.square(cmd.robot.data.root_lin_vel_w[:, :2]), dim=-1)
    raw = 1.0 - torch.exp(-v_sq / std**2)
    gate = (cmd.time_to_strike > t_min) & (cmd.time_to_strike < t_max) & ~cmd.strike_window
    return raw * gate.float()


def pre_strike_base_angvel_quiet(
    env: ManagerBasedRLEnv, command_name: str, std: float = 0.30, t_min: float = 0.05, t_max: float = 0.30
) -> torch.Tensor:
    """PENALTY: quiet the base YAW-RATE in a SHORT window just BEFORE the strike (stop turning into the hit).

    Failure it targets: the base is still rotating (finishing a turn-to-face) when the swing triggers, so
    the robot strikes mid-turn — the "歪着打" / spinning-into-the-ball compensation. Penalizes the WORLD
    yaw-rate only (``root_ang_vel_w[z]``), NOT roll/pitch: roll/pitch during the wind-up is legitimate
    swing dynamics (and is already covered by the always-on ``base_ang_vel_xy`` regularizer), whereas a
    residual yaw-rate at contact is the "still turning" pathology. Bounded ``1 − exp(−ω_z² / std²)`` gated
    to ``t_min < time_to_strike < t_max`` AND ``~strike_window`` (same gate + rationale as
    ``pre_strike_base_vel_quiet``: effective window ``max(t_min, strike_window_s) < tts < t_max``, with
    this task's defaults → 0.12–0.30 s). NOT a full-episode turn ban: the robot may still turn during the
    earlier approach; this only asks the yaw to have settled by the last ~0.3 s. ⚠ WATCH: the demo windup
    itself carries some yaw-rate in this band — at std 0.30 a nominal windup turn (~0.5 rad/s) already
    reads ~0.9, so keep the weight SMALL (−0.05 ⇒ ≤ ~0.05/swing total, cosmetic vs racket 14/14/5); if
    the windup goes timid on a resume, RAISE std toward 0.6 before touching the weight. ``std`` [rad/s]
    sets the yaw-rate scale; weight NEGATIVE. Safe no-op outside the window."""
    cmd = _cmd(env, command_name)
    wz_sq = torch.square(cmd.robot.data.root_ang_vel_w[:, 2])
    raw = 1.0 - torch.exp(-wz_sq / std**2)
    gate = (cmd.time_to_strike > t_min) & (cmd.time_to_strike < t_max) & ~cmd.strike_window
    return raw * gate.float()


def strike_heading(
    env: ManagerBasedRLEnv, command_name: str, std: float = 0.35, window_s: float = 0.15
) -> torch.Tensor:
    """POSITIVE: face SQUARE (base heading → world +x) AT the strike (``|time_to_strike| < window_s``).

    Failure it targets: the robot arrives at the ball still yawed and hits side-on. This rewards a small
    base heading error only in a short window AROUND the strike (default ±0.15 s), leaving the rest of the
    episode free. ``exp(−yaw² / std²) · gate`` with yaw = the base x-axis heading in the world XY plane
    (0 == world +x, the strike heading every swing starts from in training — same measure as
    ``hold_heading``). Deliberately NOT a full-episode heading lock and NOT a new yaw curriculum: it only
    pays for being square at the moment of contact; the approach turn OUTSIDE ±window_s is untouched.
    ⚠ WATCH: INSIDE the window the demo clips strike slightly yawed (blade re-plane: fh ~+14°, bh ~+20°
    at contact), so at std 0.35 the natural pose earns 0.63 (fh) / 0.36 (bh) vs 1.0 at square — a mild
    pull (~0.3·weight/step at the bh peak) toward un-yawing the strike pose. That direction IS the point
    of the term ("击球时不要歪着打"), but it taxes the backhand hardest: if bh composite degrades on a
    resume, LOWER the weight or WIDEN std toward 0.5 before blaming the swing. ``std`` [rad] sets the
    kernel width; ``window_s`` [s] the half-window (independent of the tighter ``strike_window_s`` used
    by the racket terms). Weight POSITIVE."""
    cmd = _cmd(env, command_name)
    q = cmd.base_quat_w  # (w, x, y, z)
    # world-frame base forward = R(q) @ x_hat; heading = atan2(fwd_y, fwd_x)  (same as hold_heading)
    fwd_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    fwd_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw = torch.atan2(fwd_y, fwd_x)
    raw = torch.exp(-torch.square(yaw) / std**2)
    gate = cmd.time_to_strike.abs() < window_s
    return raw * gate.float()


def racket_strike_success(
    env: ManagerBasedRLEnv, command_name: str, std_pos: float, std_vel: float, std_normal: float
) -> torch.Tensor:
    """Local conjunction bonus: position × velocity × normal.

    RallyV15 uses the same ±0.04 s position-derived gate as moving guidance, but deliberately does
    not inherit its task-owned temporal scale. Additive velocity and normal keep their original full
    strike-window formulas and gates.
    """
    cmd = _cmd(env, command_name)
    rp = _moving_position_kernel(cmd, std_pos) * _position_guidance_gate(
        cmd
    ).float()
    rv = racket_velocity_tracking_exp(env, command_name, std_vel)
    rn = racket_normal_tracking_exp(env, command_name, std_normal)
    return rp * rv * rn


# ============================================================================================== #
# Tier-1 VIRTUAL-BALL outcome terms (rewardDesign.md). One-shot: non-zero ONLY on the exact-strike
# step of envs that passed the capture gate (cmd.vb_fired, set by RacketTargetCommand._vb_evaluate
# from the venue-fitted contact + coarse landing rollout). All are inert (all-zero) unless
# commands.racket_target.virtual_ball is enabled. Anti-farming gates follow the adversarial
# verification (verify_tier1-reward-soundness.md (c)):
#   1. the in-bounds bonus requires landing depth > net_x + vb_min_landing_depth (dink guard),
#   2. the capture gate requires a minimum paddle approach speed (phantom-block guard, in _vb_evaluate),
#   3. the pass_net CLEAR BONUS pays only for shots that also land legally (net-without-landing
#      guard); its height KERNEL is deliberately ungated shaping — see virtual_pass_net docstring.
# ============================================================================================== #
def virtual_pass_net(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Net-height shaping at the virtual net-plane crossing + fully-gated clear bonus.

    The Gaussian kernel on (net-crossing height - (net_top + margin)) pays for ANY shot that
    reaches the net plane inside the rollout horizon (v0 ``pass_net_margin`` semantics): it is the
    CLIMB gradient that teaches a flat-hitting policy to angle shots upward. Gating it on a legal
    landing (this term's original verify (c)4 reading) starved training completely — the E-champion
    warm-start crosses the net legally on only ~0.2% of strikes, so 2.5k iterations of vb_warmE14k3
    paid exactly zero virtual reward (2026-07-03 incident). The farming surface is bounded: the
    kernel requires an actual net-plane crossing, maxes only at the correct height, and is worth at
    most 1/swing; anti-farming gates stay in full on the +0.5 clear bonus here and on the
    landing/spin terms. RewTerm weight POSITIVE.
    """
    cmd = _cmd(env, command_name)
    target_z = cmd._vb_net_top_z + float(cmd.cfg.vb_net_margin)
    err = cmd.vb_net_z - target_z
    kernel = torch.exp(-(err**2) / float(cmd.cfg.vb_net_sigma) ** 2)
    legal = cmd.vb_net_clear & cmd.vb_landing_valid & cmd.vb_on_opponent
    raw = kernel * cmd.vb_net_crossed.float() + 0.5 * legal.float()
    return raw * cmd.vb_fired.float()


def virtual_landing(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Landing-accuracy kernel + fully-gated in-bounds bonus (v0 ``landing_in_opponent_half``).

    CLIMB-PHASE shape (2026-07-04): the Gaussian kernel on ||landing_xy - target_xy|| pays for any
    landing inside the rollout horizon — NOT gated on net clearance. The E-warm-started policy
    lands ~1.9 m short of the target and reaches the net plane on only a few % of strikes, so both
    net-gated terms stayed ~zero for 5k+ iterations (vb_warmE14k3/4); this kernel is the dense
    bottom rung that pays for hitting DEEPER. Net-farming risk is bounded: the rollout has no net
    collider, so the kernel is smooth through the net plane with its single max AT the target —
    drilling the net base (err ~0.75 m) always pays less than clearing and landing deeper. The
    +1.0 bonus keeps the full gate: net clearance AND on-opponent AND depth past
    net_x + vb_min_landing_depth (verify (c)1 dink guard). Re-tighten (restore the net_clear gate
    on the kernel, sigma back toward 0.3) once virtual_net_clear_rate is healthy. RewTerm weight
    POSITIVE.
    """
    cmd = _cmd(env, command_name)
    dist2 = torch.sum(torch.square(cmd.vb_landing_xy - cmd._vb_target_xy.unsqueeze(0)), dim=-1)
    kernel = torch.exp(-dist2 / float(cmd.cfg.vb_landing_sigma) ** 2)
    bonus = (cmd.vb_landing_valid & cmd.vb_net_clear & cmd.vb_on_opponent & cmd.vb_depth_ok).float()
    raw = kernel * cmd.vb_landing_valid.float() + bonus
    return raw * cmd.vb_fired.float()


def virtual_spin(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Outgoing-topspin reward (Ace's ws-term), only for shots that land legally.

    ``clamp(topspin / vb_spin_ref, 0, 1)`` where topspin is omega_plus projected on z_hat x d_hat
    of the outgoing direction; gated on a valid net-clearing in-bounds landing so brushing wild
    swipes that miss the table cannot farm spin. RewTerm weight POSITIVE (ramp toward parity with
    landing per the Ace precedent once the wiring is validated).
    """
    cmd = _cmd(env, command_name)
    legal = cmd.vb_landing_valid & cmd.vb_net_clear & cmd.vb_on_opponent
    if getattr(cmd.cfg, "vb_spin_mode", "topspin") == "minimize":
        # Stage-1 placement-first semantics (franco 2026-07-04): the BEST shot kills the incoming
        # spin — reward small outgoing |omega|, not topspin generation (which is ball quality and
        # deliberately unrewarded in stage 1).
        kernel = torch.exp(-(cmd.vb_spin_out_norm / float(cmd.cfg.vb_spin_min_sigma)) ** 2)
        raw = kernel * legal.float()
    else:
        raw = (cmd.vb_topspin / float(cmd.cfg.vb_spin_ref)).clamp(0.0, 1.0) * legal.float()
    return raw * cmd.vb_fired.float()


# --- footwork penalties (feet may STEP; we only punish BAD foot behaviour) --------------------- #
def foot_slip_sq(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize foot slip while in contact: sum over feet of contact * ||foot_xy_velocity||² (always on).
    A planted/landing foot should not skate. Positive magnitude; RewTerm weight is negative."""
    return _cmd(env, command_name).foot_slip_sq


def foot_velocity(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize excessive/violent foot velocity: sum over feet of ||foot_velocity||². Lets the foot step
    but discourages flailing. Positive magnitude; RewTerm weight is negative."""
    return _cmd(env, command_name).foot_vel_sq


def foot_drag(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize foot dragging: lateral foot speed while the foot is near the ground (skimming instead of
    lifting cleanly to step). Positive magnitude; RewTerm weight is negative."""
    return _cmd(env, command_name).foot_drag


def v17_support_foot_slide(
    env: ManagerBasedRLEnv,
    command_name: str,
    strike_exclusion_s: float = 0.18,
) -> torch.Tensor:
    """Weak support-foot sliding cost, disabled around racket contact.

    Contacting speed is already zeroed per foot by the command term.  Keeping
    this separate from swing-foot clearance avoids charging legitimate airborne
    translation, and the strike exclusion prevents lower-body shaping from
    taxing the demonstrated hit itself.
    """

    cmd = _cmd(env, command_name)
    slip = torch.square(cmd._foot_slip_speed_per_foot).sum(dim=-1)
    active = torch.abs(cmd.time_to_strike) > float(strike_exclusion_s)
    _record_metric_snapshot(
        cmd.metrics, "v17_support_foot_slide", slip * active.float()
    )
    return slip * active.float()


def _v17_local_ground_height(
    env: ManagerBasedRLEnv, foot_xy_w: torch.Tensor
) -> torch.Tensor:
    """Bilinearly sample the exact receipt-bound rough height field."""

    scene_cfg = getattr(env.scene, "cfg", None)
    if scene_cfg is None:
        scene_cfg = getattr(getattr(env, "cfg", None), "scene", None)
    patch = (
        None
        if scene_cfg is None
        else getattr(scene_cfg, "rough_ground_patch", None)
    )
    spawn = None if patch is None else getattr(patch, "spawn", None)
    if spawn is None:
        return torch.zeros(foot_xy_w.shape[:-1], device=foot_xy_w.device)

    signature = (
        int(spawn.seed),
        str(spawn.height_field_sha256),
        tuple(float(v) for v in spawn.height_range_m),
    )
    cached = getattr(env, "_v17_terrain_reward_cache", None)
    if cached is None or cached["signature"] != signature:
        import numpy as np

        from whole_body_tracking.tasks.tracking.terrain_patch import (
            HORIZONTAL_SCALE_M,
            VERTICAL_SCALE_M,
            build_patch_height_field,
            height_field_sha256,
        )

        hf = build_patch_height_field(
            spawn.height_range_m,
            spawn.flat_from_x_m,
            spawn.x_min_m,
            spawn.x_max_m,
            spawn.y_half_m,
            np.random.default_rng(int(spawn.seed)),
        )
        actual_sha = height_field_sha256(hf)
        if actual_sha != str(spawn.height_field_sha256):
            raise RuntimeError(
                "RallyV17 reward/collision terrain receipt mismatch: "
                f"reward={actual_sha}, collision={spawn.height_field_sha256}"
            )
        cached = {
            "signature": signature,
            "height": torch.as_tensor(
                hf, dtype=torch.float32, device=foot_xy_w.device
            )
            * float(VERTICAL_SCALE_M),
            "horizontal_scale_m": float(HORIZONTAL_SCALE_M),
            "x_min_m": float(spawn.x_min_m),
            "y_half_m": float(spawn.y_half_m),
        }
        setattr(env, "_v17_terrain_reward_cache", cached)

    local_xy = foot_xy_w - env.scene.env_origins[:, None, :2]
    row = (
        local_xy[..., 0] - cached["x_min_m"]
    ) / cached["horizontal_scale_m"]
    col = (
        local_xy[..., 1] + cached["y_half_m"]
    ) / cached["horizontal_scale_m"]
    height = cached["height"]
    row0 = torch.floor(row).long().clamp(0, height.shape[0] - 2)
    col0 = torch.floor(col).long().clamp(0, height.shape[1] - 2)
    row_mix = (row - row0.float()).clamp(0.0, 1.0)
    col_mix = (col - col0.float()).clamp(0.0, 1.0)
    h00 = height[row0, col0]
    h10 = height[row0 + 1, col0]
    h01 = height[row0, col0 + 1]
    h11 = height[row0 + 1, col0 + 1]
    return (
        h00 * (1.0 - row_mix) * (1.0 - col_mix)
        + h10 * row_mix * (1.0 - col_mix)
        + h01 * (1.0 - row_mix) * col_mix
        + h11 * row_mix * col_mix
    )


def v17_terrain_swing_clearance(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg,
    target_clearance_m: float = 0.05,
    ankle_to_sole_m: float = 0.067,
    sole_forward_m: float = 0.04,
    station_move_gate_m: float = 0.03,
    strike_exclusion_s: float = 0.18,
) -> torch.Tensor:
    """Penalize insufficient airborne sole clearance relative to local terrain.

    The term is active only while a station correction is still required and a
    foot is actually airborne.  It never adds height sensing to the actor.
    """

    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    foot_pos = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    foot_quat = asset.data.body_quat_w[:, asset_cfg.body_ids, :]
    # A3 vendor MuJoCo defines both foot sites at (0.04, 0, -0.067) m in the
    # ankle-roll frame. Rotate that local point with the live foot quaternion
    # instead of treating ankle-to-sole as a fixed world-z subtraction.
    sole_offset = torch.zeros_like(foot_pos)
    sole_offset[..., 0] = float(sole_forward_m)
    sole_offset[..., 2] = -float(ankle_to_sole_m)
    sole_pos = foot_pos + quat_apply(
        foot_quat.reshape(-1, 4), sole_offset.reshape(-1, 3)
    ).reshape_as(foot_pos)
    ground = _v17_local_ground_height(env, sole_pos[..., :2])
    clearance = sole_pos[..., 2] - ground
    swing = ~cmd._feet_in_contact
    station_error = torch.linalg.norm(
        cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1
    )
    active_env = (
        (station_error > float(station_move_gate_m))
        & (torch.abs(cmd.time_to_strike) > float(strike_exclusion_s))
    )
    active = swing & active_env.unsqueeze(-1)
    shortfall = torch.clamp(float(target_clearance_m) - clearance, min=0.0)
    debt = (torch.square(shortfall) * active.float()).sum(dim=-1)
    denominator = active.float().sum(dim=-1).clamp(min=1.0)
    mean_clearance = (clearance * active.float()).sum(dim=-1) / denominator
    _record_metric_snapshot(
        cmd.metrics,
        "v17_swing_clearance_local_m",
        torch.where(active.any(dim=-1), mean_clearance, torch.zeros_like(debt)),
    )
    _record_metric_snapshot(cmd.metrics, "v17_swing_clearance_debt", debt)
    return debt


def v17_soft_landing(
    env: ManagerBasedRLEnv,
    command_name: str,
    safe_downspeed_mps: float = 0.20,
    strike_exclusion_s: float = 0.18,
) -> torch.Tensor:
    """Penalize only the excess downward speed on a contact rising edge."""

    cmd = _cmd(env, command_name)
    excess = torch.clamp(
        cmd._foot_touchdown_downspeed - float(safe_downspeed_mps), min=0.0
    )
    active = torch.abs(cmd.time_to_strike) > float(strike_exclusion_s)
    debt = torch.square(excess).sum(dim=-1) * active.float()
    _record_metric_snapshot(cmd.metrics, "v17_soft_landing_debt", debt)
    return debt


def arm_overreach(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Anti-arm-only: penalize solving the target by maxing the arm out — fraction of ARM joints within
    10% of a position limit. Encourages using the body/legs to bring the target into a comfortable arm
    range instead of stretching. Positive in [0,1]; RewTerm weight is negative."""
    return _cmd(env, command_name).arm_overreach_frac


def prestrike_waist_twist(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Anti twist-instead-of-step: penalize |waist_yaw|+|waist_roll| deviation from neutral BEFORE the
    strike. Widening the racket-target box alone did NOT force footwork — the policy just rotated its
    torso (waist yaw/roll) to face a lateral target while its feet stayed planted (arm_overreach stayed
    ~0, legs frozen). This term makes that twist costly during the approach, so getting behind a far
    target requires STEPPING. Gated by ``pre_strike`` ONLY (the strike swing's rotation is untouched) and
    ``waist_pitch`` is excluded (that is the swing wind-up / lean, not a lateral-reach cheat). Returns a
    positive magnitude (radians); the RewTerm weight is negative."""
    cmd = _cmd(env, command_name)
    return cmd.waist_twist * cmd.pre_strike.float()


# --- strike-window stability (penalize wobble/bob/skate AT the hit; gated to the strike window) - #
def strike_proj_grav_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize base tilt (||projected_gravity_xy||) DURING the strike window — be upright at the hit."""
    cmd = _cmd(env, command_name)
    return cmd.proj_grav_xy * cmd.strike_window.float()


def strike_base_ang_vel(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize base roll/pitch rate (||base_ang_vel_xy||) DURING the strike window."""
    cmd = _cmd(env, command_name)
    return cmd.base_ang_vel_xy_norm * cmd.strike_window.float()


def prestrike_proj_grav_xy(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Sim2real balance shaping (CHANGE 3): penalize base/torso forward TILT (||projected_gravity_xy||, a
    POSITION quantity) DURING the approach (pre_strike). Together with the existing strike-window
    ``strike_upright`` this keeps the CoM over the support base THROUGH the whole swing — the forward
    pitch-over is exactly the AGI-MuJoCo failure mode. Deliberately NOT an angular-velocity penalty: a
    base-ang-vel penalty is anti-correlated with swing power and is gameable; projected-gravity tilt is a
    pose, so it does not fight the swing. Gated by pre_strike ONLY (the strike window is covered by
    strike_upright). Positive magnitude; the RewTerm weight is NEGATIVE."""
    cmd = _cmd(env, command_name)
    return cmd.proj_grav_xy * cmd.pre_strike.float()


def strike_foot_velocity(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize foot motion (sum ||foot_velocity||²) DURING the strike window — plant for the hit."""
    cmd = _cmd(env, command_name)
    return cmd.foot_vel_sq * cmd.strike_window.float()


def strike_vertical_bob(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize vertical base velocity (|base_lin_vel_z|) DURING the strike window — no bob at the hit."""
    cmd = _cmd(env, command_name)
    return cmd.vertical_speed * cmd.strike_window.float()


# ============================================================================================== #
# Sim2real: torque-saturation penalty (CHANGE 2). Discourage the policy from demanding torque the
# EXPLICIT clipped-PD motor cannot deliver. Under IdealPDActuatorCfg the model computes the pre-clip
# effort (kp*(q_des-q)+kd*(-qd)) and clips it to ±effort_limit; the ratio |computed| / effort_limit >1
# is exactly the over-demand that lags on the real robot. Penalizing the mean over-limit fraction over
# the arm + waist joints teaches a swing that lives inside the torque envelope (the elbow was measured
# at ~6.7x its 24 Nm limit in the failing trace). Uses ``data.computed_torque`` (Isaac copies each
# actuator's PRE-clip computed_effort into it) and ``data.joint_effort_limits`` (the per-joint sim
# limit written from effort_limit_sim). Both degrade to a 0 reward if unavailable, so it can never crash.
# ============================================================================================== #
_TORQUE_SAT_JOINT_EXPR = [".*shoulder.*", ".*elbow.*", ".*wrist.*", "waist_.*_joint"]


def _torque_sat_joint_idx(env: ManagerBasedRLEnv, command_name: str):
    """Resolve+cache the arm+waist joint indices on the command term (once)."""
    cmd = _cmd(env, command_name)
    idx = getattr(cmd, "_torque_sat_joint_idx", None)
    if idx is None:
        try:
            idx = list(cmd.robot.find_joints(_TORQUE_SAT_JOINT_EXPR)[0])
        except Exception:
            idx = []
        cmd._torque_sat_joint_idx = idx  # cache (empty list means "unresolvable")
    return cmd, idx


def arm_torque_saturation(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Mean over-limit fraction of the COMPUTED (pre-clip) effort over the arm + waist joints:
    ``mean_j relu(|computed_torque_j| / effort_limit_j - 1)``. 0 when every arm/waist joint is inside its
    torque envelope; grows as the swing demands un-deliverable torque (the explicit-PD saturation that
    tips the free base in AGI's MuJoCo). Positive magnitude; the RewTerm weight is NEGATIVE."""
    cmd, idx = _torque_sat_joint_idx(env, command_name)
    data = cmd.robot.data
    tau = getattr(data, "computed_torque", None)
    lim = getattr(data, "joint_effort_limits", None)
    if not idx or tau is None or lim is None:
        z = torch.zeros(cmd.num_envs, device=cmd.device)
        cmd.metrics["arm_torque_sat_frac"] = z
        return z
    tau_a = torch.abs(tau[:, idx])
    lim_a = lim[:, idx].clamp(min=1e-3)  # guard against a 0/inf limit
    over = (tau_a / lim_a - 1.0).clamp(min=0.0)  # relu(ratio - 1): the un-deliverable fraction
    frac = over.mean(dim=-1)
    cmd.metrics["arm_torque_sat_frac"] = frac  # watch-metric: should fall toward 0 during fine-tune
    return frac

def _mask_when_upper_intervened(
    env, reward: torch.Tensor, intervention_action_name: str | None
) -> torch.Tensor:
    """Remove actor-uncontrollable upper-body reward during HUGWBC action replacement.

    HUGWBC separates a clean group from an upper-body-intervention group and masks upper-body
    regularization while an external controller owns those joints.  HITTER adds an upper-body
    imitation objective, so the same ownership rule must cover that objective as well: charging
    the actor for shoulder/elbow targets that the intervention path replaced would inject an
    impossible learning target into the intervention half of V15.  Legacy tasks pass ``None`` and
    keep their reward byte-for-byte unchanged.
    """
    if intervention_action_name is None:
        return reward
    action_term = env.action_manager.get_term(intervention_action_name)
    indicator = getattr(action_term, "upper_intervention_indicator", None)
    if indicator is None:
        raise RuntimeError(
            f"Action term {intervention_action_name!r} does not expose "
            "upper_intervention_indicator"
        )
    clean = indicator.squeeze(-1) < 0.5
    return reward * clean.float()


def _zero_near_racket_strike(
    env,
    reward: torch.Tensor,
    racket_command_name: str | None,
    strike_free_pre_s: float,
    follow_through_free_s: float,
) -> torch.Tensor:
    """Release the motion teacher only where the sampled racket objective must own the action."""
    if racket_command_name is None:
        return reward
    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError(
            "motion-body strike release windows must be non-negative, got "
            f"{strike_free_pre_s}/{follow_through_free_s}"
        )
    racket_cmd = env.command_manager.get_term(racket_command_name)
    free = (
        (racket_cmd.time_to_strike <= float(strike_free_pre_s))
        & (racket_cmd.time_to_strike >= -float(follow_through_free_s))
    )
    return torch.where(free, torch.zeros_like(reward), reward)


def motion_body_pos_swing_only(
    env,
    command_name: str,
    std: float,
    body_names=None,
    intervention_action_name: str | None = None,
    racket_command_name: str | None = None,
    strike_free_pre_s: float = 0.0,
    follow_through_free_s: float = 0.0,
):
    """motion_relative_body_position_error_exp gated to ~in_hold (2026-07-05): during
    hold the joint reference is the default STAND (commands.joint_pos) while the frozen
    body refs still show clip frame 0's crouch — un-gated, the two imitation pulls
    fight and the policy settles into the splayed-feet crouch-stand. Swing-only."""
    from .rewards import motion_relative_body_position_error_exp
    cmd = env.command_manager.get_term(command_name)
    r = motion_relative_body_position_error_exp(env, command_name, std, body_names)
    r = torch.where(cmd.in_hold, torch.zeros_like(r), r)
    r = _zero_near_racket_strike(
        env, r, racket_command_name, strike_free_pre_s, follow_through_free_s
    )
    return _mask_when_upper_intervened(env, r, intervention_action_name)


def motion_body_ori_swing_only(
    env,
    command_name: str,
    std: float,
    body_names=None,
    intervention_action_name: str | None = None,
    racket_command_name: str | None = None,
    strike_free_pre_s: float = 0.0,
    follow_through_free_s: float = 0.0,
):
    """See motion_body_pos_swing_only."""
    from .rewards import motion_relative_body_orientation_error_exp
    cmd = env.command_manager.get_term(command_name)
    r = motion_relative_body_orientation_error_exp(env, command_name, std, body_names)
    r = torch.where(cmd.in_hold, torch.zeros_like(r), r)
    r = _zero_near_racket_strike(
        env, r, racket_command_name, strike_free_pre_s, follow_through_free_s
    )
    return _mask_when_upper_intervened(env, r, intervention_action_name)


def _relative_velocity_tracking_exp(
    reference_relative: torch.Tensor,
    robot_relative: torch.Tensor,
    std: float,
) -> torch.Tensor:
    """BeyondMimic's mean-squared exponential kernel on already-relative velocities."""
    error = torch.sum(torch.square(reference_relative - robot_relative), dim=-1)
    return torch.exp(-torch.mean(error, dim=-1) / std**2)


def _quat_rotate_inverse_wxyz(quaternion: torch.Tensor, vector: torch.Tensor) -> torch.Tensor:
    """Rotate world vectors into a unit ``(w,x,y,z)`` quaternion frame, with broadcasting."""
    qw = quaternion[..., :1]
    # ``torch.cross`` itself does not promise broadcasting on every Isaac/PyTorch version.
    qv = quaternion[..., 1:] + torch.zeros_like(vector)
    return (
        (2.0 * torch.square(qw) - 1.0) * vector
        - 2.0 * qw * torch.cross(qv, vector, dim=-1)
        + 2.0 * qv * torch.sum(qv * vector, dim=-1, keepdim=True)
    )


def _anchor_relative_linear_velocity_tracking_exp(
    reference_body_velocity: torch.Tensor,
    robot_body_velocity: torch.Tensor,
    reference_body_position: torch.Tensor,
    robot_body_position: torch.Tensor,
    reference_anchor_position: torch.Tensor,
    robot_anchor_position: torch.Tensor,
    reference_anchor_velocity: torch.Tensor,
    robot_anchor_velocity: torch.Tensor,
    reference_anchor_ang_velocity: torch.Tensor,
    robot_anchor_ang_velocity: torch.Tensor,
    reference_anchor_quat: torch.Tensor,
    robot_anchor_quat: torch.Tensor,
    std: float,
) -> torch.Tensor:
    """Compare body linear velocities relative to each trajectory's complete anchor twist."""
    reference_offset = reference_body_position - reference_anchor_position[:, None, :]
    robot_offset = robot_body_position - robot_anchor_position[:, None, :]
    reference_anchor_omega = reference_anchor_ang_velocity[:, None, :] + torch.zeros_like(
        reference_offset
    )
    robot_anchor_omega = robot_anchor_ang_velocity[:, None, :] + torch.zeros_like(robot_offset)
    reference_rigid_velocity = reference_anchor_velocity[:, None, :] + torch.cross(
        reference_anchor_omega, reference_offset, dim=-1
    )
    robot_rigid_velocity = robot_anchor_velocity[:, None, :] + torch.cross(
        robot_anchor_omega, robot_offset, dim=-1
    )
    reference_relative = _quat_rotate_inverse_wxyz(
        reference_anchor_quat[:, None, :], reference_body_velocity - reference_rigid_velocity
    )
    robot_relative = _quat_rotate_inverse_wxyz(
        robot_anchor_quat[:, None, :], robot_body_velocity - robot_rigid_velocity
    )
    return _relative_velocity_tracking_exp(reference_relative, robot_relative, std)


def _anchor_relative_angular_velocity_tracking_exp(
    reference_body_velocity: torch.Tensor,
    robot_body_velocity: torch.Tensor,
    reference_anchor_velocity: torch.Tensor,
    robot_anchor_velocity: torch.Tensor,
    reference_anchor_quat: torch.Tensor,
    robot_anchor_quat: torch.Tensor,
    std: float,
) -> torch.Tensor:
    """Compare body angular velocity relative to each anchor, in that anchor's local frame."""
    reference_relative = _quat_rotate_inverse_wxyz(
        reference_anchor_quat[:, None, :],
        reference_body_velocity - reference_anchor_velocity[:, None, :],
    )
    robot_relative = _quat_rotate_inverse_wxyz(
        robot_anchor_quat[:, None, :],
        robot_body_velocity - robot_anchor_velocity[:, None, :],
    )
    return _relative_velocity_tracking_exp(reference_relative, robot_relative, std)


def _zero_during_hold(value: torch.Tensor, in_hold: torch.Tensor) -> torch.Tensor:
    """Apply the deploy-hidden hold gate with an exact zero, never a stillness income."""
    return torch.where(in_hold, torch.zeros_like(value), value)


def _motion_body_indexes_or_fail(command, body_names) -> list[int]:
    """Resolve the configured tracked-body subset without allowing a typo to produce NaNs."""
    body_indexes = [
        i
        for i, name in enumerate(command.cfg.body_names)
        if body_names is None or name in body_names
    ]
    if not body_indexes:
        raise RuntimeError(
            "RallyFinalV3 anchor-relative velocity imitation resolved no tracked bodies; "
            f"requested={body_names!r}, available={command.cfg.body_names!r}"
        )
    return body_indexes


def motion_body_lin_vel_anchor_relative_swing_only(
    env,
    command_name: str,
    std: float,
    body_names=None,
    intervention_action_name: str | None = None,
    racket_command_name: str | None = None,
    strike_free_pre_s: float = 0.0,
    follow_through_free_s: float = 0.0,
):
    """Anchor-relative upper-body linear-velocity imitation, disabled during ready hold.

    V7 contains a 10--12 cm common-mode pelvis drop immediately after release.  Tracking global
    body velocities would reward reproducing that collapse and would also tax any residual lateral
    station correction.  Removing each side's complete torso twist (translation plus
    ``omega x radius``), then comparing in its own torso frame, preserves the arm velocity prior
    without importing either base motion.  The hold remains exactly zero-income.
    """
    cmd = env.command_manager.get_term(command_name)
    body_indexes = _motion_body_indexes_or_fail(cmd, body_names)
    reward = _anchor_relative_linear_velocity_tracking_exp(
        cmd.body_lin_vel_w[:, body_indexes],
        cmd.robot_body_lin_vel_w[:, body_indexes],
        cmd.body_pos_w[:, body_indexes],
        cmd.robot_body_pos_w[:, body_indexes],
        cmd.anchor_pos_w,
        cmd.robot_anchor_pos_w,
        cmd.anchor_lin_vel_w,
        cmd.robot_anchor_lin_vel_w,
        cmd.anchor_ang_vel_w,
        cmd.robot_anchor_ang_vel_w,
        cmd.anchor_quat_w,
        cmd.robot_anchor_quat_w,
        std,
    )
    reward = _zero_during_hold(reward, cmd.in_hold)
    reward = _zero_near_racket_strike(
        env, reward, racket_command_name, strike_free_pre_s, follow_through_free_s
    )
    return _mask_when_upper_intervened(env, reward, intervention_action_name)


def motion_body_ang_vel_anchor_relative_swing_only(
    env,
    command_name: str,
    std: float,
    body_names=None,
    intervention_action_name: str | None = None,
    racket_command_name: str | None = None,
    strike_free_pre_s: float = 0.0,
    follow_through_free_s: float = 0.0,
):
    """Anchor-relative angular-velocity counterpart to the V3 linear-velocity term."""
    cmd = env.command_manager.get_term(command_name)
    body_indexes = _motion_body_indexes_or_fail(cmd, body_names)
    reward = _anchor_relative_angular_velocity_tracking_exp(
        cmd.body_ang_vel_w[:, body_indexes],
        cmd.robot_body_ang_vel_w[:, body_indexes],
        cmd.anchor_ang_vel_w,
        cmd.robot_anchor_ang_vel_w,
        cmd.anchor_quat_w,
        cmd.robot_anchor_quat_w,
        std,
    )
    reward = _zero_during_hold(reward, cmd.in_hold)
    reward = _zero_near_racket_strike(
        env, reward, racket_command_name, strike_free_pre_s, follow_through_free_s
    )
    return _mask_when_upper_intervened(env, reward, intervention_action_name)


def motion_global_anchor_ori_windup_only(
    env,
    command_name: str,
    racket_command_name: str,
    std: float,
    min_time_to_strike: float = 0.25,
):
    """Use the V7 torso-orientation prior only in early windup.

    The ready hold belongs to station/upright control.  Around contact and throughout recovery,
    V7's 22--32 degree torso-yaw reference directly opposes the front-facing and heading objectives,
    so those phases receive exactly zero anchor-orientation income.  Early released windup retains
    the original prior and therefore does not discard the swing style wholesale.
    """
    from .rewards import motion_global_anchor_orientation_error_exp

    cmd = env.command_manager.get_term(command_name)
    racket = env.command_manager.get_term(racket_command_name)
    reward = motion_global_anchor_orientation_error_exp(env, command_name, std)
    enabled = (~cmd.in_hold) & (racket.time_to_strike > float(min_time_to_strike))
    return torch.where(enabled, reward, torch.zeros_like(reward))


def foot_orientation_discipline(env, command_name: str, asset_cfg, hold_gate: bool = False):
    """L1 deviation of the foot-orientation joints (hip yaw/roll, ankle roll) from the
    REFERENCE joint positions — hold-aware via commands.joint_pos (default stand during
    hold, clip footwork during swings). 2026-07-05: with no joint-level imitation in
    the stack these DOF were reward-free, and the policy twisted the feet to
    -1.13/+0.90 rad during swings/side-switches vs a reference envelope of ±0.41
    (Gate 2.5 diag) — the 'weird foot placement' at strike/switch. Use a NEGATIVE
    weight (penalty); keep it small so it disciplines feet without taxing the lunge.

    ``hold_gate`` (2026-07-08 rally-recovery fix): when True, this penalty is ZEROED
    during the recovery hold (``cmd.in_hold``). During a hold the reference joint_pos is
    the SQUARE default stand, so the term penalizes exactly the hip_yaw/hip_roll/ankle
    deviation the policy needs to RE-SQUARE its base back to world +x — it directly taxes
    heading recovery (the very skill hold_heading teaches). Gating it off in the hold frees
    the turn while keeping swing-phase toe-in discipline fully intact. NO-OP on any task
    without holds (``in_hold`` never True — e.g. the single-swing precision gate
    HOPEPingPongHitterPure, hold_steps_range [0,0]). Default False = legacy always-on, so
    every existing task/RewTerm that omits this param is byte-identical.
    """
    cmd = env.command_manager.get_term(command_name)
    asset = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    ref = cmd.joint_pos[:, asset_cfg.joint_ids]
    penalty = torch.sum(torch.abs(q - ref), dim=1)
    if hold_gate:
        penalty = torch.where(cmd.in_hold, torch.zeros_like(penalty), penalty)
    return penalty


def hold_upper_pose_imitation(env, command_name: str, std: float, asset_cfg=None):
    """HOLD-gated UPPER-BODY joint-space imitation of the CURRENT clip's frame-0 ready pose.

    Fills the hold posture vacuum (2026-07-13 viewer finding): with the Cartesian imitation
    terms swing-gated and no joint-space term anywhere, NOTHING constrains posture during
    holds — the torque regularizer then actively prefers the hanging/twisted arm (lowest
    shoulder torque), which is the "twisted paddle at the hip" idle the user saw. Target =
    the selected clip's FIRST frame (the ready pose the swing starts from), so hold pose ==
    swing entry pose and stand-starts learn the raise-to-ready motion.

    UPPER BODY ONLY (arms + waist via asset_cfg.joint_ids; npz joint order == articulation
    order by csv_to_npz construction). Legs are deliberately EXCLUDED: the clip ready pose is
    a deep crouch in the legs (receipt leg_ready_default_rms 0.53 rad) and a leg target would
    fight the implicit stand attractors (reset pose + hold_ready stillness) — the original
    splayed-feet crouch-stand bug that swing-gating fixed. Zero outside holds.
    """
    cmd = env.command_manager.get_term(command_name)
    ids = list(asset_cfg.joint_ids)
    ready = cmd.motion.joint_pos[cmd.motion.seg_start[cmd.clip_id]][:, ids]
    q = cmd.robot.data.joint_pos[:, ids]
    err = torch.mean(torch.square(q - ready), dim=-1)
    r = torch.exp(-err / std**2)
    return torch.where(cmd.in_hold, r, torch.zeros_like(r))


def hold_upper_joint_deviation(
    env,
    command_name: str,
    action_name: str,
    joint_names,
):
    """Hold-gated arm/wrist deviation from the selected clip's ready pose.

    The term is joint-wise ownership aware.  When HUGWBC training intervention owns a
    shoulder/elbow channel, only that channel is masked; both wrists remain supervised.  This
    prevents the old all-or-nothing intervention mask from creating an unconstrained wrist posture
    during long wait/recovery holds.  The returned value is a non-negative mean-square debt and
    therefore requires a negative YAML weight.
    """
    cmd = env.command_manager.get_term(command_name)
    action_term = env.action_manager.get_term(action_name)
    resolved_action_names = list(getattr(action_term, "_joint_names", ()))
    requested = tuple(str(name) for name in joint_names)
    missing = [name for name in requested if name not in resolved_action_names]
    if not requested or missing:
        raise RuntimeError(
            "hold_upper_joint_deviation requires non-empty exact action joint names; "
            f"missing={missing}"
        )

    action_cols = torch.tensor(
        [resolved_action_names.index(name) for name in requested],
        dtype=torch.long,
        device=cmd.device,
    )
    action_joint_ids = getattr(action_term, "_action_joint_ids", None)
    if not torch.is_tensor(action_joint_ids):
        raise RuntimeError(
            f"Action term {action_name!r} does not expose articulation joint ids"
        )
    joint_ids = action_joint_ids.index_select(0, action_cols)
    ready_frame = cmd.motion.seg_start[cmd.clip_id]
    ready = cmd.motion.joint_pos[ready_frame].index_select(-1, joint_ids)
    q = cmd.robot.data.joint_pos.index_select(-1, joint_ids)
    squared_error = torch.square(q - ready)

    supervised = torch.ones_like(squared_error)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(action_term, "_upper_intervention_effective", None)
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        intervened_joint = (
            action_cols.unsqueeze(-1) == intervention_cols.unsqueeze(0)
        ).any(dim=-1)
        supervised = torch.where(
            intervention_effective.unsqueeze(-1) & intervened_joint.unsqueeze(0),
            torch.zeros_like(supervised),
            supervised,
        )

    debt = (squared_error * supervised).sum(dim=-1) / supervised.sum(dim=-1).clamp_min(1.0)
    cmd.metrics["hold_upper_joint_error_rms"] = torch.sqrt(debt)
    return torch.where(cmd.in_hold, debt, torch.zeros_like(debt))


def left_wrist_reference_debt(
    env, command_name: str, motion_command_name: str, asset_cfg,
    margin: float = 0.10, std: float = 0.25, max_blend: float = 0.50,
):
    """Dense V10 debt for the three non-task left-wrist joints.

    The old ``ee_wrist_pos`` termination measured only the z position of the terminal wrist body;
    it could not observe the pitch/yaw drift seen in Gate3.  This term directly tracks the selected
    clip's frame-0 wrist pose during a hold and its current wrist pose during a released swing.  A
    deadband preserves small natural motion, a Huber tail remains informative at the observed
    one-radian error, and the max component prevents one runaway joint from disappearing in a mean.
    The right wrist is deliberately absent because the racket task owns it.
    """
    if margin < 0.0 or std <= 0.0:
        raise ValueError(f"left-wrist margin/std must be non-negative/positive, got {margin}/{std}")
    if not 0.0 <= max_blend <= 1.0:
        raise ValueError(f"left-wrist max_blend must be in [0,1], got {max_blend}")

    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 3:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(f"RallyV10 left-wrist debt requires exactly three joints, got {count}")

    q = asset.data.joint_pos[:, joint_ids]
    swing_ref = motion.joint_pos[:, joint_ids]
    ready_frame = motion.motion.seg_start[motion.clip_id]
    ready_ref = motion.motion.joint_pos[ready_frame][:, joint_ids]
    ref = torch.where(motion.in_hold.unsqueeze(-1), ready_ref, swing_ref)
    error = torch.abs(q - ref)
    debt_per_joint = _deadband_huber(error, margin, std)
    debt = (
        (1.0 - float(max_blend)) * torch.mean(debt_per_joint, dim=-1)
        + float(max_blend) * torch.max(debt_per_joint, dim=-1).values
    )

    cmd.metrics["left_wrist_reference_error_mean"] = torch.mean(error, dim=-1)
    cmd.metrics["left_wrist_reference_error_max"] = torch.max(error, dim=-1).values
    cmd.metrics["left_wrist_reference_excess_frac"] = (error > margin).float().mean(dim=-1)
    return debt


def rally_idle_left_wrist_debt(
    env,
    command_name: str,
    motion_command_name: str,
    asset_cfg,
    position_margin: float = 0.02,
    position_std: float = 0.12,
    velocity_margin: float = 0.05,
    velocity_std: float = 0.25,
    velocity_blend: float = 0.25,
    max_blend: float = 0.75,
):
    """Hold-only left-wrist discipline that permits approach but rejects overshoot.

    Gate3 measures the total idle range starting from the default zero wrist pose.  The v13 ready
    roll target is about -0.324 rad, so V11's 0.08-rad deadband explicitly allowed a range near
    0.404 rad against a 0.35-rad gate.  Outside the tighter position band this term charges only
    joint velocity that increases absolute reference error; velocity toward the ready pose stays
    free.  Once inside the band it charges absolute velocity so the wrist settles instead of
    coasting through the target.  Released swings remain owned by the inherited reference term.
    """
    if position_margin < 0.0 or position_std <= 0.0:
        raise ValueError(
            f"invalid idle-wrist position margin/std: {position_margin}/{position_std}"
        )
    if velocity_margin < 0.0 or velocity_std <= 0.0:
        raise ValueError(
            f"invalid idle-wrist velocity margin/std: {velocity_margin}/{velocity_std}"
        )
    if not 0.0 <= velocity_blend <= 1.0 or not 0.0 <= max_blend <= 1.0:
        raise ValueError(
            f"invalid idle-wrist velocity/max blend: {velocity_blend}/{max_blend}"
        )

    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 3:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(f"RallyV12 idle-wrist debt requires exactly three joints, got {count}")

    ready_frame = motion.motion.seg_start[motion.clip_id]
    ready_ref = motion.motion.joint_pos[ready_frame][:, joint_ids]
    q = asset.data.joint_pos[:, joint_ids]
    qd = asset.data.joint_vel[:, joint_ids]
    signed_error = q - ready_ref
    error = torch.abs(signed_error)
    position_per_joint = _deadband_huber(error, position_margin, position_std)
    outward_velocity = torch.sign(signed_error) * qd
    charged_velocity = torch.where(
        error > float(position_margin), outward_velocity, torch.abs(qd)
    )
    velocity_per_joint = _deadband_huber(
        charged_velocity, velocity_margin, velocity_std
    )

    def _mean_max(per_joint):
        return (
            (1.0 - float(max_blend)) * torch.mean(per_joint, dim=-1)
            + float(max_blend) * torch.max(per_joint, dim=-1).values
        )

    debt = (
        (1.0 - float(velocity_blend)) * _mean_max(position_per_joint)
        + float(velocity_blend) * _mean_max(velocity_per_joint)
    )
    gate = motion.in_hold.float()
    cmd.metrics["idle_left_wrist_reference_error_max"] = torch.max(error, dim=-1).values * gate
    cmd.metrics["idle_left_wrist_speed_max"] = torch.max(torch.abs(qd), dim=-1).values * gate
    cmd.metrics["idle_left_wrist_position_excess_frac"] = (
        (error > float(position_margin)).float().mean(dim=-1) * gate
    )
    return debt * gate


def right_elbow_extension_debt(
    env, command_name: str, asset_cfg, extension_start: float = 1.30,
    std: float = 0.15, t_pre: float = 0.25, t_post: float = 0.10,
    forehand_only: bool = False,
):
    """One-sided V10 debt before the right elbow reaches its straight-arm singularity.

    On the A3 chain, increasing ``right_elbow_joint`` extends the shoulder-to-wrist reach and
    approaches a straight arm near 1.57 rad. Gate3 model_10000 reached 1.52 rad while the v13
    reference stays below 1.21 rad. This term therefore charges only ``q > extension_start`` in
    a short contact window; flexion, the demonstrated swing, and the rest of the episode are free.
    It is a soft Huber debt rather than a termination or a full reference lock.
    """
    if std <= 0.0 or extension_start < 0.0:
        raise ValueError(
            "right-elbow extension_start/std must be non-negative/positive, "
            f"got {extension_start}/{std}"
        )
    if t_pre <= 0.0 or t_post < 0.0:
        raise ValueError(
            f"right-elbow t_pre/t_post must be positive/non-negative, got {t_pre}/{t_post}"
        )

    cmd = _cmd(env, command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 1:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(f"RallyV10 right-elbow debt requires exactly one joint, got {count}")

    q = asset.data.joint_pos[:, joint_ids[0]]
    debt = _deadband_huber(q, extension_start, std)
    gate = (cmd.time_to_strike < float(t_pre)) & (cmd.time_to_strike > -float(t_post))
    if forehand_only:
        gate &= cmd.swing_sign > 0.0

    _record_metric_snapshot(cmd.metrics, "right_elbow_position", q)
    cmd.metrics["right_elbow_extension_excess"] = torch.clamp(
        q - float(extension_start), min=0.0
    )
    return debt * gate.float()


def stance_width_band(env, lo: float, hi: float, std: float, asset_cfg=None):
    """Smooth-L1 debt on the horizontal ankle-to-ankle distance outside [lo, hi].

    Metric = ||Δp_xy|| between the two ankle-roll link origins: root/yaw-invariant and
    stagger-aware (the pelvis-frame y-component under-reads a staggered stance — the 0.086 m
    false alarm of 2026-07-13). Calibration: demo stance 0.281-0.288 across BOTH v13 clips,
    deploy default stand 0.259, mechanical zero 0.246 — all inside the band, so the demo and
    normal operation pay ZERO. The linear (smooth-L1) tail keeps transient step peaks cheap
    while sustained self-invented wide splits (22 s at 0.53 in G3) pay steady rent. Always-on;
    positive magnitude, use a NEGATIVE RewTerm weight.
    """
    asset = env.scene[asset_cfg.name]
    feet = asset.data.body_pos_w[:, asset_cfg.body_ids, :2]
    d = torch.norm(feet[:, 0] - feet[:, 1], dim=-1)
    excess = torch.relu(lo - d) + torch.relu(d - hi)
    x = excess / std
    return torch.where(x < 1.0, 0.5 * x * x, x - 0.5)


def _rally_ready_stance_gate(
    cmd: RacketTargetCommand,
    motion,
    station_reach: float,
    heading_gate: float,
    speed_gate: float = 0.0,
) -> torch.Tensor:
    """Gate ready-stance shaping until the commanded step and yaw recovery are complete."""
    if station_reach <= 0.0 or heading_gate <= 0.0 or speed_gate < 0.0:
        raise ValueError(
            "ready-stance station_reach/heading_gate must be positive and speed_gate "
            f"non-negative, got {station_reach}/{heading_gate}/{speed_gate}"
        )
    held_metric = motion.metrics.get("in_hold") if hasattr(motion, "metrics") else None
    in_hold = (held_metric > 0.5) if held_metric is not None else getattr(motion, "in_hold", None)
    if in_hold is None:
        raise RuntimeError(
            "RallyV11 ready-stance rewards require MotionCommand.in_hold; refusing a "
            "fail-open lower-body reward"
        )

    station_error = torch.linalg.norm(
        cmd.base_pos_w[:, :2] - cmd.base_target_pos_w, dim=-1
    )
    q = cmd.base_quat_w
    forward_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
    forward_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
    yaw_abs = torch.abs(torch.atan2(forward_y, forward_x))
    base_speed = torch.linalg.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    gate = in_hold & (station_error < float(station_reach)) & (
        yaw_abs < float(heading_gate)
    )
    if speed_gate > 0.0:
        gate &= base_speed <= float(speed_gate)
    cmd.metrics["ready_stance_gate"] = gate.float()
    cmd.metrics["ready_stance_station_error"] = station_error
    cmd.metrics["ready_stance_heading_abs"] = yaw_abs
    cmd.metrics["ready_stance_base_speed"] = base_speed
    return gate


def rally_ready_deadline_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    motion_command_name: str,
    x_margin: float = 0.10,
    y_margin: float = 0.10,
    position_std: float = 0.10,
    speed_margin: float = 0.20,
    speed_std: float = 0.20,
    speed_blend: float = 0.60,
    final_window_s: float = 0.12,
    target_step_class: int = 3,
    target_step_classes: tuple[int, ...] | list[int] | None = None,
    match_strict_ready: bool = False,
) -> torch.Tensor:
    """Soft runner-READY debt over the final fixed-clock hold samples.

    The reward opens only during the last ``final_window_s`` of the exogenous hold and only for
    the selected station sampler classes.  The legacy path asks for per-axis station error and
    planar speed only.  ``match_strict_ready=True`` instead uses the command's single-source
    runner-equivalent thresholds for position, speed, heading, yaw rate, tilt, and joint speed,
    and charges the worst outstanding component.  Keeping every final tick debt-free is the soft
    analogue of the strict READY dwell.  It never extends the clock, blocks a swing, terminates an
    episode, or removes strike samples.
    """
    if (
        x_margin < 0.0
        or y_margin < 0.0
        or position_std <= 0.0
        or speed_margin < 0.0
        or speed_std <= 0.0
        or not 0.0 <= speed_blend <= 1.0
        or final_window_s <= 0.0
    ):
        raise ValueError(
            "invalid V11 ready-deadline parameters: "
            f"x/y={x_margin}/{y_margin}, pos_std={position_std}, "
            f"speed={speed_margin}/{speed_std}, blend={speed_blend}, "
            f"window={final_window_s}"
        )
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    held_metric = motion.metrics.get("in_hold") if hasattr(motion, "metrics") else None
    if held_metric is None:
        raise RuntimeError("RallyV11 ready-deadline debt requires MotionCommand.metrics['in_hold']")
    held = held_metric > 0.5
    remaining_s = motion.hold_counter.float() * float(cmd._env.step_dt)
    final_window = held & (remaining_s <= float(final_window_s) + 1.0e-9)
    step_class = cmd.metrics.get("station_y_step_class")
    if step_class is None:
        raise RuntimeError("RallyV11 ready-deadline debt requires station_y_step_class telemetry")
    if target_step_classes is None:
        selected_classes = (int(target_step_class),)
    else:
        selected_classes = tuple(int(value) for value in target_step_classes)
        if not selected_classes:
            raise ValueError("target_step_classes must not be empty")
    target_question = torch.zeros_like(step_class, dtype=torch.bool)
    for selected_class in selected_classes:
        target_question |= torch.abs(step_class - float(selected_class)) < 0.5
    gate = final_window & target_question

    delta = torch.abs(cmd.base_target_pos_w - cmd.base_pos_w[:, :2])
    x_threshold = (
        float(cmd.cfg.ready_monitor_x_thresh)
        if match_strict_ready
        else float(x_margin)
    )
    y_threshold = (
        float(cmd.cfg.ready_monitor_y_thresh)
        if match_strict_ready
        else float(y_margin)
    )
    speed_threshold = (
        float(cmd.cfg.ready_monitor_speed_thresh)
        if match_strict_ready
        else float(speed_margin)
    )
    x_debt = _deadband_huber(delta[:, 0], x_threshold, float(position_std))
    y_debt = _deadband_huber(delta[:, 1], y_threshold, float(position_std))
    position_debt = torch.maximum(x_debt, y_debt)
    base_speed = torch.linalg.norm(cmd.robot.data.root_lin_vel_w[:, :2], dim=-1)
    speed_debt = _deadband_huber(
        base_speed, speed_threshold, float(speed_std)
    )
    if match_strict_ready:
        q = cmd.base_quat_w
        forward_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
        forward_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
        heading = torch.abs(torch.atan2(forward_y, forward_x))
        yaw_rate = torch.abs(cmd.robot.data.root_ang_vel_b[:, 2])
        projected_gravity = getattr(
            cmd.robot.data, "projected_gravity_b", None
        )
        if projected_gravity is None:
            raise RuntimeError(
                "strict ready-deadline debt requires projected_gravity_b"
            )
        tilt = torch.linalg.norm(projected_gravity[:, :2], dim=-1)
        joint_speed = torch.sqrt(
            torch.mean(cmd.robot.data.joint_vel.square(), dim=-1)
        )
        heading_threshold = float(cmd.cfg.ready_monitor_heading_thresh_rad)
        yaw_rate_threshold = float(cmd.cfg.ready_monitor_yaw_rate_thresh)
        tilt_threshold = float(cmd.cfg.ready_monitor_tilt_thresh)
        joint_speed_threshold = float(
            cmd.cfg.ready_monitor_joint_speed_thresh
        )
        heading_debt = _deadband_huber(
            heading, heading_threshold, max(heading_threshold, 1.0e-6)
        )
        yaw_rate_debt = _deadband_huber(
            yaw_rate, yaw_rate_threshold, max(yaw_rate_threshold, 1.0e-6)
        )
        tilt_debt = _deadband_huber(
            tilt, tilt_threshold, max(tilt_threshold, 1.0e-6)
        )
        joint_speed_debt = _deadband_huber(
            joint_speed,
            joint_speed_threshold,
            max(joint_speed_threshold, 1.0e-6),
        )
        component_debts = torch.stack(
            (
                x_debt,
                y_debt,
                speed_debt,
                heading_debt,
                yaw_rate_debt,
                tilt_debt,
                joint_speed_debt,
            ),
            dim=-1,
        )
        debt = component_debts.amax(dim=-1)
        strict_pass = (
            (delta[:, 0] <= x_threshold)
            & (delta[:, 1] <= y_threshold)
            & (base_speed <= speed_threshold)
            & (heading <= heading_threshold)
            & (yaw_rate <= yaw_rate_threshold)
            & (tilt <= tilt_threshold)
            & (joint_speed <= joint_speed_threshold)
        )
        cmd.metrics["ready_deadline_heading_debt"] = (
            heading_debt * gate.float()
        )
        cmd.metrics["ready_deadline_yaw_rate_debt"] = (
            yaw_rate_debt * gate.float()
        )
        cmd.metrics["ready_deadline_tilt_debt"] = tilt_debt * gate.float()
        cmd.metrics["ready_deadline_joint_speed_debt"] = (
            joint_speed_debt * gate.float()
        )
    else:
        debt = (
            (1.0 - float(speed_blend)) * position_debt
            + float(speed_blend) * speed_debt
        )
        strict_pass = (
            (delta[:, 0] <= x_threshold)
            & (delta[:, 1] <= y_threshold)
            & (base_speed <= speed_threshold)
        )
    cmd.metrics["v11_ready_deadline_gate"] = gate.float()
    cmd.metrics["v11_ready_deadline_remaining_s"] = remaining_s * gate.float()
    cmd.metrics["v11_ready_deadline_position_debt"] = position_debt * gate.float()
    cmd.metrics["v11_ready_deadline_speed_debt"] = speed_debt * gate.float()
    cmd.metrics["ready_deadline_strict_pass"] = (
        strict_pass & gate
    ).float()
    cmd.metrics["ready_deadline_strict_debt"] = debt * gate.float()
    return debt * gate.float()


def rally_ready_stance_width_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    motion_command_name: str,
    asset_cfg,
    lo: float = 0.25,
    hi: float = 0.35,
    std: float = 0.05,
    station_reach: float = 0.10,
    heading_gate: float = 0.15,
    speed_gate: float = 0.20,
) -> torch.Tensor:
    """RallyV11 stance-width debt after arrival, never during the commanded step.

    V10 left the existing always-on stance rail at zero because it would also charge a necessary
    lateral step.  This version opens only in the exogenous hold after the base is at its station
    and nearly square.  The calibrated zero-debt band contains both v13 ready clips and the A3
    default stand, so it removes self-invented narrow/wide stances without prescribing clip legs.
    """
    if not 0.0 < lo < hi or std <= 0.0:
        raise ValueError(f"invalid ready stance-width band/std: {lo}/{hi}/{std}")
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    body_ids = asset_cfg.body_ids
    if isinstance(body_ids, slice) or len(body_ids) != 2:
        count = "slice" if isinstance(body_ids, slice) else len(body_ids)
        raise RuntimeError(f"RallyV11 stance-width debt requires exactly two feet, got {count}")

    feet = asset.data.body_pos_w[:, body_ids, :2]
    width = torch.linalg.norm(feet[:, 0] - feet[:, 1], dim=-1)
    excess = torch.relu(float(lo) - width) + torch.relu(width - float(hi))
    scaled = excess / float(std)
    debt = torch.where(scaled < 1.0, 0.5 * scaled * scaled, scaled - 0.5)
    gate = _rally_ready_stance_gate(
        cmd, motion, station_reach, heading_gate, speed_gate
    )

    cmd.metrics["ready_stance_width"] = width * gate.float()
    cmd.metrics["ready_stance_width_excess"] = excess * gate.float()
    return debt * gate.float()


def rally_ready_foot_alignment_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    motion_command_name: str,
    asset_cfg,
    margin: float = 0.12,
    std: float = 0.25,
    max_blend: float = 0.50,
    station_reach: float = 0.10,
    heading_gate: float = 0.15,
    speed_gate: float = 0.20,
) -> torch.Tensor:
    """Selective ready-foot alignment for hip yaw/roll and ankle roll only.

    This is deliberately not lower-body motion imitation: knees, hip pitch and ankle pitch stay
    free, and released swing/footwork states are untouched.  Once arrived and square, the six
    orientation joints receive a dead-banded debt relative to the safe A3 default stance.  The
    worst-joint channel prevents one asymmetric foot from hiding in the bilateral mean.
    """
    if margin < 0.0 or std <= 0.0 or not 0.0 <= max_blend <= 1.0:
        raise ValueError(
            "invalid ready-foot margin/std/max_blend: "
            f"{margin}/{std}/{max_blend}"
        )
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 6:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(
            f"RallyV11 ready-foot alignment requires exactly six joints, got {count}"
        )

    error = torch.abs(
        asset.data.joint_pos[:, joint_ids] - asset.data.default_joint_pos[:, joint_ids]
    )
    per_joint = _deadband_huber(error, float(margin), float(std))
    debt = (
        (1.0 - float(max_blend)) * torch.mean(per_joint, dim=-1)
        + float(max_blend) * torch.max(per_joint, dim=-1).values
    )
    gate = _rally_ready_stance_gate(
        cmd, motion, station_reach, heading_gate, speed_gate
    )
    gated_error = error * gate.unsqueeze(-1).float()
    cmd.metrics["ready_foot_alignment_error_mean"] = torch.mean(gated_error, dim=-1)
    cmd.metrics["ready_foot_alignment_error_max"] = torch.max(
        gated_error, dim=-1
    ).values
    cmd.metrics["ready_foot_alignment_excess_frac"] = (
        (error > float(margin)).float().mean(dim=-1) * gate.float()
    )
    return debt * gate.float()


def rally_ready_leg_settle_debt(
    env: ManagerBasedRLEnv,
    command_name: str,
    motion_command_name: str,
    asset_cfg,
    margin: float = 0.30,
    std: float = 0.80,
    station_reach: float = 0.10,
    heading_gate: float = 0.15,
) -> torch.Tensor:
    """Dead-banded leg-speed debt in the arrived, square ready stance.

    The term cannot suppress approach steps or the swing because it is hold/arrival/heading gated.
    It complements V10's planted-feet income with direct joint-speed supervision and keeps no pose
    target, so the policy may choose its own balanced knee/hip-pitch solution inside the stance rails.
    """
    if margin < 0.0 or std <= 0.0:
        raise ValueError(f"invalid ready-leg margin/std: {margin}/{std}")
    cmd = _cmd(env, command_name)
    motion = env.command_manager.get_term(motion_command_name)
    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, slice) or len(joint_ids) != 12:
        count = "slice" if isinstance(joint_ids, slice) else len(joint_ids)
        raise RuntimeError(
            f"RallyV11 ready-leg settle requires exactly twelve joints, got {count}"
        )

    speed_rms = torch.sqrt(
        torch.mean(torch.square(asset.data.joint_vel[:, joint_ids]), dim=-1)
    )
    debt = _deadband_huber(speed_rms, float(margin), float(std))
    # Unlike stance geometry, leg-speed debt is the brake: keep it active as soon as the
    # robot is inside the station/yaw gate, even if base speed has not yet fallen below READY.
    gate = _rally_ready_stance_gate(cmd, motion, station_reach, heading_gate, 0.0)
    cmd.metrics["ready_leg_speed_rms"] = speed_rms * gate.float()
    return debt * gate.float()
