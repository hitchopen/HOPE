"""Training-only deterministic q_des temporal smoothing; action ABI unchanged.

Unlike a reward on sampled actions, these losses never differentiate the policy
standard deviation. Temporal pairs are from the same environment and episode.
Only conditional HOLD samples enter the normalization; rare HOLD samples are
not diluted by the rest of a rally minibatch.
The former q_des-to-actual-q objective is removed; its legacy return slot is
identically zero. A PD offset is a detached diagnostic, not a tracking loss.
"""
from __future__ import annotations

import math
import torch


def huber_debt(error: torch.Tensor, margin: float, scale: float) -> torch.Tensor:
    if not math.isfinite(margin) or margin < 0 or not math.isfinite(scale) or scale <= 0:
        raise ValueError("invalid refinement margin/scale")
    x = (error.abs() - margin).clamp_min(0) / scale
    return torch.where(x < 1, .5 * x.square(), x - .5)


def bounded_debt(error: torch.Tensor, margin: float, scale: float) -> torch.Tensor:
    huber = huber_debt(error, margin, scale)
    return huber / (1 + huber)


def conditional_mean(value: torch.Tensor, gate: torch.Tensor) -> torch.Tensor:
    """Keep the curriculum ramp outside the count of eligible samples."""
    return (value * gate).sum() / (gate > 0).sum().clamp_min(1)


def refinement_losses(current, previous, actual_waist, gate, pair_gate,
                      local_columns, waist_column, *, step_margin=.002,
                      step_scale=.01, tracking_margin=.15, tracking_scale=.10,
                      worst_joint_blend=0.0, bounded_smooth=True):
    if not 0 <= worst_joint_blend <= 1:
        raise ValueError("invalid worst-joint smoothness blend")
    delta = (current - previous)[:, local_columns]
    # A bounded loss has a vanishing tail precisely on the largest command
    # changes. v9 keeps a linear Huber tail; PPO caps the added gradient norm.
    debt = (bounded_debt if bounded_smooth else huber_debt)(delta, step_margin, step_scale)
    smooth = conditional_mean((1-worst_joint_blend)*debt.mean(-1)
                              + worst_joint_blend*debt.amax(-1), pair_gate)
    # A static PD offset can be the torque needed to support gravity. Never
    # optimize it toward zero; keep it detached and explicitly diagnostic.
    error = (current[:, waist_column] - actual_waist).detach()
    tracking = current.sum() * 0.0  # legacy return slot; identically zero gradient
    # Conditional diagnostics deliberately retain physical units, independent of the ramp.
    return smooth, tracking, {
        "qdes_step_rad": conditional_mean(delta.abs().mean(-1), (pair_gate > 0).float()),
        "waist_qdes_offset_abs_rad": conditional_mean(error.abs(), (gate > 0).float()),
        "waist_qdes_offset_signed_rad": conditional_mean(error, (gate > 0).float()),
        "active_fraction": (gate > 0).float().mean(),
        "pair_fraction": (pair_gate > 0).float().mean(),
    }


class SettledPolicyRefinement:
    """Rollout-aligned pre-action physical context, independent of actor observations."""

    def __init__(self, env, storage, *, start_steps, ramp_steps, include_upper=False,
                 step_margin=.002, step_scale=.01, worst_joint_blend=0.0, ready_only=False,
                 bounded_smooth=True):
        if start_steps < 0 or ramp_steps <= 0:
            raise ValueError("invalid refinement curriculum")
        if step_margin < 0 or step_scale <= 0 or not math.isfinite(step_margin + step_scale):
            raise ValueError("invalid deterministic smoothness margin/scale")
        self.step_margin, self.step_scale = step_margin, step_scale
        if not 0 <= worst_joint_blend <= 1:
            raise ValueError("invalid worst-joint smoothness blend")
        self.worst_joint_blend, self.ready_only = worst_joint_blend, ready_only
        self.bounded_smooth = bounded_smooth
        self.env = env
        self.start_steps, self.ramp_steps = int(start_steps), int(ramp_steps)
        self.action = env.action_manager.get_term("joint_pos")
        if getattr(self.action, "_action_contract", None) != "v11_affine_safe_qdes_v1":
            raise ValueError("settled refinement requires the affine V11 decoder")
        names = self.action._joint_names
        wanted = [f"{side}_{j}_joint" for side in ("left", "right")
                  for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
        wanted += [f"waist_{j}_joint" for j in ("yaw", "roll", "pitch")]
        if include_upper:
            wanted += [f"{side}_{j}_joint" for side in ("left", "right")
                       for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                 "wrist_roll", "wrist_pitch", "wrist_yaw")]
        self.local_columns = [names.index(j) for j in wanted]
        self.waist_column = names.index("waist_pitch_joint")
        if ready_only:
            from .task_pose_reference import reference_joints
            self.pose_reference = reference_joints(wanted, self.action._asset.data.joint_pos)
        sensor = env.scene.sensors["contact_forces"]
        self.sensor = sensor
        self.foot_ids, _ = sensor.find_bodies(["left_ankle_roll_Link", "right_ankle_roll_Link"], preserve_order=True)
        if len(self.foot_ids) != 2:
            raise ValueError("refinement requires both A3 foot contact bodies")
        shape = (storage.num_transitions_per_env, storage.num_envs)
        self.gate = torch.zeros(shape, device=storage.device)
        self.actual_waist = torch.zeros(shape, device=storage.device)
        self.curriculum = torch.zeros(shape[0], device=storage.device)
        self.rollout_has_curriculum = False
        ids = torch.arange(storage.num_envs, device=storage.device)
        self.offset = self.action._decoder_parameter_rows(self.action._offset, ids, "offset").detach().clone()
        self.scale = self.action._decoder_parameter_rows(self.action._scale, ids, "scale").detach().clone()
        self.lower = self.action._safe_lo.detach().clone()
        self.upper = self.action._safe_hi.detach().clone()

    @torch.no_grad()
    def capture(self, step):
        motion = self.env.command_manager.get_term("motion")
        loaded = (self.sensor.data.net_forces_w[:, self.foot_ids, 2] > 20).all(-1)
        ramp = min(1., max(0., (self.env.common_step_counter-self.start_steps)/self.ramp_steps))
        if step == 0:
            self.rollout_has_curriculum = False
        self.rollout_has_curriculum |= ramp > 0.
        self.curriculum[step] = ramp
        actual = self.action._select_action_joints(self.action._asset.data.joint_pos)
        gate = (motion.in_hold & loaded).float() * ramp
        if self.ready_only:
            from .full_body_hold import quiet_gate
            cmd = self.env.command_manager.get_term("racket_target")
            distance = (cmd.base_target_pos_w-cmd.base_pos_w[:, :2]).norm(dim=-1)
            pose_error = (actual[:, self.local_columns]-self.pose_reference).abs().amax(-1)
            speed = self.action._asset.data.root_lin_vel_w[:, :2].norm(dim=-1)
            gate = gate * quiet_gate(distance, pose_error, speed)
        self.gate[step].copy_(gate)
        self.actual_waist[step].copy_(actual[:, self.waist_column])

    def decode(self, mean, env_ids):
        # No delay is applied here: regularize the deterministic commanded target,
        # not the previous rollout's randomly sampled FIFO output.
        return torch.maximum(torch.minimum(self.offset[env_ids] + self.scale[env_ids] * mean,
                                           self.upper[env_ids]), self.lower[env_ids])

    def losses(self, policy, observations, current_mean, indices, dones):
        n = self.gate.shape[1]
        previous_indices = (indices - n).clamp_min(0)
        gate = self.gate.flatten()[indices]
        prior_gate = self.gate.flatten()[previous_indices]
        pair_gate = torch.minimum(gate, prior_gate)
        pair_gate = pair_gate * (indices >= n) * ~dones.flatten()[previous_indices].bool()
        previous_obs = policy.get_actor_obs(observations[previous_indices])
        previous_mean = policy.actor(policy.actor_obs_normalizer(previous_obs))
        current = self.decode(current_mean, indices % n)
        previous = self.decode(previous_mean, indices % n)
        smooth, tracking, diagnostics = refinement_losses(
            current, previous, self.actual_waist.flatten()[indices],
            gate, pair_gate, self.local_columns, self.waist_column,
            step_margin=self.step_margin, step_scale=self.step_scale,
            worst_joint_blend=self.worst_joint_blend,
            bounded_smooth=self.bounded_smooth,
        )
        diagnostics["curriculum_scale"] = self.curriculum[indices // n].mean()
        return smooth, tracking, diagnostics
