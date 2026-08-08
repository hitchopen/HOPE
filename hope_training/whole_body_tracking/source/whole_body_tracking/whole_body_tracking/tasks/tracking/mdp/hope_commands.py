"""HOPE-specific command term: racket / base target tracking on top of BeyondMimic.

This adds the HITTER (arXiv:2508.21043) racket-target objective to the BeyondMimic motion
tracker. The base ``MotionCommand`` (in ``commands.py``) drives the imitation reward and owns the
per-env motion clock (``time_steps``). ``RacketTargetCommand`` rides on top of it:

* it samples a *desired* racket state (position, velocity, face normal) and a *desired* base XY
  position each swing — exactly the quantities the model-based planner emits at deploy time via
  ``hope_msgs/RacketCommand`` (position, velocity, normal). No ball is needed in simulation;
  training samples targets, the planner supplies them at runtime.
* it computes the *actual* racket state in simulation by forward kinematics through the fixed
  racket mount ``T_mount`` (wrist -> paddle center), so the reward can compare actual vs desired.
* it derives the strike timing from the motion clip phase, exposing ``time_to_strike`` plus the
  ``pre_strike`` / ``strike_window`` masks that gate the goal rewards.

Per the HOPE racket-tracking prohibition, there is NO measured racket feedback at deploy time:
``r_racket`` is a simulation-only signal; on hardware the policy runs open-loop on racket pose.

HITTER alignment notes (see the project HITTER verification):
* racket *position* is observed relative to the base; racket *velocity* is observed in world.
* HOPE currently also observes desired racket *normal* in the actor so the policy can respond to
  normal targets; actual racket normal remains a privileged simulation-only critic/reward signal.
* swing type is a *sampled* variable used here to (a) flag forehand/backhand and (b) select the
  reference clip; it is not required in the actor observation when separate forehand/backhand
  policies are trained (the HOPE default, reimplement.md step 17).
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    euler_xyz_from_quat,
    matrix_from_quat,
    quat_apply,
    quat_from_euler_xyz,
    quat_mul,
    quat_rotate_inverse,
    sample_uniform,
    yaw_quat,
)

from whole_body_tracking.tasks.tracking.mdp.commands import MotionCommand
from whole_body_tracking.tasks.tracking.mdp.one_step_contract import (
    advance_one_step_bout,
)
from whole_body_tracking.tasks.tracking.mdp.recovery_curriculum import (
    RecoveryCurriculumConfig,
    RecoveryCurriculumMetrics,
    RecoveryCurriculumState,
    advance_recovery_curriculum,
    release_eligible_completion_rate,
)
from whole_body_tracking.tasks.tracking.mdp.strike_curriculum import (
    VelocityStageConfig,
    VelocityStageMetrics,
    VelocityStageState,
    advance_staged_velocity_curriculum,
    advance_velocity_curriculum,
    constrained_tracking_sigma,
    interpolated_velocity_weight,
    staged_target_robustness,
    staged_velocity_sampling,
    velocity_curriculum_gate_status,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# v3 rally recovery (2026-07-08): a hold whose base |yaw| at its rising edge exceeds this (rad)
# counts as "started yawed" for the spawn-conditioned heading_recovery_* metrics. 0.30 rad ~= 17°,
# just inside the deploy engage gate (0.35 rad) — the band where re-squaring actually matters.
_RECOV_SPAWN_YAW_THRESH = 0.30


class RacketTargetCommand(CommandTerm):
    """Samples desired racket/base targets and computes the actual racket state by FK."""

    cfg: RacketTargetCommandCfg

    def __init__(self, cfg: RacketTargetCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        ready_step_lo, ready_step_hi = (float(v) for v in cfg.ready_monitor_step_range)
        if not (0.0 <= ready_step_lo <= ready_step_hi):
            raise ValueError(
                "ready_monitor_step_range must satisfy 0 <= lo <= hi; "
                f"got {cfg.ready_monitor_step_range}"
            )
        for name in (
            "ready_monitor_x_thresh",
            "ready_monitor_y_thresh",
            "ready_monitor_speed_thresh",
            "ready_monitor_yaw_rate_thresh",
            "ready_monitor_tilt_thresh",
            "ready_monitor_joint_speed_thresh",
            "ready_monitor_dwell_s",
            "ready_monitor_heading_thresh_rad",
            "ready_monitor_foot_slip_thresh",
            "ready_acquisition_bootstrap_heading_thresh_rad",
            "ready_acquisition_bootstrap_yaw_rate_thresh",
            "ready_acquisition_bootstrap_tilt_thresh",
            "ready_acquisition_bootstrap_joint_speed_thresh",
            "ready_acquisition_bootstrap_foot_slip_thresh",
            "ready_release_arm_tts_s",
            "ready_release_timeout_s",
            "metrics_pre_settle_t_max",
            "metrics_post_settle_t_lo",
            "metrics_post_settle_t_hi",
            "metrics_clearance_t_pre",
            "metrics_clearance_t_post",
            "metrics_ready_heading_t_lo",
            "metrics_ready_heading_t_hi",
            "metrics_post_heading_t_lo",
            "metrics_post_heading_t_hi",
        ):
            if float(getattr(cfg, name)) < 0.0:
                raise ValueError(f"{name} must be non-negative; got {getattr(cfg, name)}")
        if float(cfg.position_guidance_window_s) < 0.0:
            raise ValueError(
                "position_guidance_window_s must be non-negative; got "
                f"{cfg.position_guidance_window_s}"
            )
        if float(cfg.position_guidance_temporal_scale) < 0.0:
            raise ValueError(
                "position_guidance_temporal_scale must be non-negative; got "
                f"{cfg.position_guidance_temporal_scale}"
            )
        for lo_name, hi_name in (
            ("metrics_post_settle_t_lo", "metrics_post_settle_t_hi"),
            ("metrics_ready_heading_t_lo", "metrics_ready_heading_t_hi"),
            ("metrics_post_heading_t_lo", "metrics_post_heading_t_hi"),
        ):
            if float(getattr(cfg, lo_name)) >= float(getattr(cfg, hi_name)):
                raise ValueError(
                    f"{lo_name} must be smaller than {hi_name}; got "
                    f"{getattr(cfg, lo_name)} >= {getattr(cfg, hi_name)}"
                )
        if int(cfg.ready_monitor_dwell_ticks) < 0:
            raise ValueError("ready_monitor_dwell_ticks must be non-negative")
        if int(cfg.ready_acquisition_bootstrap_dwell_ticks) < 1:
            raise ValueError(
                "ready_acquisition_bootstrap_dwell_ticks must be >= 1"
            )
        if bool(cfg.ready_release_enabled):
            if int(cfg.ready_monitor_dwell_ticks) < 1:
                raise ValueError(
                    "ready_release_enabled requires ready_monitor_dwell_ticks >= 1"
                )
            if float(cfg.ready_release_arm_tts_s) <= 0.0:
                raise ValueError(
                    "ready_release_arm_tts_s must be strictly positive"
                )
            if float(cfg.ready_release_timeout_s) <= 0.0:
                raise ValueError(
                    "ready_release_timeout_s must be strictly positive"
                )
        if bool(cfg.ready_acquisition_profile_enabled):
            relaxed_to_strict = (
                (
                    "heading",
                    cfg.ready_acquisition_bootstrap_heading_thresh_rad,
                    cfg.ready_monitor_heading_thresh_rad,
                ),
                (
                    "yaw_rate",
                    cfg.ready_acquisition_bootstrap_yaw_rate_thresh,
                    cfg.ready_monitor_yaw_rate_thresh,
                ),
                (
                    "tilt",
                    cfg.ready_acquisition_bootstrap_tilt_thresh,
                    cfg.ready_monitor_tilt_thresh,
                ),
                (
                    "joint_speed",
                    cfg.ready_acquisition_bootstrap_joint_speed_thresh,
                    cfg.ready_monitor_joint_speed_thresh,
                ),
                (
                    "foot_slip",
                    cfg.ready_acquisition_bootstrap_foot_slip_thresh,
                    cfg.ready_monitor_foot_slip_thresh,
                ),
            )
            for label, relaxed, strict in relaxed_to_strict:
                if float(relaxed) < float(strict):
                    raise ValueError(
                        f"READY acquisition bootstrap {label} threshold "
                        f"{relaxed} must be >= final threshold {strict}"
                    )
            if int(cfg.ready_acquisition_bootstrap_dwell_ticks) > int(
                cfg.ready_monitor_dwell_ticks
            ):
                raise ValueError(
                    "READY acquisition bootstrap dwell must not exceed the final dwell"
                )
        if not 0.0 <= float(cfg.venue_tuple_final_mix_prob) <= 1.0:
            raise ValueError("venue_tuple_final_mix_prob must lie in [0, 1]")
        self._venue_tuple_mix_mode = str(
            getattr(cfg, "venue_tuple_mix_mode", "recovery_scaled_online_v1")
        )
        if self._venue_tuple_mix_mode not in {
            "recovery_scaled_online_v1",
            "fixed_balanced_bank_v1",
        }:
            raise ValueError(
                "venue_tuple_mix_mode must be recovery_scaled_online_v1 or "
                f"fixed_balanced_bank_v1; got {self._venue_tuple_mix_mode!r}"
            )
        if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
            if not bool(cfg.venue_tuple_enabled):
                raise ValueError(
                    "fixed_balanced_bank_v1 requires venue_tuple_enabled=True"
                )
            if not str(getattr(cfg, "venue_tuple_bank_path", "")).strip():
                raise ValueError(
                    "fixed_balanced_bank_v1 requires venue_tuple_bank_path"
                )
            bank_sha = str(
                getattr(cfg, "venue_tuple_bank_sha256", "")
            ).strip().lower()
            if len(bank_sha) != 64 or any(
                character not in "0123456789abcdef" for character in bank_sha
            ):
                raise ValueError(
                    "fixed_balanced_bank_v1 requires a 64-character lowercase "
                    "venue_tuple_bank_sha256"
                )
            for label in (
                "venue_tuple_bank_receipt_path",
                "venue_tuple_bank_receipt_sha256",
            ):
                if not str(getattr(cfg, label, "")).strip():
                    raise ValueError(
                        f"fixed_balanced_bank_v1 requires {label}"
                    )
        if float(cfg.venue_tuple_speed_limit_mps) <= 0.0:
            raise ValueError("venue_tuple_speed_limit_mps must be positive")
        if int(cfg.venue_tuple_max_resample_attempts) < 1:
            raise ValueError(
                "venue_tuple_max_resample_attempts must be at least one"
            )
        for name in (
            "vel_weight_bootstrap_normal_pass",
            "vel_weight_bootstrap_position_pass",
            "vel_weight_bootstrap_post_fall_max",
            "vel_weight_bootstrap_ready_min",
        ):
            value = float(getattr(cfg, name))
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]; got {value}")
        for name in (
            "vel_weight_bootstrap_dwell_steps",
            "vel_weight_ramp_up_steps",
            "vel_weight_ramp_down_steps",
        ):
            value = int(getattr(cfg, name))
            if value < 1:
                raise ValueError(f"{name} must be >= 1; got {value}")
        self._velocity_curriculum_mode = str(
            getattr(cfg, "velocity_curriculum_mode", "legacy_reversible_v1")
        )
        if self._velocity_curriculum_mode not in {
            "legacy_reversible_v1",
            "staged_hysteresis_v2",
        }:
            raise ValueError(
                "velocity_curriculum_mode must be legacy_reversible_v1 or "
                "staged_hysteresis_v2, got "
                f"{self._velocity_curriculum_mode!r}"
            )
        self._velocity_stage_config = VelocityStageConfig(
            stage0_weight=float(cfg.vel_stage0_weight),
            stage1_weight=float(cfg.vel_stage1_weight),
            stage2_weight=float(cfg.vel_stage2_weight),
            stage1_enter_position=float(
                cfg.vel_stage1_enter_position_pass
            ),
            stage1_enter_velocity=float(
                cfg.vel_stage1_enter_velocity_pass
            ),
            stage1_enter_normal=float(cfg.vel_stage1_enter_normal_pass),
            stage1_enter_post_fall=float(
                cfg.vel_stage1_enter_post_fall_max
            ),
            stage1_enter_ready=float(cfg.vel_stage1_enter_ready_min),
            stage1_exit_position=float(
                cfg.vel_stage1_exit_position_pass
            ),
            stage1_exit_velocity=float(
                cfg.vel_stage1_exit_velocity_pass
            ),
            stage1_exit_normal=float(cfg.vel_stage1_exit_normal_pass),
            stage1_exit_post_fall=float(
                cfg.vel_stage1_exit_post_fall_max
            ),
            stage1_exit_ready=float(cfg.vel_stage1_exit_ready_min),
            stage2_enter_position=float(
                cfg.vel_stage2_enter_position_pass
            ),
            stage2_enter_velocity=float(
                cfg.vel_stage2_enter_velocity_pass
            ),
            stage2_enter_normal=float(cfg.vel_stage2_enter_normal_pass),
            stage2_enter_post_fall=float(
                cfg.vel_stage2_enter_post_fall_max
            ),
            stage2_enter_ready=float(cfg.vel_stage2_enter_ready_min),
            stage2_exit_position=float(
                cfg.vel_stage2_exit_position_pass
            ),
            stage2_exit_velocity=float(
                cfg.vel_stage2_exit_velocity_pass
            ),
            stage2_exit_normal=float(cfg.vel_stage2_exit_normal_pass),
            stage2_exit_post_fall=float(
                cfg.vel_stage2_exit_post_fall_max
            ),
            stage2_exit_ready=float(cfg.vel_stage2_exit_ready_min),
            stage_gate_requires_normal=bool(
                cfg.vel_stage_gate_requires_normal
            ),
            stage_gate_requires_ready=bool(
                cfg.vel_stage_gate_requires_ready
            ),
            stage1_gate_requires_normal=cfg.vel_stage1_gate_requires_normal,
            stage1_gate_requires_ready=cfg.vel_stage1_gate_requires_ready,
            stage2_gate_requires_normal=cfg.vel_stage2_gate_requires_normal,
            stage2_gate_requires_ready=cfg.vel_stage2_gate_requires_ready,
            min_exact_samples_per_side=float(
                cfg.vel_stage_min_exact_samples
            ),
            min_swing_start_samples=float(
                cfg.vel_stage_min_swing_starts
            ),
            stage1_enter_dwell_steps=int(
                cfg.vel_stage1_enter_dwell_steps
            ),
            stage1_exit_dwell_steps=int(
                cfg.vel_stage1_exit_dwell_steps
            ),
            stage2_enter_dwell_steps=int(
                cfg.vel_stage2_enter_dwell_steps
            ),
            stage2_exit_dwell_steps=int(
                cfg.vel_stage2_exit_dwell_steps
            ),
            stage0_to_1_ramp_steps=int(
                cfg.vel_stage0_to_1_ramp_steps
            ),
            stage1_to_0_ramp_steps=int(
                cfg.vel_stage1_to_0_ramp_steps
            ),
            stage1_to_2_ramp_steps=int(
                cfg.vel_stage1_to_2_ramp_steps
            ),
            stage2_to_1_ramp_steps=int(
                cfg.vel_stage2_to_1_ramp_steps
            ),
        )
        self._velocity_stage_config.validate()
        self._recovery_curriculum_enabled = bool(
            cfg.recovery_curriculum_enabled
        )
        self._recovery_curriculum_config = RecoveryCurriculumConfig(
            stage1_scale=float(cfg.recovery_stage1_scale),
            minimum_environment_steps=int(cfg.recovery_min_environment_steps),
            stage1_minimum_exact_samples_per_side=float(
                cfg.recovery_stage1_min_exact_samples_per_side
            ),
            stage1_minimum_swing_starts_per_side=float(
                cfg.recovery_stage1_min_swing_starts_per_side
            ),
            stage2_minimum_exact_samples_per_side=float(
                cfg.recovery_stage2_min_exact_samples_per_side
            ),
            stage2_minimum_swing_starts_per_side=float(
                cfg.recovery_stage2_min_swing_starts_per_side
            ),
            stage2_minimum_virtual_samples_per_side=float(
                cfg.recovery_stage2_min_virtual_samples_per_side
            ),
            stage2_minimum_actual_q_window_starts_per_side=float(
                cfg.recovery_stage2_min_actual_q_window_starts_per_side
            ),
            actual_q_window_steps=int(
                cfg.recovery_actual_q_window_steps
            ),
            stage1_enter_completion=float(
                cfg.recovery_stage1_enter_completion
            ),
            stage1_enter_position=float(cfg.recovery_stage1_enter_position),
            stage1_enter_velocity=float(cfg.recovery_stage1_enter_velocity),
            stage1_enter_normal=float(cfg.recovery_stage1_enter_normal),
            stage1_enter_composite=float(cfg.recovery_stage1_enter_composite),
            stage1_enter_ready=float(cfg.recovery_stage1_enter_ready),
            stage1_enter_post_fall=float(
                cfg.recovery_stage1_enter_post_fall_max
            ),
            stage1_enter_actual_q_fault=float(
                cfg.recovery_stage1_enter_actual_q_fault_max
            ),
            stage1_enter_dwell_steps=int(
                cfg.recovery_stage1_enter_dwell_steps
            ),
            stage1_exit_completion=float(
                cfg.recovery_stage1_exit_completion
            ),
            stage1_exit_position=float(cfg.recovery_stage1_exit_position),
            stage1_exit_velocity=float(cfg.recovery_stage1_exit_velocity),
            stage1_exit_normal=float(cfg.recovery_stage1_exit_normal),
            stage1_exit_composite=float(cfg.recovery_stage1_exit_composite),
            stage1_exit_ready=float(cfg.recovery_stage1_exit_ready),
            stage1_exit_post_fall=float(
                cfg.recovery_stage1_exit_post_fall_max
            ),
            stage1_exit_actual_q_fault=float(
                cfg.recovery_stage1_exit_actual_q_fault_max
            ),
            stage1_exit_dwell_steps=int(
                cfg.recovery_stage1_exit_dwell_steps
            ),
            stage1_ready_dwell_steps=int(
                cfg.recovery_stage1_ready_dwell_steps
            ),
            stage1_acquisition_scales=tuple(
                float(value)
                for value in cfg.recovery_stage1_acquisition_scales
            ),
            stage1_acquisition_ready_thresholds=tuple(
                float(value)
                for value in cfg.recovery_stage1_acquisition_ready_thresholds
            ),
            stage1_acquisition_ramp_steps=int(
                cfg.recovery_stage1_acquisition_ramp_steps
            ),
            stage1_acquisition_timeout_steps=int(
                cfg.recovery_stage1_acquisition_timeout_steps
            ),
            stage2_enter_completion=float(
                cfg.recovery_stage2_enter_completion
            ),
            stage2_enter_position=float(cfg.recovery_stage2_enter_position),
            stage2_enter_velocity=float(cfg.recovery_stage2_enter_velocity),
            stage2_enter_normal=float(cfg.recovery_stage2_enter_normal),
            stage2_enter_composite=float(cfg.recovery_stage2_enter_composite),
            stage2_enter_ready=float(cfg.recovery_stage2_enter_ready),
            stage2_enter_safe_recovery=float(
                cfg.recovery_stage2_enter_safe_recovery
            ),
            stage2_enter_virtual_contact=float(
                cfg.recovery_stage2_enter_virtual_contact
            ),
            stage2_enter_virtual_over_net=float(
                cfg.recovery_stage2_enter_virtual_over_net
            ),
            stage2_enter_virtual_legal=float(
                cfg.recovery_stage2_enter_virtual_legal
            ),
            stage2_enter_post_fall=float(
                cfg.recovery_stage2_enter_post_fall_max
            ),
            stage2_enter_actual_q_fault=float(
                cfg.recovery_stage2_enter_actual_q_fault_max
            ),
            stage2_enter_dwell_steps=int(
                cfg.recovery_stage2_enter_dwell_steps
            ),
            stage2_exit_completion=float(
                cfg.recovery_stage2_exit_completion
            ),
            stage2_exit_position=float(cfg.recovery_stage2_exit_position),
            stage2_exit_velocity=float(cfg.recovery_stage2_exit_velocity),
            stage2_exit_normal=float(cfg.recovery_stage2_exit_normal),
            stage2_exit_composite=float(cfg.recovery_stage2_exit_composite),
            stage2_exit_ready=float(cfg.recovery_stage2_exit_ready),
            stage2_exit_safe_recovery=float(
                cfg.recovery_stage2_exit_safe_recovery
            ),
            stage2_exit_virtual_contact=float(
                cfg.recovery_stage2_exit_virtual_contact
            ),
            stage2_exit_virtual_over_net=float(
                cfg.recovery_stage2_exit_virtual_over_net
            ),
            stage2_exit_virtual_legal=float(
                cfg.recovery_stage2_exit_virtual_legal
            ),
            stage2_exit_post_fall=float(
                cfg.recovery_stage2_exit_post_fall_max
            ),
            stage2_exit_actual_q_fault=float(
                cfg.recovery_stage2_exit_actual_q_fault_max
            ),
            stage2_exit_dwell_steps=int(
                cfg.recovery_stage2_exit_dwell_steps
            ),
            stage0_to_1_ramp_steps=int(
                cfg.recovery_stage0_to_1_ramp_steps
            ),
            stage1_coverage_ramp_steps=int(
                cfg.recovery_stage1_coverage_ramp_steps
            ),
            stage1_to_0_ramp_steps=int(
                cfg.recovery_stage1_to_0_ramp_steps
            ),
            stage1_to_2_ramp_steps=int(
                cfg.recovery_stage1_to_2_ramp_steps
            ),
            stage2_to_1_ramp_steps=int(
                cfg.recovery_stage2_to_1_ramp_steps
            ),
        )
        self._recovery_curriculum_config.validate()
        if cfg.adaptive_sigma:
            if int(cfg.sigma_update_every) < 1:
                raise ValueError(
                    f"sigma_update_every must be >= 1; got {cfg.sigma_update_every}"
                )
            for channel in ("pos", "vel"):
                minimum = float(getattr(cfg, f"sigma_{channel}_min"))
                maximum = float(getattr(cfg, f"sigma_{channel}_max"))
                if minimum <= 0.0 or maximum < minimum:
                    raise ValueError(
                        f"invalid sigma_{channel} bounds: minimum={minimum}, maximum={maximum}"
                    )

        self._ability_curriculum_mode = str(cfg.ability_curriculum_mode)
        if self._ability_curriculum_mode not in {
            "disabled",
            "one_way_strike_gate_v1",
        }:
            raise ValueError(
                "ability_curriculum_mode must be disabled or "
                f"one_way_strike_gate_v1, got {self._ability_curriculum_mode!r}"
            )
        self._ability_curriculum_enabled = (
            self._ability_curriculum_mode == "one_way_strike_gate_v1"
        )
        for name in (
            "ability_min_exact_samples_per_side",
            "ability_min_completion_per_side",
            "ability_min_position_pass_per_side",
            "ability_min_composite",
            "ability_max_post_fall",
        ):
            value = float(getattr(cfg, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for name in (
            "ability_min_completion_per_side",
            "ability_min_position_pass_per_side",
            "ability_min_composite",
            "ability_max_post_fall",
        ):
            if float(getattr(cfg, name)) > 1.0:
                raise ValueError(f"{name} must not exceed 1.0")
        if int(cfg.ability_gate_dwell_steps) < 1:
            raise ValueError("ability_gate_dwell_steps must be positive")
        if int(cfg.base_mocap_robustness_ramp_steps) < 1:
            raise ValueError("base_mocap_robustness_ramp_steps must be positive")
        if self._ability_curriculum_enabled and not (
            bool(cfg.base_mocap_enabled)
            and bool(cfg.base_mocap_orientation_enabled)
        ):
            raise ValueError(
                "one_way_strike_gate_v1 requires the full-pose base mocap path"
            )
        self._ability_unlocked = not self._ability_curriculum_enabled
        self._ability_gate_dwell = 0
        self._ability_unlock_step = -1
        self._base_mocap_robustness_scale = (
            0.0 if self._ability_curriculum_enabled else 1.0
        )

        self.robot: Articulation = env.scene[cfg.asset_name]

        # Resolve the racket FK source: prefer the racket body if it survived the physics import,
        # otherwise fall back to (wrist body pose) * (constant mount offset).
        # NOTE: Articulation.find_bodies() RAISES (resolve_matching_names) when a name matches no
        # body — it does not return []. So gate on body_names membership before calling it, or the
        # wrist-offset fallback below becomes unreachable.
        if cfg.racket_body_name in self.robot.body_names:
            self._racket_mode = "body"
            self._racket_body_index = self.robot.find_bodies(cfg.racket_body_name, preserve_order=True)[0][0]
            self._wrist_body_index = -1
        else:
            self._racket_mode = "wrist_offset"
            self._racket_body_index = -1
            assert cfg.wrist_body_name in self.robot.body_names, (
                f"RacketTargetCommand: neither racket body '{cfg.racket_body_name}' nor wrist body "
                f"'{cfg.wrist_body_name}' found on asset '{cfg.asset_name}'."
            )
            self._wrist_body_index = self.robot.find_bodies(cfg.wrist_body_name, preserve_order=True)[0][0]

        self._mount_offset = torch.tensor(cfg.mount_offset, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )
        # Fixed wrist->racket rotation (used only in wrist_offset fallback mode so the face normal is
        # taken in the racket frame, not the bare wrist frame). Identity for the A3 mount (all mount
        # joints are rpy=0); set non-identity if your mount tilts the paddle relative to the wrist.
        self._mount_quat = torch.tensor(cfg.mount_quat, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )

        # The motion command (resolved lazily on first update; not guaranteed to exist at __init__).
        self._motion_term: MotionCommand | None = None
        # Per-env motion phase at the last target resample; used to detect clip wraps (new swings).
        # Stamped on every resample so a reset-time resample is not double-triggered next step.
        self._prev_motion_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Desired (sampled) targets, world frame.
        self.racket_target_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_normal_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_normal_w[:, 2] = 1.0
        self.base_target_pos_w = torch.zeros(self.num_envs, 2, device=self.device)
        self.swing_sign = torch.ones(self.num_envs, device=self.device)

        # Actual racket state, world frame (from FK).
        self.racket_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_quat_w = torch.zeros(self.num_envs, 4, device=self.device)
        self.racket_quat_w[:, 0] = 1.0
        self.racket_lin_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_normal_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_normal_w[:, 2] = 1.0

        # --- Tier-1 virtual incoming ball (rewardDesign.md): per-swing sampled incoming state and
        # the at-strike outcome caches read by the one-shot virtual_* reward terms. vb_fired is
        # recomputed EVERY step in _update_metrics (true only on a gated exact-strike frame), so the
        # cached outcome is consumed exactly once per swing. Buffers exist even when the feature is
        # off (all-zero / all-False) so the reward terms are safely inert.
        self.vb_vel_in_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.vb_spin_in_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.vb_fired = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_landing_xy = torch.zeros(self.num_envs, 2, device=self.device)
        self.vb_landing_valid = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_on_opponent = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_depth_ok = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_net_z = torch.zeros(self.num_envs, device=self.device)
        self.vb_net_clear = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_net_crossed = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.vb_topspin = torch.zeros(self.num_envs, device=self.device)
        self.vb_spin_out_norm = torch.zeros(self.num_envs, device=self.device)
        self._venue_tuple_cohort = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._venue_tuple_cohort_fraction = 0.0
        if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
            quota = int(
                round(
                    float(cfg.venue_tuple_final_mix_prob)
                    * float(self.num_envs)
                    / 2.0
                )
            )
            if 2 * quota > self.num_envs:
                raise ValueError(
                    "fixed_balanced_bank_v1 reserves more environments than exist"
                )
            self._venue_tuple_cohort[:quota] = 0
            self._venue_tuple_cohort[quota : 2 * quota] = 1
            self._venue_tuple_cohort_fraction = (
                float(2 * quota) / float(max(self.num_envs, 1))
            )
        self._venue_tuple_selected = self._venue_tuple_cohort >= 0
        # Resolve each physical question exactly once: either at its exact
        # strike frame, or as a zero-outcome failure if it resamples first.
        # This keeps numerator and denominator on the same EMA timestamp while
        # retaining pre-strike falls/misses in the unconditional rate.
        self._venue_tuple_outcome_pending = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._venue_tuple_outcome_clip = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._venue_intended_landing_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        # The tuple bank also contains the contact normal returned by the ball planner.  That
        # quantity is useful when auditing the planner/ball model, but it is NOT the HitterV11
        # V14 training normal.  V14's hitter_pure contract is n_target = normalize(v_target),
        # including for tuple-cohort questions.  Keep the planner value in a disjoint buffer so
        # it can never silently replace the normal consumed by the strike reward.
        self._venue_planner_contact_normal_w = torch.zeros(
            self.num_envs, 3, device=self.device
        )
        self._venue_outgoing_velocity_seed = torch.zeros(
            self.num_envs, 3, device=self.device
        )
        self._venue_tuple_bank_by_clip: dict[
            int, dict[str, torch.Tensor]
        ] = {}
        self._venue_tuple_bank_path: str | None = None
        self._venue_tuple_bank_receipt_path: str | None = None
        if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
            self._load_venue_tuple_bank()
        self._vb_params = None  # lazy venue-yaml load on first evaluation
        # Derived table landmarks (env frame), from geometry.py ITTF constants.
        from whole_body_tracking.tasks.table_tennis import geometry as _tt_geom

        self._vb_net_x = float(cfg.vb_table_near_x) + _tt_geom.NET_X
        self._vb_far_x = float(cfg.vb_table_near_x) + _tt_geom.TABLE_LENGTH
        self._vb_half_w = _tt_geom.TABLE_WIDTH / 2.0
        self._vb_net_top_z = float(cfg.vb_table_surface_z) + _tt_geom.NET_HEIGHT
        self._vb_ball_r = _tt_geom.BALL_RADIUS
        self._vb_target_xy = torch.tensor(
            [float(cfg.vb_target_x), float(cfg.vb_target_y)], device=self.device
        )
        # Sample-weighted EMA accumulators (same decay/min-count discipline as the exact-strike block).
        self._vb_exact_acc = 0.0
        self._vb_hit_acc = 0.0
        self._vb_net_acc = 0.0
        self._vb_land_valid_acc = 0.0
        self._vb_inb_acc = 0.0
        self._vb_exact_acc_c = {0: 0.0, 1: 0.0}
        self._vb_hit_acc_c = {0: 0.0, 1: 0.0}
        self._vb_net_acc_c = {0: 0.0, 1: 0.0}
        self._vb_legal_acc_c = {0: 0.0, 1: 0.0}
        self._vb_land_err_sum_c = {0: 0.0, 1: 0.0}
        self._vb_land_err_n_c = {0: 0.0, 1: 0.0}

        # Reference racket state at the strike frame (CONSTANT per clip): pos (env-origin relative),
        # world linear velocity, and face normal, computed by the SAME FK as the actual racket
        # (_compute_racket_state) but fed the reference MOTION's body poses. Used by the
        # "reference_perturbed" target mode so a sampled target is one the imitated swing can actually
        # reach (a perfect imitator hits it exactly). Cached lazily on first resample, after the motion
        # term is resolved and its motion_file is loaded.
        self._ref_strike_cached = False
        # Unified multi-clip: per-clip strike phase as a [num_segments] tensor (built lazily once the
        # motion term is resolved). None until then; falls back to the scalar strike_phase.
        self._strike_phase_per_clip_t = None
        # Per-clip reference paddle FACE NORMAL at the strike frame ([num_segments, 3], built lazily). In
        # uniform mode the target normal is set to the imitated swing's actual paddle normal (which the
        # policy can achieve) — NOT the racket-velocity direction, which is ~18-110 deg off the +Y blade
        # face and makes the normal goal (and thus the composite success metric) unsatisfiable.
        self._ref_normal_per_clip = None
        # Optional per-clip racket target-velocity boxes (uniform mode). Built ONCE from cfg; stays None
        # when the shared box is used (backward compatible). Shape (num_clips, 3, 2): [clip][x/y/z][lo/hi].
        self._vel_range_per_clip_t = None
        if self.cfg.racket_vel_range_per_clip is not None:
            self._vel_range_per_clip_t = torch.tensor(
                [[[float(lo), float(hi)] for (lo, hi) in clip_rng]
                 for clip_rng in self.cfg.racket_vel_range_per_clip],
                dtype=torch.float32,
                device=self.device,
            )
        # Optional velocity-box curriculum.  The final box above remains the exported/deploy
        # safety envelope; this start box only controls training samples while the command space
        # expands.  It is deliberately per-clip because the planner mismatch is dominated by the
        # backhand's upward velocity component.
        self._vel_start_range_per_clip_t = None
        if self.cfg.racket_vel_start_range_per_clip is not None:
            if self._vel_range_per_clip_t is None:
                raise ValueError(
                    "racket_vel_start_range_per_clip requires racket_vel_range_per_clip"
                )
            self._vel_start_range_per_clip_t = torch.tensor(
                [[[float(lo), float(hi)] for (lo, hi) in clip_rng]
                 for clip_rng in self.cfg.racket_vel_start_range_per_clip],
                dtype=torch.float32,
                device=self.device,
            )
            if self._vel_start_range_per_clip_t.shape != self._vel_range_per_clip_t.shape:
                raise ValueError(
                    "racket_vel_start_range_per_clip must match racket_vel_range_per_clip shape"
                )
            start_lo = self._vel_start_range_per_clip_t[..., 0]
            start_hi = self._vel_start_range_per_clip_t[..., 1]
            final_lo = self._vel_range_per_clip_t[..., 0]
            final_hi = self._vel_range_per_clip_t[..., 1]
            if bool(torch.any(start_lo < final_lo) or torch.any(start_hi > final_hi)):
                raise ValueError(
                    "racket_vel_start_range_per_clip must be contained in the final velocity box"
                )
        self._vel_bootstrap_range_per_clip_t = None
        if self.cfg.racket_vel_bootstrap_range_per_clip is not None:
            if self._vel_start_range_per_clip_t is None:
                raise ValueError(
                    "racket_vel_bootstrap_range_per_clip requires a start velocity box"
                )
            self._vel_bootstrap_range_per_clip_t = torch.tensor(
                [
                    [[float(lo), float(hi)] for (lo, hi) in clip_rng]
                    for clip_rng in self.cfg.racket_vel_bootstrap_range_per_clip
                ],
                dtype=torch.float32,
                device=self.device,
            )
            if (
                self._vel_bootstrap_range_per_clip_t.shape
                != self._vel_start_range_per_clip_t.shape
            ):
                raise ValueError(
                    "racket_vel_bootstrap_range_per_clip must match the start velocity-box shape"
                )
            bootstrap_lo = self._vel_bootstrap_range_per_clip_t[..., 0]
            bootstrap_hi = self._vel_bootstrap_range_per_clip_t[..., 1]
            start_lo = self._vel_start_range_per_clip_t[..., 0]
            start_hi = self._vel_start_range_per_clip_t[..., 1]
            if bool(
                torch.any(bootstrap_lo < start_lo)
                or torch.any(bootstrap_hi > start_hi)
            ):
                raise ValueError(
                    "racket_vel_bootstrap_range_per_clip must be contained in the start velocity box"
                )
        # Optional planner-distribution component.  The final envelope is the union accepted by
        # deployment; training samples the demonstrated core and this narrower planner box as a
        # mixture instead of drawing uniformly from every corner of that union.
        self._vel_planner_range_per_clip_t = None
        if self.cfg.racket_vel_planner_range_per_clip is not None:
            if self._vel_range_per_clip_t is None or self._vel_start_range_per_clip_t is None:
                raise ValueError(
                    "racket_vel_planner_range_per_clip requires final and start velocity boxes"
                )
            self._vel_planner_range_per_clip_t = torch.tensor(
                [[[float(lo), float(hi)] for (lo, hi) in clip_rng]
                 for clip_rng in self.cfg.racket_vel_planner_range_per_clip],
                dtype=torch.float32,
                device=self.device,
            )
            if self._vel_planner_range_per_clip_t.shape != self._vel_range_per_clip_t.shape:
                raise ValueError(
                    "racket_vel_planner_range_per_clip must match final velocity-box shape"
                )
            planner_lo = self._vel_planner_range_per_clip_t[..., 0]
            planner_hi = self._vel_planner_range_per_clip_t[..., 1]
            final_lo = self._vel_range_per_clip_t[..., 0]
            final_hi = self._vel_range_per_clip_t[..., 1]
            if bool(torch.any(planner_lo < final_lo) or torch.any(planner_hi > final_hi)):
                raise ValueError(
                    "racket_vel_planner_range_per_clip must be contained in the final velocity box"
                )
            mix_prob = float(self.cfg.racket_vel_planner_mix_prob)
            if not 0.0 <= mix_prob <= 1.0:
                raise ValueError("racket_vel_planner_mix_prob must be in [0, 1]")
            stage1_mix = float(
                self.cfg.racket_vel_stage1_planner_mix_prob
            )
            if not 0.0 <= stage1_mix <= mix_prob:
                raise ValueError(
                    "racket_vel_stage1_planner_mix_prob must be in "
                    "[0, racket_vel_planner_mix_prob]"
                )
        progress_override = self.cfg.racket_vel_curriculum_progress_override
        if progress_override is not None:
            if self._vel_planner_range_per_clip_t is None:
                raise ValueError(
                    "racket_vel_curriculum_progress_override requires a planner velocity box"
                )
            progress_override = float(progress_override)
            if not 0.0 <= progress_override <= 1.0:
                raise ValueError(
                    "racket_vel_curriculum_progress_override must be in [0, 1]"
                )
        # Optional per-clip racket target-POSITION boxes (uniform mode). Same shape/semantics as the
        # velocity one above; None -> shared pos box (backward compatible). (num_clips, 3, 2): [clip][x/y/z][lo/hi].
        self._pos_range_per_clip_t = None
        if self.cfg.racket_pos_range_per_clip is not None:
            self._pos_range_per_clip_t = torch.tensor(
                [[[float(lo), float(hi)] for (lo, hi) in clip_rng]
                 for clip_rng in self.cfg.racket_pos_range_per_clip],
                dtype=torch.float32,
                device=self.device,
            )
        # Optional PER-CLIP mount-normal SIGN (unified fh+bh policy: the two swings strike with OPPOSITE
        # paddle faces). Built ONCE from cfg; None -> scalar mount_normal_sign for every clip (backward
        # compatible). Shape (num_clips,): the racket-frame +axis sign that selects the striking FACE per clip.
        self._mount_sign_per_clip_t = None
        if self.cfg.mount_normal_sign_per_clip:
            self._mount_sign_per_clip_t = torch.tensor(
                [float(s) for s in self.cfg.mount_normal_sign_per_clip],
                dtype=torch.float32,
                device=self.device,
            )
        self._ref_racket_pos_rel = torch.zeros(3, device=self.device)
        self._ref_racket_vel_w = torch.zeros(3, device=self.device)
        self._ref_racket_normal_w = torch.zeros(3, device=self.device)
        # Reference base (root) XY at the strike + the base->racket horizontal offset. Used to COUPLE
        # base_target to racket_target so standing at base_target keeps the racket reachable.
        self._ref_base_pos_rel = torch.zeros(3, device=self.device)
        self._ref_reach_offset_xy = torch.zeros(2, device=self.device)
        self._ref_racket_pos_rel_per_clip = None
        self._ref_racket_vel_w_per_clip = None
        self._ref_racket_normal_w_per_clip = None
        self._ref_base_pos_rel_per_clip = None
        self._ref_reach_offset_xy_per_clip = None

        # Success-gated curriculum: running perturbation scale, advanced only when the smoothed
        # exact-strike composite success clears the threshold (see _perturb_scale / _update_metrics).
        self._curr_perturb_scale = float(cfg.ref_perturb_curriculum_start)

        # Decayed accumulators for the CONDITIONAL exact-strike pass rates (see _update_metrics). The
        # logged strike_*_pass_exact / strike_composite_success_exact report the fraction of *exact-strike
        # samples* that clear each acceptance threshold — NOT a per-env value held over the long
        # non-strike portion of every episode (which diluted the old metric ~10x at reset-logging time).
        # Sample-weighted EMA: acc = decay*acc + this-step-count; rate = pass_acc / sample_acc.
        self._exact_n_acc = 0.0
        self._exact_pass_comp_acc = 0.0
        self._exact_pass_pos_acc = 0.0
        self._exact_pass_vel_acc = 0.0
        self._exact_pass_normal_acc = 0.0
        # 5 cm / 10 cm position-accuracy buckets on the SAME exact-strike mask + EMA denominator as the
        # pass metrics above (so they are comparable with the composite, unlike the old window-exit-held
        # strike_success_5cm/10cm which sampled the racket ~0.26 m past target).
        self._exact_pass_5cm_acc = 0.0
        self._exact_pass_10cm_acc = 0.0
        self._exact_composite_rate = 0.0
        # P2.3 adaptive-sigma driver: global decayed error-magnitude sums over exact-strike samples
        # (mean = sum / _exact_n_acc). Per-clip variants exist below; sigma needs one global signal.
        self._exact_pos_err_sum = 0.0
        self._exact_vel_err_sum = 0.0
        # racket_normal is gated on the whole strike window, so its sigma driver is the
        # WINDOW-mean face error in radians (not the exact-frame error used above).
        self._win_normal_err_sum = 0.0
        self._win_n_acc = 0.0
        # Live adaptive sigmas (start at the cfg maxima = the hand-tuned YAML stds; only applied to
        # the reward terms when cfg.adaptive_sigma is on).
        self._adaptive_sigma_pos = float(cfg.sigma_pos_max)
        self._adaptive_sigma_vel = float(cfg.sigma_vel_max)
        self._adaptive_sigma_normal = float(cfg.sigma_normal_max)
        # Reversible precision/stability curriculum on racket_velocity (see cfg block). The full
        # weight is read lazily from the live reward term (= task-YAML value). Progress and gate
        # dwell are captured for exact resume in my_on_policy_runner.
        self._vel_weight_latched = False
        self._vel_weight_full = None
        self._vel_weight_progress = 0.0
        self._vel_weight_gate_dwell = 0
        # Staged-hysteresis v2 state. Every field that changes the live objective is a scalar
        # attribute so MotionOnPolicyRunner can checkpoint and exactly restore it.
        self._velocity_stage = 0
        self._velocity_current_weight = float(
            self._velocity_stage_config.stage0_weight
        )
        self._velocity_target_weight = float(
            self._velocity_stage_config.stage0_weight
        )
        self._velocity_stage1_enter_dwell = 0
        self._velocity_stage1_exit_dwell = 0
        self._velocity_stage2_enter_dwell = 0
        self._velocity_stage2_exit_dwell = 0
        # RallyV17 recovery curriculum. Scalar fields are intentionally explicit so exact-resume
        # checkpoints can restore every quantity that changes the live reset/reward distribution.
        self._recovery_stage = 0
        self._recovery_current_scale = 0.0
        self._recovery_target_scale = 0.0
        self._recovery_ramp_rate_per_step = 0.0
        self._recovery_coverage_scale = 0.0
        self._recovery_coverage_target_scale = 0.0
        self._recovery_coverage_ramp_rate_per_step = 0.0
        self._recovery_stage1_coverage_unlocked = False
        self._recovery_stage1_acquisition_rung = 0
        self._recovery_stage1_acquisition_failures = 0
        self._recovery_stage1_ready_dwell = 0
        self._recovery_stage1_acquisition_age = 0
        self._recovery_stage1_enter_dwell = 0
        self._recovery_stage1_exit_dwell = 0
        self._recovery_stage2_enter_dwell = 0
        self._recovery_stage2_exit_dwell = 0
        # Deterministic checkpoint qualification constructs a fresh environment, so it cannot
        # restore the training runner's curriculum scalars from the checkpoint.  The evaluator may
        # explicitly pin the already-converged V17 distribution at Stage 2.  This flag is runtime
        # only, defaults off, and is never set by a training recipe.
        self._eval_force_final_recovery_stage = False

        # Per-clip (forehand=clip 0 / backhand=clip 1) breakdown of the exact-strike metrics, so wandb
        # shows each swing separately (the aggregate composite can hide one swing lagging). Same
        # sample-weighted EMA as the global accumulators above, but each clip's exact-strike samples are
        # accumulated separately (selected by the motion command's clip_id). Populated in multiseg only.
        self._clip_names = {0: "forehand", 1: "backhand"}
        self._exact_n_acc_c = {c: 0.0 for c in self._clip_names}
        self._exact_pass_pos_acc_c = {c: 0.0 for c in self._clip_names}
        self._exact_pass_vel_acc_c = {c: 0.0 for c in self._clip_names}
        self._exact_pass_normal_acc_c = {c: 0.0 for c in self._clip_names}
        self._exact_pass_comp_acc_c = {c: 0.0 for c in self._clip_names}
        self._exact_pos_err_sum_c = {c: 0.0 for c in self._clip_names}
        self._exact_vel_err_sum_c = {c: 0.0 for c in self._clip_names}
        self._exact_nrm_err_sum_c = {c: 0.0 for c in self._clip_names}
        self._exact_pos_signed_sum_c = {
            c: torch.zeros(3, device=self.device) for c in self._clip_names
        }
        self._exact_vel_signed_sum_c = {
            c: torch.zeros(3, device=self.device) for c in self._clip_names
        }
        self._exact_normal_dot_sum_c = {
            c: torch.zeros((), device=self.device) for c in self._clip_names
        }
        # Small cohort split for the r12 fixed mixture.  These metrics answer the only useful
        # ablation question: did the 25% physical bank hurt the unchanged 75% V11 questions?
        # They are exact-strike conditional metrics; unconditional physical outcomes use the
        # venue swing-start denominator below so pre-strike falls remain failures.
        self._cohort_names = ("core", "tuple")
        self._cohort_exact_acc = {
            (cohort, clip): 0.0
            for cohort in self._cohort_names
            for clip in self._clip_names
        }
        self._cohort_pos_acc = dict.fromkeys(self._cohort_exact_acc, 0.0)
        self._cohort_vel_acc = dict.fromkeys(self._cohort_exact_acc, 0.0)
        self._cohort_comp_acc = dict.fromkeys(self._cohort_exact_acc, 0.0)
        self._venue_swing_starts_acc_c = {
            clip: 0.0 for clip in self._clip_names
        }
        # Training-only strike audit matrix.  A target is classified once, using values already
        # sampled for the command (no additional RNG): clip x velocity source x speed quartile x
        # target-z third = 2 x 2 x 4 x 3 = 48 strata.  These buffers are never read by observations,
        # rewards, target sampling or action decoding.  The runner exports their cumulative ratios
        # in one batched device->host transfer per PPO iteration.
        self._strike_audit_shape = (2, 2, 4, 3)
        self._strike_audit_size = math.prod(self._strike_audit_shape)
        self._strike_audit_context_id = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._recover_strike_audit_context_id = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._target_velocity_source = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._target_speed_quartile = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._target_z_bin = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._strike_audit_count = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        self._strike_audit_pos_abs_sum = torch.zeros(
            self._strike_audit_size, 3, device=self.device
        )
        self._strike_audit_pos_signed_sum = torch.zeros(
            self._strike_audit_size, 3, device=self.device
        )
        self._strike_audit_base_abs_sum = torch.zeros(
            self._strike_audit_size, 2, device=self.device
        )
        self._strike_audit_base_signed_sum = torch.zeros(
            self._strike_audit_size, 2, device=self.device
        )
        self._strike_audit_pass_sum = torch.zeros(
            self._strike_audit_size, 4, device=self.device
        )
        self._strike_audit_start_count = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        self._strike_audit_postfall_count = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        # Strike-local reward telemetry. Per-env accumulators produce true event returns after
        # RewardTerm weight and policy-step dt; cumulative context bins reuse the existing
        # FH/BH x core/planner x speed-quartile x z matrix and never feed the task.
        self._position_guidance_prev_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._position_guidance_event_acc = torch.zeros(
            self.num_envs, device=self.device
        )
        self._exact_position_debt_prev_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._exact_position_debt_event_acc = torch.zeros(
            self.num_envs, device=self.device
        )
        self._strike_local_position_event_count = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        self._strike_local_position_event_sum = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        self._strike_local_exact_debt_event_count = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        self._strike_local_exact_debt_event_sum = torch.zeros(
            self._strike_audit_size, device=self.device
        )
        # Per-joint audit buffers are initialized lazily once the action manager exposes its
        # resolved action-column order.  They aggregate exact-strike observations only.
        self._qdes_joint_audit_names = ()
        self._qdes_joint_audit_count = torch.zeros((), device=self.device)
        self._qdes_joint_audit_sum = None
        self._qdes_joint_audit_max = None
        self._qdes_joint_tightest_count = None
        # Foundation stability matrix: phase x (FH/BH, core/planner, speed quartile,
        # station-distance bucket, first/later strike). It is telemetry-only.
        self._stability_phases = ("pre_strike", "strike", "post_strike", "recovery")
        self._stability_metric_names = (
            "base_tilt_deg",
            "base_angular_velocity_rad_s",
            "base_linear_velocity_mps",
            "foot_contact_fraction",
            "foot_slip_mps",
            "com_support_proxy",
            "station_error_m",
            "yaw_error_deg",
            "base_x_excursion_m",
            "base_y_excursion_m",
            "foot_unloading_fraction",
            "foot_liftoff_event_count",
            "torque_peak_nm",
            "joint_near_limit_fraction",
            "qdes_interval_width_min_fraction",
            "base_x_drift_m",
            "base_y_drift_m",
            "step_reentry",
            "next_strike_readiness",
        )
        self._stability_shape = (2, 2, 4, 5, 2)
        self._stability_size = math.prod(self._stability_shape)
        phase_count = len(self._stability_phases)
        metric_count = len(self._stability_metric_names)
        self._foundation_context_id = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._foundation_recovery_context_id = torch.full_like(
            self._foundation_context_id, -1
        )
        self._foundation_strike_ordinal = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._stability_sample_count = torch.zeros(
            phase_count, self._stability_size, device=self.device
        )
        self._stability_metric_sum = torch.zeros(
            phase_count,
            self._stability_size,
            metric_count,
            device=self.device,
        )
        self._stability_metric_max = torch.full_like(
            self._stability_metric_sum, -float("inf")
        )
        self._stability_metric_min = torch.full_like(
            self._stability_metric_sum, float("inf")
        )
        self._foundation_start_count = torch.zeros(
            self._stability_size, device=self.device
        )
        self._foundation_prefall_count = torch.zeros_like(
            self._foundation_start_count
        )
        self._foundation_postfall_count = torch.zeros_like(
            self._foundation_start_count
        )
        # Recovery event times: full recovery, both-feet contact, STAND, next readiness.
        self._recovery_event_time_sum = torch.zeros(
            self._stability_size, 4, device=self.device
        )
        self._recovery_event_count = torch.zeros_like(
            self._recovery_event_time_sum
        )
        self._recovery_timer_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._recovery_age_s = torch.zeros(self.num_envs, device=self.device)
        self._recovery_strike_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._recovery_event_seen = torch.zeros(
            self.num_envs, 4, dtype=torch.bool, device=self.device
        )
        self._foundation_prev_tts = torch.full(
            (self.num_envs,), 1.0e9, device=self.device
        )
        self._stability_prev_foot_contact = torch.ones(
            self.num_envs, 2, dtype=torch.bool, device=self.device
        )

        # --- UNCONDITIONAL swing accounting (Phase A wandb fix) ------------------------------------
        # strike_composite_success_exact is CONDITIONAL: its denominator is exact-strike SAMPLES, so
        # an env that falls BEFORE the strike frame contributes nothing — composite ~1.0 coexists
        # with any pre-strike fall rate (exactly what happened in deploy). These accumulators count
        # every swing START (episode reset or clip wrap assigns a new swing) with the same EMA decay
        # as the exact accumulators, so:
        #   swing_completion_rate = exact_n_acc / swing_starts_acc   (unconditional; falls count)
        #   pre_strike_fall_rate  = pre-strike terminations / swing starts
        self._swing_starts_acc = 0.0
        self._swing_starts_acc_c = {c: 0.0 for c in self._clip_names}
        self._prestrike_fall_acc = 0.0
        # POST-strike falls (fall AFTER reaching the strike frame — the follow-through/recovery fall that
        # swing_completion_rate + pre_strike_fall_rate are both blind to; it was the actual backhand
        # deploy failure mode). Same swing-starts denominator as pre_strike_fall_rate.
        self._poststrike_fall_acc = 0.0
        # Per-clip fall attribution. NOTE: at reset time the MOTION command has already resampled
        # clip_id to the NEW swing (motion resets before racket_target), so falls are attributed via
        # _prev_clip_id — the clip snapshot taken at the END of the previous _update_command, i.e. the
        # clip the env was actually swinging when it fell.
        self._prestrike_fall_acc_c = {c: 0.0 for c in self._clip_names}
        self._poststrike_fall_acc_c = {c: 0.0 for c in self._clip_names}
        self._prev_clip_id = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # Post-wrap recovery latch: >=0 = the clip whose swing JUST finished while this env sits in the
        # post-wrap hold. A fall during that hold is physically the PREVIOUS swing's recovery fall, but
        # at the wrap the timing already describes the NEXT swing (pre_strike=True, clip_id=new random
        # clip) — without the latch such falls book as pre-strike falls of a 50%-wrong clip, which would
        # invert exactly the backhand-recovery diagnosis these metrics exist for. -1 = not recovering.
        self._recover_from_clip = torch.full((self.num_envs,), -1, dtype=torch.long, device=self.device)
        # Arrival-gated hold (cfg.hold_until_settled): per-env count of EXTRA hold steps spent waiting for
        # the base to settle at the station beyond the base countdown; reset when the env leaves the hold,
        # capped by cfg.hold_settle_max_extra_steps. 0 whenever the feature is off.
        self._hold_extra_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # RallyFinalV2 deploy-ready coverage: the training hold has a FIXED external countdown, but
        # we still score whether the policy would have cleared the native runner's readiness gate
        # before that timer released it. This is diagnostic only -- it never extends hold_counter
        # and therefore cannot recreate the V5 arrival-gated/GAE farming path.
        self._ready_elapsed_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._ready_dwell_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._ready_latched = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._ready_ever_ready = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._ready_prev_held = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._ready_transition_eligible = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._ready_latency_s = torch.zeros(self.num_envs, device=self.device)
        self._ready_release_n_acc = 0.0
        self._ready_release_pass_acc = 0.0
        # Decayed release-event ratio used by the staged curriculum. The old instantaneous
        # ``_ready_latched.mean()`` depended on how many envs happened to be inside a hold on one
        # control tick, rather than whether completed station transitions were actually READY.
        # Keep the cumulative counters above for the long-horizon audit metric and use this
        # event-conditioned EMA only as the reversible curriculum gate.
        self._ready_release_n_ema = 0.0
        self._ready_release_pass_ema = 0.0
        self._ready_release_n_ema_c = {0: 0.0, 1: 0.0}
        self._ready_release_pass_ema_c = {0: 0.0, 1: 0.0}
        self._ready_latency_sum_acc = 0.0
        self._ready_latency_n_acc = 0.0
        self._ready_release_required = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._ready_release_wait_steps = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self.ready_release_timeout = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._ready_release_timeout_count = torch.zeros(
            2, device=self.device
        )
        # Same-decay sibling of _swing_starts_acc_c.  It is used only to remove sampled READY
        # questions that timed out before release from the Stage-1 strike-safety denominator.
        # Real falls and all other aborted swings remain completion failures.
        self._ready_release_timeout_acc_c = {
            c: 0.0 for c in self._clip_names
        }
        # An exact composite hit opens a recovery question. It resolves on the next READY release
        # (pass) or reset/timeout (fail), independently per source side.
        self._safe_recovery_pending = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._safe_recovery_source_clip = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._safe_recovery_n_ema_c = {0: 0.0, 1: 0.0}
        self._safe_recovery_pass_ema_c = {0: 0.0, 1: 0.0}
        self._actual_q_fault_acc_c = {0: 0.0, 1: 0.0}
        self._actual_q_fault_counter_seen = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        # Revision-3 finite actual-q audit.  The old decayed floating-point numerator could not
        # represent the contract "zero events in a finite window": one event stayed positive for
        # an implementation-dependent duration and an underflow, rather than an auditable number
        # of control steps.  Aggregate event/start counts once per control step in a fixed ring.
        actual_q_window_steps = int(
            self._recovery_curriculum_config.actual_q_window_steps
        )
        self._actual_q_window_faults = torch.zeros(
            actual_q_window_steps, 2, device=self.device
        )
        self._actual_q_window_starts = torch.zeros_like(
            self._actual_q_window_faults
        )
        self._actual_q_window_pending_faults = torch.zeros(
            2, device=self.device
        )
        self._actual_q_window_pending_starts = torch.zeros(
            2, device=self.device
        )
        self._actual_q_window_ptr = 0
        self._actual_q_window_filled_steps = 0
        # True only while _resample_command is invoked from the intra-episode WRAP path (see
        # _update_command): wraps start a new swing but never count a pre-strike fall (a wrapped
        # env necessarily passed its strike frame alive).
        self._resample_is_wrap = False

        # --- Rally drift accounting (2026-07-07 continuous-rally upgrade) -------------------------
        # Deploy P7 failure mode: each walk-and-strike lunges forward; over consecutive swings the
        # displacement ACCUMULATES until a swing starts from an untrained stance and falls. These
        # track exactly that: per-swing base displacement (closed out at WRAPS = completed swings
        # only), its forward (x) component (drift is directional: forward), and the base->station
        # error at each swing start (how far the new station is when the swing begins — the recovery
        # debt the previous swing left behind). Same EMA decay/denominator discipline as the exact
        # accumulators; drift uses its own wrap-count denominator (resets don't close out a swing).
        self._swing_start_base_xy = torch.zeros(self.num_envs, 2, device=self.device)
        # Stamp lazily on the FIRST _update_metrics after a swing start: at reset time the cached
        # base_pos_w still holds the PRE-reset pose (events teleport the root after the snapshot),
        # so an eager stamp would book the teleport as drift of the episode's first swing.
        self._swing_start_pending = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._drift_n_acc = 0.0
        self._drift_sum_acc = 0.0
        self._drift_fwd_sum_acc = 0.0
        self._station_offset_start_sum_acc = 0.0
        # RallyFinal diagnostics held across swings.  Clearance minima reset only when a NEW
        # backhand is armed, so an intervening forehand does not erase the last safety result.
        self._bh_hand_min = torch.full((self.num_envs,), 1.0, device=self.device)
        self._bh_forearm_min = torch.full((self.num_envs,), 1.0, device=self.device)
        self._bh_left_arm_min = torch.full((self.num_envs,), 1.0, device=self.device)
        self._rally_success_run = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._rally_success_run_max = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # v2 rally: |base yaw| at HOLD EXPIRY (swing arming, post-recovery) — wrap AND stand holds.
        self._heading_expiry_sum_acc = 0.0
        self._heading_expiry_n_acc = 0.0
        self._prev_in_hold = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        # v3 rally recovery (2026-07-08): the pooled metric above averages over ~94% near-square
        # WRAP holds, so it reads ~0.05-0.10 whether recovery works or fails (dilution false-pass).
        # De-dilute by CONDITIONING on the |yaw| the hold STARTED with: stamp base |yaw| at each
        # in_hold rising edge, and at expiry accumulate spawn/expiry ONLY for holds that started
        # yawed (>_RECOV_SPAWN_YAW_THRESH). heading_recovery_expiry_yaw then means "when a hold
        # begins badly-yawed, how squared is the base when the swing arms" — the real recovery gate.
        self._hold_spawn_yaw = torch.zeros(self.num_envs, device=self.device)
        self._recov_spawn_sum_acc = 0.0
        self._recov_expiry_sum_acc = 0.0
        self._recov_n_acc = 0.0

        # --- HER-style achieved-target replay buffers (see RacketTargetCommandCfg) -----------------
        # Per-clip ring buffers of the racket state the policy ACTUALLY produced at exact-strike frames:
        # position env-origin-relative (world minus env origin), velocity world. Written in
        # _update_metrics on the exact_strike mask (alive envs only — terminated envs were reset before
        # the command computes); read in _sample_targets_uniform with prob achieved_target_mix_prob.
        _absize = max(int(cfg.achieved_buffer_size), 1)
        self._ach_pos = {c: torch.zeros(_absize, 3, device=self.device) for c in self._clip_names}
        self._ach_vel = {c: torch.zeros(_absize, 3, device=self.device) for c in self._clip_names}
        self._ach_fill = {c: 0 for c in self._clip_names}
        self._ach_ptr = {c: 0 for c in self._clip_names}
        # Decayed counters for the logged replay fraction (same EMA timescale as the exact accumulators).
        self._resample_n_acc = 0.0
        self._replay_n_acc = 0.0

        # --- A1 target latency & time-variance (mocap->planner->runner realism) --------------------
        # MOTIVATION: training previously handed the actor a PERFECT, instantly-updated target; the
        # real loop (mocap -> planner -> runner) delivers it LATE (transport + planning latency),
        # NOISY (ball-prediction error that SHRINKS as the strike approaches — SMASH Eq. 14), and
        # REFINED mid-swing (the planner re-plans WHERE while the swing clock keeps running — PACE
        # injects sensor delays for the same reason). Without modeling this, the mocap-closed-loop
        # deployment faces out-of-distribution target dynamics. ALL knobs default OFF and the default
        # path is byte-identical: delay==0 & jitter==0 make the actor-visible views ALIAS the live
        # tensors (zero overhead, no extra RNG); midswing_resample_prob==0 short-circuits before any
        # RNG draw. Only the ACTOR-visible view is degraded — rewards, metrics, the privileged critic,
        # and the achieved-target-replay WRITE always use the TRUE live target.
        self._delay_steps = max(int(cfg.target_delay_steps), 0)
        self._jitter_pos = max(float(cfg.target_jitter_pos_per_s), 0.0)
        self._jitter_vel = max(float(cfg.target_jitter_vel_per_s), 0.0)
        # Calibrated MEASUREMENT noise (ball_physics_venue.yaml `capture:` block, 2026-07-03 fit):
        # white + AR(1)-colored position error of the mocap link. Unlike the tts-scaled jitter above
        # (which models PREDICTION convergence), measurement noise does NOT shrink as the strike
        # approaches, so no tts scaling. rho is per POLICY step: venue 0.946/frame @300 Hz -> ^6 @50 Hz.
        self._mnoise_white = max(float(cfg.target_noise_white), 0.0)
        self._mnoise_ar1_sigma = max(float(cfg.target_noise_ar1_sigma), 0.0)
        self._mnoise_ar1_rho = min(max(float(cfg.target_noise_ar1_rho), 0.0), 0.9999)
        if self._mnoise_ar1_sigma > 0.0:
            self._mnoise_ar1_state = torch.zeros(self.num_envs, 3, device=self.device)
        self._drop_prob = max(float(cfg.target_dropout_prob), 0.0)
        self._post_strike_drop_steps = max(int(round(float(cfg.target_post_strike_dropout_s) * 50.0)), 0)
        self._bias_per_swing = max(float(cfg.target_bias_per_swing), 0.0)
        self._target_robustness_curriculum_by_velocity_stage = bool(
            cfg.target_robustness_curriculum_by_velocity_stage
        )
        self._target_robustness_curriculum_by_recovery_scale = bool(
            cfg.target_robustness_curriculum_by_recovery_scale
        )
        self._target_robustness_recovery_start_scale = float(
            cfg.target_robustness_recovery_start_scale
        )
        if not 0.0 <= self._target_robustness_recovery_start_scale < 1.0:
            raise ValueError(
                "target_robustness_recovery_start_scale must be in [0, 1), got "
                f"{self._target_robustness_recovery_start_scale}"
            )
        if (
            self._target_robustness_curriculum_by_velocity_stage
            and self._target_robustness_curriculum_by_recovery_scale
        ):
            raise ValueError(
                "target robustness can be driven by velocity stage or recovery scale, not both"
            )
        if (
            self._target_robustness_curriculum_by_recovery_scale
            and not self._recovery_curriculum_enabled
        ):
            raise ValueError(
                "recovery-scale target robustness requires recovery_curriculum_enabled=true"
            )
        self._target_robustness_stage1_scale = float(
            cfg.target_robustness_stage1_scale
        )
        if not 0.0 <= self._target_robustness_stage1_scale <= 1.0:
            raise ValueError(
                "target_robustness_stage1_scale must be in [0, 1], got "
                f"{self._target_robustness_stage1_scale}"
            )
        if (
            self._target_robustness_curriculum_by_velocity_stage
            and self._velocity_curriculum_mode != "staged_hysteresis_v2"
        ):
            raise ValueError(
                "target robustness curriculum requires staged_hysteresis_v2"
            )
        self._last_target_robustness_scale = (
            0.0
            if (
                self._target_robustness_curriculum_by_velocity_stage
                or self._target_robustness_curriculum_by_recovery_scale
            )
            else 1.0
        )
        self._a1v2_active = self._drop_prob > 0.0 or self._post_strike_drop_steps > 0 or self._bias_per_swing > 0.0
        if self._a1v2_active:
            self._swing_bias = torch.zeros(self.num_envs, 3, device=self.device)
            self._drop_cd = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
            self._prev_pre_strike = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
            self._held_pos = torch.zeros(self.num_envs, 3, device=self.device)
            self._held_vel = torch.zeros(self.num_envs, 3, device=self.device)
        # The actor view is materialized (separate tensors) whenever latency OR jitter is on;
        # otherwise the delayed_* attributes ARE the live tensors (which are only ever index-assigned
        # after __init__, so the alias stays valid for the whole run).
        self._actor_view_active = (
            self._delay_steps > 0 or self._jitter_pos > 0.0 or self._jitter_vel > 0.0
            or self._mnoise_white > 0.0 or self._mnoise_ar1_sigma > 0.0
            or float(cfg.target_dropout_prob) > 0.0
            or float(cfg.target_post_strike_dropout_s) > 0.0
            or float(cfg.target_bias_per_swing) > 0.0
        )
        if self._delay_steps > 0:
            # Ring buffers (length delay+1) over the ACTOR-VISIBLE target quantities: the slot
            # written this step is read back `delay` pushes later (see _push_actor_target).
            # time_to_strike is NOT buffered ON PURPOSE: the swing clock is generated robot-side by
            # the deploy runner, not by the mocap link, so it carries no mocap latency.
            _L = self._delay_steps + 1
            self._delay_buf_pos = torch.zeros(_L, self.num_envs, 3, device=self.device)
            self._delay_buf_vel = torch.zeros(_L, self.num_envs, 3, device=self.device)
            self._delay_buf_sign = torch.ones(_L, self.num_envs, device=self.device)
            self._delay_ptr = 0
        if self._actor_view_active:
            self.delayed_racket_target_pos_w = self.racket_target_pos_w.clone()
            self.delayed_racket_target_vel_w = self.racket_target_vel_w.clone()
            self.delayed_swing_sign = self.swing_sign.clone()
        else:
            # Flags off: zero-overhead aliases of the live tensors (byte-identical baseline).
            self.delayed_racket_target_pos_w = self.racket_target_pos_w
            self.delayed_racket_target_vel_w = self.racket_target_vel_w
            self.delayed_swing_sign = self.swing_sign
        # A1 metrics: per-step per-env redraw indicator (wandb reset-mean = per-step mid-swing
        # refinement fraction) + the constant delay-in-effect broadcast (refreshed every step in
        # _update_metrics because CommandTerm.reset() zeros metric entries of resetting envs).
        self.metrics["midswing_resample_count"] = torch.zeros(self.num_envs, device=self.device)
        initial_robustness_scale = (
            0.0
            if (
                self._target_robustness_curriculum_by_velocity_stage
                or self._target_robustness_curriculum_by_recovery_scale
            )
            else 1.0
        )
        self.metrics["target_delay_steps_in_effect"] = torch.full(
            (self.num_envs,),
            float(round(self._delay_steps * initial_robustness_scale)),
            device=self.device,
        )
        self.metrics["target_robustness_scale"] = torch.full(
            (self.num_envs,), initial_robustness_scale, device=self.device
        )
        self.metrics["velocity_core_box_scale"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["velocity_planner_mix_effective"] = torch.zeros(
            self.num_envs, device=self.device
        )

        # --- Actor-visible base localization ---------------------------------------------------
        # V15 uses position receipt v1. HitterPingPong enables calibrated-pose receipt v2:
        # sample/hold a delayed/noisy full pose, preserve quaternion sign continuity,
        # and bridge only short dropouts with the pelvis body-frame gyro.  The Build task owns
        # these engineering assumptions directly in YAML; no external latency receipt is used.
        self._base_mocap_enabled = bool(cfg.base_mocap_enabled)
        self._base_mocap_orientation_enabled = bool(
            cfg.base_mocap_orientation_enabled
        )
        self._base_mocap_delay_steps = max(int(cfg.base_mocap_delay_steps), 0)
        self._base_mocap_delay_by_env = torch.full(
            (self.num_envs,),
            int(
                round(
                    self._base_mocap_delay_steps
                    * self._base_mocap_robustness_scale
                )
            ),
            dtype=torch.long,
            device=self.device,
        )
        update_interval_steps = int(cfg.base_mocap_update_interval_steps)
        noise_std = tuple(float(v) for v in cfg.base_mocap_position_noise_std)
        orientation_noise_std = tuple(
            float(v) for v in cfg.base_mocap_orientation_noise_std_rad
        )
        extrinsic_residual_std = tuple(
            float(v) for v in cfg.base_mocap_extrinsic_residual_rpy_std_rad
        )
        dropout_probability = float(cfg.base_mocap_dropout_prob)
        max_age_s = float(cfg.base_mocap_max_age_s)
        max_propagation_s = float(cfg.base_mocap_max_propagation_s)
        self._base_mocap_update_interval_steps = max(update_interval_steps, 1)
        if len(noise_std) != 3 or any(v < 0.0 for v in noise_std):
            raise ValueError(
                "base_mocap_position_noise_std must contain three non-negative values"
            )
        self._base_mocap_noise_std = torch.tensor(
            noise_std, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._base_mocap_dropout_prob = dropout_probability
        if not 0.0 <= self._base_mocap_dropout_prob < 1.0:
            raise ValueError("base_mocap_dropout_prob must be in [0, 1)")
        self._base_mocap_velocity_alpha = float(cfg.base_mocap_velocity_ema_alpha)
        if not 0.0 < self._base_mocap_velocity_alpha <= 1.0:
            raise ValueError("base_mocap_velocity_ema_alpha must be in (0, 1]")
        self._base_mocap_max_age_s = max_age_s
        if self._base_mocap_max_age_s <= 0.0:
            raise ValueError("base_mocap_max_age_s must be positive")
        if (
            len(orientation_noise_std) != 3
            or any(v < 0.0 for v in orientation_noise_std)
        ):
            raise ValueError(
                "base_mocap_orientation_noise_std_rad must contain three "
                "non-negative values"
            )
        if (
            len(extrinsic_residual_std) != 3
            or any(v < 0.0 for v in extrinsic_residual_std)
        ):
            raise ValueError(
                "base_mocap_extrinsic_residual_rpy_std_rad must contain three "
                "non-negative values"
            )
        self._base_mocap_orientation_noise_std = torch.tensor(
            orientation_noise_std, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._base_mocap_extrinsic_residual_std = torch.tensor(
            extrinsic_residual_std, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._base_mocap_max_propagation_s = max_propagation_s
        if not (
            0.0
            <= self._base_mocap_max_propagation_s
            <= self._base_mocap_max_age_s
        ):
            raise ValueError(
                "base_mocap_max_propagation_s must be in "
                "[0, base_mocap_max_age_s]"
            )

        initial_base = self.base_pos_w.clone()
        initial_quat = self.base_quat_w.clone()
        identity_quat = torch.zeros(
            self.num_envs, 4, dtype=torch.float32, device=self.device
        )
        identity_quat[:, 0] = 1.0
        self._actor_base_pos_w = initial_base.clone()
        self._actor_base_quat_w = initial_quat.clone()
        self._base_mocap_extrinsic_residual_quat = identity_quat
        self._base_mocap_extrinsic_residual_rpy = torch.zeros(
            self.num_envs, 3, dtype=torch.float32, device=self.device
        )
        self._base_mocap_residual_needs_sample = torch.full(
            (self.num_envs,),
            not self._ability_curriculum_enabled,
            dtype=torch.bool,
            device=self.device,
        )
        self._actor_base_velocity_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._actor_base_age_s = torch.zeros(self.num_envs, device=self.device)
        self._base_mocap_last_received_pos = initial_base.clone()
        self._base_mocap_steps_since_receive = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._base_mocap_have_previous = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._base_mocap_reset_pending = torch.ones(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        base_buf_len = self._base_mocap_delay_steps + 1
        self._base_mocap_delay_buf = initial_base.unsqueeze(0).repeat(
            base_buf_len, 1, 1
        )
        self._base_mocap_quat_delay_buf = initial_quat.unsqueeze(0).repeat(
            base_buf_len, 1, 1
        )
        self._base_mocap_delay_ptr = 0
        self._base_mocap_step_counter = 0
        self.metrics["base_mocap_age_s"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_mocap_velocity_error"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["base_mocap_stale"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_mocap_orientation_error_rad"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["base_mocap_delay_steps_in_effect"] = (
            self._base_mocap_delay_by_env.float()
        )
        for metric_name in (
            "ability_gate_condition",
            "ability_gate_enough_samples",
            "ability_gate_completion_fh",
            "ability_gate_completion_bh",
            "ability_gate_position_fh",
            "ability_gate_position_bh",
            "ability_gate_composite",
            "ability_gate_post_fall",
            "ability_gate_dwell",
            "ability_unlocked",
            "base_mocap_robustness_scale",
        ):
            self.metrics[metric_name] = torch.zeros(
                self.num_envs, device=self.device
            )
        self.metrics["ability_unlocked"][:] = float(self._ability_unlocked)
        self.metrics["base_mocap_robustness_scale"][:] = float(
            self._base_mocap_robustness_scale
        )
        for metric_name in (
            "pd_nominal_env",
            "pd_kp_multiplier_mean",
            "pd_kd_message_multiplier_mean",
        ):
            self.metrics[metric_name] = torch.zeros(
                self.num_envs, device=self.device
            )
        self._qdes_phase_telemetry_last = {}
        for phase_name in ("hold", "strike", "recovery"):
            for channel in (
                "qdes_step_rms_rad",
                "qdes_second_difference_rms_rad",
                "qdes_reversal_hz",
            ):
                metric_name = f"{channel}_{phase_name}"
                self.metrics[metric_name] = torch.zeros(
                    self.num_envs, device=self.device
                )
                self._qdes_phase_telemetry_last[metric_name] = 0.0

        # --- V15 HUGWBC lower-body command ---------------------------------------------------
        # HITTER continues to choose a world-frame station.  The lower body does not servo that
        # position forever: each new station is converted ONCE into either STAND or a finite number
        # of complete left/right gait cycles.  When those cycles finish the command latches back to
        # STAND until the next station resample.  This removes the reward loop that made V14 keep
        # taking smaller corrective steps after it had already reached the neighbourhood of the
        # station.  The clock/contact construction follows HUGWBC's released implementation.
        self._locomotion_enabled = bool(cfg.locomotion_enabled)
        self._gait_frequency_hz = float(cfg.gait_frequency_hz)
        self._gait_duty_factor = float(cfg.gait_duty_factor)
        self._gait_move_deadband = float(cfg.gait_move_deadband)
        self._gait_step_distance = float(cfg.gait_step_distance)
        self._gait_max_cycles = int(cfg.gait_max_cycles)
        self._gait_velocity_max = float(cfg.gait_velocity_max)
        self._gait_contact_smoothing = float(cfg.gait_contact_smoothing)
        self._locomotion_planned_cycles = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._locomotion_duration_steps = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._locomotion_elapsed_steps = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._locomotion_initial_delta_y = torch.zeros(self.num_envs, device=self.device)
        self._locomotion_velocity_y = torch.zeros(self.num_envs, device=self.device)
        # True only while a planned gait clock is still advancing. Keep this distinct from
        # ``_locomotion_move``: finite_step_bout_v2 deliberately remains in STEP mode during its
        # zero-command settle dwell. The first V16 release used STEP mode as the velocity gate and
        # therefore kept rewarding non-zero lateral speed while simultaneously requiring
        # base_speed <= step_settle_speed_thresh to complete.
        self._locomotion_gait_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._locomotion_move = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._locomotion_supervision = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._gait_clock = torch.zeros(self.num_envs, 2, device=self.device)
        self._desired_contact_states = torch.ones(self.num_envs, 2, device=self.device)
        self._one_step_contract = str(cfg.one_step_contract)
        if self._one_step_contract not in ("disabled", "finite_step_bout_v2"):
            raise ValueError(
                "one_step_contract must be 'disabled' or 'finite_step_bout_v2'"
            )
        self._one_step_enabled = self._one_step_contract == "finite_step_bout_v2"
        self._step_settle_pos_thresh = float(cfg.step_settle_pos_thresh)
        self._step_settle_speed_thresh = float(cfg.step_settle_speed_thresh)
        self._step_settle_yaw_thresh = float(cfg.step_settle_yaw_thresh_rad)
        self._step_settle_contact_force_threshold = float(
            cfg.step_settle_contact_force_threshold
        )
        self._step_settle_slip_thresh = float(cfg.step_settle_slip_thresh)
        self._step_settle_dwell_steps = max(
            int(math.ceil(float(cfg.step_settle_dwell_s) / float(self._env.step_dt))),
            1,
        )
        if self._one_step_enabled and min(
            self._step_settle_pos_thresh,
            self._step_settle_speed_thresh,
            self._step_settle_yaw_thresh,
            self._step_settle_contact_force_threshold,
            self._step_settle_slip_thresh,
            float(cfg.step_settle_dwell_s),
        ) <= 0.0:
            raise ValueError("finite_step_bout_v2 settle thresholds must be positive")

        # Explicit command identity and irreversible bout latch.
        env_index = torch.arange(self.num_envs, dtype=torch.long, device=self.device)
        self._station_command_sequence = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._station_command_id = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        self._station_command_env_index = env_index
        self._step_bout_started = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._step_bout_complete = torch.zeros_like(self._step_bout_started)
        self._step_bout_count = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._step_reentry = torch.zeros_like(self._step_bout_count)
        self._step_control_steps = torch.zeros_like(self._step_bout_count)
        self._locomotion_transition_count = torch.zeros_like(self._step_bout_count)
        self._step_settle_dwell_count = torch.zeros_like(self._step_bout_count)
        self._station_command_age_steps = torch.zeros_like(self._step_bout_count)
        self._station_command_start_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._station_command_target_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._station_distance_bin = torch.zeros_like(self._step_bout_count)
        self._previous_step_mode = torch.zeros_like(self._step_bout_started)
        self._previous_lateral_direction = torch.zeros_like(self._step_bout_count)
        self._previous_foot_contact = torch.ones(
            self.num_envs, 2, dtype=torch.bool, device=self.device
        )
        self._foot_liftoff_count = torch.zeros(
            self.num_envs, 2, dtype=torch.long, device=self.device
        )
        self._foot_touchdown_count = torch.zeros_like(self._foot_liftoff_count)
        self._maximum_station_overshoot = torch.zeros(
            self.num_envs, device=self.device
        )
        self._direction_reversal_count = torch.zeros_like(self._step_bout_count)
        self._foot_slip_distance = torch.zeros(self.num_envs, device=self.device)
        self._step_exit_station_error = torch.full(
            (self.num_envs,), float("nan"), device=self.device
        )
        self._step_exit_base_speed = torch.full_like(
            self._step_exit_station_error, float("nan")
        )
        self._step_exit_yaw_error = torch.full_like(
            self._step_exit_station_error, float("nan")
        )
        self._time_to_stand = torch.full_like(
            self._step_exit_station_error, float("nan")
        )
        self._step_exit_both_feet = torch.zeros_like(self._step_bout_started)
        self._step_exit_slip_speed = torch.full_like(
            self._step_exit_station_error, float("nan")
        )
        self._one_step_release_recorded = torch.zeros_like(self._step_bout_started)
        self._one_step_event = torch.zeros(self.num_envs, device=self.device)
        self._one_step_success_event = torch.zeros(self.num_envs, device=self.device)
        self._one_step_safety_recovery_tag = torch.zeros_like(self._step_bout_count)
        self._one_step_fall_before_swing = torch.zeros_like(
            self._step_bout_started
        )
        terminations_cfg = getattr(self._env.cfg, "terminations", None)
        tilt_term_cfg = getattr(terminations_cfg, "base_fell_tilt", None)
        height_term_cfg = getattr(terminations_cfg, "base_too_low", None)
        tilt_params = getattr(tilt_term_cfg, "params", {}) or {}
        height_params = getattr(height_term_cfg, "params", {}) or {}
        self._one_step_fall_tilt_limit_rad = float(
            tilt_params.get("limit_angle", 0.70)
        )
        self._one_step_fall_height_min_m = float(
            height_params.get("minimum_height", 0.50)
        )
        # Five requested distance buckets: same, (0,.10], (.10,.20], (.20,.30], >.30.
        self._one_step_command_count = torch.zeros(5, device=self.device)
        self._one_step_attempt_count = torch.zeros(5, device=self.device)
        self._one_step_success_count = torch.zeros(5, device=self.device)
        self._one_step_reentry_count = torch.zeros(5, device=self.device)
        self._one_step_fall_count = torch.zeros(5, device=self.device)
        # End-to-end funnel for the user's actual task question:
        #   one planned lateral bout -> settled release -> strict READY -> exact strike
        #   -> composite hit -> strict recovery before the next swing.
        # All numerators are cumulative conjunctions with the previous stage and all rates use
        # planned one-step commands as their denominator. These buffers are telemetry-only.
        self._chain_shape = (2, 5)
        self._chain_attempt_count = torch.zeros(
            self._chain_shape, device=self.device
        )
        self._chain_step_settled_count = torch.zeros_like(
            self._chain_attempt_count
        )
        self._chain_ready_release_count = torch.zeros_like(
            self._chain_attempt_count
        )
        self._chain_exact_frame_count = torch.zeros_like(
            self._chain_attempt_count
        )
        self._chain_exact_hit_count = torch.zeros_like(
            self._chain_attempt_count
        )
        self._chain_safe_recovery_count = torch.zeros_like(
            self._chain_attempt_count
        )
        self._chain_clip_id = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._chain_step_ok = torch.zeros_like(self._step_bout_started)
        self._chain_ready_ok = torch.zeros_like(self._step_bout_started)
        self._chain_released = torch.zeros_like(self._step_bout_started)
        self._chain_exact_recorded = torch.zeros_like(self._step_bout_started)
        self._chain_recovery_pending = torch.zeros_like(
            self._step_bout_started
        )
        self._chain_recovery_clip_id = torch.zeros_like(self._chain_clip_id)
        self._chain_recovery_distance_bin = torch.zeros_like(
            self._station_distance_bin
        )
        self._one_step_metric_sum = {
            label: torch.zeros(5, device=self.device)
            for label in (
                "step_bout_count",
                "total_step_control_steps",
                "locomotion_mode_transition_count",
                "left_foot_liftoff_count",
                "right_foot_liftoff_count",
                "left_foot_touchdown_count",
                "right_foot_touchdown_count",
                "base_y_displacement",
                "base_x_drift",
                "maximum_station_overshoot",
                "direction_reversal_count",
                "foot_slip_distance",
                "station_error_at_step_exit",
                "base_speed_at_step_exit",
                "yaw_error_at_step_exit",
                "time_from_command_to_stand",
            )
        }
        self._feet_in_contact = torch.ones(
            self.num_envs, 2, dtype=torch.bool, device=self.device
        )
        self._foot_slip_speed_per_foot = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        # One-step contact-rising-edge pulse for soft-landing shaping.  It is
        # reward/telemetry state only and never enters the 110-D actor input.
        self._foot_touchdown_downspeed = torch.zeros(
            self.num_envs, 2, device=self.device
        )

        # HUGWBC raises/lowers its upper-body intervention curriculum from locomotion tracking at
        # command resampling.  Keep the same closed-loop rule here; the action term consumes this
        # strength but owns the actual training-only action replacement.
        self._intervention_curriculum_start = float(cfg.intervention_curriculum_start)
        self._intervention_curriculum_step = float(cfg.intervention_curriculum_step)
        self._intervention_tracking_pass = float(cfg.intervention_tracking_pass)
        self._intervention_tracking_fail = float(cfg.intervention_tracking_fail)
        self._intervention_tracking_sigma = float(cfg.intervention_tracking_sigma)
        self._intervention_strength = torch.full(
            (self.num_envs,), self._intervention_curriculum_start, device=self.device
        )
        self._locomotion_tracking_sum = torch.zeros(self.num_envs, device=self.device)
        self._locomotion_tracking_count = torch.zeros(self.num_envs, device=self.device)
        # XY station error captured ONCE per finite command — at plan time for STAND and at gait
        # completion for STEP.  The stand-phase station score reads this latch instead of the live
        # error, so post-latch drift neither re-arms corrective micro-stepping nor can be shuffled
        # back for income (audit 2026-07-23: the live binary stand score paid 1.0/step for
        # re-entering the deadband, which is exactly the shuffle incentive V15 forbids).
        self._finite_station_latched_error = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_move_mode"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_gait_active"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["locomotion_velocity_y_cmd"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_gait_phase"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_cycles_planned"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_initial_delta_y"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["locomotion_tracking_exp"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["upper_intervention_strength"] = self._intervention_strength.clone()
        self.metrics["upper_intervention_active"] = torch.zeros(
            self.num_envs, device=self.device
        )

        # Strike timing / gating.
        self.time_to_strike = torch.zeros(self.num_envs, device=self.device)
        self.pre_strike = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        self._post_strike_capture_delays_s = tuple(
            float(value) for value in cfg.post_strike_capture_delays_s
        )
        if self._post_strike_capture_delays_s:
            if float(cfg.post_strike_capture_delay_s) > 0.0:
                raise ValueError(
                    "configure either post_strike_capture_delay_s (legacy) or "
                    "post_strike_capture_delays_s (stratified), not both"
                )
            if any(
                (not math.isfinite(value) or value <= 0.0)
                for value in self._post_strike_capture_delays_s
            ):
                raise ValueError(
                    "post_strike_capture_delays_s must contain finite positive delays"
                )
            if tuple(sorted(self._post_strike_capture_delays_s)) != (
                self._post_strike_capture_delays_s
            ) or len(set(self._post_strike_capture_delays_s)) != len(
                self._post_strike_capture_delays_s
            ):
                raise ValueError(
                    "post_strike_capture_delays_s must be strictly increasing"
                )
        # Previous-tick tts for the once-per-swing hot follow-through capture edge detector.
        # Initialized far positive so a fresh env can never fire a spurious crossing.
        self._post_strike_capture_prev_tts = torch.full(
            (self.num_envs,), 1.0e9, device=self.device
        )
        # Risk edges use the same three hot phases plus a late pre-wrap phase. Both clips have at
        # least 1.20 s from strike to wrap; use 1.20 s as the wrap-phase center, giving exact
        # Voronoi boundaries 0.19/0.55/1.00 s for the recipe's 0.08/0.30/0.80 s hot centers.
        risk_phase_centers = self._post_strike_capture_delays_s
        if risk_phase_centers:
            risk_phase_centers = risk_phase_centers + (
                max(risk_phase_centers[-1] + 0.40, 1.20),
            )
        self._post_strike_capture_midpoints_s = tuple(
            0.5 * (left + right)
            for left, right in zip(
                risk_phase_centers[:-1],
                risk_phase_centers[1:],
            )
        )
        self._post_strike_replay_phase_count = len(risk_phase_centers)
        # One legal warning/near-boundary snapshot per swing, phase, and severity. The mask is
        # part of the Markov replay state so replaying a hot state cannot repeatedly reinsert the
        # same state on every following tick.
        self._post_strike_risk_capture_mask = torch.zeros(
            self.num_envs,
            self._post_strike_replay_phase_count,
            3,
            dtype=torch.bool,
            device=self.device,
        )
        self.strike_window = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        for metric_name in (
            "recovery_curriculum_stage",
            "recovery_curriculum_scale",
            "recovery_curriculum_target_scale",
            "recovery_coverage_scale",
            "recovery_coverage_target_scale",
            "recovery_stage1_coverage_unlocked",
            "recovery_stage1_acquisition_rung",
            "recovery_stage1_acquisition_rung_count",
            "recovery_stage1_acquisition_ready_threshold",
            "recovery_stage1_acquisition_failures",
            "recovery_stage1_acquisition_backoff_event",
            "recovery_ready_profile_progress",
            "recovery_ready_effective_heading_thresh_rad",
            "recovery_ready_effective_yaw_rate_thresh",
            "recovery_ready_effective_tilt_thresh",
            "recovery_ready_effective_joint_speed_thresh",
            "recovery_ready_effective_foot_slip_thresh",
            "recovery_ready_effective_dwell_ticks",
            "recovery_stage1_ready_dwell",
            "recovery_stage1_acquisition_age",
            "recovery_stage1_acquisition_failed",
            "recovery_curriculum_block_reason",
            "recovery_bootstrap_block_reason",
            "recovery_acquisition_block_reason",
            "recovery_stage2_block_reason",
            "recovery_stage1_enter_ok",
            "recovery_stage1_ready_ok",
            "recovery_stage1_safety_exit_bad",
            "recovery_stage1_exit_bad",
            "recovery_stage2_enter_ok",
            "recovery_stage2_exit_bad",
            "recovery_stage1_enter_dwell",
            "recovery_stage1_exit_dwell",
            "recovery_stage2_enter_dwell",
            "recovery_stage2_exit_dwell",
            "recovery_gate_completion",
            "recovery_gate_completion_fh",
            "recovery_gate_completion_bh",
            "recovery_gate_release_eligible_completion_fh",
            "recovery_gate_release_eligible_completion_bh",
            "recovery_gate_ready_timeout_rate_fh",
            "recovery_gate_ready_timeout_rate_bh",
            "recovery_gate_position_fh",
            "recovery_gate_position_bh",
            "recovery_gate_velocity_fh",
            "recovery_gate_velocity_bh",
            "recovery_gate_normal_fh",
            "recovery_gate_normal_bh",
            "recovery_gate_composite_fh",
            "recovery_gate_composite_bh",
            "recovery_gate_ready_fh",
            "recovery_gate_ready_bh",
            "recovery_gate_safe_recovery_fh",
            "recovery_gate_safe_recovery_bh",
            "recovery_gate_virtual_contact_fh",
            "recovery_gate_virtual_contact_bh",
            "recovery_gate_virtual_over_net_fh",
            "recovery_gate_virtual_over_net_bh",
            "recovery_gate_virtual_legal_fh",
            "recovery_gate_virtual_legal_bh",
            "recovery_gate_actual_q_fault_fh",
            "recovery_gate_actual_q_fault_bh",
            "recovery_gate_actual_q_fault_events_fh",
            "recovery_gate_actual_q_fault_events_bh",
            "recovery_gate_actual_q_window_steps",
            "recovery_gate_actual_q_window_starts_fh",
            "recovery_gate_actual_q_window_starts_bh",
            "recovery_gate_actual_q_window_ready",
            "recovery_gate_virtual_samples_fh",
            "recovery_gate_virtual_samples_bh",
            "recovery_gate_post_fall",
            "recovery_gate_post_fall_fh",
            "recovery_gate_post_fall_bh",
            "post_strike_hot_capture",
            "post_strike_risk_capture",
            "post_strike_risk_capture_warning",
            "post_strike_risk_capture_near",
        ):
            self.metrics[metric_name] = torch.zeros(
                self.num_envs, device=self.device
            )
        for phase_index in range(len(self._post_strike_capture_delays_s)):
            self.metrics[f"post_strike_hot_capture_phase_{phase_index}"] = (
                torch.zeros(self.num_envs, device=self.device)
            )
        for phase_index in range(self._post_strike_replay_phase_count):
            self.metrics[
                f"post_strike_risk_capture_phase_{phase_index}"
            ] = torch.zeros(self.num_envs, device=self.device)

        # Episode-wide tracking errors (instantaneous; averaged over terminating envs at reset).
        self.metrics["racket_pos_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_vel_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["adaptive_sigma_pos"] = torch.full((self.num_envs,), float(cfg.sigma_pos_max), device=self.device)
        self.metrics["adaptive_sigma_vel"] = torch.full((self.num_envs,), float(cfg.sigma_vel_max), device=self.device)
        self.metrics["adaptive_sigma_normal"] = torch.full(
            (self.num_envs,), float(cfg.sigma_normal_max), device=self.device
        )
        # Window-mean face error in radians -- the operating point sigma_normal must track.
        self.metrics["racket_normal_error_rad_window_mean"] = torch.zeros(self.num_envs, device=self.device)
        # Live racket_velocity weight and auditable sub-gates (0 until first compute).
        self.metrics["racket_velocity_weight_live"] = torch.zeros(self.num_envs, device=self.device)
        # Stable public names requested by the V15 fresh-train audit.  block_reason is a bitmask:
        # staged-hysteresis v2 uses 1/2=FH/BH position, 4/8=FH/BH normal,
        # 16=post-strike fall, 32=READY, 64=sample count, 128/256=FH/BH velocity.
        # Zero means the next stage's metric gate is healthy.
        for metric_name in (
            "velocity_weight_current",
            "velocity_gate_stable_steps",
            "velocity_gate_block_reason",
            "velocity_gate_position_fh",
            "velocity_gate_position_bh",
            "velocity_gate_velocity_fh",
            "velocity_gate_velocity_bh",
            "velocity_gate_normal_fh",
            "velocity_gate_normal_bh",
            "velocity_gate_post_fall",
            "velocity_gate_ready",
            "position_sigma_current",
            "velocity_stage",
            "velocity_target_weight",
            "velocity_current_weight",
            "stage1_enter_ok",
            "stage1_exit_bad",
            "stage1_enter_dwell",
            "stage1_exit_dwell",
            "stage2_enter_ok",
            "stage2_exit_bad",
            "stage2_enter_dwell",
            "stage2_exit_dwell",
            "stage2_weight_ready",
            "velocity_exact_sample_count_fh",
            "velocity_exact_sample_count_bh",
            "velocity_fall_sample_count",
            "velocity_blocked_reason_position_fh",
            "velocity_blocked_reason_position_bh",
            "velocity_blocked_reason_velocity_fh",
            "velocity_blocked_reason_velocity_bh",
            "velocity_blocked_reason_normal_fh",
            "velocity_blocked_reason_normal_bh",
            "velocity_blocked_reason_post_fall",
            "velocity_blocked_reason_ready",
            "velocity_blocked_reason_sample_count",
        ):
            self.metrics[metric_name] = torch.zeros(self.num_envs, device=self.device)
        for stage in (1, 2):
            for condition_name in (
                "position_fh_ok",
                "position_bh_ok",
                "velocity_fh_ok",
                "velocity_bh_ok",
                "normal_fh_ok",
                "normal_bh_ok",
                "post_fall_ok",
                "ready_ok",
                "sample_count_ok",
            ):
                self.metrics[f"stage{stage}_{condition_name}"] = torch.zeros(
                    self.num_envs, device=self.device
                )
        for metric_name in (
            "racket_velocity_gate_normal_ok",
            "racket_velocity_gate_position_ok",
            "racket_velocity_gate_velocity_ok",
            "racket_velocity_gate_post_fall_ok",
            "racket_velocity_gate_ready_ok",
            "racket_velocity_gate_all_ok",
            "racket_velocity_gate_dwell_fraction",
            "racket_velocity_gate_progress",
        ):
            self.metrics[metric_name] = torch.zeros(self.num_envs, device=self.device)
        for metric_name in (
            "position_guidance_active",
            "position_guidance_temporal_scale",
            "exact_position_debt_active",
            "exact_static_position_error",
            "exact_position_debt_raw",
            "exact_position_debt_weighted",
            "position_guidance_event_return",
            "exact_position_debt_event_return",
        ):
            self.metrics[metric_name] = torch.zeros(
                self.num_envs, device=self.device
            )
        for suffix in (
            "forehand",
            "backhand",
            "core",
            "planner",
            "q1",
            "q2",
            "q3",
            "q4",
        ):
            for metric_name in (
                "position_guidance_active",
                "position_guidance_temporal_scale",
                "exact_position_debt_active",
                "exact_static_position_error",
                "exact_position_debt_raw",
                "exact_position_debt_weighted",
                "position_guidance_event_return",
                "exact_position_debt_event_return",
            ):
                self.metrics[f"{metric_name}_{suffix}"] = torch.zeros(
                    self.num_envs, device=self.device
                )
        self.metrics["racket_normal_error_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_pos_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_target_source"] = torch.full(
            (self.num_envs,), -1.0, device=self.device
        )
        self.metrics["strike_target_speed_quartile"] = torch.full(
            (self.num_envs,), -1.0, device=self.device
        )
        self.metrics["strike_target_z_bin"] = torch.full(
            (self.num_envs,), -1.0, device=self.device
        )
        # How far the (coupled) base target sits from spawn — i.e. how much repositioning is commanded.
        # ~0 when base_couple_blend=0; grows with the coupling toward far racket targets.
        self.metrics["base_target_offset_norm"] = torch.zeros(self.num_envs, device=self.device)
        # Absolute commanded station-y change at each intra-episode wrap. RallyFinal constrains this
        # to station_y_step_range (20-35 cm); plain HitterPure reports its independent-draw change.
        # It is zero only on true resets, where there is no prior rally station to compare against.
        self.metrics["station_y_step_command"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["station_y_step_signed_command"] = torch.zeros(
            self.num_envs, device=self.device
        )
        # FinalV3 mixed transition class: 0=same station, 1=small adjustment, 2=main
        # lateral step, -1=true reset/no wrap.  Diagnostic only; it never enters the actor.
        self.metrics["station_y_step_class"] = torch.full(
            (self.num_envs,), -1.0, device=self.device
        )
        # Strike-window metrics: hold the value from the MOST RECENT strike — these map directly to
        # the acceptance criteria (racket pos < 7.5 cm, vel < 0.5 m/s, normal < 15 deg AT strike) and
        # are the real "is the policy learning to hit" signal (the episode-wide ones above are diluted
        # by the long non-strike portion of each swing).
        self.metrics["racket_pos_error_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_vel_error_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_normal_error_deg_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_success"] = torch.zeros(self.num_envs, device=self.device)
        # Exact-strike metrics: sampled only on the nearest control frame to the configured strike
        # step. These avoid the "within-window" dilution from the +/- strike reward window.
        self.metrics["racket_pos_error_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_vel_error_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_normal_error_deg_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_composite_success_exact"] = torch.zeros(self.num_envs, device=self.device)
        # Per-axis position error AT the exact strike frame (which axis is the miss?).
        self.metrics["racket_pos_error_x_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_pos_error_y_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_pos_error_z_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["exact_strike_hit_rate"] = torch.zeros(self.num_envs, device=self.device)
        # Conditional exact-strike pass rates (broadcast scalar; the trustworthy success signal). Each is
        # the fraction of exact-strike samples that clear that acceptance threshold, undiluted by the
        # non-strike steps that wrecked the old held metric. strike_composite_success_exact requires all
        # three (pos & vel & normal) and drives the success-gated perturbation curriculum.
        self.metrics["strike_pos_pass_exact"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_vel_pass_exact"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_normal_pass_exact"] = torch.zeros(self.num_envs, device=self.device)
        # Position-accuracy buckets + error distribution on the exact-strike sample (comparable w/ composite).
        self.metrics["exact_strike_pos_success_5cm"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["exact_strike_pos_success_10cm"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["exact_strike_pos_err_mean"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["exact_strike_pos_err_p90"] = torch.zeros(self.num_envs, device=self.device)
        # Per-clip (forehand/backhand) versions of the exact-strike pass rates + errors (multiseg only;
        # stay 0 for a single-clip run). Updated in _update_metrics.
        for _cname in self._clip_names.values():
            for _key in (
                "strike_pos_pass_exact", "strike_vel_pass_exact", "strike_normal_pass_exact",
                "strike_composite_success_exact", "racket_pos_error_exact_strike",
                "racket_vel_error_exact_strike", "racket_normal_error_deg_exact_strike",
            ):
                self.metrics[f"{_key}_{_cname}"] = torch.zeros(self.num_envs, device=self.device)
            for _axis in ("x", "y", "z"):
                self.metrics[f"racket_pos_signed_error_{_axis}_exact_strike_{_cname}"] = torch.zeros(
                    self.num_envs, device=self.device
                )
                self.metrics[f"racket_vel_signed_error_{_axis}_exact_strike_{_cname}"] = torch.zeros(
                    self.num_envs, device=self.device
                )
            self.metrics[f"racket_normal_dot_exact_strike_{_cname}"] = torch.zeros(
                self.num_envs, device=self.device
            )
            for _cohort in self._cohort_names:
                for _key in (
                    "strike_pos_pass_exact",
                    "strike_vel_pass_exact",
                    "strike_composite_success_exact",
                ):
                    self.metrics[f"{_cohort}_{_key}_{_cname}"] = torch.zeros(
                        self.num_envs, device=self.device
                    )
        self.metrics["exact_strike_sample_count_decayed"] = torch.zeros(self.num_envs, device=self.device)
        # Tier-1 virtual-ball outcome rates (broadcast sample-weighted EMAs, exact-strike denominator
        # for hit rate; hit (captured) denominator for the outcome rates). Only logged when enabled.
        if cfg.virtual_ball:
            for _vk in (
                "virtual_hit_rate", "virtual_net_clear_rate", "virtual_land_valid_rate",
                "virtual_land_inbounds_rate", "virtual_land_err_m", "virtual_topspin_revs",
                "virtual_approach_speed",
            ):
                self.metrics[_vk] = torch.zeros(self.num_envs, device=self.device)
            for _side in self._clip_names.values():
                for _vk in (
                    "virtual_sample_count",
                    "virtual_contact_rate",
                    "virtual_over_net_rate",
                    "virtual_legal_rate",
                    "virtual_landing_error_m",
                ):
                    self.metrics[f"{_vk}_{_side}"] = torch.zeros(
                        self.num_envs, device=self.device
                    )
        self.metrics["venue_tuple_selected"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["venue_tuple_mix_effective"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["venue_tuple_accept_rate"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["venue_tuple_fallback_rate"] = torch.zeros(
            self.num_envs, device=self.device
        )
        # UNCONDITIONAL swing accounting (Phase A): completion_rate = exact-strike arrivals / swing
        # STARTS (falls count against it, unlike the conditional composite above); fall rate before
        # the strike frame. Broadcast scalars like the pass rates.
        self.metrics["swing_completion_rate"] = torch.zeros(self.num_envs, device=self.device)
        for _cname in self._clip_names.values():
            self.metrics[f"swing_completion_rate_{_cname}"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["pre_strike_fall_rate"] = torch.zeros(self.num_envs, device=self.device)
        # Post-strike (follow-through/recovery) falls + per-clip fall attribution: the multi-swing
        # episode's real recovery signal. pre_strike_fall_rate alone hides a policy that hits and THEN
        # falls (100% completion, 0% pre-strike falls — the actual backhand deploy failure signature).
        self.metrics["post_strike_fall_rate"] = torch.zeros(self.num_envs, device=self.device)
        for _cname in self._clip_names.values():
            self.metrics[f"pre_strike_fall_rate_{_cname}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"post_strike_fall_rate_{_cname}"] = torch.zeros(self.num_envs, device=self.device)
        # HER-style achieved-target replay diagnostics: fraction of resampled targets drawn from the
        # achieved buffer (EMA; ~achieved_target_mix_prob once the buffers are filled) + per-clip fill.
        self.metrics["achieved_replay_frac"] = torch.zeros(self.num_envs, device=self.device)
        for _cname in self._clip_names.values():
            self.metrics[f"achieved_buffer_fill_{_cname}"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_window_hit_rate"] = torch.zeros(self.num_envs, device=self.device)
        # Base-position error while the base target is active (pre-strike), held at its last value.
        self.metrics["base_pos_error_pre_strike"] = torch.zeros(self.num_envs, device=self.device)
        # Planar base SPEED held at the strike window (= base speed AT contact). Unlike
        # base_speed_xy_prestrike (instantaneous approach speed, zeroed off pre_strike), this HOLDS the
        # strike-frame value, so low = the base actually SETTLED by the time it struck (the "arrive → calm
        # → strike" evidence). Pairs with base_pos_error_pre_strike (= AT the station by strike time).
        self.metrics["base_speed_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        # RallyFinal phase-specific stability metrics.  The exact-strike variants are held at the
        # contact frame; post-swing values are held over the configured recovery window.
        self.metrics["base_x_error_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_x_error_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_x_velocity_abs_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["station_y_error_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["station_y_error_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["pre_strike_base_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_speed_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_base_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_leg_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_foot_slip"] = torch.zeros(self.num_envs, device=self.device)
        # Stability-first recovery telemetry.  These are phase-held diagnostics rather than
        # rewards, so every Rally task exposes the same evidence even when a particular shaping
        # term is absent.  The deterministic evaluator reduces the instantaneous signals over
        # the configured post-swing window and uses them as fail-closed checkpoint gates.
        self.metrics["post_swing_base_tilt_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_root_height_m"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_foot_contact_frac"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["backhand_left_hand_min_distance"] = torch.ones(self.num_envs, device=self.device)
        self.metrics["backhand_left_forearm_min_distance"] = torch.ones(self.num_envs, device=self.device)
        self.metrics["backhand_left_arm_min_distance"] = torch.ones(self.num_envs, device=self.device)
        self.metrics["base_heading_error_deg_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["torso_heading_error_deg_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["torso_yaw_rate_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        # RallyFinal heading recovery telemetry.  Exact-strike heading alone hid a policy that was
        # square at contact but yawed in the runner's frozen-ready state and over the clip tail.
        self.metrics["ready_base_heading_error_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_base_yaw_rate_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_base_heading_error_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_base_yaw_rate_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rally_success_run"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rally_success_run_max"] = torch.zeros(self.num_envs, device=self.device)
        # Arrival-gated hold: EXTRA hold steps spent waiting for the base to settle (0 if the feature is
        # off or the base arrived within the base countdown). High mean = stations often need extension
        # (far / slow recovery); hitting hold_settle_max_extra_steps = the cap fired (unreachable/unstable).
        self.metrics["hold_extra_steps"] = torch.zeros(self.num_envs, device=self.device)
        # Fixed-clock ready diagnostics. V2 defaults mirror the C++ position/speed/dwell
        # gate; V3 additionally requires the tighter front-facing launch heading.
        # The cumulative release rate/latency count only 20-35 cm intra-episode wrap transitions;
        # easy absolute draws at true reset cannot inflate the readiness gate.
        self.metrics["ready_station_x_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_y_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_base_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_heading_error_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_yaw_rate_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_tilt"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_joint_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_foot_slip"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_station_hold_age_s"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_position_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_speed_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_heading_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_yaw_rate_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_tilt_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_joint_speed_ok"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_foot_slip_ok"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_station_latched"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_transition_eligible"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_newly_ready"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_latency_s"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_x_error_at_ready"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_y_error_at_ready"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_base_speed_at_ready"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_heading_error_at_ready_deg"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_station_release_event"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_release_pass"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_release_rate"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_station_release_rate_ema"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_station_latency_mean_s"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ready_release_required"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_release_wait_s"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["ready_release_timeout_event"] = torch.zeros(
            self.num_envs, device=self.device
        )
        # Runtime static-controller -> policy handoff audit.  These are conditional cumulative
        # statistics over only the 5% handoff subset; broadcasting the scalar estimates prevents
        # W&B's ordinary per-env mean from diluting them by the 95% non-handoff population.
        self._runtime_handoff_prev_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._runtime_handoff_entry_count = torch.zeros((), device=self.device)
        self._runtime_handoff_entry_qdes_step_peak = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_steady_element_count = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_steady_sample_count = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_qdes_step_sq_sum = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_qdes_second_sq_sum = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_tracking_sq_sum = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_projection_sq_sum = torch.zeros(
            (), device=self.device
        )
        self._runtime_handoff_reversal_count = torch.zeros(
            (), device=self.device
        )
        for _name in (
            "runtime_handoff_entry_count",
            "runtime_handoff_steady_sample_count",
            "runtime_handoff_entry_qdes_step_peak_rad",
            "runtime_handoff_steady_qdes_step_rms_rad",
            "runtime_handoff_qdes_second_difference_rms_rad",
            "runtime_handoff_reversal_hz",
            "runtime_handoff_tracking_error_rms_rad",
            "runtime_handoff_projection_distance_raw_rms",
            "runtime_handoff_current_entry_count",
            "runtime_handoff_current_steady_sample_count",
            "runtime_handoff_current_entry_qdes_step_peak_rad",
            "runtime_handoff_current_steady_qdes_step_rms_rad",
            "runtime_handoff_current_qdes_second_difference_rms_rad",
            "runtime_handoff_current_reversal_hz",
            "runtime_handoff_current_tracking_error_rms_rad",
            "runtime_handoff_current_projection_distance_raw_rms",
        ):
            self.metrics[_name] = torch.zeros(
                self.num_envs, device=self.device
            )
        for _side in self._clip_names.values():
            for _name in (
                "ready_release_rate",
                "safe_recovery_rate",
                "actual_q_fault_rate",
            ):
                self.metrics[f"{_name}_{_side}"] = torch.zeros(
                    self.num_envs, device=self.device
                )
        # External arm-deadline gate telemetry.  A V2Plus-only stateful termination term writes
        # these buffers before reset; every other task leaves them at zero.  The term owns release
        # semantics because termination is evaluated before command clocks advance in Isaac Lab.
        for _name in (
            "arm_deadline_event", "arm_deadline_pass", "arm_deadline_miss",
            "arm_deadline_pass_rate", "arm_deadline_x_error", "arm_deadline_y_error",
            "arm_deadline_base_speed", "arm_deadline_heading_error_deg",
            "arm_deadline_yaw_rate_abs", "arm_deadline_tilt", "arm_deadline_joint_speed",
            "arm_deadline_dwell_s", "arm_deadline_position_ok", "arm_deadline_speed_ok",
            "arm_deadline_heading_ok", "arm_deadline_yaw_rate_ok", "arm_deadline_tilt_ok",
            "arm_deadline_joint_speed_ok", "arm_deadline_dwell_ok",
            "arm_deadline_miss_rate_reset", "arm_deadline_miss_rate_same",
            "arm_deadline_miss_rate_small", "arm_deadline_miss_rate_main",
            "arm_deadline_miss_rate_forehand", "arm_deadline_miss_rate_backhand",
        ):
            self.metrics[_name] = torch.zeros(self.num_envs, device=self.device)
        # Station-relative reach is the planner's arm-vs-footwork decision variable:
        # target_y = station_y + reach_y.  Log it explicitly so reach envelopes can be audited.
        self.metrics["racket_reach_y_command"] = torch.zeros(self.num_envs, device=self.device)
        # Rally drift metrics (2026-07-07): EMA-broadcast per-swing drift + per-step recovery signals.
        self.metrics["base_drift_per_swing"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_drift_fwd_per_swing"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_station_offset_at_swing_start"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_heading_abs_at_swing_start"] = torch.zeros(self.num_envs, device=self.device)
        # v3 rally recovery (2026-07-08): spawn-yaw-CONDITIONED recovery pair (see __init__). Only
        # holds that START yawed (>_RECOV_SPAWN_YAW_THRESH) count; expiry is the load-bearing gate.
        self.metrics["heading_recovery_spawn_yaw"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["heading_recovery_expiry_yaw"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_strike_base_speed_xy"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_dist_from_origin"] = torch.zeros(self.num_envs, device=self.device)
        # Swing-quality detail at the most recent strike: actual paddle speed, per-axis position error,
        # and success at tighter/looser thresholds (5 cm / 10 cm) for a fuller accuracy distribution.
        self.metrics["racket_speed_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_speed_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_target_speed_exact_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_target_speed_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_pos_error_x_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_pos_error_y_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["racket_pos_error_z_at_strike"] = torch.zeros(self.num_envs, device=self.device)
        # DEPRECATED semantics: these hold the value at the WINDOW-EXIT frame (racket ~0.26 m past target),
        # NOT at contact, and use the diluting reset-mean denominator. Renamed so they stop reading as
        # "success". Use exact_strike_pos_success_5cm/10cm above for the real contact-frame accuracy.
        self.metrics["strike_success_5cm_window_exit"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["strike_success_10cm_window_exit"] = torch.zeros(self.num_envs, device=self.device)
        # Robot-health diagnostics (episode-wide, instantaneous) — logged here because this term already
        # holds ``self.robot``. Useful for sim2real: standing height, peak joint speed, actuator effort.
        self.metrics["base_height"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_upright"] = torch.zeros(self.num_envs, device=self.device)
        # Stability diagnostics (instability shows up here BEFORE a termination): absolute base roll/pitch
        # (deg; 0 = level — base_upright only gives the combined tilt), and foot contact + slip (a planted
        # foot should be ~still; horizontal foot speed while in contact = slip = the robot shuffling/sliding).
        self.metrics["base_roll_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["base_pitch_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["foot_contact_frac"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["foot_slip_speed"] = torch.zeros(self.num_envs, device=self.device)
        # Foot-slip magnitude (sum over feet of horizontal speed WHILE in contact) used by the
        # pre_strike_foot_slip reward. Recomputed each step in _update_metrics; stays 0 if the feet /
        # contact sensor cannot be resolved, so the reward is a safe no-op in that case.
        self.foot_slip_in_contact = torch.zeros(self.num_envs, device=self.device)
        # Fraction of feet in contact (mean over the 2 feet): 1.0 = both planted, 0.5 = one foot
        # unloaded, 0.0 = airborne. Clean attribute for the feet_contact stance reward (real_sensor
        # variant). Same value as metrics["foot_contact_frac"]; 0 (safe) if the sensor cannot resolve.
        self.feet_contact_frac = torch.zeros(self.num_envs, device=self.device)
        # ---- footwork-to-strike signals (base-FREE; reward/metric only, NEVER in the observation) ----
        # racket_target_distance = ||racket_FK - racket_target|| (frame-invariant, no base position).
        # racket_progress = prev_distance - current_distance (>0 = the WHOLE body moved the racket closer
        # to the target). This dense progress term is what drives footwork WITHOUT a base target.
        z = lambda: torch.zeros(self.num_envs, device=self.device)  # noqa: E731
        self.racket_target_distance = z()
        self.racket_progress = z()
        self._prev_racket_dist = z()
        self._progress_reset_mask = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        self.foot_slip_sq = z()  # sum_feet contact * ||foot_xy_vel||^2
        self.foot_vel_sq = z()  # sum_feet ||foot_vel||^2 (excessive/violent foot motion)
        self.foot_drag = z()  # sum_feet ||foot_xy_vel|| while the foot is LOW (near ground -> dragging)
        self.arm_overreach_frac = z()  # fraction of ARM joints within 10% of a position limit
        self.waist_twist = z()  # |waist_yaw - default| + |waist_roll - default| (anti twist-instead-of-step)
        self.proj_grav_xy = z()  # ||projected_gravity_xy|| = base tilt (strike-window stability)
        self.base_ang_vel_xy_norm = z()  # ||base_ang_vel_xy|| (strike-window stability)
        self.vertical_speed = z()  # |base_lin_vel_z| (vertical bob)
        self._drag_height = 0.10  # m: foot below this counts as "near ground" for the drag penalty
        self.metrics["joint_vel_abs_max"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["time_to_strike_s"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["pre_strike_flag"] = torch.zeros(self.num_envs, device=self.device)
        # P2.4 watch-metric: planar |v_base| during the approach (the base_decel_tracking reward's
        # subject). 0 outside pre_strike -> the reset-mean dilutes like the other *_prestrike metrics.
        self.metrics["base_speed_xy_prestrike"] = torch.zeros(self.num_envs, device=self.device)
        # Curriculum perturbation scale (reference_perturbed mode): 0 at start ramping to 1; lets you
        # watch the reachable target ball widen in wandb. Stays 0 in "uniform" mode.
        self.metrics["ref_perturb_scale"] = torch.zeros(self.num_envs, device=self.device)
        self._has_jpos_limits = hasattr(self.robot.data, "soft_joint_pos_limits") or hasattr(
            self.robot.data, "joint_pos_limits"
        )
        if self._has_jpos_limits:
            self.metrics["joint_pos_near_limit_frac"] = torch.zeros(self.num_envs, device=self.device)
        self._has_torque = hasattr(self.robot.data, "applied_torque")
        if self._has_torque:
            self.metrics["joint_torque_abs_mean"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics["joint_torque_abs_max"] = torch.zeros(self.num_envs, device=self.device)
        # Policy action magnitude (saturation check for sim2real).
        self.metrics["action_abs_mean"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["action_abs_max"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["action_delta_abs_mean"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["action_delta_abs_max"] = torch.zeros(self.num_envs, device=self.device)
        for axis in ("x", "y"):
            self.metrics[f"base_pos_{axis}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"base_pos_error_{axis}"] = torch.zeros(self.num_envs, device=self.device)
        for axis in ("x", "y", "z"):
            self.metrics[f"racket_pos_error_{axis}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"racket_vel_error_{axis}"] = torch.zeros(self.num_envs, device=self.device)

        # Metrics-only physical ball/table instrument from cat_stable.  It realizes the exact
        # sampled virtual-ball question in PhysX but never feeds observations, rewards or target
        # generation.  With the default flag off it creates no scene driver, metrics or RNG use.
        self._physical = None
        if not cfg.physical_ball and (
            bool(cfg.physical_ball_impulse) or int(cfg.physical_ball_substep) != 1
        ):
            raise ValueError(
                "physical_ball_impulse/physical_ball_substep require physical_ball=True"
            )
        if cfg.physical_ball:
            if not cfg.virtual_ball:
                raise ValueError(
                    "physical_ball=True requires virtual_ball=True because the physical serve "
                    "realizes the sampled incoming-ball question"
                )
            if float(cfg.midswing_resample_prob) > 0.0:
                raise ValueError(
                    "physical_ball cannot be combined with midswing_resample_prob > 0: the "
                    "served ball would realize a stale question"
                )
            from whole_body_tracking.tasks.tracking.mdp.physical_ball import PhysicalBallManager

            self._physical = PhysicalBallManager(self, env)

        # --- DEBUG: swing-through sign check + raw/gated reward kernels (cfg.debug_reward_logging) ---
        # err_minus uses the CURRENT (correct) swing-through form target - vel*t_to_strike; err_plus uses
        # the FLIPPED form target + vel*t_to_strike. In-window we expect err_minus < err_plus (sign OK) and
        # at the exact strike (t_to_strike~0) the two collapse together. raw/gated are written by the reward
        # terms in hope_rewards.py. All held over the relevant mask so the reset-mean is the in-window value.
        if self.cfg.debug_reward_logging:
            for _k in (
                "dbg_err_minus_win", "dbg_err_plus_win", "dbg_err_minus_exact", "dbg_err_plus_exact",
                "dbg_racket_pos_raw", "dbg_racket_pos_gated",
                "dbg_racket_vel_raw", "dbg_racket_vel_gated",
                "dbg_racket_normal_raw", "dbg_racket_normal_gated",
                "dbg_base_raw", "dbg_base_gated",
            ):
                self.metrics[_k] = torch.zeros(self.num_envs, device=self.device)

    # ------------------------------------------------------------------ #
    # CommandTerm API
    # ------------------------------------------------------------------ #
    def _assert_v17_metric_storage_isolation(self) -> None:
        """Reject telemetry views into V17's physical/action Markov state."""

        motion = self._motion()
        if getattr(motion, "_post_swing_replay_contract", "") not in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            return

        protected: dict[str, torch.Tensor] = {
            "robot_joint_pos": self.robot.data.joint_pos,
            "robot_joint_vel": self.robot.data.joint_vel,
            "robot_default_joint_pos": self.robot.data.default_joint_pos,
            "robot_root_state": self.robot.data.root_state_w,
            "robot_root_link_pose": self.robot.data.root_link_pose_w,
            "robot_root_com_velocity": self.robot.data.root_com_vel_w,
            "robot_body_pos": self.robot.data.body_pos_w,
            "robot_body_quat": self.robot.data.body_quat_w,
            "robot_body_lin_vel": self.robot.data.body_lin_vel_w,
            "robot_body_ang_vel": self.robot.data.body_ang_vel_w,
            "manager_action": self._env.action_manager.action,
            "manager_prev_action": self._env.action_manager.prev_action,
        }
        action_term = self._env.action_manager.get_term(
            str(motion.cfg.post_swing_action_name)
        )
        for label, attribute in (
            ("action_raw", "_raw_actions"),
            ("action_applied_raw", "_applied_raw_actions"),
            ("action_unclamped_qdes", "_unclamped_processed_actions"),
            ("action_processed_qdes", "_processed_actions"),
            ("action_executed_qdes", "_qdes_executed"),
        ):
            value = getattr(action_term, attribute, None)
            if torch.is_tensor(value):
                protected[label] = value
        for label, attribute in (
            ("action_decoder_offset", "_offset"),
            ("action_decoder_scale", "_scale"),
        ):
            value = getattr(action_term, attribute, None)
            if torch.is_tensor(value):
                protected[label] = value

        protected_by_storage: dict[int, list[str]] = {}
        for label, value in protected.items():
            pointer = int(value.untyped_storage().data_ptr())
            protected_by_storage.setdefault(pointer, []).append(label)
        aliases = {}
        for command_name, metrics in (
            ("motion", motion.metrics),
            ("racket_target", self.metrics),
        ):
            for name, value in metrics.items():
                if not torch.is_tensor(value):
                    continue
                pointer = int(value.untyped_storage().data_ptr())
                if pointer in protected_by_storage:
                    aliases[f"{command_name}/{name}"] = (
                        protected_by_storage[pointer]
                    )
        if aliases:
            raise RuntimeError(
                "V17 command metrics share storage with live Markov state; "
                f"record detached snapshots instead: {aliases}"
            )

    def reset(self, env_ids=None):
        # Base CommandTerm.reset zeros every metric in-place before resampling.
        self._assert_v17_metric_storage_isolation()
        return super().reset(env_ids)

    @property
    def command(self) -> torch.Tensor:
        """Raw desired-target vector (world frame): [pos(3), vel(3), normal(3), t_left(1), base_xy(2), swing(1)]."""
        return torch.cat(
            [
                self.racket_target_pos_w,
                self.racket_target_vel_w,
                self.racket_target_normal_w,
                self.time_to_strike.unsqueeze(-1),
                self.base_target_pos_w,
                self.swing_sign.unsqueeze(-1),
            ],
            dim=-1,
        )

    @property
    def base_pos_w(self) -> torch.Tensor:
        return self.robot.data.root_pos_w

    @property
    def base_quat_w(self) -> torch.Tensor:
        return self.robot.data.root_quat_w

    def _motion(self) -> MotionCommand:
        if self._motion_term is None:
            self._motion_term = self._env.command_manager.get_term(self.cfg.motion_command_name)
        return self._motion_term

    def _strike_frame_for_clip(self, motion, clip_id: int) -> tuple[int, float, int, int]:
        """Return (global strike frame, phase, segment start, segment len) for one reference clip."""
        nseg = int(motion.num_segments)
        if clip_id < 0 or clip_id >= nseg:
            raise IndexError(f"clip_id {clip_id} out of range for {nseg} segments")
        seg_start = int(motion.seg_start[clip_id].item())
        seg_len = int(motion.seg_len[clip_id].item())
        spc = tuple(self.cfg.strike_phase_per_clip)
        phase = float(spc[clip_id]) if spc and len(spc) == nseg else float(self.cfg.strike_phase)
        strike_step = seg_start + round(phase * (seg_len - 1))
        return int(strike_step), phase, seg_start, seg_len

    def _ref_racket_pos_at(
        self,
        motion,
        f: int,
        *,
        clip_start: int = 0,
        clip_end: int | None = None,
    ) -> torch.Tensor:
        """Racket-center FK position (env-origin rel) at reference frame ``f``.

        Uses the SAME FK as :meth:`_compute_racket_state` /
        :meth:`_ensure_reference_strike_state` (racket body, or wrist + constant mount offset) but reads
        the reference MOTION's body poses. ``f`` is clamped to the requested clip segment so clean
        centered-difference velocities never leak across a concatenated forehand/backhand boundary.
        """
        total = max(int(motion.time_step_total), 1)
        hi = total - 1 if clip_end is None else int(clip_end)
        lo = int(clip_start)
        f = int(max(lo, min(hi, f)))
        if self._racket_mode == "body":
            return motion._body_pos_w[f, self._racket_body_index]
        widx = self._wrist_body_index
        wpos = motion._body_pos_w[f, widx]
        wquat = motion._body_quat_w[f, widx]
        offset_w = quat_apply(wquat.unsqueeze(0), self._mount_offset[0:1]).squeeze(0)
        return wpos + offset_w

    def _ref_racket_pos_at_steps(
        self, motion, steps: torch.Tensor
    ) -> torch.Tensor:
        """Vectorized reference racket-center FK for one frame per environment."""

        if steps.shape != (self.num_envs,) or steps.dtype != torch.long:
            raise ValueError(
                "reference racket steps must be one int64 index per environment"
            )
        if self._racket_mode == "body":
            return motion._body_pos_w[steps, self._racket_body_index]
        wrist_pos = motion._body_pos_w[steps, self._wrist_body_index]
        wrist_quat = motion._body_quat_w[steps, self._wrist_body_index]
        mount_offset = self._mount_offset.expand(self.num_envs, -1)
        return wrist_pos + quat_apply(wrist_quat, mount_offset)

    def reference_racket_velocity_w(self) -> torch.Tensor:
        """Return a clean per-phase reference racket velocity for every environment.

        MotionLoader's stored tip velocity is noisy after 30→50 Hz interpolation.  Use the
        same centered finite-difference FK as the clean strike target, clamped independently
        inside each clip segment so forehand/backhand boundaries never leak into one another.
        """

        motion_command = self._motion()
        motion = motion_command.motion
        steps = motion_command.time_steps.long()
        window = max(int(self.cfg.clean_strike_vel_window), 1)
        if motion_command._multiseg:
            clip = motion_command.clip_id
            segment_start = motion.seg_start[clip].long()
            segment_end = (
                segment_start + motion.seg_len[clip].long() - 1
            )
        else:
            segment_start = torch.zeros_like(steps)
            segment_end = torch.full_like(
                steps, max(int(motion.time_step_total) - 1, 0)
            )
        lower = torch.maximum(steps - window, segment_start)
        upper = torch.minimum(steps + window, segment_end)
        lower_pos = self._ref_racket_pos_at_steps(motion, lower)
        upper_pos = self._ref_racket_pos_at_steps(motion, upper)
        elapsed = (
            (upper - lower).clamp_min(1).float()
            * float(self._env.step_dt)
        ).unsqueeze(-1)
        return (upper_pos - lower_pos) / elapsed

    def _ensure_reference_strike_state(self):
        """Cache per-clip reference racket/base states at each clip's strike frame.

        Target sampling in ``reference_perturbed`` mode must be centered on the exact teacher clip the
        env is imitating. The old single cached state was fine for one clip, but a unified forehand+
        backhand policy needs separate strike position, velocity, face normal, and base->racket reach
        offsets for each concatenated MotionLoader segment.
        """
        if self._ref_strike_cached:
            return
        motion = self._motion().motion  # MotionLoader
        nseg = int(motion.num_segments)
        pos_all = torch.zeros(nseg, 3, device=self.device)
        vel_all = torch.zeros(nseg, 3, device=self.device)
        nrm_all = torch.zeros(nseg, 3, device=self.device)
        base_all = torch.zeros(nseg, 3, device=self.device)
        reach_all = torch.zeros(nseg, 2, device=self.device)
        W = max(1, int(self.cfg.clean_strike_vel_window))
        dt = float(self._env.step_dt)
        report_lines = []

        for clip_id in range(nseg):
            strike_step, phase, seg_start, seg_len = self._strike_frame_for_clip(motion, clip_id)
            seg_end = seg_start + seg_len - 1
            if self._racket_mode == "body":
                idx = self._racket_body_index
                pos, quat, lin, _ = self._reference_body_state(motion, strike_step, idx)
            else:
                widx = self._wrist_body_index
                wpos, wquat, wlin, wang = self._reference_body_state(motion, strike_step, widx, require_ang_vel=True)
                offset_w = quat_apply(wquat.unsqueeze(0), self._mount_offset[0:1]).squeeze(0)
                pos = wpos + offset_w
                lin = wlin + torch.cross(wang, offset_w, dim=-1)
                quat = quat_mul(wquat.unsqueeze(0), self._mount_quat[0:1]).squeeze(0)
            # Per-clip striking-face sign (opposite paddle faces for fh/bh) so the reference face normal —
            # used by the diagnostic report and any reference-locked normal target — matches the scored face.
            _sgn = self.cfg.mount_normal_sign
            if self._mount_sign_per_clip_t is not None and clip_id < self._mount_sign_per_clip_t.shape[0]:
                _sgn = float(self._mount_sign_per_clip_t[clip_id])
            normal = matrix_from_quat(quat.unsqueeze(0))[0, :, self.cfg.mount_normal_axis] * _sgn

            # --- clean reference strike velocity --------------------------------------------------
            # Recompute the strike target velocity from the FINAL racket FK position by a centered
            # finite difference (clamped to this clip's segment), so it is consistent with the
            # position the policy actually tracks (the stored body_lin_vel_w is FD'd/interpolated and
            # ~1 m/s inconsistent at the racket tip — see cfg docs). raw_lin = legacy single-frame
            # stored velocity (kept for the flag-off path and the diagnostics).
            raw_lin = lin.detach().clone()
            fd1 = (
                self._ref_racket_pos_at(motion, strike_step + 1, clip_start=seg_start, clip_end=seg_end)
                - self._ref_racket_pos_at(motion, strike_step - 1, clip_start=seg_start, clip_end=seg_end)
            ) / (2.0 * dt)
            clean_lin = (
                self._ref_racket_pos_at(motion, strike_step + W, clip_start=seg_start, clip_end=seg_end)
                - self._ref_racket_pos_at(motion, strike_step - W, clip_start=seg_start, clip_end=seg_end)
            ) / (2.0 * W * dt)
            if self.cfg.clean_reference_strike_velocity:
                lin = clean_lin

            pos_all[clip_id] = pos.detach().clone()
            vel_all[clip_id] = lin.detach().clone()
            nrm_all[clip_id] = (normal / (torch.norm(normal) + 1e-6)).detach().clone()
            # Reference base (root) at the strike — root is articulation body index 0 (same order the
            # motion arrays use). The base->racket horizontal offset couples base_target to racket_target.
            base_all[clip_id] = self._reference_body_state(motion, strike_step, 0)[0].detach().clone()
            reach_all[clip_id] = (pos_all[clip_id, :2] - base_all[clip_id, :2]).detach().clone()

            cname = self._clip_names.get(clip_id, f"clip{clip_id}")
            p = pos_all[clip_id]
            v = vel_all[clip_id]
            nrm = nrm_all[clip_id]
            b = base_all[clip_id]
            off = reach_all[clip_id]
            report_lines.append(
                f"  {cname}: strike frame {strike_step}/{seg_end} (phase {phase:.3f}) "
                f"pos=({float(p[0]):.3f},{float(p[1]):.3f},{float(p[2]):.3f}) "
                f"vel=({float(v[0]):.3f},{float(v[1]):.3f},{float(v[2]):.3f}) "
                f"|v|={float(torch.norm(v)):.2f} "
                f"normal=({float(nrm[0]):.3f},{float(nrm[1]):.3f},{float(nrm[2]):.3f}) "
                f"baseXY=({float(b[0]):.3f},{float(b[1]):.3f}) "
                f"reachXY=({float(off[0]):.3f},{float(off[1]):.3f}) "
                f"raw_speed={float(torch.norm(raw_lin)):.3f} "
                f"clean_speed={float(torch.norm(clean_lin)):.3f} "
                f"raw_clean_diff={float(torch.norm(raw_lin - clean_lin)):.3f} "
                f"raw_fd_diff={float(torch.norm(raw_lin - fd1)):.3f}"
            )

        self._ref_racket_pos_rel_per_clip = pos_all
        self._ref_racket_vel_w_per_clip = vel_all
        self._ref_racket_normal_w_per_clip = nrm_all
        self._ref_base_pos_rel_per_clip = base_all
        self._ref_reach_offset_xy_per_clip = reach_all
        # Legacy single-clip fields (no in-file consumers besides diagnostics): mirror clip 0.
        self._ref_racket_pos_rel = pos_all[0].detach().clone()
        self._ref_racket_vel_w = vel_all[0].detach().clone()
        self._ref_racket_normal_w = nrm_all[0].detach().clone()
        self._ref_base_pos_rel = base_all[0].detach().clone()
        self._ref_reach_offset_xy = reach_all[0].detach().clone()
        self._ref_strike_cached = True
        print(
            "[RacketTargetCommand] reference strike centers per clip "
            f"(clean_reference_strike_velocity={self.cfg.clean_reference_strike_velocity}, window=+-{W}):\n"
            + "\n".join(report_lines),
            flush=True,
        )

    def _reference_body_state(self, motion, step: int, body_index: int, require_ang_vel: bool = False):
        """Return reference body state from MotionLoader's full-articulation private arrays.

        This is the current, intentional coupling point to upstream ``MotionLoader`` internals. Public
        ``MotionCommand`` buffers expose only the tracking subset, while the racket FK needs the full
        articulation body order so it can read the racket body or wrist mount. Keep direct private-field
        access centralized here until MotionLoader grows a public full-body state API.
        """
        required = ["_body_pos_w", "_body_quat_w", "_body_lin_vel_w"]
        if require_ang_vel:
            required.append("_body_ang_vel_w")
        missing = [name for name in required if not hasattr(motion, name)]
        if missing:
            raise AttributeError(
                "RacketTargetCommand requires MotionLoader full-body reference arrays "
                f"{required}, but missing {missing}. This is the HOPE coupling point for "
                "reference_perturbed racket FK; update _reference_body_state if upstream MotionLoader changes."
            )

        pos = motion._body_pos_w[step, body_index]
        quat = motion._body_quat_w[step, body_index]
        lin = motion._body_lin_vel_w[step, body_index]
        ang = motion._body_ang_vel_w[step, body_index] if require_ang_vel else None
        return pos, quat, lin, ang

    def _perturb_scale(self) -> float:
        """Curriculum factor in [start, 1.0] that widens the reference perturbation over training.

        Success-gated mode (default): return the running ``_curr_perturb_scale``, which advances only
        when the policy demonstrates exact-strike success (see :meth:`_update_metrics`). Otherwise fall
        back to the legacy open-loop ramp keyed to ``env.common_step_counter``. The returned scale is
        clamped to ``[ref_perturb_curriculum_start, 1.0]``.
        """
        start = float(self.cfg.ref_perturb_curriculum_start)
        if self.cfg.target_mode == "reference_perturbed" and self.cfg.ref_perturb_success_gated:
            scale = self._curr_perturb_scale
        else:
            steps = float(getattr(self._env, "common_step_counter", 0))
            c = float(self.cfg.ref_perturb_curriculum_steps)
            frac = 1.0 if c <= 0.0 else min(1.0, steps / c)
            scale = start + (1.0 - start) * frac
        return min(1.0, max(start, scale))

    def _ensure_ref_normal_per_clip(self):
        """Cache the reference paddle face normal at each clip's strike frame ([num_segments, 3])."""
        if self._ref_normal_per_clip is not None:
            return
        self._ensure_reference_strike_state()
        assert self._ref_racket_normal_w_per_clip is not None
        self._ref_normal_per_clip = self._ref_racket_normal_w_per_clip

    def _sample_targets_uniform(self, env_ids: Sequence[int], origins: torch.Tensor, n: int):
        """Independent box sampling (legacy mode). Ranges are PLACEHOLDERS not tied to the swing."""
        pos = origins.clone()
        motion = self._motion()
        if self._pos_range_per_clip_t is not None and motion._multiseg:
            # PER-CLIP position box (unified policy): each env samples x/y/z from ITS clip's box (added to
            # the env origin). The y range is SIGNED per clip (the configured box is used directly, so a
            # near-center backhand box is valid and does not go through the shared +/-|y| fallback). This
            # replaces the shared x-range + |y|-sign + z-range logic below and lets each clip track its own
            # reference strike point.
            clip = motion.clip_id[env_ids]                      # (n,) long, 0=forehand / 1=backhand
            rng_e = self._pos_range_per_clip_t[clip]            # (n, 3, 2): [env][x/y/z][lo/hi]
            lo = rng_e[..., 0]                                  # (n, 3)
            hi = rng_e[..., 1]                                  # (n, 3)
            pos[:, :3] += lo + (hi - lo) * torch.rand(n, 3, device=self.device)
        else:
            # Shared box (legacy / single-clip): identical sampling to before — backward compatible.
            pos[:, 0] += sample_uniform(*self.cfg.racket_pos_x_range, (n,), self.device)
            if motion._multiseg:
                # Unified policy: the target Y region is conditioned on the swing TYPE (clip) so forehand and
                # backhand regions are non-overlapping (HITTER §IV). Sample |y| and set the sign per clip:
                # forehand (clip 0) on -y if forehand_on_negative_y, backhand (clip 1) on the opposite side.
                clip = motion.clip_id[env_ids]
                ymag = sample_uniform(*self.cfg.racket_pos_y_abs_range, (n,), self.device)
                fh_sign = -1.0 if self.cfg.forehand_on_negative_y else 1.0
                sign = torch.where(clip == 0, fh_sign, -fh_sign)
                pos[:, 1] = origins[:, 1] + sign * ymag
            else:
                pos[:, 1] += sample_uniform(*self.cfg.racket_pos_y_range, (n,), self.device)
            pos[:, 2] += sample_uniform(*self.cfg.racket_pos_z_range, (n,), self.device)
        self.racket_target_pos_w[env_ids] = pos

        if self._vel_range_per_clip_t is not None and motion._multiseg:
            # PER-CLIP velocity (unified policy): each env samples from ITS clip's box, so the slower
            # backhand gets a lower target speed than the forehand instead of one shared box that
            # overshoots the backhand. Vectorized: gather each env's clip range, then uniform-sample.
            clip = motion.clip_id[env_ids]                      # (n,) long, 0=forehand / 1=backhand
            rng_e = self._vel_range_per_clip_t[clip]            # (n, 3, 2): [env][x/y/z][lo/hi]
            lo = rng_e[..., 0]                                  # (n, 3)
            hi = rng_e[..., 1]                                  # (n, 3)
            vel = lo + (hi - lo) * torch.rand(n, 3, device=self.device)
        else:
            # Shared box (legacy / single-clip): identical sampling to before — backward compatible.
            vel = torch.empty(n, 3, device=self.device)
            vel[:, 0] = sample_uniform(*self.cfg.racket_vel_x_range, (n,), self.device)
            vel[:, 1] = sample_uniform(*self.cfg.racket_vel_y_range, (n,), self.device)
            vel[:, 2] = sample_uniform(*self.cfg.racket_vel_z_range, (n,), self.device)
        self.racket_target_vel_w[env_ids] = vel

        # --- HER-style achieved-target replay (mixture) --------------------------------------------
        # With prob achieved_target_mix_prob, overwrite the freshly box-sampled pos+vel with a jittered
        # PREVIOUSLY-ACHIEVED strike state from this clip's ring buffer (written at exact-strike frames
        # in _update_metrics). Replayed targets are reachable-by-demonstration, so the target
        # distribution stops asking for points the taught swing never passes through (Ace/HER, adapted
        # forward-looking for on-policy PPO — retroactive relabel would be obs-inconsistent). Clamped
        # into the per-clip box inflated by achieved_clamp_inflate so replay can neither collapse the
        # target support nor drift outside the deploy runner's hand-synced target clips. Non-replayed
        # envs keep the pure box sample; the per-clip reference normal below is shared by both paths.
        if self.cfg.achieved_target_mix_prob > 0.0 and motion._multiseg:
            env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            clip_all = motion.clip_id[env_ids_t]
            replay = torch.rand(n, device=self.device) < float(self.cfg.achieved_target_mix_prob)
            self._resample_n_acc += float(n)
            infl = 1.0 + float(self.cfg.achieved_clamp_inflate)
            for c in self._clip_names:
                fill = self._ach_fill.get(c, 0)
                if fill < int(self.cfg.achieved_min_fill):
                    continue
                sel = replay & (clip_all == c)
                m = int(sel.sum())
                if m == 0:
                    continue
                rows = torch.randint(0, fill, (m,), device=self.device)
                rpos = self._ach_pos[c][rows] + (torch.rand(m, 3, device=self.device) * 2.0 - 1.0) * float(
                    self.cfg.achieved_jitter_pos
                )
                rvel = self._ach_vel[c][rows] + (torch.rand(m, 3, device=self.device) * 2.0 - 1.0) * float(
                    self.cfg.achieved_jitter_vel
                )
                if self._pos_range_per_clip_t is not None and c < self._pos_range_per_clip_t.shape[0]:
                    lo, hi = self._pos_range_per_clip_t[c, :, 0], self._pos_range_per_clip_t[c, :, 1]
                    ctr, half = (lo + hi) * 0.5, (hi - lo) * 0.5 * infl
                    rpos = torch.min(torch.max(rpos, ctr - half), ctr + half)
                if self._vel_range_per_clip_t is not None and c < self._vel_range_per_clip_t.shape[0]:
                    lo, hi = self._vel_range_per_clip_t[c, :, 0], self._vel_range_per_clip_t[c, :, 1]
                    ctr, half = (lo + hi) * 0.5, (hi - lo) * 0.5 * infl
                    rvel = torch.min(torch.max(rvel, ctr - half), ctr + half)
                ids_sel = env_ids_t[sel]
                self.racket_target_pos_w[ids_sel] = self._env.scene.env_origins[ids_sel] + rpos
                self.racket_target_vel_w[ids_sel] = rvel
                self._replay_n_acc += float(m)

        if motion._multiseg:
            # Unified policy: the target paddle normal is the imitated swing's actual face normal at
            # strike (reachable by the imitation). The sampled racket velocity is the SWING-PATH direction,
            # which is ~18-110 deg off the +Y blade face, so normal_mode=velocity makes the normal goal
            # unsatisfiable (normal_pass=0 -> composite success stuck at 0).
            self._ensure_ref_normal_per_clip()
            clip = motion.clip_id[env_ids]
            normal = self._ref_normal_per_clip[clip]
        elif self.cfg.normal_mode == "velocity":
            normal = vel / (torch.norm(vel, dim=-1, keepdim=True) + 1e-6)
        else:  # "sampled"
            normal = torch.empty(n, 3, device=self.device)
            normal[:, 0] = sample_uniform(*self.cfg.racket_normal_x_range, (n,), self.device)
            normal[:, 1] = sample_uniform(*self.cfg.racket_normal_y_range, (n,), self.device)
            normal[:, 2] = sample_uniform(*self.cfg.racket_normal_z_range, (n,), self.device)
            normal = normal / (torch.norm(normal, dim=-1, keepdim=True) + 1e-6)
        self.racket_target_normal_w[env_ids] = normal

    def _record_strike_audit_target(
        self,
        env_ids: torch.Tensor,
        *,
        clip: torch.Tensor,
        position_offset: torch.Tensor,
        position_box: torch.Tensor,
        velocity: torch.Tensor,
        velocity_box: torch.Tensor,
        velocity_source: torch.Tensor,
        count_start: bool,
    ) -> None:
        """Classify a sampled target for training-only stratified instrumentation.

        ``velocity_source`` is 0 for the demonstrated/core box and 1 for the planner box.
        Speed quartiles are equal-width bins over the attainable speed-norm span of the selected
        source box; z bins are equal thirds of the selected clip's configured z interval.
        """
        lo_v = velocity_box[..., 0]
        hi_v = velocity_box[..., 1]
        crosses_zero = (lo_v <= 0.0) & (hi_v >= 0.0)
        min_abs_component = torch.where(
            crosses_zero,
            torch.zeros_like(lo_v),
            torch.minimum(torch.abs(lo_v), torch.abs(hi_v)),
        )
        max_abs_component = torch.maximum(torch.abs(lo_v), torch.abs(hi_v))
        speed_min = torch.linalg.norm(min_abs_component, dim=-1)
        speed_max = torch.linalg.norm(max_abs_component, dim=-1)
        speed_fraction = (
            (torch.linalg.norm(velocity, dim=-1) - speed_min)
            / (speed_max - speed_min).clamp_min(1.0e-6)
        ).clamp(0.0, 1.0 - 1.0e-7)
        speed_quartile = torch.floor(4.0 * speed_fraction).long().clamp(0, 3)

        z_lo = position_box[:, 2, 0]
        z_hi = position_box[:, 2, 1]
        z_fraction = (
            (position_offset[:, 2] - z_lo) / (z_hi - z_lo).clamp_min(1.0e-6)
        ).clamp(0.0, 1.0 - 1.0e-7)
        z_bin = torch.floor(3.0 * z_fraction).long().clamp(0, 2)

        source = velocity_source.long()
        valid = (clip >= 0) & (clip < 2) & (source >= 0) & (source < 2)
        context = (((clip.long() * 2 + source) * 4 + speed_quartile) * 3 + z_bin)
        context = torch.where(valid, context, torch.full_like(context, -1))
        self._strike_audit_context_id[env_ids] = context
        self._target_velocity_source[env_ids] = torch.where(
            valid, source, torch.full_like(source, -1)
        )
        self._target_speed_quartile[env_ids] = torch.where(
            valid, speed_quartile, torch.full_like(speed_quartile, -1)
        )
        self._target_z_bin[env_ids] = torch.where(
            valid, z_bin, torch.full_like(z_bin, -1)
        )
        self.metrics["strike_target_source"][env_ids] = self._target_velocity_source[
            env_ids
        ].float()
        self.metrics["strike_target_speed_quartile"][env_ids] = (
            self._target_speed_quartile[env_ids].float()
        )
        self.metrics["strike_target_z_bin"][env_ids] = self._target_z_bin[
            env_ids
        ].float()
        if count_start:
            self._strike_audit_start_count += torch.bincount(
                context[valid], minlength=self._strike_audit_size
            ).to(self._strike_audit_start_count.dtype)

    def _velocity_sampling_state(self) -> tuple[float, float]:
        """Return bootstrap-core expansion and effective planner mixture."""

        final_mix = float(self.cfg.racket_vel_planner_mix_prob)
        progress_override = self.cfg.racket_vel_curriculum_progress_override
        if progress_override is not None:
            progress = float(progress_override)
            return progress, progress * final_mix
        if bool(getattr(self, "_eval_force_final_velocity_stage", False)):
            return 1.0, final_mix
        if self.cfg.racket_vel_planner_mix_by_velocity_stage:
            config = self._velocity_stage_config
            return staged_velocity_sampling(
                current_weight=self._velocity_current_weight,
                stage0_weight=config.stage0_weight,
                stage1_weight=config.stage1_weight,
                stage2_weight=config.stage2_weight,
                stage1_planner_mix=float(
                    self.cfg.racket_vel_stage1_planner_mix_prob
                ),
                final_planner_mix=final_mix,
            )
        ramp_steps = int(self.cfg.racket_vel_range_ramp_steps)
        if ramp_steps <= 0:
            raise ValueError(
                "racket_vel_range_ramp_steps must be > 0 for velocity sampling"
            )
        progress = min(
            1.0,
            float(getattr(self._env, "common_step_counter", 0))
            / float(ramp_steps),
        )
        return progress, progress * final_mix

    def _sample_targets_hitter_pure(
        self, env_ids: Sequence[int], origins: torch.Tensor, n: int, resample_base: bool = True
    ):
        """HITTER-faithful sampling (arXiv:2508.21043 §V-B-1 + §IV-C), 2026-07-07.

        Order and frames follow the paper exactly:

        1. BASE STATION first, sampled INDEPENDENTLY around the env origin (world frame) from
           ``base_target_x_range`` / ``base_target_y_range`` (which are the STATION BOX here, not a
           jitter — paper Fig. 4 evaluates initial station distances up to ±0.8 m).
        2. RACKET TARGET on a striking plane FIXED RELATIVE TO THE COMMANDED STATION ("0.4 m in
           front of the robot" on their G1; our A3 analog is the clips' blade reach x ≈ 0.70 m):
           the per-clip ``racket_pos_range_per_clip`` boxes are interpreted as STATION-RELATIVE
           x/y offsets (x degenerate = the fixed plane, y = the swing-side band) with z absolute
           above the ground. Forehand/backhand y-bands must be non-overlapping (paper §V-B-1).
        3. RACKET VELOCITY from the per-clip velocity boxes (world frame), then the target FACE
           NORMAL from ``normal_mode``: "velocity" = the paper's §IV-C impact model ("the racket
           plane is perpendicular to its velocity vector") — the policy must LEARN to orient the
           blade; do NOT fall back to the reference-clip normal here (that made the normal term
           trivially satisfied and is why deployed models could touch balls but not return them).

        No HER replay, no reference_reach coupling, no curriculum in this mode.

        ``resample_base=False`` (mid-swing refinement path): keep the CURRENT station and only
        re-draw the racket target/velocity around it — the paper's Fig. 3 refinement converges on
        WHERE the ball arrives; it never teleports the commanded stance mid-swing.
        """
        motion = self._motion()
        if self._pos_range_per_clip_t is None or self._vel_range_per_clip_t is None:
            raise RuntimeError(
                "target_mode='hitter_pure' requires racket_pos_range_per_clip AND "
                "racket_vel_range_per_clip (station-relative position boxes; see "
                "HOPEPingPongHitterPure.yaml)."
            )

        # MotionCommand chooses the clip before RacketTargetCommand runs. Keep that clip available
        # while sampling the station transition: RallyFinal constrains the two ambiguous >half-reach
        # directions below so the deploy runner's nearest-previous-station inverse chooses the SAME
        # forehand/backhand clip that training did.
        env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        previous_station_y = self.base_target_pos_w[env_ids_t, 1].clone()
        if motion._multiseg:
            clip = motion.clip_id[env_ids]
        else:
            clip = torch.zeros(n, dtype=torch.long, device=self.device)
        eval_sequence = tuple(getattr(self.cfg, "eval_gate3_sequence", ()) or ())
        sequence_rows = None
        if eval_sequence:
            if not motion._multiseg or not hasattr(motion, "eval_sequence_index"):
                raise RuntimeError(
                    "eval_gate3_sequence requires MotionCommand.eval_clip_sequence and multiseg clips"
                )
            if not hasattr(self, "_eval_gate3_sequence_t"):
                table = torch.as_tensor(eval_sequence, dtype=torch.float32, device=self.device)
                if table.ndim != 2 or table.shape[1] != 8:
                    raise ValueError(
                        "eval_gate3_sequence rows must have 8 fields: "
                        "clip,station_y,reach_xyz,target_vxyz"
                    )
                self._eval_gate3_sequence_t = table
            sequence_index = motion.eval_sequence_index[env_ids]
            sequence_rows = self._eval_gate3_sequence_t[sequence_index]
            if not torch.equal(sequence_rows[:, 0].long(), clip.long()):
                raise RuntimeError("Gate3 sequence side is out of sync with MotionCommand clip")

        # 1) base station (world xy, around the env origin).  Plain HitterPure keeps the paper's
        # independent absolute-box draw.  RallyFinal optionally constrains WRAP-to-WRAP lateral
        # station changes to ``station_y_step_range`` so consecutive balls ask for a controlled
        # 20-40 cm step rather than an independent-box jump as large as the full box width.  True
        # episode resets still draw from the absolute box (there is no previous rally station).
        if resample_base and sequence_rows is not None:
            env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            previous_y = self.base_target_pos_w[env_ids_t, 1].clone()
            base_xy = origins[:, :2].clone()
            base_xy[:, 1] += sequence_rows[:, 1]
            self.base_target_pos_w[env_ids] = base_xy
            signed_step = base_xy[:, 1] - previous_y
            step = torch.abs(signed_step)
            self.metrics["station_y_step_command"][env_ids_t] = step
            self.metrics["station_y_step_signed_command"][env_ids_t] = signed_step
            step_class = torch.where(
                step < 1.0e-5,
                torch.zeros_like(step),
                torch.where(step < 0.10, torch.ones_like(step), torch.full_like(step, 2.0)),
            )
            self.metrics["station_y_step_class"][env_ids_t] = torch.where(
                torch.full_like(step, self._resample_is_wrap, dtype=torch.bool),
                step_class,
                torch.full_like(step, -1.0),
            )
        elif resample_base:
            env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            previous_y = self.base_target_pos_w[env_ids_t, 1].clone()
            base_xy = origins[:, :2].clone()
            base_xy[:, 0] += sample_uniform(*self.cfg.base_target_x_range, (n,), self.device)
            step_range = self.cfg.station_y_step_range
            if self._resample_is_wrap and step_range is not None:
                y_lo, y_hi = (float(self.cfg.base_target_y_range[0]), float(self.cfg.base_target_y_range[1]))
                step_lo, step_hi = (float(step_range[0]), float(step_range[1]))
                if not (0.0 < step_lo <= step_hi <= (y_hi - y_lo)):
                    raise ValueError(
                        "station_y_step_range must satisfy 0 < lo <= hi <= base_target_y span; "
                        f"got step={step_range}, base_y={self.cfg.base_target_y_range}"
                    )
                prev_rel = (previous_y - origins[:, 1]).clamp(y_lo, y_hi)
                room_pos = y_hi - prev_rel
                room_neg = prev_rel - y_lo
                same_prob = float(self.cfg.station_y_same_prob)
                small_prob = float(self.cfg.station_y_small_step_prob)
                positive_main_prob = float(self.cfg.station_y_positive_main_prob)
                mixed_steps = same_prob > 0.0 or small_prob > 0.0
                if mixed_steps:
                    if not bool(self.cfg.station_side_explicit):
                        raise ValueError(
                            "station-y step mixtures require station_side_explicit=True: "
                            "the deployed runner must consume the planner's explicit swing side "
                            "instead of re-inferring it from nearest-station geometry"
                        )
                    if not (0.0 <= same_prob <= 1.0 and 0.0 <= small_prob <= 1.0
                            and same_prob + small_prob <= 1.0):
                        raise ValueError(
                            "station_y_same_prob and station_y_small_step_prob must be in [0,1] "
                            f"with sum <= 1, got {same_prob} + {small_prob}"
                        )
                    if not 0.0 <= positive_main_prob <= 1.0:
                        raise ValueError(
                            "station_y_positive_main_prob must be in [0,1], got "
                            f"{positive_main_prob}"
                        )
                    half_span = 0.5 * (y_hi - y_lo)
                    if step_lo > half_span:
                        raise ValueError(
                            "mixed station main-step lower bound must be <= half the absolute "
                            f"station span ({half_span:.3f}), got {step_lo:.3f}"
                        )
                    small_range = self.cfg.station_y_small_step_range
                    if small_prob > 0.0 and small_range is None:
                        raise ValueError(
                            "station_y_small_step_prob > 0 requires station_y_small_step_range"
                        )
                    if small_range is None:
                        small_lo = small_hi = 0.0
                    else:
                        small_lo, small_hi = (float(small_range[0]), float(small_range[1]))
                        if not (0.0 < small_lo <= small_hi <= (y_hi - y_lo)):
                            raise ValueError(
                                "station_y_small_step_range must satisfy 0 < lo <= hi <= "
                                f"base_target_y span; got {small_range}"
                            )
                        if small_lo > half_span:
                            raise ValueError(
                                "mixed station small-step lower bound must be <= half the "
                                f"absolute station span ({half_span:.3f}), got {small_lo:.3f}"
                            )

                    positive_main_range = self.cfg.station_y_positive_main_step_range
                    if positive_main_prob > 0.0 and positive_main_range is None:
                        raise ValueError(
                            "station_y_positive_main_prob > 0 requires "
                            "station_y_positive_main_step_range"
                        )
                    if positive_main_range is None:
                        positive_main_lo = positive_main_hi = 0.0
                    else:
                        positive_main_lo, positive_main_hi = (
                            float(positive_main_range[0]),
                            float(positive_main_range[1]),
                        )
                        if not (
                            0.0 < positive_main_lo <= positive_main_hi <= (y_hi - y_lo)
                        ):
                            raise ValueError(
                                "station_y_positive_main_step_range must satisfy "
                                "0 < lo <= hi <= base_target_y span; got "
                                f"{positive_main_range}"
                            )
                        if positive_main_lo > half_span:
                            raise ValueError(
                                "positive main-step lower bound must be <= half the absolute "
                                f"station span ({half_span:.3f}), got {positive_main_lo:.3f}"
                            )

                    # Category is exogenous and independent of policy state.  The remaining
                    # probability belongs to the configured main step range.
                    u = torch.rand(n, device=self.device)
                    same = u < same_prob
                    small = (u >= same_prob) & (u < same_prob + small_prob)
                    main = ~(same | small)
                    # RallyV11 Gate3 repair: when a main-step question has enough +y room,
                    # preferentially draw the measured 19--24 cm backhand-station transition.
                    # Defaults are zero/None, so every earlier task keeps the symmetric sampler.
                    positive_main = (
                        main
                        & (torch.rand(n, device=self.device) < positive_main_prob)
                        & (room_pos >= positive_main_lo)
                    )
                    req_lo = torch.where(
                        same,
                        torch.zeros(n, device=self.device),
                        torch.where(
                            small,
                            torch.full((n,), small_lo, device=self.device),
                            torch.where(
                                positive_main,
                                torch.full((n,), positive_main_lo, device=self.device),
                                torch.full((n,), step_lo, device=self.device),
                            ),
                        ),
                    )
                    req_hi = torch.where(
                        same,
                        torch.zeros(n, device=self.device),
                        torch.where(
                            small,
                            torch.full((n,), small_hi, device=self.device),
                            torch.where(
                                positive_main,
                                torch.full((n,), positive_main_hi, device=self.device),
                                torch.full((n,), step_hi, device=self.device),
                            ),
                        ),
                    )

                    # Pick only a direction with enough room for the selected class, then
                    # truncate its upper endpoint at the absolute station boundary.  Same-
                    # station transitions are exactly zero and need no direction.
                    can_pos = room_pos >= req_lo
                    can_neg = room_neg >= req_lo
                    choose_pos = torch.rand(n, device=self.device) < 0.5
                    choose_pos = torch.where(can_pos & ~can_neg, torch.ones_like(choose_pos), choose_pos)
                    choose_pos = torch.where(can_neg & ~can_pos, torch.zeros_like(choose_pos), choose_pos)
                    choose_pos = torch.where(
                        positive_main, torch.ones_like(choose_pos), choose_pos
                    )
                    room = torch.where(choose_pos, room_pos, room_neg)
                    upper = torch.minimum(room, req_hi)
                    delta = req_lo + (upper - req_lo).clamp_min(0.0) * torch.rand(
                        n, device=self.device
                    )
                    delta = torch.where(same, torch.zeros_like(delta), delta)
                    stepped = prev_rel + torch.where(choose_pos, delta, -delta)
                    base_xy[:, 1] += stepped
                    step_class = torch.where(
                        same,
                        torch.zeros(n, device=self.device),
                        torch.where(
                            small,
                            torch.ones(n, device=self.device),
                            torch.where(
                                positive_main,
                                torch.full((n,), 3.0, device=self.device),
                                torch.full((n,), 2.0, device=self.device),
                            ),
                        ),
                    )
                    self.metrics["station_y_step_class"][env_ids_t] = step_class
                else:
                    # Legacy RallyFinal path: retain the nearest-station ambiguity cap
                    # byte-for-byte when no FinalV3 mixture is configured.
                    if self._pos_range_per_clip_t.shape[0] == 2:
                        reach_y = self._pos_range_per_clip_t[:, 1].mean(dim=-1)
                        half_sep = 0.5 * torch.abs(reach_y[1] - reach_y[0])
                        room_pos = torch.where(
                            clip == 0, torch.minimum(room_pos, half_sep.expand_as(room_pos)), room_pos
                        )
                        bh_neg_cap = torch.clamp(half_sep - 1e-4, min=step_lo)
                        room_neg = torch.where(
                            clip == 1, torch.minimum(room_neg, bh_neg_cap.expand_as(room_neg)), room_neg
                        )
                    can_pos = room_pos >= step_lo
                    can_neg = room_neg >= step_lo
                    choose_pos = torch.rand(n, device=self.device) < 0.5
                    choose_pos = torch.where(can_pos & ~can_neg, torch.ones_like(choose_pos), choose_pos)
                    choose_pos = torch.where(can_neg & ~can_pos, torch.zeros_like(choose_pos), choose_pos)
                    feasible = can_pos | can_neg
                    room = torch.where(choose_pos, room_pos, room_neg)
                    upper = torch.minimum(room, torch.full_like(room, step_hi))
                    delta = step_lo + (upper - step_lo).clamp_min(0.0) * torch.rand(n, device=self.device)
                    stepped = prev_rel + torch.where(choose_pos, delta, -delta)
                    absolute = sample_uniform(y_lo, y_hi, (n,), self.device)
                    base_xy[:, 1] += torch.where(feasible, stepped, absolute)
                    self.metrics["station_y_step_class"][env_ids_t] = 2.0
            else:
                base_xy[:, 1] += sample_uniform(*self.cfg.base_target_y_range, (n,), self.device)
            self.base_target_pos_w[env_ids] = base_xy
            if self._resample_is_wrap:
                signed_step = base_xy[:, 1] - previous_y
                self.metrics["station_y_step_command"][env_ids_t] = torch.abs(signed_step)
                self.metrics["station_y_step_signed_command"][env_ids_t] = signed_step
            else:
                self.metrics["station_y_step_command"][env_ids_t] = 0.0
                self.metrics["station_y_step_signed_command"][env_ids_t] = 0.0
                self.metrics["station_y_step_class"][env_ids_t] = -1.0
        else:
            base_xy = self.base_target_pos_w[env_ids].clone()

        # 2) racket target: per-clip STATION-RELATIVE box (x = fixed plane, y = swing band), z above ground.
        rng_e = self._pos_range_per_clip_t[clip]                # (n, 3, 2): [env][x/y/z][lo/hi]
        lo, hi = rng_e[..., 0], rng_e[..., 1]
        if sequence_rows is None:
            off = lo + (hi - lo) * torch.rand(n, 3, device=self.device)
        else:
            off = sequence_rows[:, 2:5]
            if bool(((off < lo - 1.0e-5) | (off > hi + 1.0e-5)).any()):
                raise ValueError("Gate3 sequence position lies outside the trained per-clip box")
        pos = origins.clone()
        pos[:, 0] = base_xy[:, 0] + off[:, 0]
        pos[:, 1] = base_xy[:, 1] + off[:, 1]
        pos[:, 2] = origins[:, 2] + off[:, 2]
        self.racket_target_pos_w[env_ids] = pos

        # 3) racket velocity (world) + face normal from normal_mode (paper: velocity direction).
        rng_v = self._vel_range_per_clip_t[clip]
        audit_velocity_box = rng_v
        audit_velocity_source = torch.zeros(
            n, dtype=torch.long, device=self.device
        )
        if sequence_rows is not None:
            vel = sequence_rows[:, 5:8]
            # A fixed evaluator sequence is neither a random core nor planner draw.
            audit_velocity_source.fill_(-1)
            if bool(((vel < rng_v[..., 0] - 1.0e-5) | (vel > rng_v[..., 1] + 1.0e-5)).any()):
                raise ValueError("Gate3 sequence velocity lies outside the trained per-clip envelope")
        elif self._vel_planner_range_per_clip_t is not None:
            ramp_steps = int(self.cfg.racket_vel_range_ramp_steps)
            if ramp_steps <= 0:
                raise ValueError(
                    "racket_vel_range_ramp_steps must be > 0 when a planner velocity box is set"
                )
            core_scale, planner_prob = self._velocity_sampling_state()
            use_planner = (torch.rand(n, 1, 1, device=self.device) < planner_prob)
            core_rng_v = self._vel_start_range_per_clip_t[clip]
            if self._vel_bootstrap_range_per_clip_t is not None:
                bootstrap_rng_v = self._vel_bootstrap_range_per_clip_t[clip]
                core_rng_v = bootstrap_rng_v + core_scale * (
                    core_rng_v - bootstrap_rng_v
                )
            planner_rng_v = self._vel_planner_range_per_clip_t[clip]
            rng_v = torch.where(use_planner, planner_rng_v, core_rng_v)
            audit_velocity_box = rng_v
            audit_velocity_source = use_planner[:, 0, 0].long()
            self.metrics["velocity_core_box_scale"][env_ids_t] = core_scale
            self.metrics["velocity_planner_mix_effective"][
                env_ids_t
            ] = planner_prob
        elif self._vel_start_range_per_clip_t is not None:
            # Generic fallback for tasks that request bound interpolation without a planner
            # mixture. V10 uses the branch above so its union-envelope corners are never sampled.
            ramp_steps = int(self.cfg.racket_vel_range_ramp_steps)
            if ramp_steps <= 0:
                raise ValueError(
                    "racket_vel_range_ramp_steps must be > 0 when a velocity start box is set"
                )
            progress, _ = self._velocity_sampling_state()
            start_rng_v = self._vel_start_range_per_clip_t[clip]
            if self._vel_bootstrap_range_per_clip_t is not None:
                bootstrap_rng_v = self._vel_bootstrap_range_per_clip_t[clip]
                start_rng_v = bootstrap_rng_v + progress * (
                    start_rng_v - bootstrap_rng_v
                )
            rng_v = start_rng_v + progress * (rng_v - start_rng_v)
            audit_velocity_box = rng_v
            self.metrics["velocity_core_box_scale"][env_ids_t] = progress
        if sequence_rows is None:
            lo_v, hi_v = rng_v[..., 0], rng_v[..., 1]
            vel = lo_v + (hi_v - lo_v) * torch.rand(n, 3, device=self.device)
        self.racket_target_vel_w[env_ids] = vel

        if self.cfg.normal_mode == "sampled":
            normal = torch.empty(n, 3, device=self.device)
            normal[:, 0] = sample_uniform(*self.cfg.racket_normal_x_range, (n,), self.device)
            normal[:, 1] = sample_uniform(*self.cfg.racket_normal_y_range, (n,), self.device)
            normal[:, 2] = sample_uniform(*self.cfg.racket_normal_z_range, (n,), self.device)
            self.racket_target_normal_w[env_ids] = normal / (
                torch.norm(normal, dim=-1, keepdim=True) + 1e-6
            )
        else:  # "velocity" (paper §IV-C impact model)
            self.racket_target_normal_w[env_ids] = (
                self._normal_from_target_velocity(vel)
            )
        if (
            sequence_rows is None
            and resample_base
            and bool(self.cfg.venue_tuple_enabled)
        ):
            self._apply_correlated_venue_tuple(
                env_ids_t,
                origins,
                clip,
                rng_e,
                previous_station_y,
            )
            vel = self.racket_target_vel_w[env_ids_t]
            off = torch.stack(
                (
                    self.racket_target_pos_w[env_ids_t, 0]
                    - self.base_target_pos_w[env_ids_t, 0],
                    self.racket_target_pos_w[env_ids_t, 1]
                    - self.base_target_pos_w[env_ids_t, 1],
                    self.racket_target_pos_w[env_ids_t, 2]
                    - origins[:, 2],
                ),
                dim=-1,
            )
            venue = self._venue_tuple_selected[env_ids_t]
            audit_velocity_source = torch.where(
                venue,
                torch.ones_like(audit_velocity_source),
                audit_velocity_source,
            )
            audit_velocity_box = torch.where(
                venue[:, None, None],
                self._vel_range_per_clip_t[clip],
                audit_velocity_box,
            )
        # Load-bearing V14 invariant.  A tuple sampler may replace target_velocity, but no
        # planner/contact normal is allowed to change the formal hitter_pure reward target.
        if self.cfg.normal_mode == "velocity":
            self.racket_target_normal_w[env_ids_t] = (
                self._normal_from_target_velocity(
                    self.racket_target_vel_w[env_ids_t]
                )
            )
        self._record_strike_audit_target(
            env_ids_t,
            clip=clip,
            position_offset=off,
            position_box=rng_e,
            velocity=vel,
            velocity_box=audit_velocity_box,
            velocity_source=audit_velocity_source,
            count_start=bool(resample_base),
        )

    @staticmethod
    def _file_sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _normal_from_target_velocity(velocity: torch.Tensor) -> torch.Tensor:
        """Return the HitterV11 V14 face-normal target for a velocity target."""

        return velocity / (
            torch.norm(velocity, dim=-1, keepdim=True) + 1e-6
        )

    def _resolve_venue_tuple_bank_path(self, configured: str) -> Path:
        raw = Path(configured).expanduser()
        candidates = [raw] if raw.is_absolute() else [Path(os.getcwd()) / raw]
        if not raw.is_absolute():
            candidates.extend(parent / raw for parent in Path(__file__).resolve().parents)
        checked: list[str] = []
        for candidate in candidates:
            resolved = candidate.resolve()
            label = str(resolved)
            if label in checked:
                continue
            checked.append(label)
            if resolved.is_file():
                return resolved
        raise FileNotFoundError(
            "V17 physical tuple bank was not found. Checked: " + ", ".join(checked)
        )

    def _load_venue_tuple_bank(self) -> None:
        if self.cfg.normal_mode != "velocity":
            raise RuntimeError(
                "fixed_balanced_bank_v1 requires the V14 velocity-normal contract"
            )
        path = self._resolve_venue_tuple_bank_path(
            str(self.cfg.venue_tuple_bank_path)
        )
        # SHA pinning is optional: validate only when the task config carries
        # the expected digests (the public config ships without them).
        sha_pin_status = "disabled"
        expected_sha = str(
            getattr(self.cfg, "venue_tuple_bank_sha256", "") or ""
        ).strip().lower()
        if expected_sha:
            actual_sha = self._file_sha256(path)
            if actual_sha != expected_sha:
                raise RuntimeError(
                    "V17 physical tuple bank SHA256 mismatch: "
                    f"expected={expected_sha}, actual={actual_sha}, path={path}"
                )
            sha_pin_status = "verified"
        receipt_path = self._resolve_venue_tuple_bank_path(
            str(self.cfg.venue_tuple_bank_receipt_path)
        )
        expected_receipt_sha = str(
            getattr(self.cfg, "venue_tuple_bank_receipt_sha256", "") or ""
        ).strip().lower()
        if expected_receipt_sha:
            actual_receipt_sha = self._file_sha256(receipt_path)
            if actual_receipt_sha != expected_receipt_sha:
                raise RuntimeError(
                    "V17 physical tuple receipt SHA256 mismatch: "
                    f"expected={expected_receipt_sha}, actual={actual_receipt_sha}, "
                    f"path={receipt_path}"
                )
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        if (
            int(receipt.get("schema_version", -1))
            != int(self.cfg.venue_tuple_bank_schema_version)
            or int(receipt.get("contract", {}).get("recipe_revision", -1)) != 12
        ):
            raise RuntimeError(
                "V17 physical tuple receipt does not describe this r12 bank"
            )
        required = {
            "schema_version": (),
            "clip": (None,),
            "station_xy": (None, 2),
            "target_position_offset": (None, 3),
            "target_velocity": (None, 3),
            "target_normal": (None, 3),
            "incoming_velocity": (None, 3),
            "incoming_spin": (None, 3),
            "intended_landing_xy": (None, 2),
            "predicted_landing_xy": (None, 2),
            "outgoing_velocity": (None, 3),
        }
        with np.load(path, allow_pickle=False) as archive:
            missing = sorted(set(required) - set(archive.files))
            if missing:
                raise RuntimeError(
                    f"V17 physical tuple bank is missing arrays: {missing}"
                )
            arrays = {name: np.asarray(archive[name]) for name in required}
        expected_schema = int(self.cfg.venue_tuple_bank_schema_version)
        schema = int(np.asarray(arrays["schema_version"]).item())
        if schema != expected_schema:
            raise RuntimeError(
                f"V17 physical tuple bank schema {schema} != expected {expected_schema}"
            )
        clip = np.asarray(arrays["clip"], dtype=np.int64)
        row_count = int(clip.shape[0])
        for name, shape in required.items():
            value = arrays[name]
            if name == "schema_version":
                if value.shape != ():
                    raise RuntimeError("tuple bank schema_version must be a scalar")
                continue
            expected_shape = tuple(
                row_count if component is None else component for component in shape
            )
            if value.shape != expected_shape:
                raise RuntimeError(
                    f"tuple bank array {name} has shape {value.shape}, expected {expected_shape}"
                )
            if name != "clip" and not bool(np.isfinite(value).all()):
                raise RuntimeError(f"tuple bank array {name} contains NaN/Inf")
        if not bool(np.isin(clip, (0, 1)).all()):
            raise RuntimeError("tuple bank clip array must contain only 0=FH and 1=BH")
        minimum_per_side = int(self.cfg.venue_tuple_bank_min_rows_per_side)
        counts = {side: int(np.count_nonzero(clip == side)) for side in (0, 1)}
        if any(counts[side] < minimum_per_side for side in (0, 1)):
            raise RuntimeError(
                "tuple bank is not balanced/large enough: "
                f"counts={counts}, required_per_side={minimum_per_side}"
            )
        normal_norm = np.linalg.norm(arrays["target_normal"], axis=-1)
        if not bool(np.allclose(normal_norm, 1.0, rtol=0.0, atol=2.0e-4)):
            raise RuntimeError("tuple bank target normals are not unit vectors")
        speed = np.linalg.norm(arrays["target_velocity"], axis=-1)
        if bool((speed > float(self.cfg.venue_tuple_speed_limit_mps) + 1.0e-5).any()):
            raise RuntimeError("tuple bank contains a racket target above the speed limit")

        tensor_fields = (
            "station_xy",
            "target_position_offset",
            "target_velocity",
            "target_normal",
            "incoming_velocity",
            "incoming_spin",
            "intended_landing_xy",
            "predicted_landing_xy",
            "outgoing_velocity",
        )
        for side in (0, 1):
            mask = clip == side
            self._venue_tuple_bank_by_clip[side] = {
                name: torch.from_numpy(
                    np.ascontiguousarray(arrays[name][mask], dtype=np.float32)
                ).to(device=self.device)
                for name in tensor_fields
            }
        self._venue_tuple_bank_path = str(path)
        self._venue_tuple_bank_receipt_path = str(receipt_path)
        print(
            "[RacketTargetCommand] physical tuple bank ON: "
            f"path={path}, sha_pin={sha_pin_status}, FH={counts[0]}, BH={counts[1]}, "
            f"mix={float(self.cfg.venue_tuple_final_mix_prob):.3f}",
            flush=True,
        )

    def _fixed_balanced_tuple_selection(
        self,
        clip: torch.Tensor,
        probability: float,
        env_ids: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Return the permanent global-id cohort, never a reset-local random draw."""

        if env_ids is None:
            if len(clip) != self.num_envs:
                raise ValueError(
                    "partial fixed tuple selection requires explicit global env_ids"
                )
            env_ids = torch.arange(
                self.num_envs, dtype=torch.long, device=self.device
            )
        else:
            env_ids = env_ids.to(device=self.device, dtype=torch.long)
        if len(env_ids) != len(clip):
            raise ValueError("clip and env_ids must have the same length")
        expected_probability = self._venue_tuple_cohort_fraction
        tolerance = 1.0 / max(self.num_envs, 1) + 1.0e-9
        if abs(expected_probability - float(probability)) > tolerance:
            raise RuntimeError(
                "fixed tuple cohort no longer matches venue_tuple_final_mix_prob: "
                f"cohort={expected_probability:.9f}, configured={float(probability):.9f}"
            )
        cohort = self._venue_tuple_cohort[env_ids]
        selected = cohort >= 0
        mismatched = selected & (clip != cohort)
        if bool(mismatched.any()):
            examples = env_ids[mismatched][:8].tolist()
            raise RuntimeError(
                "permanent physical tuple cohort disagrees with MotionCommand clip; "
                f"env_ids={examples}"
            )
        return selected

    def _apply_precomputed_venue_tuple(
        self,
        env_ids: torch.Tensor,
        origins: torch.Tensor,
        clip: torch.Tensor,
        previous_station_y: torch.Tensor,
    ) -> None:
        probability = float(self.cfg.venue_tuple_final_mix_prob)
        self.metrics["venue_tuple_mix_effective"][env_ids] = probability
        selected_local = self._fixed_balanced_tuple_selection(
            clip, probability, env_ids
        )
        for side in (0, 1):
            local = torch.where(selected_local & (clip == side))[0]
            if len(local) == 0:
                continue
            bank = self._venue_tuple_bank_by_clip.get(side)
            if not bank:
                raise RuntimeError(f"tuple bank has no rows for clip {side}")
            bank_count = int(bank["target_velocity"].shape[0])
            row = torch.randint(
                bank_count, (len(local),), device=self.device
            )
            accepted_env = env_ids[local]
            accepted_origins = origins[local]
            station_local = bank["station_xy"][row]
            position_offset = bank["target_position_offset"][row]
            station = accepted_origins[:, :2] + station_local
            contact = accepted_origins + position_offset
            contact[:, :2] = station + position_offset[:, :2]
            self.base_target_pos_w[accepted_env] = station
            self.racket_target_pos_w[accepted_env] = contact
            target_velocity = bank["target_velocity"][row]
            self.racket_target_vel_w[accepted_env] = target_velocity
            self.racket_target_normal_w[accepted_env] = (
                self._normal_from_target_velocity(target_velocity)
            )
            self._venue_planner_contact_normal_w[accepted_env] = bank[
                "target_normal"
            ][row]
            self.vb_vel_in_w[accepted_env] = bank["incoming_velocity"][row]
            self.vb_spin_in_w[accepted_env] = bank["incoming_spin"][row]
            self._venue_intended_landing_xy[accepted_env] = bank[
                "intended_landing_xy"
            ][row]
            self._venue_outgoing_velocity_seed[accepted_env] = bank[
                "outgoing_velocity"
            ][row]
            if bool(self.cfg.venue_tuple_unconditional_outcomes):
                self._venue_tuple_outcome_pending[accepted_env] = True
                self._venue_tuple_outcome_clip[accepted_env] = side
            if self._resample_is_wrap:
                signed_step = station[:, 1] - previous_station_y[local]
                step = torch.abs(signed_step)
                self.metrics["station_y_step_command"][accepted_env] = step
                self.metrics["station_y_step_signed_command"][
                    accepted_env
                ] = signed_step
                self.metrics["station_y_step_class"][accepted_env] = torch.where(
                    step < 1.0e-5,
                    torch.zeros_like(step),
                    torch.where(
                        step < 0.10,
                        torch.ones_like(step),
                        torch.full_like(step, 2.0),
                    ),
                )
            else:
                self.metrics["station_y_step_command"][accepted_env] = 0.0
                self.metrics["station_y_step_signed_command"][accepted_env] = 0.0
                self.metrics["station_y_step_class"][accepted_env] = -1.0
        self.metrics["venue_tuple_selected"][env_ids] = (
            self._venue_tuple_selected[env_ids].float()
        )

    def _apply_correlated_venue_tuple(
        self,
        env_ids: torch.Tensor,
        origins: torch.Tensor,
        clip: torch.Tensor,
        position_box: torch.Tensor,
        previous_station_y: torch.Tensor,
    ) -> None:
        """Replace a configured subset with one coherent planner/venue tuple."""

        if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
            self._apply_precomputed_venue_tuple(
                env_ids, origins, clip, previous_station_y
            )
            return

        from whole_body_tracking.tasks.tracking.mdp.venue_target_tuple import (
            align_normal_to_reference_hemisphere,
            sample_correlated_venue_tuple,
        )

        scale = float(self._recovery_coverage_scale)
        probability = float(self.cfg.venue_tuple_final_mix_prob) * scale
        self.metrics["venue_tuple_mix_effective"][env_ids] = probability
        if probability <= 0.0:
            return
        if probability >= 1.0:
            selected_local = torch.ones(
                len(env_ids), dtype=torch.bool, device=self.device
            )
        else:
            selected_local = (
                torch.rand(len(env_ids), device=self.device) < probability
            )
        pending_local = torch.where(selected_local)[0]
        if len(pending_local) == 0:
            return

        accepted = torch.zeros_like(selected_local)
        maximum_attempts = int(self.cfg.venue_tuple_max_resample_attempts)
        for _attempt in range(maximum_attempts):
            if len(pending_local) == 0:
                break
            pending_clip = clip[pending_local]
            pending_box = position_box[pending_local]
            reach_x = pending_box[:, 0].mean(dim=-1)
            reach_y = pending_box[:, 1].mean(dim=-1)
            contact_x = origins[pending_local, 0] + reach_x
            sample = sample_correlated_venue_tuple(
                pending_clip,
                contact_x,
                table_surface_z=float(self.cfg.vb_table_surface_z),
                landing_x_range=tuple(
                    self.cfg.venue_tuple_landing_x_range
                ),
                landing_y_range=tuple(
                    self.cfg.venue_tuple_landing_y_range
                ),
            )
            contact = sample["contact_pos_w"].clone()
            # The sampler's y/z are env-local venue coordinates; x was supplied in world frame.
            contact[:, 1] += origins[pending_local, 1]
            contact[:, 2] += origins[pending_local, 2]
            station = origins[pending_local, :2].clone()
            station[:, 0] = contact[:, 0] - reach_x
            station[:, 1] = contact[:, 1] - reach_y
            station_local = station - origins[pending_local, :2]
            velocity = sample["racket_velocity_w"]
            speed = torch.linalg.norm(velocity, dim=-1)
            final_velocity_box = self._vel_range_per_clip_t[
                pending_clip
            ]
            position_offset = torch.stack(
                (
                    contact[:, 0] - station[:, 0],
                    contact[:, 1] - station[:, 1],
                    contact[:, 2] - origins[pending_local, 2],
                ),
                dim=-1,
            )
            valid = (
                (station_local[:, 0] >= float(self.cfg.base_target_x_range[0]))
                & (station_local[:, 0] <= float(self.cfg.base_target_x_range[1]))
                & (station_local[:, 1] >= float(self.cfg.base_target_y_range[0]))
                & (station_local[:, 1] <= float(self.cfg.base_target_y_range[1]))
                & (
                    (position_offset >= pending_box[..., 0] - 1.0e-6)
                    & (position_offset <= pending_box[..., 1] + 1.0e-6)
                ).all(dim=-1)
                & (
                    (velocity >= final_velocity_box[..., 0] - 1.0e-6)
                    & (velocity <= final_velocity_box[..., 1] + 1.0e-6)
                ).all(dim=-1)
                & (
                    speed
                    <= float(self.cfg.venue_tuple_speed_limit_mps)
                )
            )
            if bool(valid.any()):
                accepted_local = pending_local[valid]
                accepted_env = env_ids[accepted_local]
                sampled_normal = sample["racket_normal_w"][valid]
                reference_normal = self._ref_normal_per_clip[
                    pending_clip[valid]
                ]
                sampled_normal = align_normal_to_reference_hemisphere(
                    sampled_normal, reference_normal
                )
                self.base_target_pos_w[accepted_env] = station[valid]
                self.racket_target_pos_w[accepted_env] = contact[valid]
                accepted_velocity = velocity[valid]
                self.racket_target_vel_w[accepted_env] = accepted_velocity
                self._venue_planner_contact_normal_w[accepted_env] = (
                    sampled_normal
                )
                if self.cfg.normal_mode == "velocity":
                    self.racket_target_normal_w[accepted_env] = (
                        self._normal_from_target_velocity(accepted_velocity)
                    )
                else:
                    self.racket_target_normal_w[accepted_env] = sampled_normal
                self.vb_vel_in_w[accepted_env] = sample[
                    "incoming_velocity_w"
                ][valid]
                self.vb_spin_in_w[accepted_env] = sample[
                    "incoming_spin_w"
                ][valid]
                self._venue_intended_landing_xy[accepted_env] = sample[
                    "intended_landing_xy_w"
                ][valid]
                self._venue_outgoing_velocity_seed[accepted_env] = sample[
                    "outgoing_velocity_seed_w"
                ][valid]
                self._venue_tuple_selected[accepted_env] = True
                if self._resample_is_wrap:
                    signed_step = (
                        station[valid, 1]
                        - previous_station_y[accepted_local]
                    )
                    step = torch.abs(signed_step)
                    self.metrics["station_y_step_command"][
                        accepted_env
                    ] = step
                    self.metrics["station_y_step_signed_command"][
                        accepted_env
                    ] = signed_step
                    self.metrics["station_y_step_class"][
                        accepted_env
                    ] = torch.where(
                        step < 1.0e-5,
                        torch.zeros_like(step),
                        torch.where(
                            step < 0.10,
                            torch.ones_like(step),
                            torch.full_like(step, 2.0),
                        ),
                    )
                else:
                    self.metrics["station_y_step_command"][
                        accepted_env
                    ] = 0.0
                    self.metrics["station_y_step_signed_command"][
                        accepted_env
                    ] = 0.0
                    self.metrics["station_y_step_class"][
                        accepted_env
                    ] = -1.0
                accepted[accepted_local] = True
            pending_local = pending_local[~valid]
        # Exhausted tuples fall back to the already sampled V11 target. This is fail-soft for
        # training but explicit in telemetry; phase-0 feasibility requires >=99% acceptance.
        self.metrics["venue_tuple_selected"][env_ids] = (
            self._venue_tuple_selected[env_ids].float()
        )

    def _resolve_pending_venue_outcomes_as_failures(
        self, env_ids: torch.Tensor
    ) -> None:
        """Close tuple attempts that reset before their exact strike frame."""

        if not bool(self.cfg.venue_tuple_unconditional_outcomes):
            return
        pending = env_ids[self._venue_tuple_outcome_pending[env_ids]]
        if len(pending) == 0:
            return
        # Motion has already sampled the next clip when command terms reset, so
        # attribute the failed attempt with its stored source clip.
        source_clip = self._venue_tuple_outcome_clip[pending]
        for side in self._clip_names:
            self._venue_swing_starts_acc_c[side] += float(
                (source_clip == side).sum()
            )
        self._venue_tuple_outcome_pending[pending] = False
        self._venue_tuple_outcome_clip[pending] = -1

    def _sample_targets_reference_perturbed(self, env_ids: Sequence[int], origins: torch.Tensor, n: int):
        """Target = this env's reference racket state @ strike + curriculum-scaled perturbation.

        The target center is selected by ``motion.clip_id`` for unified forehand+backhand training, so
        the policy no longer has to imitate one teacher strike while chasing a different sampled target.
        """
        self._ensure_reference_strike_state()
        assert self._ref_racket_pos_rel_per_clip is not None
        assert self._ref_racket_vel_w_per_clip is not None
        assert self._ref_racket_normal_w_per_clip is not None
        motion = self._motion()
        if motion._multiseg:
            clip = motion.clip_id[env_ids]
        else:
            clip = torch.zeros(n, dtype=torch.long, device=self.device)
        ref_pos = self._ref_racket_pos_rel_per_clip[clip]
        ref_vel = self._ref_racket_vel_w_per_clip[clip]
        ref_nrm = self._ref_racket_normal_w_per_clip[clip]

        scale = self._perturb_scale()
        dev = self.device
        pos_h = torch.tensor(self.cfg.ref_perturb_pos, device=dev) * scale
        vel_h = torch.tensor(self.cfg.ref_perturb_vel, device=dev) * scale
        nrm_h = float(self.cfg.ref_perturb_normal) * scale

        dpos = (torch.rand(n, 3, device=dev) * 2.0 - 1.0) * pos_h
        self.racket_target_pos_w[env_ids] = origins + ref_pos + dpos

        dvel = (torch.rand(n, 3, device=dev) * 2.0 - 1.0) * vel_h
        self.racket_target_vel_w[env_ids] = ref_vel * self.cfg.ref_vel_scale + dvel

        dnrm = (torch.rand(n, 3, device=dev) * 2.0 - 1.0) * nrm_h
        normal = ref_nrm + dnrm
        self.racket_target_normal_w[env_ids] = normal / (torch.norm(normal, dim=-1, keepdim=True) + 1e-6)

        self.metrics["ref_perturb_scale"][env_ids] = scale

    @staticmethod
    def _markov_replay_tensor_fields() -> tuple[str, ...]:
        """Per-environment command state needed to resume one live recovery."""

        return (
            "racket_target_pos_w",
            "racket_target_vel_w",
            "racket_target_normal_w",
            "base_target_pos_w",
            "swing_sign",
            "delayed_racket_target_pos_w",
            "delayed_racket_target_vel_w",
            "delayed_swing_sign",
            "vb_vel_in_w",
            "vb_spin_in_w",
            "_venue_tuple_selected",
            "_venue_tuple_outcome_pending",
            "_venue_tuple_outcome_clip",
            "_venue_intended_landing_xy",
            "_venue_planner_contact_normal_w",
            "_venue_outgoing_velocity_seed",
            "_strike_audit_context_id",
            "_recover_strike_audit_context_id",
            "_target_velocity_source",
            "_target_speed_quartile",
            "_target_z_bin",
            "_foundation_context_id",
            "_foundation_recovery_context_id",
            "_prev_motion_steps",
            "_prev_clip_id",
            "_recover_from_clip",
            "_post_strike_capture_prev_tts",
            "_post_strike_risk_capture_mask",
            "_prev_racket_dist",
            "racket_progress",
            "_progress_reset_mask",
            "_ready_elapsed_steps",
            "_ready_dwell_steps",
            "_ready_latched",
            "_ready_ever_ready",
            "_ready_prev_held",
            "_ready_transition_eligible",
            "_ready_latency_s",
            "_ready_release_required",
            "_ready_release_wait_steps",
            "ready_release_timeout",
            "_safe_recovery_pending",
            "_safe_recovery_source_clip",
            "_chain_clip_id",
            "_chain_step_ok",
            "_chain_ready_ok",
            "_chain_released",
            "_chain_exact_recorded",
            "_chain_recovery_pending",
            "_chain_recovery_clip_id",
            "_chain_recovery_distance_bin",
            "_recovery_timer_active",
            "_recovery_age_s",
            "_recovery_strike_xy",
            "_recovery_event_seen",
            "_foundation_prev_tts",
            "_swing_start_base_xy",
            "_swing_start_pending",
            "_locomotion_planned_cycles",
            "_locomotion_duration_steps",
            "_locomotion_elapsed_steps",
            "_locomotion_initial_delta_y",
            "_locomotion_velocity_y",
            "_locomotion_gait_active",
            "_locomotion_move",
            "_locomotion_supervision",
            "_gait_clock",
            "_desired_contact_states",
            "_station_command_id",
            "_step_bout_started",
            "_step_bout_complete",
            "_step_settle_dwell_count",
            "_station_command_age_steps",
            "_station_command_start_xy",
            "_station_command_target_xy",
            "_station_distance_bin",
            "_previous_step_mode",
            "_previous_lateral_direction",
            "_previous_foot_contact",
            "_one_step_release_recorded",
        )

    def capture_markov_replay_state(
        self, env_ids: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        """Capture target, release, and recovery state paired with a physical snapshot."""

        env_ids = env_ids.to(device=self.device, dtype=torch.long)
        state: dict[str, torch.Tensor] = {}
        for name in self._markov_replay_tensor_fields():
            value = getattr(self, name, None)
            if not (
                torch.is_tensor(value)
                and value.ndim >= 1
                and value.shape[0] == self.num_envs
            ):
                raise RuntimeError(
                    f"V17 target Markov field {name!r} is missing or not "
                    "per-environment"
                )
            state[name] = value[env_ids].clone()

        for name in (
            "_mnoise_ar1_state",
            "_swing_bias",
            "_drop_cd",
            "_prev_pre_strike",
            "_held_pos",
            "_held_vel",
        ):
            value = getattr(self, name, None)
            if torch.is_tensor(value):
                state[name] = value[env_ids].clone()

        if self._delay_steps > 0:
            length = self._delay_steps + 1
            order = [
                (self._delay_ptr + offset) % length
                for offset in range(length)
            ]
            state["delay_history_pos"] = self._delay_buf_pos[
                order
            ][:, env_ids].permute(1, 0, 2).clone()
            state["delay_history_vel"] = self._delay_buf_vel[
                order
            ][:, env_ids].permute(1, 0, 2).clone()
            state["delay_history_sign"] = self._delay_buf_sign[
                order
            ][:, env_ids].permute(1, 0).clone()
        return state

    def restore_markov_replay_state(
        self,
        env_ids: torch.Tensor,
        state: dict[str, torch.Tensor],
    ) -> None:
        """Restore the exact target/READY/recovery state for a live replay continuation."""

        env_ids = env_ids.to(device=self.device, dtype=torch.long)
        required = set(self._markov_replay_tensor_fields())
        missing = sorted(required - set(state))
        if missing:
            raise RuntimeError(
                f"V17 target Markov replay is missing fields: {missing}"
            )
        for name, value in state.items():
            if name.startswith("delay_history_"):
                continue
            destination = getattr(self, name, None)
            if not (
                torch.is_tensor(destination)
                and destination.ndim >= 1
                and destination.shape[0] == self.num_envs
                and tuple(value.shape[1:])
                == tuple(destination.shape[1:])
                and value.dtype == destination.dtype
            ):
                raise RuntimeError(
                    f"V17 target Markov restore field {name!r} does not "
                    "match the live tensor contract"
                )
            destination[env_ids] = value

        # Re-establish the invariant after every replay restore, before the continuation can be
        # observed or scored.  The planner-only normal remains in its separate audit buffer.
        if (
            self.cfg.target_mode == "hitter_pure"
            and self.cfg.normal_mode == "velocity"
        ):
            self.racket_target_normal_w[env_ids] = (
                self._normal_from_target_velocity(
                    self.racket_target_vel_w[env_ids]
                )
            )

        if self._delay_steps > 0:
            for name in (
                "delay_history_pos",
                "delay_history_vel",
                "delay_history_sign",
            ):
                if name not in state:
                    raise RuntimeError(
                        f"V17 target Markov replay is missing {name}"
                    )
            length = self._delay_steps + 1
            order = [
                (self._delay_ptr + offset) % length
                for offset in range(length)
            ]
            for offset, slot in enumerate(order):
                self._delay_buf_pos[slot, env_ids] = state[
                    "delay_history_pos"
                ][:, offset]
                self._delay_buf_vel[slot, env_ids] = state[
                    "delay_history_vel"
                ][:, offset]
                self._delay_buf_sign[slot, env_ids] = state[
                    "delay_history_sign"
                ][:, offset]

    def _resample_command(self, env_ids: Sequence[int]):
        if len(env_ids) == 0:
            return
        n = len(env_ids)
        origins = self._env.scene.env_origins[env_ids]
        motion = self._motion()
        ids = torch.as_tensor(
            env_ids, dtype=torch.long, device=self.device
        )
        self._resolve_pending_venue_outcomes_as_failures(ids)
        self._venue_planner_contact_normal_w[ids] = 0.0
        if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
            self._venue_tuple_selected[ids] = (
                self._venue_tuple_cohort[ids] >= 0
            )
        else:
            self._venue_tuple_selected[ids] = False
        self.metrics["venue_tuple_selected"][ids] = (
            self._venue_tuple_selected[ids].float()
        )
        self.metrics["venue_tuple_mix_effective"][ids] = 0.0
        self.metrics["venue_tuple_accept_rate"][ids] = (
            1.0
            if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1"
            else 0.0
        )
        self.metrics["venue_tuple_fallback_rate"][ids] = 0.0
        action_term = self._env.action_manager.get_term("joint_pos")
        fault_counter = getattr(
            action_term, "_actual_q_fault_event_counter", None
        )
        if torch.is_tensor(fault_counter):
            fault_delta = (
                fault_counter[ids]
                - self._actual_q_fault_counter_seen[ids]
            )
            if bool((fault_delta < 0).any()):
                raise RuntimeError(
                    "actual-q fault event counter moved backwards"
                )
            source_clip = self._prev_clip_id[ids]
            for clip_id in self._clip_names:
                side_faults = float(
                    fault_delta[source_clip == clip_id].sum()
                )
                self._actual_q_fault_acc_c[clip_id] += side_faults
                if int(clip_id) < self._actual_q_window_pending_faults.numel():
                    self._actual_q_window_pending_faults[int(clip_id)] += (
                        side_faults
                    )
            self._actual_q_fault_counter_seen[ids] = fault_counter[ids]
        # A true reset before the next READY release resolves any pending post-hit recovery as a
        # failure. A normal clip wrap merely starts that recovery window and must not resolve it.
        if not self._resample_is_wrap:
            pending = ids[self._safe_recovery_pending[ids]]
            if len(pending) > 0:
                self._resolve_safe_recovery_events(
                    pending,
                    torch.zeros(
                        len(pending),
                        dtype=torch.bool,
                        device=self.device,
                    ),
                )

        replay_mask = (
            motion.post_swing_replay_active[ids]
            if (
                getattr(motion, "_post_swing_replay_contract", "")
                == "markov_side_phase_severity_v3"
            )
            else torch.zeros(
                len(ids), dtype=torch.bool, device=self.device
            )
        )
        replay_ids = ids[replay_mask]
        if len(replay_ids) > 0:
            replay_state = motion.consume_markov_replay_target_state(
                replay_ids
            )
            self.restore_markov_replay_state(
                replay_ids, replay_state
            )
            if self._venue_tuple_mix_mode == "fixed_balanced_bank_v1":
                self._venue_tuple_selected[replay_ids] = (
                    self._venue_tuple_cohort[replay_ids] >= 0
                )
            self.metrics["venue_tuple_selected"][replay_ids] = (
                self._venue_tuple_selected[replay_ids].float()
            )
            self.metrics["venue_tuple_mix_effective"][replay_ids] = (
                float(self.cfg.venue_tuple_final_mix_prob)
                * (
                    1.0
                    if self._venue_tuple_mix_mode
                    == "fixed_balanced_bank_v1"
                    else float(self._recovery_coverage_scale)
                )
            )
            self._prev_motion_steps[replay_ids] = motion.time_steps[
                replay_ids
            ]
            self._prev_racket_dist[replay_ids] = torch.linalg.norm(
                self.racket_pos_w[replay_ids]
                - self.racket_target_pos_w[replay_ids],
                dim=-1,
            ).detach()
            self.racket_progress[replay_ids] = 0.0
            self._progress_reset_mask[replay_ids] = True

            normal_ids = ids[~replay_mask]
            if len(normal_ids) == 0:
                return
            env_ids = normal_ids
            ids = normal_ids
            n = len(normal_ids)
            origins = self._env.scene.env_origins[normal_ids]

        # A normal target resample starts a new swing. Replay continuations returned above retain
        # their captured edge-detector/mask state; every genuinely new swing starts clean.
        self._post_strike_capture_prev_tts[ids] = 1.0e9
        self._post_strike_risk_capture_mask[ids] = False

        # UNCONDITIONAL swing accounting: every resample STARTS a new swing attempt. On the
        # true-reset path (not a wrap) it also ENDS the previous attempt — count a pre-strike
        # fall if the env terminated before reaching the strike frame.
        self._count_swing_starts(env_ids, count_prestrike_falls=not self._resample_is_wrap)

        # Desired racket pos/vel/normal — independent box sampling (legacy uniform), coupled to the
        # reference swing's strike state (reference_perturbed), or HITTER-faithful station-first
        # sampling (hitter_pure: base station independent, racket plane fixed relative to the station).
        if self.cfg.target_mode == "reference_perturbed":
            self._sample_targets_reference_perturbed(env_ids, origins, n)
        elif self.cfg.target_mode == "hitter_pure":
            self._sample_targets_hitter_pure(env_ids, origins, n)
        else:
            self._sample_targets_uniform(env_ids, origins, n)

        # Desired base XY (world): COUPLE it to the racket target so standing there keeps the racket
        # reachable by the imitated swing — base_target = racket_target_xy - (reference base->racket
        # offset). Independent sampling used to fight the arm's reach (the base_position reward pulled
        # the base away from where the racket needed it). base_target_*_range is now a SMALL JITTER
        # around the coupled point. Legacy "uniform" mode keeps the old origin-relative sampling.
        # hitter_pure: the station was already sampled INDEPENDENTLY inside the sampler (paper
        # §V-B-1 order: station first, racket plane relative to it) — skip every coupling path AND
        # the jitter add below (base_target_*_range already served as the station box).
        if self.cfg.target_mode == "hitter_pure":
            pass
        elif self.cfg.target_mode == "reference_perturbed":
            self._ensure_reference_strike_state()
            assert self._ref_reach_offset_xy_per_clip is not None
            if motion._multiseg:
                clip = motion.clip_id[env_ids]
            else:
                clip = torch.zeros(n, dtype=torch.long, device=self.device)
            base_xy = self.racket_target_pos_w[env_ids][:, :2] - self._ref_reach_offset_xy_per_clip[clip]
        elif self.cfg.base_couple_mode == "reference_reach":
            # uniform + HITTER separate-commands coupling (§V-B-1): base_target = racket_target_xy −
            # (reference base→racket strike offset). Same derivation as the reference_perturbed branch
            # above, but the racket target keeps the proven uniform box distribution (warm-start
            # friendly). Standing at the commanded station = racket target at the clip's reference
            # reach, so the striking plane is fixed RELATIVE TO THE COMMANDED BASE and the x-span of
            # the box moves the STATION, not the reach depth. The jitter below (base_target_*_range)
            # trains the policy to strike with the station deliberately offset — y-reach diversity.
            self._ensure_reference_strike_state()
            assert self._ref_reach_offset_xy_per_clip is not None
            if motion._multiseg:
                clip = motion.clip_id[env_ids]
            else:
                clip = torch.zeros(n, dtype=torch.long, device=self.device)
            base_xy = self.racket_target_pos_w[env_ids][:, :2] - self._ref_reach_offset_xy_per_clip[clip]
        else:
            # uniform: start at spawn, then WEAKLY couple the base toward the racket target's SIDEWAYS
            # offset (Y only; X is the fixed strike plane, so no forward repositioning). The base shifts a
            # fraction (base_couple_blend) of the target's Y offset, clamped to ±base_couple_max_offset, so
            # the robot leans/steps slightly toward far targets instead of stretching in place. blend=0 ->
            # the old spawn-only behaviour. This only moves a REWARD target — the racket target distribution
            # is unchanged. (No walking reference exists, so keep the blend small: it fights leg imitation.)
            base_xy = origins[:, :2].clone()
            blend = float(self.cfg.base_couple_blend)
            if blend > 0.0:
                racket_y_off = self.racket_target_pos_w[env_ids][:, 1] - origins[:, 1]
                base_xy[:, 1] += (blend * racket_y_off).clamp(
                    -self.cfg.base_couple_max_offset, self.cfg.base_couple_max_offset
                )
        if self.cfg.target_mode != "hitter_pure":
            # hitter_pure sampled the station inside the sampler; base_target_*_range was the
            # station box there, NOT a jitter — adding it again would double-sample.
            base_xy[:, 0] += sample_uniform(*self.cfg.base_target_x_range, (n,), self.device)
            base_xy[:, 1] += sample_uniform(*self.cfg.base_target_y_range, (n,), self.device)
            self.base_target_pos_w[env_ids] = base_xy

        # Translate HITTER's newly sampled station into one bounded HUGWBC gait command.  On a
        # wrap the current base is the previous station estimate; on a true reset the post-reset
        # base belongs at the environment origin even though the command cache can still contain
        # the pre-reset pose.  Planning from the origin avoids booking a reset teleport as a step.
        self._resample_locomotion_command(env_ids, origins)

        # Swing type. Unified multi-clip: it IS the imitated clip (forehand=clip 0 -> +1, backhand=clip 1
        # -> -1), matching the swing_type observation. Single-clip legacy: infer from the target Y side.
        if motion._multiseg:
            clip = motion.clip_id[env_ids]
            self.swing_sign[env_ids] = torch.where(clip == 0, 1.0, -1.0)
            # Start a fresh safety-minimum accumulator only for a newly armed backhand.  Forehands
            # leave the previous backhand result visible to tail-averaged evaluation.
            ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            bh_ids = ids_t[clip == 1]
            if len(bh_ids) > 0:
                self._bh_hand_min[bh_ids] = 1.0
                self._bh_forearm_min[bh_ids] = 1.0
                self._bh_left_arm_min[bh_ids] = 1.0
        else:
            base_y_nom = origins[:, 1] + self.cfg.base_nominal_offset[1]
            dy = self.racket_target_pos_w[env_ids][:, 1] - base_y_nom
            if self.cfg.forehand_on_negative_y:
                self.swing_sign[env_ids] = torch.where(dy <= 0.0, 1.0, -1.0)
            else:
                self.swing_sign[env_ids] = torch.where(dy >= 0.0, 1.0, -1.0)

        # Tier-1 virtual incoming ball: one (v_in, omega_in) per swing. The ball's position at the
        # strike time is the racket target BY CONSTRUCTION (the sampler defines the ball to arrive
        # there), so only velocity + spin are sampled. Boxes stay inside the venue-fit envelope.
        if self.cfg.virtual_ball:
            core_ids = ids[~self._venue_tuple_selected[ids]]
            core_n = len(core_ids)
            if core_n > 0:
                self.vb_vel_in_w[core_ids, 0] = sample_uniform(
                    *self.cfg.vb_vel_x_range, (core_n,), self.device
                )
                self.vb_vel_in_w[core_ids, 1] = sample_uniform(
                    *self.cfg.vb_vel_y_range, (core_n,), self.device
                )
                self.vb_vel_in_w[core_ids, 2] = sample_uniform(
                    *self.cfg.vb_vel_z_range, (core_n,), self.device
                )
                _s = float(self.cfg.vb_spin_abs_max)
                self.vb_spin_in_w[core_ids] = sample_uniform(
                    -_s, _s, (core_n, 3), self.device
                )

        # Rally drift accounting: base->NEW-station error at swing start (the recovery debt the
        # previous swing left). Wrap path only — at true resets base_pos_w still caches the
        # pre-teleport pose (the lazy-stamp rationale in __init__), which would book the reset
        # teleport as recovery debt. Denominator: _drift_n_acc (same wrap-only event count).
        if self._resample_is_wrap and n > 0:
            _ids_so = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            _off = torch.norm(self.base_pos_w[_ids_so, :2] - self.base_target_pos_w[_ids_so], dim=-1)
            self._station_offset_start_sum_acc += float(_off.sum())
        # (v2 heading debt is sampled at HOLD EXPIRY in _update_metrics — the wrap instant would
        # read the PRE-recovery debt and miss stand-start holds.)

        # Stamp the motion phase baseline for these envs so the per-swing wrap detector in
        # _update_command does not immediately re-trigger after this (e.g. reset-time) resample.
        self._prev_motion_steps[env_ids] = self._motion().time_steps[env_ids]
        self._prev_racket_dist[env_ids] = torch.norm(
            self.racket_pos_w[env_ids] - self.racket_target_pos_w[env_ids], dim=-1
        ).detach()
        self.racket_progress[env_ids] = 0.0
        self._progress_reset_mask[env_ids] = True

        # A new fixed-clock station command starts a fresh runner-readiness measurement. MotionCommand
        # has already sampled hold_counter for this reset/wrap. Do not inspect arrival here: base_pos_w
        # may still be the pre-reset cache on true resets, and the first live _update_metrics step is the
        # honest starting sample.
        ids_ready = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self._ready_elapsed_steps[ids_ready] = 0
        self._ready_dwell_steps[ids_ready] = 0
        self._ready_latched[ids_ready] = False
        self._ready_ever_ready[ids_ready] = False
        self._ready_latency_s[ids_ready] = 0.0
        self._ready_prev_held[ids_ready] = self._motion().hold_counter[ids_ready] > 0
        step_cmd = self.metrics["station_y_step_command"][ids_ready]
        ready_step_lo, ready_step_hi = self.cfg.ready_monitor_step_range
        self._ready_transition_eligible[ids_ready] = (
            bool(self._resample_is_wrap)
            & (step_cmd >= float(ready_step_lo) - 1e-6)
            & (step_cmd <= float(ready_step_hi) + 1e-6)
        )
        self.ready_release_timeout[ids_ready] = False
        self._ready_release_wait_steps[ids_ready] = 0
        # Sample one reversible release question per swing. Scale 0 executes no RNG and is exact
        # V11; scale 1 selects all swings.
        release_scale = (
            float(self._recovery_current_scale)
            if bool(self.cfg.ready_release_enabled)
            else 0.0
        )
        if release_scale <= 0.0:
            required = torch.zeros(
                len(ids_ready), dtype=torch.bool, device=self.device
            )
        elif release_scale >= 1.0:
            required = torch.ones(
                len(ids_ready), dtype=torch.bool, device=self.device
            )
        else:
            required = (
                torch.rand(len(ids_ready), device=self.device)
                < release_scale
            )
        self._ready_release_required[ids_ready] = required
        if bool(required.any()):
            selected = ids_ready[required]
            selected_clip = (
                motion.clip_id[selected]
                if motion._multiseg
                else torch.zeros(
                    len(selected), dtype=torch.long, device=self.device
                )
            )
            phase_values = tuple(self.cfg.strike_phase_per_clip)
            if (
                phase_values
                and len(phase_values) == motion.motion.num_segments
            ):
                phase = torch.tensor(
                    phase_values, device=self.device
                )[selected_clip]
            else:
                phase = torch.full(
                    (len(selected),),
                    float(self.cfg.strike_phase),
                    device=self.device,
                )
            if motion._multiseg:
                segment_start = motion.motion.seg_start[selected_clip]
                segment_length = motion.motion.seg_len[selected_clip]
            else:
                segment_start = torch.zeros_like(selected_clip)
                segment_length = torch.full_like(
                    selected_clip, int(motion.motion.time_step_total)
                )
            strike_step = segment_start + torch.round(
                phase * (segment_length - 1).float()
            ).long()
            arm_ticks = int(
                round(
                    float(self.cfg.ready_release_arm_tts_s)
                    / float(self._env.step_dt)
                )
            )
            motion.time_steps[selected] = torch.maximum(
                segment_start, strike_step - arm_ticks
            )
        self.metrics["ready_release_required"][ids_ready] = (
            required.float()
        )
        self.metrics["ready_release_wait_s"][ids_ready] = 0.0
        self.metrics["ready_release_timeout_event"][ids_ready] = 0.0

        # A1 target latency: a TRUE reset (not an intra-episode wrap) starts a fresh "deploy
        # session" — the runner latches the first planner target before the policy steps, so the
        # actor-visible view (and the whole ring buffer) is backfilled with the fresh target: no
        # cross-episode target leakage. Intra-episode WRAPS are deliberately NOT backfilled — the
        # next swing's target reaching the actor `delay` steps late is exactly the latency modeled.
        if self._actor_view_active and not self._resample_is_wrap:
            ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            self.delayed_racket_target_pos_w[ids] = self.racket_target_pos_w[ids]
            self.delayed_racket_target_vel_w[ids] = self.racket_target_vel_w[ids]
            self.delayed_swing_sign[ids] = self.swing_sign[ids]
            if self._delay_steps > 0:
                self._delay_buf_pos[:, ids] = self.racket_target_pos_w[ids].unsqueeze(0)
                self._delay_buf_vel[:, ids] = self.racket_target_vel_w[ids].unsqueeze(0)
                self._delay_buf_sign[:, ids] = self.swing_sign[ids].unsqueeze(0)
        if self._base_mocap_enabled and not self._resample_is_wrap:
            ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            self._base_mocap_reset_pending[ids] = True
            self._base_mocap_residual_needs_sample[ids] = True

        # The truth instrument is armed only after every field of the new question has been
        # sampled.  This hook is telemetry-only and cannot mutate the command buffers.
        if self._physical is not None:
            self._physical.on_resample(env_ids)

    def _resolve_safe_recovery_events(
        self, env_ids: torch.Tensor, success: torch.Tensor
    ) -> None:
        """Resolve pending exact-hit recovery questions without side averaging."""

        if len(env_ids) == 0:
            return
        env_ids = env_ids.to(device=self.device, dtype=torch.long)
        success = success.to(device=self.device, dtype=torch.bool)
        if tuple(success.shape) != (len(env_ids),):
            raise ValueError(
                "safe-recovery success must have one value per environment"
            )
        source = self._safe_recovery_source_clip[env_ids]
        for clip_id in self._clip_names:
            selected = source == int(clip_id)
            count = int(selected.sum())
            if count > 0:
                self._safe_recovery_n_ema_c[clip_id] += float(count)
                self._safe_recovery_pass_ema_c[clip_id] += float(
                    success[selected].sum()
                )
        self._safe_recovery_pending[env_ids] = False

    def _apply_ready_release_gate(self, motion) -> None:
        """Re-floor the final hold tick until strict READY or timeout."""

        if not bool(self.cfg.ready_release_enabled):
            return
        held_metric = motion.metrics.get("in_hold")
        was_held = (
            held_metric > 0.5
            if torch.is_tensor(held_metric)
            else torch.zeros(
                self.num_envs, dtype=torch.bool, device=self.device
            )
        )
        deadline = (
            self._ready_release_required
            & was_held
            & (motion.hold_counter == 0)
            & (~self.ready_release_timeout)
        )
        waiting = deadline & (~self._ready_latched)
        if bool(waiting.any()):
            self._ready_release_wait_steps[waiting] += 1
            timeout_ticks = max(
                1,
                int(
                    math.ceil(
                        float(self.cfg.ready_release_timeout_s)
                        / float(self._env.step_dt)
                    )
                ),
            )
            timed_out = waiting & (
                self._ready_release_wait_steps >= timeout_ticks
            )
            keep_waiting = waiting & (~timed_out)
            motion.hold_counter[keep_waiting] = 1
            if bool(timed_out.any()):
                # Never execute a moving-base strike on a miss. Hold the arm point until the
                # timeout termination resets the question.
                motion.hold_counter[timed_out] = 1
                self.ready_release_timeout[timed_out] = True
                self.metrics["ready_release_timeout_event"][
                    timed_out
                ] = 1.0
                if motion._multiseg:
                    for clip_id in self._clip_names:
                        side_timeouts = float(
                            (
                                timed_out & (motion.clip_id == clip_id)
                            ).float().sum()
                        )
                        self._ready_release_timeout_count[clip_id] += (
                            side_timeouts
                        )
                        self._ready_release_timeout_acc_c[clip_id] += (
                            side_timeouts
                        )
                pending = torch.where(
                    timed_out & self._safe_recovery_pending
                )[0]
                if len(pending) > 0:
                    self._resolve_safe_recovery_events(
                        pending,
                        torch.zeros(
                            len(pending),
                            dtype=torch.bool,
                            device=self.device,
                        ),
                    )
        self.metrics["ready_release_wait_s"] = (
            self._ready_release_wait_steps.float()
            * float(self._env.step_dt)
        )

    def _one_step_distance_bucket(self, delta_y: torch.Tensor) -> torch.Tensor:
        distance = torch.abs(delta_y)
        bucket = torch.full_like(distance, 4, dtype=torch.long)
        bucket = torch.where(distance <= 0.30 + 1.0e-9, 3, bucket)
        bucket = torch.where(distance <= 0.20 + 1.0e-9, 2, bucket)
        bucket = torch.where(distance <= 0.10 + 1.0e-9, 1, bucket)
        return torch.where(distance <= 1.0e-6, torch.zeros_like(bucket), bucket)

    def _chain_index_add(
        self,
        counter: torch.Tensor,
        env_ids: torch.Tensor,
        values: torch.Tensor,
        *,
        recovery_context: bool = False,
    ) -> None:
        """Accumulate a per-env event into the fixed FH/BH x distance matrix."""

        if env_ids.numel() == 0:
            return
        if recovery_context:
            clip = self._chain_recovery_clip_id[env_ids]
            bucket = self._chain_recovery_distance_bin[env_ids]
        else:
            clip = self._chain_clip_id[env_ids]
            bucket = self._station_distance_bin[env_ids]
        flat_index = clip.clamp(0, 1) * 5 + bucket.clamp(0, 4)
        counter.view(-1).index_add_(0, flat_index, values)

    def _finalize_one_step_commands(
        self,
        env_ids: torch.Tensor,
        *,
        released_to_swing: bool,
        ready_release_pass: torch.Tensor | None = None,
    ) -> None:
        """Book each command once; an unreleased reset is a failed step attempt.

        ``ready_release_pass`` is the same-tick strict READY result captured from the
        pre-decrement hold metric.  Passing it explicitly prevents command finalization from
        racing the readiness monitor by one control tick.
        """

        if not self._one_step_enabled or env_ids.numel() == 0:
            return
        if ready_release_pass is None:
            ready_release_pass = torch.zeros(
                self.num_envs, dtype=torch.bool, device=self.device
            )
        elif (
            ready_release_pass.shape != (self.num_envs,)
            or ready_release_pass.dtype != torch.bool
        ):
            raise ValueError(
                "ready_release_pass must be one bool per environment, got "
                f"shape={tuple(ready_release_pass.shape)}, "
                f"dtype={ready_release_pass.dtype}"
            )
        ids = env_ids
        # Reset callbacks run after termination evaluation. Fold the exact
        # physical-fall done terms into the command latch before booking it.
        termination_manager = getattr(self._env, "termination_manager", None)
        if termination_manager is not None:
            for term_name in ("base_fell_tilt", "base_too_low"):
                try:
                    term_done = termination_manager.get_term(term_name).bool()
                except (AttributeError, KeyError, RuntimeError, ValueError):
                    continue
                self._one_step_fall_before_swing[ids] |= term_done[ids]
        valid = (self._station_command_id[ids] >= 0) & (
            ~self._one_step_release_recorded[ids]
        )
        ids = ids[valid]
        if ids.numel() == 0:
            return
        bucket = self._station_distance_bin[ids]
        ones = torch.ones(ids.numel(), device=self.device)
        self._one_step_command_count.index_add_(0, bucket, ones)
        planned = self._locomotion_duration_steps[ids] > 0
        planned_ids = ids[planned]
        planned_bucket = bucket[planned]
        if planned_ids.numel() > 0:
            planned_ones = torch.ones(planned_ids.numel(), device=self.device)
            self._one_step_attempt_count.index_add_(
                0, planned_bucket, planned_ones
            )
            self._chain_index_add(
                self._chain_attempt_count, planned_ids, planned_ones
            )
            live_error = torch.linalg.norm(
                self.base_pos_w[planned_ids, :2]
                - self._station_command_target_xy[planned_ids],
                dim=-1,
            )
            live_speed = torch.linalg.norm(
                self.robot.data.root_lin_vel_w[planned_ids, :2], dim=-1
            )
            q = self.base_quat_w[planned_ids]
            yaw = torch.abs(
                torch.atan2(
                    2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3]),
                    1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
                )
            )
            exit_error = torch.nan_to_num(
                self._step_exit_station_error[planned_ids], nan=0.0
            )
            exit_speed = torch.nan_to_num(
                self._step_exit_base_speed[planned_ids], nan=0.0
            )
            exit_yaw = torch.nan_to_num(
                self._step_exit_yaw_error[planned_ids], nan=0.0
            )
            exit_time = torch.nan_to_num(
                self._time_to_stand[planned_ids], nan=0.0
            )
            completed = self._step_bout_complete[planned_ids]
            exit_error = torch.where(completed, exit_error, live_error)
            exit_speed = torch.where(completed, exit_speed, live_speed)
            exit_yaw = torch.where(completed, exit_yaw, yaw)
            exit_time = torch.where(
                completed,
                exit_time,
                self._station_command_age_steps[planned_ids].float()
                * float(self._env.step_dt),
            )
            success = (
                bool(released_to_swing)
                & (self._step_bout_count[planned_ids] == 1)
                & (self._step_reentry[planned_ids] == 0)
                & completed
                & (exit_error <= self._step_settle_pos_thresh)
                & (exit_speed <= self._step_settle_speed_thresh)
                & (exit_yaw <= self._step_settle_yaw_thresh)
                & self._step_exit_both_feet[planned_ids]
                & (
                    self._step_exit_slip_speed[planned_ids]
                    <= self._step_settle_slip_thresh
                )
                & (self._one_step_safety_recovery_tag[planned_ids] == 0)
                & (~self._one_step_fall_before_swing[planned_ids])
            )
            self._one_step_success_count.index_add_(
                0, planned_bucket, success.float()
            )
            ready_success = success & ready_release_pass[planned_ids]
            self._chain_index_add(
                self._chain_step_settled_count,
                planned_ids,
                success.float(),
            )
            self._chain_index_add(
                self._chain_ready_release_count,
                planned_ids,
                ready_success.float(),
            )
            self._chain_step_ok[planned_ids] = success
            self._chain_ready_ok[planned_ids] = ready_success
            self._chain_released[planned_ids] = bool(released_to_swing)
            self._chain_exact_recorded[planned_ids] = False
            self._one_step_reentry_count.index_add_(
                0,
                planned_bucket,
                (self._step_reentry[planned_ids] > 0).float(),
            )
            self._one_step_fall_count.index_add_(
                0,
                planned_bucket,
                self._one_step_fall_before_swing[planned_ids].float(),
            )
            displacement = (
                self.base_pos_w[planned_ids, :2]
                - self._station_command_start_xy[planned_ids]
            )
            values = {
                "step_bout_count": self._step_bout_count[planned_ids].float(),
                "total_step_control_steps": self._step_control_steps[
                    planned_ids
                ].float(),
                "locomotion_mode_transition_count": self._locomotion_transition_count[
                    planned_ids
                ].float(),
                "left_foot_liftoff_count": self._foot_liftoff_count[
                    planned_ids, 0
                ].float(),
                "right_foot_liftoff_count": self._foot_liftoff_count[
                    planned_ids, 1
                ].float(),
                "left_foot_touchdown_count": self._foot_touchdown_count[
                    planned_ids, 0
                ].float(),
                "right_foot_touchdown_count": self._foot_touchdown_count[
                    planned_ids, 1
                ].float(),
                "base_y_displacement": displacement[:, 1],
                "base_x_drift": torch.abs(displacement[:, 0]),
                "maximum_station_overshoot": self._maximum_station_overshoot[
                    planned_ids
                ],
                "direction_reversal_count": self._direction_reversal_count[
                    planned_ids
                ].float(),
                "foot_slip_distance": self._foot_slip_distance[planned_ids],
                "station_error_at_step_exit": exit_error,
                "base_speed_at_step_exit": exit_speed,
                "yaw_error_at_step_exit": exit_yaw,
                "time_from_command_to_stand": exit_time,
            }
            for label, value in values.items():
                self._one_step_metric_sum[label].index_add_(
                    0, planned_bucket, value
                )
            self._one_step_event[planned_ids] = 1.0
            self._one_step_success_event[planned_ids] = success.float()
        unplanned_ids = ids[~planned]
        if unplanned_ids.numel() > 0:
            self._chain_step_ok[unplanned_ids] = False
            self._chain_ready_ok[unplanned_ids] = False
            self._chain_released[unplanned_ids] = False
            self._chain_exact_recorded[unplanned_ids] = False
        self._one_step_release_recorded[ids] = True

    def _reset_one_step_command(
        self,
        ids: torch.Tensor,
        start_xy: torch.Tensor,
        target_xy: torch.Tensor,
        delta_y: torch.Tensor,
    ) -> None:
        if not self._one_step_enabled:
            return
        self._station_command_sequence[ids] += 1
        self._station_command_id[ids] = (
            self._station_command_sequence[ids] * self.num_envs
            + self._station_command_env_index[ids]
        )
        self._station_command_start_xy[ids] = start_xy
        self._station_command_target_xy[ids] = target_xy
        self._station_distance_bin[ids] = self._one_step_distance_bucket(delta_y)
        motion = self._motion()
        if motion._multiseg:
            self._chain_clip_id[ids] = motion.clip_id[ids].clamp(0, 1)
        else:
            self._chain_clip_id[ids] = 0
        self._chain_step_ok[ids] = False
        self._chain_ready_ok[ids] = False
        self._chain_released[ids] = False
        self._chain_exact_recorded[ids] = False
        for value in (
            self._step_bout_count,
            self._step_reentry,
            self._step_control_steps,
            self._locomotion_transition_count,
            self._step_settle_dwell_count,
            self._station_command_age_steps,
            self._previous_lateral_direction,
            self._direction_reversal_count,
            self._one_step_safety_recovery_tag,
        ):
            value[ids] = 0
        for value in (
            self._step_bout_started,
            self._step_bout_complete,
            self._previous_step_mode,
            self._one_step_release_recorded,
            self._step_exit_both_feet,
            self._one_step_fall_before_swing,
        ):
            value[ids] = False
        self._foot_liftoff_count[ids] = 0
        self._foot_touchdown_count[ids] = 0
        self._maximum_station_overshoot[ids] = 0.0
        self._foot_slip_distance[ids] = 0.0
        self._step_exit_station_error[ids] = float("nan")
        self._step_exit_base_speed[ids] = float("nan")
        self._step_exit_yaw_error[ids] = float("nan")
        self._step_exit_slip_speed[ids] = float("nan")
        self._time_to_stand[ids] = float("nan")
        self._previous_foot_contact[ids] = self._feet_in_contact[ids]

    def _extend_hold_for_one_step_contract(self, motion) -> None:
        if not self._one_step_enabled:
            return
        hold_counter = getattr(motion, "hold_counter", None)
        was_holding = motion.metrics.get("in_hold") if hasattr(motion, "metrics") else None
        if hold_counter is None or was_holding is None:
            raise RuntimeError(
                "finite_step_bout_v2 requires MotionCommand hold_counter/in_hold"
            )
        countdown_done = (was_holding > 0.5) & (hold_counter == 0)
        needs_settle = (
            (self._locomotion_duration_steps > 0)
            & (~self._step_bout_complete)
        )
        extend = countdown_done & needs_settle
        motion.hold_counter = torch.where(
            extend, torch.ones_like(hold_counter), hold_counter
        )

    def _assign_foundation_context(
        self, ids: torch.Tensor, motion: MotionCommand
    ) -> None:
        if ids.numel() == 0:
            return
        if self._resample_is_wrap:
            self._foundation_strike_ordinal[ids] += 1
        else:
            self._foundation_strike_ordinal[ids] = 0
        if motion._multiseg:
            clip = motion.clip_id[ids].clamp(0, 1)
        else:
            clip = torch.zeros_like(ids)
        source = self._target_velocity_source[ids].clamp(0, 1)
        speed = self._target_speed_quartile[ids].clamp(0, 3)
        distance = self._station_distance_bin[ids].clamp(0, 4)
        order = (self._foundation_strike_ordinal[ids] > 0).long()
        context = ((((clip * 2 + source) * 4 + speed) * 5 + distance) * 2 + order)
        self._foundation_context_id[ids] = context
        self._foundation_start_count.index_add_(
            0, context, torch.ones(ids.numel(), device=self.device)
        )

    def _resample_locomotion_command(
        self, env_ids: Sequence[int], origins: torch.Tensor
    ) -> None:
        """Create one finite STAND/STEP command for each newly sampled HITTER station."""
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if ids.numel() == 0:
            return
        # A true reset before swing release means the previous command fell/timed out. A normal
        # wrap has already been booked on its release edge, so this is idempotent.
        self._finalize_one_step_commands(ids, released_to_swing=False)

        # Update HUGWBC's adaptive intervention curriculum from the command that just ended.
        count = self._locomotion_tracking_count[ids]
        valid = count > 0.0
        score = self._locomotion_tracking_sum[ids] / count.clamp_min(1.0)
        strength = self._intervention_strength[ids]
        strength = torch.where(
            valid & (score > self._intervention_tracking_pass),
            strength + self._intervention_curriculum_step,
            torch.where(
                valid & (score < self._intervention_tracking_fail),
                strength - self._intervention_curriculum_step,
                strength,
            ),
        ).clamp(0.0, 1.0)
        self._intervention_strength[ids] = strength
        self._locomotion_tracking_sum[ids] = 0.0
        self._locomotion_tracking_count[ids] = 0.0

        if self._resample_is_wrap:
            # Wraps happen mid-step: live robot data is fresh.
            start_xy = self.base_pos_w[ids, :2]
        else:
            # True resets: robot data buffers can be STALE inside the reset callback chain, so
            # read the root position the motion command actually WROTE (stand / post-swing
            # replay / RSI).  Planning from origin+nominal gave the 25% post-swing replay
            # resets a wrong-magnitude, often wrong-sign gait velocity (audit 2026-07-23).
            written = getattr(self._motion(), "last_reset_root_pos_w", None)
            if written is not None:
                start_xy = written[ids, :2]
            else:
                start_xy = torch.stack(
                    [
                        origins[:, 0] + float(self.cfg.base_nominal_offset[0]),
                        origins[:, 1] + float(self.cfg.base_nominal_offset[1]),
                    ],
                    dim=-1,
                )
        start_y = start_xy[:, 1]
        delta_y = self.base_target_pos_w[ids, 1] - start_y
        move = torch.abs(delta_y) >= self._gait_move_deadband
        if self._resample_is_wrap:
            # A SAME-STATION planner question is an explicit STAND command.  Do not turn a
            # residual tracking error from the previous cycle into another corrective gait.
            move &= self.metrics["station_y_step_class"][ids] > 0.5
        if not self._locomotion_enabled:
            move.zero_()
        cycles = torch.ceil(torch.abs(delta_y) / max(self._gait_step_distance, 1.0e-6)).long()
        cycles = cycles.clamp(min=1, max=max(self._gait_max_cycles, 1))
        cycles = torch.where(move, cycles, torch.zeros_like(cycles))
        duration_s = cycles.float() / max(self._gait_frequency_hz, 1.0e-6)
        duration_steps = torch.ceil(duration_s / float(self._env.step_dt)).long()
        duration_steps = torch.where(move, duration_steps.clamp_min(1), torch.zeros_like(duration_steps))
        velocity_y = torch.where(
            move,
            (delta_y / (duration_steps.float() * float(self._env.step_dt)).clamp_min(1.0e-6))
            .clamp(-self._gait_velocity_max, self._gait_velocity_max),
            torch.zeros_like(delta_y),
        )

        self._locomotion_planned_cycles[ids] = cycles
        self._locomotion_duration_steps[ids] = duration_steps
        self._locomotion_elapsed_steps[ids] = 0
        self._locomotion_initial_delta_y[ids] = delta_y
        self._locomotion_velocity_y[ids] = velocity_y
        self._locomotion_gait_active[ids] = False
        self._locomotion_move[ids] = False
        self._locomotion_supervision[ids] = False
        self._gait_clock[ids] = 0.0
        self._desired_contact_states[ids] = 1.0
        # Latch the plan-time station error.  STAND commands keep it for the whole hold; STEP
        # commands overwrite it once at gait completion (_update_locomotion_command).
        self._finite_station_latched_error[ids] = torch.linalg.norm(
            start_xy - self.base_target_pos_w[ids], dim=-1
        )
        self._reset_one_step_command(
            ids,
            start_xy,
            self.base_target_pos_w[ids],
            delta_y,
        )
        self._assign_foundation_context(ids, self._motion())

    def _refresh_current_foot_contact_state(self) -> None:
        """Refresh the contact/slip buffers consumed by finite-bout transitions."""
        # Resolve foot body indices + the contact sensor once (robust to USD Link/link casing).
        # Missing contact data remains fail-closed for finite_step_bout_v2: both-feet contact is
        # false, so the dwell cannot complete and the swing cannot be released.
        if not getattr(self, "_stab_resolved", False):
            self._stab_resolved = True
            self._foot_idx_robot, self._foot_idx_contact, self._contact_sensor = (
                [],
                [],
                None,
            )
            try:
                self._foot_idx_robot = list(
                    self.robot.find_bodies([".*ankle_roll.*"])[0]
                )
            except Exception:
                pass
            try:
                contact_sensor = self._env.scene.sensors["contact_forces"]
                self._contact_sensor = contact_sensor
                self._foot_idx_contact = list(
                    contact_sensor.find_bodies([".*ankle_roll.*"])[0]
                )
            except Exception:
                pass

        valid_contact_source = (
            len(self._foot_idx_robot) == self._feet_in_contact.shape[1]
            and self._contact_sensor is not None
            and len(self._foot_idx_contact) == self._feet_in_contact.shape[1]
        )
        if valid_contact_source:
            foot_force = torch.norm(
                self._contact_sensor.data.net_forces_w[
                    :, self._foot_idx_contact, :
                ],
                dim=-1,
            )
            in_contact_bool = (
                foot_force > self._step_settle_contact_force_threshold
            )
            in_contact = in_contact_bool.float()
            foot_velocity = self.robot.data.body_lin_vel_w[
                :, self._foot_idx_robot, :
            ]
            foot_speed = torch.norm(foot_velocity[..., :2], dim=-1)
            contacting_slip = foot_speed * in_contact
            slip_sum = contacting_slip.sum(dim=-1)
            contact_fraction = in_contact.mean(dim=-1)
            mean_contacting_slip = slip_sum / in_contact.sum(dim=-1).clamp(
                min=1.0
            )

            touchdown = (~self._feet_in_contact) & in_contact_bool
            self._foot_touchdown_downspeed.copy_(
                touchdown.float()
                * torch.clamp(-foot_velocity[..., 2], min=0.0)
            )
            self._feet_in_contact.copy_(in_contact_bool)
            self._foot_slip_speed_per_foot.copy_(contacting_slip)
            self.metrics["foot_contact_frac"] = contact_fraction
            self.metrics["foot_slip_speed"] = mean_contacting_slip
            self.metrics["foot_touchdown_downspeed"] = (
                self._foot_touchdown_downspeed.max(dim=-1).values
            )
            self.feet_contact_frac = contact_fraction
            self.foot_slip_in_contact = slip_sum
            return

        self._feet_in_contact.zero_()
        self._foot_slip_speed_per_foot.zero_()
        self._foot_touchdown_downspeed.zero_()
        zeros = torch.zeros(self.num_envs, device=self.device)
        self.metrics["foot_contact_frac"] = zeros
        self.metrics["foot_slip_speed"] = zeros
        self.metrics["foot_touchdown_downspeed"] = zeros
        self.feet_contact_frac = zeros
        self.foot_slip_in_contact = zeros

    def restore_exact_resume_runtime_references(self) -> None:
        """Rebind scene-owned objects omitted from serialized command state.

        The resolution flag and body-index lists are logical state.  The
        contact-sensor Python object is owned by the new scene and cannot be
        serialized, so schema-3 restores that reference explicitly before the
        first continuation step.
        """
        if getattr(self, "_stab_resolved", False):
            try:
                contact_sensor = self._env.scene.sensors["contact_forces"]
            except (KeyError, AttributeError):
                contact_sensor = None
            if getattr(self, "_foot_idx_contact", ()) and contact_sensor is None:
                raise RuntimeError(
                    "Exact resume cannot rebind the saved contact-force sensor"
                )
            self._contact_sensor = contact_sensor

        # Adaptive sigma and staged velocity curriculum mutate live reward
        # term configs.  Those config objects are intentionally not duplicated
        # in the generic tensor checkpoint, so reconstruct their values from
        # the restored authoritative command-state scalars.
        reward_manager = self._env.reward_manager
        try:
            reward_manager.get_term_cfg("racket_position").params["std"] = (
                float(self._adaptive_sigma_pos)
            )
            velocity_term = reward_manager.get_term_cfg("racket_velocity")
            velocity_term.params["std"] = float(self._adaptive_sigma_vel)
            velocity_term.weight = float(self._velocity_current_weight)
            success_params = reward_manager.get_term_cfg(
                "racket_strike_success"
            ).params
            success_params["std_pos"] = float(self._adaptive_sigma_pos)
            success_params["std_vel"] = float(self._adaptive_sigma_vel)
            reward_manager.get_term_cfg("racket_normal").params["std"] = (
                float(self._adaptive_sigma_normal)
            )
        except (AttributeError, ValueError):
            pass
        try:
            self._motion().set_recovery_curriculum_scale(
                float(self._recovery_coverage_scale)
            )
        except (AttributeError, ValueError):
            pass

    def _update_locomotion_command(self, motion: MotionCommand) -> None:
        """Advance one finite gait bout, settle continuously, then latch STAND."""
        if not self._locomotion_enabled:
            self._locomotion_gait_active.zero_()
            self._locomotion_move.zero_()
            self._locomotion_supervision.zero_()
            self._locomotion_velocity_y.zero_()
            self._gait_clock.zero_()
            self._desired_contact_states.fill_(1.0)
            return

        self._one_step_event.zero_()
        self._one_step_success_event.zero_()
        # Do not let the STEP -> STAND edge depend on a previously cached metric.  CommandTerm
        # currently calls _update_metrics first, but the safety contract remains correct even if
        # that framework ordering changes or this method is invoked directly.
        self._refresh_current_foot_contact_state()
        in_hold = getattr(motion, "in_hold", torch.zeros_like(self._locomotion_move))
        planned = self._locomotion_duration_steps > 0
        unfinished = self._locomotion_elapsed_steps < self._locomotion_duration_steps
        requested_gait = in_hold & planned & unfinished
        if self._one_step_enabled:
            base_error = torch.linalg.norm(
                self.base_pos_w[:, :2] - self._station_command_target_xy, dim=-1
            )
            base_speed = torch.linalg.norm(
                self.robot.data.root_lin_vel_w[:, :2], dim=-1
            )
            quat = self.base_quat_w
            yaw_error = torch.abs(
                torch.atan2(
                    2.0
                    * (
                        quat[:, 1] * quat[:, 2]
                        + quat[:, 0] * quat[:, 3]
                    ),
                    1.0
                    - 2.0 * (quat[:, 2] ** 2 + quat[:, 3] ** 2),
                )
            )
            both_feet = self._feet_in_contact.all(dim=-1)
            foot_slip = self.metrics["foot_slip_speed"]
            settled = (
                (base_error <= self._step_settle_pos_thresh)
                & (base_speed <= self._step_settle_speed_thresh)
                & (yaw_error <= self._step_settle_yaw_thresh)
                & both_feet
                & (foot_slip <= self._step_settle_slip_thresh)
            )
            bout = advance_one_step_bout(
                torch,
                in_hold=in_hold,
                planned=planned,
                elapsed_steps=self._locomotion_elapsed_steps,
                duration_steps=self._locomotion_duration_steps,
                started=self._step_bout_started,
                complete=self._step_bout_complete,
                dwell_count=self._step_settle_dwell_count,
                settled=settled,
                required_dwell_steps=self._step_settle_dwell_steps,
            )
            self._step_reentry.add_(bout.blocked_reentry.long())
            self._step_bout_started.copy_(bout.started)
            self._step_bout_count.add_(bout.new_bout.long())
            gait_active = bout.gait_active
            step_mode = bout.step_mode_before_completion
        else:
            gait_active = requested_gait
            step_mode = gait_active
        self._locomotion_supervision.copy_(in_hold)
        self._locomotion_gait_active.copy_(gait_active)

        phase = torch.remainder(
            self._locomotion_elapsed_steps.float()
            * float(self._env.step_dt)
            * self._gait_frequency_hz,
            1.0,
        )
        # Lead with the foot on the side of travel: left leads for +y steps (the legacy fixed
        # assignment), right leads for -y steps.  A one-cycle rightward step otherwise always has
        # to cross-step with the left foot first (audit 2026-07-23), and the mirror loss cannot
        # supply right-leading data because it never occurred in rollouts.  The C++ scheduler and
        # the MuJoCo harness implement the identical rule.
        lead_left = self._locomotion_velocity_y >= 0.0
        half_offset = torch.where(
            lead_left, torch.full_like(phase, 0.5), torch.zeros_like(phase)
        )
        left_phase = torch.remainder(phase + half_offset, 1.0)
        right_phase = torch.remainder(phase + 0.5 - half_offset, 1.0)

        # HUGWBC remaps the commanded duty factor onto equal [0, 0.5) stance and
        # [0.5, 1) swing clock halves before publishing the sine clocks/contact targets.
        # This keeps the observation convention fixed when the YAML changes duty factor.
        duty = min(max(self._gait_duty_factor, 1.0e-4), 1.0 - 1.0e-4)

        def remap_foot_phase(raw: torch.Tensor) -> torch.Tensor:
            return torch.where(
                raw < duty,
                raw * (0.5 / duty),
                0.5 + (raw - duty) * (0.5 / (1.0 - duty)),
            )

        left_clock_phase = remap_foot_phase(left_phase)
        right_clock_phase = remap_foot_phase(right_phase)
        self._gait_clock[:, 0] = torch.where(
            gait_active,
            torch.sin(2.0 * math.pi * left_clock_phase),
            torch.zeros_like(phase),
        )
        self._gait_clock[:, 1] = torch.where(
            gait_active,
            torch.sin(2.0 * math.pi * right_clock_phase),
            torch.zeros_like(phase),
        )

        # Released HUGWBC contact-probability construction (Gaussian-smoothed stance half-cycle).
        kappa = max(self._gait_contact_smoothing, 1.0e-6)
        def normal_cdf(x: torch.Tensor) -> torch.Tensor:
            return 0.5 * (1.0 + torch.erf(x / (kappa * math.sqrt(2.0))))

        for foot, foot_phase in enumerate((left_clock_phase, right_clock_phase)):
            contact = (
                normal_cdf(foot_phase) * (1.0 - normal_cdf(foot_phase - 0.5))
                + normal_cdf(foot_phase - 1.0)
                * (1.0 - normal_cdf(foot_phase - 1.5))
            )
            self._desired_contact_states[:, foot] = torch.where(
                gait_active, contact, torch.ones_like(contact)
            )

        desired_vy = torch.where(
            gait_active, self._locomotion_velocity_y, torch.zeros_like(phase)
        )
        tracking = torch.exp(
            -torch.square(self.robot.data.root_lin_vel_w[:, 1] - desired_vy)
            / max(self._intervention_tracking_sigma, 1.0e-6)
        )
        self._locomotion_tracking_sum.add_(tracking * gait_active.float())
        self._locomotion_tracking_count.add_(gait_active.float())
        self.metrics["locomotion_velocity_y_cmd"] = desired_vy
        self.metrics["locomotion_gait_phase"] = torch.where(
            gait_active, phase, torch.zeros_like(phase)
        )
        self.metrics["locomotion_cycles_planned"] = self._locomotion_planned_cycles.float()
        self.metrics["locomotion_initial_delta_y"] = self._locomotion_initial_delta_y
        self.metrics["locomotion_tracking_exp"] = tracking
        self.metrics["upper_intervention_strength"] = self._intervention_strength
        if self._one_step_enabled:
            self._locomotion_elapsed_steps.copy_(bout.elapsed_steps)
        else:
            self._locomotion_elapsed_steps.add_(gait_active.long())

        if self._one_step_enabled:
            active_command = self._station_command_id >= 0
            fall_like = (
                active_command
                & (~self._one_step_release_recorded)
                & (
                    self.proj_grav_xy
                    >= math.sin(self._one_step_fall_tilt_limit_rad)
                )
            ) | (
                active_command
                & (~self._one_step_release_recorded)
                & (
                    self.robot.data.root_pos_w[:, 2]
                    < self._one_step_fall_height_min_m
                )
            )
            self._one_step_fall_before_swing |= fall_like
            self._station_command_age_steps.add_(active_command.long())
            self._step_settle_dwell_count.copy_(bout.dwell_count)
            self._step_bout_complete.copy_(bout.complete)
            just_completed = bout.just_completed
            final_step_mode = bout.step_mode

            # Per-command locomotion telemetry (normal left/right contacts inside one bout are
            # counted, not treated as multiple STEP entries).
            liftoff = self._previous_foot_contact & (~self._feet_in_contact)
            touchdown = (~self._previous_foot_contact) & self._feet_in_contact
            self._foot_liftoff_count.add_(
                (liftoff & step_mode.unsqueeze(-1)).long()
            )
            self._foot_touchdown_count.add_(
                (touchdown & step_mode.unsqueeze(-1)).long()
            )
            self._previous_foot_contact.copy_(self._feet_in_contact)
            self._step_control_steps.add_(step_mode.long())
            transition = final_step_mode != self._previous_step_mode
            self._locomotion_transition_count.add_(
                (transition & active_command).long()
            )
            self._previous_step_mode.copy_(final_step_mode)
            displacement = self.base_pos_w[:, :2] - self._station_command_start_xy
            direction = torch.sign(
                self.robot.data.root_lin_vel_w[:, 1]
            ).long()
            direction = torch.where(
                torch.abs(self.robot.data.root_lin_vel_w[:, 1]) >= 0.03,
                direction,
                torch.zeros_like(direction),
            )
            reversal = (
                step_mode
                & (direction != 0)
                & (self._previous_lateral_direction != 0)
                & (direction != self._previous_lateral_direction)
            )
            self._direction_reversal_count.add_(reversal.long())
            self._previous_lateral_direction = torch.where(
                step_mode & (direction != 0),
                direction,
                self._previous_lateral_direction,
            )
            command_sign = torch.sign(self._locomotion_initial_delta_y)
            progress = command_sign * displacement[:, 1]
            overshoot = torch.clamp(
                progress - torch.abs(self._locomotion_initial_delta_y), min=0.0
            )
            self._maximum_station_overshoot.copy_(
                torch.maximum(self._maximum_station_overshoot, overshoot)
            )
            self._foot_slip_distance.add_(
                foot_slip * step_mode.float() * float(self._env.step_dt)
            )
            # Tags are diagnostic only and never re-arm STEP.  1=lost balance, 3=actor base
            # localization stale.  A changed station receives a new command_id instead.
            projected_gravity = getattr(
                self.robot.data, "projected_gravity_b", None
            )
            if projected_gravity is not None:
                tilt = torch.linalg.norm(projected_gravity[:, :2], dim=-1)
                self._one_step_safety_recovery_tag = torch.where(
                    step_mode & (tilt >= 0.45),
                    torch.ones_like(self._one_step_safety_recovery_tag),
                    self._one_step_safety_recovery_tag,
                )
            self._one_step_safety_recovery_tag = torch.where(
                step_mode
                & (self._actor_base_age_s >= self._base_mocap_max_age_s),
                torch.full_like(self._one_step_safety_recovery_tag, 3),
                self._one_step_safety_recovery_tag,
            )
            if bool(just_completed.any()):
                self._step_exit_station_error = torch.where(
                    just_completed, base_error, self._step_exit_station_error
                )
                self._step_exit_base_speed = torch.where(
                    just_completed, base_speed, self._step_exit_base_speed
                )
                self._step_exit_yaw_error = torch.where(
                    just_completed, yaw_error, self._step_exit_yaw_error
                )
                self._step_exit_slip_speed = torch.where(
                    just_completed, foot_slip, self._step_exit_slip_speed
                )
                self._step_exit_both_feet = torch.where(
                    just_completed, both_feet, self._step_exit_both_feet
                )
                self._time_to_stand = torch.where(
                    just_completed,
                    self._station_command_age_steps.float()
                    * float(self._env.step_dt),
                    self._time_to_stand,
                )
                self._finite_station_latched_error = torch.where(
                    just_completed, base_error, self._finite_station_latched_error
                )
            self._locomotion_move.copy_(final_step_mode)
        else:
            self._locomotion_move.copy_(step_mode)
            just_finished = gait_active & (
                self._locomotion_elapsed_steps >= self._locomotion_duration_steps
            )
            if bool(just_finished.any()):
                live_error = torch.linalg.norm(
                    self.base_pos_w[:, :2] - self.base_target_pos_w, dim=-1
                )
                self._finite_station_latched_error = torch.where(
                    just_finished, live_error, self._finite_station_latched_error
                )

        self.metrics["locomotion_move_mode"] = self._locomotion_move.float()
        self.metrics["locomotion_gait_active"] = (
            self._locomotion_gait_active.float()
        )
        self.metrics["station_command_id"] = self._station_command_id.float()
        self.metrics["station_delta_y"] = self._locomotion_initial_delta_y
        self.metrics["step_bout_count"] = self._step_bout_count.float()
        self.metrics["total_STEP_control_steps"] = self._step_control_steps.float()
        self.metrics["locomotion_mode_transition_count"] = (
            self._locomotion_transition_count.float()
        )
        self.metrics["left_foot_liftoff_count"] = self._foot_liftoff_count[:, 0].float()
        self.metrics["right_foot_liftoff_count"] = self._foot_liftoff_count[:, 1].float()
        self.metrics["left_foot_touchdown_count"] = self._foot_touchdown_count[:, 0].float()
        self.metrics["right_foot_touchdown_count"] = self._foot_touchdown_count[:, 1].float()
        displacement = self.base_pos_w[:, :2] - self._station_command_start_xy
        self.metrics["base_y_displacement"] = displacement[:, 1]
        self.metrics["base_x_drift_one_step"] = torch.abs(displacement[:, 0])
        self.metrics["maximum_station_overshoot"] = self._maximum_station_overshoot
        self.metrics["direction_reversal_count"] = self._direction_reversal_count.float()
        self.metrics["foot_slip_distance"] = self._foot_slip_distance
        self.metrics["station_error_at_STEP_exit"] = torch.nan_to_num(
            self._step_exit_station_error, nan=0.0
        )
        self.metrics["base_speed_at_STEP_exit"] = torch.nan_to_num(
            self._step_exit_base_speed, nan=0.0
        )
        self.metrics["yaw_error_at_STEP_exit"] = torch.nan_to_num(
            self._step_exit_yaw_error, nan=0.0
        )
        self.metrics["time_from_command_to_STAND"] = torch.nan_to_num(
            self._time_to_stand, nan=0.0
        )
        self.metrics["STEP_reentry_before_next_command"] = self._step_reentry.float()
        self.metrics["one_step_event"] = self._one_step_event
        self.metrics["one_step_success"] = self._one_step_success_event
        self.metrics["one_step_safety_recovery_tag"] = (
            self._one_step_safety_recovery_tag.float()
        )
        self.metrics["fall_before_next_swing"] = (
            self._one_step_fall_before_swing.float()
        )

    def _racket_fk(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return pure, side-effect-free racket FK for the physical-ball callback.

        The returned tuple is ``(position, quaternion, link-point velocity, raw normal,
        signed striking-face normal)``.  Nothing is assigned to the actor/reward-visible
        ``self.racket_*`` buffers, so enabling physical telemetry cannot advance their state.
        """

        data = self.robot.data
        if self._racket_mode == "body":
            idx = self._racket_body_index
            pos = data.body_pos_w[:, idx]
            quat = data.body_quat_w[:, idx]
            if not hasattr(data, "body_link_lin_vel_w"):
                raise RuntimeError(
                    "physical-ball racket FK requires Isaac Lab body_link_lin_vel_w"
                )
            lin_vel = data.body_link_lin_vel_w[:, idx]
        else:
            widx = self._wrist_body_index
            wpos = data.body_pos_w[:, widx]
            wquat = data.body_quat_w[:, widx]
            if not hasattr(data, "body_link_lin_vel_w") or not hasattr(
                data, "body_link_ang_vel_w"
            ):
                raise RuntimeError(
                    "physical-ball wrist FK requires Isaac Lab body_link_{lin,ang}_vel_w"
                )
            wlin = data.body_link_lin_vel_w[:, widx]
            wang = data.body_link_ang_vel_w[:, widx]
            offset_w = quat_apply(wquat, self._mount_offset)
            pos = wpos + offset_w
            lin_vel = wlin + torch.cross(wang, offset_w, dim=-1)
            quat = quat_mul(wquat, self._mount_quat)

        raw_normal = matrix_from_quat(quat)[:, :, self.cfg.mount_normal_axis]
        if self._mount_sign_per_clip_t is not None and self._motion()._multiseg:
            clip = self._motion().clip_id.clamp(
                max=self._mount_sign_per_clip_t.shape[0] - 1
            )
            sign = self._mount_sign_per_clip_t[clip].unsqueeze(-1)
        else:
            sign = self.cfg.mount_normal_sign
        return pos, quat, lin_vel, raw_normal, raw_normal * sign

    def _compute_racket_state(self):
        data = self.robot.data
        if self._racket_mode == "body":
            idx = self._racket_body_index
            self.racket_pos_w = data.body_pos_w[:, idx]
            self.racket_quat_w = data.body_quat_w[:, idx]
            self.racket_lin_vel_w = data.body_lin_vel_w[:, idx]
        else:
            widx = self._wrist_body_index
            wpos = data.body_pos_w[:, widx]
            wquat = data.body_quat_w[:, widx]
            wlin = data.body_lin_vel_w[:, widx]
            wang = data.body_ang_vel_w[:, widx]
            offset_w = quat_apply(wquat, self._mount_offset)
            self.racket_pos_w = wpos + offset_w
            self.racket_lin_vel_w = wlin + torch.cross(wang, offset_w, dim=-1)
            self.racket_quat_w = quat_mul(wquat, self._mount_quat)
        # Face normal = chosen local axis of the racket frame, mapped to world, times the striking-FACE sign.
        # In a unified forehand+backhand policy the two swings strike with OPPOSITE paddle faces, so the sign
        # is per-clip (indexed by clip_id) when mount_normal_sign_per_clip is set; otherwise the scalar sign
        # applies to every env (backward compatible). This is what the racket_normal reward + the
        # racket_normal_error_deg metric score, so per-clip here fixes both at once.
        # TODO(asset): confirm mount_normal_axis/sign against pingpang_red_Link.STL (see hope-a3-racket-mount).
        axis_w = matrix_from_quat(self.racket_quat_w)[:, :, self.cfg.mount_normal_axis]
        if self._mount_sign_per_clip_t is not None and self._motion()._multiseg:
            clip = self._motion().clip_id.clamp(max=self._mount_sign_per_clip_t.shape[0] - 1)
            sign = self._mount_sign_per_clip_t[clip].unsqueeze(-1)  # (num_envs, 1)
        else:
            sign = self.cfg.mount_normal_sign
        self.racket_normal_w = axis_w * sign

    def _resolve_body_index(self, name: str) -> int:
        """Resolve one articulation body without letting ``find_bodies`` throw on a missing name."""
        cache = getattr(self, "_body_index_cache", None)
        if cache is None:
            cache = {}
            self._body_index_cache = cache
        if name not in cache:
            if name not in self.robot.body_names:
                cache[name] = -1
            else:
                try:
                    ids = self.robot.find_bodies(name, preserve_order=True)[0]
                    cache[name] = int(ids[0]) if len(ids) else -1
                except Exception:
                    cache[name] = -1
        return int(cache[name])

    def left_arm_clearance(
        self,
        hand_body_name: str = "left_wrist_yaw_Link",
        elbow_body_name: str = "left_elbow_Link",
        forearm_end_body_name: str = "left_wrist_roll_Link",
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Racket-center distance to the left-hand proxy and left-forearm line segment.

        The fixed ``left_hand_Link`` is merged into ``left_wrist_yaw_Link`` by the articulation
        importer, so the wrist-yaw body is the deploy-consistent hand proxy.  The forearm uses the
        closest point on the elbow -> wrist-roll segment instead of either endpoint alone.  Missing
        bodies return ``+inf`` (zero clearance penalty) and never crash a training job.
        """
        inf = torch.full((self.num_envs,), float("inf"), device=self.device)
        hand_idx = self._resolve_body_index(hand_body_name)
        elbow_idx = self._resolve_body_index(elbow_body_name)
        forearm_end_idx = self._resolve_body_index(forearm_end_body_name)
        hand_dist = inf
        if hand_idx >= 0:
            hand_dist = torch.linalg.norm(
                self.racket_pos_w - self.robot.data.body_pos_w[:, hand_idx], dim=-1
            )
        forearm_dist = inf
        if elbow_idx >= 0 and forearm_end_idx >= 0:
            a = self.robot.data.body_pos_w[:, elbow_idx]
            b = self.robot.data.body_pos_w[:, forearm_end_idx]
            ab = b - a
            ap = self.racket_pos_w - a
            t = (torch.sum(ap * ab, dim=-1) / torch.sum(ab * ab, dim=-1).clamp_min(1e-8)).clamp(0.0, 1.0)
            closest = a + t.unsqueeze(-1) * ab
            forearm_dist = torch.linalg.norm(self.racket_pos_w - closest, dim=-1)
        return hand_dist, forearm_dist

    def _compute_strike_timing(self):
        """Refresh time_to_strike / pre_strike / strike_window from the CURRENT motion phase.

        The ``motion`` command term computes before this one (it is a parent-class field, registered
        first), so by now ``motion.time_steps`` is already advanced for this control step. Computing the
        timing here — at the top of _update_metrics, alongside the fresh racket FK — keeps the strike
        masks ALIGNED with the racket pose they gate. Previously the masks were only set in
        _update_command (which runs AFTER _update_metrics), so _update_metrics read a 1-step-stale
        time_to_strike: ``exact_strike`` fired one control frame LATE, after the paddle had flown ~one
        step past the strike (~12 cm at a 6 m/s swing) — which collapsed the measured position accuracy
        (exact-strike pos<7.5cm read ~11% instead of the true ~68%) while barely moving velocity (flat
        near the peak). That made strike_composite_success_exact ~6x pessimistic vs the honest probe.
        """
        motion = self._motion()
        ml = motion.motion
        if motion._multiseg:
            # Per-clip strike frame on the concatenated time axis: the contact phase differs per swing
            # (v2 blade re-plane: forehand 0.47, backhand 0.333), so resolve strike_step per env from its clip.
            if self._strike_phase_per_clip_t is None:
                sp = tuple(self.cfg.strike_phase_per_clip)
                if sp and len(sp) == ml.num_segments:
                    self._strike_phase_per_clip_t = torch.tensor([float(x) for x in sp], device=self.device)
                else:
                    self._strike_phase_per_clip_t = torch.full(
                        (ml.num_segments,), float(self.cfg.strike_phase), device=self.device
                    )
            clip = motion.clip_id
            seg_start = ml.seg_start[clip]
            seg_len = ml.seg_len[clip]
            phase = self._strike_phase_per_clip_t[clip]
            strike_step = seg_start + (phase * (seg_len - 1).float()).round().long()
            self.time_to_strike = (strike_step - motion.time_steps).float() * self._env.step_dt
        else:
            total = max(int(ml.time_step_total), 1)
            strike_step = round(self.cfg.strike_phase * (total - 1))
            self.time_to_strike = (strike_step - motion.time_steps).float() * self._env.step_dt
        self.pre_strike = self.time_to_strike > 0.0
        self.strike_window = self.time_to_strike.abs() <= self.cfg.strike_window_s

    def _update_command(self):
        motion = self._motion()
        # Timing is refreshed in _update_metrics (aligned with the FK); recompute here too so a direct
        # _update_command call outside the compute() path stays correct. Idempotent within a step
        # (motion.time_steps is unchanged between the two calls).
        self._compute_strike_timing()

        # A true V17 replay reset starts from a previous swing's live follow-through but is paired
        # with the next clip's held command. Attribute any physical failure during that recovery
        # hold to the replay source clip as a POST-strike fall. Without this latch, the reversible
        # curriculum would misclassify replay-recovery falls as pre-strike failures and its
        # dedicated post_fall regression gate would be systematically optimistic.
        replay_active = getattr(motion, "post_swing_replay_active", None)
        replay_bucket = getattr(motion, "_v17_replay_last_sample_bucket", None)
        if (
            motion._multiseg
            and torch.is_tensor(replay_active)
            and torch.is_tensor(replay_bucket)
        ):
            valid_replay = replay_active & (replay_bucket >= 0)
            if bool(valid_replay.any()):
                phase_bins = int(motion.cfg.post_swing_capture_phase_bins)
                severity_bins = (
                    int(motion.cfg.post_swing_capture_severity_bins)
                    if getattr(
                        motion, "_post_swing_replay_contract", ""
                    )
                    == "markov_side_phase_severity_v3"
                    else 1
                )
                self._recover_from_clip[valid_replay] = torch.div(
                    replay_bucket[valid_replay],
                    phase_bins * severity_bins,
                    rounding_mode="floor",
                )

        # Re-sample the target at each new swing. Use the motion command's robust just_resampled signal
        # (set this same step when it wrapped a swing) instead of a time_steps<prev heuristic — the latter
        # fails for the unified policy when a wrap jumps the index to a HIGHER concatenated segment start
        # (forehand->backhand). Targets for fresh episodes are sampled by the manager's reset.
        wrapped = torch.where(motion.just_resampled)[0] if hasattr(motion, "just_resampled") else \
            torch.where(motion.time_steps < self._prev_motion_steps)[0]
        if len(wrapped) > 0:
            # Wrap path: a wrapped env passed its strike frame alive (strike < seg end), so no
            # pre-strike fall is counted — the flag only gates fall accounting inside
            # _resample_command's _count_swing_starts hook.
            if motion._multiseg:
                # Latch the clip that JUST finished (before _prev_clip_id is re-snapshotted below):
                # while the post-wrap hold lasts, a fall belongs to THIS swing's recovery.
                self._recover_from_clip[wrapped] = self._prev_clip_id[wrapped]
                self._recover_strike_audit_context_id[wrapped] = (
                    self._strike_audit_context_id[wrapped]
                )
                self._foundation_recovery_context_id[wrapped] = (
                    self._foundation_context_id[wrapped]
                )
            self._resample_is_wrap = True
            try:
                self._resample_command(wrapped)
            finally:
                self._resample_is_wrap = False
        # The recovery window ends when the post-wrap hold expires (the new swing's clock starts
        # advancing) — the latch clear itself runs BELOW, after _extend_hold_until_settled (2026-07-09
        # audit fix): in_hold is the live post-decrement property, so clearing HERE would drop the latch
        # on the countdown-expiry step BEFORE the arrival gate re-floors the counter — falls during the
        # extension (the unsettled, highest-risk stretch of the recovery) would then book as pre-strike
        # falls of the NEW clip, polluting the G1 pre_strike_fall_rate criterion.
        self._prev_motion_steps = motion.time_steps.clone()
        # Snapshot the clip each env is swinging THIS step: at the next true reset the motion command
        # will already have resampled clip_id, so fall attribution reads this snapshot instead.
        if motion._multiseg:
            self._prev_clip_id = motion.clip_id.clone()

        # --- A1 mid-swing target refinement (the planner refines WHERE, not WHEN) -------------------
        # Each step, envs still approaching the strike (pre_strike AND time_to_strike > tts floor)
        # re-draw their target with per-step prob p, exactly as the deploy planner refines its ball
        # prediction mid-swing. ONLY the target sampling runs (position/velocity/normal through the
        # existing uniform / per-clip-box / reference-perturbed path, including the HER achieved-target
        # mixture inside it):
        #   * strike timing untouched — same strike step, the swing clock keeps running;
        #   * NO _count_swing_starts — a refinement is not a new swing attempt (metrics denominators
        #     would otherwise be inflated);
        #   * base target / swing type / _prev_motion_steps untouched;
        #   * the racket-progress baseline is reset via _progress_reset_mask (same mechanism as the
        #     resample path) so the target jump creates no fake progress reward;
        #   * the achieved-target replay WRITE is unaffected (it stores the LIVE target state at
        #     exact-strike frames, and tts floor > 0 keeps refinement away from the strike frame).
        # prob==0 (default) short-circuits before any RNG draw — byte-identical baseline.
        _ms_prob = float(self.cfg.midswing_resample_prob)
        if _ms_prob > 0.0:
            eligible = self.pre_strike & (self.time_to_strike > float(self.cfg.midswing_resample_tts_floor))
            redraw = eligible & (torch.rand(self.num_envs, device=self.device) < _ms_prob)
            ids = torch.where(redraw)[0]
            if len(ids) > 0:
                origins = self._env.scene.env_origins[ids]
                if self.cfg.target_mode == "reference_perturbed":
                    self._sample_targets_reference_perturbed(ids, origins, len(ids))
                elif self.cfg.target_mode == "hitter_pure":
                    # Refinement re-draws WHERE around the UNCHANGED station (paper Fig. 3
                    # convergence; the commanded stance never teleports mid-swing).
                    self._sample_targets_hitter_pure(ids, origins, len(ids), resample_base=False)
                else:
                    self._sample_targets_uniform(ids, origins, len(ids))
                self._prev_racket_dist[ids] = torch.norm(
                    self.racket_pos_w[ids] - self.racket_target_pos_w[ids], dim=-1
                ).detach()
                self.racket_progress[ids] = 0.0
                self._progress_reset_mask[ids] = True
            # Per-env 0/1 indicator; the wandb reset-mean = per-step refinement fraction (~ prob *
            # eligible fraction). Written every step while the feature is on so zero-redraw steps count.
            self.metrics["midswing_resample_count"] = redraw.float()

        # Arrival-gated hold release (cfg.hold_until_settled): extend the hold while the base has not
        # settled at the station, so the swing arms only AFTER arrival. Runs AFTER the MotionCommand
        # decremented hold_counter this step (motion updates before racket_target) and BEFORE the actor
        # target refresh; a no-op when the feature is off.
        if self.cfg.hold_until_settled:
            self._extend_hold_until_settled(motion)
        # V15's explicit one-bout state machine owns a separate, non-negotiable release gate:
        # the swing clock cannot leave HOLD while a started STEP is still active or settling.
        self._extend_hold_for_one_step_contract(motion)
        # V17 samples this gate with probability equal to the reversible readiness scale. It
        # runs after all minimum-hold logic and re-floors only the final tick; the actor cannot
        # stretch or shorten the 1.5 s safety timeout.
        self._apply_ready_release_gate(motion)

        # The recovery window ends when the post-wrap hold expires (the new swing's clock starts
        # advancing) — from then on falls are genuinely pre-strike of the new clip. Runs AFTER the
        # arrival-gate extension: in_hold is the live post-decrement (and now post-extension) hold
        # state, so extension steps keep the latch (their falls stay attributed to the previous
        # swing's recovery) while a genuine release — or a zero-length hold — clears it this step.
        if motion._multiseg and hasattr(motion, "in_hold"):
            self._recover_from_clip[~motion.in_hold] = -1

        # Advance the lower-body command after the arrival/hold state is final for this
        # control step.  A station transition yields one finite gait episode; once its
        # planned cycles expire this remains latched in STAND until the next resample.
        self._update_locomotion_command(motion)

        # A1 target latency/jitter: refresh the ACTOR-visible target view once per step (no-op alias
        # when the knobs are off). Runs LAST so it sees this step's wrap/refinement target updates.
        self._push_actor_target()
        self._push_actor_base_mocap()

    def _push_actor_base_mocap(self) -> None:
        """Refresh the delayed full-pose actor localization view once per policy step."""
        if not self._base_mocap_enabled:
            self._actor_base_pos_w.copy_(self.base_pos_w)
            self._actor_base_quat_w.copy_(self.base_quat_w)
            self._actor_base_velocity_xy.copy_(self.robot.data.root_lin_vel_w[:, :2])
            self._actor_base_age_s.zero_()
            self.metrics["base_mocap_orientation_error_rad"].zero_()
            return

        dt = float(self._env.step_dt)
        current = self.base_pos_w
        current_quat = self.base_quat_w
        robustness_scale = float(self._base_mocap_robustness_scale)
        effective_delay = int(
            round(self._base_mocap_delay_steps * robustness_scale)
        )
        self._base_mocap_delay_by_env.fill_(effective_delay)
        self.metrics["base_mocap_delay_steps_in_effect"][:] = float(
            effective_delay
        )
        self.metrics["base_mocap_robustness_scale"][:] = robustness_scale

        # Scale zero is an exact clean full-pose path: no delay, no corruption, no dropout,
        # and importantly no hidden RNG consumption before strike competence is admitted.
        if robustness_scale > 0.0 and bool(
            (self._base_mocap_noise_std > 0.0).any()
        ):
            captured = current + (
                torch.randn_like(current)
                * self._base_mocap_noise_std
                * robustness_scale
            )
        else:
            captured = current
        if self._base_mocap_orientation_enabled:
            needs_residual = self._base_mocap_residual_needs_sample
            if robustness_scale > 0.0 and bool(needs_residual.any()):
                count = int(needs_residual.sum().item())
                self._base_mocap_extrinsic_residual_rpy[needs_residual] = (
                    torch.randn(count, 3, device=self.device)
                    * self._base_mocap_extrinsic_residual_std
                )
                self._base_mocap_residual_needs_sample[needs_residual] = False
            residual_rpy = (
                self._base_mocap_extrinsic_residual_rpy * robustness_scale
            )
            residual_quat = quat_from_euler_xyz(
                residual_rpy[:, 0], residual_rpy[:, 1], residual_rpy[:, 2]
            )
            self._base_mocap_extrinsic_residual_quat.copy_(residual_quat)
            if robustness_scale > 0.0 and bool(
                (self._base_mocap_orientation_noise_std > 0.0).any()
            ):
                noise_rpy = (
                    torch.randn(self.num_envs, 3, device=self.device)
                    * self._base_mocap_orientation_noise_std
                    * robustness_scale
                )
            else:
                noise_rpy = torch.zeros(
                    self.num_envs, 3, device=self.device
                )
            noise_quat = quat_from_euler_xyz(
                noise_rpy[:, 0], noise_rpy[:, 1], noise_rpy[:, 2]
            )
            captured_quat = quat_mul(
                quat_mul(
                    current_quat,
                    self._base_mocap_extrinsic_residual_quat,
                ),
                noise_quat,
            )
            captured_quat = captured_quat / torch.linalg.norm(
                captured_quat, dim=-1, keepdim=True
            ).clamp_min(1.0e-9)
        else:
            captured_quat = current_quat

        w = self._base_mocap_delay_ptr
        self._base_mocap_delay_buf[w].copy_(captured)
        self._base_mocap_quat_delay_buf[w].copy_(captured_quat)
        buffer_length = self._base_mocap_delay_steps + 1
        self._base_mocap_delay_ptr = (w + 1) % buffer_length
        read_rows = torch.remainder(
            w - self._base_mocap_delay_by_env, buffer_length
        )
        env_rows = torch.arange(self.num_envs, device=self.device)

        # Keep an immutable mask for the rest of this update.  ``pending`` used to alias
        # ``_base_mocap_reset_pending`` and was cleared below, so it could not be used to protect
        # the reset rows from the ordinary dropout/receive path afterwards.
        reset_rows = self._base_mocap_reset_pending.clone()
        if bool(reset_rows.any()):
            # A true reset starts a new localization session. Backfill every delay slot with the
            # post-reset pose BEFORE selecting the delayed candidate.  Tensor advanced indexing
            # materializes a copy; selecting first and backfilling second would leave a stale
            # previous-episode candidate in hand even though the ring itself had been cleared.
            reset_quat = quat_mul(
                current_quat[reset_rows],
                self._base_mocap_extrinsic_residual_quat[reset_rows],
            )
            reset_quat = reset_quat / torch.linalg.norm(
                reset_quat, dim=-1, keepdim=True
            ).clamp_min(1.0e-9)
            self._base_mocap_delay_buf[:, reset_rows] = current[reset_rows].unsqueeze(0)
            self._base_mocap_quat_delay_buf[:, reset_rows] = reset_quat.unsqueeze(0)
            self._actor_base_pos_w[reset_rows] = current[reset_rows]
            self._actor_base_quat_w[reset_rows] = reset_quat
            self._base_mocap_last_received_pos[reset_rows] = current[reset_rows]
            self._actor_base_velocity_xy[reset_rows] = 0.0
            self._actor_base_age_s[reset_rows] = 0.0
            self._base_mocap_steps_since_receive[reset_rows] = 0
            self._base_mocap_have_previous[reset_rows] = True
            self._base_mocap_reset_pending[reset_rows] = False

        # Read only after reset rows have been backfilled.  Advanced indexing returns independent
        # tensors, so these candidates now contain the post-reset pose for every reset environment
        # while non-reset environments retain their configured delayed stream.
        candidate = self._base_mocap_delay_buf[read_rows, env_rows]
        candidate_quat = self._base_mocap_quat_delay_buf[read_rows, env_rows]

        self._base_mocap_steps_since_receive.add_(1)
        due = (self._base_mocap_step_counter % self._base_mocap_update_interval_steps) == 0
        receive = torch.full(
            (self.num_envs,), bool(due), dtype=torch.bool, device=self.device
        )
        effective_dropout = self._base_mocap_dropout_prob * robustness_scale
        if effective_dropout > 0.0:
            receive &= (
                torch.rand(self.num_envs, device=self.device)
                >= effective_dropout
            )
        # Reset initialization is not a transport packet and must not be dropped or deferred.  It
        # establishes the new episode's localization origin and makes the first returned actor
        # observation current with zero differentiated velocity.
        receive[reset_rows] = True

        if bool(receive.any()):
            elapsed = self._base_mocap_steps_since_receive[receive].float().clamp_min(1.0) * dt
            inst = (
                candidate[receive, :2]
                - self._base_mocap_last_received_pos[receive, :2]
            ) / elapsed.unsqueeze(-1)
            alpha = self._base_mocap_velocity_alpha
            filtered = (
                alpha * inst
                + (1.0 - alpha) * self._actor_base_velocity_xy[receive]
            )
            have_previous = self._base_mocap_have_previous[receive].unsqueeze(-1)
            self._actor_base_velocity_xy[receive] = torch.where(
                have_previous, filtered, torch.zeros_like(filtered)
            )
            self._actor_base_pos_w[receive] = candidate[receive]
            received_quat = candidate_quat[receive]
            # q and -q are the same attitude. Keep one continuous representative
            # so delayed packets cannot create a fake 2*pi jump.
            quat_dot = torch.sum(
                received_quat * self._actor_base_quat_w[receive],
                dim=-1,
                keepdim=True,
            )
            sign = torch.where(
                quat_dot < 0.0,
                -torch.ones_like(quat_dot),
                torch.ones_like(quat_dot),
            )
            received_quat = received_quat * sign
            self._actor_base_quat_w[receive] = received_quat / torch.linalg.norm(
                received_quat, dim=-1, keepdim=True
            ).clamp_min(1.0e-9)
            self._base_mocap_last_received_pos[receive] = candidate[receive]
            self._actor_base_age_s[receive] = 0.0
            self._base_mocap_steps_since_receive[receive] = 0
            self._base_mocap_have_previous[receive] = True

        self._actor_base_age_s[~receive] += dt
        propagate = (
            (~receive)
            & self._base_mocap_orientation_enabled
            & (self._actor_base_age_s <= self._base_mocap_max_propagation_s)
        )
        if bool(propagate.any()):
            omega = self.robot.data.root_ang_vel_b[propagate]
            angle = torch.linalg.norm(omega, dim=-1) * dt
            half = 0.5 * angle
            axis = omega / torch.linalg.norm(
                omega, dim=-1, keepdim=True
            ).clamp_min(1.0e-9)
            delta_quat = torch.cat(
                [
                    torch.cos(half).unsqueeze(-1),
                    axis * torch.sin(half).unsqueeze(-1),
                ],
                dim=-1,
            )
            propagated = quat_mul(
                self._actor_base_quat_w[propagate], delta_quat
            )
            self._actor_base_quat_w[propagate] = propagated / torch.linalg.norm(
                propagated, dim=-1, keepdim=True
            ).clamp_min(1.0e-9)
        if not self._base_mocap_orientation_enabled:
            self._actor_base_quat_w.copy_(current_quat)
        stale = self._actor_base_age_s >= self._base_mocap_max_age_s
        self._actor_base_velocity_xy[stale] = 0.0
        self._base_mocap_step_counter += 1

        true_vel = self.robot.data.root_lin_vel_w[:, :2]
        self.metrics["base_mocap_age_s"].copy_(self._actor_base_age_s)
        self.metrics["base_mocap_velocity_error"].copy_(torch.linalg.norm(
            self._actor_base_velocity_xy - true_vel, dim=-1
        ))
        self.metrics["base_mocap_stale"].copy_(stale.float())
        quat_dot = torch.abs(
            torch.sum(self._actor_base_quat_w * current_quat, dim=-1)
        ).clamp(0.0, 1.0)
        self.metrics["base_mocap_orientation_error_rad"].copy_(
            2.0 * torch.acos(quat_dot)
        )

    def _extend_hold_until_settled(self, motion) -> None:
        """Keep the pre-swing hold open until the base has SETTLED at the commanded station (arrival-gated
        swing release, cfg.hold_until_settled). The base ``hold_steps_range`` countdown sets the MINIMUM
        hold; this lengthens it while ``base->station error`` or base speed is still large, capped at
        ``hold_settle_max_extra_steps`` (a safety valve so an unreachable/unstable station cannot hang the
        episode). Runs after MotionCommand decremented ``hold_counter`` this step, so re-flooring it to 1
        keeps ``in_hold`` True next step (freezing the reference phase / tts at the windup). The tts /
        pre_strike for THIS step were already computed at the top of _update_command from the frozen
        phase, so the extension takes effect with one-step granularity (the intended behaviour)."""
        hc = getattr(motion, "hold_counter", None)
        was_holding = motion.metrics.get("in_hold") if hasattr(motion, "metrics") else None
        if hc is None or was_holding is None:
            return  # motion command has no hold state -> nothing to extend
        was_holding = was_holding > 0.5  # in_hold THIS step (held before the decrement; True on the last hold step)
        base_err = torch.norm(self.base_pos_w[:, :2] - self.base_target_pos_w, dim=-1)
        base_speed = torch.norm(self.robot.data.root_lin_vel_w[:, :2], dim=-1)
        settled = (base_err < self.cfg.hold_settle_pos_thresh) & (base_speed < self.cfg.hold_settle_speed_thresh)
        # Optionally also require the base to be SQUARED before release (only bites when heading recovery is
        # on — hold_heading squares up during the hold; default yaw_thresh ~pi makes this a no-op). Same yaw
        # as hold_heading: world-frame base x-heading, 0 == +x. Prevents arming the swing while still yawed.
        if float(self.cfg.hold_settle_yaw_thresh) < 3.14159:
            q = self.base_quat_w  # (w, x, y, z)
            fwd_x = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
            fwd_y = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
            yaw_abs = torch.atan2(fwd_y, fwd_x).abs()
            settled = settled & (yaw_abs < self.cfg.hold_settle_yaw_thresh)
        # The base (or a prior extension) countdown just reached 0 this step -> decide whether to hold on.
        countdown_done = was_holding & (hc == 0)
        extend = countdown_done & (~settled) & (self._hold_extra_steps < int(self.cfg.hold_settle_max_extra_steps))
        # Re-floor the counter to 1 for extending envs so in_hold stays True next step (phase stays frozen).
        motion.hold_counter = torch.where(extend, torch.ones_like(hc), hc)
        # Count extension steps while extending; reset to 0 once the env leaves the hold entirely.
        self._hold_extra_steps = torch.where(
            was_holding,
            torch.where(extend, self._hold_extra_steps + 1, self._hold_extra_steps),
            torch.zeros_like(self._hold_extra_steps),
        )
        self.metrics["hold_extra_steps"] = self._hold_extra_steps.float()

    def _effective_target_robustness(self) -> tuple[float, int]:
        """Return weight-ramped corruption scale and integer delay."""

        if self._target_robustness_curriculum_by_recovery_scale:
            start = self._target_robustness_recovery_start_scale
            scale = min(
                max(
                    (float(self._recovery_coverage_scale) - start)
                    / max(1.0 - start, 1.0e-9),
                    0.0,
                ),
                1.0,
            )
            return scale, int(round(float(self._delay_steps) * scale))
        config = self._velocity_stage_config
        return staged_target_robustness(
            self._velocity_stage,
            current_weight=self._velocity_current_weight,
            stage0_weight=config.stage0_weight,
            stage1_weight=config.stage1_weight,
            stage2_weight=config.stage2_weight,
            max_delay_steps=self._delay_steps,
            stage1_scale=self._target_robustness_stage1_scale,
            enabled=self._target_robustness_curriculum_by_velocity_stage,
        )

    def _push_actor_target(self):
        """A1: refresh the ACTOR-visible target view once per control step (latency + jitter).

        Applied on PUSH (not on read) so the jitter is drawn ONCE per step and every actor obs term
        reads the same tensor within the step (determinism). The jitter std decays with the time to
        strike (SMASH Eq. 14 — the mocap ball prediction converges as the strike approaches):
        per-step std = knob * clamp(time_to_strike, 0, 1). The ring buffer stores the jittered
        values, so a delayed read reproduces the prediction noise AS OF push time (what the mocap
        link actually emitted then). The TRUE live target is untouched — rewards, metrics, the
        privileged critic, and the achieved-target-replay write keep reading racket_target_pos_w /
        racket_target_vel_w / swing_sign. time_to_strike is never delayed: the swing clock is
        generated robot-side by the deploy runner, not by the mocap link.
        """
        if not self._actor_view_active:
            return  # default path: delayed_* alias the live tensors — nothing to compute, no RNG
        robustness_scale, effective_delay = self._effective_target_robustness()
        self.metrics["target_robustness_scale"][:] = robustness_scale
        self.metrics["target_delay_steps_in_effect"][:] = float(effective_delay)
        pos = self.racket_target_pos_w
        vel = self.racket_target_vel_w
        jitter_pos = self._jitter_pos * robustness_scale
        jitter_vel = self._jitter_vel * robustness_scale
        if jitter_pos > 0.0 or jitter_vel > 0.0:
            scale = self.time_to_strike.clamp(0.0, 1.0).unsqueeze(-1)
            if jitter_pos > 0.0:
                pos = pos + torch.randn_like(pos) * (jitter_pos * scale)
            if jitter_vel > 0.0:
                vel = vel + torch.randn_like(vel) * (jitter_vel * scale)
        if self._mnoise_ar1_sigma > 0.0:
            if robustness_scale != self._last_target_robustness_scale:
                if self._last_target_robustness_scale > 0.0:
                    self._mnoise_ar1_state.mul_(
                        robustness_scale
                        / self._last_target_robustness_scale
                    )
                else:
                    self._mnoise_ar1_state.zero_()
            if robustness_scale > 0.0:
                rho = self._mnoise_ar1_rho
                self._mnoise_ar1_state.mul_(rho).add_(
                    torch.randn_like(self._mnoise_ar1_state),
                    alpha=(
                        self._mnoise_ar1_sigma
                        * robustness_scale
                        * (1.0 - rho * rho) ** 0.5
                    ),
                )
                pos = pos + self._mnoise_ar1_state
            else:
                self._mnoise_ar1_state.zero_()
        self._last_target_robustness_scale = robustness_scale
        white_noise = self._mnoise_white * robustness_scale
        if white_noise > 0.0:
            pos = pos + torch.randn_like(pos) * white_noise
        if self._a1v2_active:
            # (c) per-swing systematic bias: resample at each strike moment (pre_strike falling edge
            # = sensor re-lock after the contact), constant until the next strike.
            struck = self._prev_pre_strike & ~self.pre_strike
            if self._bias_per_swing > 0.0 and robustness_scale > 0.0 and struck.any():
                self._swing_bias[struck] = torch.randn(int(struck.sum()), 3, device=self.device) * self._bias_per_swing
            # (b) forced hold-last window right after the strike (sensor loses the target at contact)
            effective_post_drop_steps = int(
                round(float(self._post_strike_drop_steps) * robustness_scale)
            )
            if effective_post_drop_steps > 0 and struck.any():
                self._drop_cd[struck] = effective_post_drop_steps
            self._prev_pre_strike.copy_(self.pre_strike)
            pos = pos + self._swing_bias * robustness_scale
            # (a) random frame loss + (b) countdown: actor view HOLDS the last emitted value
            if robustness_scale <= 0.0:
                self._drop_cd.zero_()
            drop = self._drop_cd > 0
            effective_drop_prob = self._drop_prob * robustness_scale
            if effective_drop_prob > 0.0:
                drop = drop | (
                    torch.rand(self.num_envs, device=self.device)
                    < effective_drop_prob
                )
            self._drop_cd = (self._drop_cd - 1).clamp_(min=0)
            d3 = drop.unsqueeze(-1)
            pos = torch.where(d3, self._held_pos, pos)
            vel = torch.where(d3, self._held_vel, vel)
            self._held_pos.copy_(pos)
            self._held_vel.copy_(vel)
        if self._delay_steps > 0:
            # Write this step's (jittered) target into slot `w`; the next slot in the length-
            # (delay+1) ring was written exactly `delay` pushes ago — that is the actor's view.
            w = self._delay_ptr
            self._delay_buf_pos[w].copy_(pos)
            self._delay_buf_vel[w].copy_(vel)
            self._delay_buf_sign[w].copy_(self.swing_sign)
            r = (w - effective_delay) % (self._delay_steps + 1)
            self._delay_ptr = (w + 1) % (self._delay_steps + 1)
            self.delayed_racket_target_pos_w.copy_(self._delay_buf_pos[r])
            self.delayed_racket_target_vel_w.copy_(self._delay_buf_vel[r])
            self.delayed_swing_sign.copy_(self._delay_buf_sign[r])
        else:
            # Jitter-only (delay==0): the actor view is live + this step's noise, no latency.
            self.delayed_racket_target_pos_w.copy_(pos)
            self.delayed_racket_target_vel_w.copy_(vel)
            self.delayed_swing_sign.copy_(self.swing_sign)

    def _count_swing_starts(self, env_ids, count_prestrike_falls: bool) -> None:
        """UNCONDITIONAL swing accounting (Phase A wandb fix). Increment-only here; the decay is
        applied once per step in _update_metrics next to the exact accumulators, so
        swing_completion_rate = exact_n_acc / swing_starts_acc shares one EMA timescale.
        NOTE: an episode TIMEOUT mid-swing counts as an uncompleted start (slight deflation,
        ~one boundary swing per 10 s episode) but never as a fall (terminated excludes timeouts)."""
        n = int(len(env_ids))
        if n == 0:
            return
        self._swing_starts_acc += float(n)
        motion = self._motion()
        if motion._multiseg:
            clips = motion.clip_id[env_ids]
            for c in self._clip_names:
                side_starts = float((clips == c).sum())
                self._swing_starts_acc_c[c] += side_starts
                if int(c) < self._actual_q_window_pending_starts.numel():
                    self._actual_q_window_pending_starts[int(c)] += (
                        side_starts
                    )
        # Rally drift close-out: a WRAP means the previous swing ran to completion — book its base
        # displacement (norm + forward component) from the swing-start stamp to the current base.
        # True resets never close out (the swing was aborted/fallen; the teleport is not drift).
        _ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if not count_prestrike_falls:  # wrap path (see _resample_is_wrap)
            _stamped = ~self._swing_start_pending[_ids_t]  # only swings whose start was stamped
            if bool(_stamped.any()):
                _d = self.base_pos_w[_ids_t, :2] - self._swing_start_base_xy[_ids_t]
                self._drift_sum_acc += float((torch.norm(_d, dim=-1) * _stamped.float()).sum())
                self._drift_fwd_sum_acc += float((_d[:, 0] * _stamped.float()).sum())
                self._drift_n_acc += float(_stamped.sum())
        # Stamp the NEW swing's start lazily (first _update_metrics after this resample): at reset
        # time base_pos_w still caches the PRE-teleport pose.
        self._swing_start_pending[_ids_t] = True
        if count_prestrike_falls:
            # A true reset/timeout closes any unfinished move->hit->recovery funnel. The attempt
            # stays in the denominator and receives no safe-recovery numerator.
            self._chain_recovery_pending[_ids_t] = False
            self._chain_released[_ids_t] = False
            termination_manager = self._env.termination_manager
            term = termination_manager.terminated[env_ids]
            # An optional diagnostic task may terminate when an exogenous arm deadline arrives
            # before the robot is ready.  That is a missed rally, not a physical fall.  Preserve
            # coincident real failures by rebuilding the OR of every other non-timeout term rather
            # than using ``terminated & ~deadline_miss`` (which would hide a fall on the same tick).
            if "arm_deadline_miss" in termination_manager.active_terms:
                physical_term = torch.zeros_like(term)
                for _name, _cfg in zip(
                    termination_manager._term_names, termination_manager._term_cfgs
                ):
                    if _name != "arm_deadline_miss" and not bool(_cfg.time_out):
                        physical_term |= termination_manager.get_term(_name)[env_ids]
                term = physical_term
            pre = self.pre_strike[env_ids]
            # POST-strike fall = terminated at/after the strike frame (tts <= 0, follow-through) OR
            # during the post-wrap hold (_recover_from_clip latch >= 0: the previous swing's recovery,
            # even though the wrap already flipped pre_strike=True for the NEXT swing). Both are the
            # "hit, then fall while recovering" failure that completion + pre-strike metrics miss.
            rec = self._recover_from_clip[env_ids]
            recovering = rec >= 0
            true_pre = term & pre & ~recovering
            post = term & (~pre | recovering)
            # Foundation instrumentation uses the same physical-fall union as
            # the frozen evaluator. Keep the legacy all-non-timeout ``term``
            # path above for existing curriculum/EMA behavior, but do not
            # mislabel anchor/EE guard terminations as physical falls.
            foundation_physical_fall = torch.zeros_like(term)
            for fall_name in ("base_fell_tilt", "base_too_low"):
                try:
                    foundation_physical_fall |= termination_manager.get_term(
                        fall_name
                    )[env_ids].bool()
                except (AttributeError, KeyError, RuntimeError, ValueError):
                    continue
            foundation_true_pre = (
                foundation_physical_fall & pre & ~recovering
            )
            foundation_post = foundation_physical_fall & (
                ~pre | recovering
            )
            foundation_context = torch.where(
                recovering,
                self._foundation_recovery_context_id[env_ids],
                self._foundation_context_id[env_ids],
            )
            valid_foundation = foundation_context >= 0
            self._foundation_prefall_count += torch.bincount(
                foundation_context[foundation_true_pre & valid_foundation],
                minlength=self._stability_size,
            ).to(self._foundation_prefall_count.dtype)
            self._foundation_postfall_count += torch.bincount(
                foundation_context[foundation_post & valid_foundation],
                minlength=self._stability_size,
            ).to(self._foundation_postfall_count.dtype)
            self._prestrike_fall_acc += float(true_pre.sum())
            self._poststrike_fall_acc += float(post.sum())
            audit_context = torch.where(
                recovering,
                self._recover_strike_audit_context_id[env_ids],
                self._strike_audit_context_id[env_ids],
            )
            audit_post = post & (audit_context >= 0)
            self._strike_audit_postfall_count += torch.bincount(
                audit_context[audit_post], minlength=self._strike_audit_size
            ).to(self._strike_audit_postfall_count.dtype)
            if motion._multiseg:
                # Attribute the fall to the clip the env was ON when it fell: pre-strike falls to the
                # _prev_clip_id snapshot (motion already resampled clip_id for the new episode);
                # post-wrap-hold falls to the latched clip whose swing caused the recovery.
                fall_clips = torch.where(recovering, rec, self._prev_clip_id[env_ids])
                for c in self._clip_names:
                    csel = fall_clips == c
                    self._prestrike_fall_acc_c[c] += float((true_pre & csel).sum())
                    self._poststrike_fall_acc_c[c] += float((post & csel).sum())
            # True reset: the new episode starts fresh (its stand-start/reset hold is genuine
            # pre-strike preparation, not recovery), so clear the latch for these envs.
            env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            self._recover_from_clip[env_ids_t] = -1
            self._recover_strike_audit_context_id[env_ids_t] = -1
            self._foundation_recovery_context_id[env_ids_t] = -1
            self._recovery_timer_active[env_ids_t] = False
            # ... and restore the FULL arrival-gate extension budget (2026-07-09 audit fix): the
            # buffer is otherwise only zeroed by _extend_hold_until_settled's ~was_holding branch,
            # and a reset landing mid-extension goes straight into the new episode's first hold
            # (in_hold True from step one) — the spent budget would silently shrink that hold's
            # extension cap.
            self._hold_extra_steps[env_ids_t] = 0
            # Any true termination breaks the consecutive-rally chain, whether it happened before
            # contact or in recovery after a successful strike.  Timeout/reset boundaries also start
            # a fresh closed-loop sequence, so clear every resampled env on this path.
            self._rally_success_run[env_ids_t] = 0
            self._rally_success_run_max[env_ids_t] = 0

    def _accumulate_strike_instrumentation(
        self,
        *,
        exact_strike: torch.Tensor,
        racket_pos_error: torch.Tensor,
        base_error_xy: torch.Tensor,
        pass_position: torch.Tensor,
        pass_velocity: torch.Tensor,
        pass_normal: torch.Tensor,
        pass_composite: torch.Tensor,
    ) -> None:
        """Accumulate exact-strike, target-stratified diagnostics without affecting the task."""
        context = self._strike_audit_context_id
        valid = exact_strike & (context >= 0)
        indices = context[valid]
        self._strike_audit_count += torch.bincount(
            indices, minlength=self._strike_audit_size
        ).to(self._strike_audit_count.dtype)
        for axis in range(3):
            self._strike_audit_pos_signed_sum[:, axis] += torch.bincount(
                indices,
                weights=racket_pos_error[valid, axis],
                minlength=self._strike_audit_size,
            )
            self._strike_audit_pos_abs_sum[:, axis] += torch.bincount(
                indices,
                weights=torch.abs(racket_pos_error[valid, axis]),
                minlength=self._strike_audit_size,
            )
        for axis in range(2):
            self._strike_audit_base_signed_sum[:, axis] += torch.bincount(
                indices,
                weights=base_error_xy[valid, axis],
                minlength=self._strike_audit_size,
            )
            self._strike_audit_base_abs_sum[:, axis] += torch.bincount(
                indices,
                weights=torch.abs(base_error_xy[valid, axis]),
                minlength=self._strike_audit_size,
            )
        for channel, passed in enumerate(
            (pass_position, pass_velocity, pass_normal, pass_composite)
        ):
            self._strike_audit_pass_sum[:, channel] += torch.bincount(
                indices,
                weights=passed[valid].float(),
                minlength=self._strike_audit_size,
            )

    def _accumulate_qdes_joint_instrumentation(
        self, exact_strike: torch.Tensor
    ) -> None:
        """Accumulate per-action-column exact-strike q_des diagnostics."""
        try:
            action_term = self._env.action_manager.get_term("joint_pos")
        except (AttributeError, ValueError):
            return
        interval_fraction = getattr(
            action_term, "feasible_interval_width_fraction", None
        )
        utilization = getattr(action_term, "feasible_action_utilization", None)
        executed_delta = getattr(action_term, "executed_qdes_delta", None)
        raw_action = getattr(action_term, "raw_actions", None)
        if not all(
            torch.is_tensor(value)
            for value in (
                interval_fraction,
                utilization,
                executed_delta,
                raw_action,
            )
        ):
            return
        num_joints = int(raw_action.shape[-1])
        if not self._qdes_joint_audit_names:
            names = tuple(
                str(value) for value in getattr(action_term, "_joint_names", ())
            )
            if len(names) != num_joints:
                names = tuple(f"action_{index}" for index in range(num_joints))
            self._qdes_joint_audit_names = names
        if self._qdes_joint_audit_sum is None:
            # signed raw, signed tanh, signed executed delta, then their absolute magnitudes,
            # interval width in rad/fraction, and feasible-interval utilization.
            self._qdes_joint_audit_sum = torch.zeros(
                num_joints, 9, device=self.device
            )
            self._qdes_joint_audit_max = torch.zeros(
                num_joints, 3, device=self.device
            )
            self._qdes_joint_tightest_count = torch.zeros(
                num_joints, 4, device=self.device
            )

        rows = exact_strike
        count = rows.sum().to(self._qdes_joint_audit_count.dtype)
        self._qdes_joint_audit_count += count
        safe_lo = getattr(action_term, "_safe_lo")
        safe_hi = getattr(action_term, "_safe_hi")
        interval_width_rad = interval_fraction * (safe_hi - safe_lo)
        tanh_action = torch.tanh(raw_action)
        values = (
            raw_action,
            tanh_action,
            executed_delta,
            torch.abs(raw_action),
            torch.abs(tanh_action),
            torch.abs(executed_delta),
            interval_width_rad,
            interval_fraction,
            utilization,
        )
        for column, value in enumerate(values):
            self._qdes_joint_audit_sum[:, column] += value[rows].sum(dim=0)
        for column, value in enumerate(
            (torch.abs(raw_action), torch.abs(tanh_action), torch.abs(executed_delta))
        ):
            masked_value = torch.where(
                rows.unsqueeze(-1), value, torch.zeros_like(value)
            )
            self._qdes_joint_audit_max[:, column] = torch.maximum(
                self._qdes_joint_audit_max[:, column],
                masked_value.max(dim=0).values,
            )

        rate = getattr(action_term, "feasible_rate_bound_active")[rows]
        tracking = getattr(action_term, "feasible_tracking_bound_active")[rows]
        torque = getattr(action_term, "feasible_torque_bound_active")[rows]
        # Categorical priority follows the nested executable interval: rate is most local,
        # then tracking, then torque, otherwise the safe range itself is the limiting interval.
        tightest = torch.zeros_like(rate, dtype=torch.long)
        tightest = torch.where(torque, torch.full_like(tightest, 3), tightest)
        tightest = torch.where(tracking, torch.full_like(tightest, 2), tightest)
        tightest = torch.where(rate, torch.ones_like(tightest), tightest)
        for category in range(4):
            self._qdes_joint_tightest_count[:, category] += (
                tightest == category
            ).sum(dim=0)

    def instrumentation_scalars(self) -> dict[str, torch.Tensor]:
        """Return compact cumulative audit scalars for the runner's batched logger."""
        result: dict[str, torch.Tensor] = {}
        count = self._strike_audit_count
        denom = count.clamp_min(1.0)
        starts = self._strike_audit_start_count
        start_denom = starts.clamp_min(1.0)
        clip_names = ("fh", "bh")
        source_names = ("core", "planner")
        z_names = ("low", "mid", "high")
        for clip in range(2):
            for source in range(2):
                for speed in range(4):
                    for z_bin in range(3):
                        index = (((clip * 2 + source) * 4 + speed) * 3 + z_bin)
                        prefix = (
                            f"strike_matrix/{clip_names[clip]}/"
                            f"{source_names[source]}/speed_q{speed + 1}/z_{z_names[z_bin]}"
                        )
                        result[f"{prefix}/sample_count"] = count[index]
                        for axis, name in enumerate(("x", "y", "z")):
                            result[f"{prefix}/position_error_signed_{name}_m"] = (
                                self._strike_audit_pos_signed_sum[index, axis]
                                / denom[index]
                            )
                            result[f"{prefix}/position_error_abs_{name}_m"] = (
                                self._strike_audit_pos_abs_sum[index, axis]
                                / denom[index]
                            )
                        for axis, name in enumerate(("x", "y")):
                            result[f"{prefix}/base_station_error_signed_{name}_m"] = (
                                self._strike_audit_base_signed_sum[index, axis]
                                / denom[index]
                            )
                            result[f"{prefix}/base_station_error_abs_{name}_m"] = (
                                self._strike_audit_base_abs_sum[index, axis]
                                / denom[index]
                            )
                        for channel, name in enumerate(
                            ("position_pass", "velocity_pass", "normal_pass", "composite")
                        ):
                            result[f"{prefix}/{name}"] = (
                                self._strike_audit_pass_sum[index, channel]
                                / denom[index]
                            )
                        result[f"{prefix}/swing_start_count"] = starts[index]
                        result[f"{prefix}/post_strike_fall"] = (
                            self._strike_audit_postfall_count[index]
                            / start_denom[index]
                        )
                        position_event_count = (
                            self._strike_local_position_event_count[index]
                        )
                        debt_event_count = (
                            self._strike_local_exact_debt_event_count[index]
                        )
                        result[
                            f"{prefix}/position_guidance_event_count"
                        ] = position_event_count
                        result[
                            f"{prefix}/position_guidance_event_return"
                        ] = (
                            self._strike_local_position_event_sum[index]
                            / position_event_count.clamp_min(1.0)
                        )
                        result[
                            f"{prefix}/exact_position_debt_event_count"
                        ] = debt_event_count
                        result[
                            f"{prefix}/exact_position_debt_event_return"
                        ] = (
                            self._strike_local_exact_debt_event_sum[index]
                            / debt_event_count.clamp_min(1.0)
                        )

        if (
            self._qdes_joint_audit_sum is not None
            and self._qdes_joint_audit_max is not None
            and self._qdes_joint_tightest_count is not None
        ):
            joint_denom = self._qdes_joint_audit_count.clamp_min(1.0)
            mean_names = (
                "raw_action_signed_mean",
                "tanh_action_signed_mean",
                "executed_qdes_delta_signed_mean_rad",
                "raw_action_abs_mean",
                "tanh_action_abs_mean",
                "executed_qdes_delta_abs_mean_rad",
                "feasible_interval_width_mean_rad",
                "feasible_interval_width_mean_fraction",
                "feasible_interval_utilization_mean",
            )
            max_names = (
                "raw_action_abs_max",
                "tanh_action_abs_max",
                "executed_qdes_delta_abs_max_rad",
            )
            tightest_names = ("safe", "rate", "tracking", "torque")
            for joint, name in enumerate(self._qdes_joint_audit_names):
                label = name.removesuffix("_joint")
                prefix = f"qdes_joint/{label}"
                for column, metric_name in enumerate(mean_names):
                    result[f"{prefix}/{metric_name}"] = (
                        self._qdes_joint_audit_sum[joint, column] / joint_denom
                    )
                for column, metric_name in enumerate(max_names):
                    result[f"{prefix}/{metric_name}"] = self._qdes_joint_audit_max[
                        joint, column
                    ]
                for category, metric_name in enumerate(tightest_names):
                    result[f"{prefix}/tightest_{metric_name}_fraction"] = (
                        self._qdes_joint_tightest_count[joint, category]
                        / joint_denom
                    )
        return result

    def _update_foundation_stability_instrumentation(self) -> None:
        """Accumulate phase/context stability signals without feeding policy or reward."""

        recovering = self._recover_from_clip >= 0
        context = torch.where(
            recovering,
            self._foundation_recovery_context_id,
            self._foundation_context_id,
        )
        valid = context >= 0
        data = self.robot.data
        tilt_deg = torch.rad2deg(
            torch.asin(self.proj_grav_xy.clamp(0.0, 1.0))
        )
        angular_velocity = torch.linalg.norm(data.root_ang_vel_b, dim=-1)
        linear_velocity = torch.linalg.norm(data.root_lin_vel_w, dim=-1)
        foot_contact = self.metrics["foot_contact_frac"]
        foot_slip = self.metrics["foot_slip_speed"]
        support_proxy = foot_contact * (1.0 - self.proj_grav_xy.clamp(0.0, 1.0))
        station_error = torch.linalg.norm(
            self.base_pos_w[:, :2] - self.base_target_pos_w, dim=-1
        )
        quat = self.base_quat_w
        yaw_error = torch.abs(
            torch.atan2(
                2.0 * (quat[:, 1] * quat[:, 2] + quat[:, 0] * quat[:, 3]),
                1.0 - 2.0 * (quat[:, 2] ** 2 + quat[:, 3] ** 2),
            )
        )
        displacement = self.base_pos_w[:, :2] - self._station_command_start_xy
        recovery_drift = self.base_pos_w[:, :2] - self._recovery_strike_xy
        liftoff = self._stability_prev_foot_contact & (~self._feet_in_contact)
        liftoff_event = liftoff.float().sum(dim=-1)
        self._stability_prev_foot_contact.copy_(self._feet_in_contact)
        torque_peak = self.metrics["joint_torque_abs_max"]
        near_limit = self.metrics["joint_pos_near_limit_frac"]
        try:
            action_term = self._env.action_manager.get_term("joint_pos")
        except (AttributeError, ValueError):
            action_term = None
        interval_width = getattr(
            action_term, "feasible_interval_width_fraction", None
        )
        if torch.is_tensor(interval_width):
            interval_width = interval_width.min(dim=-1).values
        else:
            interval_width = torch.ones(self.num_envs, device=self.device)
        values = torch.stack(
            (
                tilt_deg,
                angular_velocity,
                linear_velocity,
                foot_contact,
                foot_slip,
                support_proxy,
                station_error,
                torch.rad2deg(yaw_error),
                torch.abs(displacement[:, 0]),
                torch.abs(displacement[:, 1]),
                1.0 - foot_contact,
                liftoff_event,
                torque_peak,
                near_limit,
                interval_width,
                torch.abs(recovery_drift[:, 0]),
                torch.abs(recovery_drift[:, 1]),
                (self._step_reentry > 0).float(),
                self._ready_latched.float(),
            ),
            dim=-1,
        )
        strike_window = self.time_to_strike.abs() <= float(self.cfg.strike_window_s)
        phase_masks = (
            (~recovering)
            & (self.time_to_strike > float(self.cfg.strike_window_s)),
            (~recovering) & strike_window,
            (~recovering)
            & (self.time_to_strike < -float(self.cfg.strike_window_s)),
            recovering,
        )
        for phase, mask in enumerate(phase_masks):
            selected = valid & mask
            ids = context[selected]
            if ids.numel() == 0:
                continue
            selected_values = values[selected]
            self._stability_sample_count[phase].index_add_(
                0, ids, torch.ones(ids.numel(), device=self.device)
            )
            self._stability_metric_sum[phase].index_add_(
                0, ids, selected_values
            )
            expanded = ids.unsqueeze(-1).expand_as(selected_values)
            self._stability_metric_max[phase].scatter_reduce_(
                0,
                expanded,
                selected_values,
                reduce="amax",
                include_self=True,
            )
            self._stability_metric_min[phase].scatter_reduce_(
                0,
                expanded,
                selected_values,
                reduce="amin",
                include_self=True,
            )

        # Recovery event clocks start at the physical strike and remain bound to that strike's
        # context across the wrap into the subsequent recovery hold.
        crossed_strike = (
            (self._foundation_prev_tts > 0.0)
            & (self.time_to_strike <= 0.0)
            & (self._foundation_context_id >= 0)
        )
        self._recovery_timer_active |= crossed_strike
        self._recovery_age_s = torch.where(
            crossed_strike, torch.zeros_like(self._recovery_age_s), self._recovery_age_s
        )
        self._recovery_strike_xy = torch.where(
            crossed_strike.unsqueeze(-1),
            self.base_pos_w[:, :2],
            self._recovery_strike_xy,
        )
        self._foundation_recovery_context_id = torch.where(
            crossed_strike,
            self._foundation_context_id,
            self._foundation_recovery_context_id,
        )
        self._recovery_event_seen[crossed_strike] = False
        self._recovery_age_s.add_(
            self._recovery_timer_active.float() * float(self._env.step_dt)
        )
        both_feet = self._feet_in_contact.all(dim=-1)
        recovered = (
            (self.proj_grav_xy <= float(self.cfg.ready_monitor_tilt_thresh))
            & (torch.linalg.norm(data.root_lin_vel_w[:, :2], dim=-1)
               <= self._step_settle_speed_thresh)
            & (yaw_error <= self._step_settle_yaw_thresh)
            & both_feet
            & (foot_slip <= self._step_settle_slip_thresh)
        )
        stand = recovering & self._locomotion_supervision & (~self._locomotion_move)
        ready = recovering & self._ready_latched
        event_conditions = (recovered, both_feet, stand, ready)
        event_context = self._foundation_recovery_context_id
        for event, condition in enumerate(event_conditions):
            first = (
                self._recovery_timer_active
                & condition
                & (~self._recovery_event_seen[:, event])
                & (event_context >= 0)
            )
            ids = event_context[first]
            if ids.numel() > 0:
                self._recovery_event_time_sum[:, event].index_add_(
                    0, ids, self._recovery_age_s[first]
                )
                self._recovery_event_count[:, event].index_add_(
                    0, ids, torch.ones(ids.numel(), device=self.device)
                )
            self._recovery_event_seen[:, event] |= first
        self._recovery_timer_active &= ~(
            self._recovery_event_seen[:, 3]
            | (
                self._recovery_age_s
                >= float(getattr(self._env, "max_episode_length_s", 60.0))
            )
        )
        self._foundation_prev_tts.copy_(self.time_to_strike)

    def foundation_instrumentation_scalars(self) -> dict[str, torch.Tensor]:
        """Cumulative V15 stability/one-step evidence, independent of reward."""

        result: dict[str, torch.Tensor] = {}
        bucket_names = ("same", "0_0p10", "0p10_0p20", "0p20_0p30", "gt_0p30")
        for bucket, name in enumerate(bucket_names):
            command_denom = self._one_step_command_count[bucket].clamp_min(1.0)
            attempt_denom = self._one_step_attempt_count[bucket].clamp_min(1.0)
            prefix = f"one_step/distance_{name}"
            result[f"{prefix}/command_count"] = self._one_step_command_count[bucket]
            result[f"{prefix}/step_attempt_count"] = self._one_step_attempt_count[
                bucket
            ]
            result[f"{prefix}/one_step_success"] = (
                self._one_step_success_count[bucket] / attempt_denom
            )
            result[f"{prefix}/step_reentry_fraction"] = (
                self._one_step_reentry_count[bucket] / attempt_denom
            )
            result[f"{prefix}/fall_before_next_swing_fraction"] = (
                self._one_step_fall_count[bucket] / attempt_denom
            )
            for label, values in self._one_step_metric_sum.items():
                result[f"{prefix}/{label}_mean"] = values[bucket] / attempt_denom
            # Expose the denominator distinction explicitly: same-station commands expect zero
            # STEP bouts and therefore are not silently counted as one-step successes.
            result[f"{prefix}/step_attempt_fraction"] = (
                self._one_step_attempt_count[bucket] / command_denom
            )
        total_attempts = self._one_step_attempt_count.sum().clamp_min(1.0)
        result["one_step/all/attempt_count"] = self._one_step_attempt_count.sum()
        result["one_step/all/one_step_success"] = (
            self._one_step_success_count.sum() / total_attempts
        )
        result["one_step/all/step_reentry_fraction"] = (
            self._one_step_reentry_count.sum() / total_attempts
        )
        result["one_step/all/fall_before_next_swing_fraction"] = (
            self._one_step_fall_count.sum() / total_attempts
        )
        result["one_step/threshold/station_error_m"] = torch.tensor(
            self._step_settle_pos_thresh, device=self.device
        )
        result["one_step/threshold/base_speed_mps"] = torch.tensor(
            self._step_settle_speed_thresh, device=self.device
        )
        result["one_step/threshold/yaw_error_rad"] = torch.tensor(
            self._step_settle_yaw_thresh, device=self.device
        )
        result["one_step/threshold/contact_force_n"] = torch.tensor(
            self._step_settle_contact_force_threshold, device=self.device
        )
        result["one_step/threshold/foot_slip_mps"] = torch.tensor(
            self._step_settle_slip_thresh, device=self.device
        )
        result["one_step/threshold/stable_dwell_s"] = torch.tensor(
            self._step_settle_dwell_steps * float(self._env.step_dt),
            device=self.device,
        )

        def add_chain(prefix: str, selection) -> None:
            attempts = self._chain_attempt_count[selection].sum()
            step = self._chain_step_settled_count[selection].sum()
            ready = self._chain_ready_release_count[selection].sum()
            exact = self._chain_exact_frame_count[selection].sum()
            hit = self._chain_exact_hit_count[selection].sum()
            safe = self._chain_safe_recovery_count[selection].sum()
            attempt_denom = attempts.clamp_min(1.0)
            hit_denom = hit.clamp_min(1.0)
            result[f"{prefix}/attempt_count"] = attempts
            result[f"{prefix}/step_settled_count"] = step
            result[f"{prefix}/ready_release_count"] = ready
            result[f"{prefix}/exact_frame_count"] = exact
            result[f"{prefix}/exact_hit_count"] = hit
            result[f"{prefix}/safe_recovery_count"] = safe
            result[f"{prefix}/step_settled_fraction"] = step / attempt_denom
            result[f"{prefix}/ready_release_fraction"] = ready / attempt_denom
            result[f"{prefix}/exact_frame_fraction"] = exact / attempt_denom
            result[f"{prefix}/exact_hit_fraction"] = hit / attempt_denom
            result[f"{prefix}/safe_recovery_given_hit"] = safe / hit_denom
            result[f"{prefix}/end_to_end_success"] = safe / attempt_denom

        add_chain("chain/all", (slice(None), slice(None)))
        for clip, clip_name in enumerate(("fh", "bh")):
            add_chain(f"chain/{clip_name}/all", (clip, slice(None)))
            for bucket, bucket_name in enumerate(bucket_names):
                add_chain(
                    f"chain/{clip_name}/distance_{bucket_name}",
                    (clip, bucket),
                )

        metric_index = {
            name: index
            for index, name in enumerate(self._stability_metric_names)
        }
        phase_metrics = {
            "pre_strike": (
                ("base_tilt_deg", "mean"),
                ("base_tilt_deg", "max"),
                ("base_angular_velocity_rad_s", "mean"),
                ("base_linear_velocity_mps", "mean"),
                ("foot_contact_fraction", "mean"),
                ("foot_slip_mps", "max"),
                ("com_support_proxy", "mean"),
                ("station_error_m", "mean"),
                ("yaw_error_deg", "max"),
            ),
            "strike": (
                ("base_tilt_deg", "max"),
                ("base_x_excursion_m", "max"),
                ("base_y_excursion_m", "max"),
                ("foot_unloading_fraction", "max"),
                ("foot_liftoff_event_count", "sum"),
                ("foot_slip_mps", "max"),
                ("torque_peak_nm", "max"),
                ("joint_near_limit_fraction", "max"),
                ("qdes_interval_width_min_fraction", "min_proxy"),
            ),
            "post_strike": (
                ("base_tilt_deg", "max"),
                ("base_angular_velocity_rad_s", "max"),
                ("base_linear_velocity_mps", "max"),
                ("foot_contact_fraction", "mean"),
                ("foot_slip_mps", "max"),
                ("base_x_drift_m", "max"),
                ("base_y_drift_m", "max"),
                ("yaw_error_deg", "max"),
                ("step_reentry", "max"),
                ("next_strike_readiness", "mean"),
            ),
            "recovery": (
                ("base_tilt_deg", "max"),
                ("base_angular_velocity_rad_s", "max"),
                ("base_linear_velocity_mps", "max"),
                ("foot_contact_fraction", "mean"),
                ("foot_slip_mps", "max"),
                ("base_x_drift_m", "max"),
                ("base_y_drift_m", "max"),
                ("yaw_error_deg", "max"),
                ("step_reentry", "max"),
                ("next_strike_readiness", "mean"),
            ),
        }
        clip_names = ("fh", "bh")
        source_names = ("core", "planner")
        distance_names = ("same", "0_0p10", "0p10_0p20", "0p20_0p30", "gt_0p30")
        order_names = ("first_strike", "later_strikes")
        for clip in range(2):
            for source in range(2):
                for speed in range(4):
                    for distance in range(5):
                        for order in range(2):
                            context = (
                                (((clip * 2 + source) * 4 + speed) * 5 + distance)
                                * 2
                                + order
                            )
                            label = (
                                f"{clip_names[clip]}/{source_names[source]}/"
                                f"speed_q{speed + 1}/distance_{distance_names[distance]}/"
                                f"{order_names[order]}"
                            )
                            starts = self._foundation_start_count[
                                context
                            ].clamp_min(1.0)
                            for phase, phase_name in enumerate(
                                self._stability_phases
                            ):
                                count = self._stability_sample_count[
                                    phase, context
                                ]
                                denom = count.clamp_min(1.0)
                                prefix = f"stability/{phase_name}/{label}"
                                result[f"{prefix}/sample_count"] = count
                                for metric, reducer in phase_metrics[phase_name]:
                                    column = metric_index[metric]
                                    if reducer == "mean":
                                        value = (
                                            self._stability_metric_sum[
                                                phase, context, column
                                            ]
                                            / denom
                                        )
                                        suffix = f"{metric}_mean"
                                    elif reducer == "sum":
                                        value = self._stability_metric_sum[
                                            phase, context, column
                                        ]
                                        suffix = f"{metric}_sum"
                                    elif reducer == "min_proxy":
                                        raw = self._stability_metric_min[
                                            phase, context, column
                                        ]
                                        value = torch.where(
                                            count > 0,
                                            raw,
                                            torch.zeros_like(raw),
                                        )
                                        suffix = f"{metric}_min"
                                    else:
                                        raw = self._stability_metric_max[
                                            phase, context, column
                                        ]
                                        value = torch.where(
                                            count > 0,
                                            raw,
                                            torch.zeros_like(raw),
                                        )
                                        suffix = f"{metric}_max"
                                    result[f"{prefix}/{suffix}"] = value
                                if phase_name in ("post_strike", "recovery"):
                                    result[f"{prefix}/post_strike_fall_rate"] = (
                                        self._foundation_postfall_count[context]
                                        / starts
                                    )
                                    result[f"{prefix}/pre_strike_fall_rate"] = (
                                        self._foundation_prefall_count[context]
                                        / starts
                                    )
                            recovery_prefix = f"stability/recovery/{label}"
                            for event, event_name in enumerate(
                                (
                                    "time_to_recover_s",
                                    "time_to_both_feet_contact_s",
                                    "time_to_STAND_s",
                                    "time_to_next_strike_readiness_s",
                                )
                            ):
                                event_count = self._recovery_event_count[
                                    context, event
                                ]
                                result[f"{recovery_prefix}/{event_name}_count"] = (
                                    event_count
                                )
                                result[f"{recovery_prefix}/{event_name}_mean"] = (
                                    self._recovery_event_time_sum[context, event]
                                    / event_count.clamp_min(1.0)
                                )
        return result

    def _update_footwork_signals(self, racket_dist: torch.Tensor) -> None:
        """Base-FREE footwork-to-strike signals (reward/metric only; NEVER observed). The legs are driven
        to move by racket PROGRESS (reducing the racket->target distance), not by any base target. All
        guards degrade to 0 if a body/sensor cannot resolve, so this can never crash training."""
        data = self.robot.data
        # --- racket-target distance + dense progress (the base-free movement driver) ---
        self.racket_target_distance = racket_dist
        # progress = previous - current distance. Resample/reset steps are not learnable progress:
        # the target and/or reference clip jumped, so reset the baseline and emit exactly zero.
        motion = self._motion()
        reset_progress = self._progress_reset_mask.clone()
        if hasattr(motion, "just_resampled"):
            reset_progress |= motion.just_resampled
        progress = (self._prev_racket_dist - racket_dist).clamp(-0.15, 0.15)
        self.racket_progress = torch.where(reset_progress, torch.zeros_like(progress), progress)
        self._prev_racket_dist = racket_dist.detach()
        self._progress_reset_mask.zero_()
        self.metrics["racket_target_distance"] = racket_dist
        self.metrics["racket_progress"] = self.racket_progress
        self.metrics["racket_progress_prestrike"] = torch.where(
            self.pre_strike, self.racket_progress, torch.zeros_like(self.racket_progress)
        )
        # --- base stability components (training-only) ---
        pg = getattr(data, "projected_gravity_b", None)
        if pg is not None:
            self.proj_grav_xy = torch.norm(pg[:, :2], dim=-1)
        self.base_ang_vel_xy_norm = torch.norm(data.root_ang_vel_b[:, :2], dim=-1)
        self.vertical_speed = torch.abs(data.root_lin_vel_b[:, 2])
        self.metrics["proj_grav_xy"] = self.proj_grav_xy
        self.metrics["base_ang_vel_xy"] = self.base_ang_vel_xy_norm
        self.metrics["base_vertical_speed"] = self.vertical_speed
        # P2.4 (PACE smooth deceleration): mean planar base speed during the approach — the quantity
        # the base_decel_tracking reward shapes toward v_des = clamp(v_gain*dist_xy, 0, v_max).
        # Watch it fall on far targets when the term is enabled (task.rewards.base_decel_weight>0).
        base_speed_xy = torch.norm(data.root_lin_vel_w[:, :2], dim=-1)
        self.metrics["base_speed_xy_prestrike"] = torch.where(
            self.pre_strike, base_speed_xy, torch.zeros_like(base_speed_xy)
        )
        # Rally: planar base speed through the FOLLOW-THROUGH (strike-window exit -> wrap) — the
        # braking window the post_strike_brake reward shapes. Held-write (carries the last
        # in-window value between swings) so the tail-mean reads the typical post-strike speed.
        _brake_win = (~self.pre_strike) & (~self.strike_window)
        self.metrics["post_strike_base_speed_xy"] = torch.where(
            _brake_win, base_speed_xy, self.metrics["post_strike_base_speed_xy"]
        )
        # Rally: cumulative displacement from the env origin (the P7 forward-drift accumulator).
        self.metrics["base_dist_from_origin"] = torch.norm(
            self.base_pos_w[:, :2] - self._env.scene.env_origins[:, :2], dim=-1
        )
        # v2 rally heading debt at HOLD EXPIRY (2026-07-08; review fix — sampling at the wrap
        # instant would read the PRE-recovery debt and miss stand-start holds entirely): on the
        # in_hold True->False falling edge — the moment the swing actually arms, after the
        # recovery hold ran — accumulate |base yaw vs world +x|. Covers BOTH wrap holds and
        # stand-start/reset holds (the yawed-spawn data source). This is the self-squaring gate:
        # deploy refuses engages >0.35 rad (20°), so a working rally2 reads well under that.
        _ih = getattr(self._motion(), "in_hold", None)
        if _ih is not None:
            _ihb = _ih.bool()
            _expired = self._prev_in_hold & ~_ihb
            if bool(_expired.any()):
                _qh = self.base_quat_w[_expired]
                _fxh = 1.0 - 2.0 * (_qh[:, 2] ** 2 + _qh[:, 3] ** 2)
                _fyh = 2.0 * (_qh[:, 1] * _qh[:, 2] + _qh[:, 0] * _qh[:, 3])
                _exp_yaw = torch.atan2(_fyh, _fxh).abs()
                self._heading_expiry_sum_acc += float(_exp_yaw.sum())
                self._heading_expiry_n_acc += float(_expired.sum())
                # v3 spawn-conditioned recovery: keep only holds that STARTED yawed, so the near-
                # square wrap holds cannot dilute the signal into a false pass.
                _sp = self._hold_spawn_yaw[_expired]
                _yawed = _sp > _RECOV_SPAWN_YAW_THRESH
                if bool(_yawed.any()):
                    self._recov_spawn_sum_acc += float(_sp[_yawed].sum())
                    self._recov_expiry_sum_acc += float(_exp_yaw[_yawed].sum())
                    self._recov_n_acc += float(_yawed.sum())
            # v3: stamp |base yaw| at each hold RISING edge = the spawn heading-debt of that hold.
            _started = (~self._prev_in_hold) & _ihb
            if bool(_started.any()):
                _qs = self.base_quat_w[_started]
                _fxs = 1.0 - 2.0 * (_qs[:, 2] ** 2 + _qs[:, 3] ** 2)
                _fys = 2.0 * (_qs[:, 1] * _qs[:, 2] + _qs[:, 0] * _qs[:, 3])
                self._hold_spawn_yaw[_started] = torch.atan2(_fys, _fxs).abs()
            self._prev_in_hold = _ihb.clone()
        # Rally: lazy swing-start stamp (fresh base_pos_w — see __init__ rationale).
        if bool(self._swing_start_pending.any()):
            _pend = self._swing_start_pending
            self._swing_start_base_xy[_pend] = self.base_pos_w[_pend, :2]
            self._swing_start_pending.zero_()
        # --- foot footwork signals (slip² / velocity / drag); feet may STEP, so this is PENALTY-only ---
        if self._foot_idx_robot and self._contact_sensor is not None and self._foot_idx_contact:
            f_force = torch.norm(self._contact_sensor.data.net_forces_w[:, self._foot_idx_contact, :], dim=-1)
            in_contact = (
                f_force > self._step_settle_contact_force_threshold
            ).float()  # (E,2)
            f_vel = data.body_lin_vel_w[:, self._foot_idx_robot, :]  # (E,2,3)
            f_xy_speed = torch.norm(f_vel[..., :2], dim=-1)  # (E,2)
            f_speed = torch.norm(f_vel, dim=-1)  # (E,2)
            f_height = data.body_pos_w[:, self._foot_idx_robot, 2]  # (E,2)
            self.foot_slip_sq = (in_contact * f_xy_speed.square()).sum(dim=-1)  # contact * ||v_xy||²
            self.foot_vel_sq = f_speed.square().sum(dim=-1)  # excessive/violent foot motion
            # Phase A fix: the old height gate (f_height < 0.10 m) sat BELOW the planted ankle
            # origin (~0.07 m), so EVERY low step counted as "dragging" — stepping itself was
            # taxed, one of the reasons the policy never learned to move left/right. "Drag" now
            # means lateral speed while the foot is LOADED (in contact) = sliding under load;
            # airborne swing-leg motion is free (foot_vel_sq still bounds violent motion).
            self.foot_drag = (in_contact * f_xy_speed).sum(dim=-1)
            self.metrics["foot_slip_sq"] = self.foot_slip_sq
            self.metrics["foot_vel_mean"] = f_speed.mean(dim=-1)
            self.metrics["foot_lift_rate"] = (1.0 - in_contact).mean(dim=-1)  # 0 = both planted, 1 = airborne
            self.metrics["foot_vel_at_strike"] = torch.where(
                self.strike_window, f_speed.mean(dim=-1), torch.zeros(self.num_envs, device=self.device)
            )
        # --- anti-arm-only: ARM joints near a limit + arm joint velocity (resolve arm joint idx once) ---
        if not getattr(self, "_arm_resolved", False):
            self._arm_resolved = True
            self._arm_joint_idx, self._leg_joint_idx, self._waist_twist_idx = [], [], []
            try:
                self._arm_joint_idx = list(self.robot.find_joints([".*shoulder.*", ".*elbow.*", ".*wrist.*"])[0])
            except Exception:
                pass
            try:
                self._leg_joint_idx = list(self.robot.find_joints([".*hip.*", ".*knee.*", ".*ankle.*"])[0])
            except Exception:
                pass
            # waist YAW+ROLL: the "twist/lean instead of step" DOFs the policy uses to face a lateral
            # target without moving its feet. Penalized (pre-strike) so reaching a far target needs footwork.
            # waist_pitch is EXCLUDED (it is the swing wind-up / natural lean, not a lateral-reach cheat).
            try:
                self._waist_twist_idx = list(self.robot.find_joints(["waist_yaw_joint", "waist_roll_joint"])[0])
            except Exception:
                pass
        limits = getattr(data, "soft_joint_pos_limits", None)
        if limits is None:
            limits = getattr(data, "joint_pos_limits", None)
        if self._arm_joint_idx and limits is not None:
            ai = self._arm_joint_idx
            half = ((limits[:, ai, 1] - limits[:, ai, 0]) * 0.5).clamp(min=1e-6)
            d = torch.minimum(
                data.joint_pos[:, ai] - limits[:, ai, 0], limits[:, ai, 1] - data.joint_pos[:, ai]
            ).clamp(min=0.0)
            self.arm_overreach_frac = ((d / half) < 0.1).float().mean(dim=-1)  # within 10% of a limit
            self.metrics["arm_overreach_frac"] = self.arm_overreach_frac
            self.metrics["arm_joint_vel_max"] = torch.max(torch.abs(data.joint_vel[:, ai]), dim=-1).values
        # --- diagnostic: do the LEGS actually move before the strike? (footwork is happening) ---
        if self._leg_joint_idx:
            leg_vel = torch.max(torch.abs(data.joint_vel[:, self._leg_joint_idx]), dim=-1).values
            self.metrics["leg_joint_vel_max"] = leg_vel
            self.metrics["leg_moving_prestrike"] = torch.where(
                self.pre_strike, (leg_vel > 0.2).float(), torch.zeros(self.num_envs, device=self.device)
            )
        # --- anti twist-instead-of-step: |waist_yaw|+|waist_roll| deviation from the default (neutral,
        #     facing-forward) pose. This is the magnitude the prestrike_waist_twist reward penalizes, so
        #     reaching a lateral target by turning the torso (feet planted) becomes costly -> step instead. ---
        if self._waist_twist_idx:
            wi = self._waist_twist_idx
            self.waist_twist = torch.abs(data.joint_pos[:, wi] - data.default_joint_pos[:, wi]).sum(dim=-1)
            self.metrics["waist_twist_abs"] = self.waist_twist
            self.metrics["waist_twist_prestrike"] = torch.where(
                self.pre_strike, self.waist_twist, torch.zeros(self.num_envs, device=self.device)
            )

    def _vb_evaluate(self, exact_strike: torch.Tensor, pos_err: torch.Tensor):
        """Tier-1 at-strike virtual-ball evaluation (rewardDesign.md).

        Runs once per control step from ``_update_metrics`` (rewards/obs read the same fresh
        buffers after ``command_manager.compute()``). Whenever ANY env sits at its exact-strike
        frame, the FULL batch goes through contact + coarse rollout — the cost is kernel-launch
        bound and batch-size independent, so gathering the ~30 striking envs saves nothing
        (verify_tier1 (b)); ``vb_fired`` masks consumption. On strike-free steps the one-shot
        mask is cleared and nothing is computed.
        """
        from whole_body_tracking.tasks.tracking.mdp import virtual_ball as _vb

        if not bool(exact_strike.any()):
            self.vb_fired.zero_()
            return
        if self._vb_params is None:
            self._vb_params = _vb.load_venue_params()
            print(
                f"[RacketTargetCommand] virtual ball ON: venue constants from "
                f"{self._vb_params.source_path} (k_d={self._vb_params.k_d}, "
                f"k_m={self._vb_params.k_m}, e(u_n)={self._vb_params.paddle_e_g1}"
                f"*exp({self._vb_params.paddle_e_g2}*u_n), a_t={self._vb_params.paddle_a_t})",
                flush=True,
            )
        prm = self._vb_params

        v_in, w_in = self.vb_vel_in_w, self.vb_spin_in_w
        v_r, n_face = self.racket_lin_vel_w, self.racket_normal_w
        # CAPTURE GATE: close enough at the strike frame AND paddle moving INTO the ball along the
        # oriented contact normal (a stationary/retreating wall-block scores nothing — verify (c)3).
        n_or = _vb.orient_normal(n_face, v_in, v_r)
        approach = torch.sum(v_r * n_or, dim=-1)
        gate = (
            exact_strike
            & (pos_err < float(self.cfg.vb_capture_radius))
            & (approach > float(self.cfg.vb_min_approach_speed))
        )

        # Achieved-state contact (venue paddle model, e(u_n)) + coarse landing rollout.
        # The rollout must start in the ENV-LOCAL frame: the virtual table landmarks
        # (vb_table_near_x / net / far end) and vb_target_xy are per-env offsets from the env
        # origin, while racket_pos_w is TRUE world frame (env grids span tens of meters at 4096
        # envs — using it raw put every landing ~|env_origin| away from the target; caught in the
        # first vb_warmE14k run: virtual_land_err_m ~62 m). Env grids are pure translations, so
        # velocities, normals, and spins need no correction.
        v_plus, w_plus = _vb.predict_paddle_contact(v_in, v_r, n_face, w_in, prm)
        land = _vb.coarse_landing(
            self.racket_pos_w - self._env.scene.env_origins,
            v_plus,
            w_plus,
            prm,
            surface_z=float(self.cfg.vb_table_surface_z),
            net_x=self._vb_net_x,
            h=float(self.cfg.vb_rollout_h),
            n_steps=int(self.cfg.vb_rollout_steps),
        )
        lx, ly = land["land_xy"][:, 0], land["land_xy"][:, 1]
        on_opp = (
            land["land_valid"] & (lx > self._vb_net_x) & (lx <= self._vb_far_x) & (ly.abs() <= self._vb_half_w)
        )
        depth_ok = lx > (self._vb_net_x + float(self.cfg.vb_min_landing_depth))
        net_clear = land["net_valid"] & (land["net_z"] > self._vb_net_top_z + self._vb_ball_r)
        # Outgoing topspin component about t_hat = z_hat x d_hat of the outgoing horizontal
        # direction (Ace-style): omega . t_hat = -w_x*d_y + w_y*d_x.
        d_xy = v_plus[:, :2]
        d_hat = d_xy / (torch.linalg.norm(d_xy, dim=-1, keepdim=True) + 1e-9)
        topspin = -w_plus[:, 0] * d_hat[:, 1] + w_plus[:, 1] * d_hat[:, 0]
        intended = torch.where(
            self._venue_tuple_selected.unsqueeze(-1),
            self._venue_intended_landing_xy,
            self._vb_target_xy.unsqueeze(0),
        )
        landing_error = torch.linalg.norm(
            land["land_xy"] - intended, dim=-1
        )

        # One-shot caches consumed by hope_rewards.virtual_* THIS step.
        self.vb_fired = gate
        self.vb_landing_xy = land["land_xy"]
        self.vb_landing_valid = land["land_valid"]
        self.vb_on_opponent = on_opp
        self.vb_depth_ok = depth_ok
        self.vb_net_z = land["net_z"]
        self.vb_net_clear = net_clear
        self.vb_net_crossed = land["net_valid"]
        self.vb_topspin = topspin
        self.vb_spin_out_norm = torch.linalg.norm(w_plus, dim=-1)

        # Sample-weighted EMA rates (hit rate over exact-strike samples; outcome rates over captured
        # hits). NOTE: accumulators only decay on strike-carrying steps — exact at 4096 envs (a
        # strike happens ~every step), slightly stale at small env counts (diagnostics only).
        decay = float(self.cfg.exact_success_decay)
        self._vb_exact_acc = decay * self._vb_exact_acc + float(exact_strike.sum())
        self._vb_hit_acc = decay * self._vb_hit_acc + float(gate.sum())
        self._vb_net_acc = decay * self._vb_net_acc + float((gate & net_clear).sum())
        self._vb_land_valid_acc = decay * self._vb_land_valid_acc + float((gate & land["land_valid"]).sum())
        self._vb_inb_acc = decay * self._vb_inb_acc + float((gate & net_clear & on_opp).sum())
        # Physical-bank outcomes are UNCONDITIONAL over resolved selected
        # questions and stay separate per side. A miss/fall therefore cannot
        # vanish from the denominator or hide behind an FH/BH average.
        motion = self._motion()
        venue_exact = (
            exact_strike
            & self._venue_tuple_selected
            & self._venue_tuple_outcome_pending
        )
        for clip_id, clip_name in self._clip_names.items():
            side_exact = venue_exact & (motion.clip_id == clip_id)
            if bool(self.cfg.venue_tuple_unconditional_outcomes):
                # Denominator and results are committed on the same step. A
                # tuple that reset before this frame was committed as a
                # zero-result failure by _resample_command.
                self._venue_swing_starts_acc_c[clip_id] += float(
                    side_exact.sum()
                )
                self._vb_exact_acc_c[clip_id] += float(side_exact.sum())
                self._vb_hit_acc_c[clip_id] += float((side_exact & gate).sum())
                self._vb_net_acc_c[clip_id] += float(
                    (side_exact & gate & net_clear).sum()
                )
                self._vb_legal_acc_c[clip_id] += float(
                    (side_exact & gate & net_clear & on_opp).sum()
                )
                side_landed = side_exact & gate & land["land_valid"]
                self._vb_land_err_sum_c[clip_id] += float(
                    (landing_error * side_landed).sum()
                )
                self._vb_land_err_n_c[clip_id] += float(side_landed.sum())
                denominator_value = self._venue_swing_starts_acc_c[clip_id]
            else:
                self._vb_exact_acc_c[clip_id] = (
                    decay * self._vb_exact_acc_c[clip_id]
                    + float(side_exact.sum())
                )
                self._vb_hit_acc_c[clip_id] = (
                    decay * self._vb_hit_acc_c[clip_id]
                    + float((side_exact & gate).sum())
                )
                self._vb_net_acc_c[clip_id] = (
                    decay * self._vb_net_acc_c[clip_id]
                    + float((side_exact & gate & net_clear).sum())
                )
                self._vb_legal_acc_c[clip_id] = (
                    decay * self._vb_legal_acc_c[clip_id]
                    + float((side_exact & gate & net_clear & on_opp).sum())
                )
                side_landed = side_exact & gate & land["land_valid"]
                self._vb_land_err_sum_c[clip_id] = (
                    decay * self._vb_land_err_sum_c[clip_id]
                    + float((landing_error * side_landed).sum())
                )
                self._vb_land_err_n_c[clip_id] = (
                    decay * self._vb_land_err_n_c[clip_id]
                    + float(side_landed.sum())
                )
                denominator_value = self._vb_exact_acc_c[clip_id]
            side_denominator = max(denominator_value, 1.0e-6)
            self.metrics[f"virtual_sample_count_{clip_name}"][:] = (
                denominator_value
            )
            self.metrics[f"virtual_contact_rate_{clip_name}"][:] = (
                self._vb_hit_acc_c[clip_id] / side_denominator
            )
            self.metrics[f"virtual_over_net_rate_{clip_name}"][:] = (
                self._vb_net_acc_c[clip_id] / side_denominator
            )
            self.metrics[f"virtual_legal_rate_{clip_name}"][:] = (
                self._vb_legal_acc_c[clip_id] / side_denominator
            )
            self.metrics[f"virtual_landing_error_m_{clip_name}"][:] = (
                self._vb_land_err_sum_c[clip_id]
                / max(self._vb_land_err_n_c[clip_id], 1.0e-6)
            )
        resolved = torch.where(venue_exact)[0]
        if len(resolved) > 0:
            self._venue_tuple_outcome_pending[resolved] = False
            self._venue_tuple_outcome_clip[resolved] = -1
        enough_e = self._vb_exact_acc >= float(self.cfg.exact_success_min_count)
        enough_h = self._vb_hit_acc >= 1.0
        self.metrics["virtual_hit_rate"][:] = (self._vb_hit_acc / max(self._vb_exact_acc, 1e-6)) if enough_e else 0.0
        self.metrics["virtual_net_clear_rate"][:] = (self._vb_net_acc / max(self._vb_hit_acc, 1e-6)) if enough_h else 0.0
        self.metrics["virtual_land_valid_rate"][:] = (
            (self._vb_land_valid_acc / max(self._vb_hit_acc, 1e-6)) if enough_h else 0.0
        )
        self.metrics["virtual_land_inbounds_rate"][:] = (
            (self._vb_inb_acc / max(self._vb_hit_acc, 1e-6)) if enough_h else 0.0
        )
        self.metrics["virtual_approach_speed"] = torch.where(
            exact_strike, approach, self.metrics["virtual_approach_speed"]
        )
        fired_valid = gate & land["land_valid"]
        if bool(fired_valid.any()):
            self.metrics["virtual_land_err_m"][:] = landing_error[
                fired_valid
            ].mean()
            self.metrics["virtual_topspin_revs"][:] = (topspin[fired_valid] / (2.0 * math.pi)).mean()

    @property
    def recovery_curriculum_scale(self) -> float:
        """Live READY/safe-set supervision scale.

        Revision 3 intentionally separates this from coverage scale.  The motion command owns
        replay/post-wrap coverage, while this command owns sampled READY release and its bounded
        safe-set gradient.
        """

        return float(self._recovery_current_scale)

    @property
    def recovery_coverage_scale(self) -> float:
        """Live venue/replay/target-robustness coverage scale."""

        return float(self._recovery_coverage_scale)

    def _advance_actual_q_window(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Commit one control-step bucket and return finite-window fault/start totals."""

        ptr = int(self._actual_q_window_ptr)
        self._actual_q_window_faults[ptr].copy_(
            self._actual_q_window_pending_faults
        )
        self._actual_q_window_starts[ptr].copy_(
            self._actual_q_window_pending_starts
        )
        self._actual_q_window_pending_faults.zero_()
        self._actual_q_window_pending_starts.zero_()
        self._actual_q_window_ptr = (
            ptr + 1
        ) % int(self._actual_q_window_faults.shape[0])
        self._actual_q_window_filled_steps = min(
            self._actual_q_window_filled_steps + 1,
            int(self._actual_q_window_faults.shape[0]),
        )
        return (
            self._actual_q_window_faults.sum(dim=0),
            self._actual_q_window_starts.sum(dim=0),
        )

    def force_final_recovery_stage_for_evaluation(self) -> None:
        """Pin V17's full recovery distribution for deterministic qualification.

        Actor checkpoints do not contain environment command state when loaded by
        ``eval_deterministic.py``.  Without this explicit pin, every evaluation silently restarts
        V17 at Stage 0 / scale 0 and cannot reach Stage 2 during a normal 1200-step rollout.
        """

        if not self._recovery_curriculum_enabled:
            raise RuntimeError(
                "cannot force final recovery stage when recovery_curriculum_enabled is false"
            )
        self._eval_force_final_recovery_stage = True
        self._recovery_stage = 2
        self._recovery_current_scale = 1.0
        self._recovery_target_scale = 1.0
        self._recovery_ramp_rate_per_step = 0.0
        self._recovery_coverage_scale = 1.0
        self._recovery_coverage_target_scale = 1.0
        self._recovery_coverage_ramp_rate_per_step = 0.0
        self._recovery_stage1_coverage_unlocked = True
        self._recovery_stage1_acquisition_rung = (
            len(
                self._recovery_curriculum_config.acquisition_scales()
            )
            - 1
        )
        self._recovery_stage1_ready_dwell = 0
        self._recovery_stage1_acquisition_age = 0
        self._recovery_stage1_enter_dwell = 0
        self._recovery_stage1_exit_dwell = 0
        self._recovery_stage2_enter_dwell = 0
        self._recovery_stage2_exit_dwell = 0
        self._motion().set_recovery_curriculum_scale(1.0)
        self.metrics["recovery_curriculum_stage"][:] = 2.0
        self.metrics["recovery_curriculum_scale"][:] = 1.0
        self.metrics["recovery_curriculum_target_scale"][:] = 1.0
        self.metrics["recovery_coverage_scale"][:] = 1.0
        self.metrics["recovery_coverage_target_scale"][:] = 1.0

    def _ready_acquisition_profile(self) -> dict[str, float | int]:
        """Return the current READY gate, ending exactly at the deploy thresholds."""

        if not bool(self.cfg.ready_acquisition_profile_enabled):
            progress = 1.0
        elif (
            int(self._recovery_stage) == 1
            and not bool(self._recovery_stage1_coverage_unlocked)
        ):
            rung_count = len(
                self._recovery_curriculum_config.acquisition_scales()
            )
            progress = (
                float(self._recovery_stage1_acquisition_rung)
                / float(rung_count - 1)
                if rung_count > 1
                else 1.0
            )
        else:
            # Stage 0 remains a strict diagnostic baseline; Stage 2 and unlocked coverage always
            # use the final deploy READY contract.
            progress = 1.0

        def _lerp(relaxed: float, strict: float) -> float:
            return float(relaxed) + progress * (
                float(strict) - float(relaxed)
            )

        relaxed_dwell = int(
            self.cfg.ready_acquisition_bootstrap_dwell_ticks
        )
        strict_dwell = int(self.cfg.ready_monitor_dwell_ticks)
        effective_dwell = int(
            math.ceil(
                relaxed_dwell
                + progress * (strict_dwell - relaxed_dwell)
                - 1.0e-12
            )
        )
        return {
            "progress": progress,
            "heading": _lerp(
                self.cfg.ready_acquisition_bootstrap_heading_thresh_rad,
                self.cfg.ready_monitor_heading_thresh_rad,
            ),
            "yaw_rate": _lerp(
                self.cfg.ready_acquisition_bootstrap_yaw_rate_thresh,
                self.cfg.ready_monitor_yaw_rate_thresh,
            ),
            "tilt": _lerp(
                self.cfg.ready_acquisition_bootstrap_tilt_thresh,
                self.cfg.ready_monitor_tilt_thresh,
            ),
            "joint_speed": _lerp(
                self.cfg.ready_acquisition_bootstrap_joint_speed_thresh,
                self.cfg.ready_monitor_joint_speed_thresh,
            ),
            "foot_slip": _lerp(
                self.cfg.ready_acquisition_bootstrap_foot_slip_thresh,
                self.cfg.ready_monitor_foot_slip_thresh,
            ),
            "dwell_ticks": effective_dwell,
        }

    def _reset_ready_acquisition_ema(self) -> None:
        """Start every acquisition rung with evidence collected under that rung's gate."""

        self._ready_release_n_ema = 0.0
        self._ready_release_pass_ema = 0.0
        for clip_id in self._clip_names:
            self._ready_release_n_ema_c[clip_id] = 0.0
            self._ready_release_pass_ema_c[clip_id] = 0.0
            self._safe_recovery_n_ema_c[clip_id] = 0.0
            self._safe_recovery_pass_ema_c[clip_id] = 0.0

    def _update_recovery_curriculum(self) -> None:
        """Advance RallyV17 from per-side, sample-weighted EMA strike statistics."""

        (
            actual_q_window_faults,
            actual_q_window_starts,
        ) = self._advance_actual_q_window()
        if len(self._clip_names) >= 2:
            fh_id, bh_id = tuple(self._clip_names)[:2]
            exact_fh = float(self._exact_n_acc_c[fh_id])
            exact_bh = float(self._exact_n_acc_c[bh_id])
            starts_fh = float(self._swing_starts_acc_c[fh_id])
            starts_bh = float(self._swing_starts_acc_c[bh_id])
            completion_fh = float(exact_fh / max(starts_fh, 1.0e-6))
            completion_bh = float(exact_bh / max(starts_bh, 1.0e-6))
            (
                release_eligible_completion_fh,
                release_eligible_starts_fh,
            ) = release_eligible_completion_rate(
                exact_fh,
                starts_fh,
                float(self._ready_release_timeout_acc_c[fh_id]),
            )
            (
                release_eligible_completion_bh,
                release_eligible_starts_bh,
            ) = release_eligible_completion_rate(
                exact_bh,
                starts_bh,
                float(self._ready_release_timeout_acc_c[bh_id]),
            )
            position_fh = float(
                self._exact_pass_pos_acc_c[fh_id] / max(exact_fh, 1.0e-6)
            )
            position_bh = float(
                self._exact_pass_pos_acc_c[bh_id] / max(exact_bh, 1.0e-6)
            )
            velocity_fh = float(
                self._exact_pass_vel_acc_c[fh_id] / max(exact_fh, 1.0e-6)
            )
            velocity_bh = float(
                self._exact_pass_vel_acc_c[bh_id] / max(exact_bh, 1.0e-6)
            )
            normal_fh = float(
                self._exact_pass_normal_acc_c[fh_id] / max(exact_fh, 1.0e-6)
            )
            normal_bh = float(
                self._exact_pass_normal_acc_c[bh_id] / max(exact_bh, 1.0e-6)
            )
            composite_fh = float(
                self._exact_pass_comp_acc_c[fh_id] / max(exact_fh, 1.0e-6)
            )
            composite_bh = float(
                self._exact_pass_comp_acc_c[bh_id] / max(exact_bh, 1.0e-6)
            )
            post_fall_fh = float(
                self._poststrike_fall_acc_c[fh_id] / max(starts_fh, 1.0e-6)
            )
            post_fall_bh = float(
                self._poststrike_fall_acc_c[bh_id] / max(starts_bh, 1.0e-6)
            )
            ready_fh = float(
                self._ready_release_pass_ema_c[fh_id]
                / max(self._ready_release_n_ema_c[fh_id], 1.0e-6)
            )
            ready_bh = float(
                self._ready_release_pass_ema_c[bh_id]
                / max(self._ready_release_n_ema_c[bh_id], 1.0e-6)
            )
            safe_recovery_fh = float(
                self._safe_recovery_pass_ema_c[fh_id]
                / max(self._safe_recovery_n_ema_c[fh_id], 1.0e-6)
            )
            safe_recovery_bh = float(
                self._safe_recovery_pass_ema_c[bh_id]
                / max(self._safe_recovery_n_ema_c[bh_id], 1.0e-6)
            )
            if bool(self.cfg.venue_tuple_unconditional_outcomes):
                virtual_samples_fh = float(
                    self._venue_swing_starts_acc_c[fh_id]
                )
                virtual_samples_bh = float(
                    self._venue_swing_starts_acc_c[bh_id]
                )
            else:
                virtual_samples_fh = float(self._vb_exact_acc_c[fh_id])
                virtual_samples_bh = float(self._vb_exact_acc_c[bh_id])
            virtual_contact_fh = float(
                self._vb_hit_acc_c[fh_id]
                / max(virtual_samples_fh, 1.0e-6)
            )
            virtual_contact_bh = float(
                self._vb_hit_acc_c[bh_id]
                / max(virtual_samples_bh, 1.0e-6)
            )
            virtual_over_net_fh = float(
                self._vb_net_acc_c[fh_id]
                / max(virtual_samples_fh, 1.0e-6)
            )
            virtual_over_net_bh = float(
                self._vb_net_acc_c[bh_id]
                / max(virtual_samples_bh, 1.0e-6)
            )
            virtual_legal_fh = float(
                self._vb_legal_acc_c[fh_id]
                / max(virtual_samples_fh, 1.0e-6)
            )
            virtual_legal_bh = float(
                self._vb_legal_acc_c[bh_id]
                / max(virtual_samples_bh, 1.0e-6)
            )
            actual_q_fault_events_fh = float(
                actual_q_window_faults[int(fh_id)]
            )
            actual_q_fault_events_bh = float(
                actual_q_window_faults[int(bh_id)]
            )
            actual_q_window_starts_fh = float(
                actual_q_window_starts[int(fh_id)]
            )
            actual_q_window_starts_bh = float(
                actual_q_window_starts[int(bh_id)]
            )
            actual_q_fault_fh = float(
                actual_q_fault_events_fh
                / max(actual_q_window_starts_fh, 1.0e-6)
            )
            actual_q_fault_bh = float(
                actual_q_fault_events_bh
                / max(actual_q_window_starts_bh, 1.0e-6)
            )
        else:
            exact_fh = exact_bh = float(self._exact_n_acc)
            denominator = max(float(self._exact_n_acc), 1.0e-6)
            starts_fh = starts_bh = float(self._swing_starts_acc)
            completion_fh = completion_bh = float(
                self._exact_n_acc / max(starts_fh, 1.0e-6)
            )
            ready_timeouts = float(
                sum(self._ready_release_timeout_acc_c.values())
            )
            (
                release_eligible_completion_fh,
                release_eligible_starts_fh,
            ) = release_eligible_completion_rate(
                float(self._exact_n_acc),
                starts_fh,
                ready_timeouts,
            )
            release_eligible_completion_bh = (
                release_eligible_completion_fh
            )
            release_eligible_starts_bh = release_eligible_starts_fh
            position_fh = position_bh = float(
                self._exact_pass_pos_acc / denominator
            )
            velocity_fh = velocity_bh = float(
                self._exact_pass_vel_acc / denominator
            )
            normal_fh = normal_bh = float(
                self._exact_pass_normal_acc / denominator
            )
            composite_fh = composite_bh = float(
                self._exact_pass_comp_acc / denominator
            )
            post_fall_fh = post_fall_bh = float(
                self._poststrike_fall_acc / max(starts_fh, 1.0e-6)
            )
            ready_fh = ready_bh = float(
                self._ready_release_pass_ema
                / max(self._ready_release_n_ema, 1.0e-6)
            )
            safe_recovery_fh = safe_recovery_bh = 0.0
            virtual_samples_fh = virtual_samples_bh = float(
                self._vb_exact_acc
            )
            virtual_contact_fh = virtual_contact_bh = float(
                self._vb_hit_acc / max(self._vb_exact_acc, 1.0e-6)
            )
            virtual_over_net_fh = virtual_over_net_bh = float(
                self._vb_net_acc / max(self._vb_exact_acc, 1.0e-6)
            )
            virtual_legal_fh = virtual_legal_bh = float(
                self._vb_inb_acc / max(self._vb_exact_acc, 1.0e-6)
            )
            actual_q_fault_fh = actual_q_fault_bh = 0.0
            actual_q_fault_events_fh = actual_q_fault_events_bh = 0.0
            actual_q_window_starts_fh = actual_q_window_starts_bh = 0.0
        starts = float(self._swing_starts_acc)
        completion = min(
            max(float(self._exact_n_acc) / max(starts, 1.0e-6), 0.0),
            1.0,
        )
        post_fall = min(
            max(float(self._poststrike_fall_acc) / max(starts, 1.0e-6), 0.0),
            1.0,
        )
        metrics = RecoveryCurriculumMetrics(
            environment_steps=int(
                getattr(self._env, "common_step_counter", 0)
            ),
            completion_fh=min(max(completion_fh, 0.0), 1.0),
            completion_bh=min(max(completion_bh, 0.0), 1.0),
            release_eligible_completion_fh=(
                release_eligible_completion_fh
            ),
            release_eligible_completion_bh=(
                release_eligible_completion_bh
            ),
            position_fh=min(max(position_fh, 0.0), 1.0),
            position_bh=min(max(position_bh, 0.0), 1.0),
            velocity_fh=min(max(velocity_fh, 0.0), 1.0),
            velocity_bh=min(max(velocity_bh, 0.0), 1.0),
            normal_fh=min(max(normal_fh, 0.0), 1.0),
            normal_bh=min(max(normal_bh, 0.0), 1.0),
            composite_fh=min(max(composite_fh, 0.0), 1.0),
            composite_bh=min(max(composite_bh, 0.0), 1.0),
            ready_fh=min(max(ready_fh, 0.0), 1.0),
            ready_bh=min(max(ready_bh, 0.0), 1.0),
            safe_recovery_fh=min(max(safe_recovery_fh, 0.0), 1.0),
            safe_recovery_bh=min(max(safe_recovery_bh, 0.0), 1.0),
            virtual_contact_fh=min(max(virtual_contact_fh, 0.0), 1.0),
            virtual_contact_bh=min(max(virtual_contact_bh, 0.0), 1.0),
            virtual_over_net_fh=min(max(virtual_over_net_fh, 0.0), 1.0),
            virtual_over_net_bh=min(max(virtual_over_net_bh, 0.0), 1.0),
            virtual_legal_fh=min(max(virtual_legal_fh, 0.0), 1.0),
            virtual_legal_bh=min(max(virtual_legal_bh, 0.0), 1.0),
            post_fall_fh=min(max(post_fall_fh, 0.0), 1.0),
            post_fall_bh=min(max(post_fall_bh, 0.0), 1.0),
            actual_q_fault_fh=min(max(actual_q_fault_fh, 0.0), 1.0),
            actual_q_fault_bh=min(max(actual_q_fault_bh, 0.0), 1.0),
            exact_samples_fh=exact_fh,
            exact_samples_bh=exact_bh,
            swing_starts_fh=starts_fh,
            swing_starts_bh=starts_bh,
            virtual_samples_fh=virtual_samples_fh,
            virtual_samples_bh=virtual_samples_bh,
            actual_q_fault_events_fh=actual_q_fault_events_fh,
            actual_q_fault_events_bh=actual_q_fault_events_bh,
            actual_q_window_steps=int(
                self._actual_q_window_filled_steps
            ),
            actual_q_window_starts_fh=actual_q_window_starts_fh,
            actual_q_window_starts_bh=actual_q_window_starts_bh,
        )
        previous_acquisition_rung = int(
            self._recovery_stage1_acquisition_rung
        )
        previous_acquisition_failures = int(
            self._recovery_stage1_acquisition_failures
        )
        state = RecoveryCurriculumState(
            stage=int(self._recovery_stage),
            current_scale=float(self._recovery_current_scale),
            target_scale=float(self._recovery_target_scale),
            ramp_rate_per_step=float(self._recovery_ramp_rate_per_step),
            coverage_scale=float(self._recovery_coverage_scale),
            coverage_target_scale=float(
                self._recovery_coverage_target_scale
            ),
            coverage_ramp_rate_per_step=float(
                self._recovery_coverage_ramp_rate_per_step
            ),
            stage1_coverage_unlocked=bool(
                self._recovery_stage1_coverage_unlocked
            ),
            stage1_acquisition_rung=previous_acquisition_rung,
            stage1_acquisition_failures=(
                previous_acquisition_failures
            ),
            stage1_ready_dwell=int(self._recovery_stage1_ready_dwell),
            stage1_acquisition_age=int(
                self._recovery_stage1_acquisition_age
            ),
            stage1_enter_dwell=int(self._recovery_stage1_enter_dwell),
            stage1_exit_dwell=int(self._recovery_stage1_exit_dwell),
            stage2_enter_dwell=int(self._recovery_stage2_enter_dwell),
            stage2_exit_dwell=int(self._recovery_stage2_exit_dwell),
        )
        state, conditions = advance_recovery_curriculum(
            state,
            metrics,
            self._recovery_curriculum_config,
            enabled=self._recovery_curriculum_enabled,
        )
        if self._eval_force_final_recovery_stage:
            # Keep computing the live gate conditions for telemetry, but do not let the fresh
            # evaluator's initially empty EMA counters ramp a converged checkpoint back down.
            state = RecoveryCurriculumState(
                stage=2,
                current_scale=1.0,
                target_scale=1.0,
                coverage_scale=1.0,
                coverage_target_scale=1.0,
                stage1_coverage_unlocked=True,
                stage1_acquisition_rung=(
                    len(
                        self._recovery_curriculum_config.acquisition_scales()
                    )
                    - 1
                ),
            )
        self._recovery_stage = int(state.stage)
        self._recovery_current_scale = float(state.current_scale)
        self._recovery_target_scale = float(state.target_scale)
        self._recovery_ramp_rate_per_step = float(state.ramp_rate_per_step)
        self._recovery_coverage_scale = float(state.coverage_scale)
        self._recovery_coverage_target_scale = float(
            state.coverage_target_scale
        )
        self._recovery_coverage_ramp_rate_per_step = float(
            state.coverage_ramp_rate_per_step
        )
        self._recovery_stage1_coverage_unlocked = bool(
            state.stage1_coverage_unlocked
        )
        self._recovery_stage1_acquisition_rung = int(
            state.stage1_acquisition_rung
        )
        self._recovery_stage1_acquisition_failures = int(
            state.stage1_acquisition_failures
        )
        self._recovery_stage1_ready_dwell = int(state.stage1_ready_dwell)
        self._recovery_stage1_acquisition_age = int(
            state.stage1_acquisition_age
        )
        self._recovery_stage1_enter_dwell = int(state.stage1_enter_dwell)
        self._recovery_stage1_exit_dwell = int(state.stage1_exit_dwell)
        self._recovery_stage2_enter_dwell = int(state.stage2_enter_dwell)
        self._recovery_stage2_exit_dwell = int(state.stage2_exit_dwell)
        motion = self._motion()
        motion.set_recovery_curriculum_scale(self._recovery_coverage_scale)

        acquisition_backoff = bool(
            self._recovery_stage1_acquisition_failures
            > previous_acquisition_failures
        )
        # R9 keeps one strict physical READY definition and preserves its EMA across monotonic
        # exposure changes. Clearing the evidence at every rung was part of R8's oscillation.
        acquisition_scales = (
            self._recovery_curriculum_config.acquisition_scales()
        )
        acquisition_ready_thresholds = (
            self._recovery_curriculum_config.acquisition_ready_thresholds()
        )
        acquisition_rung = self._recovery_stage1_acquisition_rung
        ready_profile = self._ready_acquisition_profile()
        scalar_metrics = {
            "recovery_curriculum_stage": self._recovery_stage,
            "recovery_curriculum_scale": self._recovery_current_scale,
            "recovery_curriculum_target_scale": self._recovery_target_scale,
            "recovery_coverage_scale": self._recovery_coverage_scale,
            "recovery_coverage_target_scale": (
                self._recovery_coverage_target_scale
            ),
            "recovery_stage1_coverage_unlocked": float(
                self._recovery_stage1_coverage_unlocked
            ),
            "recovery_stage1_acquisition_rung": acquisition_rung,
            "recovery_stage1_acquisition_rung_count": len(
                acquisition_scales
            ),
            "recovery_stage1_acquisition_ready_threshold": (
                acquisition_ready_thresholds[acquisition_rung]
            ),
            "recovery_stage1_acquisition_failures": (
                self._recovery_stage1_acquisition_failures
            ),
            "recovery_stage1_acquisition_backoff_event": float(
                acquisition_backoff
            ),
            "recovery_ready_profile_progress": ready_profile["progress"],
            "recovery_ready_effective_heading_thresh_rad": (
                ready_profile["heading"]
            ),
            "recovery_ready_effective_yaw_rate_thresh": (
                ready_profile["yaw_rate"]
            ),
            "recovery_ready_effective_tilt_thresh": (
                ready_profile["tilt"]
            ),
            "recovery_ready_effective_joint_speed_thresh": (
                ready_profile["joint_speed"]
            ),
            "recovery_ready_effective_foot_slip_thresh": (
                ready_profile["foot_slip"]
            ),
            "recovery_ready_effective_dwell_ticks": (
                ready_profile["dwell_ticks"]
            ),
            "recovery_stage1_ready_dwell": self._recovery_stage1_ready_dwell,
            "recovery_stage1_acquisition_age": (
                self._recovery_stage1_acquisition_age
            ),
            # Backward-compatible dashboard key: it is now a one-tick backoff event, not a
            # permanently asserted deadlock flag.
            "recovery_stage1_acquisition_failed": float(
                acquisition_backoff
            ),
            "recovery_curriculum_block_reason": (
                conditions.block_reason
                if self._recovery_stage == 0
                else (
                    conditions.acquisition_block_reason
                    if (
                        self._recovery_stage == 1
                        and not self._recovery_stage1_coverage_unlocked
                    )
                    else conditions.stage2_block_reason
                )
            ),
            "recovery_bootstrap_block_reason": conditions.block_reason,
            "recovery_acquisition_block_reason": (
                conditions.acquisition_block_reason
            ),
            "recovery_stage2_block_reason": conditions.stage2_block_reason,
            "recovery_stage1_enter_ok": float(conditions.stage1_enter_ok),
            "recovery_stage1_ready_ok": float(conditions.stage1_ready_ok),
            "recovery_stage1_safety_exit_bad": float(
                conditions.stage1_safety_exit_bad
            ),
            "recovery_stage1_exit_bad": float(conditions.stage1_exit_bad),
            "recovery_stage2_enter_ok": float(conditions.stage2_enter_ok),
            "recovery_stage2_exit_bad": float(conditions.stage2_exit_bad),
            "recovery_stage1_enter_dwell": self._recovery_stage1_enter_dwell,
            "recovery_stage1_exit_dwell": self._recovery_stage1_exit_dwell,
            "recovery_stage2_enter_dwell": self._recovery_stage2_enter_dwell,
            "recovery_stage2_exit_dwell": self._recovery_stage2_exit_dwell,
            "recovery_gate_completion": completion,
            "recovery_gate_completion_fh": completion_fh,
            "recovery_gate_completion_bh": completion_bh,
            "recovery_gate_release_eligible_completion_fh": (
                release_eligible_completion_fh
            ),
            "recovery_gate_release_eligible_completion_bh": (
                release_eligible_completion_bh
            ),
            "recovery_gate_ready_timeout_rate_fh": (
                max(starts_fh - release_eligible_starts_fh, 0.0)
                / max(starts_fh, 1.0e-6)
            ),
            "recovery_gate_ready_timeout_rate_bh": (
                max(starts_bh - release_eligible_starts_bh, 0.0)
                / max(starts_bh, 1.0e-6)
            ),
            "recovery_gate_position_fh": position_fh,
            "recovery_gate_position_bh": position_bh,
            "recovery_gate_velocity_fh": velocity_fh,
            "recovery_gate_velocity_bh": velocity_bh,
            "recovery_gate_normal_fh": normal_fh,
            "recovery_gate_normal_bh": normal_bh,
            "recovery_gate_composite_fh": composite_fh,
            "recovery_gate_composite_bh": composite_bh,
            "recovery_gate_ready_fh": ready_fh,
            "recovery_gate_ready_bh": ready_bh,
            "recovery_gate_safe_recovery_fh": safe_recovery_fh,
            "recovery_gate_safe_recovery_bh": safe_recovery_bh,
            "recovery_gate_virtual_contact_fh": virtual_contact_fh,
            "recovery_gate_virtual_contact_bh": virtual_contact_bh,
            "recovery_gate_virtual_over_net_fh": virtual_over_net_fh,
            "recovery_gate_virtual_over_net_bh": virtual_over_net_bh,
            "recovery_gate_virtual_legal_fh": virtual_legal_fh,
            "recovery_gate_virtual_legal_bh": virtual_legal_bh,
            "recovery_gate_actual_q_fault_fh": actual_q_fault_fh,
            "recovery_gate_actual_q_fault_bh": actual_q_fault_bh,
            "recovery_gate_actual_q_fault_events_fh": (
                actual_q_fault_events_fh
            ),
            "recovery_gate_actual_q_fault_events_bh": (
                actual_q_fault_events_bh
            ),
            "recovery_gate_actual_q_window_steps": (
                self._actual_q_window_filled_steps
            ),
            "recovery_gate_actual_q_window_starts_fh": (
                actual_q_window_starts_fh
            ),
            "recovery_gate_actual_q_window_starts_bh": (
                actual_q_window_starts_bh
            ),
            "recovery_gate_actual_q_window_ready": float(
                conditions.actual_q_window_ready
            ),
            "recovery_gate_virtual_samples_fh": virtual_samples_fh,
            "recovery_gate_virtual_samples_bh": virtual_samples_bh,
            "recovery_gate_post_fall": post_fall,
            "recovery_gate_post_fall_fh": post_fall_fh,
            "recovery_gate_post_fall_bh": post_fall_bh,
        }
        for name, value in scalar_metrics.items():
            self.metrics[name][:] = float(value)

    def _update_staged_velocity_curriculum(self) -> None:
        """Apply the RallyV15 14→18→24 metric-hysteresis state machine."""
        try:
            velocity_term = self._env.reward_manager.get_term_cfg(
                "racket_velocity"
            )
        except ValueError:
            return
        if bool(getattr(self, "_eval_force_final_velocity_stage", False)):
            final_weight = float(self._velocity_stage_config.stage2_weight)
            self._velocity_stage = 2
            self._velocity_current_weight = final_weight
            self._velocity_target_weight = final_weight
            velocity_term.weight = final_weight
            self.metrics["velocity_stage"][:] = 2.0
            self.metrics["velocity_target_weight"][:] = final_weight
            self.metrics["velocity_weight_current"][:] = final_weight
            return

        clip_ids = tuple(self._clip_names)
        if len(clip_ids) >= 2:
            fh_id, bh_id = clip_ids[:2]
            exact_samples_fh = float(self._exact_n_acc_c[fh_id])
            exact_samples_bh = float(self._exact_n_acc_c[bh_id])
            position_fh = float(
                self._exact_pass_pos_acc_c[fh_id]
                / max(exact_samples_fh, 1.0e-6)
            )
            position_bh = float(
                self._exact_pass_pos_acc_c[bh_id]
                / max(exact_samples_bh, 1.0e-6)
            )
            velocity_fh = float(
                self._exact_pass_vel_acc_c[fh_id]
                / max(exact_samples_fh, 1.0e-6)
            )
            velocity_bh = float(
                self._exact_pass_vel_acc_c[bh_id]
                / max(exact_samples_bh, 1.0e-6)
            )
            normal_fh = float(
                self._exact_pass_normal_acc_c[fh_id]
                / max(exact_samples_fh, 1.0e-6)
            )
            normal_bh = float(
                self._exact_pass_normal_acc_c[bh_id]
                / max(exact_samples_bh, 1.0e-6)
            )
        else:
            exact_samples_fh = exact_samples_bh = float(self._exact_n_acc)
            denominator = max(float(self._exact_n_acc), 1.0e-6)
            position_fh = position_bh = float(
                self._exact_pass_pos_acc / denominator
            )
            velocity_fh = velocity_bh = float(
                self._exact_pass_vel_acc / denominator
            )
            normal_fh = normal_bh = float(
                self._exact_pass_normal_acc / denominator
            )

        swing_start_samples = float(self._swing_starts_acc)
        post_fall = float(
            min(
                max(
                    self._poststrike_fall_acc
                    / max(swing_start_samples, 1.0e-6),
                    0.0,
                ),
                1.0,
            )
        )
        ready = float(
            min(
                max(
                    self._ready_release_pass_ema
                    / max(self._ready_release_n_ema, 1.0e-6),
                    0.0,
                ),
                1.0,
            )
        )
        stage_metrics = VelocityStageMetrics(
            position_fh=min(max(position_fh, 0.0), 1.0),
            position_bh=min(max(position_bh, 0.0), 1.0),
            velocity_fh=min(max(velocity_fh, 0.0), 1.0),
            velocity_bh=min(max(velocity_bh, 0.0), 1.0),
            normal_fh=min(max(normal_fh, 0.0), 1.0),
            normal_bh=min(max(normal_bh, 0.0), 1.0),
            post_fall=post_fall,
            ready=ready,
            exact_samples_fh=exact_samples_fh,
            exact_samples_bh=exact_samples_bh,
            swing_start_samples=swing_start_samples,
        )
        stage_state = VelocityStageState(
            stage=int(self._velocity_stage),
            current_weight=float(self._velocity_current_weight),
            target_weight=float(self._velocity_target_weight),
            stage1_enter_dwell=int(self._velocity_stage1_enter_dwell),
            stage1_exit_dwell=int(self._velocity_stage1_exit_dwell),
            stage2_enter_dwell=int(self._velocity_stage2_enter_dwell),
            stage2_exit_dwell=int(self._velocity_stage2_exit_dwell),
        )
        stage_state, conditions = advance_staged_velocity_curriculum(
            stage_state, stage_metrics, self._velocity_stage_config
        )

        self._velocity_stage = int(stage_state.stage)
        self._velocity_current_weight = float(stage_state.current_weight)
        self._velocity_target_weight = float(stage_state.target_weight)
        self._velocity_stage1_enter_dwell = int(
            stage_state.stage1_enter_dwell
        )
        self._velocity_stage1_exit_dwell = int(
            stage_state.stage1_exit_dwell
        )
        self._velocity_stage2_enter_dwell = int(
            stage_state.stage2_enter_dwell
        )
        self._velocity_stage2_exit_dwell = int(
            stage_state.stage2_exit_dwell
        )
        velocity_term.weight = self._velocity_current_weight

        # Legacy aliases stay available for dashboards and old runner tooling. They no longer
        # drive the staged state machine.
        config = self._velocity_stage_config
        self._vel_weight_full = float(config.stage2_weight)
        self._vel_weight_latched = bool(
            self._velocity_stage == 2
            and abs(self._velocity_current_weight - config.stage2_weight)
            <= 1.0e-9
        )
        self._vel_weight_progress = min(
            max(
                (self._velocity_current_weight - config.stage0_weight)
                / (config.stage2_weight - config.stage0_weight),
                0.0,
            ),
            1.0,
        )
        if self._velocity_stage <= 0:
            active_enter_ok = conditions.stage1_enter_ok
            active_position_ok = bool(
                conditions.stage1_position_fh_ok
                and conditions.stage1_position_bh_ok
            )
            active_velocity_ok = bool(
                conditions.stage1_velocity_fh_ok
                and conditions.stage1_velocity_bh_ok
            )
            active_normal_ok = bool(
                conditions.stage1_normal_fh_ok
                and conditions.stage1_normal_bh_ok
            )
            active_post_fall_ok = conditions.stage1_post_fall_ok
            active_ready_ok = conditions.stage1_ready_ok
            active_dwell = self._velocity_stage1_enter_dwell
            active_dwell_limit = config.stage1_enter_dwell_steps
        else:
            active_enter_ok = conditions.stage2_enter_ok
            active_position_ok = bool(
                conditions.stage2_position_fh_ok
                and conditions.stage2_position_bh_ok
            )
            active_velocity_ok = bool(
                conditions.stage2_velocity_fh_ok
                and conditions.stage2_velocity_bh_ok
            )
            active_normal_ok = bool(
                conditions.stage2_normal_fh_ok
                and conditions.stage2_normal_bh_ok
            )
            active_post_fall_ok = conditions.stage2_post_fall_ok
            active_ready_ok = conditions.stage2_ready_ok
            active_dwell = self._velocity_stage2_enter_dwell
            active_dwell_limit = config.stage2_enter_dwell_steps
        self._vel_weight_gate_dwell = int(active_dwell)
        blocked_reasons = (
            conditions.stage1_blocked_reasons
            if self._velocity_stage <= 0
            else conditions.stage2_blocked_reasons
        )
        block_mask = conditions.blocked_reason_mask(self._velocity_stage)
        stage2_weight_ready = bool(
            abs(self._velocity_current_weight - config.stage1_weight)
            <= 1.0e-9
            or self._velocity_stage == 2
        )

        self.metrics["racket_velocity_weight_live"][:] = (
            self._velocity_current_weight
        )
        self.metrics["velocity_weight_current"][:] = (
            self._velocity_current_weight
        )
        self.metrics["velocity_gate_stable_steps"][:] = float(active_dwell)
        self.metrics["velocity_gate_block_reason"][:] = float(block_mask)
        self.metrics["velocity_gate_position_fh"][:] = position_fh
        self.metrics["velocity_gate_position_bh"][:] = position_bh
        self.metrics["velocity_gate_velocity_fh"][:] = velocity_fh
        self.metrics["velocity_gate_velocity_bh"][:] = velocity_bh
        self.metrics["velocity_gate_normal_fh"][:] = normal_fh
        self.metrics["velocity_gate_normal_bh"][:] = normal_bh
        self.metrics["velocity_gate_post_fall"][:] = post_fall
        self.metrics["velocity_gate_ready"][:] = ready
        self.metrics["racket_velocity_gate_normal_ok"][:] = float(
            active_normal_ok
        )
        self.metrics["racket_velocity_gate_position_ok"][:] = float(
            active_position_ok
        )
        self.metrics["racket_velocity_gate_velocity_ok"][:] = float(
            active_velocity_ok
        )
        self.metrics["racket_velocity_gate_post_fall_ok"][:] = float(
            active_post_fall_ok
        )
        self.metrics["racket_velocity_gate_ready_ok"][:] = float(
            active_ready_ok
        )
        self.metrics["racket_velocity_gate_all_ok"][:] = float(
            active_enter_ok
        )
        self.metrics["racket_velocity_gate_dwell_fraction"][:] = min(
            active_dwell / float(active_dwell_limit), 1.0
        )
        self.metrics["racket_velocity_gate_progress"][:] = (
            self._vel_weight_progress
        )

        self.metrics["velocity_stage"][:] = float(self._velocity_stage)
        self.metrics["velocity_target_weight"][:] = (
            self._velocity_target_weight
        )
        self.metrics["velocity_current_weight"][:] = (
            self._velocity_current_weight
        )
        self.metrics["stage1_enter_ok"][:] = float(
            conditions.stage1_enter_ok
        )
        self.metrics["stage1_exit_bad"][:] = float(
            conditions.stage1_exit_bad
        )
        self.metrics["stage1_enter_dwell"][:] = float(
            self._velocity_stage1_enter_dwell
        )
        self.metrics["stage1_exit_dwell"][:] = float(
            self._velocity_stage1_exit_dwell
        )
        self.metrics["stage2_enter_ok"][:] = float(
            conditions.stage2_enter_ok
        )
        self.metrics["stage2_exit_bad"][:] = float(
            conditions.stage2_exit_bad
        )
        self.metrics["stage2_enter_dwell"][:] = float(
            self._velocity_stage2_enter_dwell
        )
        self.metrics["stage2_exit_dwell"][:] = float(
            self._velocity_stage2_exit_dwell
        )
        self.metrics["stage2_weight_ready"][:] = float(stage2_weight_ready)
        self.metrics["velocity_exact_sample_count_fh"][:] = exact_samples_fh
        self.metrics["velocity_exact_sample_count_bh"][:] = exact_samples_bh
        self.metrics["velocity_fall_sample_count"][:] = swing_start_samples
        for stage in (1, 2):
            self.metrics[f"stage{stage}_sample_count_ok"][:] = float(
                conditions.sample_count_ok
            )
            for condition_name in (
                "position_fh_ok",
                "position_bh_ok",
                "velocity_fh_ok",
                "velocity_bh_ok",
                "normal_fh_ok",
                "normal_bh_ok",
                "post_fall_ok",
                "ready_ok",
            ):
                self.metrics[f"stage{stage}_{condition_name}"][:] = float(
                    getattr(conditions, f"stage{stage}_{condition_name}")
                )
        for reason in (
            "position_fh",
            "position_bh",
            "velocity_fh",
            "velocity_bh",
            "normal_fh",
            "normal_bh",
            "post_fall",
            "ready",
            "sample_count",
        ):
            self.metrics[f"velocity_blocked_reason_{reason}"][:] = float(
                reason in blocked_reasons
            )

    def _update_strike_local_reward_instrumentation(self) -> None:
        """Log strike-local reward timing without changing reward or observations."""
        position_window_s = float(
            getattr(self.cfg, "position_guidance_window_s", 0.0)
        )
        if position_window_s <= 0.0:
            return
        try:
            reward_manager = self._env.reward_manager
            position_term = reward_manager.get_term_cfg("racket_position")
            debt_term = reward_manager.get_term_cfg(
                "racket_exact_position_debt"
            )
        except ValueError:
            return

        temporal_scale = float(self.cfg.position_guidance_temporal_scale)
        position_std = float(position_term.params["std"])
        position_weight = float(position_term.weight)
        debt_margin = float(debt_term.params["margin"])
        debt_scale = float(debt_term.params["huber_scale"])
        debt_window_s = float(debt_term.params["window_s"])
        debt_weight = float(debt_term.weight)
        step_dt = float(self._env.step_dt)

        moving_target = (
            self.racket_target_pos_w
            - self.racket_target_vel_w
            * self.time_to_strike.unsqueeze(-1)
        )
        moving_error_sq = torch.sum(
            torch.square(self.racket_pos_w - moving_target), dim=-1
        )
        static_error = torch.linalg.norm(
            self.racket_pos_w - self.racket_target_pos_w, dim=-1
        )
        position_active = (
            self.time_to_strike.abs() <= position_window_s + 1.0e-6
        )
        debt_active = (
            self.time_to_strike.abs() <= debt_window_s + 1.0e-6
        )
        position_kernel = torch.exp(
            -moving_error_sq / position_std**2
        )
        position_contribution = (
            position_weight
            * temporal_scale
            * position_kernel
            * position_active.float()
            * step_dt
        )
        debt_scaled = torch.clamp(
            static_error - debt_margin, min=0.0
        ) / debt_scale
        debt_raw = torch.where(
            debt_scaled <= 1.0,
            0.5 * torch.square(debt_scaled),
            debt_scaled - 0.5,
        )
        debt_raw_gated = debt_raw * debt_active.float()
        debt_weighted = debt_weight * debt_raw_gated
        debt_contribution = debt_weighted * step_dt

        self.metrics["position_guidance_active"][:] = (
            position_active.float()
        )
        self.metrics["position_guidance_temporal_scale"][:] = (
            temporal_scale
        )
        self.metrics["exact_position_debt_active"][:] = debt_active.float()
        self.metrics["exact_static_position_error"][:] = torch.where(
            debt_active, static_error, torch.zeros_like(static_error)
        )
        self.metrics["exact_position_debt_raw"][:] = debt_raw_gated
        self.metrics["exact_position_debt_weighted"][:] = debt_weighted

        position_entry = (
            position_active & (~self._position_guidance_prev_active)
        )
        self._position_guidance_event_acc = torch.where(
            position_entry,
            torch.zeros_like(self._position_guidance_event_acc),
            self._position_guidance_event_acc,
        )
        self._position_guidance_event_acc += position_contribution
        position_exit = (
            (~position_active)
            & self._position_guidance_prev_active
            & (self.time_to_strike < -position_window_s)
        )
        self.metrics["position_guidance_event_return"][:] = torch.where(
            position_exit,
            self._position_guidance_event_acc,
            self.metrics["position_guidance_event_return"],
        )
        self._position_guidance_prev_active.copy_(position_active)

        debt_entry = debt_active & (~self._exact_position_debt_prev_active)
        self._exact_position_debt_event_acc = torch.where(
            debt_entry,
            torch.zeros_like(self._exact_position_debt_event_acc),
            self._exact_position_debt_event_acc,
        )
        self._exact_position_debt_event_acc += debt_contribution
        debt_exit = (
            (~debt_active)
            & self._exact_position_debt_prev_active
            & (self.time_to_strike < -debt_window_s)
        )
        self.metrics["exact_position_debt_event_return"][:] = torch.where(
            debt_exit,
            self._exact_position_debt_event_acc,
            self.metrics["exact_position_debt_event_return"],
        )
        self._exact_position_debt_prev_active.copy_(debt_active)

        context = self._strike_audit_context_id
        for exited, event_values, counts, sums in (
            (
                position_exit,
                self._position_guidance_event_acc,
                self._strike_local_position_event_count,
                self._strike_local_position_event_sum,
            ),
            (
                debt_exit,
                self._exact_position_debt_event_acc,
                self._strike_local_exact_debt_event_count,
                self._strike_local_exact_debt_event_sum,
            ),
        ):
            valid = exited & (context >= 0)
            indices = context[valid]
            counts += torch.bincount(
                indices, minlength=self._strike_audit_size
            ).to(counts.dtype)
            sums += torch.bincount(
                indices,
                weights=event_values[valid],
                minlength=self._strike_audit_size,
            ).to(sums.dtype)

        # Live marginal aliases complement the full cross-stratified Instrumentation matrix.
        flat_index = torch.arange(
            self._strike_audit_size, device=self.device
        )
        categories = {
            "forehand": (flat_index // 24) == 0,
            "backhand": (flat_index // 24) == 1,
            "core": ((flat_index // 12) % 2) == 0,
            "planner": ((flat_index // 12) % 2) == 1,
            "q1": ((flat_index // 3) % 4) == 0,
            "q2": ((flat_index // 3) % 4) == 1,
            "q3": ((flat_index // 3) % 4) == 2,
            "q4": ((flat_index // 3) % 4) == 3,
        }
        valid_context = context >= 0
        valid_indices = context[valid_context]
        context_count = torch.bincount(
            valid_indices, minlength=self._strike_audit_size
        ).to(static_error.dtype)
        position_active_count = torch.bincount(
            valid_indices,
            weights=position_active[valid_context].to(static_error.dtype),
            minlength=self._strike_audit_size,
        )
        debt_active_count = torch.bincount(
            valid_indices,
            weights=debt_active[valid_context].to(static_error.dtype),
            minlength=self._strike_audit_size,
        )
        exact_static_error_sum = torch.bincount(
            valid_indices,
            weights=(
                static_error[valid_context]
                * debt_active[valid_context].to(static_error.dtype)
            ),
            minlength=self._strike_audit_size,
        )
        exact_debt_raw_sum = torch.bincount(
            valid_indices,
            weights=debt_raw_gated[valid_context],
            minlength=self._strike_audit_size,
        )
        exact_debt_weighted_sum = torch.bincount(
            valid_indices,
            weights=debt_weighted[valid_context],
            minlength=self._strike_audit_size,
        )
        for suffix, selected in categories.items():
            category_count = context_count[selected].sum().clamp_min(1.0)
            category_debt_active_count = (
                debt_active_count[selected].sum().clamp_min(1.0)
            )
            self.metrics[f"position_guidance_active_{suffix}"][:] = (
                position_active_count[selected].sum() / category_count
            )
            self.metrics[
                f"position_guidance_temporal_scale_{suffix}"
            ][:] = temporal_scale
            self.metrics[f"exact_position_debt_active_{suffix}"][:] = (
                debt_active_count[selected].sum() / category_count
            )
            self.metrics[f"exact_static_position_error_{suffix}"][:] = (
                exact_static_error_sum[selected].sum()
                / category_debt_active_count
            )
            self.metrics[f"exact_position_debt_raw_{suffix}"][:] = (
                exact_debt_raw_sum[selected].sum()
                / category_debt_active_count
            )
            self.metrics[f"exact_position_debt_weighted_{suffix}"][:] = (
                exact_debt_weighted_sum[selected].sum()
                / category_debt_active_count
            )
            position_event_count = self._strike_local_position_event_count[
                selected
            ].sum()
            debt_event_count = self._strike_local_exact_debt_event_count[
                selected
            ].sum()
            self.metrics[
                f"position_guidance_event_return_{suffix}"
            ][:] = (
                self._strike_local_position_event_sum[selected].sum()
                / position_event_count.clamp_min(1.0)
            )
            self.metrics[
                f"exact_position_debt_event_return_{suffix}"
            ][:] = (
                self._strike_local_exact_debt_event_sum[selected].sum()
                / debt_event_count.clamp_min(1.0)
            )

    def _update_runtime_handoff_metrics(self, action_term) -> None:
        """Accumulate runner-entry smoothness only over static->policy handoff holds.

        MotionCommand owns the sampled handoff subset.  The action term owns the exact executed
        q_des history, so computing this audit here keeps it read-only and avoids changing the
        110-D actor observation or the V11 affine decoder.
        """

        motion = self._motion()
        active = getattr(motion, "runtime_handoff_active", None)
        if not torch.is_tensor(active):
            return
        active = active.bool()
        entered = active & (~self._runtime_handoff_prev_active)
        steady = active & self._runtime_handoff_prev_active

        qdes_delta = getattr(action_term, "_qdes_delta", None)
        qdes_second = getattr(action_term, "_qdes_second_difference", None)
        projection_distance = getattr(
            action_term, "_qdes_projection_distance", None
        )
        if not (
            torch.is_tensor(qdes_delta)
            and torch.is_tensor(qdes_second)
            and torch.is_tensor(projection_distance)
        ):
            raise RuntimeError(
                "V17 runtime handoff audit requires executed q_des history and "
                "raw/executed projection distance from the V11 action term"
            )

        executable = torch.ones(
            qdes_delta.shape[1], dtype=torch.bool, device=self.device
        )
        passive_columns = getattr(action_term, "_passive_action_cols", None)
        if torch.is_tensor(passive_columns) and passive_columns.numel() > 0:
            executable[passive_columns] = False
        executable_count = int(executable.sum().item())
        if executable_count <= 0:
            raise RuntimeError(
                "V17 runtime handoff audit resolved no executable action channels"
            )

        zero = torch.zeros((), device=self.device)
        current_entry_peak = zero
        if bool(entered.any()):
            entry_step = torch.abs(qdes_delta[entered][:, executable]).amax()
            current_entry_peak = entry_step
            self._runtime_handoff_entry_qdes_step_peak.copy_(
                torch.maximum(
                    self._runtime_handoff_entry_qdes_step_peak, entry_step
                )
            )
            self._runtime_handoff_entry_count.add_(entered.sum())

        current_step_rms = zero
        current_second_rms = zero
        current_tracking_rms = zero
        current_projection_rms = zero
        current_reversal_hz = zero
        if bool(steady.any()):
            step = qdes_delta[steady][:, executable]
            second = qdes_second[steady][:, executable]
            actual_q = action_term._select_action_joints(
                self.robot.data.joint_pos
            )[steady][:, executable]
            executed_qdes = getattr(action_term, "_qdes_executed")[
                steady
            ][:, executable]
            tracking_error = executed_qdes - actual_q
            self._runtime_handoff_qdes_step_sq_sum.add_(step.square().sum())
            self._runtime_handoff_qdes_second_sq_sum.add_(
                second.square().sum()
            )
            self._runtime_handoff_tracking_sq_sum.add_(
                tracking_error.square().sum()
            )
            self._runtime_handoff_projection_sq_sum.add_(
                projection_distance[steady].square().sum()
            )
            reversal_fraction = getattr(
                action_term, "_qdes_reversal_fraction", None
            )
            if not torch.is_tensor(reversal_fraction):
                raise RuntimeError(
                    "V17 runtime handoff audit requires q_des reversal telemetry"
                )
            steady_count = steady.sum()
            current_element_count = steady_count * executable_count
            current_step_rms = torch.sqrt(
                step.square().sum() / current_element_count
            )
            current_second_rms = torch.sqrt(
                second.square().sum() / current_element_count
            )
            current_tracking_rms = torch.sqrt(
                tracking_error.square().sum() / current_element_count
            )
            current_projection_rms = torch.sqrt(
                projection_distance[steady].square().mean()
            )
            current_reversal_hz = (
                reversal_fraction[steady].mean()
                / float(self._env.step_dt)
            )
            self._runtime_handoff_reversal_count.add_(
                reversal_fraction[steady].sum() * float(executable_count)
            )
            self._runtime_handoff_steady_sample_count.add_(steady_count)
            self._runtime_handoff_steady_element_count.add_(
                steady_count * executable_count
            )

        denominator = self._runtime_handoff_steady_element_count.clamp_min(
            1.0
        )
        sample_denominator = (
            self._runtime_handoff_steady_sample_count.clamp_min(1.0)
        )
        values = {
            "runtime_handoff_entry_count": self._runtime_handoff_entry_count,
            "runtime_handoff_steady_sample_count": (
                self._runtime_handoff_steady_sample_count
            ),
            "runtime_handoff_entry_qdes_step_peak_rad": (
                self._runtime_handoff_entry_qdes_step_peak
            ),
            "runtime_handoff_steady_qdes_step_rms_rad": torch.sqrt(
                self._runtime_handoff_qdes_step_sq_sum / denominator
            ),
            "runtime_handoff_qdes_second_difference_rms_rad": torch.sqrt(
                self._runtime_handoff_qdes_second_sq_sum / denominator
            ),
            "runtime_handoff_reversal_hz": (
                self._runtime_handoff_reversal_count
                / denominator
                / float(self._env.step_dt)
            ),
            "runtime_handoff_tracking_error_rms_rad": torch.sqrt(
                self._runtime_handoff_tracking_sq_sum / denominator
            ),
            "runtime_handoff_projection_distance_raw_rms": torch.sqrt(
                self._runtime_handoff_projection_sq_sum
                / sample_denominator
            ),
            # Per-control-step conditional estimates let W&B rolling windows describe the
            # current policy. The cumulative values above remain provenance counters.
            "runtime_handoff_current_entry_count": entered.sum(),
            "runtime_handoff_current_steady_sample_count": steady.sum(),
            "runtime_handoff_current_entry_qdes_step_peak_rad": (
                current_entry_peak
            ),
            "runtime_handoff_current_steady_qdes_step_rms_rad": (
                current_step_rms
            ),
            "runtime_handoff_current_qdes_second_difference_rms_rad": (
                current_second_rms
            ),
            "runtime_handoff_current_reversal_hz": current_reversal_hz,
            "runtime_handoff_current_tracking_error_rms_rad": (
                current_tracking_rms
            ),
            "runtime_handoff_current_projection_distance_raw_rms": (
                current_projection_rms
            ),
        }
        for name, value in values.items():
            self.metrics[name][:] = value
        self._runtime_handoff_prev_active.copy_(active)

    def _capture_post_strike_risk_edges(self, motion) -> None:
        """Capture legal warning/near states before a follow-through fault.

        Fixed-time captures alone are survivor-biased: an environment that crosses a safety
        boundary before the next 0.08/0.30/0.80 s timestamp disappears without contributing a
        recovery question. This edge path records the first legal warning and near-boundary state
        in each hot/late-pre-wrap phase. It never records an already-faulted state and never changes the
        reset distribution while the recovery curriculum scale is zero.
        """

        metric_names = (
            "post_strike_risk_capture",
            "post_strike_risk_capture_warning",
            "post_strike_risk_capture_near",
        )
        for name in metric_names:
            self.metrics[name].zero_()
        for phase_index in range(self._post_strike_replay_phase_count):
            self.metrics[
                f"post_strike_risk_capture_phase_{phase_index}"
            ].zero_()

        if not bool(
            getattr(motion.cfg, "post_swing_risk_edge_capture", False)
        ):
            return
        if not motion.post_swing_capture_enabled():
            return
        if (
            getattr(motion, "_post_swing_replay_contract", "")
            != "markov_side_phase_severity_v3"
        ):
            raise RuntimeError(
                "post_swing_risk_edge_capture requires "
                "markov_side_phase_severity_v3"
            )
        phase_count = self._post_strike_replay_phase_count
        if phase_count == 0 or int(
            motion.cfg.post_swing_capture_phase_bins
        ) != phase_count:
            raise RuntimeError(
                "risk-edge capture phase count must match the complete "
                "fixed-hot plus late pre-wrap replay contract"
            )
        if tuple(self._post_strike_risk_capture_mask.shape[1:]) != (
            phase_count,
            3,
        ):
            raise RuntimeError(
                "risk-edge capture mask no longer matches the replay bucket contract"
            )

        age_s = -self.time_to_strike
        candidate = (
            (
                age_s
                >= float(motion.cfg.post_swing_risk_capture_min_age_s)
            )
            & (
                age_s
                <= float(motion.cfg.post_swing_risk_capture_max_age_s)
            )
        )
        env_ids = torch.where(candidate)[0]
        if len(env_ids) == 0:
            return

        action_term = self._env.action_manager.get_term(
            str(motion.cfg.post_swing_action_name)
        )
        severity, legal = motion._replay_severity(
            env_ids, action_term
        )
        if self._post_strike_capture_midpoints_s:
            boundaries = torch.as_tensor(
                self._post_strike_capture_midpoints_s,
                dtype=age_s.dtype,
                device=self.device,
            )
            phase = torch.bucketize(age_s[env_ids], boundaries)
        else:
            phase = torch.zeros_like(env_ids)
        phase = phase.clamp(0, phase_count - 1)
        already_captured = self._post_strike_risk_capture_mask[
            env_ids, phase, severity.clamp(0, 2)
        ]
        capture = legal & (severity >= 1) & (~already_captured)
        if not bool(capture.any()):
            return

        selected = env_ids[capture]
        selected_phase = phase[capture]
        selected_severity = severity[capture]
        # Set before serializing: a restored snapshot represents the state after this edge has
        # been consumed, so it cannot insert itself repeatedly on replay.
        self._post_strike_risk_capture_mask[
            selected, selected_phase, selected_severity
        ] = True
        for phase_index in range(phase_count):
            phase_ids = selected[
                selected_phase == phase_index
            ]
            if len(phase_ids) > 0:
                motion._capture_post_swing_states(
                    phase_ids,
                    phase_bin=phase_index,
                    source_clip_ids=motion.clip_id[
                        phase_ids
                    ].clone(),
                )
            self.metrics[
                f"post_strike_risk_capture_phase_{phase_index}"
            ][phase_ids] = 1.0

        self.metrics["post_strike_risk_capture"][selected] = 1.0
        warning_ids = selected[selected_severity == 1]
        near_ids = selected[selected_severity == 2]
        self.metrics["post_strike_risk_capture_warning"][
            warning_ids
        ] = 1.0
        self.metrics["post_strike_risk_capture_near"][near_ids] = 1.0

    def _refresh_build_static_telemetry(self) -> None:
        """Republish startup-randomized plant state after CommandTerm metric resets."""

        pd_state = getattr(self.robot, "_hope_a3_pd_telemetry", None)
        if not isinstance(pd_state, dict):
            return
        values = {
            "pd_nominal_env": pd_state["nominal_mask"].float(),
            "pd_kp_multiplier_mean": pd_state["alpha"].mean(dim=-1),
            "pd_kd_message_multiplier_mean": pd_state["beta"].mean(dim=-1),
        }
        for name, value in values.items():
            self.metrics[name].copy_(value)

    def _update_qdes_phase_telemetry(self) -> None:
        """Report executed affine q_des dynamics without changing the controller."""

        try:
            action_term = self._env.action_manager.get_term("joint_pos")
        except (AttributeError, KeyError, ValueError):
            return
        delta = getattr(action_term, "executed_qdes_delta", None)
        second = getattr(
            action_term, "executed_qdes_second_difference", None
        )
        reversal = getattr(action_term, "_qdes_reversal_fraction", None)
        if not (
            torch.is_tensor(delta)
            and torch.is_tensor(second)
            and torch.is_tensor(reversal)
        ):
            return
        step_rms = torch.sqrt(torch.mean(delta.square(), dim=-1))
        second_rms = torch.sqrt(torch.mean(second.square(), dim=-1))
        reversal_hz = reversal / max(float(self._env.step_dt), 1.0e-9)
        motion = self._motion()
        phase_masks = {
            "hold": motion.in_hold,
            "strike": (
                (self.time_to_strike <= 0.12)
                & (self.time_to_strike >= -0.30)
                & (~motion.in_hold)
            ),
            "recovery": (self.time_to_strike < -0.30) & (~motion.in_hold),
        }
        channels = {
            "qdes_step_rms_rad": step_rms,
            "qdes_second_difference_rms_rad": second_rms,
            "qdes_reversal_hz": reversal_hz,
        }
        for phase_name, mask in phase_masks.items():
            for channel, values in channels.items():
                metric_name = f"{channel}_{phase_name}"
                if bool(mask.any()):
                    self._qdes_phase_telemetry_last[metric_name] = float(
                        values[mask].mean().item()
                    )
                self.metrics[metric_name][:] = (
                    self._qdes_phase_telemetry_last[metric_name]
                )

    def _update_build_ability_curriculum(self) -> None:
        """Latch only mocap corruption after the unchanged V14 strike task is learned.

        Post-swing capture/replay is deliberately independent and follows V14 from fresh
        startup. This gate has one admission condition, no exit condition, and no path that
        can lower an already-reached mocap-corruption scale.
        """

        if not self._ability_curriculum_enabled:
            self._base_mocap_robustness_scale = 1.0
            self.metrics["ability_unlocked"][:] = 1.0
            self.metrics["base_mocap_robustness_scale"][:] = 1.0
            return

        clip_ids = tuple(self._clip_names)[:2]
        if len(clip_ids) != 2:
            raise RuntimeError(
                "one_way_strike_gate_v1 requires forehand and backhand clips"
            )
        fh_id, bh_id = clip_ids
        exact_fh = float(self._exact_n_acc_c[fh_id])
        exact_bh = float(self._exact_n_acc_c[bh_id])
        starts_fh = float(self._swing_starts_acc_c[fh_id])
        starts_bh = float(self._swing_starts_acc_c[bh_id])
        completion_fh = min(exact_fh / max(starts_fh, 1.0e-6), 1.0)
        completion_bh = min(exact_bh / max(starts_bh, 1.0e-6), 1.0)
        position_fh = self._exact_pass_pos_acc_c[fh_id] / max(
            exact_fh, 1.0e-6
        )
        position_bh = self._exact_pass_pos_acc_c[bh_id] / max(
            exact_bh, 1.0e-6
        )
        composite = self._exact_pass_comp_acc / max(
            self._exact_n_acc, 1.0e-6
        )
        post_fall = self._poststrike_fall_acc / max(
            self._swing_starts_acc, 1.0e-6
        )
        enough_samples = (
            exact_fh >= float(self.cfg.ability_min_exact_samples_per_side)
            and exact_bh >= float(self.cfg.ability_min_exact_samples_per_side)
        )
        gate_condition = bool(
            enough_samples
            and completion_fh
            >= float(self.cfg.ability_min_completion_per_side)
            and completion_bh
            >= float(self.cfg.ability_min_completion_per_side)
            and position_fh
            >= float(self.cfg.ability_min_position_pass_per_side)
            and position_bh
            >= float(self.cfg.ability_min_position_pass_per_side)
            and composite >= float(self.cfg.ability_min_composite)
            and post_fall <= float(self.cfg.ability_max_post_fall)
        )

        if not self._ability_unlocked:
            self._ability_gate_dwell = (
                self._ability_gate_dwell + 1 if gate_condition else 0
            )
            if self._ability_gate_dwell >= int(
                self.cfg.ability_gate_dwell_steps
            ):
                self._ability_unlocked = True
                self._ability_unlock_step = int(
                    getattr(self._env, "common_step_counter", 0)
                )
                self._base_mocap_residual_needs_sample.fill_(True)

        if self._ability_unlocked:
            if self._ability_unlock_step < 0:
                self._ability_unlock_step = int(
                    getattr(self._env, "common_step_counter", 0)
                )
            elapsed = max(
                int(getattr(self._env, "common_step_counter", 0))
                - self._ability_unlock_step,
                0,
            )
            self._base_mocap_robustness_scale = max(
                self._base_mocap_robustness_scale,
                min(
                    float(elapsed)
                    / float(self.cfg.base_mocap_robustness_ramp_steps),
                    1.0,
                ),
            )
        else:
            self._base_mocap_robustness_scale = 0.0

        metric_values = {
            "ability_gate_condition": float(gate_condition),
            "ability_gate_enough_samples": float(enough_samples),
            "ability_gate_completion_fh": completion_fh,
            "ability_gate_completion_bh": completion_bh,
            "ability_gate_position_fh": position_fh,
            "ability_gate_position_bh": position_bh,
            "ability_gate_composite": composite,
            "ability_gate_post_fall": post_fall,
            "ability_gate_dwell": float(self._ability_gate_dwell),
            "ability_unlocked": float(self._ability_unlocked),
            "base_mocap_robustness_scale": self._base_mocap_robustness_scale,
        }
        for name, value in metric_values.items():
            self.metrics[name][:] = float(value)

    def _update_metrics(self):
        # CommandTerm.compute() runs _update_metrics() BEFORE _update_command(), so refresh the
        # actual racket FK AND the strike timing here (once per step) — metrics, rewards, and
        # observations then all read the same fresh, phase-aligned buffers (rewards/obs read them
        # after the full command_manager.compute()). _compute_strike_timing must run before any
        # exact_strike / strike_window gating below (see its docstring: the old stale read measured
        # the strike one control frame too late).
        self._compute_racket_state()
        self._compute_strike_timing()
        origins = self._env.scene.env_origins
        # commanded base repositioning = distance of the base target from spawn (0 if coupling disabled).
        self.metrics["base_target_offset_norm"] = torch.norm(self.base_target_pos_w - origins[:, :2], dim=-1)
        pos_err = torch.norm(self.racket_pos_w - self.racket_target_pos_w, dim=-1)
        vel_err = torch.norm(self.racket_lin_vel_w - self.racket_target_vel_w, dim=-1)
        cos_ang = torch.sum(self.racket_normal_w * self.racket_target_normal_w, dim=-1).clamp(-1.0, 1.0)
        normal_err_deg = torch.acos(cos_ang) * (180.0 / math.pi)
        base_err = torch.norm(self.base_pos_w[:, :2] - self.base_target_pos_w, dim=-1)
        base_pos_rel = self.base_pos_w[:, :2] - origins[:, :2]
        base_err_xy = self.base_pos_w[:, :2] - self.base_target_pos_w
        racket_pos_err_vec = self.racket_pos_w - self.racket_target_pos_w
        racket_vel_err_vec = self.racket_lin_vel_w - self.racket_target_vel_w

        # Episode-wide (instantaneous) errors.
        self.metrics["racket_pos_error"] = pos_err
        self.metrics["racket_vel_error"] = vel_err
        self.metrics["racket_normal_error_deg"] = normal_err_deg
        self.metrics["base_pos_error"] = base_err
        self.metrics["time_to_strike_s"] = self.time_to_strike
        self.metrics["pre_strike_flag"] = self.pre_strike.float()
        self.metrics["strike_window_hit_rate"] = self.strike_window.float()
        if self.cfg.target_mode == "reference_perturbed":
            self.metrics["ref_perturb_scale"] = torch.full_like(pos_err, self._perturb_scale())
        else:
            self.metrics["ref_perturb_scale"].zero_()
        # Per-axis ERROR components only (which direction is the miss?). The per-axis actual/target
        # state and the speed/normal-cos scalars were dropped as redundant wandb clutter.
        for axis_idx, axis in enumerate(("x", "y")):
            self.metrics[f"base_pos_{axis}"] = base_pos_rel[:, axis_idx]
            self.metrics[f"base_pos_error_{axis}"] = base_err_xy[:, axis_idx]
        for axis_idx, axis in enumerate(("x", "y", "z")):
            self.metrics[f"racket_pos_error_{axis}"] = racket_pos_err_vec[:, axis_idx]
            self.metrics[f"racket_vel_error_{axis}"] = racket_vel_err_vec[:, axis_idx]

        # Strike-window-gated: hold the value sampled during the most recent strike window. The gating
        # masks come from the previous _update_command (<=1-step / 20 ms lag at 50 Hz — negligible vs
        # the ±strike_window_s window). Between strikes the held value carries to the next reset.
        in_win = self.strike_window
        exact_strike = torch.abs(self.time_to_strike) <= (0.5 * self._env.step_dt + 1e-6)
        self.metrics["exact_strike_hit_rate"] = exact_strike.float()

        # Per-joint exact-strike posture telemetry for the YAML-selected racket arm. Safe/rate/
        # tracking/torque projection prevents illegal q_des, but it cannot by itself distinguish
        # a useful face adjustment from an abnormal yet in-range wrist twist. Log the physical
        # reference deviation plus actual/q_des distance to the configured safe rails so a fresh
        # run exposes that failure before checkpoint evaluation. This is read-only telemetry.
        try:
            action_term = self._env.action_manager.get_term("joint_pos")
        except (AttributeError, ValueError):
            action_term = None
        if action_term is not None:
            action_joint_ids = getattr(action_term, "_action_joint_ids", None)
            qdes = getattr(action_term, "_qdes_executed", None)
            safe_lo = getattr(action_term, "_safe_lo", None)
            safe_hi = getattr(action_term, "_safe_hi", None)
            debug_joints = getattr(action_term, "qdes_debug_joints", ())
            if (
                torch.is_tensor(action_joint_ids)
                and torch.is_tensor(qdes)
                and torch.is_tensor(safe_lo)
                and torch.is_tensor(safe_hi)
            ):
                resolved_debug_joints = getattr(
                    self, "_qdes_posture_debug_joints", None
                )
                if resolved_debug_joints is None:
                    resolved_debug_joints = tuple(
                        (
                            str(joint_name),
                            int(col),
                            int(action_joint_ids[int(col)].item()),
                        )
                        for joint_name, col in debug_joints
                    )
                    self._qdes_posture_debug_joints = resolved_debug_joints
                motion_joint_pos = self._motion().joint_pos
                for joint_name, col, joint_id in resolved_debug_joints:
                    actual = self.robot.data.joint_pos[:, joint_id]
                    reference = motion_joint_pos[:, joint_id]
                    span = (safe_hi[:, col] - safe_lo[:, col]).clamp_min(
                        1.0e-6
                    )
                    actual_margin = torch.minimum(
                        actual - safe_lo[:, col],
                        safe_hi[:, col] - actual,
                    ) / span
                    qdes_col = qdes[:, col]
                    qdes_margin = torch.minimum(
                        qdes_col - safe_lo[:, col],
                        safe_hi[:, col] - qdes_col,
                    ) / span
                    label = joint_name.removesuffix("_joint")
                    for key, value in (
                        (
                            f"joint_reference_error_exact_strike_{label}",
                            torch.abs(actual - reference),
                        ),
                        (
                            f"joint_safe_margin_fraction_exact_strike_{label}",
                            actual_margin.clamp(0.0, 0.5),
                        ),
                        (
                            f"qdes_safe_margin_fraction_exact_strike_{label}",
                            qdes_margin.clamp(0.0, 0.5),
                        ),
                    ):
                        previous = self.metrics.get(key)
                        if previous is None:
                            previous = torch.zeros_like(actual)
                        self.metrics[key] = torch.where(
                            exact_strike, value, previous
                        )

        if (
            float(
                getattr(
                    self._motion().cfg,
                    "runtime_handoff_start_prob",
                    0.0,
                )
            )
            > 0.0
        ):
            if action_term is None:
                raise RuntimeError(
                    "V17 runtime handoff audit requires the joint_pos action term"
                )
            self._update_runtime_handoff_metrics(action_term)

        # RallyFinal stability/safety telemetry.  Keep signed legacy metrics above for diagnosis,
        # but gate acceptance on these absolute/phase-specific quantities so +x/-x errors cannot
        # cancel and legal lateral station motion is not confused with forward drift.
        base_v_xy = self.robot.data.root_lin_vel_w[:, :2]
        base_speed_xy = torch.linalg.norm(base_v_xy, dim=-1)
        x_abs = torch.abs(base_err_xy[:, 0])
        y_abs = torch.abs(base_err_xy[:, 1])

        def _yaw_from_quat(q):
            fx = 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2)
            fy = 2.0 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
            return torch.atan2(fy, fx)

        base_yaw = _yaw_from_quat(self.base_quat_w)
        base_yaw_abs = torch.abs(base_yaw)
        base_yaw_abs_deg = torch.rad2deg(base_yaw_abs)
        base_yaw_rate_abs = torch.abs(self.robot.data.root_ang_vel_b[:, 2])
        projected_gravity = getattr(self.robot.data, "projected_gravity_b", None)
        if projected_gravity is None:
            # A3 exposes projected_gravity_b.  Keep diagnostics finite for any legacy asset that
            # does not; the V2Plus deadline term fails closed instead of using this fallback.
            base_tilt = torch.zeros_like(base_yaw_abs)
        else:
            base_tilt = torch.linalg.norm(projected_gravity[:, :2], dim=-1)
        joint_speed = torch.sqrt(torch.mean(self.robot.data.joint_vel.square(), dim=-1))

        # RallyFinalV2 runner-equivalent readiness monitor. MotionCommand.metrics["in_hold"] is the
        # pre-decrement truth for THIS control step; the live ``motion.in_hold`` property is already
        # false on the last frozen tick and would shorten every dwell by one sample. This monitor never
        # modifies the clock. It only answers whether the externally sampled release timer was long
        # enough for the policy to satisfy the deploy thresholds.
        motion = self._motion()
        held_metric = motion.metrics.get("in_hold") if hasattr(motion, "metrics") else None
        held = (held_metric > 0.5) if held_metric is not None else torch.zeros_like(self._ready_latched)
        self._ready_elapsed_steps = torch.where(
            held, self._ready_elapsed_steps + 1, self._ready_elapsed_steps
        )
        position_ok = (
            (x_abs <= float(self.cfg.ready_monitor_x_thresh))
            & (y_abs <= float(self.cfg.ready_monitor_y_thresh))
        )
        speed_ok = base_speed_xy <= float(self.cfg.ready_monitor_speed_thresh)
        ready_profile = self._ready_acquisition_profile()
        heading_ok = base_yaw_abs <= float(ready_profile["heading"])
        yaw_rate_ok = base_yaw_rate_abs <= float(ready_profile["yaw_rate"])
        tilt_ok = base_tilt <= float(ready_profile["tilt"])
        joint_speed_ok = joint_speed <= float(
            ready_profile["joint_speed"]
        )
        foot_slip = self.metrics["foot_slip_speed"]
        foot_slip_ok = (
            foot_slip <= float(ready_profile["foot_slip"])
        )
        ready_sample = (
            held
            & position_ok
            & speed_ok
            & heading_ok
            & yaw_rate_ok
            & tilt_ok
            & joint_speed_ok
            & foot_slip_ok
        )
        self._ready_dwell_steps = torch.where(
            ready_sample, self._ready_dwell_steps + 1, torch.zeros_like(self._ready_dwell_steps)
        )
        # C++ seeds its dwell timer on the first qualifying tick (elapsed=0 there), then requires
        # tick_since_start*dt >= 0.12. Match that exactly: at 50 Hz this is seven good samples, not six.
        configured_ticks = int(ready_profile["dwell_ticks"])
        dwell_required = (
            configured_ticks
            if configured_ticks > 0
            else max(
                1,
                int(
                    math.ceil(
                        float(self.cfg.ready_monitor_dwell_s)
                        / float(self._env.step_dt)
                    )
                )
                + 1,
            )
        )
        ready_now = ready_sample & (self._ready_dwell_steps >= dwell_required)
        newly_ready = ready_now & (~self._ready_ever_ready)
        latency_now = self._ready_elapsed_steps.float() * float(self._env.step_dt)
        self._ready_latency_s = torch.where(newly_ready, latency_now, self._ready_latency_s)
        newly_ready_transition = newly_ready & self._ready_transition_eligible
        if bool(newly_ready_transition.any()):
            self._ready_latency_sum_acc += float(latency_now[newly_ready_transition].sum())
            self._ready_latency_n_acc += float(newly_ready_transition.sum())
        ready_before_release = self._ready_latched.clone()
        self._ready_ever_ready |= ready_now
        self._ready_latched = ready_now

        released = self._ready_prev_held & (~held)
        released_transition = released & self._ready_transition_eligible
        ready_decay = float(self.cfg.exact_success_decay)
        self._ready_release_n_ema *= ready_decay
        self._ready_release_pass_ema *= ready_decay
        for clip_id in self._clip_names:
            self._ready_release_n_ema_c[clip_id] *= ready_decay
            self._ready_release_pass_ema_c[clip_id] *= ready_decay
            self._safe_recovery_n_ema_c[clip_id] *= ready_decay
            self._safe_recovery_pass_ema_c[clip_id] *= ready_decay
            self._actual_q_fault_acc_c[clip_id] *= ready_decay
        if bool(released_transition.any()):
            self._ready_release_n_acc += float(released_transition.sum())
            self._ready_release_pass_acc += float(ready_before_release[released_transition].sum())
            self._ready_release_n_ema += float(released_transition.sum())
            self._ready_release_pass_ema += float(
                ready_before_release[released_transition].sum()
            )
        if bool(released.any()):
            for clip_id in self._clip_names:
                selected_transition = (
                    released_transition & (motion.clip_id == clip_id)
                )
                if bool(selected_transition.any()):
                    self._ready_release_n_ema_c[clip_id] += float(
                        selected_transition.sum()
                    )
                    self._ready_release_pass_ema_c[clip_id] += float(
                        ready_before_release[selected_transition].sum()
                    )
            safe_raw = self.metrics.get("recovery_safe_set_raw_cost")
            if not torch.is_tensor(safe_raw):
                safe_raw = torch.zeros(
                    self.num_envs, device=self.device
                )
            pending_release = torch.where(
                released & self._safe_recovery_pending
            )[0]
            if len(pending_release) > 0:
                # Fixed-station V17 uses the next ball-clock release as the recovery outcome:
                # reaching it means the robot survived the previous hit. A physical/actual-q
                # termination reaches the true-reset path first and records failure there. Legacy
                # READY-gated recipes retain their stricter READY + zero-safe-set definition.
                recovery_success = (
                    ready_before_release[pending_release]
                    & (safe_raw[pending_release] <= 1.0e-6)
                    if bool(self.cfg.ready_release_enabled)
                    else torch.ones(
                        len(pending_release),
                        dtype=torch.bool,
                        device=self.device,
                    )
                )
                self._resolve_safe_recovery_events(
                    pending_release,
                    recovery_success,
                )
            self._ready_release_required[released] = False
        release_pass = released_transition & ready_before_release
        # Finalize only after the strict READY monitor has produced the release event for this
        # exact control tick.  The old _update_command call observed live ``motion.in_hold``
        # after MotionCommand decremented the counter, one tick before this monitor observed the
        # falling edge in its pre-decrement metric.  That ordering permanently booked READY=False
        # and made every downstream chain counter zero.
        released_one_step = (
            released
            & self._step_bout_complete
            & (~self._one_step_release_recorded)
        )
        self._finalize_one_step_commands(
            torch.where(released_one_step)[0],
            released_to_swing=True,
            ready_release_pass=release_pass,
        )
        # A previously qualified exact hit becomes an end-to-end success only when the next
        # transition reaches strict READY. Releasing the next swing first closes that pending
        # recovery as a failure; a later READY event must not retroactively repair it.
        safe_recovery = newly_ready_transition & self._chain_recovery_pending
        safe_recovery_ids = torch.where(safe_recovery)[0]
        self._chain_index_add(
            self._chain_safe_recovery_count,
            safe_recovery_ids,
            torch.ones(safe_recovery_ids.numel(), device=self.device),
            recovery_context=True,
        )
        self._chain_recovery_pending[safe_recovery_ids] = False
        missed_recovery = released_transition & self._chain_recovery_pending
        self._chain_recovery_pending[missed_recovery] = False
        self._ready_prev_held = held.clone()
        release_rate = self._ready_release_pass_acc / max(self._ready_release_n_acc, 1.0)
        release_rate_ema = self._ready_release_pass_ema / max(
            self._ready_release_n_ema, 1.0e-6
        )
        latency_mean = self._ready_latency_sum_acc / max(self._ready_latency_n_acc, 1.0)
        self.metrics["ready_station_x_error"] = torch.where(
            held, x_abs, self.metrics["ready_station_x_error"]
        )
        self.metrics["ready_station_y_error"] = torch.where(
            held, y_abs, self.metrics["ready_station_y_error"]
        )
        self.metrics["ready_station_base_speed"] = torch.where(
            held, base_speed_xy, self.metrics["ready_station_base_speed"]
        )
        self.metrics["ready_station_heading_error_deg"] = torch.where(
            held, base_yaw_abs_deg, self.metrics["ready_station_heading_error_deg"]
        )
        self.metrics["ready_station_yaw_rate_abs"] = torch.where(
            held, base_yaw_rate_abs, self.metrics["ready_station_yaw_rate_abs"]
        )
        self.metrics["ready_station_tilt"] = torch.where(
            held, base_tilt, self.metrics["ready_station_tilt"]
        )
        self.metrics["ready_station_joint_speed"] = torch.where(
            held, joint_speed, self.metrics["ready_station_joint_speed"]
        )
        self.metrics["ready_station_foot_slip"] = torch.where(
            held, foot_slip, self.metrics["ready_station_foot_slip"]
        )
        self.metrics["ready_station_hold_age_s"] = self._ready_elapsed_steps.float() * float(self._env.step_dt)
        self.metrics["ready_station_position_ok"] = (held & position_ok).float()
        self.metrics["ready_station_speed_ok"] = (held & speed_ok).float()
        self.metrics["ready_station_heading_ok"] = (held & heading_ok).float()
        self.metrics["ready_station_yaw_rate_ok"] = (held & yaw_rate_ok).float()
        self.metrics["ready_station_tilt_ok"] = (held & tilt_ok).float()
        self.metrics["ready_station_joint_speed_ok"] = (held & joint_speed_ok).float()
        self.metrics["ready_station_foot_slip_ok"] = (
            held & foot_slip_ok
        ).float()
        self.metrics["ready_station_latched"] = self._ready_latched.float()
        self.metrics["ready_station_transition_eligible"] = self._ready_transition_eligible.float()
        self.metrics["ready_station_newly_ready"] = newly_ready_transition.float()
        self.metrics["ready_station_latency_s"] = self._ready_latency_s
        self.metrics["ready_station_x_error_at_ready"] = torch.where(
            newly_ready_transition, x_abs, self.metrics["ready_station_x_error_at_ready"]
        )
        self.metrics["ready_station_y_error_at_ready"] = torch.where(
            newly_ready_transition, y_abs, self.metrics["ready_station_y_error_at_ready"]
        )
        self.metrics["ready_station_base_speed_at_ready"] = torch.where(
            newly_ready_transition, base_speed_xy, self.metrics["ready_station_base_speed_at_ready"]
        )
        self.metrics["ready_station_heading_error_at_ready_deg"] = torch.where(
            newly_ready_transition,
            base_yaw_abs_deg,
            self.metrics["ready_station_heading_error_at_ready_deg"],
        )
        self.metrics["ready_station_release_event"] = released_transition.float()
        self.metrics["ready_station_release_pass"] = release_pass.float()
        self.metrics["ready_station_release_rate"] = torch.full_like(x_abs, release_rate)
        self.metrics["ready_station_release_rate_ema"] = torch.full_like(
            x_abs, release_rate_ema
        )
        self.metrics["ready_station_latency_mean_s"] = torch.full_like(x_abs, latency_mean)
        self.metrics["ready_release_required"] = (
            self._ready_release_required.float()
        )
        for clip_id, clip_name in self._clip_names.items():
            ready_rate_side = (
                self._ready_release_pass_ema_c[clip_id]
                / max(self._ready_release_n_ema_c[clip_id], 1.0e-6)
            )
            safe_rate_side = (
                self._safe_recovery_pass_ema_c[clip_id]
                / max(self._safe_recovery_n_ema_c[clip_id], 1.0e-6)
            )
            side_index = int(clip_id)
            window_faults = float(
                self._actual_q_window_faults[:, side_index].sum()
                + self._actual_q_window_pending_faults[side_index]
            )
            window_starts = float(
                self._actual_q_window_starts[:, side_index].sum()
                + self._actual_q_window_pending_starts[side_index]
            )
            fault_rate_side = window_faults / max(window_starts, 1.0e-6)
            self.metrics[f"ready_release_rate_{clip_name}"][:] = min(
                max(ready_rate_side, 0.0), 1.0
            )
            self.metrics[f"safe_recovery_rate_{clip_name}"][:] = min(
                max(safe_rate_side, 0.0), 1.0
            )
            self.metrics[f"actual_q_fault_rate_{clip_name}"][:] = min(
                max(fault_rate_side, 0.0), 1.0
            )

        self.metrics["base_x_error_abs"] = x_abs
        self.metrics["station_y_error_abs"] = y_abs
        self.metrics["racket_reach_y_command"] = (
            self.racket_target_pos_w[:, 1] - self.base_target_pos_w[:, 1]
        )
        self.metrics["base_x_error_exact_strike"] = torch.where(
            exact_strike, x_abs, self.metrics["base_x_error_exact_strike"]
        )
        self.metrics["base_x_velocity_abs_exact_strike"] = torch.where(
            exact_strike, torch.abs(base_v_xy[:, 0]), self.metrics["base_x_velocity_abs_exact_strike"]
        )
        self.metrics["station_y_error_exact_strike"] = torch.where(
            exact_strike, y_abs, self.metrics["station_y_error_exact_strike"]
        )
        pre_settle_window = (
            (self.time_to_strike > float(self.cfg.strike_window_s))
            & (self.time_to_strike <= float(self.cfg.metrics_pre_settle_t_max))
        )
        post_settle_window = (
            (self.time_to_strike < -float(self.cfg.metrics_post_settle_t_lo))
            & (self.time_to_strike > -float(self.cfg.metrics_post_settle_t_hi))
        )
        self.metrics["pre_strike_base_speed"] = torch.where(
            pre_settle_window, base_speed_xy, self.metrics["pre_strike_base_speed"]
        )
        self.metrics["base_speed_exact_strike"] = torch.where(
            exact_strike, base_speed_xy, self.metrics["base_speed_exact_strike"]
        )
        self.metrics["post_swing_base_speed"] = torch.where(
            post_settle_window, base_speed_xy, self.metrics["post_swing_base_speed"]
        )

        # Backhand-only danger-window clearance (hand proxy + elbow->wrist forearm segment).
        hand_dist, forearm_dist = self.left_arm_clearance()
        backhand = self.swing_sign < 0.0
        clearance_gate = (
            backhand
            & (self.time_to_strike < float(self.cfg.metrics_clearance_t_pre))
            & (self.time_to_strike > -float(self.cfg.metrics_clearance_t_post))
        )
        self._bh_hand_min = torch.where(clearance_gate, torch.minimum(self._bh_hand_min, hand_dist), self._bh_hand_min)
        self._bh_forearm_min = torch.where(
            clearance_gate, torch.minimum(self._bh_forearm_min, forearm_dist), self._bh_forearm_min
        )
        self._bh_left_arm_min = torch.minimum(self._bh_hand_min, self._bh_forearm_min)
        self.metrics["backhand_left_hand_min_distance"] = self._bh_hand_min
        self.metrics["backhand_left_forearm_min_distance"] = self._bh_forearm_min
        self.metrics["backhand_left_arm_min_distance"] = self._bh_left_arm_min

        # Exact-strike base/torso heading and torso yaw-rate.  These are reward/eval-only simulation
        # state; the 110-D actor observation remains unchanged and deploy-honest.
        base_wz_abs = torch.abs(self.robot.data.root_ang_vel_w[:, 2])
        torso_idx = self._resolve_body_index("torso_Link")
        if torso_idx >= 0:
            torso_yaw = _yaw_from_quat(self.robot.data.body_quat_w[:, torso_idx])
            torso_wz = self.robot.data.body_ang_vel_w[:, torso_idx, 2]
        else:
            torso_yaw = torch.zeros_like(base_yaw)
            torso_wz = torch.zeros_like(base_yaw)
        self.metrics["base_heading_error_deg_exact_strike"] = torch.where(
            exact_strike, base_yaw_abs_deg, self.metrics["base_heading_error_deg_exact_strike"]
        )
        self.metrics["torso_heading_error_deg_exact_strike"] = torch.where(
            exact_strike, torch.rad2deg(torch.abs(torso_yaw)), self.metrics["torso_heading_error_deg_exact_strike"]
        )
        self.metrics["torso_yaw_rate_exact_strike"] = torch.where(
            exact_strike, torch.abs(torso_wz), self.metrics["torso_yaw_rate_exact_strike"]
        )
        ready_heading_window = (
            (self.time_to_strike > float(self.cfg.metrics_ready_heading_t_lo))
            & (self.time_to_strike < float(self.cfg.metrics_ready_heading_t_hi))
        )
        post_heading_window = (
            (self.time_to_strike < -float(self.cfg.metrics_post_heading_t_lo))
            & (self.time_to_strike > -float(self.cfg.metrics_post_heading_t_hi))
        )
        self.metrics["ready_base_heading_error_deg"] = torch.where(
            ready_heading_window, base_yaw_abs_deg, self.metrics["ready_base_heading_error_deg"]
        )
        self.metrics["ready_base_yaw_rate_abs"] = torch.where(
            ready_heading_window, base_wz_abs, self.metrics["ready_base_yaw_rate_abs"]
        )
        self.metrics["post_swing_base_heading_error_deg"] = torch.where(
            post_heading_window, base_yaw_abs_deg,
            self.metrics["post_swing_base_heading_error_deg"],
        )
        self.metrics["post_swing_base_yaw_rate_abs"] = torch.where(
            post_heading_window, base_wz_abs, self.metrics["post_swing_base_yaw_rate_abs"]
        )

        # --- DEBUG: swing-through sign verification (cfg.debug_reward_logging) -----------------------
        # err_minus = ||racket_pos - (target - vel*t_to_strike)||  (the CURRENT form used by the reward)
        # err_plus  = ||racket_pos - (target + vel*t_to_strike)||  (the FLIPPED form the user suspected)
        # Held over the strike window / exact strike so the reset-mean reports the in-window value. Expect
        # err_minus_win < err_plus_win (sign correct) and err_minus_exact ~= err_plus_exact (t~0 collapse).
        if self.cfg.debug_reward_logging:
            _ttf = self.time_to_strike.unsqueeze(-1)
            _tp_minus = self.racket_target_pos_w - self.racket_target_vel_w * _ttf
            _tp_plus = self.racket_target_pos_w + self.racket_target_vel_w * _ttf
            _err_minus = torch.norm(self.racket_pos_w - _tp_minus, dim=-1)
            _err_plus = torch.norm(self.racket_pos_w - _tp_plus, dim=-1)
            self.metrics["dbg_err_minus_win"] = torch.where(in_win, _err_minus, self.metrics["dbg_err_minus_win"])
            self.metrics["dbg_err_plus_win"] = torch.where(in_win, _err_plus, self.metrics["dbg_err_plus_win"])
            self.metrics["dbg_err_minus_exact"] = torch.where(
                exact_strike, _err_minus, self.metrics["dbg_err_minus_exact"]
            )
            self.metrics["dbg_err_plus_exact"] = torch.where(
                exact_strike, _err_plus, self.metrics["dbg_err_plus_exact"]
            )
        self.metrics["racket_pos_error_exact_strike"] = torch.where(
            exact_strike, pos_err, self.metrics["racket_pos_error_exact_strike"]
        )
        self.metrics["racket_vel_error_exact_strike"] = torch.where(
            exact_strike, vel_err, self.metrics["racket_vel_error_exact_strike"]
        )
        self.metrics["racket_normal_error_deg_exact_strike"] = torch.where(
            exact_strike, normal_err_deg, self.metrics["racket_normal_error_deg_exact_strike"]
        )
        # --- CONDITIONAL exact-strike success (the trustworthy, undiluted metric) -------------------
        # Old bug: strike_composite_success_exact was a per-env HELD value (last exact-strike result,
        # else init 0). CommandTerm.reset() logs mean(metric[env_ids]) over the RESETTING envs then zeros
        # them, so every env that reset without ever registering an exact-strike frame contributed 0 ->
        # the logged value was ~10x diluted vs the true conditional pass rate (raw probe ~0.32 logged
        # ~0.03), and the success-gated curriculum never advanced off ref_perturb_curriculum_start.
        # Fix: report the fraction of *exact-strike samples* that pass each threshold as a sample-weighted
        # EMA, broadcast to every env, so the reset-mean, the curriculum's .mean(), and the per-env value
        # all equal the conditional rate. pos/vel/normal are also logged separately.
        pass_pos = (pos_err < self.cfg.strike_success_pos_thresh) & exact_strike
        pass_vel = (vel_err < self.cfg.strike_success_vel_thresh) & exact_strike
        pass_normal = (normal_err_deg < self.cfg.strike_success_normal_thresh_deg) & exact_strike
        pass_comp = pass_pos & pass_vel & pass_normal
        # Every same-strike composite hit opens one task-level recovery question. If an older
        # question somehow remains unresolved, close it as a failure before replacing it; no
        # later READY event may retroactively repair two swings at once.
        replacement = torch.where(
            pass_comp & self._safe_recovery_pending
        )[0]
        if len(replacement) > 0:
            self._resolve_safe_recovery_events(
                replacement,
                torch.zeros(
                    len(replacement),
                    dtype=torch.bool,
                    device=self.device,
                ),
            )
        new_recovery = torch.where(pass_comp)[0]
        if len(new_recovery) > 0:
            self._safe_recovery_pending[new_recovery] = True
            self._safe_recovery_source_clip[new_recovery] = (
                self._motion().clip_id[new_recovery]
            )
        # One-shot end-to-end funnel accounting. A strike only advances the chain when the same
        # planned command both completed its finite bout and passed strict READY at release.
        chain_exact_event = (
            exact_strike
            & self._chain_released
            & (~self._chain_exact_recorded)
        )
        chain_exact_qualified = (
            chain_exact_event
            & self._chain_step_ok
            & self._chain_ready_ok
        )
        chain_hit = chain_exact_qualified & pass_comp
        chain_exact_ids = torch.where(chain_exact_qualified)[0]
        self._chain_index_add(
            self._chain_exact_frame_count,
            chain_exact_ids,
            torch.ones(chain_exact_ids.numel(), device=self.device),
        )
        chain_hit_ids = torch.where(chain_hit)[0]
        self._chain_index_add(
            self._chain_exact_hit_count,
            chain_hit_ids,
            torch.ones(chain_hit_ids.numel(), device=self.device),
        )
        if chain_hit_ids.numel() > 0:
            self._chain_recovery_pending[chain_hit_ids] = True
            self._chain_recovery_clip_id[chain_hit_ids] = self._chain_clip_id[
                chain_hit_ids
            ]
            self._chain_recovery_distance_bin[chain_hit_ids] = (
                self._station_distance_bin[chain_hit_ids]
            )
        self._chain_exact_recorded |= chain_exact_event
        self._chain_released &= ~chain_exact_event
        self._accumulate_strike_instrumentation(
            exact_strike=exact_strike,
            racket_pos_error=racket_pos_err_vec,
            base_error_xy=base_err_xy,
            pass_position=pass_pos,
            pass_velocity=pass_vel,
            pass_normal=pass_normal,
            pass_composite=pass_comp,
        )
        self._accumulate_qdes_joint_instrumentation(exact_strike)
        self._rally_success_run = torch.where(
            exact_strike,
            torch.where(pass_comp, self._rally_success_run + 1, torch.zeros_like(self._rally_success_run)),
            self._rally_success_run,
        )
        self._rally_success_run_max = torch.maximum(self._rally_success_run_max, self._rally_success_run)
        self.metrics["rally_success_run"] = self._rally_success_run.float()
        self.metrics["rally_success_run_max"] = self._rally_success_run_max.float()
        decay = float(self.cfg.exact_success_decay)
        self._exact_n_acc = decay * self._exact_n_acc + float(exact_strike.sum())
        self._exact_pass_comp_acc = decay * self._exact_pass_comp_acc + float(pass_comp.sum())
        self._exact_pass_pos_acc = decay * self._exact_pass_pos_acc + float(pass_pos.sum())
        self._exact_pass_vel_acc = decay * self._exact_pass_vel_acc + float(pass_vel.sum())
        # 5/10 cm position buckets on the exact-strike sample (NOT the window-exit frame).
        _pass_5cm = (pos_err < 0.05) & exact_strike
        _pass_10cm = (pos_err < 0.10) & exact_strike
        self._exact_pass_5cm_acc = decay * self._exact_pass_5cm_acc + float(_pass_5cm.sum())
        self._exact_pass_10cm_acc = decay * self._exact_pass_10cm_acc + float(_pass_10cm.sum())
        self._exact_pass_normal_acc = decay * self._exact_pass_normal_acc + float(pass_normal.sum())
        # Tier-1 virtual-ball at-strike evaluation (one-shot buffers consumed by the virtual_*
        # reward terms after this compute()); no-op (and vb_fired stays False) when disabled.
        if self.cfg.virtual_ball:
            if bool(self.cfg.venue_tuple_unconditional_outcomes):
                for clip_id in self._clip_names:
                    self._venue_swing_starts_acc_c[clip_id] *= decay
                    self._vb_exact_acc_c[clip_id] *= decay
                    self._vb_hit_acc_c[clip_id] *= decay
                    self._vb_net_acc_c[clip_id] *= decay
                    self._vb_legal_acc_c[clip_id] *= decay
                    self._vb_land_err_sum_c[clip_id] *= decay
                    self._vb_land_err_n_c[clip_id] *= decay
            self._vb_evaluate(exact_strike, pos_err)
        if self._physical is not None:
            self._physical.update(exact_strike)
        # GLOBAL error-magnitude EMAs (P2.3 adaptive sigma driver) — same decay/mask as the pass
        # counters above; per-clip variants exist further down but sigma needs one global signal.
        self._exact_pos_err_sum = decay * self._exact_pos_err_sum + float((pos_err * exact_strike).sum())
        self._exact_vel_err_sum = decay * self._exact_vel_err_sum + float((vel_err * exact_strike).sum())
        # racket_normal's gate is the whole strike window, so its sigma driver must be the
        # window-mean face error (radians), decayed at the same rate as the exact accumulators.
        _normal_err_rad = normal_err_deg * (math.pi / 180.0)
        self._win_n_acc = decay * self._win_n_acc + float(in_win.sum())
        self._win_normal_err_sum = (
            decay * self._win_normal_err_sum + float((_normal_err_rad * in_win).sum())
        )
        self.metrics["racket_normal_error_rad_window_mean"][:] = (
            self._win_normal_err_sum / max(self._win_n_acc, 1.0e-6)
        )
        # UNCONDITIONAL swing accounting: decay the start/fall accumulators at the SAME per-step
        # rate as the exact accumulators (increments happen in _count_swing_starts), then report
        #   swing_completion_rate = exact-strike arrivals / swing starts   (falls count against it)
        #   pre_strike_fall_rate  = pre-strike terminations / swing starts
        # These are the honest companions to the CONDITIONAL composite below, whose denominator
        # only contains exact-strike samples (pre-strike falls are invisible to it).
        self._swing_starts_acc = decay * self._swing_starts_acc
        self._prestrike_fall_acc = decay * self._prestrike_fall_acc
        self._poststrike_fall_acc = decay * self._poststrike_fall_acc
        self._resample_n_acc = decay * self._resample_n_acc
        self._replay_n_acc = decay * self._replay_n_acc
        # Rally drift accumulators share the same EMA timescale (increments: _count_swing_starts
        # wrap close-out + _resample_command wrap station-offset).
        self._drift_n_acc = decay * self._drift_n_acc
        self._drift_sum_acc = decay * self._drift_sum_acc
        self._drift_fwd_sum_acc = decay * self._drift_fwd_sum_acc
        self._station_offset_start_sum_acc = decay * self._station_offset_start_sum_acc
        self._heading_expiry_sum_acc = decay * self._heading_expiry_sum_acc
        self._heading_expiry_n_acc = decay * self._heading_expiry_n_acc
        self._recov_spawn_sum_acc = decay * self._recov_spawn_sum_acc
        self._recov_expiry_sum_acc = decay * self._recov_expiry_sum_acc
        self._recov_n_acc = decay * self._recov_n_acc
        for _c in self._clip_names:
            self._swing_starts_acc_c[_c] = decay * self._swing_starts_acc_c[_c]
            self._ready_release_timeout_acc_c[_c] = (
                decay * self._ready_release_timeout_acc_c[_c]
            )
            self._prestrike_fall_acc_c[_c] = decay * self._prestrike_fall_acc_c[_c]
            self._poststrike_fall_acc_c[_c] = decay * self._poststrike_fall_acc_c[_c]
        _s_denom = max(self._swing_starts_acc, 1e-6)
        _s_enough = self._swing_starts_acc >= float(self.cfg.exact_success_min_count)
        self.metrics["swing_completion_rate"][:] = min(self._exact_n_acc / _s_denom, 1.0) if _s_enough else 0.0
        self.metrics["pre_strike_fall_rate"][:] = min(self._prestrike_fall_acc / _s_denom, 1.0) if _s_enough else 0.0
        self.metrics["post_strike_fall_rate"][:] = (
            min(self._poststrike_fall_acc / _s_denom, 1.0) if _s_enough else 0.0
        )
        # Rally drift ratios: denominator = completed (wrapped) swings, NOT all starts.
        _d_denom = max(self._drift_n_acc, 1e-6)
        _d_enough = self._drift_n_acc >= float(self.cfg.exact_success_min_count)
        self.metrics["base_drift_per_swing"][:] = (self._drift_sum_acc / _d_denom) if _d_enough else 0.0
        self.metrics["base_drift_fwd_per_swing"][:] = (self._drift_fwd_sum_acc / _d_denom) if _d_enough else 0.0
        self.metrics["base_station_offset_at_swing_start"][:] = (
            (self._station_offset_start_sum_acc / _d_denom) if _d_enough else 0.0
        )
        # v2 rally heading debt (2026-07-08): mean |base yaw vs world +x| at HOLD EXPIRY — the
        # moment each swing arms AFTER its recovery hold ran (wrap AND stand-start holds; own
        # denominator, not _drift_n_acc, because stand-start expiries are not wraps). The deploy
        # engage gate refuses >0.35 rad (20°); a working rally2 reads well under that — this is
        # the "self-squares during the hold" eval gate.
        _h_denom = max(self._heading_expiry_n_acc, 1e-6)
        _h_enough = self._heading_expiry_n_acc >= float(self.cfg.exact_success_min_count)
        self.metrics["base_heading_abs_at_swing_start"][:] = (
            (self._heading_expiry_sum_acc / _h_denom) if _h_enough else 0.0
        )
        # v3 spawn-conditioned recovery pair (2026-07-08): mean spawn/expiry |yaw| over holds that
        # STARTED yawed (>_RECOV_SPAWN_YAW_THRESH). expiry is THE recovery gate — in a dedicated
        # recovery eval (stand_start_prob=1.0, yaw ±0.9) it is undiluted; reads 0.0 until enough
        # yawed holds have accumulated (own denominator, not the pooled one above).
        _r_denom = max(self._recov_n_acc, 1e-6)
        _r_enough = self._recov_n_acc >= float(self.cfg.exact_success_min_count)
        self.metrics["heading_recovery_spawn_yaw"][:] = (
            (self._recov_spawn_sum_acc / _r_denom) if _r_enough else 0.0
        )
        self.metrics["heading_recovery_expiry_yaw"][:] = (
            (self._recov_expiry_sum_acc / _r_denom) if _r_enough else 0.0
        )
        # HER replay diagnostics: fraction of resampled targets drawn from the achieved buffer
        # (~achieved_target_mix_prob once the per-clip buffers pass achieved_min_fill).
        self.metrics["achieved_replay_frac"][:] = (
            self._replay_n_acc / max(self._resample_n_acc, 1e-6)
            if self._resample_n_acc >= float(self.cfg.exact_success_min_count)
            else 0.0
        )
        for _c, _cn in self._clip_names.items():
            _cd = max(self._swing_starts_acc_c[_c], 1e-6)
            _ce = self._swing_starts_acc_c[_c] >= float(self.cfg.exact_success_min_count)
            self.metrics[f"swing_completion_rate_{_cn}"][:] = (
                min(self._exact_n_acc_c[_c] / _cd, 1.0) if _ce else 0.0
            )
            # Fall attribution uses _prev_clip_id (the clip during the fall) while starts use the NEW
            # clip; with uniform clip resampling the denominators match in expectation.
            self.metrics[f"pre_strike_fall_rate_{_cn}"][:] = (
                min(self._prestrike_fall_acc_c[_c] / _cd, 1.0) if _ce else 0.0
            )
            self.metrics[f"post_strike_fall_rate_{_cn}"][:] = (
                min(self._poststrike_fall_acc_c[_c] / _cd, 1.0) if _ce else 0.0
            )

        enough = self._exact_n_acc >= float(self.cfg.exact_success_min_count)
        denom = max(self._exact_n_acc, 1e-6)
        self._exact_composite_rate = (self._exact_pass_comp_acc / denom) if enough else 0.0
        # Broadcast in place so the entries reset() zeros are refreshed before the next reset logs them.
        self.metrics["strike_composite_success_exact"][:] = self._exact_composite_rate
        self.metrics["strike_pos_pass_exact"][:] = (self._exact_pass_pos_acc / denom) if enough else 0.0
        self.metrics["strike_vel_pass_exact"][:] = (self._exact_pass_vel_acc / denom) if enough else 0.0
        self.metrics["strike_normal_pass_exact"][:] = (self._exact_pass_normal_acc / denom) if enough else 0.0
        # Exact-strike position accuracy buckets (comparable with composite: same mask + EMA denominator).
        self.metrics["exact_strike_pos_success_5cm"][:] = (self._exact_pass_5cm_acc / denom) if enough else 0.0
        self.metrics["exact_strike_pos_success_10cm"][:] = (self._exact_pass_10cm_acc / denom) if enough else 0.0
        # Distribution of position error over THIS step's exact-strike samples (p90 + mean), broadcast.
        _ex_errs = pos_err[exact_strike]
        if _ex_errs.numel() > 0:
            self.metrics["exact_strike_pos_err_mean"][:] = _ex_errs.mean()
            self.metrics["exact_strike_pos_err_p90"][:] = torch.quantile(_ex_errs, 0.90)
        self.metrics["exact_strike_sample_count_decayed"][:] = self._exact_n_acc
        # --- per-clip (forehand/backhand) breakdown of the exact-strike pass rates + errors -----------
        # Same sample-weighted EMA as the global block above, selected by the motion command's clip_id so
        # wandb shows each swing separately. pass_pos/vel/normal already include `& exact_strike`. Multiseg
        # (unified forehand+backhand) only; single-clip leaves these at 0.
        _motion = self._motion()
        if getattr(_motion, "_multiseg", False):
            _clip = _motion.clip_id
            for _c, _cn in self._clip_names.items():
                _sel = exact_strike & (_clip == _c)
                _self_f = _sel.float()
                self._exact_n_acc_c[_c] = decay * self._exact_n_acc_c[_c] + float(_sel.sum())
                self._exact_pass_pos_acc_c[_c] = decay * self._exact_pass_pos_acc_c[_c] + float((pass_pos & _sel).sum())
                self._exact_pass_vel_acc_c[_c] = decay * self._exact_pass_vel_acc_c[_c] + float((pass_vel & _sel).sum())
                self._exact_pass_normal_acc_c[_c] = decay * self._exact_pass_normal_acc_c[_c] + float((pass_normal & _sel).sum())
                self._exact_pass_comp_acc_c[_c] = decay * self._exact_pass_comp_acc_c[_c] + float((pass_comp & _sel).sum())
                self._exact_pos_err_sum_c[_c] = decay * self._exact_pos_err_sum_c[_c] + float((pos_err * _self_f).sum())
                self._exact_vel_err_sum_c[_c] = decay * self._exact_vel_err_sum_c[_c] + float((vel_err * _self_f).sum())
                self._exact_nrm_err_sum_c[_c] = decay * self._exact_nrm_err_sum_c[_c] + float((normal_err_deg * _self_f).sum())
                _self_f3 = _self_f.unsqueeze(-1)
                self._exact_pos_signed_sum_c[_c] = (
                    decay * self._exact_pos_signed_sum_c[_c]
                    + (racket_pos_err_vec * _self_f3).sum(dim=0)
                )
                self._exact_vel_signed_sum_c[_c] = (
                    decay * self._exact_vel_signed_sum_c[_c]
                    + (racket_vel_err_vec * _self_f3).sum(dim=0)
                )
                self._exact_normal_dot_sum_c[_c] = (
                    decay * self._exact_normal_dot_sum_c[_c]
                    + (cos_ang * _self_f).sum()
                )
                _n = self._exact_n_acc_c[_c]
                # rate = acc / n once enough decayed samples accumulated (else 0). errors = decayed mean
                # error over THIS clip's exact-strike samples. _scale folds in the "enough" gate.
                _scale = (1.0 / max(_n, 1e-6)) if _n >= float(self.cfg.exact_success_min_count) else 0.0
                self.metrics[f"strike_pos_pass_exact_{_cn}"][:] = self._exact_pass_pos_acc_c[_c] * _scale
                self.metrics[f"strike_vel_pass_exact_{_cn}"][:] = self._exact_pass_vel_acc_c[_c] * _scale
                self.metrics[f"strike_normal_pass_exact_{_cn}"][:] = self._exact_pass_normal_acc_c[_c] * _scale
                self.metrics[f"strike_composite_success_exact_{_cn}"][:] = self._exact_pass_comp_acc_c[_c] * _scale
                self.metrics[f"racket_pos_error_exact_strike_{_cn}"][:] = self._exact_pos_err_sum_c[_c] * _scale
                self.metrics[f"racket_vel_error_exact_strike_{_cn}"][:] = self._exact_vel_err_sum_c[_c] * _scale
                self.metrics[f"racket_normal_error_deg_exact_strike_{_cn}"][:] = self._exact_nrm_err_sum_c[_c] * _scale
                for _ai, _axis in enumerate(("x", "y", "z")):
                    self.metrics[f"racket_pos_signed_error_{_axis}_exact_strike_{_cn}"][:] = (
                        self._exact_pos_signed_sum_c[_c][_ai] * _scale
                    )
                    self.metrics[f"racket_vel_signed_error_{_axis}_exact_strike_{_cn}"][:] = (
                        self._exact_vel_signed_sum_c[_c][_ai] * _scale
                    )
                self.metrics[f"racket_normal_dot_exact_strike_{_cn}"][:] = (
                    self._exact_normal_dot_sum_c[_c] * _scale
                )
                for _cohort in self._cohort_names:
                    if _cohort == "tuple":
                        _cohort_sel = _sel & self._venue_tuple_selected
                    else:
                        _cohort_sel = _sel & ~self._venue_tuple_selected
                    _cohort_key = (_cohort, _c)
                    self._cohort_exact_acc[_cohort_key] = (
                        decay * self._cohort_exact_acc[_cohort_key]
                        + float(_cohort_sel.sum())
                    )
                    self._cohort_pos_acc[_cohort_key] = (
                        decay * self._cohort_pos_acc[_cohort_key]
                        + float((pass_pos & _cohort_sel).sum())
                    )
                    self._cohort_vel_acc[_cohort_key] = (
                        decay * self._cohort_vel_acc[_cohort_key]
                        + float((pass_vel & _cohort_sel).sum())
                    )
                    self._cohort_comp_acc[_cohort_key] = (
                        decay * self._cohort_comp_acc[_cohort_key]
                        + float((pass_comp & _cohort_sel).sum())
                    )
                    _cohort_n = self._cohort_exact_acc[_cohort_key]
                    _cohort_scale = (
                        1.0 / max(_cohort_n, 1.0e-6)
                        if _cohort_n
                        >= float(self.cfg.exact_success_min_count)
                        else 0.0
                    )
                    self.metrics[
                        f"{_cohort}_strike_pos_pass_exact_{_cn}"
                    ][:] = self._cohort_pos_acc[_cohort_key] * _cohort_scale
                    self.metrics[
                        f"{_cohort}_strike_vel_pass_exact_{_cn}"
                    ][:] = self._cohort_vel_acc[_cohort_key] * _cohort_scale
                    self.metrics[
                        f"{_cohort}_strike_composite_success_exact_{_cn}"
                    ][:] = self._cohort_comp_acc[_cohort_key] * _cohort_scale
            # --- HER achieved-target buffer WRITE ---------------------------------------------------
            # Record the racket state the policy ACTUALLY produced at this step's exact-strike frames
            # (pos env-origin-relative, vel world). Alive envs only by construction: terminated envs
            # were reset before the command computes, so their state never lands here. Gated on the mix
            # prob so the buffers cost nothing when replay is off.
            if self.cfg.achieved_target_mix_prob > 0.0:
                for _c in self._clip_names:
                    _bidx = torch.where(exact_strike & (_clip == _c))[0]
                    _m = int(_bidx.numel())
                    if _m == 0:
                        continue
                    _size = self._ach_pos[_c].shape[0]
                    _rows = (self._ach_ptr[_c] + torch.arange(_m, device=self.device)) % _size
                    self._ach_pos[_c][_rows] = self.racket_pos_w[_bidx] - origins[_bidx]
                    self._ach_vel[_c][_rows] = self.racket_lin_vel_w[_bidx]
                    self._ach_ptr[_c] = int((self._ach_ptr[_c] + _m) % _size)
                    self._ach_fill[_c] = min(self._ach_fill[_c] + _m, _size)
            for _c, _cn in self._clip_names.items():
                self.metrics[f"achieved_buffer_fill_{_cn}"][:] = float(self._ach_fill[_c])
        # Per-axis position error AT the exact strike frame (which axis is the miss?). The position-only
        # strike_success_exact was dropped — strike_pos_pass_exact above is the same signal, undiluted.
        _axis_err_exact = torch.abs(self.racket_pos_w - self.racket_target_pos_w)
        for _ai, _ax in enumerate(("x", "y", "z")):
            self.metrics[f"racket_pos_error_{_ax}_exact_strike"] = torch.where(
                exact_strike, _axis_err_exact[:, _ai], self.metrics[f"racket_pos_error_{_ax}_exact_strike"]
            )
        # P2.3 SMASH-style ADAPTIVE TRACKING SIGMA (coarse-to-fine): every sigma_update_every steps,
        # set the racket position/velocity reward stds from the decayed MEAN exact-strike error.
        # With sigma_monotonic=True this is a curriculum, not a moving normalization: widths may
        # tighten as precision improves but can never reopen when absolute error regresses. The
        # 2026-07-24 run proved why that distinction matters: pos error worsened 0.169 -> 0.188 m,
        # sigma widened 0.176 -> 0.196 m, and the logged position reward rose despite the regression.
        # Mutates the LIVE reward-term params and keeps racket_strike_success in lockstep.
        if (
            self.cfg.adaptive_sigma
            and enough
            and self._env.common_step_counter % int(self.cfg.sigma_update_every) == 0
        ):
            pos_mean = self._exact_pos_err_sum / denom
            vel_mean = self._exact_vel_err_sum / denom
            sigma_pos = constrained_tracking_sigma(
                self._adaptive_sigma_pos,
                float(self.cfg.sigma_ema_scale) * pos_mean,
                float(self.cfg.sigma_pos_min),
                float(self.cfg.sigma_pos_max),
                monotonic=bool(self.cfg.sigma_monotonic),
            )
            sigma_vel = constrained_tracking_sigma(
                self._adaptive_sigma_vel,
                float(self.cfg.sigma_ema_scale) * vel_mean,
                float(self.cfg.sigma_vel_min),
                float(self.cfg.sigma_vel_max),
                monotonic=bool(self.cfg.sigma_monotonic),
            )
            rm = self._env.reward_manager
            try:
                rm.get_term_cfg("racket_position").params["std"] = sigma_pos
                rm.get_term_cfg("racket_velocity").params["std"] = sigma_vel
                succ = rm.get_term_cfg("racket_strike_success").params
                succ["std_pos"] = sigma_pos
                succ["std_vel"] = sigma_vel
            except ValueError:
                pass  # a variant task without these terms: adaptive sigma is a no-op there
            self._adaptive_sigma_pos = sigma_pos
            self._adaptive_sigma_vel = sigma_vel
            # racket_normal on the SAME schedule, but driven by the strike-window face error
            # so the kernel keeps u = error/sigma ~ 1 (the Gaussian gradient optimum) instead
            # of drifting to u = 0.53 as it did through 2026-07-25.
            if self.cfg.adaptive_sigma_normal and self._win_n_acc > 0.0:
                normal_mean_rad = self._win_normal_err_sum / max(self._win_n_acc, 1.0e-6)
                sigma_normal = constrained_tracking_sigma(
                    self._adaptive_sigma_normal,
                    float(self.cfg.sigma_ema_scale) * normal_mean_rad,
                    float(self.cfg.sigma_normal_min),
                    float(self.cfg.sigma_normal_max),
                    monotonic=bool(self.cfg.sigma_monotonic),
                )
                try:
                    rm.get_term_cfg("racket_normal").params["std"] = sigma_normal
                except ValueError:
                    pass
                self._adaptive_sigma_normal = sigma_normal
        if self.cfg.adaptive_sigma:
            self.metrics["adaptive_sigma_pos"][:] = self._adaptive_sigma_pos
            self.metrics["adaptive_sigma_vel"][:] = self._adaptive_sigma_vel
            self.metrics["adaptive_sigma_normal"][:] = self._adaptive_sigma_normal
        self.metrics["position_sigma_current"][:] = self._adaptive_sigma_pos
        # REVERSIBLE PRECISION/STABILITY CURRICULUM (2026-07-24): a normal-only one-way latch
        # enabled velocity weight 24 at iteration 147, long before position or recovery was learned.
        # Require BOTH clips to clear normal + exact-position gates, and require aggregate post-fall
        # and runner-equivalent READY gates. After a healthy dwell, ramp up gradually; when any
        # signal regresses, ramp back down instead of permanently rewarding an unstable shortcut.
        _vel_boot = float(getattr(self.cfg, "vel_weight_bootstrap", 0.0))
        if self._velocity_curriculum_mode == "staged_hysteresis_v2":
            self._update_staged_velocity_curriculum()
        elif _vel_boot > 0.0:
            try:
                _vel_term = self._env.reward_manager.get_term_cfg("racket_velocity")
            except ValueError:
                _vel_term = None  # variant task without the term: the gate is a no-op
            if _vel_term is not None:
                if self._vel_weight_full is None:
                    self._vel_weight_full = float(_vel_term.weight)
                _normal_gate = float(self.cfg.vel_weight_bootstrap_normal_pass)
                _position_gate = float(self.cfg.vel_weight_bootstrap_position_pass)
                _min_n = float(self.cfg.exact_success_min_count)
                if self._clip_names and len(self._clip_names) > 1:
                    _qualified = all(
                        self._exact_n_acc_c[_c] >= _min_n for _c in self._clip_names
                    )
                    _normal_rates = [
                        (
                            self._exact_pass_normal_acc_c[_c]
                            / max(self._exact_n_acc_c[_c], 1e-6)
                        )
                        for _c in self._clip_names
                    ]
                    _position_rates = [
                        (
                            self._exact_pass_pos_acc_c[_c]
                            / max(self._exact_n_acc_c[_c], 1e-6)
                        )
                        for _c in self._clip_names
                    ]
                else:
                    _qualified = bool(enough)
                    _normal_rates = [self._exact_pass_normal_acc / denom]
                    _position_rates = [self._exact_pass_pos_acc / denom]

                _post_fall_max = float(
                    self.cfg.vel_weight_bootstrap_post_fall_max
                )
                _swing_denom = max(self._swing_starts_acc, 1e-6)
                _post_fall_rate = min(
                    self._poststrike_fall_acc / _swing_denom, 1.0
                )
                _ready_rate = float(self._ready_latched.float().mean())
                (
                    _normal_ok,
                    _position_ok,
                    _post_fall_ok,
                    _ready_ok,
                    _all_ok,
                ) = velocity_curriculum_gate_status(
                    qualified=bool(_qualified),
                    normal_rates=_normal_rates,
                    position_rates=_position_rates,
                    normal_min=_normal_gate,
                    position_min=_position_gate,
                    post_fall_rate=_post_fall_rate,
                    post_fall_max=_post_fall_max,
                    ready_rate=_ready_rate,
                    ready_min=float(self.cfg.vel_weight_bootstrap_ready_min),
                )
                # A fall ratio is not trustworthy until the independent swing-start EMA has
                # enough mass, even if its startup default happens to be below the threshold.
                _post_fall_ok = bool(_s_enough and _post_fall_ok)
                _all_ok = bool(_all_ok and _post_fall_ok)
                _position_fh = float(_position_rates[0])
                _position_bh = float(
                    _position_rates[1]
                    if len(_position_rates) > 1
                    else _position_rates[0]
                )
                _normal_fh = float(_normal_rates[0])
                _normal_bh = float(
                    _normal_rates[1]
                    if len(_normal_rates) > 1
                    else _normal_rates[0]
                )
                _block_reason = 0
                if not _qualified:
                    _block_reason |= 1
                if _position_fh < _position_gate:
                    _block_reason |= 2
                if _position_bh < _position_gate:
                    _block_reason |= 4
                if _normal_fh < _normal_gate:
                    _block_reason |= 8
                if _normal_bh < _normal_gate:
                    _block_reason |= 16
                if not _post_fall_ok:
                    _block_reason |= 32
                if not _ready_ok:
                    _block_reason |= 64
                self._vel_weight_progress, self._vel_weight_gate_dwell = (
                    advance_velocity_curriculum(
                        self._vel_weight_progress,
                        self._vel_weight_gate_dwell,
                        gates_ok=_all_ok,
                        dwell_steps=int(self.cfg.vel_weight_bootstrap_dwell_steps),
                        ramp_up_steps=int(self.cfg.vel_weight_ramp_up_steps),
                        ramp_down_steps=int(self.cfg.vel_weight_ramp_down_steps),
                    )
                )
                # Retain the legacy field for old exact-resume state compatibility; it now means
                # "currently at full curriculum progress", not a permanent one-way latch.
                self._vel_weight_latched = self._vel_weight_progress >= 1.0
                _vel_term.weight = interpolated_velocity_weight(
                    _vel_boot, self._vel_weight_full, self._vel_weight_progress
                )
                self.metrics["racket_velocity_weight_live"][:] = float(_vel_term.weight)
                self.metrics["velocity_weight_current"][:] = float(_vel_term.weight)
                self.metrics["velocity_gate_stable_steps"][:] = float(
                    self._vel_weight_gate_dwell
                )
                self.metrics["velocity_gate_block_reason"][:] = float(
                    _block_reason
                )
                self.metrics["velocity_gate_position_fh"][:] = _position_fh
                self.metrics["velocity_gate_position_bh"][:] = _position_bh
                self.metrics["velocity_gate_normal_fh"][:] = _normal_fh
                self.metrics["velocity_gate_normal_bh"][:] = _normal_bh
                self.metrics["velocity_gate_post_fall"][:] = _post_fall_rate
                self.metrics["velocity_gate_ready"][:] = _ready_rate
                self.metrics["racket_velocity_gate_normal_ok"][:] = float(_normal_ok)
                self.metrics["racket_velocity_gate_position_ok"][:] = float(_position_ok)
                self.metrics["racket_velocity_gate_post_fall_ok"][:] = float(_post_fall_ok)
                self.metrics["racket_velocity_gate_ready_ok"][:] = float(_ready_ok)
                self.metrics["racket_velocity_gate_all_ok"][:] = float(_all_ok)
                self.metrics["racket_velocity_gate_dwell_fraction"][:] = min(
                    self._vel_weight_gate_dwell
                    / float(self.cfg.vel_weight_bootstrap_dwell_steps),
                    1.0,
                )
                self.metrics["racket_velocity_gate_progress"][:] = (
                    self._vel_weight_progress
                )
        else:
            # Fixed-weight/legacy recipes do not enter either curriculum branch. The old metrics
            # therefore stayed at their zero initialization even while the live reward manager
            # correctly used weight 14, making a healthy reward look disabled in W&B.
            try:
                _fixed_velocity_weight = float(
                    self._env.reward_manager.get_term_cfg(
                        "racket_velocity"
                    ).weight
                )
            except ValueError:
                _fixed_velocity_weight = 0.0
            for _metric in (
                "racket_velocity_weight_live",
                "velocity_weight_current",
                "velocity_current_weight",
                "velocity_target_weight",
            ):
                self.metrics[_metric][:] = _fixed_velocity_weight
        self._update_recovery_curriculum()
        self._update_strike_local_reward_instrumentation()
        # A1 target-latency diagnostics, refreshed every step because
        # CommandTerm.reset() zeros metric entries of resetting envs before logging them.
        # (midswing_resample_count is written per step in _update_command while the feature is on.)
        robustness_scale, effective_delay = self._effective_target_robustness()
        self.metrics["target_robustness_scale"][:] = robustness_scale
        self.metrics["target_delay_steps_in_effect"][:] = float(effective_delay)

        # Success-gated curriculum: widen the perturbation only once the smoothed CONDITIONAL exact-strike
        # composite success (fraction of exact-strike samples passing all three thresholds) clears the bar.
        if self.cfg.target_mode == "reference_perturbed" and self.cfg.ref_perturb_success_gated:
            if (
                self._curr_perturb_scale < 1.0
                and enough
                and self._exact_composite_rate > self.cfg.ref_perturb_advance_threshold
            ):
                self._curr_perturb_scale = min(
                    1.0, self._curr_perturb_scale + float(self.cfg.ref_perturb_advance_rate)
                )
        self.metrics["racket_pos_error_at_strike"] = torch.where(
            in_win, pos_err, self.metrics["racket_pos_error_at_strike"]
        )
        self.metrics["racket_vel_error_at_strike"] = torch.where(
            in_win, vel_err, self.metrics["racket_vel_error_at_strike"]
        )
        self.metrics["racket_normal_error_deg_at_strike"] = torch.where(
            in_win, normal_err_deg, self.metrics["racket_normal_error_deg_at_strike"]
        )
        self.metrics["strike_success"] = torch.where(
            in_win, (pos_err < self.cfg.strike_success_pos_thresh).float(), self.metrics["strike_success"]
        )
        # Base target is tracked before the strike, so log that error during the pre-strike phase.
        self.metrics["base_pos_error_pre_strike"] = torch.where(
            self.pre_strike, base_err, self.metrics["base_pos_error_pre_strike"]
        )
        # Planar base speed HELD at the strike window (= base speed AT contact; low = settled by strike).
        base_speed_xy_now = torch.norm(self.robot.data.root_lin_vel_w[:, :2], dim=-1)
        self.metrics["base_speed_at_strike"] = torch.where(
            in_win, base_speed_xy_now, self.metrics["base_speed_at_strike"]
        )

        # Swing-quality detail held at the most recent strike: actual/target paddle speed and the
        # per-axis position error (which direction is the miss?).
        racket_speed = torch.norm(self.racket_lin_vel_w, dim=-1)
        target_speed = torch.norm(self.racket_target_vel_w, dim=-1)
        axis_err = torch.abs(self.racket_pos_w - self.racket_target_pos_w)
        self.metrics["racket_speed_at_strike"] = torch.where(
            in_win, racket_speed, self.metrics["racket_speed_at_strike"]
        )
        # WINDOW-EXIT BIAS (2026-07-24 audit): the *_at_strike family is overwritten on every
        # in-window step, so the surviving value is the window-EXIT frame (~0.12 s AFTER contact,
        # mid follow-through deceleration) — the same mechanism that broke the 5cm/10cm windows
        # (hope-wbc-5cm-10cm-broken).  The exact-strike-masked twins below are the trustworthy
        # speed numbers; use THEM to adjudicate the momentum ledger, not *_at_strike.
        self.metrics["racket_speed_exact_strike"] = torch.where(
            exact_strike, racket_speed, self.metrics["racket_speed_exact_strike"]
        )
        self.metrics["racket_target_speed_exact_strike"] = torch.where(
            exact_strike, target_speed, self.metrics["racket_target_speed_exact_strike"]
        )
        self.metrics["racket_target_speed_at_strike"] = torch.where(
            in_win, target_speed, self.metrics["racket_target_speed_at_strike"]
        )
        self.metrics["racket_pos_error_x_at_strike"] = torch.where(
            in_win, axis_err[:, 0], self.metrics["racket_pos_error_x_at_strike"]
        )
        self.metrics["racket_pos_error_y_at_strike"] = torch.where(
            in_win, axis_err[:, 1], self.metrics["racket_pos_error_y_at_strike"]
        )
        self.metrics["racket_pos_error_z_at_strike"] = torch.where(
            in_win, axis_err[:, 2], self.metrics["racket_pos_error_z_at_strike"]
        )
        # Window-exit-held (kept for continuity; the trustworthy contact-frame version is
        # exact_strike_pos_success_5cm/10cm, computed on the exact-strike mask above).
        self.metrics["strike_success_5cm_window_exit"] = torch.where(
            in_win, (pos_err < 0.05).float(), self.metrics["strike_success_5cm_window_exit"]
        )
        self.metrics["strike_success_10cm_window_exit"] = torch.where(
            in_win, (pos_err < 0.10).float(), self.metrics["strike_success_10cm_window_exit"]
        )

        # Robot-health diagnostics (episode-wide, instantaneous).
        data = self.robot.data
        self.metrics["base_height"].copy_(data.root_pos_w[:, 2])
        self.metrics["base_upright"] = matrix_from_quat(self.base_quat_w)[:, 2, 2]  # 1.0 = perfectly upright
        self.metrics["joint_vel_abs_max"] = torch.max(torch.abs(data.joint_vel), dim=-1).values
        # --- stability diagnostics: absolute base roll/pitch + foot contact/slip --------------------
        _roll, _pitch, _ = euler_xyz_from_quat(self.base_quat_w)
        # wrap to (-180, 180] so a level base reads ~0 (euler_xyz_from_quat can return [0, 2pi))
        self.metrics["base_roll_deg"] = torch.rad2deg(torch.atan2(torch.sin(_roll), torch.cos(_roll)))
        self.metrics["base_pitch_deg"] = torch.rad2deg(torch.atan2(torch.sin(_pitch), torch.cos(_pitch)))
        self._refresh_current_foot_contact_state()
        # footwork-to-strike signals (racket progress, foot slip²/vel/drag, arm overreach, strike stability)
        self._update_footwork_signals(pos_err)
        # HOT FOLLOW-THROUGH CAPTURE (2026-07-23 fall-phase fix): the post-swing replay buffer was
        # fed only at clip WRAP (~1.3 s after impact), but the deterministic eval located ~30% of
        # per-swing falls in the post-strike segment BEFORE wrap — the most dangerous
        # follow-through states never entered the recovery curriculum (survivor bias).  Capture
        # every live env once per swing as tts crosses -capture_delay (peak-momentum
        # follow-through), tilt-filtered so clearly-doomed states are not replayed.  This changes
        # ONLY the reset distribution; no reward lives on these frames (post_strike_brake/GAE
        # lesson, see hope-rally-recovery-root-cause).
        capture_delay = float(getattr(self.cfg, "post_strike_capture_delay_s", 0.0))
        capture_delays = self._post_strike_capture_delays_s
        if capture_delays:
            motion = self._motion()
            previous_capture_tts = self._post_strike_capture_prev_tts.clone()
            # A captured Markov state must contain the edge-detector value that the next policy
            # step would observe. Advance it before serializing any snapshot, while retaining the
            # previous value locally for this tick's crossing test.
            self._post_strike_capture_prev_tts.copy_(self.time_to_strike)
            if getattr(motion, "_post_swing_replay_contract", "") not in {
                "markov_stratified_v2",
                "markov_side_phase_severity_v3",
            }:
                raise RuntimeError(
                    "post_strike_capture_delays_s requires "
                    "a Markov replay contract"
                )
            expected_bins = len(capture_delays) + 1
            if int(motion.cfg.post_swing_capture_phase_bins) != expected_bins:
                raise RuntimeError(
                    "V17 capture contract mismatch: motion phase bins must equal "
                    f"len(racket capture delays)+1 ({expected_bins})"
                )
            self.metrics["post_strike_hot_capture"].zero_()
            any_eligible = torch.zeros(
                self.num_envs, dtype=torch.bool, device=self.device
            )
            if motion.post_swing_capture_enabled():
                max_tilt = float(
                    getattr(self.cfg, "post_strike_capture_max_tilt", 1.0)
                )
                for phase_index, delay in enumerate(capture_delays):
                    crossed = (
                        (self.time_to_strike <= -delay)
                        & (previous_capture_tts > -delay)
                    )
                    eligible = crossed & (self.proj_grav_xy <= max_tilt)
                    ids = torch.where(eligible)[0]
                    if len(ids) > 0:
                        motion._capture_post_swing_states(
                            ids,
                            phase_bin=phase_index,
                            source_clip_ids=motion.clip_id[ids].clone(),
                        )
                    self.metrics[
                        f"post_strike_hot_capture_phase_{phase_index}"
                    ] = eligible.float()
                    any_eligible |= eligible
                self.metrics["post_strike_hot_capture"] = any_eligible.float()
            self._capture_post_strike_risk_edges(motion)
        elif capture_delay > 0.0:
            motion = self._motion()
            previous_capture_tts = self._post_strike_capture_prev_tts.clone()
            self._post_strike_capture_prev_tts.copy_(self.time_to_strike)
            crossed = (
                (self.time_to_strike <= -capture_delay)
                & (previous_capture_tts > -capture_delay)
            )
            if motion.post_swing_capture_enabled():
                max_tilt = float(
                    getattr(self.cfg, "post_strike_capture_max_tilt", 1.0)
                )
                eligible = crossed & (self.proj_grav_xy <= max_tilt)
                ids = torch.where(eligible)[0]
                if len(ids) > 0:
                    motion._capture_post_swing_states(ids)
                self.metrics["post_strike_hot_capture"] = eligible.float()
        else:
            # Keep risk-edge telemetry explicitly zero for tasks without the V17 capture path.
            for metric_name in (
                "post_strike_risk_capture",
                "post_strike_risk_capture_warning",
                "post_strike_risk_capture_near",
            ):
                self.metrics[metric_name].zero_()
            for phase_index in range(
                self._post_strike_replay_phase_count
            ):
                self.metrics[
                    f"post_strike_risk_capture_phase_{phase_index}"
                ].zero_()
        # Phase-specific recovery diagnostics use the contact/leg buffers refreshed immediately above.
        # ``post_settle_window`` is defined from the current, FK-aligned tts near the top of this method.
        if self._leg_joint_idx:
            leg_rms = torch.sqrt(
                torch.mean(torch.square(data.joint_vel[:, self._leg_joint_idx]), dim=-1).clamp_min(0.0)
            )
        else:
            leg_rms = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_leg_speed"] = torch.where(
            post_settle_window, leg_rms, self.metrics["post_swing_leg_speed"]
        )
        self.metrics["post_swing_foot_slip"] = torch.where(
            post_settle_window, self.metrics["foot_slip_speed"], self.metrics["post_swing_foot_slip"]
        )
        base_tilt_deg = torch.rad2deg(
            torch.asin(self.proj_grav_xy.clamp(min=0.0, max=1.0))
        )
        self.metrics["post_swing_base_tilt_deg"] = torch.where(
            post_settle_window, base_tilt_deg, self.metrics["post_swing_base_tilt_deg"]
        )
        self.metrics["post_swing_root_height_m"] = torch.where(
            post_settle_window, data.root_pos_w[:, 2], self.metrics["post_swing_root_height_m"]
        )
        self.metrics["post_swing_foot_contact_frac"] = torch.where(
            post_settle_window,
            self.metrics["foot_contact_frac"],
            self.metrics["post_swing_foot_contact_frac"],
        )
        if self._has_jpos_limits:
            limits = getattr(data, "soft_joint_pos_limits", None)
            if limits is None:
                limits = data.joint_pos_limits
            half_span = ((limits[..., 1] - limits[..., 0]) * 0.5).clamp(min=1e-6)
            dist = torch.minimum(data.joint_pos - limits[..., 0], limits[..., 1] - data.joint_pos).clamp(min=0.0)
            self.metrics["joint_pos_near_limit_frac"] = ((dist / half_span) < 0.1).float().mean(dim=-1)
        if self._has_torque:
            tau_abs = torch.abs(data.applied_torque)
            self.metrics["joint_torque_abs_mean"] = torch.mean(tau_abs, dim=-1)
            self.metrics["joint_torque_abs_max"] = torch.max(tau_abs, dim=-1).values
        act = getattr(self._env.action_manager, "action", None)
        if act is not None:
            a_abs = torch.abs(act)
            self.metrics["action_abs_mean"] = torch.mean(a_abs, dim=-1)
            self.metrics["action_abs_max"] = torch.max(a_abs, dim=-1).values
            prev_act = getattr(self._env.action_manager, "prev_action", None)
            if prev_act is not None:
                delta_abs = torch.abs(act - prev_act)
                self.metrics["action_delta_abs_mean"] = torch.mean(delta_abs, dim=-1)
                self.metrics["action_delta_abs_max"] = torch.max(delta_abs, dim=-1).values
            else:
                self.metrics["action_delta_abs_mean"].zero_()
                self.metrics["action_delta_abs_max"].zero_()
        self._refresh_build_static_telemetry()
        self._update_qdes_phase_telemetry()
        self._update_build_ability_curriculum()
        self._update_foundation_stability_instrumentation()

    # ------------------------------------------------------------------ #
    # Observation helpers (base-relative quantities)
    # ------------------------------------------------------------------ #
    def racket_target_pos_b(self) -> torch.Tensor:
        """Desired racket position relative to the base (yaw-heading frame). HITTER actor obs.

        PRIVILEGED: uses ``base_pos_w`` (world base position). Mocap streams the base pose at 300 Hz
        during play, but that link is not bridged into the deploy front-end, so this term is fabricated
        at deploy; base-position-freedom is a deliberate robustness choice. Used by the `full` obs mode;
        the deploy-parity mode (legacy task name: `real_sensor_only`) replaces it with
        :meth:`racket_target_pos_b_rel`.
        """
        return quat_rotate_inverse(yaw_quat(self.base_quat_w), self.racket_target_pos_w - self.base_pos_w)

    def racket_target_pos_b_rel(self) -> torch.Tensor:
        """Desired racket position relative to the CURRENT racket (FK), in the yaw-heading frame.

        DEPLOY-HONEST (no world base position): expanding the rotation, the base position cancels::

            R_yaw^T (target_w - racket_w) = R_yaw^T(target_w - base_w) - R_yaw^T(racket_w - base_w)
                                          = (target in base frame) - (racket FK in base frame)

        Both terms are computable on the real robot from the planner's target + racket forward
        kinematics (joint encoders), WITHOUT a fabricated base pose. Replaces
        :meth:`racket_target_pos_b` in the deploy-parity observation mode (legacy task name:
        ``real_sensor_only``).

        A1: reads the ACTOR-visible target view (delayed/jittered when the A1 knobs are on;
        the live tensor itself otherwise — byte-identical default). This method backs the
        deploy-parity ACTOR obs only; the critic's :meth:`racket_target_pos_b` stays live.
        """
        return quat_rotate_inverse(yaw_quat(self.base_quat_w), self.actor_racket_target_pos_w() - self.racket_pos_w)

    # --- A1 ACTOR-visible target accessors (delayed/jittered view; live aliases when off) ------- #
    def actor_racket_target_pos_w(self) -> torch.Tensor:
        """ACTOR-visible desired racket position (world): the A1 delayed/jittered view when target
        latency/jitter is enabled, else the live tensor itself (zero-overhead alias). Rewards,
        metrics, and the privileged critic keep reading the TRUE live ``racket_target_pos_w``."""
        return self.delayed_racket_target_pos_w

    def actor_racket_target_vel_w(self) -> torch.Tensor:
        """ACTOR-visible desired racket velocity (world). See :meth:`actor_racket_target_pos_w`."""
        return self.delayed_racket_target_vel_w

    def actor_swing_sign(self) -> torch.Tensor:
        """ACTOR-visible swing sign (forehand +1 / backhand -1), delayed with the target when A1
        latency is on (the swing-type flag rides the same planner->runner message as the target)."""
        return self.delayed_swing_sign

    def base_target_pos_b(self) -> torch.Tensor:
        """Desired base XY position relative to the current base (yaw-heading frame). HITTER actor obs."""
        delta_xy = self.base_target_pos_w - self.base_pos_w[:, :2]
        delta = torch.cat([delta_xy, torch.zeros(self.num_envs, 1, device=self.device)], dim=-1)
        return quat_rotate_inverse(yaw_quat(self.base_quat_w), delta)[:, :2]

    # --- HITTER Table-I exact accessors (hitter_pure contract, 2026-07-07) ----------------------- #
    # The paper expresses target vectors in the WORLD frame and gives the actor the base forward
    # vector e_base,x separately (instead of pre-rotating into the heading frame). Deploy sources:
    # position differences = planner target − mocap base position (both in the mocap/table world
    # frame, no rotation needed); e_base,x = IMU orientation after the runner's yaw-align-at-engage.
    def base_forward_xy(self) -> torch.Tensor:
        """Base forward unit vector e_base,x, world-frame xy (HITTER Table I)."""
        fwd = quat_apply(
            self.base_quat_w,
            torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, 3),
        )[:, :2]
        return fwd / (torch.norm(fwd, dim=-1, keepdim=True) + 1e-6)

    def base_target_delta_xy_w(self) -> torch.Tensor:
        """Target base position p̂_base,xy − p_base,xy, WORLD frame (HITTER Table I)."""
        return self.base_target_pos_w - self.base_pos_w[:, :2]

    def racket_target_rel_base_w(self) -> torch.Tensor:
        """Target racket position relative to the base, WORLD frame (HITTER Table I / §V-B-1:
        "the racket position relative to the base ... expressed in the world frame").
        A1: reads the ACTOR-visible target view (delayed/jittered when the knobs are on)."""
        return self.actor_racket_target_pos_w() - self.base_pos_w

    # --- Deploy-matched mocap actor view ------------------------------------------------------ #
    def actor_base_quat_w(self) -> torch.Tensor:
        """Delayed/noisy mocap-anchored world<-base quaternion (w,x,y,z)."""
        return self._actor_base_quat_w

    def actor_projected_gravity_b(self) -> torch.Tensor:
        gravity_w = torch.tensor(
            [0.0, 0.0, -1.0], device=self.device
        ).expand(self.num_envs, 3)
        return quat_rotate_inverse(self._actor_base_quat_w, gravity_w)

    def actor_base_forward_xy(self) -> torch.Tensor:
        forward_w = quat_apply(
            self._actor_base_quat_w,
            torch.tensor(
                [1.0, 0.0, 0.0], device=self.device
            ).expand(self.num_envs, 3),
        )[:, :2]
        return forward_w / torch.linalg.norm(
            forward_w, dim=-1, keepdim=True
        ).clamp_min(1.0e-6)

    def actor_base_target_delta_xy_w(self) -> torch.Tensor:
        return self.base_target_pos_w - self._actor_base_pos_w[:, :2]

    def actor_racket_target_rel_base_w(self) -> torch.Tensor:
        return self.actor_racket_target_pos_w() - self._actor_base_pos_w

    def actor_base_velocity_xy_w(self) -> torch.Tensor:
        return self._actor_base_velocity_xy

    def actor_base_localization_age(self) -> torch.Tensor:
        return (self._actor_base_age_s / self._base_mocap_max_age_s).clamp(0.0, 1.0)

    # --- V15 HUGWBC locomotion command ------------------------------------------------------- #
    def desired_lateral_velocity(self) -> torch.Tensor:
        """Finite lateral velocity command; zero after the planned gait cycles finish."""
        return torch.where(
            self._locomotion_gait_active,
            self._locomotion_velocity_y,
            torch.zeros_like(self._locomotion_velocity_y),
        ).unsqueeze(-1)

    def gait_clock(self) -> torch.Tensor:
        """Left/right HUGWBC sine clocks; both zero in latched STAND mode."""
        return self._gait_clock

    def desired_contact_states(self) -> torch.Tensor:
        """Smoothed left/right desired stance probabilities used by contact rewards."""
        return self._desired_contact_states

    def locomotion_mode(self) -> torch.Tensor:
        """Actor-visible phase: +1=finite STEP, 0=hold/STAND, -1=racket swing/recovery."""
        mode = torch.where(
            self._locomotion_supervision,
            self._locomotion_move.float(),
            -torch.ones_like(self._locomotion_velocity_y),
        )
        return mode.unsqueeze(-1)

    def locomotion_supervision(self) -> torch.Tensor:
        """Mask for lower-body rewards: active only during the pre-swing hold."""
        return self._locomotion_supervision

    def finite_station_latched_error(self) -> torch.Tensor:
        """XY station error captured once per finite command (plan for STAND, completion for STEP)."""
        return self._finite_station_latched_error

    def balance_supervision(self) -> torch.Tensor:
        """HUGWBC lower-body foundation mask, active through hold, swing and recovery.

        The finite station displacement itself is prepared during ``in_hold`` only, but balance
        must not disappear when the racket wind-up starts.  HUGWBC's defining contract is that
        locomotion remains controlled under upper-body motion; turning these terms off for the
        swing would recreate V14's unsupported lower body at exactly the dangerous instant.
        """
        return torch.full_like(self._locomotion_move, self._locomotion_enabled)

    def upper_intervention_supervision(self) -> torch.Tensor:
        """Windows where training-only HUGWBC arm replacement cannot corrupt the strike.

        Intervention runs while the finite STAND/STEP command is prepared and again after the
        scored strike window.  The released wind-up/contact stays policy-controlled so the clean
        half of the population can still learn HITTER, while recovery is trained against arbitrary
        arm configurations instead of only the two recorded swings.
        """
        post_strike = self.time_to_strike < -float(self.cfg.strike_window_s)
        return self._locomotion_supervision | post_strike

    def upper_intervention_strength(self) -> torch.Tensor:
        """Adaptive HUGWBC curriculum strength consumed by the action term."""
        return self._intervention_strength

    # ------------------------------------------------------------------ #
    # Debug visualization (no-op stubs; targets are world-frame buffers).
    # ------------------------------------------------------------------ #
    def _set_debug_vis_impl(self, debug_vis: bool):
        pass

    def _debug_vis_callback(self, event):
        pass


@configclass
class RacketTargetCommandCfg(CommandTermCfg):
    """Configuration for :class:`RacketTargetCommand`."""

    class_type: type = RacketTargetCommand

    # Deterministic evaluator only. Each row is
    # (clip_id, station_y, reach_x, reach_y, target_z, target_vx, target_vy, target_vz).
    # MotionCommand owns the synchronized cyclic index. Empty is the train/play default.
    eval_gate3_sequence: tuple[tuple[float, ...], ...] = ()

    asset_name: str = MISSING
    motion_command_name: str = "motion"

    # The target is re-sampled per swing (on clip wrap / reset), not on a fixed time schedule,
    # so disable the base CommandTerm time-based resampling.
    resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)

    # --- racket mount FK ---
    racket_body_name: str = "pingpang_red_Link"
    wrist_body_name: str = "right_wrist_yaw_Link"
    mount_offset: tuple[float, float, float] = (0.210211399202899, 0.0320784994676765, 0.0320358706296689)
    # Fixed wrist->racket rotation (w, x, y, z); only used in the wrist_offset FK fallback. Identity
    # for the A3 ping-pong URDF (all mount joints are rpy=0). Set non-identity if the mount tilts the
    # paddle relative to the wrist frame.
    mount_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    mount_normal_axis: int = 1  # racket-local +Y is the face normal (red/hitting face; confirmed in Step 11)
    mount_normal_sign: float = 1.0  # +1 = red/forehand face; -1 = black/backhand face
    # Per-clip override of mount_normal_sign for a UNIFIED forehand+backhand policy. A real paddle strikes
    # with OPPOSITE faces on the two swings (forehand=red/+Y, backhand=black/−Y). With a single scalar sign
    # the backhand's face-normal target (normal_mode="velocity") is ~180° unreachable: the +Y face can never
    # lead the backhand swing, so its normal error is pinned (observed 2026-07-07: fh normal 4°, bh 137°) and
    # its composite success stays 0 even though pos/vel pass. Set (forehand, backhand) e.g. (1.0, -1.0) to
    # score each swing's real striking face. Empty () -> scalar mount_normal_sign for every clip (back-compat).
    mount_normal_sign_per_clip: tuple = ()

    # --- strike timing (fraction of the reference clip where the paddle meets the ball) ---
    strike_phase: float = 0.46  # HITTER clip: strike at frame 43/94 ≈ 0.46
    # Unified multi-clip (HITTER forehand+backhand single policy): per-clip strike phase, aligned with the
    # MotionLoader segment order (i.e. the order of motion files: forehand, backhand). Empty -> use the
    # scalar strike_phase for every clip. e.g. (0.36, 0.74) for forehand_new + backhand_new.
    strike_phase_per_clip: tuple = ()
    strike_window_s: float = 0.1  # half-window; goal-racket reward active within ±strike_window_s
    # Independent position-derived gate. <=0 preserves the legacy full strike window. RallyV15
    # sets 0.04 s and owns its temporal scale in YAML; velocity/normal remain on strike_window_s.
    position_guidance_window_s: float = 0.0
    position_guidance_temporal_scale: float = 1.0
    strike_success_pos_thresh: float = 0.075  # m; "strike_success" metric = fraction of strikes with racket pos error below this
    strike_success_vel_thresh: float = 0.5  # m/s; exact-strike racket velocity acceptance threshold
    strike_success_normal_thresh_deg: float = 15.0  # deg; exact-strike face-normal acceptance threshold
    # Hot follow-through capture into the post-swing replay buffer (2026-07-23 fall-phase fix):
    # capture each live env once per swing when tts crosses -delay (peak-momentum follow-through),
    # in ADDITION to the legacy settled wrap capture.  0.0 = off (legacy wrap-only buffer).
    post_strike_capture_delay_s: float = 0.0
    # V17 stratified successor: one ordered hot capture per delay, plus a final wrap bin owned by
    # MotionCommand. Empty preserves the scalar legacy path.
    post_strike_capture_delays_s: tuple[float, ...] = ()
    # Tilt filter (||projected_gravity_xy|| ~= sin(tilt)): states already past this tilt are
    # near-certain falls and would waste replay resets.  0.45 ~= 26.7 deg vs the 40 deg terminator.
    post_strike_capture_max_tilt: float = 0.45

    # --- RallyV17 reversible recovery curriculum ----------------------------------------------
    # Defaults disabled: V11--V16 keep their exact reset/reward distribution. V17 starts at scale
    # zero (exact V11) and advances only after BOTH clips demonstrate strike competence.
    recovery_curriculum_enabled: bool = False
    recovery_stage1_scale: float = 0.5
    recovery_min_environment_steps: int = 24_000
    # Legacy v1 minima remain for frozen recipes. Revision 2 uses stage-specific per-side
    # denominators and never advances on an FH/BH average.
    recovery_min_exact_samples_per_side: float = 50.0
    recovery_min_swing_starts: float = 100.0
    recovery_stage1_min_exact_samples_per_side: float = 200.0
    recovery_stage1_min_swing_starts_per_side: float = 400.0
    recovery_stage2_min_exact_samples_per_side: float = 500.0
    recovery_stage2_min_swing_starts_per_side: float = 1000.0
    recovery_stage2_min_virtual_samples_per_side: float = 200.0
    recovery_stage2_min_actual_q_window_starts_per_side: float = 1000.0
    recovery_actual_q_window_steps: int = 500
    recovery_stage1_enter_completion: float = 0.55
    recovery_stage1_enter_position: float = 0.12
    recovery_stage1_enter_velocity: float = 0.08
    recovery_stage1_enter_normal: float = 0.25
    recovery_stage1_enter_composite: float = 0.01
    recovery_stage1_enter_ready: float = 0.10
    recovery_stage1_enter_post_fall_max: float = 0.20
    recovery_stage1_enter_actual_q_fault_max: float = 0.50
    recovery_stage1_enter_dwell_steps: int = 250
    recovery_stage1_exit_completion: float = 0.45
    recovery_stage1_exit_position: float = 0.08
    recovery_stage1_exit_velocity: float = 0.04
    recovery_stage1_exit_normal: float = 0.18
    recovery_stage1_exit_composite: float = 0.005
    recovery_stage1_exit_ready: float = 0.05
    recovery_stage1_exit_post_fall_max: float = 0.30
    recovery_stage1_exit_actual_q_fault_max: float = 0.75
    recovery_stage1_exit_dwell_steps: int = 100
    recovery_stage1_ready_dwell_steps: int = 250
    recovery_stage1_acquisition_scales: tuple[float, ...] = ()
    recovery_stage1_acquisition_ready_thresholds: tuple[float, ...] = ()
    recovery_stage1_acquisition_ramp_steps: int = 2_000
    recovery_stage1_acquisition_timeout_steps: int = 500
    recovery_stage2_enter_completion: float = 0.70
    recovery_stage2_enter_position: float = 0.18
    recovery_stage2_enter_velocity: float = 0.40
    recovery_stage2_enter_normal: float = 0.40
    recovery_stage2_enter_composite: float = 0.25
    recovery_stage2_enter_ready: float = 0.20
    recovery_stage2_enter_safe_recovery: float = 0.20
    recovery_stage2_enter_virtual_contact: float = 0.20
    recovery_stage2_enter_virtual_over_net: float = 0.15
    recovery_stage2_enter_virtual_legal: float = 0.10
    recovery_stage2_enter_post_fall_max: float = 0.10
    recovery_stage2_enter_actual_q_fault_max: float = 0.25
    recovery_stage2_enter_dwell_steps: int = 500
    recovery_stage2_exit_completion: float = 0.60
    recovery_stage2_exit_position: float = 0.14
    recovery_stage2_exit_velocity: float = 0.30
    recovery_stage2_exit_normal: float = 0.30
    recovery_stage2_exit_composite: float = 0.15
    recovery_stage2_exit_ready: float = 0.10
    recovery_stage2_exit_safe_recovery: float = 0.10
    recovery_stage2_exit_virtual_contact: float = 0.10
    recovery_stage2_exit_virtual_over_net: float = 0.08
    recovery_stage2_exit_virtual_legal: float = 0.05
    recovery_stage2_exit_post_fall_max: float = 0.15
    recovery_stage2_exit_actual_q_fault_max: float = 0.50
    recovery_stage2_exit_dwell_steps: int = 150
    recovery_stage0_to_1_ramp_steps: int = 8_000
    recovery_stage1_coverage_ramp_steps: int = 8_000
    recovery_stage1_to_0_ramp_steps: int = 4_000
    recovery_stage1_to_2_ramp_steps: int = 12_000
    recovery_stage2_to_1_ramp_steps: int = 6_000

    # --- nominal stance (offset of the base from the env origin) ---
    base_nominal_offset: tuple[float, float, float] = (0.0, 0.0, 0.93)

    # --- target generation mode ---
    # "uniform": independent box sampling from the *_range fields below (legacy; the boxes are
    #   PLACEHOLDERS not tied to the swing, so the imitated swing's racket may never pass through them).
    # "reference_perturbed": target = the reference swing's racket state AT the strike frame (pos/vel/
    #   normal, computed by the same FK as the actual racket) + a curriculum-scaled uniform perturbation.
    #   Reachable by construction (a perfect imitator scores exactly); the *_range fields are ignored.
    # "hitter_pure": HITTER-faithful (arXiv:2508.21043 §V-B-1 + §IV-C, 2026-07-07): base station sampled
    #   INDEPENDENTLY from base_target_*_range (a STATION BOX, not jitter); racket target on a striking
    #   plane FIXED RELATIVE to the commanded station (racket_pos_range_per_clip = STATION-RELATIVE x/y
    #   offsets, z absolute); face-normal target from normal_mode ("velocity" = paper impact model) —
    #   NEVER the reference-clip normal. No HER, no reference_reach coupling, no curriculum.
    target_mode: str = "reference_perturbed"

    # reference_perturbed perturbation (final half-extents; scaled 0->1 by the curriculum below).
    ref_perturb_pos: tuple[float, float, float] = (0.15, 0.20, 0.15)  # m, per-axis half-range
    ref_perturb_vel: tuple[float, float, float] = (1.0, 1.0, 0.8)  # m/s, per-axis half-range
    ref_perturb_normal: float = 0.30  # face-normal jitter magnitude (added then renormalized)
    # Curriculum: perturbation half-extents ramp from `start`*final to 1.0*final.
    # Success-gated mode (default): `_curr_perturb_scale` starts at `ref_perturb_curriculum_start` and
    # only advances (by `ref_perturb_advance_rate` per control step) once the smoothed exact-strike
    # composite success exceeds `ref_perturb_advance_threshold` — keeps the strike error inside the
    # racket reward kernel's responsive band until the policy demonstrably hits, then widens.
    # Open-loop fallback (success_gated=False): ramp over `ref_perturb_curriculum_steps` control steps
    # (env.common_step_counter); set steps<=0 to disable the ramp (always full).
    ref_perturb_curriculum_steps: int = 30000
    ref_perturb_curriculum_start: float = 0.05
    ref_perturb_success_gated: bool = True
    ref_perturb_advance_threshold: float = 0.30  # widen once smoothed exact-strike composite success > this
    ref_perturb_advance_rate: float = 1.0e-5  # scale increment per control step while above threshold

    # --- reference velocity scaling (stage slow -> fast hitting) ---
    # The sampled racket-velocity target is `ref_vel_scale * reference_vel + perturbation`. The reference
    # forehand clip strikes at ~6 m/s; set <1.0 to train a slower, more controllable hit FIRST (e.g. 0.6
    # -> ~3.6 m/s) and ramp back to 1.0 once the slow strike is accurate. NOTE: at scale!=1.0 the target
    # velocity no longer equals the imitated swing's velocity, so a perfect imitator no longer matches it
    # exactly (the "reachable by construction" guarantee holds for position/normal, not scaled velocity).
    ref_vel_scale: float = 1.0

    # --- clean reference strike velocity (denoise the target velocity) ---
    # The motion's stored body_lin_vel_w is a finite-difference (torch.gradient) of the 30->50 fps
    # interpolated joint trajectory propagated through FK (see scripts/csv_to_npz.py). At the fast,
    # high-jerk racket tip those FD/interpolation errors accumulate to ~1 m/s and are INCONSISTENT with
    # the position trajectory (stored-vel vs central-diff-of-pos differ ~1.1 m/s near the strike). Since
    # the racket-velocity reward target is essentially the reference velocity, that ~1 m/s noise is the
    # floor on racket_vel_error_exact_strike (velEx parked ~0.74 regardless of reward tuning).
    # When True, the cached strike target velocity is recomputed from the FINAL racket FK position
    # (body_pos_w, the same FK as the actual racket) by a centered finite difference over +-window frames,
    # which is consistent with the position the policy actually tracks and rejects single-frame jitter.
    # False keeps the legacy single-frame stored-velocity path.
    clean_reference_strike_velocity: bool = False
    clean_strike_vel_window: int = 2  # half-window (frames) for the centered finite difference (try 2 or 3)

    # --- debug logging (sign verification + raw/gated reward kernels) ---
    # When True, RacketTargetCommand logs dbg_err_{minus,plus}_{win,exact} (swing-through sign check) and
    # the reward terms log dbg_{racket_pos,racket_vel,racket_normal,base}_{raw,gated}. Pure logging; no
    # behaviour change. Turn off for production runs (extra wandb scalars).
    debug_reward_logging: bool = False

    # --- conditional exact-strike success metric (logging + curriculum gating) ---
    # The logged strike_*_pass_exact / strike_composite_success_exact are a sample-weighted EMA of the
    # exact-strike pass rate: acc = decay*acc + this-step-count each control step. decay ~0.99 gives a
    # ~100-step (~2 s @ 50 Hz) memory; higher = smoother but slower to reflect the current policy. The
    # rate (and the curriculum) only trust it once `exact_success_min_count` decayed samples accumulate.
    exact_success_decay: float = 0.99
    exact_success_min_count: float = 50.0

    # --- P2.3 SMASH-style adaptive tracking sigma (coarse-to-fine reward kernel widths) ---
    # When on, every `sigma_update_every` control steps the racket_position/racket_velocity reward
    # stds (and racket_strike_success's std_pos/std_vel) are set to
    #   clamp(sigma_ema_scale * decayed_mean_exact_strike_error, sigma_min, sigma_max)
    # With sigma_monotonic, the result is additionally capped by the previous live width so an
    # absolute-error regression cannot make the objective more forgiving while raising its logged
    # reward. sigma_*_min should sit at the acceptance thresholds (0.075 m / 0.5 m/s).
    adaptive_sigma: bool = False
    sigma_update_every: int = 500
    sigma_ema_scale: float = 1.0
    sigma_monotonic: bool = False
    sigma_pos_min: float = 0.075
    sigma_pos_max: float = 0.20
    sigma_vel_min: float = 0.5
    sigma_vel_max: float = 1.0
    # racket_normal was left out of the coarse-to-fine schedule until 2026-07-25. Position and
    # velocity annealed 6.7x / 1.9x while sigma_normal never moved, so the face objective was
    # silently and monotonically de-prioritised: a Gaussian's marginal gradient peaks at
    # sigma == error, and racket_normal sat at u = error/sigma = 0.53 (58% of the attainable
    # gradient). Unlike position/velocity this one is driven by the STRIKE-WINDOW error, not the
    # exact-frame error, because racket_normal_tracking_exp is gated on cmd.strike_window
    # (+/-0.12 s); using the exact-frame error here would target the wrong operating point by
    # ~1.6x. sigma_normal_min sits at the 15 deg acceptance threshold (0.262 rad).
    adaptive_sigma_normal: bool = False
    sigma_normal_min: float = 0.262
    sigma_normal_max: float = 0.80

    # --- REVERSIBLE PRECISION/STABILITY CURRICULUM on racket_velocity --------------------------
    # Hold racket_velocity at vel_weight_bootstrap until BOTH clips clear normal + position pass
    # gates and aggregate post-strike fall + runner-equivalent READY gates. After a consecutive
    # healthy dwell, ramp toward the task-YAML full weight; ramp back toward bootstrap whenever any
    # gate regresses. This prevents a transient face-only pass from permanently enabling an
    # aggressive, inaccurate swing. Set vel_weight_bootstrap <= 0 to disable the curriculum.
    vel_weight_bootstrap: float = 0.0
    vel_weight_bootstrap_normal_pass: float = 0.50
    vel_weight_bootstrap_position_pass: float = 0.0
    vel_weight_bootstrap_post_fall_max: float = 1.0
    vel_weight_bootstrap_ready_min: float = 0.0
    vel_weight_bootstrap_dwell_steps: int = 1
    vel_weight_ramp_up_steps: int = 1
    vel_weight_ramp_down_steps: int = 1

    # RallyV15 staged-hysteresis v2. The legacy fields above remain for frozen old-run exact
    # resumes; this mode is selected explicitly by the new V15 YAML.
    velocity_curriculum_mode: str = "legacy_reversible_v1"
    vel_stage0_weight: float = 14.0
    vel_stage1_weight: float = 18.0
    vel_stage2_weight: float = 24.0
    vel_stage_min_exact_samples: float = 50.0
    vel_stage_min_swing_starts: float = 50.0
    # 2026-07-25: drop the normal / ready competence proxies from the stage transition
    # condition (both enter AND exit -- see strike_curriculum.VelocityStageConfig). The
    # post_fall gate is what actually guards the documented risk. Defaults keep the old
    # 7-way AND for every task that does not opt out.
    vel_stage_gate_requires_normal: bool = True
    vel_stage_gate_requires_ready: bool = True
    vel_stage1_gate_requires_normal: bool | None = None
    vel_stage1_gate_requires_ready: bool | None = None
    vel_stage2_gate_requires_normal: bool | None = None
    vel_stage2_gate_requires_ready: bool | None = None

    vel_stage1_enter_position_pass: float = 0.12
    vel_stage1_enter_velocity_pass: float = 0.005
    vel_stage1_enter_normal_pass: float = 0.45
    vel_stage1_enter_post_fall_max: float = 0.10
    vel_stage1_enter_ready_min: float = 0.15
    vel_stage1_enter_dwell_steps: int = 250
    vel_stage1_exit_position_pass: float = 0.10
    vel_stage1_exit_velocity_pass: float = 0.0025
    vel_stage1_exit_normal_pass: float = 0.40
    vel_stage1_exit_post_fall_max: float = 0.12
    vel_stage1_exit_ready_min: float = 0.12
    vel_stage1_exit_dwell_steps: int = 100
    vel_stage0_to_1_ramp_steps: int = 8000
    vel_stage1_to_0_ramp_steps: int = 4000

    vel_stage2_enter_position_pass: float = 0.15
    vel_stage2_enter_velocity_pass: float = 0.05
    vel_stage2_enter_normal_pass: float = 0.50
    vel_stage2_enter_post_fall_max: float = 0.08
    vel_stage2_enter_ready_min: float = 0.15
    vel_stage2_enter_dwell_steps: int = 500
    vel_stage2_exit_position_pass: float = 0.13
    vel_stage2_exit_velocity_pass: float = 0.03
    vel_stage2_exit_normal_pass: float = 0.45
    vel_stage2_exit_post_fall_max: float = 0.10
    vel_stage2_exit_ready_min: float = 0.12
    vel_stage2_exit_dwell_steps: int = 150
    vel_stage1_to_2_ramp_steps: int = 12000
    vel_stage2_to_1_ramp_steps: int = 6000

    # --- A1 target latency & time-variance (mocap->planner->runner realism; roadmap A1) -------------
    # MOTIVATION: training otherwise hands the actor a PERFECT, instantly-updated target, while the
    # real loop (mocap -> planner -> runner) delivers it LATE (transport + planning latency), NOISY
    # (ball-prediction error that shrinks as the strike approaches — SMASH Eq. 14), and REFINED
    # mid-swing (the planner re-plans WHERE, not WHEN). PACE injects sensor delays for the same
    # reason. Without this, the mocap-closed-loop deployment faces out-of-distribution target
    # dynamics. Scope: ONLY the ACTOR-visible target view (pos/vel/swing_sign) is degraded; rewards,
    # metrics, the privileged critic, and the achieved-target-replay write use the TRUE live target.
    # time_to_strike is NEVER delayed: the swing clock is generated robot-side by the deploy runner,
    # not by the mocap link. ALL defaults OFF => byte-identical baseline (delay==0 aliases the live
    # tensors; jitter==0 / prob==0 short-circuit before any RNG draw).
    target_delay_steps: int = 0  # actor sees target pos/vel/swing_sign this many control steps (50 Hz) late
    # SMASH-style tts-decaying gaussian noise on the ACTOR-visible target, drawn ONCE per step on the
    # ring-buffer push (determinism within a step): per-step std = knob * clamp(time_to_strike, 0, 1),
    # i.e. the knob is the std at time_to_strike >= 1 s, decaying to 0 at the strike (prediction
    # convergence). Units: m (pos) / m/s (vel).
    target_jitter_pos_per_s: float = 0.0
    # Calibrated mocap MEASUREMENT noise on the actor-visible target position (m). Venue fit
    # 2026-07-03 (`capture.position_noise`): white 0.0019, ar1 marginal 0.0052, rho/frame 0.946
    # @300 Hz -> 0.946**6 = 0.717 per 50 Hz policy step. Defaults OFF.
    target_noise_white: float = 0.0
    target_noise_ar1_sigma: float = 0.0
    target_noise_ar1_rho: float = 0.717
    # A1 v2 — the three Ace-style sensor defects the mocap link actually has (venue capture fit:
    # occlusion gaps concentrate at contacts, gap_p50 10 ms / racket occlusion ~30 ms; re-lock
    # after a contact carries a fresh systematic bias). All default OFF.
    target_dropout_prob: float = 0.0        # per-step P(frame lost) -> actor view holds last value
    target_post_strike_dropout_s: float = 0.0  # forced hold-last window right after each strike (s)
    target_bias_per_swing: float = 0.0      # m: constant bias per swing, resampled at swing start
    target_jitter_vel_per_s: float = 0.0
    # One-run robustness curriculum: target-stream defects follow the live staged velocity
    # weight continuously. Scale is 0 at Stage-0 weight, ``target_robustness_stage1_scale`` at
    # Stage-1 weight, and 1 at Stage-2 weight; reverse ramps use the same map after a safety exit.
    # Disabled by default so existing tasks and frozen resumes remain byte-compatible.
    target_robustness_curriculum_by_velocity_stage: bool = False
    target_robustness_stage1_scale: float = 0.5
    # V17 alternative: keep target corruption exactly zero through the first half of the recovery
    # curriculum, then map [start_scale, 1] linearly to [0, 1]. Mutually exclusive with the
    # velocity-stage driver.
    target_robustness_curriculum_by_recovery_scale: bool = False
    target_robustness_recovery_start_scale: float = 0.5

    # --- Actor-visible robot localization ------------------------------------------------------
    # V15 uses position receipt v1. V17 additionally enables the calibrated full-pose schema-2
    # view. The privileged critic and rewards retain Isaac ground truth.
    base_mocap_enabled: bool = False
    base_mocap_orientation_enabled: bool = False
    # Fixed Build engineering assumptions. They are deliberately not described as measured
    # transport latency; HitterPingPong no longer owns a latency-receipt pipeline.
    base_mocap_delay_steps: int = 0
    base_mocap_update_interval_steps: int = 1
    base_mocap_position_noise_std: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_mocap_orientation_noise_std_rad: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )
    base_mocap_extrinsic_residual_rpy_std_rad: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )
    base_mocap_dropout_prob: float = 0.0
    base_mocap_velocity_ema_alpha: float = 0.25
    base_mocap_max_age_s: float = 0.20
    base_mocap_max_propagation_s: float = 0.0
    # Build-only one-way mocap-corruption gate. It does not alter V14 rewards, swing release,
    # post-swing capture, or replay sampling.
    ability_curriculum_mode: str = "disabled"
    ability_min_exact_samples_per_side: float = 50.0
    ability_min_completion_per_side: float = 0.55
    ability_min_position_pass_per_side: float = 0.15
    ability_min_composite: float = 0.03
    ability_max_post_fall: float = 0.10
    ability_gate_dwell_steps: int = 250
    base_mocap_robustness_ramp_steps: int = 8000

    # --- V15 HUGWBC lower-body command -------------------------------------------------------
    # HITTER still samples a desired station.  At each station transition it is translated once
    # into STAND or a finite number of complete gait cycles; the command never re-arms merely
    # because a residual position error remains.  All numbers are owned by the task YAML.
    locomotion_enabled: bool = False
    gait_frequency_hz: float = 1.5
    gait_duty_factor: float = 0.5
    gait_move_deadband: float = 0.10
    gait_step_distance: float = 0.24
    gait_max_cycles: int = 1
    gait_velocity_max: float = 0.40
    gait_contact_smoothing: float = 0.05
    # V15 one-command/one-bout contract. A planned gait remains in STEP after its active clock
    # ends until every settle predicate holds continuously for ``step_settle_dwell_s``; only then
    # is it irreversibly latched STAND. Defaults keep older tasks inert.
    one_step_contract: str = "disabled"
    step_settle_pos_thresh: float = 0.10
    step_settle_speed_thresh: float = 0.20
    step_settle_yaw_thresh_rad: float = math.pi
    step_settle_contact_force_threshold: float = 10.0
    step_settle_slip_thresh: float = 0.03
    step_settle_dwell_s: float = 0.30
    intervention_curriculum_start: float = 0.0
    intervention_curriculum_step: float = 0.05
    intervention_tracking_pass: float = 0.60
    intervention_tracking_fail: float = 0.45
    intervention_tracking_sigma: float = 0.25
    # Mid-swing target refinement: each control step, envs with pre_strike AND time_to_strike >
    # midswing_resample_tts_floor re-draw their target (position/velocity/normal via the existing
    # sampling path) with this per-step probability. Strike timing is untouched (same strike step),
    # no swing start is counted, and the racket-progress baseline is reset so the target jump creates
    # no fake progress.
    midswing_resample_prob: float = 0.0
    midswing_resample_tts_floor: float = 0.3  # s; no refinement inside the last `floor` seconds before the strike

    # --- ARRIVAL-GATED HOLD RELEASE (2026-07-09, default OFF) ------------------------------------
    # When True, the pre-swing HOLD is EXTENDED past its base ``hold_steps_range`` countdown while the
    # base has NOT yet SETTLED at the commanded station (base->station error > hold_settle_pos_thresh OR
    # base planar speed > hold_settle_speed_thresh), up to hold_settle_max_extra_steps extra steps (a
    # SAFETY CAP so an unreachable station cannot hang the episode). Makes the swing arm only AFTER
    # arrival ("到位才放行") instead of on a fixed timer. Requires the pre-swing hold to exist
    # (hold_steps_range > 0); a no-op otherwise. The base countdown still sets the MINIMUM hold; this
    # only lengthens it when the robot is still moving to / not yet at the station. The V2Hold task
    # enables it. Extends the hold from RacketTargetCommand (which owns base_target_pos_w) after the
    # MotionCommand decrements — see RacketTargetCommand._extend_hold_until_settled.
    hold_until_settled: bool = False
    hold_settle_pos_thresh: float = 0.12   # m; base within this of the station = "arrived"
    hold_settle_speed_thresh: float = 0.20  # m/s; base slower than this = "calm"
    hold_settle_max_extra_steps: int = 100  # cap on the extension (@50 Hz ≈ 2 s) — safety valve
    # Also require the base to be SQUARED (|world-frame heading| < this, rad) before releasing the swing —
    # only meaningful with heading recovery on (hold_heading + yawed stand starts). Default pi ≈ OFF (the
    # gate is position+speed only) so tasks without heading recovery are unaffected. V2Hold sets ~0.30.
    hold_settle_yaw_thresh: float = 3.15

    # --- Tier-1 VIRTUAL INCOMING BALL + at-strike landing evaluation (rewardDesign.md) -----------
    # Per swing, a virtual incoming ball (v_in, omega_in) is sampled that BY CONSTRUCTION arrives at
    # the racket target point at the strike time. On the exact-strike frame, the achieved racket FK
    # state is pushed through the venue-fitted paddle contact model (virtual_ball.predict_paddle_
    # contact, e(u_n) restitution) and a coarse RK4 landing rollout; the cached outcome buffers feed
    # the one-shot virtual_* reward terms in hope_rewards.py. No obs change; 175-D contract untouched.
    virtual_ball: bool = False
    # Real PhysX ball/table truth instrument.  It is strictly telemetry-only: no observation,
    # reward or question-bank term reads its state.  Phase-B impulse uses the same fitted venue
    # contact model; substep=1 means no extra aero integration subdivision.
    physical_ball: bool = False
    physical_ball_impulse: bool = False
    physical_ball_substep: int = 1
    # Incoming-ball velocity box (world/env frame, m/s; -x = toward the robot). Kept inside the venue
    # fit's validity envelope (ball speed 1-7 m/s); vertical component ~near-apex-to-descending.
    vb_vel_x_range: tuple[float, float] = (-4.5, -2.0)
    vb_vel_y_range: tuple[float, float] = (-0.6, 0.6)
    vb_vel_z_range: tuple[float, float] = (-1.0, 0.5)
    # Incoming spin: per-axis uniform (rad/s). 50 rad/s ~ 8 rev/s per axis keeps |omega| inside the
    # quaternion-validated 0-15 rev/s envelope.
    vb_spin_abs_max: float = 50.0
    # virtual_spin reward semantics: "topspin" = Ace-style outgoing-topspin generation (ball
    # quality); "minimize" = stage-1 placement-first mode (franco 2026-07-04) — reward CANCELING
    # the incoming spin, kernel exp(-|omega_out|^2 / vb_spin_min_sigma^2) on the outgoing spin
    # magnitude, same legal-landing gate. Sigma in rad/s (10 ~ 1.6 rev/s residual).
    vb_spin_mode: str = "topspin"
    vb_spin_min_sigma: float = 10.0
    # CAPTURE GATE: the virtual contact only evaluates when (a) the racket center is within this
    # distance of the ball (= racket 0.075 + ball 0.020, the v0 real-hit radius) at the exact-strike
    # frame, and (b) the paddle is actively moving INTO the ball along the oriented contact normal
    # faster than vb_min_approach_speed (kills the phantom-block / retreating-racket exploit,
    # verify_tier1 (c)3 — a stationary wall-block scores nothing).
    vb_capture_radius: float = 0.095
    vb_min_approach_speed: float = 0.3
    # Virtual table placement in the env frame. The _hopex clips are HOPE +X aligned with the root at
    # the env origin, so the HOPE convention (robot ~0.5 m behind its table end, centered on the
    # width) puts the near table edge at x = +0.5 and the surface at z = +0.76 above the env origin.
    # Net/far-end/half-width follow from the ITTF table (geometry.py): net at near_x + 1.37 etc.
    vb_table_near_x: float = 0.5
    vb_table_surface_z: float = 0.76
    # Landing target on the opponent half (env frame). Default = P2 half center (near_x + 2.055, 0).
    vb_target_x: float = 2.555
    vb_target_y: float = 0.0
    # Reward shaping constants (read by hope_rewards.virtual_*).
    vb_landing_sigma: float = 0.3     # m — Gaussian width on ||landing_xy - target_xy|| (v0 parity)
    vb_net_margin: float = 0.12      # m — target clearance above the net top (v0 pass_net parity)
    vb_net_sigma: float = 0.10       # m — Gaussian width on the net-clearance error
    vb_spin_ref: float = 250.0       # rad/s (~40 rev/s) — full-credit outgoing topspin (Ace-style)
    vb_min_landing_depth: float = 0.3  # m past the net for the in-bounds bonus (dink guard, verify (c)1)
    # Coarse rollout resolution (verify_tier1 (b): h=10 ms, 1.0 s horizon covers 1-7 m/s shots).
    vb_rollout_h: float = 0.01
    vb_rollout_steps: int = 100

    # --- reachable racket-target workspace (offsets from the env origin, world frame, meters) ---
    # Used only by target_mode="uniform". PLACEHOLDER ranges (not the reference strike point).
    racket_pos_x_range: tuple[float, float] = (0.25, 0.55)
    racket_pos_y_range: tuple[float, float] = (-0.45, 0.45)
    # Unified multi-clip: |y| sampling range; the SIGN is set per clip (forehand on -y, backhand on +y,
    # per forehand_on_negative_y) so forehand/backhand target regions are non-overlapping (HITTER §IV).
    racket_pos_y_abs_range: tuple[float, float] = (0.05, 0.45)
    racket_pos_z_range: tuple[float, float] = (0.70, 1.15)

    # --- desired racket velocity (world frame, m/s) ---
    racket_vel_x_range: tuple[float, float] = (1.5, 4.0)
    racket_vel_y_range: tuple[float, float] = (-1.0, 1.0)
    racket_vel_z_range: tuple[float, float] = (0.0, 1.5)
    # Optional PER-CLIP velocity boxes (uniform mode + unified multi-clip only). None -> use the SHARED
    # racket_vel_*_range above for every clip (BACKWARD COMPATIBLE: old behavior, nothing changes). When
    # set, it is a tuple indexed by clip_id (0=forehand, 1=backhand — same order as strike_phase_per_clip /
    # the command's _clip_names), each entry ((x_lo,x_hi),(y_lo,y_hi),(z_lo,z_hi)). Reason: the forehand and
    # backhand reference clips have DIFFERENT natural strike speeds (~2.6 vs ~2.0 m/s at the racket), so a
    # single shared box overshoots the slower backhand and its strike can never satisfy the velocity gate.
    # Confirmed by the MuJoCo per-clip eval probe: lowering only the backhand target box raised backhand
    # composite 0.32->0.79 (deterministic) / 0.39->0.77 (dither) with forehand byte-identical.
    racket_vel_range_per_clip: tuple | None = None
    # Optional demonstrated/core box for a velocity curriculum. With a planner component it stays
    # one mixture component; without one, its bounds interpolate to the final exported envelope.
    # Defaults preserve every existing task's pure final-box sampling.
    racket_vel_start_range_per_clip: tuple | None = None
    # Optional narrow clip-compatible core. It expands to ``start`` with learned velocity
    # competence and is training-only; final/exported target boxes remain unchanged.
    racket_vel_bootstrap_range_per_clip: tuple | None = None
    # Optional planner-distribution box mixed with the demonstrated/core start box. The final
    # range remains a union safety envelope and is not sampled directly when this is configured.
    racket_vel_planner_range_per_clip: tuple | None = None
    racket_vel_planner_mix_prob: float = 0.0
    racket_vel_planner_mix_by_velocity_stage: bool = False
    racket_vel_stage1_planner_mix_prob: float = 0.25
    racket_vel_range_ramp_steps: int = 0
    # Eval-only escape hatch: None follows env.common_step_counter; [0,1] pins curriculum progress.
    # Training tasks leave this unset. Deterministic checkpoint ranking sets 1.0 before env creation.
    racket_vel_curriculum_progress_override: float | None = None

    # OPTIONAL per-clip racket target-POSITION boxes (uniform mode, unified multi-clip policy). None ->
    # use the shared racket_pos_x_range + |y|-sign + racket_pos_z_range box for every clip (BACKWARD
    # COMPATIBLE: old behavior, nothing changes). When set, it is a tuple indexed by clip_id (0=forehand,
    # 1=backhand — same order as strike_phase_per_clip), each entry ((x_lo,x_hi),(y_lo,y_hi),(z_lo,z_hi))
    # added to the env origin. NOTE the y range is SIGNED here and used directly, so it REPLACES the
    # shared |y|-sign logic. Reason: each clip's strike frame can sit at a different height/depth/lateral
    # offset, so a shared box can make one clip's strike-frame position unreachable. Per-clip boxes let
    # each clip's target track its own reference strike point.
    racket_pos_range_per_clip: tuple | None = None

    # --- HER-style achieved-target replay (uniform mode + unified multi-clip only) -------------------
    # On-policy-compatible hindsight relabeling (Ace/HER, adapted for PPO): true retroactive relabeling is
    # observation-inconsistent here (the target is in the actor obs every step), so instead the NEXT
    # swing's target is drawn, with probability `achieved_target_mix_prob`, from a per-clip ring buffer of
    # racket states the policy ACTUALLY produced at previous exact-strike frames (pos env-origin-relative,
    # vel world). Every replayed target is reachable-by-demonstration — it kills the "the box asks for a
    # point the taught swing never passes through" mismatch without moving the box. Mixture (not pure
    # replay) + jitter + clamping into the per-clip box inflated by `achieved_clamp_inflate` (clamp
    # applies only when per-clip boxes are configured — the unified task always sets them) prevent the
    # target distribution from collapsing onto what the policy already does or drifting far from the
    # training workspace. NOTE the deploy-side target clips must be re-synced to the training boxes
    # whenever the boxes change (they are hand-maintained in pp_policy.hpp / imitate_presets.py).
    # 0.0 = OFF (backward compatible: pure box sampling). TRAIN-ONLY: eval entry points force this to
    # 0.0 so checkpoints are always scored on the pure box distribution. The buffer only fills at
    # exact-strike frames of envs that are still alive, so fallen approaches never contribute targets.
    achieved_target_mix_prob: float = 0.0
    achieved_buffer_size: int = 4096  # per-clip ring buffer capacity (entries)
    achieved_min_fill: int = 256  # replay only once a clip's buffer holds at least this many entries
    achieved_jitter_pos: float = 0.03  # m, uniform per-axis jitter added to a replayed position
    achieved_jitter_vel: float = 0.15  # m/s, uniform per-axis jitter added to a replayed velocity
    achieved_clamp_inflate: float = 0.20  # clamp replayed targets into the per-clip box inflated by this fraction

    # --- desired racket face normal ---
    normal_mode: str = "velocity"  # "velocity" (n = v/|v|) or "sampled"
    racket_normal_x_range: tuple[float, float] = (0.5, 1.0)
    racket_normal_y_range: tuple[float, float] = (-0.3, 0.3)
    racket_normal_z_range: tuple[float, float] = (-0.3, 0.3)

    # --- desired base XY target (offsets from the env origin, world frame, meters) ---
    base_target_x_range: tuple[float, float] = (-0.10, 0.10)
    base_target_y_range: tuple[float, float] = (-0.35, 0.35)
    # HITTER-PURE optional consecutive-rally station sampler.  None preserves the paper-faithful
    # independent absolute-box draw.  When set, TRUE resets still sample base_target_y_range, while
    # clip WRAPS choose a signed step whose magnitude lies in this interval and remains inside the
    # absolute y box.  RallyFinal uses (0.20, 0.35) m: meaningful lateral footwork without OOD jumps.
    station_y_step_range: tuple[float, float] | None = None
    # FinalV3 exogenous wrap mixture.  Defaults preserve every existing task exactly.
    # When enabled, an intra-episode wrap samples: same station with
    # ``station_y_same_prob``; a small adjustment from ``station_y_small_step_range``
    # with ``station_y_small_step_prob``; otherwise the main ``station_y_step_range``.
    # The deployed flat planner must publish an explicit swing side, hence the required
    # station_side_explicit flag; actor observation shape is unchanged.
    station_y_same_prob: float = 0.0
    station_y_small_step_prob: float = 0.0
    station_y_small_step_range: tuple[float, float] | None = None
    # Optional main-step sub-bucket for one measured direction. RallyV11 uses it to place most
    # main-step questions in the +y 19--24 cm band that model_10500 repeatedly completed one serve
    # late. A zero probability preserves the symmetric sampler for every older task.
    station_y_positive_main_prob: float = 0.0
    station_y_positive_main_step_range: tuple[float, float] | None = None
    station_side_explicit: bool = False
    # Deploy-parity readiness thresholds.  They are read-only diagnostics in legacy tasks.  A task
    # that explicitly registers ``ArmDeadlineMiss`` (currently FinalV2Plus only) uses the same single
    # source of truth to make one decision at the exogenous arm deadline: ready -> release; otherwise
    # terminate the missed rally before MotionCommand can advance the swing clock.  This is NOT the
    # old arrival-controlled hold extension and gives the policy no way to stretch the deadline.
    ready_monitor_step_range: tuple[float, float] = (0.20, 0.35)
    ready_monitor_x_thresh: float = 0.10
    ready_monitor_y_thresh: float = 0.10
    ready_monitor_speed_thresh: float = 0.20
    # Defaults below are effectively disabled for existing tasks.  V2Plus explicitly tightens them
    # using only deploy-available IMU/proprioceptive signals.
    ready_monitor_yaw_rate_thresh: float = 1.0e6
    ready_monitor_tilt_thresh: float = 1.0
    ready_monitor_joint_speed_thresh: float = 1.0e6
    ready_monitor_dwell_s: float = 0.12
    ready_monitor_heading_thresh_rad: float = math.pi
    ready_monitor_foot_slip_thresh: float = 1.0e6
    # V17 can make only the Stage-1 acquisition gate learnable, then interpolate every component
    # back to the unchanged deploy READY contract at the final rung.
    ready_acquisition_profile_enabled: bool = False
    ready_acquisition_bootstrap_heading_thresh_rad: float = math.pi
    ready_acquisition_bootstrap_yaw_rate_thresh: float = 1.0e6
    ready_acquisition_bootstrap_tilt_thresh: float = 1.0
    ready_acquisition_bootstrap_joint_speed_thresh: float = 1.0e6
    ready_acquisition_bootstrap_foot_slip_thresh: float = 1.0e6
    ready_acquisition_bootstrap_dwell_ticks: int = 1
    # Sampled READY release. The selected probability equals readiness supervision scale; disabled
    # tasks retain the old fixed external hold exactly.
    ready_monitor_dwell_ticks: int = 0
    ready_release_enabled: bool = False
    ready_release_arm_tts_s: float = 0.30
    ready_release_timeout_s: float = 1.50
    # Fail-closed identity bit for the V2Plus-only termination term.  Setting this alone has no
    # effect; the EnvCfg must also register ArmDeadlineMiss.  Conversely, that term refuses to run
    # unless this bit is true and hold_until_settled is false.
    arm_deadline_gate: bool = False

    # Correlated planner/venue target tuples. These remain latent command-generation data: no
    # incoming-ball field is added to the 110-D actor observation.
    venue_tuple_enabled: bool = False
    venue_tuple_final_mix_prob: float = 0.0
    # Legacy revisions sampled the fast mirror-law tuple online and multiplied its probability by
    # recovery coverage.  V17-r12 instead uses a high-fidelity bank at a fixed balanced
    # probability, independent of every performance/curriculum signal. Private recipes may pin
    # its digest; the public recipe relies on the schema and numerical checks above.
    venue_tuple_mix_mode: str = "recovery_scaled_online_v1"
    venue_tuple_bank_path: str = ""
    venue_tuple_bank_sha256: str = ""
    venue_tuple_bank_receipt_path: str = ""
    venue_tuple_bank_receipt_sha256: str = ""
    venue_tuple_bank_schema_version: int = 0
    venue_tuple_bank_min_rows_per_side: int = 0
    venue_tuple_unconditional_outcomes: bool = False
    venue_tuple_speed_limit_mps: float = 3.5
    venue_tuple_max_resample_attempts: int = 8
    venue_tuple_landing_x_range: tuple[float, float] = (2.17, 3.04)
    venue_tuple_landing_y_range: tuple[float, float] = (-0.50, 0.50)

    # Phase windows used only by command telemetry.  Defaults are the pre-V3 literals, so old task
    # behavior and logged metrics remain unchanged.  FinalV3 aligns them with its longer cyclic
    # ready/recovery windows; reward gates remain independently configured in the reward terms.
    metrics_pre_settle_t_max: float = 0.45
    metrics_post_settle_t_lo: float = 0.20
    metrics_post_settle_t_hi: float = 1.00
    metrics_clearance_t_pre: float = 0.70
    metrics_clearance_t_post: float = 0.20
    metrics_ready_heading_t_lo: float = 0.45
    metrics_ready_heading_t_hi: float = 1.40
    metrics_post_heading_t_lo: float = 0.35
    metrics_post_heading_t_hi: float = 1.80
    # Weak base->racket coupling (UNIFORM mode only). base_couple_blend = fraction of the racket target's
    # sideways (Y) offset that the base target shifts toward; clamped to ±base_couple_max_offset meters.
    # 0.0 = disabled (spawn-only). Conservative because no walking reference exists (it fights leg imitation).
    base_couple_blend: float = 0.0
    base_couple_max_offset: float = 0.20
    # UNIFORM-mode base-target derivation (HITTER §V-B-1 alignment, 2026-07-05):
    #   "blend"           — legacy: spawn + weak Y blend above (BASE-FREE tasks leave this the default).
    #   "reference_reach" — HITTER separate-commands scheme: base_target = racket_target_xy −
    #                       (reference base→racket strike offset, per clip). Standing AT the commanded
    #                       station puts the racket target at the clip's reference reach — the striking
    #                       plane is fixed RELATIVE TO THE COMMANDED BASE (HITTER's "0.4 m in front"),
    #                       and footwork (mostly lateral) is driven by the base channel, not by
    #                       stretching at a deep world point. base_target_*_range then acts as a JITTER
    #                       around the coupled station (widen y to train y-reach diversity).
    # Sim2real: the paired actor obs (base_target_pos_b) is a RELATIVE Δxy in the yaw-heading frame —
    # deployable from mocap base position (300 Hz, position-only) without any absolute world frame; if
    # mocap drops, feeding Δ=0 degrades gracefully to "already at station" (today's BASE-FREE behavior).
    base_couple_mode: str = "blend"

    # --- swing-type convention ---
    forehand_on_negative_y: bool = True  # right arm holds the paddle: target on -Y side -> forehand (+1)
