from __future__ import annotations

import math
import numpy as np
import os
import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    quat_apply,
    quat_error_magnitude,
    quat_from_euler_xyz,
    quat_inv,
    quat_mul,
    sample_uniform,
    yaw_quat,
)

from whole_body_tracking.utils.motion_schema import select_motion_bodies

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class MotionLoader:
    """Loads one or more motion clips into a single concatenated time axis.

    Passing several files (HITTER unified policy: forehand + backhand) concatenates them along the time
    dimension and records per-clip ``seg_start`` / ``seg_len`` so the command can step/wrap/strike within
    one clip ("segment") at a time, selected per-env by swing type. A single file behaves exactly as
    before: one segment spanning the whole motion, ``time_step_total`` unchanged.
    """

    def __init__(
        self,
        motion_file,
        body_indexes: Sequence[int],
        device: str = "cpu",
        articulation_body_count: int | None = None,
    ):
        files = [motion_file] if isinstance(motion_file, str) else list(motion_file)
        assert len(files) >= 1, "MotionLoader needs at least one motion file"
        jp, jv, bp, bq, bl, ba = [], [], [], [], [], []
        seg_lens = []
        self.fps = None
        for f in files:
            assert os.path.isfile(f), f"Invalid file path: {f}"
            data = np.load(f)
            if self.fps is None:
                self.fps = data["fps"]
            jp.append(torch.tensor(data["joint_pos"], dtype=torch.float32, device=device))
            jv.append(torch.tensor(data["joint_vel"], dtype=torch.float32, device=device))
            bp.append(
                torch.tensor(
                    select_motion_bodies(
                        data["body_pos_w"],
                        body_indexes,
                        f,
                        "body_pos_w",
                        articulation_body_count,
                    ),
                    dtype=torch.float32,
                    device=device,
                )
            )
            bq.append(
                torch.tensor(
                    select_motion_bodies(
                        data["body_quat_w"],
                        body_indexes,
                        f,
                        "body_quat_w",
                        articulation_body_count,
                    ),
                    dtype=torch.float32,
                    device=device,
                )
            )
            bl.append(
                torch.tensor(
                    select_motion_bodies(
                        data["body_lin_vel_w"],
                        body_indexes,
                        f,
                        "body_lin_vel_w",
                        articulation_body_count,
                    ),
                    dtype=torch.float32,
                    device=device,
                )
            )
            ba.append(
                torch.tensor(
                    select_motion_bodies(
                        data["body_ang_vel_w"],
                        body_indexes,
                        f,
                        "body_ang_vel_w",
                        articulation_body_count,
                    ),
                    dtype=torch.float32,
                    device=device,
                )
            )
            seg_lens.append(jp[-1].shape[0])
        self.joint_pos = torch.cat(jp, dim=0)
        self.joint_vel = torch.cat(jv, dim=0)
        self._body_pos_w = torch.cat(bp, dim=0)
        self._body_quat_w = torch.cat(bq, dim=0)
        self._body_lin_vel_w = torch.cat(bl, dim=0)
        self._body_ang_vel_w = torch.cat(ba, dim=0)
        self.time_step_total = self.joint_pos.shape[0]
        # Per-clip segment boundaries on the concatenated time axis.
        self.num_segments = len(seg_lens)
        self.seg_len = torch.tensor(seg_lens, dtype=torch.long, device=device)
        self.seg_start = torch.zeros(self.num_segments, dtype=torch.long, device=device)
        if self.num_segments > 1:
            self.seg_start[1:] = torch.cumsum(self.seg_len, dim=0)[:-1]

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self._body_pos_w

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self._body_quat_w

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        return self._body_lin_vel_w

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        return self._body_ang_vel_w


class MotionCommand(CommandTerm):
    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        # Reset-mix sanity (2026-07-24): the three-way split in _resample_command draws ONE
        # uniform u, so any post_swing mass above u=1.0 silently truncates — stand 0.75 +
        # post 0.35 "raised the replay share" while arithmetically keeping it at 0.25.
        # Fail loudly at construction instead of shipping a dead curriculum lever.
        _reset_mass = float(cfg.stand_start_prob) + float(cfg.post_swing_start_prob)
        if _reset_mass > 1.0 + 1e-9:
            raise ValueError(
                f"stand_start_prob ({cfg.stand_start_prob}) + post_swing_start_prob "
                f"({cfg.post_swing_start_prob}) = {_reset_mass:.3f} > 1.0: the overflow mass is "
                "silently truncated by the single-draw reset split. Lower stand_start_prob so the "
                "intended replay share is actually sampled."
            )
        self._post_swing_replay_contract = str(
            getattr(cfg, "post_swing_replay_contract", "legacy_state_v1")
        )
        if self._post_swing_replay_contract not in {
            "legacy_state_v1",
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            raise ValueError(
                "post_swing_replay_contract must be legacy_state_v1, "
                "markov_stratified_v2, or markov_side_phase_severity_v3; got "
                f"{self._post_swing_replay_contract!r}"
            )
        if int(cfg.post_swing_buffer_size) < 1:
            raise ValueError("post_swing_buffer_size must be positive")
        if int(cfg.post_swing_min_fill) < 1:
            raise ValueError("post_swing_min_fill must be positive")
        if int(cfg.post_swing_min_fill) > int(cfg.post_swing_buffer_size):
            raise ValueError(
                "post_swing_min_fill cannot exceed post_swing_buffer_size"
            )
        if self._post_swing_replay_contract in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            if int(cfg.post_swing_capture_phase_bins) < 2:
                raise ValueError(
                    "Markov replay requires at least two capture phase bins "
                    "(one hot state plus clip wrap)"
                )
            if int(cfg.post_swing_min_fill_per_bucket) < 1:
                raise ValueError(
                    "post_swing_min_fill_per_bucket must be positive"
                )
        if self._post_swing_replay_contract == "markov_side_phase_severity_v3":
            if int(cfg.post_swing_capture_severity_bins) != 3:
                raise ValueError(
                    "markov_side_phase_severity_v3 requires exactly three severity "
                    "bins: safe, warning, near-boundary"
                )
            if not (
                0.0
                < float(cfg.post_swing_near_boundary_hard_margin_fraction)
                < float(cfg.post_swing_warning_hard_margin_fraction)
                < 0.5
            ):
                raise ValueError(
                    "post-swing hard-margin thresholds must satisfy "
                    "0 < near_boundary < warning < 0.5"
                )
            if not (
                0.0
                <= float(cfg.post_swing_warning_tilt)
                < float(cfg.post_swing_near_boundary_tilt)
                < 1.0
            ):
                raise ValueError(
                    "post-swing tilt thresholds must satisfy "
                    "0 <= warning < near_boundary < 1"
                )
            if bool(cfg.post_swing_risk_edge_capture) and not (
                0.0
                <= float(cfg.post_swing_risk_capture_min_age_s)
                < float(cfg.post_swing_risk_capture_max_age_s)
            ):
                raise ValueError(
                    "post-swing risk capture ages must satisfy "
                    "0 <= min_age < max_age"
                )
            if bool(cfg.post_swing_failure_adaptive):
                uniform = float(cfg.post_swing_failure_uniform_ratio)
                alpha = float(cfg.post_swing_failure_ema_alpha)
                neighbor = float(
                    cfg.post_swing_failure_phase_neighbor_blend
                )
                if not math.isfinite(uniform) or uniform <= 0.0:
                    raise ValueError(
                        "post_swing_failure_uniform_ratio must be finite and positive"
                    )
                if not math.isfinite(alpha) or not 0.0 < alpha <= 1.0:
                    raise ValueError(
                        "post_swing_failure_ema_alpha must lie in (0, 1]"
                    )
                if not math.isfinite(neighbor) or not 0.0 <= neighbor <= 0.5:
                    raise ValueError(
                        "post_swing_failure_phase_neighbor_blend must lie in [0, 0.5]"
                    )
        elif bool(cfg.post_swing_failure_adaptive):
            raise ValueError(
                "post_swing_failure_adaptive requires markov_side_phase_severity_v3"
            )
        self._post_swing_ability_gate_enabled = bool(
            cfg.post_swing_ability_gate_enabled
        )
        replay_ramp = tuple(
            float(value) for value in cfg.post_swing_replay_ramp_probabilities
        )
        if self._post_swing_ability_gate_enabled:
            if (
                not replay_ramp
                or any(not math.isfinite(value) for value in replay_ramp)
                or any(value <= 0.0 for value in replay_ramp)
                or any(
                    current < previous
                    for previous, current in zip(replay_ramp, replay_ramp[1:])
                )
                or replay_ramp[-1] > float(cfg.post_swing_start_prob) + 1.0e-9
            ):
                raise ValueError(
                    "ability-gated replay probabilities must be finite, positive, "
                    "monotonic, and no larger than post_swing_start_prob"
                )
            if int(cfg.post_swing_replay_ramp_interval_steps) < 1:
                raise ValueError(
                    "post_swing_replay_ramp_interval_steps must be positive"
                )
            if self._post_swing_replay_contract != "markov_side_phase_severity_v3":
                raise ValueError(
                    "post_swing_ability_gate_enabled requires "
                    "markov_side_phase_severity_v3"
                )
        self._post_swing_replay_ramp_probabilities = replay_ramp
        handoff_probability = float(cfg.runtime_handoff_start_prob)
        if not 0.0 <= handoff_probability <= float(cfg.stand_start_prob):
            raise ValueError(
                "runtime_handoff_start_prob must be a subset of stand_start_prob; "
                f"got {handoff_probability} > {cfg.stand_start_prob}"
            )
        handoff_lo, handoff_hi = (
            int(value) for value in cfg.runtime_handoff_hold_steps_range
        )
        if handoff_lo < 0 or handoff_hi < handoff_lo:
            raise ValueError(
                "runtime_handoff_hold_steps_range must satisfy 0 <= lo <= hi"
            )

        self.robot: Articulation = env.scene[cfg.asset_name]
        self.robot_anchor_body_index = self.robot.body_names.index(self.cfg.anchor_body_name)
        self.motion_anchor_body_index = self.cfg.body_names.index(self.cfg.anchor_body_name)
        self.body_indexes = torch.tensor(
            self.robot.find_bodies(self.cfg.body_names, preserve_order=True)[0], dtype=torch.long, device=self.device
        )

        self.motion = MotionLoader(
            self.cfg.motion_file,
            self.body_indexes,
            device=self.device,
            articulation_body_count=len(self.robot.body_names),
        )
        # GROUNDING preflight (2026-07-03): the actor obs consumes the RAW clip-world anchor quat,
        # and the racket-target boxes are planned in the +X-grounded frame — a clip that was never
        # re-grounded (frame-0 anchor yaw far from 0, e.g. registry v4 at ~+84 deg) trains a
        # TURN-AND-WALK policy whose footwork is undeployable without real base localization
        # (the 2026-07-03 model_9000 backward-jump lesson). Warn loudly; do not silently train.
        for _c in range(self.motion.num_segments):
            _q0 = self.motion.body_quat_w[int(self.motion.seg_start[_c]), self.motion_anchor_body_index]
            _w, _x, _y, _z = (float(_q0[0]), float(_q0[1]), float(_q0[2]), float(_q0[3]))
            _yaw0 = math.degrees(math.atan2(2.0 * (_w * _z + _x * _y), 1.0 - 2.0 * (_y * _y + _z * _z)))
            if abs(_yaw0) > 10.0:
                print(
                    f"[MotionCommand WARN] clip {_c} frame-0 anchor yaw = {_yaw0:+.1f} deg — this clip "
                    "was NOT re-grounded to +X (scripts/reground_hope_frame.py). Target boxes assume "
                    "+X grounding; training on it produces a turn-and-walk policy that needs "
                    "oracle/mocap localization at deploy. Pin registry_name to the re-grounded "
                    "lineage (hopex/v3) or re-ground and re-upload before training.",
                    flush=True,
                )
        self.time_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # Unified multi-clip (HITTER forehand+backhand) support. With one clip these are inert and the
        # behaviour below is byte-identical to the single-clip path. clip_id[env] selects which segment
        # (swing type) the env is currently imitating.
        self._multiseg = self.motion.num_segments > 1
        self.clip_id = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # Robust per-step "this env just wrapped to a new swing" signal, consumed by the racket-target
        # command to resample its target. Replaces a time_steps<prev heuristic that fails when a clip
        # wrap jumps the index to a HIGHER segment start (forehand->backhand on the concatenated axis).
        self.just_resampled = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        # Pre-swing hold state (see cfg.hold_steps_range): while hold_counter > 0 the reference
        # clock is frozen at the swing's first frame ("waiting for the ball"). _update_command
        # decrements it. in_hold is exposed for rewards/metrics.
        self.hold_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # True only while _resample_command is being invoked from an intra-episode clip WRAP
        # (as opposed to a true episode reset) — wraps skip the RSI teleport (cfg.wrap_teleport).
        self._resampling_from_wrap = False
        # A8: post-swing initial-state ring buffer (root state stored ORIGIN-RELATIVE in [:3]).
        # V17 r2 restores only an environment's own snapshots so startup-randomized plant
        # parameters remain identical; older replay contracts may still use cross-env entries.
        # Tensors are allocated lazily at first capture (dof count comes from live robot data).
        self._post_swing_root: torch.Tensor | None = None
        self._post_swing_joint_pos: torch.Tensor | None = None
        self._post_swing_joint_vel: torch.Tensor | None = None
        self._post_swing_count = 0
        self._post_swing_ptr = 0
        # V17 Markov replay. Allocated lazily at first capture because action/joint dimensions
        # resolve only after every manager has been constructed. Capacity is exactly balanced
        # across source-clip x phase buckets; no bucket can crowd another out of the ring.
        self._v17_replay_root: torch.Tensor | None = None
        self._v17_replay_joint_pos: torch.Tensor | None = None
        self._v17_replay_joint_vel: torch.Tensor | None = None
        self._v17_replay_manager_action: torch.Tensor | None = None
        self._v17_replay_manager_prev_action: torch.Tensor | None = None
        self._v17_replay_action_raw: torch.Tensor | None = None
        self._v17_replay_action_applied_raw: torch.Tensor | None = None
        self._v17_replay_action_unclamped_qdes: torch.Tensor | None = None
        self._v17_replay_action_processed_qdes: torch.Tensor | None = None
        self._v17_replay_action_commanded_qdes: torch.Tensor | None = None
        self._v17_replay_action_executed_qdes: torch.Tensor | None = None
        self._v17_replay_action_previous_executed_qdes: (
            torch.Tensor | None
        ) = None
        self._v17_replay_action_qdes_delta: torch.Tensor | None = None
        self._v17_replay_action_previous_qdes_delta: (
            torch.Tensor | None
        ) = None
        self._v17_replay_action_qdes_second_difference: (
            torch.Tensor | None
        ) = None
        self._v17_replay_action_decoder_offset: torch.Tensor | None = None
        self._v17_replay_action_decoder_scale: torch.Tensor | None = None
        self._v17_replay_action_delay_queue: torch.Tensor | None = None
        self._v17_replay_action_delay_steps: torch.Tensor | None = None
        self._v17_replay_source_env: torch.Tensor | None = None
        self._v17_replay_local_latest_slot: torch.Tensor | None = None
        self._v17_replay_joint_stiffness: torch.Tensor | None = None
        self._v17_replay_joint_damping: torch.Tensor | None = None
        self._v17_replay_motion_time_steps: torch.Tensor | None = None
        self._v17_replay_motion_clip_id: torch.Tensor | None = None
        self._v17_replay_motion_hold_counter: torch.Tensor | None = None
        self._v17_replay_target_state: dict[str, torch.Tensor] = {}
        self._v17_replay_pending_target_state: dict[str, torch.Tensor] = {}
        self._v17_replay_pending_target_mask = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._v17_replay_bucket_count: torch.Tensor | None = None
        self._v17_replay_bucket_ptr: torch.Tensor | None = None
        self._v17_replay_bucket_capacity = 0
        self._v17_replay_total_count = 0
        # Historical tasks may opt into the generic gate below. HitterPingPong deliberately
        # leaves it off: V14 recovery capture starts immediately and is independent of the
        # racket command's mocap-corruption curriculum.
        self._post_swing_ability_unlocked = not self._post_swing_ability_gate_enabled
        self._post_swing_replay_ready_step = -1
        self._v17_replay_last_sample_bucket = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        # Diagnostic provenance for the most recent replay reset. This is not part of the
        # Markov state and does not affect sampling; it lets runtime verification prove that
        # every restored tensor came from the same ring-buffer slot.
        self._v17_replay_last_sample_slot = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        replay_bucket_count = self._v17_replay_bucket_total()
        self._v17_replay_failure_score = torch.zeros(
            replay_bucket_count, dtype=torch.float, device=self.device
        )
        self._v17_replay_pending_failure_count = torch.zeros_like(
            self._v17_replay_failure_score
        )
        self._v17_replay_sampling_probability = torch.full_like(
            self._v17_replay_failure_score,
            1.0 / float(max(replay_bucket_count, 1)),
        )
        self.runtime_handoff_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        # True only while a replay-reset env is in its fixed exogenous recovery hold. This latch
        # lets V17 avoid applying the clip-relative foot-z termination to a deliberately off-clip
        # Markov state; physical tilt/height guards remain active.
        self.post_swing_replay_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._recovery_curriculum_scale = (
            0.0 if bool(cfg.post_swing_curriculum_scaled) else 1.0
        )
        # World-frame root position actually WRITTEN by the most recent true reset of each env
        # (stand / post-swing replay / RSI).  Robot data buffers can be stale inside the reset
        # callback chain, so downstream consumers (the V15 finite-gait planner) read this instead
        # of assuming every reset lands at origin + nominal offset — the 2026-07-23 audit found the
        # 25% post-swing replay resets received gait commands planned from the wrong start point.
        self.last_reset_root_pos_w = (
            env.scene.env_origins.clone()
            + self.robot.data.default_root_state[:, :3].clone()
        )
        self.body_pos_relative_w = torch.zeros(self.num_envs, len(cfg.body_names), 3, device=self.device)
        self.body_quat_relative_w = torch.zeros(self.num_envs, len(cfg.body_names), 4, device=self.device)
        self.body_quat_relative_w[:, :, 0] = 1.0

        self.bin_count = int(self.motion.time_step_total // (1 / (env.cfg.decimation * env.cfg.sim.dt))) + 1
        self.bin_failed_count = torch.zeros(self.bin_count, dtype=torch.float, device=self.device)
        self._current_bin_failed = torch.zeros(self.bin_count, dtype=torch.float, device=self.device)
        self.kernel = torch.tensor(
            [self.cfg.adaptive_lambda**i for i in range(self.cfg.adaptive_kernel_size)], device=self.device
        )
        self.kernel = self.kernel / self.kernel.sum()

        self.metrics["error_anchor_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_rot"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_lin_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_ang_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_rot"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_entropy"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_top1_prob"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_top1_bin"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["post_swing_recovery_scale"] = torch.full(
            (self.num_envs,),
            float(self._recovery_curriculum_scale),
            device=self.device,
        )
        self.metrics["post_swing_replay_probability_effective"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_replay_buffer_fill"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_replay_eligible_buckets"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_replay_reset"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_replay_bucket"] = torch.full(
            (self.num_envs,), -1.0, device=self.device
        )
        self.metrics["post_swing_replay_ready"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_ability_unlocked"] = torch.full(
            (self.num_envs,),
            float(self._post_swing_ability_unlocked),
            device=self.device,
        )
        self.metrics["post_swing_capture_enabled"] = torch.full(
            (self.num_envs,),
            float(self._post_swing_ability_unlocked),
            device=self.device,
        )
        self.metrics["post_swing_replay_ramp_stage"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_replay_local_coverage"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_failure_sampling_entropy"] = torch.ones(
            self.num_envs, device=self.device
        )
        self.metrics["post_swing_failure_sampling_top1_prob"] = torch.full(
            (self.num_envs,),
            1.0 / float(max(replay_bucket_count, 1)),
            device=self.device,
        )
        self.metrics["post_swing_failure_sampling_top1_bucket"] = torch.zeros(
            self.num_envs, device=self.device
        )
        if (
            self._post_swing_replay_contract
            == "markov_side_phase_severity_v3"
        ):
            for bucket in range(self._v17_replay_bucket_total()):
                label = self._v17_replay_bucket_label(bucket)
                self.metrics[f"post_swing_replay_count_{label}"] = (
                    torch.zeros(self.num_envs, device=self.device)
                )
                self.metrics[f"post_swing_replay_fill_{label}"] = (
                    torch.zeros(self.num_envs, device=self.device)
                )
        self.metrics["runtime_handoff_reset"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["runtime_handoff_active"] = torch.zeros(
            self.num_envs, device=self.device
        )
        self.metrics["motion_phase"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["in_hold"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_rot_deg"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_pos_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_pos_max_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_vel_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_vel_max_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["reference_anchor_speed"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["robot_anchor_speed"] = torch.zeros(self.num_envs, device=self.device)
        for axis in ("x", "y", "z"):
            self.metrics[f"reference_anchor_pos_{axis}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"robot_anchor_pos_{axis}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"reference_anchor_lin_vel_{axis}"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"robot_anchor_lin_vel_{axis}"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:  # TODO Consider again if this is the best observation
        return torch.cat([self.joint_pos, self.joint_vel], dim=1)

    @property
    def in_hold(self) -> torch.Tensor:
        """Bool mask: env is in the pre-swing hold (reference frozen at the swing's first frame)."""
        return self.hold_counter > 0

    @property
    def joint_pos(self) -> torch.Tensor:
        # HOLD imitates the READY STAND, not the windup crouch (2026-07-05, pragmatic
        # P2.0): clip frame 0 is an asymmetric mid-crouch (knee 0.62/0.52 vs stand 0.25,
        # left hip_roll +0.14) — imitating it all hold long produced the splayed-feet
        # crouch-stand seen in Gate 2.5/3. During hold the joint reference is the
        # default stand pose; the release (stand -> windup) is exactly the trained
        # stand_start transition. C++ mirrors this (pp_policy: refs.joint_pos =
        # default_q at level 0) — keep them in lockstep.
        jp = self.motion.joint_pos[self.time_steps]
        dq = self.robot.data.default_joint_pos
        return torch.where(self.in_hold[:, None], dq, jp)

    @property
    def joint_vel(self) -> torch.Tensor:
        # HOLD = a STATIONARY reference (2026-07-05): clip frame 0 is a mid-crouch
        # TRANSIENT (knee +7.8 rad/s, torso -1.11 m/s DOWN in the hopex clips). Feeding
        # its raw velocities through the whole hold taught the policy to fight a phantom
        # squat at soft gains and made "sink slowly" the velocity-reward optimum — the
        # AGI-sim / hardware bare-hold fall (Gate 2.5 P2, 3-5 s tip). A frozen reference
        # is not moving: zero its velocities on held envs. The C++ runner mirrors this
        # (pp_policy zeroes refs.joint_vel in its hold states) — keep them in lockstep.
        jv = self.motion.joint_vel[self.time_steps]
        return torch.where(self.in_hold[:, None], torch.zeros_like(jv), jv)

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self.motion.body_pos_w[self.time_steps] + self._env.scene.env_origins[:, None, :]

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self.motion.body_quat_w[self.time_steps]

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        # Zeroed during hold — see joint_vel. Un-gated motion_body_lin_vel otherwise
        # pays for tracking frame-0's -1.11 m/s DOWNWARD torso velocity all hold long.
        v = self.motion.body_lin_vel_w[self.time_steps]
        return torch.where(self.in_hold[:, None, None], torch.zeros_like(v), v)

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        v = self.motion.body_ang_vel_w[self.time_steps]
        return torch.where(self.in_hold[:, None, None], torch.zeros_like(v), v)

    @property
    def anchor_pos_w(self) -> torch.Tensor:
        return self.motion.body_pos_w[self.time_steps, self.motion_anchor_body_index] + self._env.scene.env_origins

    @property
    def anchor_quat_w(self) -> torch.Tensor:
        return self.motion.body_quat_w[self.time_steps, self.motion_anchor_body_index]

    @property
    def anchor_lin_vel_w(self) -> torch.Tensor:
        return self.motion.body_lin_vel_w[self.time_steps, self.motion_anchor_body_index]

    @property
    def anchor_ang_vel_w(self) -> torch.Tensor:
        return self.motion.body_ang_vel_w[self.time_steps, self.motion_anchor_body_index]

    @property
    def robot_joint_pos(self) -> torch.Tensor:
        return self.robot.data.joint_pos

    @property
    def robot_joint_vel(self) -> torch.Tensor:
        return self.robot.data.joint_vel

    @property
    def robot_body_pos_w(self) -> torch.Tensor:
        return self.robot.data.body_pos_w[:, self.body_indexes]

    @property
    def robot_body_quat_w(self) -> torch.Tensor:
        return self.robot.data.body_quat_w[:, self.body_indexes]

    @property
    def robot_body_lin_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_lin_vel_w[:, self.body_indexes]

    @property
    def robot_body_ang_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_ang_vel_w[:, self.body_indexes]

    @property
    def robot_anchor_pos_w(self) -> torch.Tensor:
        return self.robot.data.body_pos_w[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_quat_w(self) -> torch.Tensor:
        return self.robot.data.body_quat_w[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_lin_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_lin_vel_w[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_ang_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_ang_vel_w[:, self.robot_anchor_body_index]

    def _update_metrics(self):
        anchor_pos_err = self.anchor_pos_w - self.robot_anchor_pos_w
        anchor_rot_err = quat_error_magnitude(self.anchor_quat_w, self.robot_anchor_quat_w)
        anchor_lin_vel_err = self.anchor_lin_vel_w - self.robot_anchor_lin_vel_w
        anchor_ang_vel_err = self.anchor_ang_vel_w - self.robot_anchor_ang_vel_w

        self.metrics["error_anchor_pos"] = torch.norm(anchor_pos_err, dim=-1)
        self.metrics["error_anchor_rot"] = anchor_rot_err
        self.metrics["error_anchor_lin_vel"] = torch.norm(anchor_lin_vel_err, dim=-1)
        self.metrics["error_anchor_ang_vel"] = torch.norm(anchor_ang_vel_err, dim=-1)
        self.metrics["error_anchor_rot_deg"] = anchor_rot_err * (180.0 / math.pi)

        self.metrics["error_body_pos"] = torch.norm(self.body_pos_relative_w - self.robot_body_pos_w, dim=-1).mean(
            dim=-1
        )
        self.metrics["error_body_rot"] = quat_error_magnitude(self.body_quat_relative_w, self.robot_body_quat_w).mean(
            dim=-1
        )

        self.metrics["error_body_lin_vel"] = torch.norm(self.body_lin_vel_w - self.robot_body_lin_vel_w, dim=-1).mean(
            dim=-1
        )
        self.metrics["error_body_ang_vel"] = torch.norm(self.body_ang_vel_w - self.robot_body_ang_vel_w, dim=-1).mean(
            dim=-1
        )

        joint_pos_err = self.joint_pos - self.robot_joint_pos
        joint_vel_err = self.joint_vel - self.robot_joint_vel
        self.metrics["error_joint_pos"] = torch.norm(joint_pos_err, dim=-1)
        self.metrics["error_joint_vel"] = torch.norm(joint_vel_err, dim=-1)
        self.metrics["error_joint_pos_mean_abs"] = torch.mean(torch.abs(joint_pos_err), dim=-1)
        self.metrics["error_joint_pos_max_abs"] = torch.max(torch.abs(joint_pos_err), dim=-1).values
        self.metrics["error_joint_vel_mean_abs"] = torch.mean(torch.abs(joint_vel_err), dim=-1)
        self.metrics["error_joint_vel_max_abs"] = torch.max(torch.abs(joint_vel_err), dim=-1).values

        # Log anchor states in an env-origin-relative frame so cross-env averages remain meaningful.
        anchor_ref_rel = self.anchor_pos_w - self._env.scene.env_origins
        anchor_robot_rel = self.robot_anchor_pos_w - self._env.scene.env_origins
        for axis_idx, axis in enumerate(("x", "y", "z")):
            self.metrics[f"reference_anchor_pos_{axis}"].copy_(
                anchor_ref_rel[:, axis_idx]
            )
            self.metrics[f"robot_anchor_pos_{axis}"].copy_(
                anchor_robot_rel[:, axis_idx]
            )
            self.metrics[f"reference_anchor_lin_vel_{axis}"].copy_(
                self.anchor_lin_vel_w[:, axis_idx]
            )
            # robot_anchor_lin_vel_w is a mutable view into Isaac's body-state cache.
            # CommandTerm.reset zeros metrics in-place, so assigning the view here would zero
            # actor/critic physical state during reset.
            self.metrics[f"robot_anchor_lin_vel_{axis}"].copy_(
                self.robot_anchor_lin_vel_w[:, axis_idx]
            )

        self.metrics["reference_anchor_speed"] = torch.norm(self.anchor_lin_vel_w, dim=-1)
        self.metrics["robot_anchor_speed"] = torch.norm(self.robot_anchor_lin_vel_w, dim=-1)
        if self._multiseg:
            seg_start = self.motion.seg_start[self.clip_id]
            seg_len = self.motion.seg_len[self.clip_id].clamp(min=2)
            self.metrics["motion_phase"] = (self.time_steps - seg_start).float() / (seg_len - 1).float()
        else:
            self.metrics["motion_phase"] = self.time_steps.float() / max(self.motion.time_step_total - 1, 1)

        # CommandTerm.reset zeros metrics in-place. Republish the ability/replay state every
        # control step so a fresh all-zero bank remains explicit instead of looking missing.
        self._update_v17_replay_metrics()

    def _adaptive_sampling(self, env_ids: Sequence[int]):
        if self._multiseg:
            # HITTER unified policy: each new swing uniformly samples the swing TYPE (clip) and starts at
            # that clip's first frame (reference-state-init at the swing start). The adaptive failure-bin
            # curriculum is single-clip BeyondMimic machinery and is bypassed here.
            n = len(env_ids)
            if n > 0:
                eval_sequence = tuple(getattr(self.cfg, "eval_clip_sequence", ()) or ())
                if eval_sequence:
                    if any(int(value) < 0 or int(value) >= self.motion.num_segments for value in eval_sequence):
                        raise ValueError(
                            f"eval_clip_sequence contains invalid clip id: {eval_sequence}"
                        )
                    if not hasattr(self, "_eval_sequence_counter"):
                        self._eval_sequence_counter = (
                            torch.arange(self.num_envs, device=self.device, dtype=torch.long)
                            % len(eval_sequence)
                        )
                        self.eval_sequence_index = torch.zeros(
                            self.num_envs, device=self.device, dtype=torch.long
                        )
                    sequence = torch.as_tensor(eval_sequence, device=self.device, dtype=torch.long)
                    sequence_index = self._eval_sequence_counter[env_ids] % len(eval_sequence)
                    new_clip = sequence[sequence_index]
                    self.eval_sequence_index[env_ids] = sequence_index
                    self._eval_sequence_counter[env_ids] += 1
                else:
                    new_clip = torch.randint(0, self.motion.num_segments, (n,), device=self.device)
                fixed_fraction = float(
                    getattr(self.cfg, "fixed_clip_env_fraction_per_clip", 0.0)
                )
                if fixed_fraction > 0.0:
                    if fixed_fraction * self.motion.num_segments > 1.0 + 1.0e-12:
                        raise ValueError(
                            "fixed_clip_env_fraction_per_clip * num_segments must be <= 1"
                        )
                    quota = int(round(fixed_fraction * self.num_envs))
                    env_ids_t = torch.as_tensor(
                        env_ids, dtype=torch.long, device=self.device
                    )
                    for clip_id in range(self.motion.num_segments):
                        start = clip_id * quota
                        stop = start + quota
                        permanent = (env_ids_t >= start) & (env_ids_t < stop)
                        new_clip[permanent] = clip_id
                self.clip_id[env_ids] = new_clip
                self.time_steps[env_ids] = self.motion.seg_start[new_clip]
            # Report the REAL clip-sampling distribution (repurpose the bin-sampling metrics for clips):
            # entropy of the per-clip env fraction (1.0 = balanced), and the most-sampled clip + its share.
            counts = torch.bincount(self.clip_id, minlength=self.motion.num_segments).float()
            probs = counts / counts.sum().clamp(min=1.0)
            H = -(probs * (probs + 1e-12).log()).sum()
            self.metrics["sampling_entropy"][:] = H / math.log(max(self.motion.num_segments, 2))
            pmax, imax = probs.max(dim=0)
            self.metrics["sampling_top1_prob"][:] = pmax
            self.metrics["sampling_top1_bin"][:] = imax.float() / max(self.motion.num_segments, 1)
            return
        episode_failed = self._env.termination_manager.terminated[env_ids]
        if torch.any(episode_failed):
            current_bin_index = torch.clamp(
                (self.time_steps * self.bin_count) // max(self.motion.time_step_total, 1), 0, self.bin_count - 1
            )
            fail_bins = current_bin_index[env_ids][episode_failed]
            self._current_bin_failed[:] = torch.bincount(fail_bins, minlength=self.bin_count)

        # Sample
        sampling_probabilities = self.bin_failed_count + self.cfg.adaptive_uniform_ratio / float(self.bin_count)
        sampling_probabilities = torch.nn.functional.pad(
            sampling_probabilities.unsqueeze(0).unsqueeze(0),
            (0, self.cfg.adaptive_kernel_size - 1),  # Non-causal kernel
            mode="replicate",
        )
        sampling_probabilities = torch.nn.functional.conv1d(sampling_probabilities, self.kernel.view(1, 1, -1)).view(-1)

        sampling_probabilities = sampling_probabilities / sampling_probabilities.sum()

        sampled_bins = torch.multinomial(sampling_probabilities, len(env_ids), replacement=True)

        self.time_steps[env_ids] = (
            (sampled_bins + sample_uniform(0.0, 1.0, (len(env_ids),), device=self.device))
            / self.bin_count
            * (self.motion.time_step_total - 1)
        ).long()

        # Metrics
        H = -(sampling_probabilities * (sampling_probabilities + 1e-12).log()).sum()
        H_norm = H / math.log(self.bin_count)
        pmax, imax = sampling_probabilities.max(dim=0)
        self.metrics["sampling_entropy"][:] = H_norm
        self.metrics["sampling_top1_prob"][:] = pmax
        self.metrics["sampling_top1_bin"][:] = imax.float() / self.bin_count

    @property
    def recovery_curriculum_scale(self) -> float:
        """Current bounded scale used by V17 replay/recovery terms."""

        return float(self._recovery_curriculum_scale)

    def set_recovery_curriculum_scale(self, scale: float) -> None:
        """Set the live V17 scale, failing closed on a non-finite/out-of-range value.

        This deliberately does not clear replay storage or touch policy state.  A regression only
        reduces the probability/difficulty of future recovery samples; already collected Markov
        snapshots remain available if strike competence later recovers.
        """

        scale = float(scale)
        if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
            raise ValueError(
                f"recovery curriculum scale must be finite and in [0, 1], got {scale}"
            )
        self._recovery_curriculum_scale = scale
        self.metrics["post_swing_recovery_scale"][:] = scale

    def _v17_replay_bucket_total(self) -> int:
        severity_bins = (
            int(self.cfg.post_swing_capture_severity_bins)
            if self._post_swing_replay_contract
            == "markov_side_phase_severity_v3"
            else 1
        )
        return (
            int(self.motion.num_segments)
            * int(self.cfg.post_swing_capture_phase_bins)
            * severity_bins
        )

    def _v17_replay_bucket_label(self, bucket: int) -> str:
        """Return one stable, readable side/phase/severity label for telemetry."""

        phase_bins = int(self.cfg.post_swing_capture_phase_bins)
        severity_bins = int(self.cfg.post_swing_capture_severity_bins)
        clip, remainder = divmod(int(bucket), phase_bins * severity_bins)
        phase, severity = divmod(remainder, severity_bins)
        if int(self.motion.num_segments) == 2:
            side = ("fh", "bh")[clip]
        else:
            side = f"clip{clip}"
        phase_names = ("p008", "p030", "p080", "wrap")
        phase_name = (
            phase_names[phase]
            if phase < len(phase_names)
            else f"phase{phase}"
        )
        severity_names = ("safe", "warning", "near")
        severity_name = (
            severity_names[severity]
            if severity < len(severity_names)
            else f"severity{severity}"
        )
        return f"{side}_{phase_name}_{severity_name}"

    def _ensure_v17_replay_storage(self) -> None:
        if self._v17_replay_root is not None:
            return
        phase_bins = int(self.cfg.post_swing_capture_phase_bins)
        severity_bins = (
            int(self.cfg.post_swing_capture_severity_bins)
            if self._post_swing_replay_contract
            == "markov_side_phase_severity_v3"
            else 1
        )
        bucket_count = int(self.motion.num_segments) * phase_bins * severity_bins
        total_capacity = int(self.cfg.post_swing_buffer_size)
        if total_capacity % bucket_count != 0:
            raise ValueError(
                "Markov replay requires post_swing_buffer_size to be divisible by "
                f"num_clips*phase_bins*severity_bins ({bucket_count}), got "
                f"{total_capacity}"
            )
        capacity = total_capacity // bucket_count
        if capacity < int(self.cfg.post_swing_min_fill_per_bucket):
            raise ValueError(
                "post_swing buffer capacity per bucket is smaller than "
                "post_swing_min_fill_per_bucket"
            )
        action_manager = self._env.action_manager
        action_term = action_manager.get_term(str(self.cfg.post_swing_action_name))
        capture = getattr(action_term, "capture_markov_replay_state", None)
        restore = getattr(action_term, "restore_markov_replay_state", None)
        if not callable(capture) or not callable(restore):
            raise RuntimeError(
                "Markov replay requires an action term exposing "
                "capture_markov_replay_state/restore_markov_replay_state"
            )
        if (
            len(action_manager.active_terms) != 1
            or int(action_manager.action.shape[1]) != int(action_term.action_dim)
        ):
            raise RuntimeError(
                "Markov replay currently requires exactly one action term so "
                "manager action history can be decoder-remapped without ambiguity"
            )
        joint_dim = int(self.robot.data.joint_pos.shape[1])
        manager_action_dim = int(action_manager.action.shape[1])
        term_action_dim = int(action_term.action_dim)
        shape = (bucket_count, capacity)
        self._v17_replay_root = torch.zeros(*shape, 13, device=self.device)
        self._v17_replay_joint_pos = torch.zeros(
            *shape, joint_dim, device=self.device
        )
        self._v17_replay_joint_vel = torch.zeros_like(
            self._v17_replay_joint_pos
        )
        self._v17_replay_manager_action = torch.zeros(
            *shape, manager_action_dim, device=self.device
        )
        self._v17_replay_manager_prev_action = torch.zeros_like(
            self._v17_replay_manager_action
        )
        self._v17_replay_action_raw = torch.zeros(
            *shape, term_action_dim, device=self.device
        )
        self._v17_replay_action_applied_raw = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_unclamped_qdes = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_processed_qdes = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_commanded_qdes = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_executed_qdes = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_previous_executed_qdes = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_qdes_delta = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_previous_qdes_delta = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_qdes_second_difference = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_decoder_offset = torch.zeros_like(
            self._v17_replay_action_raw
        )
        self._v17_replay_action_decoder_scale = torch.zeros_like(
            self._v17_replay_action_raw
        )
        delay_depth = int(
            getattr(action_term, "_qdes_delay_max_steps", 0)
        ) + 1
        self._v17_replay_action_delay_queue = torch.zeros(
            *shape,
            delay_depth,
            term_action_dim,
            device=self.device,
        )
        self._v17_replay_action_delay_steps = torch.zeros(
            shape, dtype=torch.long, device=self.device
        )
        self._v17_replay_source_env = torch.full(
            shape, -1, dtype=torch.long, device=self.device
        )
        # The V17 r2 plant randomizers for friction, mass, CoM, and default-q run at startup,
        # therefore replaying into the SAME environment preserves them exactly.  Reset-time PD
        # gains are the only plant latent that must be snapshotted explicitly.
        self._v17_replay_local_latest_slot = torch.full(
            (bucket_count, self.num_envs),
            -1,
            dtype=torch.long,
            device=self.device,
        )
        self._v17_replay_joint_stiffness = torch.zeros(
            *shape, joint_dim, device=self.device
        )
        self._v17_replay_joint_damping = torch.zeros_like(
            self._v17_replay_joint_stiffness
        )
        self._v17_replay_motion_time_steps = torch.zeros(
            shape, dtype=torch.long, device=self.device
        )
        self._v17_replay_motion_clip_id = torch.zeros_like(
            self._v17_replay_motion_time_steps
        )
        self._v17_replay_motion_hold_counter = torch.zeros_like(
            self._v17_replay_motion_time_steps
        )
        self._v17_replay_bucket_count = torch.zeros(
            bucket_count, dtype=torch.long, device=self.device
        )
        self._v17_replay_bucket_ptr = torch.zeros_like(
            self._v17_replay_bucket_count
        )
        self._v17_replay_bucket_capacity = capacity

    def _v17_target_term(self):
        try:
            target_term = self._env.command_manager.get_term(
                "racket_target"
            )
        except (AttributeError, ValueError) as exc:
            raise RuntimeError(
                "V17 Markov replay requires the racket_target command term"
            ) from exc
        if not (
            callable(
                getattr(
                    target_term, "capture_markov_replay_state", None
                )
            )
            and callable(
                getattr(
                    target_term, "restore_markov_replay_state", None
                )
            )
        ):
            raise RuntimeError(
                "V17 Markov replay requires racket_target "
                "capture/restore_markov_replay_state"
            )
        return target_term

    def _store_v17_target_state(
        self,
        bucket: int,
        slots: torch.Tensor,
        state: dict[str, torch.Tensor],
    ) -> None:
        """Store a shape-checked target/READY/recovery state dictionary."""

        count = int(slots.numel())
        prefix = (
            int(self._v17_replay_bucket_count.numel()),
            int(self._v17_replay_bucket_capacity),
        )
        for name, value in state.items():
            if not torch.is_tensor(value) or value.shape[0] != count:
                raise RuntimeError(
                    f"V17 target replay field {name!r} must be a tensor "
                    f"with leading size {count}"
                )
            destination = self._v17_replay_target_state.get(name)
            expected_shape = prefix + tuple(value.shape[1:])
            if destination is None:
                destination = torch.zeros(
                    expected_shape,
                    dtype=value.dtype,
                    device=self.device,
                )
                self._v17_replay_target_state[name] = destination
            elif (
                tuple(destination.shape) != expected_shape
                or destination.dtype != value.dtype
            ):
                raise RuntimeError(
                    f"V17 target replay field {name!r} changed contract: "
                    f"{tuple(destination.shape)}/{destination.dtype} vs "
                    f"{expected_shape}/{value.dtype}"
                )
            destination[bucket, slots] = value

    def _v17_local_replay_available(
        self, env_ids: torch.Tensor
    ) -> torch.Tensor:
        if (
            self._v17_replay_local_latest_slot is None
            or len(env_ids) == 0
        ):
            return torch.zeros(
                len(env_ids), dtype=torch.bool, device=self.device
            )
        return (
            self._v17_replay_local_latest_slot[:, env_ids] >= 0
        ).any(dim=0)

    def consume_markov_replay_target_state(
        self, env_ids: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        """Consume the exact target-side state staged by a replay reset."""

        env_ids = env_ids.to(device=self.device, dtype=torch.long)
        if not bool(self._v17_replay_pending_target_mask[env_ids].all()):
            raise RuntimeError(
                "V17 target replay was consumed without one pending state "
                "for every requested environment"
            )
        state = {
            name: value[env_ids].clone()
            for name, value in self._v17_replay_pending_target_state.items()
        }
        self._v17_replay_pending_target_mask[env_ids] = False
        return state

    def _restore_v17_joint_gains(
        self,
        env_ids: torch.Tensor,
        stiffness: torch.Tensor,
        damping: torch.Tensor,
    ) -> None:
        """Restore the reset-randomized PD latent into buffers, actuators, and PhysX."""

        self.robot.write_joint_stiffness_to_sim(
            stiffness, env_ids=env_ids
        )
        self.robot.write_joint_damping_to_sim(
            damping, env_ids=env_ids
        )
        for actuator in self.robot.actuators.values():
            joint_ids = actuator.joint_indices
            actuator.stiffness[env_ids] = stiffness[:, joint_ids]
            actuator.damping[env_ids] = damping[:, joint_ids]

    def post_swing_capture_enabled(self) -> bool:
        """Whether genuine on-policy follow-through states may enter the replay bank."""

        return bool(self._post_swing_ability_unlocked)

    def _clear_post_swing_replay_bank(self) -> None:
        """Invalidate every pre-gate sample without reallocating large device tensors."""

        self._post_swing_count = 0
        self._post_swing_ptr = 0
        self._v17_replay_total_count = 0
        self._post_swing_replay_ready_step = -1
        if self._v17_replay_bucket_count is not None:
            self._v17_replay_bucket_count.zero_()
        if self._v17_replay_bucket_ptr is not None:
            self._v17_replay_bucket_ptr.zero_()
        if self._v17_replay_local_latest_slot is not None:
            self._v17_replay_local_latest_slot.fill_(-1)
        self._v17_replay_last_sample_bucket.fill_(-1)
        self._v17_replay_last_sample_slot.fill_(-1)
        self._v17_replay_pending_target_mask.zero_()
        self._v17_replay_failure_score.zero_()
        self._v17_replay_pending_failure_count.zero_()
        self._update_v17_failure_sampling()

    def unlock_post_swing_ability_gate(self) -> None:
        """Latch Build replay capture on after the strike-only competence gate."""

        if self._post_swing_ability_unlocked:
            return
        self._post_swing_ability_unlocked = True
        # Defensive even though capture is fail-closed before the gate: exact-resume and future
        # callers cannot leak a random fresh actor's warning states into the admitted bank.
        self._clear_post_swing_replay_bank()
        self.metrics["post_swing_ability_unlocked"][:] = 1.0
        self.metrics["post_swing_capture_enabled"][:] = 1.0

    def _ability_gated_post_probability(self) -> tuple[float, int]:
        """Return the one-way replay probability and its zero-based ramp stage."""

        if not self._post_swing_ability_gate_enabled:
            recovery_scale = (
                float(self._recovery_curriculum_scale)
                if bool(self.cfg.post_swing_curriculum_scaled)
                else 1.0
            )
            return float(self.cfg.post_swing_start_prob) * recovery_scale, 0
        if not self._post_swing_ability_unlocked or not self._v17_replay_ready():
            return 0.0, 0
        step = int(getattr(self._env, "common_step_counter", 0))
        if self._post_swing_replay_ready_step < 0:
            self._post_swing_replay_ready_step = step
        interval = int(self.cfg.post_swing_replay_ramp_interval_steps)
        stage = min(
            max((step - self._post_swing_replay_ready_step) // interval, 0),
            len(self._post_swing_replay_ramp_probabilities) - 1,
        )
        return self._post_swing_replay_ramp_probabilities[stage], int(stage)

    def _v17_eligible_replay_buckets(self) -> torch.Tensor:
        if self._v17_replay_bucket_count is None:
            return torch.empty(0, dtype=torch.long, device=self.device)
        minimum = int(self.cfg.post_swing_min_fill_per_bucket)
        return torch.where(self._v17_replay_bucket_count >= minimum)[0]

    def _v17_replay_ready(self) -> bool:
        if (
            self._v17_replay_bucket_count is None
            or self._v17_replay_total_count < int(self.cfg.post_swing_min_fill)
        ):
            return False
        minimum = int(self.cfg.post_swing_min_fill_per_bucket)
        if self._post_swing_replay_contract == "markov_side_phase_severity_v3":
            # Match V14's warm-up rule: total genuine captures reaching min_fill is the only
            # admission condition. Same-environment availability is checked per reset below,
            # so sparse phase/severity buckets cannot form another curriculum deadlock.
            return True
        return bool((self._v17_replay_bucket_count >= minimum).all())

    def _v17_failure_probabilities(self) -> torch.Tensor:
        """Return Unitree-style smoothed failure mass with a uniform floor."""

        score = self._v17_replay_failure_score
        if not bool(self.cfg.post_swing_failure_adaptive):
            return torch.full_like(score, 1.0 / float(max(score.numel(), 1)))
        clips = int(self.motion.num_segments)
        phases = int(self.cfg.post_swing_capture_phase_bins)
        severities = int(self.cfg.post_swing_capture_severity_bins)
        shaped = score.reshape(clips, phases, severities)
        blend = float(self.cfg.post_swing_failure_phase_neighbor_blend)
        if blend > 0.0 and phases > 1:
            left = torch.cat((shaped[:, :1], shaped[:, :-1]), dim=1)
            right = torch.cat((shaped[:, 1:], shaped[:, -1:]), dim=1)
            shaped = (1.0 - blend) * shaped + blend * 0.5 * (left + right)
        probability = shaped.reshape(-1) + (
            float(self.cfg.post_swing_failure_uniform_ratio)
            / float(max(score.numel(), 1))
        )
        return probability / probability.sum().clamp_min(1.0e-12)

    def _update_v17_failure_sampling(self) -> None:
        """Apply pending reset failures once per control step and publish compact health metrics."""

        if not bool(self.cfg.post_swing_failure_adaptive):
            return
        alpha = float(self.cfg.post_swing_failure_ema_alpha)
        self._v17_replay_failure_score.mul_(1.0 - alpha).add_(
            self._v17_replay_pending_failure_count, alpha=alpha
        )
        self._v17_replay_pending_failure_count.zero_()
        probability = self._v17_failure_probabilities()
        self._v17_replay_sampling_probability.copy_(probability)
        entropy = -(probability * (probability + 1.0e-12).log()).sum()
        entropy /= math.log(max(int(probability.numel()), 2))
        top_probability, top_bucket = probability.max(dim=0)
        self.metrics["post_swing_failure_sampling_entropy"][:] = entropy
        self.metrics["post_swing_failure_sampling_top1_prob"][:] = top_probability
        self.metrics["post_swing_failure_sampling_top1_bucket"][:] = (
            top_bucket.float() / float(max(int(probability.numel()), 1))
        )

    def _record_v17_replay_failures(self, env_ids: torch.Tensor) -> None:
        """Attribute terminal outcomes to the replay bucket that seeded each environment."""

        if (
            not bool(self.cfg.post_swing_failure_adaptive)
            or self._resampling_from_wrap
            or env_ids.numel() == 0
        ):
            return
        buckets = self._v17_replay_last_sample_bucket[env_ids]
        has_bucket = buckets >= 0
        if not bool(has_bucket.any()):
            return
        termination = self._env.termination_manager
        failed = termination.terminated[env_ids].clone()
        time_outs = getattr(termination, "time_outs", None)
        if torch.is_tensor(time_outs):
            failed |= time_outs[env_ids]
        try:
            ready_timeout = termination.get_term("ready_release_timeout")
        except (AttributeError, KeyError, ValueError):
            ready_timeout = None
        if torch.is_tensor(ready_timeout):
            failed |= ready_timeout[env_ids]
        failed &= has_bucket
        if bool(failed.any()):
            counts = torch.bincount(
                buckets[failed], minlength=self._v17_replay_failure_score.numel()
            ).to(dtype=self._v17_replay_pending_failure_count.dtype)
            self._v17_replay_pending_failure_count.add_(counts)

    def _update_v17_replay_metrics(self) -> None:
        if self._post_swing_replay_contract not in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            replay_ready = self._post_swing_count >= int(
                self.cfg.post_swing_min_fill
            )
            self.metrics["post_swing_replay_buffer_fill"][:] = (
                float(self._post_swing_count)
                / float(max(int(self.cfg.post_swing_buffer_size), 1))
            )
            self.metrics["post_swing_replay_eligible_buckets"][:] = float(
                replay_ready
            )
            self.metrics["post_swing_replay_ready"][:] = float(replay_ready)
            effective_probability, ramp_stage = (
                self._ability_gated_post_probability()
            )
            self.metrics["post_swing_replay_probability_effective"][:] = (
                effective_probability if replay_ready else 0.0
            )
            self.metrics["post_swing_replay_ramp_stage"][:] = float(
                ramp_stage
            )
            self.metrics["post_swing_ability_unlocked"][:] = float(
                self._post_swing_ability_unlocked
            )
            self.metrics["post_swing_capture_enabled"][:] = float(
                self.post_swing_capture_enabled()
            )
            return
        eligible = self._v17_eligible_replay_buckets()
        self.metrics["post_swing_replay_buffer_fill"][:] = (
            float(self._v17_replay_total_count)
            / float(max(int(self.cfg.post_swing_buffer_size), 1))
        )
        self.metrics["post_swing_replay_eligible_buckets"][:] = float(
            len(eligible)
        )
        replay_ready = self._v17_replay_ready()
        self.metrics["post_swing_replay_ready"][:] = float(replay_ready)
        effective_probability, ramp_stage = (
            self._ability_gated_post_probability()
        )
        self.metrics["post_swing_replay_probability_effective"][:] = (
            effective_probability if replay_ready else 0.0
        )
        self.metrics["post_swing_replay_ramp_stage"][:] = float(ramp_stage)
        self.metrics["post_swing_ability_unlocked"][:] = float(
            self._post_swing_ability_unlocked
        )
        self.metrics["post_swing_capture_enabled"][:] = float(
            self.post_swing_capture_enabled()
        )
        if self._v17_replay_local_latest_slot is not None:
            local_coverage = (
                self._v17_replay_local_latest_slot >= 0
            ).any(dim=0).float().mean()
            self.metrics["post_swing_replay_local_coverage"][:] = (
                local_coverage
            )
        if (
            self._post_swing_replay_contract
            == "markov_side_phase_severity_v3"
        ):
            capacity = float(max(self._v17_replay_bucket_capacity, 1))
            counts = (
                self._v17_replay_bucket_count.tolist()
                if self._v17_replay_bucket_count is not None
                else [0] * self._v17_replay_bucket_total()
            )
            for bucket, count in enumerate(counts):
                label = self._v17_replay_bucket_label(bucket)
                self.metrics[f"post_swing_replay_count_{label}"][:] = (
                    float(count)
                )
                self.metrics[f"post_swing_replay_fill_{label}"][:] = (
                    float(count) / capacity
                )

    def _replay_severity(
        self, env_ids: torch.Tensor, action_term
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Classify legal pre-fault states as safe/warning/near-boundary.

        Severity is the union of base tilt and the tightest actual-q hard-limit margin. States
        already outside a hard joint limit, or beyond the capture tilt ceiling, are rejected
        rather than replayed as a recovery question.
        """

        projected = getattr(self.robot.data, "projected_gravity_b", None)
        if projected is None:
            raise RuntimeError(
                "severity-stratified replay requires projected_gravity_b"
            )
        tilt = torch.linalg.norm(projected[env_ids, :2], dim=-1)
        q = action_term._select_action_joints(
            self.robot.data.joint_pos
        )[env_ids]
        hard_lo = action_term._hard_lo[env_ids]
        hard_hi = action_term._hard_hi[env_ids]
        span = hard_hi - hard_lo
        lower_fraction = (q - hard_lo) / span
        upper_fraction = (hard_hi - q) / span
        tightest_margin = torch.minimum(
            lower_fraction, upper_fraction
        ).min(dim=-1).values
        legal = (
            torch.isfinite(tilt)
            & torch.isfinite(tightest_margin)
            # The physical termination is |tilt angle| > 0.7 rad. Reject the boundary itself so
            # replay contains only legal, pre-fault states; the racket-side hot-capture filter is
            # tighter (0.45) and remains an independent data-quality gate.
            & (tilt < math.sin(0.7))
            & (tightest_margin > 0.0)
        )
        warning = (
            (tilt >= float(self.cfg.post_swing_warning_tilt))
            | (
                tightest_margin
                <= float(
                    self.cfg.post_swing_warning_hard_margin_fraction
                )
            )
        )
        near = (
            (tilt >= float(self.cfg.post_swing_near_boundary_tilt))
            | (
                tightest_margin
                <= float(
                    self.cfg.post_swing_near_boundary_hard_margin_fraction
                )
            )
        )
        severity = torch.where(
            near,
            torch.full_like(tightest_margin, 2, dtype=torch.long),
            torch.where(
                warning,
                torch.ones_like(tightest_margin, dtype=torch.long),
                torch.zeros_like(tightest_margin, dtype=torch.long),
            ),
        )
        return severity, legal

    def _capture_post_swing_states(
        self,
        env_ids: torch.Tensor,
        *,
        phase_bin: int | None = None,
        source_clip_ids: torch.Tensor | None = None,
    ):
        """A8: snapshot follow-through robot states into the ring buffer.

        Two callers, both mid-episode with no teleport in between, so every entry is a genuine
        follow-through state: (1) clip WRAP (settled, ~1.3 s after impact — the legacy path) and
        (2) the racket command's hot capture at tts = -capture_delay (peak momentum, 2026-07-23
        fall-phase fix — without it the ~30% of swings that fall BEFORE wrap never fed the
        recovery curriculum).  Root position is stored origin-relative; write pairs
        root_state_w <-> write_root_state_to_sim (com-frame velocities) to match the stand/RSI
        branches.
        """
        if not self.post_swing_capture_enabled():
            return
        n = len(env_ids)
        if n == 0:
            return
        if self._post_swing_replay_contract in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            self._ensure_v17_replay_storage()
            phase_bins = int(self.cfg.post_swing_capture_phase_bins)
            if phase_bin is None or not 0 <= int(phase_bin) < phase_bins:
                raise ValueError(
                    f"V17 replay phase_bin must be in [0, {phase_bins}), got {phase_bin}"
                )
            env_ids = env_ids.to(device=self.device, dtype=torch.long)
            if source_clip_ids is None:
                source_clip_ids = self.clip_id[env_ids]
            else:
                source_clip_ids = source_clip_ids.to(
                    device=self.device, dtype=torch.long
                )
            if tuple(source_clip_ids.shape) != (len(env_ids),):
                raise ValueError(
                    "source_clip_ids must have one entry for every captured environment"
                )
            if bool(
                (source_clip_ids < 0).any()
                or (source_clip_ids >= int(self.motion.num_segments)).any()
            ):
                raise ValueError("source_clip_ids contains an invalid motion clip")

            action_manager = self._env.action_manager
            action_term = action_manager.get_term(
                str(self.cfg.post_swing_action_name)
            )
            target_term = self._v17_target_term()
            severity_bins = (
                int(self.cfg.post_swing_capture_severity_bins)
                if self._post_swing_replay_contract
                == "markov_side_phase_severity_v3"
                else 1
            )
            if severity_bins > 1:
                severity, legal = self._replay_severity(
                    env_ids, action_term
                )
                env_ids = env_ids[legal]
                source_clip_ids = source_clip_ids[legal]
                severity = severity[legal]
                if len(env_ids) == 0:
                    return
            else:
                severity = torch.zeros(
                    len(env_ids), dtype=torch.long, device=self.device
                )
            bucket_ids = (
                (source_clip_ids * phase_bins + int(phase_bin))
                * severity_bins
                + severity
            )
            capacity = int(self._v17_replay_bucket_capacity)
            for bucket_tensor in torch.unique(bucket_ids):
                bucket = int(bucket_tensor.item())
                bucket_mask = bucket_ids == bucket
                selected = env_ids[bucket_mask]
                selected_source_clip_ids = source_clip_ids[bucket_mask]
                # A single 4096-env capture can exceed one balanced bucket. Use an evenly-spaced
                # deterministic slice: capture must not consume the environment RNG while the
                # recovery scale is zero, otherwise "Stage 0 = exact V11" would be false even
                # though replay itself is disabled.
                if len(selected) > capacity:
                    order = torch.floor(
                        torch.arange(capacity, device=self.device).float()
                        * (float(len(selected)) / float(capacity))
                    ).long()
                    selected = selected[order]
                    selected_source_clip_ids = selected_source_clip_ids[order]
                count = len(selected)
                if count == 0:
                    continue

                # Motion segments are half-open [seg_start, seg_end). `_update_command` detects
                # a natural wrap only after incrementing the clock to exactly seg_end, while the
                # physical robot state still corresponds to the final valid reference frame.
                # Persist seg_end - 1 for that wrap snapshot.  In particular, the BH seg_end is
                # the concatenated motion length, so storing it verbatim would cause a CUDA
                # out-of-bounds read when replay later evaluates `body_pos_w[time_steps]`.
                segment_start = self.motion.seg_start[
                    selected_source_clip_ids
                ]
                segment_end = (
                    segment_start
                    + self.motion.seg_len[selected_source_clip_ids]
                )
                raw_motion_time_steps = self.time_steps[selected]
                invalid_clock = (
                    (raw_motion_time_steps < segment_start)
                    | (raw_motion_time_steps > segment_end)
                    | (
                        (raw_motion_time_steps == segment_end)
                        & (
                            int(phase_bin)
                            != int(self.cfg.post_swing_capture_phase_bins) - 1
                        )
                    )
                )
                if bool(invalid_clock.any()):
                    raise RuntimeError(
                        "V17 replay capture received a motion clock outside "
                        "its source clip"
                    )
                stored_motion_time_steps = torch.minimum(
                    raw_motion_time_steps, segment_end - 1
                )

                root = self.robot.data.root_state_w[selected].clone()
                root[:, :3] -= self._env.scene.env_origins[selected]
                action_state = action_term.capture_markov_replay_state(selected)
                target_state = target_term.capture_markov_replay_state(
                    selected
                )
                pointer = int(self._v17_replay_bucket_ptr[bucket].item())
                slots = (
                    pointer + torch.arange(count, device=self.device)
                ) % capacity
                previous_sources = self._v17_replay_source_env[
                    bucket, slots
                ]
                previous_valid = previous_sources >= 0
                if bool(previous_valid.any()):
                    previous_envs = previous_sources[previous_valid]
                    previous_slots = slots[previous_valid]
                    was_latest = (
                        self._v17_replay_local_latest_slot[
                            bucket, previous_envs
                        ]
                        == previous_slots
                    )
                    if bool(was_latest.any()):
                        self._v17_replay_local_latest_slot[
                            bucket, previous_envs[was_latest]
                        ] = -1
                self._v17_replay_root[bucket, slots] = root
                self._v17_replay_joint_pos[bucket, slots] = (
                    self.robot.data.joint_pos[selected]
                )
                self._v17_replay_joint_vel[bucket, slots] = (
                    self.robot.data.joint_vel[selected]
                )
                self._v17_replay_joint_stiffness[bucket, slots] = (
                    self.robot.data.joint_stiffness[selected]
                )
                self._v17_replay_joint_damping[bucket, slots] = (
                    self.robot.data.joint_damping[selected]
                )
                self._v17_replay_motion_time_steps[bucket, slots] = (
                    stored_motion_time_steps
                )
                self._v17_replay_motion_clip_id[bucket, slots] = (
                    selected_source_clip_ids
                )
                self._v17_replay_motion_hold_counter[bucket, slots] = (
                    self.hold_counter[selected]
                )
                self._v17_replay_manager_action[bucket, slots] = (
                    action_manager.action[selected]
                )
                self._v17_replay_manager_prev_action[bucket, slots] = (
                    action_manager.prev_action[selected]
                )
                self._v17_replay_action_raw[bucket, slots] = action_state["raw"]
                self._v17_replay_action_applied_raw[bucket, slots] = action_state[
                    "applied_raw"
                ]
                self._v17_replay_action_unclamped_qdes[bucket, slots] = (
                    action_state["unclamped_qdes"]
                )
                self._v17_replay_action_processed_qdes[bucket, slots] = (
                    action_state["processed_qdes"]
                )
                self._v17_replay_action_commanded_qdes[bucket, slots] = (
                    action_state["commanded_qdes"]
                )
                self._v17_replay_action_executed_qdes[bucket, slots] = (
                    action_state["executed_qdes"]
                )
                self._v17_replay_action_previous_executed_qdes[
                    bucket, slots
                ] = action_state["previous_executed_qdes"]
                self._v17_replay_action_qdes_delta[bucket, slots] = (
                    action_state["qdes_delta"]
                )
                self._v17_replay_action_previous_qdes_delta[
                    bucket, slots
                ] = action_state["previous_qdes_delta"]
                self._v17_replay_action_qdes_second_difference[
                    bucket, slots
                ] = action_state["qdes_second_difference"]
                self._v17_replay_action_decoder_offset[bucket, slots] = (
                    action_state["decoder_offset"]
                )
                self._v17_replay_action_decoder_scale[bucket, slots] = (
                    action_state["decoder_scale"]
                )
                self._v17_replay_action_delay_queue[bucket, slots] = (
                    action_state["delay_queue"]
                )
                self._v17_replay_action_delay_steps[bucket, slots] = (
                    action_state["delay_steps"]
                )
                self._store_v17_target_state(
                    bucket, slots, target_state
                )
                self._v17_replay_source_env[bucket, slots] = selected
                self._v17_replay_local_latest_slot[
                    bucket, selected
                ] = slots
                self._v17_replay_bucket_ptr[bucket] = (pointer + count) % capacity
                self._v17_replay_bucket_count[bucket] = min(
                    int(self._v17_replay_bucket_count[bucket].item()) + count,
                    capacity,
                )
            self._v17_replay_total_count = int(
                self._v17_replay_bucket_count.sum().item()
            )
            self._update_v17_replay_metrics()
            return

        root = self.robot.data.root_state_w[env_ids].clone()
        root[:, :3] -= self._env.scene.env_origins[env_ids]
        jp = self.robot.data.joint_pos[env_ids].clone()
        jv = self.robot.data.joint_vel[env_ids].clone()
        size = int(self.cfg.post_swing_buffer_size)
        if self._post_swing_root is None:
            self._post_swing_root = torch.zeros(size, 13, device=self.device)
            self._post_swing_joint_pos = torch.zeros(size, jp.shape[1], device=self.device)
            self._post_swing_joint_vel = torch.zeros(size, jv.shape[1], device=self.device)
        # ring write (n < size in practice; wrap the slot indices just in case)
        slots = (self._post_swing_ptr + torch.arange(n, device=self.device)) % size
        self._post_swing_root[slots] = root
        self._post_swing_joint_pos[slots] = jp
        self._post_swing_joint_vel[slots] = jv
        self._post_swing_ptr = int((self._post_swing_ptr + n) % size)
        self._post_swing_count = min(self._post_swing_count + n, size)
        self._update_v17_replay_metrics()

    def _write_post_swing_states(self, env_ids: torch.Tensor):
        """A8: initialize `env_ids` from random buffered end-of-swing states (origin re-based)."""
        if self._post_swing_replay_contract in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            if not self._v17_replay_ready():
                raise RuntimeError("V17 replay write requested before the buffer is ready")
            if (
                self._post_swing_replay_contract
                == "markov_side_phase_severity_v3"
            ):
                latest = self._v17_replay_local_latest_slot[:, env_ids]
                available = latest >= 0
                if not bool(available.any(dim=0).all()):
                    raise RuntimeError(
                        "V17 local replay write requested for an environment "
                        "without a same-plant snapshot"
                    )
                # Keep the same-environment plant constraint, but use Unitree/BeyondMimic-style
                # failure mass instead of inverse global fill. Choose FH/BH (or a general clip)
                # uniformly first, then choose phase/severity within that side. This prevents a
                # hard side from monopolising the batch while retaining the uniform bucket floor.
                if bool(self.cfg.post_swing_failure_adaptive):
                    probabilities = self._v17_failure_probabilities()
                    clips = int(self.motion.num_segments)
                    buckets_per_clip = (
                        int(self.cfg.post_swing_capture_phase_bins)
                        * int(self.cfg.post_swing_capture_severity_bins)
                    )
                    by_side = available.reshape(
                        clips, buckets_per_clip, len(env_ids)
                    ).any(dim=1).transpose(0, 1).float()
                    chosen_side = torch.multinomial(
                        by_side, 1, replacement=True
                    ).squeeze(1)
                    bucket_side = torch.arange(
                        probabilities.numel(), device=self.device
                    ) // buckets_per_clip
                    weights = available.transpose(0, 1).float()
                    weights *= probabilities.unsqueeze(0)
                    weights *= (
                        bucket_side.unsqueeze(0) == chosen_side.unsqueeze(1)
                    ).float()
                else:
                    weights = available.transpose(0, 1).float()
                    weights = weights / self._v17_replay_bucket_count.float().clamp_min(
                        1.0
                    ).unsqueeze(0)
                chosen_bucket = torch.multinomial(
                    weights, 1, replacement=True
                ).squeeze(1)
                chosen_slot = latest[
                    chosen_bucket,
                    torch.arange(len(env_ids), device=self.device),
                ]
                if not bool(
                    (
                        self._v17_replay_source_env[
                            chosen_bucket, chosen_slot
                        ]
                        == env_ids
                    ).all()
                ):
                    raise RuntimeError(
                        "V17 local replay index no longer points to the "
                        "destination environment's own plant snapshot"
                    )
            else:
                eligible = self._v17_eligible_replay_buckets()
                chosen_bucket = eligible[
                    torch.randint(
                        0,
                        len(eligible),
                        (len(env_ids),),
                        device=self.device,
                    )
                ]
                counts = self._v17_replay_bucket_count[chosen_bucket]
                random_fraction = torch.rand(len(env_ids), device=self.device)
                chosen_slot = torch.floor(
                    random_fraction * counts.float()
                ).long()
            root = self._v17_replay_root[chosen_bucket, chosen_slot].clone()
            root[:, :3] += self._env.scene.env_origins[env_ids]
            self.robot.write_root_state_to_sim(root, env_ids=env_ids)
            self.last_reset_root_pos_w[env_ids] = root[:, :3]
            joint_pos = self._v17_replay_joint_pos[
                chosen_bucket, chosen_slot
            ].clone()
            joint_vel = self._v17_replay_joint_vel[
                chosen_bucket, chosen_slot
            ].clone()
            # Passive head joints are not policy-controlled. Rebase them to the destination
            # environment's calibration/default rather than importing a source environment's
            # startup-offset DR and creating a first-step PD target jump.
            action_manager = self._env.action_manager
            action_term = action_manager.get_term(
                str(self.cfg.post_swing_action_name)
            )
            passive_joint_ids = getattr(
                action_term, "_passive_joint_ids", None
            )
            if (
                torch.is_tensor(passive_joint_ids)
                and passive_joint_ids.numel() > 0
            ):
                destination_default = (
                    self.robot.data.default_joint_pos[env_ids].index_select(
                        -1, passive_joint_ids
                    )
                )
                joint_pos.index_copy_(
                    -1, passive_joint_ids, destination_default
                )
                joint_vel.index_fill_(-1, passive_joint_ids, 0.0)
            self.robot.write_joint_state_to_sim(
                joint_pos,
                joint_vel,
                env_ids=env_ids,
            )
            self._restore_v17_joint_gains(
                env_ids,
                self._v17_replay_joint_stiffness[
                    chosen_bucket, chosen_slot
                ].clone(),
                self._v17_replay_joint_damping[
                    chosen_bucket, chosen_slot
                ].clone(),
            )
            # A reset callback executes between physics steps, so IsaacLab's state writer must
            # update both PhysX and its actor-visible lazy buffers immediately. Fail closed if a
            # future IsaacLab change breaks that contract: the first policy observation must
            # describe the physical replay state that was just written.
            if not bool(
                torch.allclose(
                    self.robot.data.joint_pos[env_ids],
                    joint_pos,
                    rtol=0.0,
                    atol=1.0e-6,
                )
                and torch.allclose(
                    self.robot.data.joint_vel[env_ids],
                    joint_vel,
                    rtol=0.0,
                    atol=1.0e-6,
                )
            ):
                raise RuntimeError(
                    "V17 replay joint-state write did not update IsaacLab's live state buffers"
                )

            source_offset = self._v17_replay_action_decoder_offset[
                chosen_bucket, chosen_slot
            ]
            source_scale = self._v17_replay_action_decoder_scale[
                chosen_bucket, chosen_slot
            ]
            remapped_term_state = action_term.restore_markov_replay_state(
                env_ids,
                {
                    "raw": self._v17_replay_action_raw[
                        chosen_bucket, chosen_slot
                    ],
                    "applied_raw": self._v17_replay_action_applied_raw[
                        chosen_bucket, chosen_slot
                    ],
                    "unclamped_qdes": self._v17_replay_action_unclamped_qdes[
                        chosen_bucket, chosen_slot
                    ],
                    "processed_qdes": self._v17_replay_action_processed_qdes[
                        chosen_bucket, chosen_slot
                    ],
                    "commanded_qdes": self._v17_replay_action_commanded_qdes[
                        chosen_bucket, chosen_slot
                    ],
                    "executed_qdes": self._v17_replay_action_executed_qdes[
                        chosen_bucket, chosen_slot
                    ],
                    "previous_executed_qdes": (
                        self._v17_replay_action_previous_executed_qdes[
                            chosen_bucket, chosen_slot
                        ]
                    ),
                    "qdes_delta": self._v17_replay_action_qdes_delta[
                        chosen_bucket, chosen_slot
                    ],
                    "previous_qdes_delta": (
                        self._v17_replay_action_previous_qdes_delta[
                            chosen_bucket, chosen_slot
                        ]
                    ),
                    "qdes_second_difference": (
                        self._v17_replay_action_qdes_second_difference[
                            chosen_bucket, chosen_slot
                        ]
                    ),
                    "decoder_offset": source_offset,
                    "decoder_scale": source_scale,
                    "delay_queue": self._v17_replay_action_delay_queue[
                        chosen_bucket, chosen_slot
                    ],
                    "delay_steps": self._v17_replay_action_delay_steps[
                        chosen_bucket, chosen_slot
                    ],
                },
            )
            manager_action = action_term.remap_markov_replay_raw_actions(
                env_ids,
                self._v17_replay_manager_action[
                    chosen_bucket, chosen_slot
                ],
                source_offset,
                source_scale,
            )
            manager_prev_action = (
                action_term.remap_markov_replay_raw_actions(
                    env_ids,
                    self._v17_replay_manager_prev_action[
                        chosen_bucket, chosen_slot
                    ],
                    source_offset,
                    source_scale,
                )
            )
            if not bool(
                torch.allclose(
                    manager_action,
                    remapped_term_state["raw"],
                    rtol=0.0,
                    atol=1.0e-7,
                )
            ):
                raise RuntimeError(
                    "V17 replay captured inconsistent ActionManager/action-term raw state"
                )
            action_manager._action[env_ids] = manager_action
            action_manager._prev_action[env_ids] = manager_prev_action
            restored_time_steps = self._v17_replay_motion_time_steps[
                chosen_bucket, chosen_slot
            ]
            restored_clip_id = self._v17_replay_motion_clip_id[
                chosen_bucket, chosen_slot
            ]
            if bool(
                (restored_clip_id < 0).any()
                or (
                    restored_clip_id
                    >= int(self.motion.num_segments)
                ).any()
            ):
                raise RuntimeError(
                    "V17 replay restore selected an invalid motion clip"
                )
            restored_segment_start = self.motion.seg_start[
                restored_clip_id
            ]
            restored_segment_end = (
                restored_segment_start
                + self.motion.seg_len[restored_clip_id]
            )
            if bool(
                (
                    (restored_time_steps < restored_segment_start)
                    | (restored_time_steps >= restored_segment_end)
                ).any()
            ):
                raise RuntimeError(
                    "V17 replay restore selected a motion clock outside "
                    "its half-open clip interval"
                )
            self.clip_id[env_ids] = restored_clip_id
            self.time_steps[env_ids] = restored_time_steps
            self.hold_counter[env_ids] = (
                self._v17_replay_motion_hold_counter[
                    chosen_bucket, chosen_slot
                ]
            )
            for name, source in self._v17_replay_target_state.items():
                values = source[chosen_bucket, chosen_slot]
                pending = self._v17_replay_pending_target_state.get(name)
                expected_shape = (self.num_envs,) + tuple(values.shape[1:])
                if pending is None:
                    pending = torch.zeros(
                        expected_shape,
                        dtype=values.dtype,
                        device=self.device,
                    )
                    self._v17_replay_pending_target_state[name] = pending
                elif (
                    tuple(pending.shape) != expected_shape
                    or pending.dtype != values.dtype
                ):
                    raise RuntimeError(
                        f"V17 pending target field {name!r} changed contract"
                    )
                pending[env_ids] = values
            self._v17_replay_pending_target_mask[env_ids] = True
            self.post_swing_replay_active[env_ids] = True
            self._v17_replay_last_sample_bucket[env_ids] = chosen_bucket
            self._v17_replay_last_sample_slot[env_ids] = chosen_slot
            self.metrics["post_swing_replay_bucket"][env_ids] = (
                chosen_bucket.float()
                / float(
                    max(
                        int(self.motion.num_segments)
                        * int(self.cfg.post_swing_capture_phase_bins)
                        * (
                            int(self.cfg.post_swing_capture_severity_bins)
                            if self._post_swing_replay_contract
                            == "markov_side_phase_severity_v3"
                            else 1
                        )
                        - 1,
                        1,
                    )
                )
            )
            return

        picks = torch.randint(0, self._post_swing_count, (len(env_ids),), device=self.device)
        root = self._post_swing_root[picks].clone()
        root[:, :3] += self._env.scene.env_origins[env_ids]
        self.robot.write_root_state_to_sim(root, env_ids=env_ids)
        self.last_reset_root_pos_w[env_ids] = root[:, :3]
        self.robot.write_joint_state_to_sim(
            self._post_swing_joint_pos[picks].clone(),
            self._post_swing_joint_vel[picks].clone(),
            env_ids=env_ids,
        )

    def _resample_command(self, env_ids: Sequence[int]):
        if len(env_ids) == 0:
            return
        env_ids_t = env_ids if torch.is_tensor(env_ids) else torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        env_ids_t = env_ids_t.to(device=self.device, dtype=torch.long)
        # Read terminal provenance before clip/time and last-sampled-bucket state are replaced.
        self._record_v17_replay_failures(env_ids_t)
        self._adaptive_sampling(env_ids_t)

        recovery_scale = (
            float(self._recovery_curriculum_scale)
            if bool(self.cfg.post_swing_curriculum_scaled)
            else 1.0
        )
        effective_post_p, replay_ramp_stage = (
            self._ability_gated_post_probability()
        )
        self.metrics["post_swing_recovery_scale"][:] = recovery_scale
        if self._post_swing_replay_contract in {
            "markov_stratified_v2",
            "markov_side_phase_severity_v3",
        }:
            replay_ready = self._v17_replay_ready()
        else:
            replay_ready = self._post_swing_count >= int(
                self.cfg.post_swing_min_fill
            )
        self.metrics["post_swing_replay_probability_effective"][:] = (
            effective_post_p if replay_ready else 0.0
        )
        self.metrics["post_swing_replay_ramp_stage"][:] = float(
            replay_ramp_stage
        )
        self.metrics["post_swing_ability_unlocked"][:] = float(
            self._post_swing_ability_unlocked
        )
        self.metrics["post_swing_capture_enabled"][:] = float(
            self.post_swing_capture_enabled()
        )
        self.metrics["post_swing_replay_reset"][env_ids_t] = 0.0
        self.metrics["runtime_handoff_reset"][env_ids_t] = 0.0
        if not self._resampling_from_wrap:
            self.post_swing_replay_active[env_ids_t] = False
            self.runtime_handoff_active[env_ids_t] = False
            self._v17_replay_last_sample_bucket[env_ids_t] = -1
            self._v17_replay_last_sample_slot[env_ids_t] = -1
            self.metrics["post_swing_replay_bucket"][env_ids_t] = -1.0

        # Pre-swing HOLD (Phase A): freeze the reference at the swing's first frame for a random
        # number of control steps ("the ball is not reaching yet"). Applies to resets AND wraps.
        lo, hi = self.cfg.hold_steps_range
        self.hold_counter[env_ids_t] = torch.randint(int(lo), int(hi) + 1, (len(env_ids_t),), device=self.device)
        # LONG-HOLD TAIL (2026-07-11 RallyV8): with prob hold_long_prob the hold is drawn from
        # hold_long_steps_range instead — deploy active-idle windows run 4-7 s (post-miss waits,
        # yawed-release re-squares) while the base range covers only 0.5-2.5 s, and models
        # trained without long-hold coverage drift/fall in exactly those windows (AGI G3
        # 2026-07-11 residual). The draw is exogenous (never policy-extendable), so hold-gated
        # income stays un-farmable. 0.0 = off (all existing tasks unchanged).
        long_p = float(getattr(self.cfg, "hold_long_prob", 0.0))
        if long_p > 0.0:
            llo, lhi = self.cfg.hold_long_steps_range
            if int(lhi) >= int(llo) > 0:
                long_mask = torch.rand(len(env_ids_t), device=self.device) < long_p
                if long_mask.any():
                    long_ids = env_ids_t[long_mask]
                    self.hold_counter[long_ids] = torch.randint(
                        int(llo), int(lhi) + 1, (len(long_ids),), device=self.device
                    )

        # Intra-episode clip WRAP: no teleport (deploy case) — the policy must physically carry
        # the body from the previous swing's end into the new swing's windup. The imitation
        # targets are anchor-relative, so the new reference re-anchors to the robot where it is.
        # Teleporting at a wrap (legacy RSI behavior) requires wrap_teleport=True.
        if self._resampling_from_wrap and not self.cfg.wrap_teleport:
            min_hold = int(
                round(float(self.cfg.post_wrap_min_hold) * recovery_scale)
            )
            if min_hold > 0:
                # A bare clamp collapses ~85% of wrap holds onto EXACTLY min_hold (audit
                # 2026-07-23): the U[lo,hi] base draw sits entirely below the floor.  Re-draw the
                # clamped mass uniformly inside [min_hold, min_hold + jitter] so wrap-hold length
                # is genuinely random above the floor; jitter=0 keeps the legacy exact clamp.
                jitter = int(
                    round(
                        float(
                            getattr(
                                self.cfg,
                                "post_wrap_hold_jitter_steps",
                                0,
                            )
                        )
                        * recovery_scale
                    )
                )
                below = self.hold_counter[env_ids_t] < min_hold
                if jitter > 0 and bool(below.any()):
                    below_ids = env_ids_t[below]
                    self.hold_counter[below_ids] = torch.randint(
                        min_hold, min_hold + jitter + 1, (len(below_ids),), device=self.device
                    )
                else:
                    self.hold_counter[env_ids_t] = torch.clamp(
                        self.hold_counter[env_ids_t], min=min_hold
                    )
            return

        # TRUE episode reset: three-way split — DEFAULT STAND (deploy entry) / POST-SWING buffer
        # (A8: the policy's own end-of-swing states) / legacy RSI teleport onto the (noised)
        # reference frame. One uniform draw per env: u < stand_p -> stand; stand_p <= u <
        # stand_p + post_p -> post-swing (only once the buffer has post_swing_min_fill entries);
        # else RSI.
        u = torch.rand(len(env_ids_t), device=self.device)
        stand_mask = torch.zeros(len(env_ids_t), dtype=torch.bool, device=self.device)
        post_mask = torch.zeros(len(env_ids_t), dtype=torch.bool, device=self.device)
        if not self._resampling_from_wrap:
            stand_p = float(self.cfg.stand_start_prob)
            post_p = effective_post_p
            if stand_p > 0.0:
                stand_mask = u < stand_p
            if post_p > 0.0 and replay_ready:
                post_mask = (u >= stand_p) & (u < stand_p + post_p)
            elif post_p > 0.0 and bool(self.cfg.post_swing_fallback_to_stand):
                # The on-policy follow-through buffer is necessarily empty at fresh-run startup.
                # Reassign only its unavailable probability mass to deploy-matched stand states;
                # otherwise the intended recovery quarter is silently populated by arbitrary RSI.
                stand_mask = u < min(stand_p + post_p, 1.0)
            if (
                self._post_swing_replay_contract
                == "markov_side_phase_severity_v3"
                and bool(post_mask.any())
            ):
                proposed = torch.where(post_mask)[0]
                local_available = self._v17_local_replay_available(
                    env_ids_t[proposed]
                )
                unavailable = proposed[~local_available]
                post_mask[unavailable] = False
                if (
                    len(unavailable) > 0
                    and bool(self.cfg.post_swing_fallback_to_stand)
                ):
                    stand_mask[unavailable] = True
        stand_ids = env_ids_t[stand_mask]
        post_ids = env_ids_t[post_mask]
        rsi_ids = env_ids_t[~(stand_mask | post_mask)]

        if len(stand_ids) > 0:
            default_root = self.robot.data.default_root_state[stand_ids].clone()
            default_root[:, :3] += self._env.scene.env_origins[stand_ids]
            default_root[:, 7:] = 0.0  # zero lin/ang velocity
            # FIXED-STATION RECOVERY DR: offset the robot while leaving the commanded station at
            # the environment origin.  This creates an actual return-to-station question instead
            # of moving the target station.  It is applied only to DEFAULT-STAND starts; RSI and
            # post-swing replay keep their existing semantics.  (0, 0) preserves every historical
            # task exactly.
            xl, xh = self.cfg.stand_start_x_range
            if float(xh) > float(xl):
                default_root[:, 0] += sample_uniform(
                    float(xl), float(xh), (len(stand_ids),), self.device
                )
            yl, yh = self.cfg.stand_start_y_range
            if float(yh) > float(yl):
                default_root[:, 1] += sample_uniform(
                    float(yl), float(yh), (len(stand_ids),), self.device
                )
            # HEADING-RECOVERY DR (2026-07-08 rally-gate finding): optionally spawn stand starts
            # YAWED. Deploy follow-throughs leave the robot 30-55° off the strike heading, and
            # with stand starts always exactly square (plus no RSI yaw noise) the policy never
            # sees a yawed state — so it never learns to turn back and the runner must gate
            # engages on heading + wait for an operator re-stand. A yawed stand start + the
            # hold_heading reward trains exactly that recovery: turn square during the hold,
            # then swing. (0,0) = off (legacy exact-square stands).
            yl, yh = self.cfg.stand_start_yaw_range
            # STEP-SCHEDULED YAW RAMP (2026-07-11 RallyV8): scale the yaw band linearly from 0
            # to its configured width over stand_start_yaw_ramp_steps env control steps. A
            # fresh run that starts at the full ±0.6 band never converges its heading recovery
            # (V6 regression: idle base rotates ~1 rad/s hunting its heading), while the manual
            # fix was staged warm-resumes (±0.2 -> ±0.35 -> ±0.6). This bakes that curriculum
            # into one run. Reads env.common_step_counter (same source as the ref-perturb
            # curriculum); 0 = off (full band immediately, all existing tasks unchanged).
            ramp_steps = int(getattr(self.cfg, "stand_start_yaw_ramp_steps", 0))
            if ramp_steps > 0:
                ramp = min(1.0, float(self._env.common_step_counter) / float(ramp_steps))
                yl, yh = float(yl) * ramp, float(yh) * ramp
            if float(yh) > float(yl):
                yaws = sample_uniform(float(yl), float(yh), (len(stand_ids),), self.device)
                zeros = torch.zeros_like(yaws)
                dq = quat_from_euler_xyz(zeros, zeros, yaws)
                default_root[:, 3:7] = quat_mul(dq, default_root[:, 3:7])
            self.robot.write_root_state_to_sim(default_root, env_ids=stand_ids)
            self.last_reset_root_pos_w[stand_ids] = default_root[:, :3]
            self.robot.write_joint_state_to_sim(
                self.robot.data.default_joint_pos[stand_ids],
                torch.zeros_like(self.robot.data.default_joint_vel[stand_ids]),
                env_ids=stand_ids,
            )
            # Give the stand-started envs time to travel stand -> windup before the clip runs.
            self.hold_counter[stand_ids] = torch.clamp(
                self.hold_counter[stand_ids], min=int(self.cfg.stand_start_min_hold)
            )
            # Runtime handoff is a strict subset of the inherited stand-start mass: the plant,
            # q_des and action history all begin at the same default pose, while only the
            # exogenous no-ball hold is lengthened to the runner's 1--3 s entry window.
            handoff_probability = float(self.cfg.runtime_handoff_start_prob)
            if handoff_probability > 0.0:
                conditional_probability = handoff_probability / max(
                    float(self.cfg.stand_start_prob), 1.0e-12
                )
                if conditional_probability >= 1.0:
                    handoff_ids = stand_ids
                else:
                    handoff_ids = stand_ids[
                        torch.rand(len(stand_ids), device=self.device)
                        < conditional_probability
                    ]
                if len(handoff_ids) > 0:
                    hlo, hhi = (
                        int(value)
                        for value in self.cfg.runtime_handoff_hold_steps_range
                    )
                    # The ordinary V11 stand branch deliberately adds x/yaw recovery DR.  A
                    # runtime-handoff sample instead represents the runner's exact static pose:
                    # overwrite that subset with the unperturbed default root and bind both
                    # ActionManager history slots plus the term-local q_des history to affine
                    # action zero (q_des == current/default q).
                    static_root = self.robot.data.default_root_state[
                        handoff_ids
                    ].clone()
                    static_root[:, :3] += self._env.scene.env_origins[
                        handoff_ids
                    ]
                    static_root[:, 7:] = 0.0
                    self.robot.write_root_state_to_sim(
                        static_root, env_ids=handoff_ids
                    )
                    self.last_reset_root_pos_w[handoff_ids] = static_root[:, :3]
                    static_joint_pos = self.robot.data.default_joint_pos[
                        handoff_ids
                    ].clone()
                    static_joint_vel = torch.zeros_like(
                        self.robot.data.default_joint_vel[handoff_ids]
                    )
                    self.robot.write_joint_state_to_sim(
                        static_joint_pos,
                        static_joint_vel,
                        env_ids=handoff_ids,
                    )
                    action_manager = self._env.action_manager
                    action_term = action_manager.get_term(
                        str(self.cfg.post_swing_action_name)
                    )
                    action_term.reset(handoff_ids)
                    action_manager._action[handoff_ids] = 0.0
                    action_manager._prev_action[handoff_ids] = 0.0
                    qdes = getattr(action_term, "_qdes_executed", None)
                    selected_q = getattr(
                        action_term, "_select_action_joints", lambda value: None
                    )(self.robot.data.joint_pos)
                    if not (
                        torch.is_tensor(qdes)
                        and torch.is_tensor(selected_q)
                        and torch.allclose(
                            qdes[handoff_ids],
                            selected_q[handoff_ids],
                            rtol=0.0,
                            atol=1.0e-6,
                        )
                    ):
                        raise RuntimeError(
                            "runtime handoff failed to align q_des, last-action, "
                            "and the exact static plant pose"
                        )
                    self.hold_counter[handoff_ids] = torch.randint(
                        hlo,
                        hhi + 1,
                        (len(handoff_ids),),
                        device=self.device,
                    )
                    self.runtime_handoff_active[handoff_ids] = True
                    self.metrics["runtime_handoff_reset"][handoff_ids] = 1.0

        if len(post_ids) > 0:
            self._write_post_swing_states(post_ids)
            self.metrics["post_swing_replay_reset"][post_ids] = 1.0
            # Settle follow-through -> windup before the clip runs. Scale the extra floor
            # continuously from the unchanged V11 random hold at scale 0 to the full recovery
            # window at scale 1; the replay probability is independently scale-ramped above.
            # Markov-v3 resumes the captured follow-through phase itself.  Its ordinary clip wrap
            # will create the scaled recovery hold; inserting a hold at the hot snapshot would
            # replace rather than replay the captured MDP transition.
            if (
                self._post_swing_replay_contract
                != "markov_side_phase_severity_v3"
            ):
                replay_min_hold = int(
                    round(
                        float(self.cfg.post_swing_min_hold)
                        * recovery_scale
                    )
                )
                self.hold_counter[post_ids] = torch.clamp(
                    self.hold_counter[post_ids], min=replay_min_hold
                )

        if len(rsi_ids) == 0:
            return
        env_ids = rsi_ids

        root_pos = self.body_pos_w[:, 0].clone()
        root_ori = self.body_quat_w[:, 0].clone()
        root_lin_vel = self.body_lin_vel_w[:, 0].clone()
        root_ang_vel = self.body_ang_vel_w[:, 0].clone()

        range_list = [self.cfg.pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
        ranges = torch.tensor(range_list, device=self.device)
        rand_samples = sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=self.device)
        root_pos[env_ids] += rand_samples[:, 0:3]
        orientations_delta = quat_from_euler_xyz(rand_samples[:, 3], rand_samples[:, 4], rand_samples[:, 5])
        root_ori[env_ids] = quat_mul(orientations_delta, root_ori[env_ids])
        range_list = [self.cfg.velocity_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
        ranges = torch.tensor(range_list, device=self.device)
        rand_samples = sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=self.device)
        root_lin_vel[env_ids] += rand_samples[:, :3]
        root_ang_vel[env_ids] += rand_samples[:, 3:]

        joint_pos = self.joint_pos.clone()
        joint_vel = self.joint_vel.clone()

        joint_pos += sample_uniform(*self.cfg.joint_position_range, joint_pos.shape, joint_pos.device)
        soft_joint_pos_limits = self.robot.data.soft_joint_pos_limits[env_ids]
        joint_pos[env_ids] = torch.clip(
            joint_pos[env_ids], soft_joint_pos_limits[:, :, 0], soft_joint_pos_limits[:, :, 1]
        )
        self.robot.write_joint_state_to_sim(joint_pos[env_ids], joint_vel[env_ids], env_ids=env_ids)
        self.robot.write_root_state_to_sim(
            torch.cat([root_pos[env_ids], root_ori[env_ids], root_lin_vel[env_ids], root_ang_vel[env_ids]], dim=-1),
            env_ids=env_ids,
        )
        self.last_reset_root_pos_w[env_ids] = root_pos[env_ids]

    def _update_command(self):
        # Pre-swing HOLD: held envs keep the reference frozen at the swing's first frame
        # ("waiting for the ball"); everyone else advances the clip clock.
        held = self.hold_counter > 0
        self.post_swing_replay_active &= held
        self.runtime_handoff_active &= held
        self.hold_counter = torch.clamp(self.hold_counter - 1, min=0)
        self.metrics["in_hold"] = held.float()
        self.metrics["runtime_handoff_active"] = (
            self.runtime_handoff_active.float()
        )
        if "clip_switch_count" not in self.metrics:
            self.metrics["clip_switch_count"] = torch.zeros(self.num_envs, device=self.device)
        self.time_steps += (~held).long()
        if self._multiseg:
            # Wrap at the END of the env's current clip/segment, not the global concatenated end.
            seg_end = self.motion.seg_start[self.clip_id] + self.motion.seg_len[self.clip_id]
            wrap_ids = torch.where(self.time_steps >= seg_end)[0]
            # DEPLOY-PARITY CLIP SWITCH (venue falls 2026-07-04): the runner's reference clock flips
            # clip_id whenever the planner re-sides the target — at an ARBITRARY mid-swing moment —
            # and the reference jumps to the new clip's first frame (pp_reference_clock.hpp clamps
            # tts-large to seg_start). Training previously only switched clips at clip END, so the
            # policy never saw that discontinuity and falls at 准备/正手/反手 switches on hardware.
            # With per-step prob clip_switch_prob an env aborts its swing operator-style and routes
            # through the SAME wrap-resample path (uniform new clip, frame 0, hold, fresh target).
            # NOTE: aborted swings count as uncompleted starts (slight completion-rate deflation).
            if float(self.cfg.clip_switch_prob) > 0.0:
                sw = torch.rand(self.num_envs, device=self.device) < float(self.cfg.clip_switch_prob)
                sw[wrap_ids] = False
                self.metrics["clip_switch_count"] = sw.float()
                switch_ids = torch.where(sw)[0]
                env_ids = torch.cat([wrap_ids, switch_ids]) if len(switch_ids) > 0 else wrap_ids
            else:
                env_ids = wrap_ids
        else:
            env_ids = torch.where(self.time_steps >= self.motion.time_step_total)[0]
            wrap_ids = env_ids
        self.just_resampled = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        if len(env_ids) > 0:
            self.just_resampled[env_ids] = True
            # A8: only envs that physically COMPLETED a swing (true wraps — passed the strike alive,
            # not teleported, not aborted-by-switch) feed the post-swing ring buffer.
            if self.post_swing_capture_enabled() and len(wrap_ids) > 0:
                if self._post_swing_replay_contract in {
                    "markov_stratified_v2",
                    "markov_side_phase_severity_v3",
                }:
                    self._capture_post_swing_states(
                        wrap_ids,
                        phase_bin=int(self.cfg.post_swing_capture_phase_bins) - 1,
                        source_clip_ids=self.clip_id[wrap_ids].clone(),
                    )
                else:
                    self._capture_post_swing_states(wrap_ids)
        # Wrap-path resample: skips the RSI teleport (cfg.wrap_teleport=False) so the policy
        # physically transitions swing -> swing. True resets go through reset()/manager instead.
        self._resampling_from_wrap = True
        try:
            self._resample_command(env_ids)
        finally:
            self._resampling_from_wrap = False

        anchor_pos_w_repeat = self.anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        anchor_quat_w_repeat = self.anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_pos_w_repeat = self.robot_anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_quat_w_repeat = self.robot_anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)

        delta_pos_w = robot_anchor_pos_w_repeat
        delta_pos_w[..., 2] = anchor_pos_w_repeat[..., 2]
        delta_ori_w = yaw_quat(quat_mul(robot_anchor_quat_w_repeat, quat_inv(anchor_quat_w_repeat)))

        self.body_quat_relative_w = quat_mul(delta_ori_w, self.body_quat_w)
        self.body_pos_relative_w = delta_pos_w + quat_apply(delta_ori_w, self.body_pos_w - anchor_pos_w_repeat)

        self.bin_failed_count = (
            self.cfg.adaptive_alpha * self._current_bin_failed + (1 - self.cfg.adaptive_alpha) * self.bin_failed_count
        )
        self._current_bin_failed.zero_()
        self._update_v17_failure_sampling()

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "current_anchor_visualizer"):
                self.current_anchor_visualizer = VisualizationMarkers(
                    self.cfg.anchor_visualizer_cfg.replace(prim_path="/Visuals/Command/current/anchor")
                )
                self.goal_anchor_visualizer = VisualizationMarkers(
                    self.cfg.anchor_visualizer_cfg.replace(prim_path="/Visuals/Command/goal/anchor")
                )

                self.current_body_visualizers = []
                self.goal_body_visualizers = []
                for name in self.cfg.body_names:
                    self.current_body_visualizers.append(
                        VisualizationMarkers(
                            self.cfg.body_visualizer_cfg.replace(prim_path="/Visuals/Command/current/" + name)
                        )
                    )
                    self.goal_body_visualizers.append(
                        VisualizationMarkers(
                            self.cfg.body_visualizer_cfg.replace(prim_path="/Visuals/Command/goal/" + name)
                        )
                    )

            self.current_anchor_visualizer.set_visibility(True)
            self.goal_anchor_visualizer.set_visibility(True)
            for i in range(len(self.cfg.body_names)):
                self.current_body_visualizers[i].set_visibility(True)
                self.goal_body_visualizers[i].set_visibility(True)

        else:
            if hasattr(self, "current_anchor_visualizer"):
                self.current_anchor_visualizer.set_visibility(False)
                self.goal_anchor_visualizer.set_visibility(False)
                for i in range(len(self.cfg.body_names)):
                    self.current_body_visualizers[i].set_visibility(False)
                    self.goal_body_visualizers[i].set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return

        self.current_anchor_visualizer.visualize(self.robot_anchor_pos_w, self.robot_anchor_quat_w)
        self.goal_anchor_visualizer.visualize(self.anchor_pos_w, self.anchor_quat_w)

        for i in range(len(self.cfg.body_names)):
            self.current_body_visualizers[i].visualize(self.robot_body_pos_w[:, i], self.robot_body_quat_w[:, i])
            self.goal_body_visualizers[i].visualize(self.body_pos_relative_w[:, i], self.body_quat_relative_w[:, i])


@configclass
class MotionCommandCfg(CommandTermCfg):
    """Configuration for the motion command."""

    class_type: type = MotionCommand

    asset_name: str = MISSING

    motion_file: str = MISSING
    anchor_body_name: str = MISSING
    body_names: list[str] = MISSING

    pose_range: dict[str, tuple[float, float]] = {}
    velocity_range: dict[str, tuple[float, float]] = {}

    joint_position_range: tuple[float, float] = (-0.52, 0.52)

    # --- Phase A (2026-07-02): swing ENTRY / TRANSITION / WAITING coverage --------------------
    # Deploy enters every swing from a NOMINAL STAND, waits at the windup while the ball is not
    # yet reaching, and must physically transition between swings — none of which the pure-RSI
    # scheme ever produced (teleport at every episode start AND every clip wrap). These knobs
    # close that gap; the imitation targets are anchor-RELATIVE (re-anchored to the robot's
    # current xy+yaw every step), so no-teleport starts/wraps are well-posed.
    # Fraction of TRUE episode resets that start from the robot's DEFAULT STAND pose (zero
    # velocities) instead of teleporting onto the reference clip frame (RSI).
    stand_start_prob: float = 0.25
    # Teleport the robot onto the new clip's start frame at intra-episode wraps (legacy RSI
    # behavior). False = the policy must physically transition swing->swing (the deploy case).
    wrap_teleport: bool = False
    # Pre-swing HOLD: on every swing (re)start, freeze the reference at the clip's first frame
    # for U[lo,hi] control steps (50 Hz). While held, time_to_strike sits at its per-clip
    # maximum — exactly the deploy runner's clamped "waiting for the ball" pairing.
    hold_steps_range: tuple[int, int] = (0, 100)
    # LONG-HOLD TAIL (RallyV8): with prob hold_long_prob the hold draw comes from
    # hold_long_steps_range instead of hold_steps_range — covers the deploy active-idle
    # regime (4-7 s waits) the base range misses. Exogenous, never policy-extendable.
    # hold_long_prob 0.0 = off (default; existing tasks unchanged).
    hold_long_prob: float = 0.0
    hold_long_steps_range: tuple[int, int] = (0, 0)
    # Stand-started envs get at least this much hold (they must travel stand -> windup first).
    stand_start_min_hold: int = 25
    # Yaw range (rad, uniform) applied to STAND starts (2026-07-08 rally-gate finding): deploy
    # follow-throughs leave the robot 30-55° off the strike heading and the exact-square stand
    # starts meant "yawed -> turn back" was never in the training data. (lo, hi); (0, 0) = off.
    # Pair with the hold_heading reward (the recovery income) — see hope_rewards.hold_heading.
    stand_start_yaw_range: tuple[float, float] = (0.0, 0.0)
    # XY offsets from the commanded station applied only to DEFAULT-STAND starts. The station
    # itself remains fixed, so symmetric bands train correction of strike/reset drift without
    # teaching the policy to travel between planner-selected stations. (0, 0) = off per axis.
    stand_start_x_range: tuple[float, float] = (0.0, 0.0)
    stand_start_y_range: tuple[float, float] = (0.0, 0.0)
    # STEP-SCHEDULED YAW RAMP (RallyV8): linearly scale the stand_start_yaw_range band from 0
    # to full width over this many env control steps (env.common_step_counter; one training
    # iteration = num_steps_per_env control steps). Bakes the manual ±0.2 -> ±0.6 staged-resume
    # curriculum into a single fresh run. 0 = off (full band immediately).
    stand_start_yaw_ramp_steps: int = 0
    # --- A8 (Ace recipe): post-swing initial-state distribution ------------------------------
    # Snapshot contract. ``legacy_state_v1`` stores root/q/qd in one unstratified ring and is kept
    # for frozen V12--V15 recipes. ``markov_stratified_v2`` additionally restores the action
    # history observed by the actor/action-rate reward and balances replay across
    # source-clip x follow-through-phase buckets. Existing tasks stay byte-compatible by default.
    post_swing_replay_contract: str = "legacy_state_v1"
    # Fraction of TRUE episode resets initialized from a ring buffer of the policy's OWN
    # end-of-swing states (captured at every intra-episode clip wrap — envs that physically
    # completed a swing). Teaches "start the next swing from wherever the last one left you"
    # even for single-swing episodes. Drawn AFTER stand_start_prob from the remaining resets;
    # falls back to RSI while the buffer has fewer than post_swing_min_fill entries.
    post_swing_start_prob: float = 0.0
    post_swing_buffer_size: int = 4096
    post_swing_min_fill: int = 256
    # Markov-v2 only: the final bin is the clip-wrap snapshot; preceding bins correspond to the
    # ordered hot-capture delays configured by RacketTargetCommand.
    post_swing_capture_phase_bins: int = 1
    # Markov-v3 additionally stratifies each side/phase by a pre-fault severity class. Exactly
    # three bins are used by V17 r2: safe, warning, near-boundary.
    post_swing_capture_severity_bins: int = 1
    post_swing_warning_tilt: float = 0.18
    post_swing_near_boundary_tilt: float = 0.32
    post_swing_warning_hard_margin_fraction: float = 0.12
    post_swing_near_boundary_hard_margin_fraction: float = 0.07
    # Fixed 0.08/0.30/0.80 s captures populate ordinary follow-through states. Optionally also
    # capture the first legal warning/near-boundary state in each phase so a failure before the
    # next fixed timestamp is not removed from recovery training by survivor bias.
    post_swing_risk_edge_capture: bool = False
    post_swing_risk_capture_min_age_s: float = 0.10
    post_swing_risk_capture_max_age_s: float = 1.10
    # A bucket must reach this count before balanced replay samples from it. Replay additionally
    # requires at least one eligible bucket for every motion clip.
    post_swing_min_fill_per_bucket: int = 1
    # Unitree/BeyondMimic-style failure-adaptive sampling over V17's existing local Markov
    # buckets. A uniform floor preserves coverage and phase-neighbor smoothing avoids brittle
    # single-frame spikes. Defaults are inert for every historical task.
    post_swing_failure_adaptive: bool = False
    post_swing_failure_uniform_ratio: float = 0.1
    post_swing_failure_ema_alpha: float = 0.001
    post_swing_failure_phase_neighbor_blend: float = 0.2
    # Action term whose feedback/history is part of the V17 Markov snapshot.
    post_swing_action_name: str = "joint_pos"
    # If true, ``post_swing_start_prob`` and the post-wrap hold floor are multiplied by the
    # metric-gated recovery scale written by RacketTargetCommand. Scale 0 is exact V11.
    post_swing_curriculum_scaled: bool = False
    # Build one-way strike-ability gate. Historical tasks leave this false and preserve their
    # original replay behavior. When true, capture/replay are both exactly zero until explicitly
    # unlocked; after a clean refill the true-reset probability advances monotonically.
    post_swing_ability_gate_enabled: bool = False
    post_swing_replay_ramp_probabilities: tuple[float, ...] = ()
    post_swing_replay_ramp_interval_steps: int = 8000
    # Post-swing-started envs get at least this much hold (settle follow-through -> windup).
    post_swing_min_hold: int = 25
    # If the on-policy buffer is not full yet, redirect its probability mass to DEFAULT STAND
    # instead of legacy arbitrary-frame RSI. False preserves every existing task.
    post_swing_fallback_to_stand: bool = False
    # Minimum no-teleport hold immediately after every physically completed intra-episode swing.
    # This is distinct from post_swing_min_hold, which applies only to buffer-based true resets.
    post_wrap_min_hold: int = 0
    # Wrap draws below post_wrap_min_hold are re-drawn uniform in [min, min + jitter] instead of
    # clamped onto exactly min (which made ~85% of wrap holds identical). 0 = legacy exact clamp.
    post_wrap_hold_jitter_steps: int = 0
    # Static-controller -> learned-policy entry coverage. This probability is a subset of
    # stand_start_prob, not a fourth reset branch.
    runtime_handoff_start_prob: float = 0.0
    runtime_handoff_hold_steps_range: tuple[int, int] = (0, 0)
    # Per-step per-env probability of an operator-style mid-swing clip switch (deploy parity —
    # see the venue-falls note in _update_command). 0.002 ~ one switch per ~3-4 swings. Default off.
    clip_switch_prob: float = 0.0
    # Permanently reserve the first fraction of global environment ids for clip 0, the next
    # fraction for clip 1, and so on. Resets/wraps may resample a row within that clip, but never
    # change the reserved environment's clip identity. Zero preserves every historical task.
    fixed_clip_env_fraction_per_clip: float = 0.0
    # Deterministic evaluator only: cycle each env through a frozen external serve-side sequence.
    # Empty keeps all train/play behavior byte-identical. Training YAMLs must never set this field.
    eval_clip_sequence: tuple[int, ...] = ()

    adaptive_kernel_size: int = 1
    adaptive_lambda: float = 0.8
    adaptive_uniform_ratio: float = 0.1
    adaptive_alpha: float = 0.001

    anchor_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/Command/pose")
    anchor_visualizer_cfg.markers["frame"].scale = (0.2, 0.2, 0.2)

    body_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/Command/pose")
    body_visualizer_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
