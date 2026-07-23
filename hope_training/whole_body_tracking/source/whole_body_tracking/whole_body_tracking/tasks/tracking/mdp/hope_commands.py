"""Racket-target command: the ping-pong goal on top of motion imitation.

:class:`RacketTargetCommand` rides on the :class:`~whole_body_tracking.tasks.tracking.mdp.commands.MotionCommand`.
Each swing it samples the quantities the model-based planner supplies at deploy time — a desired racket
position, a desired racket velocity, and a time-to-strike — plus the swing side (forehand/backhand),
which is locked for the duration of that swing. It also:

* holds a FIXED station target (a startup constant = the environment origin): the robot base drifts
  across a rally, and the ``fixed_station_error_xy`` observation feeds that drift back so the policy
  can re-center in place. The station never moves; this is not station planning.
* computes the ACTUAL racket state in simulation by forward kinematics through the fixed racket mount
  (wrist -> paddle center), so the reward can compare actual vs desired.
* derives the strike timing from the reference clip phase, and evaluates a simple no-spin outgoing
  ball at the strike (contact + net crossing + opponent-half first bounce) for the return rewards.

There is no measured racket feedback at deploy: the racket FK, its face normal, and the ball
evaluation are simulation-only signals used by rewards/critic, never by the actor observation.
Swing side selection is uniform per swing and follows the imitated clip (clip 0 = forehand -> +1,
clip 1 = backhand -> -1), so all four forehand/backhand transitions appear across the batch.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import matrix_from_quat, quat_apply, quat_mul, sample_uniform

from whole_body_tracking.tasks.tracking.mdp.ballistics import (
    GRAVITY as _GRAVITY,
    ballistic_velocity_from_landing as _ballistic_velocity_from_landing,
    ballistic_z_at_x as _ballistic_z_at_x,
)
from whole_body_tracking.tasks.tracking.mdp.commands import MotionCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

class RacketTargetCommand(CommandTerm):
    """Samples desired racket/station targets and computes the actual racket state by FK."""

    cfg: RacketTargetCommandCfg

    def __init__(self, cfg: RacketTargetCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]

        # Racket FK source: prefer a dedicated racket body, else (wrist pose) * (fixed mount offset).
        if cfg.racket_body_name in self.robot.body_names:
            self._racket_mode = "body"
            self._racket_body_index = self.robot.find_bodies(cfg.racket_body_name, preserve_order=True)[0][0]
            self._wrist_body_index = -1
        else:
            assert cfg.wrist_body_name in self.robot.body_names, (
                f"RacketTargetCommand: neither racket body '{cfg.racket_body_name}' nor wrist body "
                f"'{cfg.wrist_body_name}' found on asset '{cfg.asset_name}'."
            )
            self._racket_mode = "wrist_offset"
            self._racket_body_index = -1
            self._wrist_body_index = self.robot.find_bodies(cfg.wrist_body_name, preserve_order=True)[0][0]
        self._mount_offset = torch.tensor(cfg.mount_offset, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )
        self._mount_quat = torch.tensor(cfg.mount_quat, dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )

        self._motion_term: MotionCommand | None = None

        # Desired (sampled) targets, world frame.
        self.racket_target_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_normal_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_target_normal_w[:, 2] = 1.0
        self.incoming_ball_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.swing_sign = torch.ones(self.num_envs, device=self.device)

        # Actual racket state (FK), world frame.
        self.racket_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_quat_w = torch.zeros(self.num_envs, 4, device=self.device)
        self.racket_quat_w[:, 0] = 1.0
        self.racket_lin_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_normal_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.racket_normal_w[:, 2] = 1.0

        # Strike timing.
        self.time_to_strike = torch.zeros(self.num_envs, device=self.device)
        self.pre_strike = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.strike_window = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        # Reward helper signals.
        self.racket_target_distance = torch.zeros(self.num_envs, device=self.device)
        self.feet_contact_frac = torch.zeros(self.num_envs, device=self.device)
        # No-spin return evaluation caches (one-shot at the exact strike frame).
        self.strike_fired = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.ball_contact = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.ball_net_cross = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.ball_on_opponent = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.impact_ball_out_vel_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.impact_ball_out_error = torch.zeros(self.num_envs, device=self.device)

        # Per-clip strike phase / target boxes (resolved lazily once the motion term is available).
        self._strike_phase_per_clip = None
        self._swing_side_per_clip = (
            torch.tensor([float(s) for s in cfg.swing_side_per_clip], device=self.device)
            if cfg.swing_side_per_clip
            else None
        )
        self._pos_box = _boxes_to_tensor(cfg.racket_pos_range_per_clip, self.device)  # (C,3,2) or None
        self._vel_box = _boxes_to_tensor(cfg.racket_vel_range_per_clip, self.device)
        self._mount_sign_per_clip = (
            torch.tensor([float(s) for s in cfg.mount_normal_sign_per_clip], device=self.device)
            if cfg.mount_normal_sign_per_clip
            else None
        )

        # Feet resolution for contact fraction (degrades to 0 if it cannot resolve — never crashes).
        try:
            self._contact_sensor = env.scene.sensors["contact_forces"]
        except (KeyError, AttributeError, TypeError):
            self._contact_sensor = None
        self._foot_idx_contact: list[int] = []
        if self._contact_sensor is not None:
            sensor_bodies = list(self._contact_sensor.body_names)
            self._foot_idx_contact = [sensor_bodies.index(n) for n in cfg.feet_body_names if n in sensor_bodies]

        for key in ("racket_pos_error", "racket_vel_error", "time_to_strike", "return_success"):
            self.metrics[key] = torch.zeros(self.num_envs, device=self.device)

    # --- helpers -------------------------------------------------------------------------------- #
    def _motion(self) -> MotionCommand:
        if self._motion_term is None:
            self._motion_term = self._env.command_manager.get_term(self.cfg.motion_command_name)
        return self._motion_term

    @property
    def base_pos_w(self) -> torch.Tensor:
        return self.robot.data.root_pos_w

    @property
    def base_quat_w(self) -> torch.Tensor:
        return self.robot.data.root_quat_w

    @property
    def fixed_station_w(self) -> torch.Tensor:
        """Fixed startup station XY = the environment origin plus a nominal offset (constant)."""
        off = torch.tensor(self.cfg.station_nominal_offset_xy, device=self.device)
        return self._env.scene.env_origins[:, :2] + off

    @property
    def command(self) -> torch.Tensor:
        """Raw target vector (world): [pos(3), vel(3), tts(1), station(2), swing(1)]."""
        return torch.cat(
            [
                self.racket_target_pos_w,
                self.racket_target_vel_w,
                self.time_to_strike.unsqueeze(-1),
                self.fixed_station_w,
                self.swing_sign.unsqueeze(-1),
            ],
            dim=-1,
        )

    # --- observation accessors ------------------------------------------------------- #
    def base_forward_xy(self) -> torch.Tensor:
        """Base forward unit vector e_base,x, world XY (2)."""
        fwd = quat_apply(
            self.base_quat_w, torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, 3)
        )[:, :2]
        return fwd / (torch.norm(fwd, dim=-1, keepdim=True) + 1e-6)

    def fixed_station_error_xy(self) -> torch.Tensor:
        """Fixed startup station XY minus current base XY, world frame (2)."""
        return self.fixed_station_w - self.base_pos_w[:, :2]

    def racket_target_rel_base_w(self) -> torch.Tensor:
        """Target racket position minus base position, world frame (3)."""
        return self.racket_target_pos_w - self.base_pos_w

    # --- sampling ------------------------------------------------------------------------------- #
    def _sample_targets(self, env_ids: torch.Tensor):
        motion = self._motion()
        n = len(env_ids)
        clip = motion.clip_id[env_ids] if motion._multiseg else torch.zeros(n, dtype=torch.long, device=self.device)
        station = self.fixed_station_w[env_ids]  # (n, 2)

        pos_box = self._resolve_box(self._pos_box, clip, self.cfg.racket_pos_range)  # (n, 3, 2)
        vel_box = self._resolve_box(self._vel_box, clip, self.cfg.racket_vel_range)

        # Position: x/y are STATION-RELATIVE (fixed striking plane in front + side band), z absolute.
        pos = sample_uniform(pos_box[..., 0], pos_box[..., 1], (n, 3), self.device)
        pos[:, 0] = station[:, 0] + pos[:, 0]
        pos[:, 1] = station[:, 1] + pos[:, 1]
        self.racket_target_pos_w[env_ids] = pos

        if self.cfg.racket_velocity_mode == "range":
            vel = sample_uniform(vel_box[..., 0], vel_box[..., 1], (n, 3), self.device)
        elif self.cfg.racket_velocity_mode == "ballistic_landing":
            vel = self._sample_ballistic_target_velocity(env_ids, pos, station)
        else:
            raise ValueError(f"Unsupported racket_velocity_mode: {self.cfg.racket_velocity_mode}")
        self.racket_target_vel_w[env_ids] = vel

        incoming_vel = self._sample_incoming_ball_velocity(env_ids, pos)
        self.incoming_ball_vel_w[env_ids] = incoming_vel

        # Desired blade normal: normal impulse direction that turns the nominal incoming ball into
        # the desired outgoing ball velocity. This is still a no-spin approximation, but it avoids
        # the old over-coupling "face normal == outgoing velocity direction" that produced large
        # MuJoCo lateral rebounds.
        normal = vel - incoming_vel
        self.racket_target_normal_w[env_ids] = normal / (torch.norm(normal, dim=-1, keepdim=True) + 1e-6)

        # Swing side follows per-clip metadata when provided. The historical two-clip default remains
        # clip 0 = forehand, everything else = backhand for compatibility with older configs.
        if self._swing_side_per_clip is not None and motion._multiseg:
            if self._swing_side_per_clip.shape[0] != motion.motion.num_segments:
                raise ValueError(
                    "RacketTargetCommandCfg.swing_side_per_clip length "
                    f"({self._swing_side_per_clip.shape[0]}) must match motion clips "
                    f"({motion.motion.num_segments})."
                )
            self.swing_sign[env_ids] = self._swing_side_per_clip[clip]
        elif motion._multiseg:
            self.swing_sign[env_ids] = torch.where(clip == 0, 1.0, -1.0)
        else:
            self.swing_sign[env_ids] = 1.0

    def _resolve_box(self, per_clip, clip: torch.Tensor, shared_range) -> torch.Tensor:
        """Return an (n, 3, 2) [lo, hi] box per env: per-clip if configured, else the shared box."""
        if per_clip is not None:
            return per_clip[clip]
        shared = torch.tensor(shared_range, dtype=torch.float32, device=self.device)  # (3, 2)
        return shared.unsqueeze(0).expand(len(clip), 3, 2)

    def _sample_ballistic_target_velocity(
        self,
        env_ids: torch.Tensor,
        target_pos_w: torch.Tensor,
        station_xy_w: torch.Tensor,
    ) -> torch.Tensor:
        """Sample a target velocity that is self-consistent with the simplified return evaluator."""
        n = len(env_ids)
        origins = self._env.scene.env_origins[env_ids]
        p0 = target_pos_w - origins
        center_y = station_xy_w[:, 1] - origins[:, 1]

        x_lo, x_hi = (float(v) for v in self.cfg.ballistic_land_x_range)
        y_lo, y_hi = (float(v) for v in self.cfg.ballistic_land_y_range)
        t_lo, t_hi = (float(v) for v in self.cfg.ballistic_flight_time_range)
        net_x = float(self.cfg.table_near_x) + float(self.cfg.net_x)
        net_top = float(self.cfg.table_surface_z) + float(self.cfg.net_height) + float(self.cfg.net_margin)

        out = torch.zeros((n, 3), dtype=torch.float32, device=self.device)
        resolved = torch.zeros(n, dtype=torch.bool, device=self.device)
        attempts = max(int(self.cfg.ballistic_sample_attempts), 1)

        for _ in range(attempts):
            flight_time = sample_uniform(t_lo, t_hi, (n,), self.device)
            land_x = sample_uniform(x_lo, x_hi, (n,), self.device)
            land_y = center_y + sample_uniform(y_lo, y_hi, (n,), self.device)
            land_xy = torch.stack((land_x, land_y), dim=-1)
            candidate = _ballistic_velocity_from_landing(
                p0, land_xy, flight_time, float(self.cfg.table_surface_z)
            )
            t_net = (net_x - p0[:, 0]) / candidate[:, 0].clamp_min(1.0e-3)
            z_net = _ballistic_z_at_x(p0, candidate, net_x)
            valid = (
                (candidate[:, 0] > float(self.cfg.ballistic_min_forward_speed))
                & (t_net > 0.0)
                & (z_net > net_top)
            )
            take = (~resolved) & valid
            out[take] = candidate[take]
            resolved |= take
            if bool(torch.all(resolved)):
                break

        if not bool(torch.all(resolved)):
            flight_time = torch.full((n,), 0.5 * (t_lo + t_hi), dtype=torch.float32, device=self.device)
            land_x = torch.full((n,), 0.5 * (x_lo + x_hi), dtype=torch.float32, device=self.device)
            land_y = center_y
            land_xy = torch.stack((land_x, land_y), dim=-1)
            fallback = _ballistic_velocity_from_landing(
                p0, land_xy, flight_time, float(self.cfg.table_surface_z)
            )
            out[~resolved] = fallback[~resolved]
        return out

    def _sample_incoming_ball_velocity(self, env_ids: torch.Tensor, target_pos_w: torch.Tensor) -> torch.Tensor:
        """Nominal incoming ball velocity at the strike point.

        The actor does not observe this extra signal; it is used only to shape the critic/rewards
        around a more realistic moving-racket impact model. The distribution mirrors the MuJoCo
        evaluator's serve geometry closely enough to make the desired racket face normal meaningful.
        """
        n = len(env_ids)
        origins = self._env.scene.env_origins[env_ids]
        x_lo, x_hi = (float(v) for v in self.cfg.incoming_origin_x_range)
        y_lo, y_hi = (float(v) for v in self.cfg.incoming_origin_y_jitter_range)
        z_lo, z_hi = (float(v) for v in self.cfg.incoming_origin_z_above_table_range)
        t_lo, t_hi = (float(v) for v in self.cfg.incoming_flight_time_range)

        origin = torch.zeros((n, 3), dtype=torch.float32, device=self.device)
        origin[:, 0] = origins[:, 0] + float(self.cfg.table_near_x) + sample_uniform(x_lo, x_hi, (n,), self.device)
        origin[:, 1] = target_pos_w[:, 1] + sample_uniform(y_lo, y_hi, (n,), self.device)
        origin[:, 2] = origins[:, 2] + float(self.cfg.table_surface_z) + sample_uniform(z_lo, z_hi, (n,), self.device)

        flight_time = sample_uniform(t_lo, t_hi, (n,), self.device)
        accel = torch.tensor([0.0, 0.0, -_GRAVITY], dtype=torch.float32, device=self.device)
        v0 = (target_pos_w - origin) / flight_time.unsqueeze(-1) - 0.5 * accel * flight_time.unsqueeze(-1)
        return v0 + accel * flight_time.unsqueeze(-1)

    def _moving_racket_impact_velocity(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Approximate no-spin ball velocity after contact with a moving racket plane."""
        normal = self.racket_normal_w / (torch.norm(self.racket_normal_w, dim=-1, keepdim=True) + 1e-6)
        rel_in = self.incoming_ball_vel_w - self.racket_lin_vel_w
        rel_n = torch.sum(rel_in * normal, dim=-1, keepdim=True)
        rel_t = rel_in - rel_n * normal
        rel_out = float(self.cfg.paddle_tangent_retain) * rel_t - float(self.cfg.paddle_restitution) * rel_n * normal
        return self.racket_lin_vel_w + rel_out, rel_n.squeeze(-1)

    def _resample_command(self, env_ids: Sequence[int]):
        if len(env_ids) == 0:
            return
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self._sample_targets(env_ids)

    # --- per-step updates ----------------------------------------------------------------------- #
    def _compute_strike_timing(self):
        motion = self._motion()
        ml = motion.motion
        if self._strike_phase_per_clip is None:
            sp = tuple(self.cfg.strike_phase_per_clip)
            if sp and len(sp) == ml.num_segments:
                self._strike_phase_per_clip = torch.tensor([float(x) for x in sp], device=self.device)
            else:
                self._strike_phase_per_clip = torch.full((ml.num_segments,), float(self.cfg.strike_phase), device=self.device)
        clip = motion.clip_id
        seg_start = ml.seg_start[clip]
        seg_len = ml.seg_len[clip]
        phase = self._strike_phase_per_clip[clip]
        strike_step = seg_start + (phase * (seg_len - 1).float()).round().long()
        self.time_to_strike = (strike_step - motion.time_steps).float() * self._env.step_dt
        self.pre_strike = self.time_to_strike > 0.0
        self.strike_window = self.time_to_strike.abs() <= self.cfg.strike_window_s

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
        # Face normal = a chosen local axis of the racket frame, times the striking-face sign (the
        # forehand and backhand strike with opposite faces).
        axis_w = matrix_from_quat(self.racket_quat_w)[:, :, self.cfg.mount_normal_axis]
        if self._mount_sign_per_clip is not None and self._motion()._multiseg:
            clip = self._motion().clip_id.clamp(max=self._mount_sign_per_clip.shape[0] - 1)
            sign = self._mount_sign_per_clip[clip].unsqueeze(-1)
        else:
            sign = self.cfg.mount_normal_sign
        self.racket_normal_w = axis_w * sign

    def _update_feet_contact(self):
        if self._contact_sensor is None or not self._foot_idx_contact:
            return
        forces = torch.norm(self._contact_sensor.data.net_forces_w[:, self._foot_idx_contact, :], dim=-1)
        in_contact = (forces > self.cfg.contact_force_threshold).float()
        self.feet_contact_frac = in_contact.mean(dim=-1)

    def _evaluate_return(self):
        """Simple no-spin outgoing-ball evaluation at the exact strike frame (contact/net/bounce).

        The paddle is assumed to carry the ball off at the achieved racket velocity (no spin). The
        outgoing flight is a gravity-only ballistic arc; net clearance and the first table bounce are
        solved in closed form. All quantities are example approximations for training shaping.
        """
        exact = self.time_to_strike.abs() <= (0.5 * self._env.step_dt + 1e-6)
        self.strike_fired = exact

        pos_err = torch.norm(self.racket_pos_w - self.racket_target_pos_w, dim=-1)
        self.racket_target_distance = pos_err
        # Contact requires the racket to be near the target and moving through the strike.  The
        # target-velocity mode avoids a singular "toward the target point" test when the racket is
        # already at the contact point.
        if self.cfg.contact_approach_mode == "target_velocity":
            approach_dir = self.racket_target_vel_w / (torch.norm(self.racket_target_vel_w, dim=-1, keepdim=True) + 1e-6)
        elif self.cfg.contact_approach_mode == "target_point":
            to_target = self.racket_target_pos_w - self.racket_pos_w
            approach_dir = to_target / (torch.norm(to_target, dim=-1, keepdim=True) + 1e-6)
        else:
            raise ValueError(f"Unsupported contact_approach_mode: {self.cfg.contact_approach_mode}")
        approach = torch.sum(self.racket_lin_vel_w * approach_dir, dim=-1)
        contact = exact & (pos_err < self.cfg.contact_radius) & (approach > self.cfg.min_approach_speed)

        if self.cfg.return_model == "racket_velocity":
            out_vel = self.racket_lin_vel_w
        elif self.cfg.return_model == "moving_racket_impact":
            out_vel, rel_normal_speed = self._moving_racket_impact_velocity()
            contact = contact & ((-rel_normal_speed) > float(self.cfg.min_normal_closing_speed))
        else:
            raise ValueError(f"Unsupported return_model: {self.cfg.return_model}")
        self.impact_ball_out_vel_w = out_vel
        self.impact_ball_out_error = torch.norm(out_vel - self.racket_target_vel_w, dim=-1)

        # Outgoing ballistic arc from the strike point (env-local frame) at the predicted ball velocity.
        p0 = self.racket_pos_w - self._env.scene.env_origins
        v = out_vel
        x0, y0, z0 = p0[:, 0], p0[:, 1], p0[:, 2]
        vx, vy, vz = v[:, 0], v[:, 1], v[:, 2]

        near_x = float(self.cfg.table_near_x)
        net_x = near_x + float(self.cfg.net_x)
        far_x = near_x + float(self.cfg.table_length)
        half_w = 0.5 * float(self.cfg.table_width)
        surface_z = float(self.cfg.table_surface_z)
        net_top = surface_z + float(self.cfg.net_height) + float(self.cfg.net_margin)
        center_y = self.fixed_station_w[:, 1] - self._env.scene.env_origins[:, 1]  # env-local station y

        # Net crossing height (ball travels in +x toward the opponent).
        moving_fwd = vx > 0.1
        t_net = (net_x - x0) / vx.clamp_min(1e-3)
        z_net = z0 + vz * t_net - 0.5 * _GRAVITY * t_net**2
        net_cross = contact & moving_fwd & (t_net > 0) & (z_net > net_top)

        # First bounce on the table surface (descending root of the ballistic arc).
        disc = (vz**2 + 2.0 * _GRAVITY * (z0 - surface_z)).clamp_min(0.0)
        t_bounce = (vz + torch.sqrt(disc)) / _GRAVITY
        land_x = x0 + vx * t_bounce
        land_y = y0 + vy * t_bounce
        on_opponent = net_cross & (land_x > net_x) & (land_x < far_x) & ((land_y - center_y).abs() < half_w)

        self.ball_contact = contact
        self.ball_net_cross = net_cross
        self.ball_on_opponent = on_opponent

    def _update_metrics(self):
        # Timing + FK must be fresh before the reward reads them (motion updated first this step).
        self._compute_strike_timing()
        self._compute_racket_state()
        self._update_feet_contact()
        self._evaluate_return()
        self.metrics["racket_pos_error"] = torch.where(
            self.strike_window, self.racket_target_distance, self.metrics["racket_pos_error"]
        )
        self.metrics["racket_vel_error"] = torch.where(
            self.strike_window,
            torch.norm(self.racket_lin_vel_w - self.racket_target_vel_w, dim=-1),
            self.metrics["racket_vel_error"],
        )
        self.metrics["time_to_strike"] = self.time_to_strike
        self.metrics["return_success"] = torch.where(
            self.strike_fired, self.ball_on_opponent.float(), self.metrics["return_success"]
        )

    def _update_command(self):
        self._compute_strike_timing()
        # Re-sample the target at each new swing (the motion command sets just_resampled this step
        # when it wrapped a swing). Reset-time resampling is handled by the manager's reset -> _resample.
        motion = self._motion()
        wrapped = torch.where(motion.just_resampled)[0]
        if len(wrapped) > 0:
            self._resample_command(wrapped)

    def _set_debug_vis_impl(self, debug_vis: bool):
        pass

    def _debug_vis_callback(self, event):
        pass


def _boxes_to_tensor(per_clip, device):
    """Convert ((xlo,xhi),(ylo,yhi),(zlo,zhi)) x num_clips into an (C, 3, 2) tensor, or None."""
    if per_clip is None:
        return None
    return torch.tensor(
        [[[float(lo), float(hi)] for (lo, hi) in clip_rng] for clip_rng in per_clip],
        dtype=torch.float32,
        device=device,
    )


@configclass
class RacketTargetCommandCfg(CommandTermCfg):
    """Configuration for :class:`RacketTargetCommand`."""

    class_type: type = RacketTargetCommand

    asset_name: str = MISSING
    motion_command_name: str = "motion"
    # Targets are re-sampled per swing (on wrap / reset), not on a timer.
    resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)

    # --- racket mount FK ---
    racket_body_name: str = "pingpang_red_Link"
    wrist_body_name: str = "right_wrist_yaw_Link"
    mount_offset: tuple[float, float, float] = (0.21, 0.032, 0.032)
    mount_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    mount_normal_axis: int = 1  # racket-local +Y is the blade face normal
    mount_normal_sign: float = 1.0
    # Per-clip striking-face sign (forehand and backhand strike with opposite faces), e.g. (1.0, -1.0).
    mount_normal_sign_per_clip: tuple = ()
    # Per-clip swing side exposed to the actor: forehand +1, backhand -1.
    swing_side_per_clip: tuple = ()

    # --- fixed station (startup constant) ---
    station_nominal_offset_xy: tuple[float, float] = (0.0, 0.0)

    # --- feet (for the contact fraction used by the follow-through/recovery reward) ---
    feet_body_names: tuple[str, ...] = ("left_ankle_roll_Link", "right_ankle_roll_Link")
    contact_force_threshold: float = 10.0

    # --- strike timing (fraction of the reference clip at which the paddle meets the ball) ---
    strike_phase: float = 0.5
    strike_phase_per_clip: tuple = ()  # e.g. (0.47, 0.33); empty -> scalar strike_phase for every clip
    strike_window_s: float = 0.12  # half-window in which the racket-tracking rewards are active

    # --- racket target boxes ---
    # x/y are STATION-RELATIVE (fixed striking plane in front + swing-side band), z is absolute height.
    racket_pos_range: tuple = ((0.45, 0.55), (-0.35, 0.35), (0.7, 1.1))
    racket_vel_range: tuple = ((1.0, 2.5), (-1.5, 1.5), (0.0, 1.0))
    racket_velocity_mode: str = "range"  # range | ballistic_landing
    # Optional per-clip boxes (indexed by clip_id 0=forehand, 1=backhand). None -> shared boxes above.
    racket_pos_range_per_clip: tuple | None = None
    racket_vel_range_per_clip: tuple | None = None

    # --- no-spin return evaluation (example table placement in the env frame; tune to your scene) ---
    contact_radius: float = 0.095   # racket radius + ball radius
    min_approach_speed: float = 0.3  # racket must be moving into the target this fast to "contact"
    contact_approach_mode: str = "target_point"  # target_point | target_velocity
    return_model: str = "moving_racket_impact"  # racket_velocity | moving_racket_impact
    paddle_restitution: float = 0.654
    paddle_tangent_retain: float = 0.85
    min_normal_closing_speed: float = 0.2
    incoming_origin_x_range: tuple = (1.9, 2.4)
    incoming_origin_y_jitter_range: tuple = (-0.15, 0.15)
    incoming_origin_z_above_table_range: tuple = (0.15, 0.35)
    incoming_flight_time_range: tuple = (0.75, 1.0)
    table_near_x: float = 0.5       # x of the robot's own table end (robot sits behind it)
    table_surface_z: float = 0.76   # table surface height above the env origin
    table_length: float = 2.74      # ITTF table length (+x)
    table_width: float = 1.525      # ITTF table width (y)
    net_x: float = 1.37             # net plane from the near edge
    net_height: float = 0.1525      # net height above the surface
    net_margin: float = 0.02        # required clearance above the net top
    ballistic_flight_time_range: tuple = (0.45, 0.75)
    ballistic_land_x_range: tuple = (2.05, 2.95)
    ballistic_land_y_range: tuple = (-0.45, 0.45)
    ballistic_min_forward_speed: float = 0.3
    ballistic_sample_attempts: int = 8
