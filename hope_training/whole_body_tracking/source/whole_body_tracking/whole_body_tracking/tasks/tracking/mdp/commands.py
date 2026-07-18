"""Motion-imitation command: replays a reference clip and exposes tracking targets.

:class:`MotionCommand` owns the per-env reference clock and drives the imitation reward. It supports
one or more clips concatenated on a single time axis (the HOPE default passes a forehand and a
backhand clip); each env imitates one clip ("segment") at a time, chosen uniformly per swing so all
forehand/backhand transitions appear in training. The racket-target command (``hope_commands.py``)
rides on top of this term and reads ``clip_id`` / ``time_steps`` / ``in_hold``.

Continuous-rally lifecycle: at a clip wrap the robot is NOT teleported — it must physically
carry its body from the previous swing's end into the next swing's windup (``wrap_teleport=False``).
Only a true episode reset re-initializes the robot (default stand, or reference-state-init onto the
clip frame).

NPZ schema consumed by :class:`MotionLoader` (per clip; keep small, see the motion YAML sidecars):

    fps            : scalar frames-per-second
    joint_pos      : float32 [T, 31]         joint order = canonical joint order
    joint_vel      : float32 [T, 31]
    body_pos_w     : float32 [T, B, 3]        tracked bodies only, in ``body_names`` order
    body_quat_w    : float32 [T, B, 4]        (w, x, y, z)
    body_lin_vel_w : float32 [T, B, 3]
    body_ang_vel_w : float32 [T, B, 3]

``B`` is the number of tracked bodies (``len(cfg.body_names)``) and the body axis is stored in
``body_names`` order, so no re-indexing by articulation is needed.
"""

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

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class MotionLoader:
    """Loads one or more reference clips onto a single concatenated time axis.

    Body arrays are stored exactly as they appear in the npz (tracked bodies, in ``body_names``
    order). Passing several files concatenates them along time and records per-clip ``seg_start`` /
    ``seg_len`` so a command can step and wrap within one clip at a time.
    """

    def __init__(self, motion_file, num_bodies: int, device: str = "cpu"):
        files = [motion_file] if isinstance(motion_file, str) else list(motion_file)
        assert len(files) >= 1, "MotionLoader needs at least one motion file"
        jp, jv, bp, bq, bl, ba = [], [], [], [], [], []
        seg_lens = []
        self.fps = None
        for f in files:
            assert os.path.isfile(f), f"Invalid motion file path: {f}"
            data = np.load(f)
            if self.fps is None:
                self.fps = float(data["fps"])
            jp.append(torch.tensor(data["joint_pos"], dtype=torch.float32, device=device))
            jv.append(torch.tensor(data["joint_vel"], dtype=torch.float32, device=device))
            bp.append(torch.tensor(data["body_pos_w"], dtype=torch.float32, device=device))
            bq.append(torch.tensor(data["body_quat_w"], dtype=torch.float32, device=device))
            bl.append(torch.tensor(data["body_lin_vel_w"], dtype=torch.float32, device=device))
            ba.append(torch.tensor(data["body_ang_vel_w"], dtype=torch.float32, device=device))
            if bp[-1].shape[1] != num_bodies:
                raise ValueError(
                    f"Motion file {f} stores {bp[-1].shape[1]} bodies but the task tracks {num_bodies} "
                    "(cfg.body_names). The body axis must match, in body_names order."
                )
            seg_lens.append(jp[-1].shape[0])
        self.joint_pos = torch.cat(jp, dim=0)
        self.joint_vel = torch.cat(jv, dim=0)
        self.body_pos_w = torch.cat(bp, dim=0)
        self.body_quat_w = torch.cat(bq, dim=0)
        self.body_lin_vel_w = torch.cat(bl, dim=0)
        self.body_ang_vel_w = torch.cat(ba, dim=0)
        self.time_step_total = self.joint_pos.shape[0]
        self.num_segments = len(seg_lens)
        self.seg_len = torch.tensor(seg_lens, dtype=torch.long, device=device)
        self.seg_start = torch.zeros(self.num_segments, dtype=torch.long, device=device)
        if self.num_segments > 1:
            self.seg_start[1:] = torch.cumsum(self.seg_len, dim=0)[:-1]


class MotionCommand(CommandTerm):
    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]
        self.robot_anchor_body_index = self.robot.body_names.index(self.cfg.anchor_body_name)
        self.motion_anchor_body_index = self.cfg.body_names.index(self.cfg.anchor_body_name)
        # Articulation indices of the tracked bodies (for reading the robot's own body states).
        self.body_indexes = torch.tensor(
            self.robot.find_bodies(self.cfg.body_names, preserve_order=True)[0], dtype=torch.long, device=self.device
        )

        self.motion = MotionLoader(self.cfg.motion_file, len(self.cfg.body_names), device=self.device)

        self.time_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._multiseg = self.motion.num_segments > 1
        # Which clip (swing side) each env is currently imitating: 0 = forehand, 1 = backhand.
        self.clip_id = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # "This env just started a new swing this step" — consumed by the racket-target command.
        self.just_resampled = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        # Pre-swing hold: while hold_counter > 0 the reference clock is frozen at the swing's first
        # frame ("waiting for the ball"); ``in_hold`` is exposed for rewards.
        self.hold_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._resampling_from_wrap = False

        # Anchor-re-anchored reference body targets (recomputed each step in _update_command).
        self.body_pos_relative_w = torch.zeros(self.num_envs, len(cfg.body_names), 3, device=self.device)
        self.body_quat_relative_w = torch.zeros(self.num_envs, len(cfg.body_names), 4, device=self.device)
        self.body_quat_relative_w[:, :, 0] = 1.0

        # A small set of logging metrics (the training runner logs success_rate only, but the base
        # CommandTerm expects this dict to exist and be populated each step).
        for key in ("error_joint_pos", "error_body_pos", "error_body_rot", "motion_phase", "in_hold"):
            self.metrics[key] = torch.zeros(self.num_envs, device=self.device)

    # --- reference (target) state, hold-aware ------------------------------------------------- #
    @property
    def command(self) -> torch.Tensor:
        """Reference joint stream [joint_pos(31), joint_vel(31)] — critic/privileged use only."""
        return torch.cat([self.joint_pos, self.joint_vel], dim=1)

    @property
    def in_hold(self) -> torch.Tensor:
        return self.hold_counter > 0

    @property
    def joint_pos(self) -> torch.Tensor:
        # During the hold the reference is the default stand pose (a frozen, settled "ready"), not the
        # clip's first-frame windup transient.
        jp = self.motion.joint_pos[self.time_steps]
        dq = self.robot.data.default_joint_pos
        return torch.where(self.in_hold[:, None], dq, jp)

    @property
    def joint_vel(self) -> torch.Tensor:
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

    # --- robot state (tracked bodies) --------------------------------------------------------- #
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

    def _update_metrics(self):
        self.metrics["error_joint_pos"] = torch.norm(self.joint_pos - self.robot_joint_pos, dim=-1)
        self.metrics["error_body_pos"] = torch.norm(
            self.body_pos_relative_w - self.robot_body_pos_w, dim=-1
        ).mean(dim=-1)
        self.metrics["error_body_rot"] = quat_error_magnitude(
            self.body_quat_relative_w, self.robot_body_quat_w
        ).mean(dim=-1)
        self.metrics["in_hold"] = self.in_hold.float()
        if self._multiseg:
            seg_start = self.motion.seg_start[self.clip_id]
            seg_len = self.motion.seg_len[self.clip_id].clamp(min=2)
            self.metrics["motion_phase"] = (self.time_steps - seg_start).float() / (seg_len - 1).float()
        else:
            self.metrics["motion_phase"] = self.time_steps.float() / max(self.motion.time_step_total - 1, 1)

    def _sample_clip_and_start(self, env_ids: torch.Tensor, at_segment_start: bool):
        """Pick a clip per env (uniform; covers all forehand/backhand transitions) and a start frame."""
        n = len(env_ids)
        if self._multiseg:
            new_clip = torch.randint(0, self.motion.num_segments, (n,), device=self.device)
        else:
            new_clip = torch.zeros(n, dtype=torch.long, device=self.device)
        self.clip_id[env_ids] = new_clip
        seg_start = self.motion.seg_start[new_clip]
        if at_segment_start:
            self.time_steps[env_ids] = seg_start
        else:
            # Reference-state-init: start at a uniformly random frame inside the chosen segment.
            seg_len = self.motion.seg_len[new_clip]
            frac = sample_uniform(0.0, 1.0, (n,), device=self.device)
            self.time_steps[env_ids] = seg_start + (frac * (seg_len - 1).float()).long()

    def _resample_command(self, env_ids: Sequence[int]):
        if len(env_ids) == 0:
            return
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)

        # Pre-swing hold (freeze the reference at the swing's first frame for U[lo, hi] control steps).
        lo, hi = self.cfg.hold_steps_range
        self.hold_counter[env_ids] = torch.randint(int(lo), int(hi) + 1, (len(env_ids),), device=self.device)

        # Intra-episode clip WRAP: pick the next swing side, start at its first frame, and do NOT
        # teleport — the policy physically carries its body between swings (deploy case).
        if self._resampling_from_wrap and not self.cfg.wrap_teleport:
            self._sample_clip_and_start(env_ids, at_segment_start=True)
            return

        # TRUE episode reset: DEFAULT STAND (deploy entry) or reference-state-init (RSI) onto the clip.
        u = torch.rand(len(env_ids), device=self.device)
        stand_mask = u < float(self.cfg.stand_start_prob)
        stand_ids = env_ids[stand_mask]
        rsi_ids = env_ids[~stand_mask]

        if len(stand_ids) > 0:
            self._sample_clip_and_start(stand_ids, at_segment_start=True)
            default_root = self.robot.data.default_root_state[stand_ids].clone()
            default_root[:, :3] += self._env.scene.env_origins[stand_ids]
            default_root[:, 7:] = 0.0  # zero linear/angular velocity
            self.robot.write_root_state_to_sim(default_root, env_ids=stand_ids)
            self.robot.write_joint_state_to_sim(
                self.robot.data.default_joint_pos[stand_ids],
                torch.zeros_like(self.robot.data.default_joint_vel[stand_ids]),
                env_ids=stand_ids,
            )
            # Give stand starts time to settle before the clip advances.
            self.hold_counter[stand_ids] = torch.clamp(
                self.hold_counter[stand_ids], min=int(self.cfg.stand_start_min_hold)
            )

        if len(rsi_ids) == 0:
            return
        self._sample_clip_and_start(rsi_ids, at_segment_start=False)

        root_pos = self.body_pos_w[:, 0].clone()
        root_ori = self.body_quat_w[:, 0].clone()
        root_lin_vel = self.body_lin_vel_w[:, 0].clone()
        root_ang_vel = self.body_ang_vel_w[:, 0].clone()

        range_list = [self.cfg.pose_range.get(k, (0.0, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        ranges = torch.tensor(range_list, device=self.device)
        rand = sample_uniform(ranges[:, 0], ranges[:, 1], (len(rsi_ids), 6), device=self.device)
        root_pos[rsi_ids] += rand[:, 0:3]
        root_ori[rsi_ids] = quat_mul(quat_from_euler_xyz(rand[:, 3], rand[:, 4], rand[:, 5]), root_ori[rsi_ids])

        range_list = [self.cfg.velocity_range.get(k, (0.0, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        ranges = torch.tensor(range_list, device=self.device)
        rand = sample_uniform(ranges[:, 0], ranges[:, 1], (len(rsi_ids), 6), device=self.device)
        root_lin_vel[rsi_ids] += rand[:, :3]
        root_ang_vel[rsi_ids] += rand[:, 3:]

        joint_pos = self.joint_pos.clone()
        joint_vel = self.joint_vel.clone()
        joint_pos += sample_uniform(*self.cfg.joint_position_range, joint_pos.shape, joint_pos.device)
        limits = self.robot.data.soft_joint_pos_limits[rsi_ids]
        joint_pos[rsi_ids] = torch.clip(joint_pos[rsi_ids], limits[:, :, 0], limits[:, :, 1])
        self.robot.write_joint_state_to_sim(joint_pos[rsi_ids], joint_vel[rsi_ids], env_ids=rsi_ids)
        self.robot.write_root_state_to_sim(
            torch.cat(
                [root_pos[rsi_ids], root_ori[rsi_ids], root_lin_vel[rsi_ids], root_ang_vel[rsi_ids]], dim=-1
            ),
            env_ids=rsi_ids,
        )

    def _update_command(self):
        held = self.in_hold
        self.hold_counter = torch.clamp(self.hold_counter - 1, min=0)
        self.time_steps += (~held).long()

        if self._multiseg:
            seg_end = self.motion.seg_start[self.clip_id] + self.motion.seg_len[self.clip_id]
            env_ids = torch.where(self.time_steps >= seg_end)[0]
        else:
            env_ids = torch.where(self.time_steps >= self.motion.time_step_total)[0]

        self.just_resampled = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        if len(env_ids) > 0:
            self.just_resampled[env_ids] = True
            self._resampling_from_wrap = True
            try:
                self._resample_command(env_ids)
            finally:
                self._resampling_from_wrap = False

        # Re-anchor the reference body targets onto the robot's current xy + yaw so imitation is
        # invariant to where the robot actually is (only the anchor's z uses the reference height).
        n = len(self.cfg.body_names)
        anchor_pos = self.anchor_pos_w[:, None, :].repeat(1, n, 1)
        anchor_quat = self.anchor_quat_w[:, None, :].repeat(1, n, 1)
        robot_anchor_pos = self.robot_anchor_pos_w[:, None, :].repeat(1, n, 1)
        robot_anchor_quat = self.robot_anchor_quat_w[:, None, :].repeat(1, n, 1)

        delta_pos = robot_anchor_pos.clone()
        delta_pos[..., 2] = anchor_pos[..., 2]
        delta_ori = yaw_quat(quat_mul(robot_anchor_quat, quat_inv(anchor_quat)))
        self.body_quat_relative_w = quat_mul(delta_ori, self.body_quat_w)
        self.body_pos_relative_w = delta_pos + quat_apply(delta_ori, self.body_pos_w - anchor_pos)

    # --- debug visualization ------------------------------------------------------------------ #
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_body_visualizers"):
                self.goal_body_visualizers = [
                    VisualizationMarkers(self.cfg.body_visualizer_cfg.replace(prim_path="/Visuals/Command/goal/" + name))
                    for name in self.cfg.body_names
                ]
            for vis in self.goal_body_visualizers:
                vis.set_visibility(True)
        elif hasattr(self, "goal_body_visualizers"):
            for vis in self.goal_body_visualizers:
                vis.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return
        for i in range(len(self.cfg.body_names)):
            self.goal_body_visualizers[i].visualize(self.body_pos_relative_w[:, i], self.body_quat_relative_w[:, i])


@configclass
class MotionCommandCfg(CommandTermCfg):
    """Configuration for :class:`MotionCommand`."""

    class_type: type = MotionCommand

    asset_name: str = MISSING
    motion_file: str = MISSING  # a single path, or a list of clip paths (concatenated on time)
    anchor_body_name: str = MISSING
    body_names: list[str] = MISSING

    # Reference-state-init noise (applied only on true resets, RSI branch).
    pose_range: dict[str, tuple[float, float]] = {}
    velocity_range: dict[str, tuple[float, float]] = {}
    joint_position_range: tuple[float, float] = (-0.1, 0.1)

    # Fraction of true episode resets that start from the robot's DEFAULT STAND (deploy entry pose)
    # instead of reference-state-init onto the clip frame.
    stand_start_prob: float = 0.25
    stand_start_min_hold: int = 25

    # Pre-swing hold: freeze the reference at the swing's first frame for U[lo, hi] control steps.
    hold_steps_range: tuple[int, int] = (0, 0)

    # Teleport the robot onto the new clip frame at intra-episode wraps. MUST be False for the
    # continuous-rally lifecycle (the policy physically transitions swing -> swing).
    wrap_teleport: bool = False

    body_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/Command/pose")
    body_visualizer_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
