from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import ManagerTermBase, SceneEntityCfg

from whole_body_tracking.tasks.tracking.mdp.commands import MotionCommand
from whole_body_tracking.tasks.tracking.mdp.rewards import _get_body_indexes


def bad_anchor_pos(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command: MotionCommand = env.command_manager.get_term(command_name)
    return torch.norm(command.anchor_pos_w - command.robot_anchor_pos_w, dim=1) > threshold


def bad_anchor_pos_z_only(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command: MotionCommand = env.command_manager.get_term(command_name)
    return torch.abs(command.anchor_pos_w[:, -1] - command.robot_anchor_pos_w[:, -1]) > threshold


def bad_anchor_ori(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, command_name: str, threshold: float
) -> torch.Tensor:
    asset: RigidObject | Articulation = env.scene[asset_cfg.name]

    command: MotionCommand = env.command_manager.get_term(command_name)
    motion_projected_gravity_b = math_utils.quat_rotate_inverse(command.anchor_quat_w, asset.data.GRAVITY_VEC_W)

    robot_projected_gravity_b = math_utils.quat_rotate_inverse(command.robot_anchor_quat_w, asset.data.GRAVITY_VEC_W)

    return (motion_projected_gravity_b[:, 2] - robot_projected_gravity_b[:, 2]).abs() > threshold


def bad_motion_body_pos(
    env: ManagerBasedRLEnv, command_name: str, threshold: float, body_names: list[str] | None = None
) -> torch.Tensor:
    command: MotionCommand = env.command_manager.get_term(command_name)

    body_indexes = _get_body_indexes(command, body_names)
    error = torch.norm(command.body_pos_relative_w[:, body_indexes] - command.robot_body_pos_w[:, body_indexes], dim=-1)
    return torch.any(error > threshold, dim=-1)


def bad_motion_body_pos_z_only(
    env: ManagerBasedRLEnv, command_name: str, threshold: float, body_names: list[str] | None = None
) -> torch.Tensor:
    command: MotionCommand = env.command_manager.get_term(command_name)

    body_indexes = _get_body_indexes(command, body_names)
    error = torch.abs(command.body_pos_relative_w[:, body_indexes, -1] - command.robot_body_pos_w[:, body_indexes, -1])
    return torch.any(error > threshold, dim=-1)


def bad_motion_body_pos_z_only_outside_replay(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float,
    body_names: list[str] | None = None,
) -> torch.Tensor:
    """Keep the legacy clip-z guard except during an explicit Markov replay hold.

    A V17 replay reset deliberately pairs a real post-strike physical state with the next clip's
    held command.  Clip-relative foot z is not a physical fall criterion in that interval and can
    reject the sample before the policy acts.  Only this reference guard is masked; absolute
    base-tilt, root-height and actual-q hard-limit terminations remain active.
    """

    command: MotionCommand = env.command_manager.get_term(command_name)
    replay_active = getattr(command, "post_swing_replay_active", None)
    if replay_active is None:
        raise RuntimeError(
            "replay-aware body-z termination requires post_swing_replay_active"
        )
    legacy_bad = bad_motion_body_pos_z_only(
        env,
        command_name=command_name,
        threshold=threshold,
        body_names=body_names,
    )
    return legacy_bad & (~replay_active)


def base_x_drift_from_station(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    """Terminate when the base leaves the LOCKED x-plane: |base_x - station_x| > threshold (world x).

    x-LOCKED HITTER regime (2026-07-08): with base_target_x_range fixed at [0,0] the station x is
    pinned at spawn, so the robot must hold a fixed distance from the table and reach the ball by
    ARM EXTENSION, never by lunging the base forward. A soft base_position reward alone did NOT stop
    the forward creep (warm-started policies inherit the x-mobility habit and the soft gradient can't
    unlearn it; drift_fwd stayed ~0.03 m/swing over 1k fine-tune iters and accumulated to 1.5-2.4 m
    -> fall in the deploy rally). This termination gives x-discipline TEETH from step 0: leaving the
    x-plane is a FAILURE, so a from-scratch policy must learn to strike WITHOUT the lunge. y is
    unconstrained (footwork is y-only). The command term (racket_target) exposes base_pos_w
    (robot base world position, [:,0]=x) and base_target_pos_w (station world xy, [:,0]=x).
    """
    command = env.command_manager.get_term(command_name)
    x_err = torch.abs(command.base_pos_w[:, 0] - command.base_target_pos_w[:, 0])
    return x_err > threshold


def base_mocap_stale(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Fail closed once the actor's full-pose mocap sample exceeds its configured age.

    Short dropouts are already handled inside ``RacketTargetCommand`` by bounded propagation.
    This termination prevents the policy from continuing indefinitely on the last position and
    quaternion after that bounded window expires.
    """

    command = env.command_manager.get_term(command_name)
    if not bool(getattr(command.cfg, "base_mocap_enabled", False)):
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    age = getattr(command, "_actor_base_age_s", None)
    max_age = float(getattr(command, "_base_mocap_max_age_s", 0.0))
    if not torch.is_tensor(age) or max_age <= 0.0:
        raise RuntimeError(
            "base_mocap_stale requires a positive full-pose mocap age contract"
        )
    return age >= max_age


def _arm_deadline_step(
    hold_counter: torch.Tensor,
    ready_sample: torch.Tensor,
    previous_dwell: torch.Tensor,
    dwell_required: int,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Pure one-step kernel for an exogenous arm deadline.

    ``hold_counter == 1`` is the final frozen action because terminations are evaluated before
    MotionCommand decrements its counter.  A miss therefore resets the environment before the
    reference clock can advance; a pass lets the normal command update release it.  No timer is
    extended and the policy cannot alter the deadline.
    """
    dwell = torch.where(ready_sample, previous_dwell + 1, torch.zeros_like(previous_dwell))
    deadline = hold_counter == 1
    release = deadline & ready_sample & (dwell >= int(dwell_required))
    miss = deadline & ~release
    return dwell, deadline, release, miss


class ArmDeadlineMiss(ManagerTermBase):
    """Terminate a rally when the robot is not settled at the fixed arm deadline.

    The runner only has mocap base position, IMU and joint proprioception.  To keep this gate
    deploy-honest, planar base speed is estimated from position differences and filtered with the
    same EMA form as the C++ runner instead of reading simulator root linear velocity.  All other
    readiness signals are likewise available on the robot: station error, IMU heading/yaw rate and
    tilt, and joint velocity.  The term is registered only by RallyFinalV2Plus.
    """

    _STATION_CLASSES = ((-1, "reset"), (0, "same"), (1, "small"), (2, "main"))

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        params = cfg.params
        self._motion = env.command_manager.get_term(params["motion_command_name"])
        self._racket = env.command_manager.get_term(params["racket_command_name"])
        self._alpha = float(params.get("speed_ema_alpha", 0.25))
        if not (0.0 < self._alpha <= 1.0):
            raise ValueError(f"speed_ema_alpha must be in (0, 1], got {self._alpha}")
        if not bool(getattr(self._racket.cfg, "arm_deadline_gate", False)):
            raise ValueError("ArmDeadlineMiss requires racket.arm_deadline_gate=true")
        if bool(getattr(self._racket.cfg, "hold_until_settled", False)):
            raise ValueError("ArmDeadlineMiss is incompatible with hold_until_settled")

        self._previous_base_xy = self._racket.base_pos_w[:, :2].clone()
        self._base_speed_ema = torch.zeros(self.num_envs, device=self.device)
        self._speed_valid = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._dwell_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._dwell_required = max(
            1,
            int(math.ceil(float(self._racket.cfg.ready_monitor_dwell_s) / float(env.step_dt))) + 1,
        )

        self._deadline_count = torch.zeros((), device=self.device)
        self._deadline_pass_count = torch.zeros((), device=self.device)
        self._class_count = torch.zeros(len(self._STATION_CLASSES), device=self.device)
        self._class_miss_count = torch.zeros_like(self._class_count)
        self._clip_count = torch.zeros(2, device=self.device)
        self._clip_miss_count = torch.zeros_like(self._clip_count)

    def reset(self, env_ids=None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._previous_base_xy[env_ids] = self._racket.base_pos_w[env_ids, :2]
        self._base_speed_ema[env_ids] = 0.0
        self._speed_valid[env_ids] = False
        self._dwell_steps[env_ids] = 0

    def reset_statistics(self) -> None:
        """Clear evaluator-facing deadline counters without changing per-env readiness state."""
        self._deadline_count.zero_()
        self._deadline_pass_count.zero_()
        self._class_count.zero_()
        self._class_miss_count.zero_()
        self._clip_count.zero_()
        self._clip_miss_count.zero_()

    def statistics(self) -> dict[str, torch.Tensor]:
        """Return cloned event counters for deterministic evaluation."""
        return {
            "deadline_count": self._deadline_count.clone(),
            "deadline_pass_count": self._deadline_pass_count.clone(),
            "station_class_count": self._class_count.clone(),
            "station_class_miss_count": self._class_miss_count.clone(),
            "clip_count": self._clip_count.clone(),
            "clip_miss_count": self._clip_miss_count.clone(),
        }

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        motion_command_name: str,
        racket_command_name: str,
        speed_ema_alpha: float = 0.25,
    ) -> torch.Tensor:
        del env, motion_command_name, racket_command_name, speed_ema_alpha
        cfg = self._racket.cfg
        if not bool(cfg.arm_deadline_gate) or bool(cfg.hold_until_settled):
            raise RuntimeError(
                "ArmDeadlineMiss contract drift: arm_deadline_gate must be true and "
                "hold_until_settled must be false"
            )

        base_xy = self._racket.base_pos_w[:, :2]
        had_speed_sample = self._speed_valid.clone()
        speed_instant = torch.linalg.norm(base_xy - self._previous_base_xy, dim=-1) / float(
            self._env.step_dt
        )
        self._base_speed_ema = torch.where(
            had_speed_sample,
            self._alpha * speed_instant + (1.0 - self._alpha) * self._base_speed_ema,
            torch.zeros_like(speed_instant),
        )
        self._previous_base_xy.copy_(base_xy)
        self._speed_valid[:] = True

        base_error = base_xy - self._racket.base_target_pos_w
        x_error = torch.abs(base_error[:, 0])
        y_error = torch.abs(base_error[:, 1])
        position_ok = (
            (x_error <= float(cfg.ready_monitor_x_thresh))
            & (y_error <= float(cfg.ready_monitor_y_thresh))
        )
        speed_ok = had_speed_sample & (
            self._base_speed_ema <= float(cfg.ready_monitor_speed_thresh)
        )

        quat = self._racket.base_quat_w
        forward_x = 1.0 - 2.0 * (quat[:, 2].square() + quat[:, 3].square())
        forward_y = 2.0 * (quat[:, 1] * quat[:, 2] + quat[:, 0] * quat[:, 3])
        heading_abs = torch.abs(torch.atan2(forward_y, forward_x))
        heading_ok = heading_abs <= float(cfg.ready_monitor_heading_thresh_rad)

        data = self._racket.robot.data
        yaw_rate_abs = torch.abs(data.root_ang_vel_b[:, 2])
        yaw_rate_ok = yaw_rate_abs <= float(cfg.ready_monitor_yaw_rate_thresh)
        projected_gravity = getattr(data, "projected_gravity_b", None)
        if projected_gravity is None:
            raise RuntimeError("ArmDeadlineMiss requires deploy-honest projected_gravity_b")
        tilt = torch.linalg.norm(projected_gravity[:, :2], dim=-1)
        tilt_ok = tilt <= float(cfg.ready_monitor_tilt_thresh)
        joint_speed = torch.sqrt(torch.mean(data.joint_vel.square(), dim=-1))
        joint_speed_ok = joint_speed <= float(cfg.ready_monitor_joint_speed_thresh)

        holding = self._motion.hold_counter > 0
        ready_sample = (
            holding
            & position_ok
            & speed_ok
            & heading_ok
            & yaw_rate_ok
            & tilt_ok
            & joint_speed_ok
        )
        self._dwell_steps, deadline, release, miss = _arm_deadline_step(
            self._motion.hold_counter,
            ready_sample,
            self._dwell_steps,
            self._dwell_required,
        )

        deadline_f = deadline.float()
        miss_f = miss.float()
        self._deadline_count += deadline_f.sum()
        self._deadline_pass_count += release.float().sum()
        for index, (class_id, _name) in enumerate(self._STATION_CLASSES):
            class_mask = deadline & (self._racket.metrics["station_y_step_class"] == float(class_id))
            self._class_count[index] += class_mask.float().sum()
            self._class_miss_count[index] += (class_mask & miss).float().sum()
        if getattr(self._motion, "_multiseg", False):
            for clip_id in range(min(2, int(self._motion.motion.num_segments))):
                clip_mask = deadline & (self._motion.clip_id == clip_id)
                self._clip_count[clip_id] += clip_mask.float().sum()
                self._clip_miss_count[clip_id] += (clip_mask & miss).float().sum()

        metrics = self._racket.metrics
        metrics["arm_deadline_event"] = deadline_f
        metrics["arm_deadline_pass"] = release.float()
        metrics["arm_deadline_miss"] = miss_f
        metrics["arm_deadline_pass_rate"] = torch.ones_like(x_error) * (
            self._deadline_pass_count / self._deadline_count.clamp_min(1.0)
        )
        event_values = {
            "arm_deadline_x_error": x_error,
            "arm_deadline_y_error": y_error,
            "arm_deadline_base_speed": self._base_speed_ema,
            "arm_deadline_heading_error_deg": torch.rad2deg(heading_abs),
            "arm_deadline_yaw_rate_abs": yaw_rate_abs,
            "arm_deadline_tilt": tilt,
            "arm_deadline_joint_speed": joint_speed,
            "arm_deadline_dwell_s": self._dwell_steps.float() * float(self._env.step_dt),
            "arm_deadline_position_ok": position_ok.float(),
            "arm_deadline_speed_ok": speed_ok.float(),
            "arm_deadline_heading_ok": heading_ok.float(),
            "arm_deadline_yaw_rate_ok": yaw_rate_ok.float(),
            "arm_deadline_tilt_ok": tilt_ok.float(),
            "arm_deadline_joint_speed_ok": joint_speed_ok.float(),
            "arm_deadline_dwell_ok": (self._dwell_steps >= self._dwell_required).float(),
        }
        for name, value in event_values.items():
            metrics[name] = torch.where(deadline, value, metrics[name])
        for index, (_class_id, name) in enumerate(self._STATION_CLASSES):
            metrics[f"arm_deadline_miss_rate_{name}"] = torch.ones_like(x_error) * (
                self._class_miss_count[index] / self._class_count[index].clamp_min(1.0)
            )
        for clip_id, name in enumerate(("forehand", "backhand")):
            metrics[f"arm_deadline_miss_rate_{name}"] = torch.ones_like(x_error) * (
                self._clip_miss_count[clip_id] / self._clip_count[clip_id].clamp_min(1.0)
            )
        return miss.clone()


def ready_release_timeout(
    env: ManagerBasedRLEnv, command_name: str = "racket_target"
) -> torch.Tensor:
    """Return V17's sampled READY-latch timeout event.

    The swing clock remains held while this bit is raised. The EnvCfg marks the term as a timeout
    so a failed recovery question is resampled without being mislabeled as a physical fall.
    """

    command = env.command_manager.get_term(command_name)
    timeout = getattr(command, "ready_release_timeout", None)
    if not torch.is_tensor(timeout):
        raise RuntimeError(
            "ready_release_timeout requires a racket command exposing "
            "ready_release_timeout"
        )
    return timeout
