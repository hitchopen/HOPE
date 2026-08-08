"""Deploy-faithful action terms.

ClampedJointPositionAction (2026-07-05): the C++ deploy runner clamps q_des to the
A3 hard joint limits before publishing (pp_joint_limits.hpp) — a SAFETY feature — but
training ran with no clamp (clip_actions=null, and PhysX implicit drives accept
out-of-range targets as "saturated torque, please"). The policy legitimately
learned to command PAST the ankle limit to buy kp-saturated torque when arresting
a forward tip (118 Nm requested -> clamp cut it to ~41 Nm on 34% of bare-hold
ticks in the Gate 2.5 P2 log = ~65% of the tipping-arrest torque silently
removed at deploy). Isaac clamps the processed target to its configured *soft* limits, a
conservative subset of the deploy hard limits. This removes the unbounded-target mismatch, but it is
not numerically identical to the hard clamp; RallyFinalV2 logs both envelopes and additionally keeps
ankle-roll q_des inside an absolute +/-0.20 rad safety band.
"""

from __future__ import annotations

import math
import os
import pathlib
import re
import time

import torch

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import configclass

from .qdes_contract import (
    feasible_qdes_v3,
    feasible_qdes_v3_dynamic_inputs_finite,
)


A3_PASSIVE_HEAD_JOINT_NAMES = ("head_yaw_joint", "head_pitch_joint")


BOUNDED_QDES_ACTION_CONTRACT = "bounded_qdes_v2"
FEASIBLE_QDES_ACTION_CONTRACT = "feasible_qdes_v3"
ACTUAL_Q_GUARD_CONTRACT = "predictive_safe_boundary_brake_v1"
# v5 (2026-07-23 pre-launch audit): v4 decoded into the FULL safe span through one tanh whose
# bias/gain were fitted only at a=0.  For joints whose default sits near a safe-range rail
# (A3 shoulder_roll: normalized default +/-0.962, knee: -0.898) that single curve amplified the
# local slope up to 13.5x and made the response violently asymmetric — ordinary exploration noise
# swept the racket shoulder across ~1.5 rad, which is the audited root cause of the twisted swing
# arm.  v5 anchors the decode at the DEFAULT posture with an independent tanh room per side, so
# dq/da equals the legacy affine scale in BOTH directions at a=0 for every joint, while the
# asymptotes remain exactly the safe range.  No v4 checkpoint was ever trained to completion; the
# string is retired rather than dual-supported.
ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT = "absolute_feasible_qdes_v5"
V11_SAFE_QDES_ACTION_CONTRACT = "v11_affine_safe_qdes_v1"


def _resolve_joint_pattern_values(
    joint_names: list[str], patterns, *, label: str
) -> list[float]:
    """Resolve one exhaustive, non-overlapping regex value for every action joint.

    V15 stores the per-family limits in the task YAML.  Requiring exactly one regex match keeps
    that YAML authoritative without introducing a second, code-side table or order-dependent
    "last regex wins" behavior.
    """
    items = [(str(pattern), float(value)) for pattern, value in dict(patterns).items()]
    if not items:
        raise RuntimeError(f"{label} must define at least one joint regex")
    resolved: list[float] = []
    for name in joint_names:
        matches = [(pattern, value) for pattern, value in items if re.fullmatch(pattern, name)]
        if len(matches) != 1:
            raise RuntimeError(
                f"{label} must match joint {name!r} exactly once; matches={matches}"
            )
        resolved.append(matches[0][1])
    return resolved


def resolve_action_columns_by_joint_name(action_term, joint_names) -> list[int]:
    """Resolve exact joint names to action columns, raising on any contract mismatch.

    Articulation order, action order, and the SDK wire order are not interchangeable on A3.  Any
    passive-joint operation must therefore use the names resolved by the action term itself instead
    of assuming that the head happens to occupy columns 3 and 4.
    """
    action_joint_names = getattr(action_term, "_joint_names", None)
    if action_joint_names is None:
        raise RuntimeError(
            "Passive-joint action contract requires the action term's resolved joint names"
        )
    action_joint_names = list(action_joint_names)
    if len(action_joint_names) != len(set(action_joint_names)):
        raise RuntimeError(
            f"Passive-joint action contract found duplicate resolved names: {action_joint_names}"
        )

    requested = list(joint_names)
    if not requested:
        return []
    if len(requested) != len(set(requested)):
        raise RuntimeError(f"Passive-joint names must be unique, got {requested}")
    missing = [name for name in requested if name not in action_joint_names]
    if missing:
        raise RuntimeError(
            "Passive joints missing from the configured action term: "
            f"{missing}; resolved action joints are {action_joint_names}"
        )
    return [action_joint_names.index(name) for name in requested]


def _action_joint_ids_in_column_order(action_term) -> list[int]:
    """Return articulation joint ids in action-column order, without order assumptions."""
    joint_ids = getattr(action_term, "_joint_ids", None)
    if joint_ids is None:
        raise RuntimeError("Passive-joint action contract cannot resolve articulation joint ids")
    if isinstance(joint_ids, slice):
        joint_ids = list(range(len(action_term._asset.joint_names)))[joint_ids]
    elif torch.is_tensor(joint_ids):
        joint_ids = joint_ids.detach().cpu().tolist()
    else:
        joint_ids = list(joint_ids)
    if len(joint_ids) != len(action_term._joint_names):
        raise RuntimeError(
            "Passive-joint action contract found inconsistent action names/ids: "
            f"{len(action_term._joint_names)} names vs {len(joint_ids)} ids"
        )
    return [int(joint_id) for joint_id in joint_ids]


class ClampedJointPositionAction(JointPositionAction):
    """JointPositionAction with deploy-faithful safety clamp and optional passive joints.

    ``cfg.passive_joint_names`` is opt-in so generic G1 tasks keep their existing contract.  For an
    A3 passive-head task, set it to ``A3_PASSIVE_HEAD_JOINT_NAMES``.  The policy still emits 31 raw
    action values, but the selected joints are applied at their articulation defaults and their
    actor-visible applied-action feedback is exactly zero, matching the deploy runner.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._num_envs = int(self._raw_actions.shape[0])
        # Keep the deploy-pre-clamp target available to reward/diagnostic terms.  Looking only at
        # ``_processed_actions`` hides exactly the failure we care about: after the safety clamp a
        # policy that requested +0.8 rad and one that requested the legal +0.35 rad look identical.
        # This buffer is reward-only and never enters the actor observation.
        self._unclamped_processed_actions = torch.zeros_like(self._processed_actions)
        # ``raw_actions`` must remain the unmodified network output so a reward can train unused
        # passive channels toward zero.  This second buffer is the action that was actually applied,
        # expressed in the original action domain, and is the one fed back to the actor.
        self._applied_raw_actions = torch.zeros_like(self._raw_actions)

        passive_names = tuple(self.cfg.passive_joint_names)
        passive_cols = resolve_action_columns_by_joint_name(self, passive_names)
        action_joint_ids = _action_joint_ids_in_column_order(self)
        self._passive_joint_names = passive_names
        self._passive_action_cols = torch.tensor(
            passive_cols, dtype=torch.long, device=self.device
        )
        self._passive_joint_ids = torch.tensor(
            [action_joint_ids[col] for col in passive_cols], dtype=torch.long, device=self.device
        )
        print("[hope_actions] q_des CLAMP ACTIVE: processed targets use conservative Isaac soft "
              "limits; deploy uses hard pp_joint_limits", flush=True)
        if passive_names:
            print(
                "[hope_actions] PASSIVE JOINTS ACTIVE: fixed at articulation defaults and zeroed "
                f"in applied-last-action feedback: {list(passive_names)}",
                flush=True,
            )

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        self._applied_raw_actions.copy_(self._raw_actions)
        self._unclamped_processed_actions.copy_(self._processed_actions)
        limits = self._asset.data.soft_joint_pos_limits[:, self._joint_ids, :]
        self._processed_actions = torch.clamp(
            self._processed_actions, min=limits[..., 0], max=limits[..., 1]
        )
        if self._passive_action_cols.numel() > 0:
            default_q = self._asset.data.default_joint_pos.index_select(
                -1, self._passive_joint_ids
            )
            self._processed_actions.index_copy_(-1, self._passive_action_cols, default_q)
            self._applied_raw_actions.index_fill_(-1, self._passive_action_cols, 0.0)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if env_ids is None:
            self._applied_raw_actions.zero_()
            self._unclamped_processed_actions.zero_()
        else:
            self._applied_raw_actions[env_ids] = 0.0
            self._unclamped_processed_actions[env_ids] = 0.0

    @property
    def unclamped_processed_actions(self) -> torch.Tensor:
        """Absolute joint-position targets before the deploy-faithful safety clamp."""
        return self._unclamped_processed_actions

    @property
    def applied_raw_actions(self) -> torch.Tensor:
        """Raw-domain action actually applied, with passive-joint columns set to zero."""
        return self._applied_raw_actions

    @property
    def passive_joint_names(self) -> tuple[str, ...]:
        return self._passive_joint_names


def passive_head_raw_action_penalty(
    env,
    action_name: str = "joint_pos",
    std: float = 1.0,
    loss_type: str = "huber",
    huber_delta: float = 1.0,
) -> torch.Tensor:
    """Penalize raw A3 head outputs even though the passive head does not execute them.

    The returned value is non-negative; configure it with a negative reward weight.  ``huber``
    retains a linear tail for an already-wound-up policy, while ``l2`` is useful when training from
    scratch.  This deliberately reads ``raw_actions`` rather than the zeroed applied-action buffer.
    """
    if std <= 0.0:
        raise ValueError(f"passive-head action std must be positive, got {std}")
    if huber_delta <= 0.0:
        raise ValueError(f"passive-head Huber delta must be positive, got {huber_delta}")
    action_term = env.action_manager.get_term(action_name)
    cols = resolve_action_columns_by_joint_name(action_term, A3_PASSIVE_HEAD_JOINT_NAMES)
    col_tensor = torch.tensor(cols, dtype=torch.long, device=action_term.raw_actions.device)
    scaled = action_term.raw_actions.index_select(-1, col_tensor) / float(std)
    if loss_type == "l2":
        loss = torch.square(scaled)
    elif loss_type == "huber":
        magnitude = torch.abs(scaled)
        delta = float(huber_delta)
        loss = torch.where(
            magnitude <= delta,
            0.5 * torch.square(magnitude),
            delta * (magnitude - 0.5 * delta),
        )
    else:
        raise ValueError(
            f"passive-head action loss_type must be 'huber' or 'l2', got {loss_type!r}"
        )
    return torch.mean(loss, dim=-1)


@configclass
class ClampedJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = ClampedJointPositionAction

    passive_joint_names: tuple[str, ...] = ()
    """Exact joint names fixed at default q and zeroed in applied last-action feedback.

    Empty by default to avoid changing non-A3 tasks.  A passive-head A3 task must explicitly set
    this to ``("head_yaw_joint", "head_pitch_joint")``; a missing/misspelled joint fails at startup.
    """


class V11SafeClampedJointPositionAction(ClampedJointPositionAction):
    """V11 affine action semantics with explicit final-q_des and actual-q audits.

    ``ClampedJointPositionAction`` already implements the plant behavior used to train
    ``model_11800``: ``default_q + scale * action`` followed by the articulation's conservative
    soft-limit clamp, with passive head joints fixed at their defaults.  This subclass deliberately
    does not add a tanh decoder, rate limiter, tracking window, or torque projector.  By default it
    is byte-for-byte stateless with respect to the plant target.  ``HitterPingPong`` opts into the
    reviewed 0--2 control-tick post-clamp FIFO; a zero maximum lag retains the historical fast path.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._action_contract = str(cfg.action_contract)
        if self._action_contract != V11_SAFE_QDES_ACTION_CONTRACT:
            raise RuntimeError(
                "V11SafeClampedJointPositionAction requires action_contract="
                f"{V11_SAFE_QDES_ACTION_CONTRACT!r}, got {cfg.action_contract!r}"
            )
        self._actual_q_hard_tolerance_rad = float(
            cfg.actual_q_hard_tolerance_rad
        )
        if (
            not math.isfinite(self._actual_q_hard_tolerance_rad)
            or self._actual_q_hard_tolerance_rad < 0.0
        ):
            raise RuntimeError(
                "actual_q_hard_tolerance_rad must be finite and non-negative"
            )
        self._actual_q_hard_audit_mode = str(
            cfg.actual_q_hard_audit_mode
        ).strip().lower()
        if self._actual_q_hard_audit_mode not in {"termination", "telemetry"}:
            raise RuntimeError(
                "actual_q_hard_audit_mode must be 'termination' or 'telemetry', "
                f"got {cfg.actual_q_hard_audit_mode!r}"
            )

        self._qdes_delay_min_steps = int(cfg.qdes_delay_min_steps)
        self._qdes_delay_max_steps = int(cfg.qdes_delay_max_steps)
        self._qdes_delay_nominal_fraction = float(
            cfg.qdes_delay_nominal_fraction
        )
        if not (
            0 <= self._qdes_delay_min_steps <= self._qdes_delay_max_steps
        ):
            raise RuntimeError(
                "qdes delay must satisfy 0 <= min_steps <= max_steps"
            )
        if not (
            math.isfinite(self._qdes_delay_nominal_fraction)
            and 0.0 <= self._qdes_delay_nominal_fraction <= 1.0
        ):
            raise RuntimeError(
                "qdes_delay_nominal_fraction must be finite and in [0, 1]"
            )

        self._action_joint_ids = torch.tensor(
            _action_joint_ids_in_column_order(self),
            dtype=torch.long,
            device=self.device,
        )
        hard = self._select_action_joints(self._asset.data.joint_pos_limits)
        safe = self._asset.data.soft_joint_pos_limits[
            :, self._joint_ids, :
        ]
        self._hard_lo = hard[..., 0]
        self._hard_hi = hard[..., 1]
        self._safe_lo = safe[..., 0]
        self._safe_hi = safe[..., 1]
        if not bool(
            torch.isfinite(self._hard_lo).all()
            and torch.isfinite(self._hard_hi).all()
            and torch.isfinite(self._safe_lo).all()
            and torch.isfinite(self._safe_hi).all()
        ):
            raise RuntimeError("V11 safe q_des limits contain NaN/Inf")
        if bool(
            (self._hard_lo >= self._hard_hi).any()
            or (self._safe_lo >= self._safe_hi).any()
            or (self._safe_lo < self._hard_lo).any()
            or (self._safe_hi > self._hard_hi).any()
        ):
            raise RuntimeError(
                "V11 safe q_des interval must be a non-empty subset of hard limits"
            )

        default_qdes = self._select_action_joints(
            self._asset.data.default_joint_pos
        ).clone()
        self._qdes_commanded = default_qdes.clone()
        self._qdes_executed = default_qdes.clone()
        self._previous_qdes_executed = self._qdes_executed.clone()
        self._qdes_delta = torch.zeros_like(self._qdes_executed)
        self._previous_qdes_delta = torch.zeros_like(self._qdes_executed)
        self._qdes_second_difference = torch.zeros_like(
            self._qdes_executed
        )
        self._qdes_projection_distance = torch.zeros(
            self._num_envs, device=self.device
        )
        self._qdes_reversal_fraction = torch.zeros(
            self._num_envs, device=self.device
        )
        self._last_actual_q_fault = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._actual_q_fault_event_counter = torch.zeros(
            self._num_envs, dtype=torch.long, device=self.device
        )
        joint_count = len(self._joint_names)
        self._qdes_audit_samples = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._qdes_clamp_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._safety_violation_count = {
            label: torch.zeros(joint_count, dtype=torch.long, device=self.device)
            for label in (
                "qdes_safe_lower",
                "qdes_safe_upper",
                "q_hard_lower",
                "q_hard_upper",
            )
        }
        self._safety_violation_max = {
            label: torch.zeros(joint_count, device=self.device)
            for label in self._safety_violation_count
        }
        self._actual_q_hard_exceedance_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        # Legacy V15/V16 code still names this tensor after the DoneTerm that consumes it.
        # HitterPingPong uses the same counter only as non-terminating telemetry.
        self._actual_q_hard_termination_count = (
            self._actual_q_hard_exceedance_count
        )
        self._actual_q_audit_sample_count = torch.zeros(
            (), dtype=torch.long, device=self.device
        )
        self._actual_q_hard_exceedance_env_count = torch.zeros(
            (), dtype=torch.long, device=self.device
        )
        self._q_nonfinite_latched = False
        self._qdes_nonfinite_latched = False
        self._qdes_safety_fault_latched = False
        self._physical_q_hard_fault_latched = False
        self._last_safety_fault_path = ""
        env_ids = torch.arange(
            self._num_envs, dtype=torch.long, device=self.device
        )
        self._qdes_delay_nominal_mask = self._select_nominal_delay_envs(
            env_ids
        )
        self._qdes_delay_steps = torch.zeros(
            self._num_envs, dtype=torch.long, device=self.device
        )
        self._qdes_delay_queue = default_qdes.unsqueeze(1).repeat(
            1, self._qdes_delay_max_steps + 1, 1
        )
        self._qdes_delay_seed_pending = torch.ones(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._resample_qdes_delay(env_ids)
        print(
            "[hope_actions] V11 AFFINE SAFE q_des v1 ACTIVE: exact legacy affine "
            "decode + existing Isaac soft-limit clamp; no stateful projection; "
            f"q_des delay={self._qdes_delay_min_steps}.."
            f"{self._qdes_delay_max_steps} control ticks; actual-q audit="
            f"{self._actual_q_hard_audit_mode}",
            flush=True,
        )

    def _select_nominal_delay_envs(
        self, env_ids: torch.Tensor
    ) -> torch.Tensor:
        """Return a deterministic nominal cohort that cannot drift on partial reset."""

        fraction = self._qdes_delay_nominal_fraction
        if fraction <= 0.0:
            return torch.zeros_like(env_ids, dtype=torch.bool)
        if fraction >= 1.0:
            return torch.ones_like(env_ids, dtype=torch.bool)
        reciprocal = 1.0 / fraction
        stride = int(round(reciprocal))
        if not math.isclose(
            reciprocal, float(stride), rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise RuntimeError(
                "qdes_delay_nominal_fraction must be the reciprocal of an "
                "integer so the nominal cohort is stable"
            )
        return torch.remainder(env_ids, stride) == 0

    def _resample_qdes_delay(self, env_ids: torch.Tensor) -> None:
        """Sample one fixed FIFO lag for each randomized episode."""

        if env_ids.numel() == 0:
            return
        sampled = torch.randint(
            self._qdes_delay_min_steps,
            self._qdes_delay_max_steps + 1,
            (env_ids.numel(),),
            device=self.device,
        )
        nominal = self._qdes_delay_nominal_mask[env_ids]
        sampled[nominal] = 0
        self._qdes_delay_steps[env_ids] = sampled

    def _select_action_joints(self, value: torch.Tensor) -> torch.Tensor:
        """Select action joints from either [J...] or [N,J...] articulation tensors."""
        if value.ndim == 1:
            return value.index_select(0, self._action_joint_ids).unsqueeze(0).expand(
                self._num_envs, -1
            )
        return value.index_select(1, self._action_joint_ids)

    def _accumulate_violation(
        self, label: str, magnitude: torch.Tensor, *, tolerance: float
    ) -> torch.Tensor:
        mask = magnitude > float(tolerance)
        self._safety_violation_count[label].add_(mask.sum(dim=0))
        self._safety_violation_max[label].copy_(
            torch.maximum(
                self._safety_violation_max[label],
                magnitude.max(dim=0).values,
            )
        )
        return mask

    def _write_physical_fault_snapshot(
        self,
        mask: torch.Tensor,
        *,
        q: torch.Tensor,
        qd: torch.Tensor,
    ) -> None:
        if self._physical_q_hard_fault_latched:
            return
        self._physical_q_hard_fault_latched = True
        path = pathlib.Path(
            f"/tmp/v16_actual_q_fault_{os.getpid()}_{time.time_ns()}.pt"
        )
        payload = {
            "contract": self._action_contract,
            "kind": "actual_q_hard_limit",
            "joint_names": list(self._joint_names),
            "violating_indices": torch.nonzero(
                mask, as_tuple=False
            ).detach().cpu(),
            "state": {
                "raw_action": self._raw_actions.detach().cpu(),
                "q": q.detach().cpu(),
                "qd": qd.detach().cpu(),
                "qdes": self._qdes_executed.detach().cpu(),
                "hard_lo": self._hard_lo.detach().cpu(),
                "hard_hi": self._hard_hi.detach().cpu(),
                "hard_tolerance_rad": torch.tensor(
                    self._actual_q_hard_tolerance_rad
                ),
            },
        }
        try:
            torch.save(payload, path)
            self._last_safety_fault_path = str(path)
        except Exception as exc:  # pragma: no cover - already a physical fault
            self._last_safety_fault_path = f"<snapshot failed: {exc!r}>"

    def process_actions(self, actions: torch.Tensor):
        # Decode and clamp exactly as V11, then apply only the configured control-tick FIFO.
        # Actual-q is deliberately NOT sampled here: process_actions runs before physics, and a
        # previous-step terminal plant state has already been reset by this point.  Telemetry mode
        # is sampled by its non-terminating TerminationManager term after physics and before reset.
        previous_qdes = self._qdes_executed.clone()
        previous_delta = self._qdes_delta.clone()
        super().process_actions(actions)
        self._qdes_commanded.copy_(self._processed_actions)
        if self._qdes_delay_max_steps == 0:
            # Preserve the exact V16/V17-r12 behavior when the opt-in Build FIFO is off.
            self._qdes_executed.copy_(self._processed_actions)
        else:
            self._qdes_delay_queue[:, :-1].copy_(
                self._qdes_delay_queue[:, 1:].clone()
            )
            self._qdes_delay_queue[:, -1].copy_(self._qdes_commanded)
            pending = self._qdes_delay_seed_pending
            if bool(pending.any()):
                self._qdes_delay_queue[pending] = self._qdes_commanded[
                    pending
                ].unsqueeze(1)
            read_index = self._qdes_delay_max_steps - self._qdes_delay_steps
            rows = torch.arange(self._num_envs, device=self.device)
            self._qdes_executed.copy_(
                self._qdes_delay_queue[rows, read_index]
            )
        self._processed_actions.copy_(self._qdes_executed)
        self._qdes_delay_seed_pending.zero_()
        self._previous_qdes_executed.copy_(previous_qdes)
        self._previous_qdes_delta.copy_(previous_delta)
        self._qdes_delta.copy_(self._qdes_executed - previous_qdes)
        self._qdes_second_difference.copy_(
            self._qdes_delta - previous_delta
        )
        active = torch.ones(
            self.action_dim, dtype=torch.bool, device=self.device
        )
        if self._passive_action_cols.numel() > 0:
            active[self._passive_action_cols] = False
        source_offset = self._decoder_parameter_rows(
            self._offset,
            torch.arange(self._num_envs, device=self.device),
            "offset",
        )
        source_scale = self._decoder_parameter_rows(
            self._scale,
            torch.arange(self._num_envs, device=self.device),
            "scale",
        )
        executable_raw = (
            self._qdes_commanded - source_offset
        ) / source_scale
        self._qdes_projection_distance.copy_(
            torch.sqrt(
                torch.mean(
                    torch.square(
                        executable_raw[:, active] - actions[:, active]
                    ),
                    dim=-1,
                )
            )
        )
        reversal = (
            (self._qdes_delta[:, active] * previous_delta[:, active] < 0.0)
            & (torch.abs(self._qdes_delta[:, active]) > 1.0e-5)
            & (torch.abs(previous_delta[:, active]) > 1.0e-5)
        )
        self._qdes_reversal_fraction.copy_(reversal.float().mean(dim=-1))
        if not bool(torch.isfinite(self._qdes_executed).all()):
            self._qdes_nonfinite_latched = True
            raise RuntimeError(
                "V16 V11-safe q_des produced NaN/Inf after the final clamp"
            )

        lower_mag = torch.clamp(
            self._safe_lo - self._qdes_executed, min=0.0
        )
        upper_mag = torch.clamp(
            self._qdes_executed - self._safe_hi, min=0.0
        )
        lower = self._accumulate_violation(
            "qdes_safe_lower", lower_mag, tolerance=0.0
        )
        upper = self._accumulate_violation(
            "qdes_safe_upper", upper_mag, tolerance=0.0
        )
        self._qdes_audit_samples.add_(self._num_envs)
        self._qdes_clamp_count.add_(
            (
                torch.abs(
                    self._unclamped_processed_actions
                    - self._qdes_commanded
                )
                > 1.0e-7
            ).sum(dim=0)
        )
        if bool(lower.any()) or bool(upper.any()):
            self._qdes_safety_fault_latched = True
            raise RuntimeError(
                "V16 V11-safe final q_des escaped the Isaac soft-limit interval"
            )

    def audit_actual_q_post_physics(self) -> torch.Tensor:
        """Record actual-q hard-limit evidence and return the per-environment fault mask.

        The caller decides policy semantics.  Legacy tasks use the mask as a DoneTerm;
        HitterPingPong records it once per control cycle and never resets an environment for it.
        """
        q = self._select_action_joints(self._asset.data.joint_pos)
        qd = self._select_action_joints(self._asset.data.joint_vel)
        if not bool(torch.isfinite(q).all() and torch.isfinite(qd).all()):
            self._q_nonfinite_latched = True
            raise RuntimeError("V16 actual-q audit observed NaN/Inf plant state")

        lower_mag = torch.clamp(self._hard_lo - q, min=0.0)
        upper_mag = torch.clamp(q - self._hard_hi, min=0.0)
        self._accumulate_violation(
            "q_hard_lower", lower_mag, tolerance=0.0
        )
        self._accumulate_violation(
            "q_hard_upper", upper_mag, tolerance=0.0
        )
        physical = (
            lower_mag > self._actual_q_hard_tolerance_rad
        ) | (
            upper_mag > self._actual_q_hard_tolerance_rad
        )
        env_fault = physical.any(dim=-1)
        self._last_actual_q_fault.copy_(env_fault)
        self._actual_q_audit_sample_count.add_(self._num_envs)
        self._actual_q_fault_event_counter.add_(
            self._last_actual_q_fault.long()
        )
        self._actual_q_hard_exceedance_env_count.add_(env_fault.sum())
        self._actual_q_hard_exceedance_count.add_(physical.sum(dim=0))
        if bool(physical.any()):
            self._write_physical_fault_snapshot(
                physical, q=q, qd=qd
            )
        return env_fault

    def reset(self, env_ids=None):
        super().reset(env_ids)
        default_q = self._select_action_joints(
            self._asset.data.default_joint_pos
        )
        if env_ids is None:
            env_ids_t = torch.arange(
                self._num_envs, dtype=torch.long, device=self.device
            )
            self._resample_qdes_delay(env_ids_t)
            self._qdes_commanded.copy_(default_q)
            self._qdes_executed.copy_(default_q)
            self._previous_qdes_executed.copy_(default_q)
            self._qdes_delta.zero_()
            self._previous_qdes_delta.zero_()
            self._qdes_second_difference.zero_()
            self._qdes_projection_distance.zero_()
            self._qdes_reversal_fraction.zero_()
            self._last_actual_q_fault.zero_()
            self._qdes_delay_queue.copy_(
                default_q.unsqueeze(1).expand_as(self._qdes_delay_queue)
            )
            self._qdes_delay_seed_pending.fill_(True)
        else:
            env_ids_t = torch.as_tensor(
                env_ids, dtype=torch.long, device=self.device
            )
            self._resample_qdes_delay(env_ids_t)
            self._qdes_commanded[env_ids] = default_q[env_ids]
            self._qdes_executed[env_ids] = default_q[env_ids]
            self._previous_qdes_executed[env_ids] = default_q[env_ids]
            self._qdes_delta[env_ids] = 0.0
            self._previous_qdes_delta[env_ids] = 0.0
            self._qdes_second_difference[env_ids] = 0.0
            self._qdes_projection_distance[env_ids] = 0.0
            self._qdes_reversal_fraction[env_ids] = 0.0
            self._last_actual_q_fault[env_ids] = False
            self._qdes_delay_queue[env_ids] = default_q[env_ids].unsqueeze(1)
            self._qdes_delay_seed_pending[env_ids] = True

    def capture_markov_replay_state(self, env_ids) -> dict[str, torch.Tensor]:
        """Capture the action-side state paired with a physical replay snapshot.

        RallyV11's actor observes ``applied_raw_actions`` and its action-rate reward depends on
        both of ActionManager's most recent raw actions.  A physical ``q/qd`` snapshot paired with
        reset-zero action history is therefore not a state sampled from the live MDP.  MotionCommand
        owns the manager-level history; this method exposes the term-local tensors without making
        the command term depend on implementation-private attribute names.
        """

        state = {
            "raw": self._raw_actions[env_ids].clone(),
            "applied_raw": self._applied_raw_actions[env_ids].clone(),
            "unclamped_qdes": self._unclamped_processed_actions[env_ids].clone(),
            "processed_qdes": self._processed_actions[env_ids].clone(),
            "commanded_qdes": self._qdes_commanded[env_ids].clone(),
            "executed_qdes": self._qdes_executed[env_ids].clone(),
            "previous_executed_qdes": self._previous_qdes_executed[
                env_ids
            ].clone(),
            "qdes_delta": self._qdes_delta[env_ids].clone(),
            "previous_qdes_delta": self._previous_qdes_delta[
                env_ids
            ].clone(),
            "qdes_second_difference": self._qdes_second_difference[
                env_ids
            ].clone(),
            # Startup calibration DR makes the affine decoder offset environment-specific.
            # These parameters let a global replay slot be re-encoded into the destination
            # environment's action coordinates without changing its absolute q_des.
            "decoder_offset": self._decoder_parameter_rows(
                self._offset, env_ids, "offset"
            ).clone(),
            "decoder_scale": self._decoder_parameter_rows(
                self._scale, env_ids, "scale"
            ).clone(),
            "delay_queue": self._qdes_delay_queue[env_ids].clone(),
            "delay_steps": self._qdes_delay_steps[env_ids].clone(),
        }
        if not all(bool(torch.isfinite(value).all()) for value in state.values()):
            raise RuntimeError("V17 replay capture observed NaN/Inf in action state")
        return state

    def _decoder_parameter_rows(
        self, parameter, env_ids, name: str
    ) -> torch.Tensor:
        """Resolve a scalar/vector/per-env affine parameter to ``[len(env_ids), A]``."""

        count = len(env_ids)
        if not torch.is_tensor(parameter):
            return torch.full(
                (count, self.action_dim),
                float(parameter),
                device=self.device,
                dtype=self._raw_actions.dtype,
            )
        value = parameter.to(device=self.device, dtype=self._raw_actions.dtype)
        if value.ndim == 0:
            return value.expand(count, self.action_dim)
        if value.ndim == 1 and tuple(value.shape) == (self.action_dim,):
            return value.unsqueeze(0).expand(count, -1)
        if value.ndim == 2 and value.shape[1] == self.action_dim:
            if value.shape[0] == 1:
                return value.expand(count, -1)
            if value.shape[0] == self._num_envs:
                return value[env_ids]
        raise RuntimeError(
            f"V17 cannot resolve action decoder {name} with shape "
            f"{tuple(value.shape)} to ({count}, {self.action_dim})"
        )

    def remap_markov_replay_raw_actions(
        self,
        env_ids,
        raw_actions: torch.Tensor,
        source_decoder_offset: torch.Tensor,
        source_decoder_scale: torch.Tensor,
    ) -> torch.Tensor:
        """Re-encode source raw actions under the destination environment's affine decoder.

        For active joints this solves

        ``a_dst * scale_dst + offset_dst = a_src * scale_src + offset_src``.

        Thus the absolute unclamped q_des is unchanged. Passive-head raw outputs are kept
        unchanged because they are never executed and retain their original regularization state.
        """

        expected_shape = (len(env_ids), self.action_dim)
        for name, value in (
            ("raw_actions", raw_actions),
            ("source_decoder_offset", source_decoder_offset),
            ("source_decoder_scale", source_decoder_scale),
        ):
            if not torch.is_tensor(value) or tuple(value.shape) != expected_shape:
                raise RuntimeError(
                    f"V17 replay {name} has shape "
                    f"{None if not torch.is_tensor(value) else tuple(value.shape)}; "
                    f"expected {expected_shape}"
                )
            if not bool(torch.isfinite(value).all()):
                raise RuntimeError(f"V17 replay {name} contains NaN/Inf")

        destination_offset = self._decoder_parameter_rows(
            self._offset, env_ids, "offset"
        )
        destination_scale = self._decoder_parameter_rows(
            self._scale, env_ids, "scale"
        )
        if bool(
            (torch.abs(source_decoder_scale) <= 1.0e-12).any()
            or (torch.abs(destination_scale) <= 1.0e-12).any()
        ):
            raise RuntimeError("V17 replay cannot remap a zero action-decoder scale")
        if not bool(
            torch.allclose(
                source_decoder_scale,
                destination_scale,
                rtol=0.0,
                atol=1.0e-12,
            )
        ):
            raise RuntimeError(
                "V17 replay requires environment-invariant action scale so affine "
                "remapping preserves the raw action-rate difference exactly"
            )
        source_unclamped = (
            raw_actions * source_decoder_scale + source_decoder_offset
        )
        remapped = (
            source_unclamped - destination_offset
        ) / destination_scale
        if self._passive_action_cols.numel() > 0:
            remapped.index_copy_(
                -1,
                self._passive_action_cols,
                raw_actions.index_select(-1, self._passive_action_cols),
            )
        if not bool(torch.isfinite(remapped).all()):
            raise RuntimeError("V17 replay raw-action remap produced NaN/Inf")
        return remapped

    def remap_markov_replay_state(
        self, env_ids, state: dict[str, torch.Tensor]
    ) -> dict[str, torch.Tensor]:
        """Build a destination-consistent action state without mutating live buffers."""

        expected_shape = (len(env_ids), self.action_dim)
        required = (
            "raw",
            "applied_raw",
            "unclamped_qdes",
            "processed_qdes",
            "commanded_qdes",
            "executed_qdes",
            "previous_executed_qdes",
            "qdes_delta",
            "previous_qdes_delta",
            "qdes_second_difference",
            "decoder_offset",
            "decoder_scale",
        )
        for name in required:
            value = state.get(name)
            if value is None or tuple(value.shape) != expected_shape:
                raise RuntimeError(
                    f"V17 replay action state {name!r} has shape "
                    f"{None if value is None else tuple(value.shape)}; "
                    f"expected {expected_shape}"
                )
            if not bool(torch.isfinite(value).all()):
                raise RuntimeError(
                    f"V17 replay action state {name!r} contains NaN/Inf"
                )

        delay_queue = state.get("delay_queue")
        expected_queue_shape = (
            len(env_ids),
            self._qdes_delay_max_steps + 1,
            self.action_dim,
        )
        if (
            delay_queue is None
            or tuple(delay_queue.shape) != expected_queue_shape
            or not bool(torch.isfinite(delay_queue).all())
        ):
            raise RuntimeError(
                "V17 replay action delay_queue has shape "
                f"{None if delay_queue is None else tuple(delay_queue.shape)}; "
                f"expected {expected_queue_shape} with finite values"
            )
        delay_steps = state.get("delay_steps")
        if (
            delay_steps is None
            or tuple(delay_steps.shape) != (len(env_ids),)
            or delay_steps.dtype not in (torch.int32, torch.int64)
            or bool((delay_steps < 0).any())
            or bool((delay_steps > self._qdes_delay_max_steps).any())
        ):
            raise RuntimeError(
                "V17 replay action delay_steps violates the configured FIFO contract"
            )
        nominal = self._qdes_delay_nominal_mask[env_ids]
        if bool((delay_steps[nominal] != 0).any()):
            raise RuntimeError(
                "V17 replay attempted a non-zero q_des lag in the nominal cohort"
            )

        source_unclamped = (
            state["raw"] * state["decoder_scale"] + state["decoder_offset"]
        )
        if not bool(
            torch.allclose(
                source_unclamped,
                state["unclamped_qdes"],
                rtol=0.0,
                atol=1.0e-6,
            )
        ):
            raise RuntimeError(
                "V17 replay source raw action is inconsistent with its affine decoder"
            )

        raw = self.remap_markov_replay_raw_actions(
            env_ids,
            state["raw"],
            state["decoder_offset"],
            state["decoder_scale"],
        )
        destination_offset = self._decoder_parameter_rows(
            self._offset, env_ids, "offset"
        )
        destination_scale = self._decoder_parameter_rows(
            self._scale, env_ids, "scale"
        )
        unclamped = raw * destination_scale + destination_offset
        processed = torch.clamp(
            unclamped,
            min=self._safe_lo[env_ids],
            max=self._safe_hi[env_ids],
        )
        applied_raw = raw.clone()
        active_mask = torch.ones(
            self.action_dim, dtype=torch.bool, device=self.device
        )
        if self._passive_action_cols.numel() > 0:
            active_mask[self._passive_action_cols] = False
            default_q = self._asset.data.default_joint_pos[env_ids].index_select(
                -1, self._passive_joint_ids
            )
            processed.index_copy_(
                -1, self._passive_action_cols, default_q
            )
            applied_raw.index_fill_(
                -1, self._passive_action_cols, 0.0
            )

        if not bool(
            torch.allclose(
                state["applied_raw"][:, active_mask],
                state["raw"][:, active_mask],
                rtol=0.0,
                atol=1.0e-7,
            )
        ):
            raise RuntimeError(
                "V17 replay captured inconsistent raw/applied active action"
            )
        if not bool(
            torch.allclose(
                processed[:, active_mask],
                state["commanded_qdes"][:, active_mask],
                rtol=0.0,
                atol=1.0e-6,
            )
        ):
            raise RuntimeError(
                "V17 replay commanded q_des cannot be preserved under the destination decoder"
            )
        if not bool(
            torch.allclose(
                state["processed_qdes"],
                state["executed_qdes"],
                rtol=0.0,
                atol=1.0e-7,
            )
        ):
            raise RuntimeError(
                "V17 replay captured inconsistent processed/executed q_des"
            )
        if bool(
            (state["executed_qdes"] < self._safe_lo[env_ids] - 1.0e-7).any()
            or (state["executed_qdes"] > self._safe_hi[env_ids] + 1.0e-7).any()
            or (delay_queue < self._safe_lo[env_ids].unsqueeze(1) - 1.0e-7).any()
            or (delay_queue > self._safe_hi[env_ids].unsqueeze(1) + 1.0e-7).any()
        ):
            raise RuntimeError(
                "V17 replay executed q_des lies outside the V11 safe interval"
            )
        if self._passive_action_cols.numel() > 0:
            applied_head = state["applied_raw"].index_select(
                -1, self._passive_action_cols
            )
            if bool((torch.abs(applied_head) > 1.0e-7).any()):
                raise RuntimeError(
                    "V17 replay violates the passive-head applied-action contract"
                )
        return {
            "raw": raw,
            "applied_raw": applied_raw,
            "unclamped_qdes": unclamped,
            "processed_qdes": state["executed_qdes"].clone(),
            "commanded_qdes": processed,
            "executed_qdes": state["executed_qdes"].clone(),
            "previous_executed_qdes": state[
                "previous_executed_qdes"
            ].clone(),
            "qdes_delta": state["qdes_delta"].clone(),
            "previous_qdes_delta": state["previous_qdes_delta"].clone(),
            "qdes_second_difference": state[
                "qdes_second_difference"
            ].clone(),
            "delay_queue": delay_queue.clone(),
            "delay_steps": delay_steps.clone(),
        }

    def restore_markov_replay_state(
        self, env_ids, state: dict[str, torch.Tensor]
    ) -> dict[str, torch.Tensor]:
        """Restore an action-side state captured by :meth:`capture_markov_replay_state`."""

        remapped = self.remap_markov_replay_state(env_ids, state)
        destinations = {
            "raw": self._raw_actions,
            "applied_raw": self._applied_raw_actions,
            "unclamped_qdes": self._unclamped_processed_actions,
            "processed_qdes": self._processed_actions,
            "commanded_qdes": self._qdes_commanded,
            "executed_qdes": self._qdes_executed,
            "previous_executed_qdes": self._previous_qdes_executed,
            "qdes_delta": self._qdes_delta,
            "previous_qdes_delta": self._previous_qdes_delta,
            "qdes_second_difference": self._qdes_second_difference,
        }
        for name, destination in destinations.items():
            destination[env_ids] = remapped[name]
        self._qdes_delay_queue[env_ids] = remapped["delay_queue"]
        self._qdes_delay_steps[env_ids] = remapped["delay_steps"]
        self._qdes_delay_seed_pending[env_ids] = False
        return remapped

    @property
    def executed_qdes_delta(self) -> torch.Tensor:
        return self._qdes_delta

    @property
    def executed_qdes_second_difference(self) -> torch.Tensor:
        return self._qdes_second_difference

    @property
    def executed_qdes_delta_normalized(self) -> torch.Tensor:
        """Delta of executable affine action, exactly ``delta(q_des) / action_scale``."""

        env_ids = torch.arange(self._num_envs, device=self.device)
        scale = self._decoder_parameter_rows(self._scale, env_ids, "scale")
        return self._qdes_delta / torch.abs(scale).clamp_min(1.0e-6)

    @property
    def executed_qdes_second_difference_normalized(self) -> torch.Tensor:
        env_ids = torch.arange(self._num_envs, device=self.device)
        scale = self._decoder_parameter_rows(self._scale, env_ids, "scale")
        return self._qdes_second_difference / torch.abs(scale).clamp_min(1.0e-6)

    def safety_instrumentation_scalars(self) -> dict[str, torch.Tensor]:
        """Cumulative proof that the exact V11 plant target remains safely clamped."""
        samples = self._qdes_audit_samples.sum().clamp_min(1)
        result = {
            "qdes_isfinite": torch.tensor(
                0.0 if self._qdes_nonfinite_latched else 1.0,
                device=self.device,
            ),
            "q_isfinite": torch.tensor(
                0.0 if self._q_nonfinite_latched else 1.0,
                device=self.device,
            ),
            "numeric_or_qdes_safety_fault_latched": torch.tensor(
                float(
                    self._q_nonfinite_latched
                    or self._qdes_nonfinite_latched
                    or self._qdes_safety_fault_latched
                ),
                device=self.device,
            ),
            "physical_q_hard_fault_latched": torch.tensor(
                float(self._physical_q_hard_fault_latched),
                device=self.device,
            ),
            "actual_q_hard_tolerance_rad": torch.tensor(
                self._actual_q_hard_tolerance_rad,
                device=self.device,
            ),
            "actual_q_audit_sample_count": self._actual_q_audit_sample_count,
            "actual_q_hard_exceedance_count": (
                self._actual_q_hard_exceedance_count.sum()
            ),
            "actual_q_hard_exceedance_env_count": (
                self._actual_q_hard_exceedance_env_count
            ),
            "actual_q_hard_exceedance_rate": (
                self._actual_q_hard_exceedance_env_count.float()
                / self._actual_q_audit_sample_count.clamp_min(1).float()
            ),
            "actual_q_hard_excess_max_rad": torch.maximum(
                self._safety_violation_max["q_hard_lower"].max(),
                self._safety_violation_max["q_hard_upper"].max(),
            ),
            "qdes_clamp_count": self._qdes_clamp_count.sum(),
            "qdes_clamp_fraction": (
                self._qdes_clamp_count.sum().float()
                / samples.float()
            ),
            "safe_interval_width_min_rad": (
                self._safe_hi - self._safe_lo
            ).min(),
            "qdes_step_rms_rad": torch.sqrt(
                torch.mean(self._qdes_delta.square())
            ),
            "qdes_second_difference_rms_rad": torch.sqrt(
                torch.mean(self._qdes_second_difference.square())
            ),
            "qdes_projection_distance_raw_rms": (
                self._qdes_projection_distance.mean()
            ),
            "qdes_reversal_fraction": self._qdes_reversal_fraction.mean(),
            "qdes_delay_mean_steps": self._qdes_delay_steps.float().mean(),
            "qdes_delay_zero_fraction": (
                self._qdes_delay_steps == 0
            ).float().mean(),
            "qdes_delay_nominal_fraction": (
                self._qdes_delay_nominal_mask.float().mean()
            ),
        }
        for label in self._safety_violation_count:
            result[f"{label}_violation_count"] = (
                self._safety_violation_count[label].sum()
            )
            result[f"{label}_violation_max_rad"] = (
                self._safety_violation_max[label].max()
            )
        if self._actual_q_hard_audit_mode == "termination":
            result["actual_q_hard_termination_count"] = (
                self._actual_q_hard_exceedance_count.sum()
            )
        return result

    def resolved_contract_metadata(self) -> dict:
        delay_contract = (
            "episode_fixed_post_clamp_fifo_v1"
            if self._qdes_delay_max_steps > 0
            else "none"
        )
        return {
            "qdes_action_contract": self._action_contract,
            "qdes_policy_feedback_contract": "legacy_applied_raw_v1",
            "qdes_reward_audit_target_contract": "legacy_affine_qdes_v1",
            "qdes_safe_interval_source": "isaac_soft_joint_pos_limits_v1",
            "qdes_actual_q_hard_tolerance_rad": (
                self._actual_q_hard_tolerance_rad
            ),
            "qdes_actual_q_hard_audit_mode": (
                self._actual_q_hard_audit_mode
            ),
            "qdes_delay_contract": delay_contract,
            "qdes_delay_min_steps": self._qdes_delay_min_steps,
            "qdes_delay_max_steps": self._qdes_delay_max_steps,
            "qdes_delay_nominal_fraction": self._qdes_delay_nominal_fraction,
            "qdes_joint_names": list(self._joint_names),
            "qdes_safe_lower_rad": self._safe_lo[0].detach().cpu().tolist(),
            "qdes_safe_upper_rad": self._safe_hi[0].detach().cpu().tolist(),
            "qdes_hard_lower_rad": self._hard_lo[0].detach().cpu().tolist(),
            "qdes_hard_upper_rad": self._hard_hi[0].detach().cpu().tolist(),
        }


@configclass
class V11SafeClampedJointPositionActionCfg(ClampedJointPositionActionCfg):
    class_type: type = V11SafeClampedJointPositionAction

    action_contract: str = V11_SAFE_QDES_ACTION_CONTRACT
    actual_q_hard_tolerance_rad: float = 0.0
    actual_q_hard_audit_mode: str = "termination"
    qdes_delay_min_steps: int = 0
    qdes_delay_max_steps: int = 0
    qdes_delay_nominal_fraction: float = 1.0


class BoundedJointPositionAction(JointPositionAction):
    """Stateful q_des contract shared with the deploy runner.

    ``bounded_qdes_v2`` and ``feasible_qdes_v3`` are retained for old checkpoints.  V3 is an
    incremental, previous-q_des-anchored decoder.  ``absolute_feasible_qdes_v5`` instead decodes
    every network output into one absolute joint target anchored at the default posture with an
    independent tanh room per side, then projects that target into the current
    safe/rate/tracking/PD-effort intersection.  A constant network output therefore means a
    constant posture request; it cannot accumulate into a joint rail over repeated ticks.

    Guarantee scope (2026-07-23 audit): the published q_des is ALWAYS inside the safe range, and
    respects the per-tick rate limit whenever the full constraint intersection is non-empty.  When
    the intersection is empty (measured q far from the previous target, e.g. after an external
    shove), the rate constraint deliberately yields first and q_des jumps toward the tracking
    window around measured q — a bounded, deterministic snap shared by all runtimes.  The
    PD-effort interval uses NOMINAL kp/kd; with ±15% PD-gain domain randomization the in-sim
    torque can reach ~1.035x the effort limit on high draws (PhysX clips it); the deploy-side
    guarantee, which runs the same nominal table, is exact.

    All numerical limits are resolved from regex maps in the selected task YAML.  Export writes
    the resolved 31-joint arrays into ONNX metadata so C++ performs the same arithmetic.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._num_envs = int(self._raw_actions.shape[0])
        self._action_contract = str(cfg.action_contract)
        if self._action_contract not in (
            BOUNDED_QDES_ACTION_CONTRACT,
            FEASIBLE_QDES_ACTION_CONTRACT,
            ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT,
        ):
            raise RuntimeError(
                "BoundedJointPositionAction requires action_contract in "
                f"{(BOUNDED_QDES_ACTION_CONTRACT, FEASIBLE_QDES_ACTION_CONTRACT, ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT)!r}, "
                f"got {cfg.action_contract!r}"
            )
        self._actual_q_guard_contract = str(cfg.actual_q_guard_contract)
        self._actual_q_guard_enabled = (
            self._actual_q_guard_contract == ACTUAL_Q_GUARD_CONTRACT
        )
        self._actual_q_guard_horizon_s = float(cfg.actual_q_guard_horizon_s)
        self._actual_q_hard_tolerance_rad = float(
            cfg.actual_q_hard_tolerance_rad
        )
        if (
            not math.isfinite(self._actual_q_hard_tolerance_rad)
            or self._actual_q_hard_tolerance_rad < 0.0
        ):
            raise RuntimeError(
                "actual_q_hard_tolerance_rad must be finite and non-negative"
            )
        if self._actual_q_guard_contract not in (
            "",
            ACTUAL_Q_GUARD_CONTRACT,
        ):
            raise RuntimeError(
                "unsupported actual_q_guard_contract="
                f"{self._actual_q_guard_contract!r}"
            )
        if (
            self._action_contract != FEASIBLE_QDES_ACTION_CONTRACT
            and self._actual_q_guard_contract
        ):
            raise RuntimeError(
                "actual_q_guard_contract is only defined for feasible_qdes_v3"
            )

        self._action_joint_ids = torch.tensor(
            _action_joint_ids_in_column_order(self), dtype=torch.long, device=self.device
        )
        names = list(self._joint_names)
        self._passive_joint_names = tuple(cfg.passive_joint_names)
        passive_cols = resolve_action_columns_by_joint_name(self, self._passive_joint_names)
        self._passive_action_cols = torch.tensor(
            passive_cols, dtype=torch.long, device=self.device
        )
        self._passive_joint_ids = self._action_joint_ids.index_select(
            0, self._passive_action_cols
        ) if passive_cols else torch.empty(0, dtype=torch.long, device=self.device)

        def per_joint(patterns, label: str) -> torch.Tensor:
            values = _resolve_joint_pattern_values(names, patterns, label=label)
            return torch.tensor(values, dtype=torch.float32, device=self.device).unsqueeze(0)

        margin = per_joint(cfg.safe_margin_fraction, "safe_margin_fraction")
        if bool(torch.any(margin < 0.0) or torch.any(margin >= 0.5)):
            raise RuntimeError("safe_margin_fraction values must be in [0, 0.5)")
        self._qdes_rate_limit = per_joint(cfg.rate_limit_rad_s, "rate_limit_rad_s")
        self._tracking_error_limit = per_joint(
            cfg.tracking_error_limit_rad, "tracking_error_limit_rad"
        )
        if bool(torch.any(self._qdes_rate_limit < 0.0)):
            raise RuntimeError("rate_limit_rad_s values must be non-negative")
        if bool(torch.any(self._tracking_error_limit < 0.0)):
            raise RuntimeError("tracking_error_limit_rad values must be non-negative")
        self._torque_headroom_fraction = float(cfg.torque_headroom_fraction)
        if not 0.0 < self._torque_headroom_fraction <= 1.0:
            raise RuntimeError("torque_headroom_fraction must be in (0, 1]")

        hard = self._select_asset_data(self._asset.data.joint_pos_limits)
        self._hard_lo = hard[..., 0]
        self._hard_hi = hard[..., 1]
        span = (hard[..., 1] - hard[..., 0]).clamp_min(1.0e-6)
        self._safe_lo = hard[..., 0] + margin * span
        self._safe_hi = hard[..., 1] - margin * span
        self._safe_mid = 0.5 * (self._safe_lo + self._safe_hi)
        self._safe_half = 0.5 * (self._safe_hi - self._safe_lo)
        self._hard_mid = 0.5 * (self._hard_lo + self._hard_hi)
        self._hard_half = 0.5 * (self._hard_hi - self._hard_lo)
        if bool(torch.any(self._safe_lo >= self._safe_hi)):
            raise RuntimeError("V15 q_des safe interval is empty")

        self._default_q = self._select_asset_data(self._asset.data.default_joint_pos)
        if bool(torch.any(self._default_q <= self._safe_lo) or torch.any(self._default_q >= self._safe_hi)):
            bad = [
                name for i, name in enumerate(names)
                if bool(torch.any(self._default_q[:, i] <= self._safe_lo[:, i]))
                or bool(torch.any(self._default_q[:, i] >= self._safe_hi[:, i]))
            ]
            raise RuntimeError(f"default q lies outside the V15 safe range for joints {bad}")
        normalized_default = (
            (self._default_q - self._safe_mid) / self._safe_half.clamp_min(1.0e-6)
        ).clamp(-0.999999, 0.999999)
        # bias/gain parameterize the LEGACY single-tanh decode (bounded_qdes_v2 only).  v5 does not
        # use them: a single curve fitted at a=0 explodes for near-rail defaults (see the contract
        # constant comment above).
        self._tanh_bias = torch.atanh(normalized_default)
        if torch.is_tensor(self._scale):
            local_action_scale = self._scale
        else:
            local_action_scale = torch.full_like(self._safe_half, float(self._scale))
        local_tanh_derivative = self._safe_half * (1.0 - torch.square(normalized_default))
        self._tanh_input_gain = local_action_scale / local_tanh_derivative.clamp_min(1.0e-6)
        # v5 absolute decode: q(a) = default + room(a) * tanh(scale * a / room(a)) with
        # room(a>=0) = safe_hi - default and room(a<0) = default - safe_lo.  Slope at a=0 is the
        # legacy affine scale from BOTH sides; each side saturates at its own physical rail.
        self._decode_scale = local_action_scale
        self._decode_room_pos = (self._safe_hi - self._default_q).clamp_min(1.0e-6)
        self._decode_room_neg = (self._default_q - self._safe_lo).clamp_min(1.0e-6)
        self._feedback_scale = self._safe_half.clamp_min(float(cfg.feedback_scale_floor))

        self._projector_kp = self._select_asset_data(
            self._asset.data.default_joint_stiffness
        )
        self._projector_kd = self._select_asset_data(
            self._asset.data.default_joint_damping
        )
        self._effort_limit = self._select_asset_data(self._asset.data.joint_effort_limits)
        if bool(torch.any(self._projector_kp <= 0.0) or torch.any(self._projector_kd < 0.0)):
            raise RuntimeError("V15 q_des projector requires positive kp and non-negative kd")
        if bool(torch.any(self._effort_limit <= 0.0)):
            raise RuntimeError("V15 q_des projector requires positive effort limits")

        self._step_dt = float(env.step_dt)
        if self._step_dt <= 0.0:
            raise RuntimeError(f"V15 q_des projector requires positive env.step_dt, got {self._step_dt}")
        if self._actual_q_guard_enabled and (
            not math.isfinite(self._actual_q_guard_horizon_s)
            or self._actual_q_guard_horizon_s < self._step_dt
        ):
            raise RuntimeError(
                "predictive actual-q guard requires a finite horizon no shorter "
                f"than env.step_dt={self._step_dt}, got "
                f"{self._actual_q_guard_horizon_s}"
            )
        startup_arrays = {
            "hard_lo": self._hard_lo,
            "hard_hi": self._hard_hi,
            "safe_lo": self._safe_lo,
            "safe_hi": self._safe_hi,
            "default_q": self._default_q,
            "rate_limit": self._qdes_rate_limit,
            "tracking_limit": self._tracking_error_limit,
            "kp": self._projector_kp,
            "kd": self._projector_kd,
            "effort_limit": self._effort_limit,
        }
        nonfinite = [
            label
            for label, value in startup_arrays.items()
            if not bool(torch.isfinite(value).all())
        ]
        if nonfinite:
            raise RuntimeError(
                "V15 q_des projector metadata contains NaN/Inf: "
                + ", ".join(nonfinite)
            )

        self._qdes_nominal = self._default_q.clone()
        self._qdes_executed = self._default_q.clone()
        self._executed_feedback = torch.zeros_like(self._raw_actions)
        self._unclamped_processed_actions = self._qdes_nominal
        self._applied_raw_actions = self._executed_feedback
        self._projector_initialized = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._projection_error_normalized = torch.zeros_like(self._raw_actions)
        self._projection_active = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._rate_active = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._tracking_active = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._torque_active = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._infeasible = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._feasible_rate_bound = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._feasible_tracking_bound = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._feasible_torque_bound = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._feasible_action_utilization = torch.zeros_like(self._raw_actions)
        self._feasible_interval_width_fraction = torch.zeros_like(self._raw_actions)
        self._qdes_rate_utilization = torch.zeros_like(self._raw_actions)
        self._decode_utilization = torch.zeros_like(self._raw_actions)
        # Runtime safety audit.  It is observation/reward independent: counters are read only by
        # the runner's Instrumentation logger.  Any non-finite input/final target or final target
        # outside the YAML-derived safe interval takes the fail-fast path below.
        joint_count = len(names)
        self._safety_tolerance = 1.0e-6
        self._selected_lo = self._safe_lo.clone()
        self._selected_hi = self._safe_hi.clone()
        self._fallback_level = torch.zeros_like(self._raw_actions, dtype=torch.long)
        self._tightest_constraint = torch.zeros_like(
            self._raw_actions, dtype=torch.long
        )
        self._actual_q_guard_active = torch.zeros_like(
            self._raw_actions, dtype=torch.bool
        )
        self._actual_q_guard_upper = torch.zeros_like(
            self._raw_actions, dtype=torch.bool
        )
        self._actual_q_guard_lower = torch.zeros_like(
            self._raw_actions, dtype=torch.bool
        )
        self._actual_q_guard_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._actual_q_guard_upper_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._actual_q_guard_lower_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._safety_fault_latched = False
        self._physical_safety_fault_latched = False
        self._q_nonfinite_latched = False
        self._qdes_nonfinite_latched = False
        self._last_safety_fault_path = ""
        self._safety_audit_samples = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._safety_violation_count = {
            label: torch.zeros(joint_count, dtype=torch.long, device=self.device)
            for label in (
                "qdes_safe_lower",
                "qdes_safe_upper",
                "q_hard_lower",
                "q_hard_upper",
                "qdes_rate",
                "qdes_tracking",
                "qdes_torque",
            )
        }
        self._safety_violation_max = {
            label: torch.zeros(joint_count, device=self.device)
            for label in self._safety_violation_count
        }
        self._interval_empty_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._fallback_level_count = torch.zeros(
            joint_count, 4, dtype=torch.long, device=self.device
        )
        self._tightest_constraint_count = torch.zeros(
            joint_count, 4, dtype=torch.long, device=self.device
        )
        self._qdes_safe_margin_min = torch.full(
            (joint_count,), float("inf"), device=self.device
        )
        self._q_hard_margin_min = torch.full(
            (joint_count,), float("inf"), device=self.device
        )
        self._safe_interval_width_min = (
            self._safe_hi[0] - self._safe_lo[0]
        ).clone()
        self._final_interval_width_min = (
            self._safe_hi[0] - self._safe_lo[0]
        ).clone()
        self._qdes_near_limit_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        self._q_hard_near_limit_count = torch.zeros(
            joint_count, dtype=torch.long, device=self.device
        )
        # Exact streaming quantiles for a fixed 0.001 utilization grid.  Utilization is distance
        # from the safe midpoint divided by safe half-width (0=center, 1=rail).
        self._safety_histogram_bins = 1001
        self._qdes_utilization_histogram = torch.zeros(
            joint_count,
            self._safety_histogram_bins,
            dtype=torch.long,
            device=self.device,
        )
        self._q_hard_utilization_histogram = torch.zeros_like(
            self._qdes_utilization_histogram
        )
        # Smoothness is defined on the command that survives the complete q_des projector, not on
        # the actor output.  A raw action can stay constant while a state-dependent tracking/PD
        # interval moves the published target; conversely, intervention-owned raw action changes
        # are not under policy control.  Keep both first and second differences in joint radians and
        # a safe-range-normalized form used by reward terms.
        self._executed_qdes_delta = torch.zeros_like(self._raw_actions)
        self._previous_executed_qdes_delta = torch.zeros_like(self._raw_actions)
        self._executed_qdes_second_difference = torch.zeros_like(self._raw_actions)
        self._executed_qdes_delta_normalized = torch.zeros_like(self._raw_actions)
        self._executed_qdes_second_difference_normalized = torch.zeros_like(
            self._raw_actions
        )

        # HUGWBC upper-body intervention group.  This is a training-time robustness device,
        # not a deploy controller: a fixed fraction of parallel environments occasionally has
        # shoulder/elbow q_des replaced while the lower body executes pre-swing STAND/STEP or
        # post-strike recovery.  Wind-up/contact remain policy-controlled so HITTER's racket
        # objective is still learnable.  Numeric choices and exact joint names come only from the
        # task YAML.
        self._upper_intervention_enabled = bool(cfg.upper_intervention_enabled)
        intervention_cols = resolve_action_columns_by_joint_name(
            self, tuple(cfg.upper_intervention_joint_names)
        )
        self._upper_intervention_cols = torch.tensor(
            intervention_cols, dtype=torch.long, device=self.device
        )
        self._qdes_debug_joint_names = tuple(cfg.qdes_debug_joint_names)
        debug_cols = resolve_action_columns_by_joint_name(
            self, self._qdes_debug_joint_names
        )
        self._qdes_debug_cols = tuple(debug_cols)
        group_count = int(round(self._num_envs * float(cfg.upper_intervention_group_fraction)))
        group_count = min(max(group_count, 0), self._num_envs)
        self._upper_intervention_group = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._upper_intervention_group[:group_count] = self._upper_intervention_enabled
        self._upper_intervention_mask = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._upper_intervention_effective = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._upper_intervention_prev_supervision = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self.device
        )
        self._upper_intervention_age = torch.zeros(
            self._num_envs, dtype=torch.long, device=self.device
        )
        intervention_shape = (self._num_envs, int(self._upper_intervention_cols.numel()))
        self._upper_intervention_start_action = torch.zeros(
            intervention_shape, dtype=self._raw_actions.dtype, device=self.device
        )
        self._upper_intervention_target_action = torch.zeros_like(
            self._upper_intervention_start_action
        )
        self._upper_intervention_current_action = torch.zeros_like(
            self._upper_intervention_start_action
        )
        self._upper_intervention_target_q = self._default_q.index_select(
            -1, self._upper_intervention_cols
        ).clone()
        self._upper_intervention_switch_prob = float(cfg.upper_intervention_switch_prob)
        self._upper_intervention_resample_steps = max(
            int(cfg.upper_intervention_resample_steps), 1
        )
        self._upper_intervention_radius_rad = float(cfg.upper_intervention_radius_rad)
        intervention_action_range = tuple(float(v) for v in cfg.upper_intervention_action_range)
        if (
            self._upper_intervention_enabled
            and self._action_contract != FEASIBLE_QDES_ACTION_CONTRACT
            and (
                len(intervention_action_range) != 2
                or not intervention_action_range[0] < intervention_action_range[1]
            )
        ):
            raise RuntimeError(
                "upper_intervention_action_range must be [lo, hi] with lo < hi"
            )
        if (
            self._upper_intervention_enabled
            and self._action_contract == FEASIBLE_QDES_ACTION_CONTRACT
            and self._upper_intervention_radius_rad <= 0.0
        ):
            raise RuntimeError(
                "feasible_qdes_v3 upper intervention requires upper_intervention_radius_rad > 0"
            )
        self._upper_intervention_action_lo = (
            intervention_action_range[0] if len(intervention_action_range) == 2 else 0.0
        )
        self._upper_intervention_action_hi = (
            intervention_action_range[1] if len(intervention_action_range) == 2 else 0.0
        )
        self._racket_command_name = str(cfg.upper_intervention_command_name)
        self._racket_command_term = None

        if self._action_contract == FEASIBLE_QDES_ACTION_CONTRACT:
            contract_message = (
                "V15 FEASIBLE q_des v3 ACTIVE: previous-target anchored tanh inside "
                "safe/rate/tracking/PD intersection"
                + (
                    " + predictive actual-q brake"
                    f" (horizon={self._actual_q_guard_horizon_s:.3f}s)"
                    if self._actual_q_guard_enabled
                    else " (legacy unguarded)"
                )
            )
        elif self._action_contract == ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT:
            contract_message = (
                "V15 ABSOLUTE FEASIBLE q_des v5 ACTIVE: default-anchored per-side safe tanh + "
                "rate/tracking/PD projection"
            )
        else:
            contract_message = (
                "BOUNDED q_des v2 ACTIVE: local-scale tanh safe range + "
                "rate/tracking/PD projector"
            )
        print(
            f"[hope_actions] {contract_message}, dt={self._step_dt:.4f}s, "
            f"torque_headroom={self._torque_headroom_fraction:.3f}",
            flush=True,
        )
        if self._upper_intervention_enabled:
            print(
                "[hope_actions] HUGWBC UPPER-BODY INTERVENTION ACTIVE during STAND/STEP "
                f"and post-strike recovery: group={group_count}/{self._num_envs}, joints="
                f"{list(cfg.upper_intervention_joint_names)}",
                flush=True,
            )

    def _select_asset_data(self, value: torch.Tensor) -> torch.Tensor:
        """Select action joints from either [J...] or [N,J...] articulation tensors."""
        if value.ndim == 1:
            return value.index_select(0, self._action_joint_ids).unsqueeze(0).expand(
                self._num_envs, -1
            )
        return value.index_select(1, self._action_joint_ids)

    def _racket_command(self):
        if self._racket_command_term is None:
            self._racket_command_term = self._env.command_manager.get_term(
                self._racket_command_name
            )
        return self._racket_command_term

    def _sample_upper_action_intervention_target(
        self, env_mask: torch.Tensor, current_action: torch.Tensor
    ) -> None:
        if self._upper_intervention_cols.numel() == 0 or not bool(env_mask.any()):
            return
        rows = torch.where(env_mask)[0]
        current = current_action.index_select(-1, self._upper_intervention_cols)[rows]
        self._upper_intervention_start_action[rows] = current
        self._upper_intervention_current_action[rows] = current
        random_unit = torch.rand_like(current)
        self._upper_intervention_target_action[rows] = (
            self._upper_intervention_action_lo
            + random_unit
            * (self._upper_intervention_action_hi - self._upper_intervention_action_lo)
        )
        self._upper_intervention_age[rows] = 0

    def _apply_upper_action_intervention(self, policy_action: torch.Tensor) -> torch.Tensor:
        """Apply HUGWBC action replacement in policy-action space.

        This is the absolute-v5 intervention contract. Incremental feasible-v3 actions are
        velocities inside a moving feasible interval, so holding a non-zero action target would
        integrate q_des into a rail. V3 uses the bounded q_des-space intervention below.

        The sampled target is held for ``resample_steps`` and reached by linear interpolation over
        the first two thirds of that interval, matching the published HUGWBC intervention rather
        than clamping a random q target into a moving measured-q-centered radius.
        """
        if not self._upper_intervention_enabled or self._upper_intervention_cols.numel() == 0:
            self._upper_intervention_effective.zero_()
            return policy_action

        cmd = self._racket_command()
        replaced_action = policy_action.clone()
        supervision = cmd.upper_intervention_supervision()
        entering = supervision & ~self._upper_intervention_prev_supervision
        entering_group = entering & self._upper_intervention_group
        if bool(entering_group.any()):
            draws = torch.rand(self._num_envs, device=self.device) < 0.5
            self._upper_intervention_mask[entering_group] = draws[entering_group]
            self._sample_upper_action_intervention_target(entering_group, policy_action)

        switch = (
            supervision
            & self._upper_intervention_group
            & (torch.rand(self._num_envs, device=self.device) < self._upper_intervention_switch_prob)
        )
        self._upper_intervention_mask[switch] = ~self._upper_intervention_mask[switch]
        newly_active = switch & self._upper_intervention_mask
        self._sample_upper_action_intervention_target(newly_active, policy_action)

        strength_per_env = cmd.upper_intervention_strength()
        effective = (
            supervision
            & self._upper_intervention_group
            & self._upper_intervention_mask
            & (strength_per_env > 0.0)
        )
        due = effective & (self._upper_intervention_age >= self._upper_intervention_resample_steps)
        if bool(due.any()):
            current_action = policy_action.clone()
            current_action.index_copy_(
                -1, self._upper_intervention_cols, self._upper_intervention_current_action
            )
            self._sample_upper_action_intervention_target(due, current_action)
        self._upper_intervention_effective.copy_(effective)
        self._upper_intervention_prev_supervision.copy_(supervision)

        if bool(effective.any()):
            cols = self._upper_intervention_cols
            policy_subset = policy_action.index_select(-1, cols)
            ramp_steps = max((2.0 / 3.0) * self._upper_intervention_resample_steps, 1.0)
            ratio = torch.clamp(
                self._upper_intervention_age.to(policy_action.dtype) / ramp_steps,
                min=0.0,
                max=1.0,
            ).unsqueeze(-1)
            noise_action = self._upper_intervention_start_action + ratio * (
                self._upper_intervention_target_action
                - self._upper_intervention_start_action
            )
            self._upper_intervention_current_action.copy_(noise_action)
            strength = strength_per_env.unsqueeze(-1)
            blended = (1.0 - strength) * policy_subset + strength * noise_action
            replaced_subset = torch.where(effective.unsqueeze(-1), blended, policy_subset)
            replaced_action.index_copy_(-1, cols, replaced_subset)
            self._upper_intervention_age[effective] += 1

        cmd.metrics["upper_intervention_active"] = effective.float()
        return replaced_action

    def _sample_upper_qdes_intervention_target(
        self, env_mask: torch.Tensor, measured_q: torch.Tensor
    ) -> None:
        if self._upper_intervention_cols.numel() == 0 or not bool(env_mask.any()):
            return
        rows = torch.where(env_mask)[0]
        center = measured_q.index_select(-1, self._upper_intervention_cols)[rows]
        lo = self._safe_lo.index_select(-1, self._upper_intervention_cols)[rows]
        hi = self._safe_hi.index_select(-1, self._upper_intervention_cols)[rows]
        offset = (2.0 * torch.rand_like(center) - 1.0) * self._upper_intervention_radius_rad
        self._upper_intervention_target_q[rows] = torch.minimum(
            torch.maximum(center + offset, lo), hi
        )
        self._upper_intervention_age[rows] = 0

    def _apply_upper_qdes_intervention(
        self,
        measured_q: torch.Tensor,
        selected_lo: torch.Tensor,
        selected_hi: torch.Tensor,
    ) -> torch.Tensor:
        """Apply the feasible-v3 intervention as a bounded posture disturbance.

        Each segment samples one fixed target inside a radius around the measured posture, then
        constrains it by the same executable interval as the policy. The target does not follow
        the live posture, so a segment cannot integrate q_des toward a joint rail.
        """
        if not self._upper_intervention_enabled or self._upper_intervention_cols.numel() == 0:
            self._upper_intervention_effective.zero_()
            return self._qdes_nominal.clone()

        cmd = self._racket_command()
        supervision = cmd.upper_intervention_supervision()
        entering = supervision & ~self._upper_intervention_prev_supervision
        entering_group = entering & self._upper_intervention_group
        if bool(entering_group.any()):
            draws = torch.rand(self._num_envs, device=self.device) < 0.5
            self._upper_intervention_mask[entering_group] = draws[entering_group]
            self._sample_upper_qdes_intervention_target(entering_group, measured_q)

        switch = (
            supervision
            & self._upper_intervention_group
            & (torch.rand(self._num_envs, device=self.device) < self._upper_intervention_switch_prob)
        )
        self._upper_intervention_mask[switch] = ~self._upper_intervention_mask[switch]
        newly_active = switch & self._upper_intervention_mask
        self._sample_upper_qdes_intervention_target(newly_active, measured_q)

        strength_per_env = cmd.upper_intervention_strength()
        effective = (
            supervision
            & self._upper_intervention_group
            & self._upper_intervention_mask
            & (strength_per_env > 0.0)
        )
        due = effective & (self._upper_intervention_age >= self._upper_intervention_resample_steps)
        self._sample_upper_qdes_intervention_target(due, measured_q)
        self._upper_intervention_effective.copy_(effective)
        self._upper_intervention_prev_supervision.copy_(supervision)

        if bool(effective.any()):
            cols = self._upper_intervention_cols
            policy_q = self._qdes_nominal.index_select(-1, cols)
            strength = strength_per_env.unsqueeze(-1)
            target = (
                (1.0 - strength) * policy_q
                + strength * self._upper_intervention_target_q
            )
            replaced = torch.where(effective.unsqueeze(-1), target, policy_q)
            self._qdes_nominal.index_copy_(-1, cols, replaced)
            self._upper_intervention_age[effective] += 1

        cmd.metrics["upper_intervention_active"] = effective.float()
        return torch.minimum(torch.maximum(self._qdes_nominal, selected_lo), selected_hi)

    def _latch_safety_fault(
        self,
        kind: str,
        mask: torch.Tensor,
        **state: torch.Tensor,
    ) -> None:
        """Persist a compact q_des/numeric first-fault snapshot, then stop the run."""

        self._safety_fault_latched = True
        path, indices = self._write_safety_fault_snapshot(kind, mask, **state)
        first = indices[0].detach().cpu().tolist() if indices.numel() else []
        raise RuntimeError(
            "V15 q_des safety fail-fast: "
            f"kind={kind}, first_index={first}, snapshot={path}"
        )

    def _write_safety_fault_snapshot(
        self,
        kind: str,
        mask: torch.Tensor,
        **state: torch.Tensor,
    ) -> tuple[str, torch.Tensor]:
        """Persist one compact safety snapshot and return its path and indices."""

        stamp = time.time_ns()
        path = pathlib.Path(
            f"/tmp/v15_qdes_safety_fault_{os.getpid()}_{stamp}.pt"
        )
        indices = torch.nonzero(mask, as_tuple=False)
        payload = {
            "contract": self._action_contract,
            "kind": kind,
            "joint_names": list(self._joint_names),
            "violating_indices": indices.detach().cpu(),
            "state": {
                label: value.detach().cpu()
                for label, value in state.items()
                if torch.is_tensor(value)
            },
        }
        try:
            torch.save(payload, path)
            self._last_safety_fault_path = str(path)
        except Exception as exc:  # pragma: no cover - only an already-fatal I/O path
            self._last_safety_fault_path = f"<snapshot failed: {exc!r}>"
        return self._last_safety_fault_path, indices

    def _record_physical_safety_fault(
        self,
        mask: torch.Tensor,
        **state: torch.Tensor,
    ) -> None:
        """Latch and snapshot the first physical fault without killing other environments.

        Actual joint state is a plant outcome, not a q_des contract output.  A violating
        simulation must be terminated and reset, but one bad domain-randomized episode must not
        abort a vectorized PPO run.  Numeric and final-q_des faults still use the fatal path above.
        """

        if self._physical_safety_fault_latched:
            return
        self._physical_safety_fault_latched = True
        self._write_safety_fault_snapshot(
            "actual_q_hard_limit",
            mask,
            **state,
        )

    def _accumulate_safety_violation(
        self,
        label: str,
        magnitude: torch.Tensor,
        *,
        tolerance: float | None = None,
    ) -> torch.Tensor:
        threshold = self._safety_tolerance if tolerance is None else float(tolerance)
        mask = magnitude > threshold
        self._safety_violation_count[label].add_(mask.sum(dim=0))
        self._safety_violation_max[label].copy_(
            torch.maximum(
                self._safety_violation_max[label],
                magnitude.max(dim=0).values,
            )
        )
        return mask

    def _audit_actual_q(
        self, raw_action: torch.Tensor, q: torch.Tensor, qd: torch.Tensor
    ) -> torch.Tensor:
        finite = feasible_qdes_v3_dynamic_inputs_finite(
            torch,
            raw_action,
            self._qdes_executed,
            q,
            qd,
        )
        if not bool(finite.all()):
            self._q_nonfinite_latched |= not bool(
                torch.isfinite(q).all() and torch.isfinite(qd).all()
            )
            self._qdes_nonfinite_latched |= not bool(
                torch.isfinite(self._qdes_executed).all()
            )
            self._latch_safety_fault(
                "numeric_nonfinite_input",
                ~finite,
                raw_action=raw_action,
                q=q,
                qd=qd,
                previous_qdes=self._qdes_executed,
            )
        lower_mag = torch.clamp(self._hard_lo - q, min=0.0)
        upper_mag = torch.clamp(q - self._hard_hi, min=0.0)
        # Record every representable excursion as audit evidence, including solver slop.
        self._accumulate_safety_violation(
            "q_hard_lower", lower_mag, tolerance=0.0
        )
        self._accumulate_safety_violation(
            "q_hard_upper", upper_mag, tolerance=0.0
        )
        # Retain every representable hard-bound crossing in the raw counters above.  PhysX can
        # penetrate a joint stop by sub-milliradian solver slop, so only the separately configured
        # excess threshold is a physical episode fault.
        physical = (
            lower_mag > self._actual_q_hard_tolerance_rad
        ) | (
            upper_mag > self._actual_q_hard_tolerance_rad
        )
        if bool(physical.any()):
            self._record_physical_safety_fault(
                physical,
                raw_action=raw_action,
                q=q,
                qd=qd,
                hard_lo=self._hard_lo,
                hard_hi=self._hard_hi,
                hard_tolerance_rad=torch.tensor(
                    self._actual_q_hard_tolerance_rad, device=self.device
                ),
                previous_qdes=self._qdes_executed,
            )
        return physical.any(dim=-1)

    def audit_actual_q_post_physics(self) -> torch.Tensor:
        """Return environments whose hard-limit excess crossed the physical tolerance."""

        q = self._select_asset_data(self._asset.data.joint_pos)
        qd = self._select_asset_data(self._asset.data.joint_vel)
        return self._audit_actual_q(self._raw_actions, q, qd)

    def _audit_final_qdes(
        self,
        raw_action: torch.Tensor,
        previous_qdes: torch.Tensor,
        q: torch.Tensor,
        qd: torch.Tensor,
        projected: torch.Tensor,
        contract,
    ) -> None:
        """Update all-joint runtime counters and enforce the safe-position invariant."""

        if not bool(torch.isfinite(projected).all()):
            self._qdes_nonfinite_latched = True
            self._latch_safety_fault(
                "numeric_nonfinite_final_qdes",
                ~torch.isfinite(projected),
                raw_action=raw_action,
                q=q,
                qd=qd,
                previous_qdes=previous_qdes,
                qdes=projected,
                selected_lo=contract.selected_lo,
                selected_hi=contract.selected_hi,
                rate_lo=contract.rate_lo,
                rate_hi=contract.rate_hi,
                tracking_lo=contract.tracking_lo,
                tracking_hi=contract.tracking_hi,
                torque_lo=contract.torque_lo,
                torque_hi=contract.torque_hi,
            )

        qdes_safe_lower_mag = torch.clamp(self._safe_lo - projected, min=0.0)
        qdes_safe_upper_mag = torch.clamp(projected - self._safe_hi, min=0.0)
        qdes_safe_lower = self._accumulate_safety_violation(
            "qdes_safe_lower", qdes_safe_lower_mag, tolerance=0.0
        )
        qdes_safe_upper = self._accumulate_safety_violation(
            "qdes_safe_upper", qdes_safe_upper_mag, tolerance=0.0
        )
        rate_mag = torch.maximum(
            torch.clamp(contract.rate_lo - projected, min=0.0),
            torch.clamp(projected - contract.rate_hi, min=0.0),
        )
        tracking_mag = torch.maximum(
            torch.clamp(contract.tracking_lo - projected, min=0.0),
            torch.clamp(projected - contract.tracking_hi, min=0.0),
        )
        torque_mag = torch.maximum(
            torch.clamp(contract.torque_lo - projected, min=0.0),
            torch.clamp(projected - contract.torque_hi, min=0.0),
        )
        self._accumulate_safety_violation("qdes_rate", rate_mag)
        self._accumulate_safety_violation("qdes_tracking", tracking_mag)
        self._accumulate_safety_violation("qdes_torque", torque_mag)

        sample_count = projected.shape[0]
        self._safety_audit_samples.add_(sample_count)
        self._interval_empty_count.add_((contract.fallback_level > 0).sum(dim=0))
        for level in range(4):
            self._fallback_level_count[:, level].add_(
                (contract.fallback_level == level).sum(dim=0)
            )
            self._tightest_constraint_count[:, level].add_(
                (contract.tightest_constraint == level).sum(dim=0)
            )
        safe_margin = torch.minimum(
            projected - self._safe_lo, self._safe_hi - projected
        )
        hard_margin = torch.minimum(q - self._hard_lo, self._hard_hi - q)
        self._qdes_safe_margin_min.copy_(
            torch.minimum(self._qdes_safe_margin_min, safe_margin.min(dim=0).values)
        )
        self._q_hard_margin_min.copy_(
            torch.minimum(self._q_hard_margin_min, hard_margin.min(dim=0).values)
        )
        self._final_interval_width_min.copy_(
            torch.minimum(
                self._final_interval_width_min,
                (contract.selected_hi - contract.selected_lo).min(dim=0).values,
            )
        )
        utilization = torch.abs(
            (projected - self._safe_mid) / self._safe_half.clamp_min(1.0e-9)
        ).clamp(0.0, 1.0)
        q_hard_utilization = torch.abs(
            (q - self._hard_mid) / self._hard_half.clamp_min(1.0e-9)
        ).clamp(0.0, 1.0)
        self._qdes_near_limit_count.add_((utilization >= 0.90).sum(dim=0))
        self._q_hard_near_limit_count.add_(
            (q_hard_utilization >= 0.90).sum(dim=0)
        )
        bins = self._safety_histogram_bins
        offsets = (
            torch.arange(projected.shape[1], device=self.device).unsqueeze(0) * bins
        )
        for values, histogram in (
            (utilization, self._qdes_utilization_histogram),
            (q_hard_utilization, self._q_hard_utilization_histogram),
        ):
            bin_index = torch.floor(values * (bins - 1)).long()
            update = torch.bincount(
                (bin_index + offsets).reshape(-1),
                minlength=projected.shape[1] * bins,
            ).reshape(projected.shape[1], bins)
            histogram.add_(update)

        safe_fault = qdes_safe_lower | qdes_safe_upper
        if bool(safe_fault.any()):
            self._latch_safety_fault(
                "final_qdes_safe_limit",
                safe_fault,
                raw_action=raw_action,
                q=q,
                qd=qd,
                previous_qdes=previous_qdes,
                qdes=projected,
                safe_lo=self._safe_lo,
                safe_hi=self._safe_hi,
                selected_lo=contract.selected_lo,
                selected_hi=contract.selected_hi,
                rate_lo=contract.rate_lo,
                rate_hi=contract.rate_hi,
                tracking_lo=contract.tracking_lo,
                tracking_hi=contract.tracking_hi,
                torque_lo=contract.torque_lo,
                torque_hi=contract.torque_hi,
                fallback_level=contract.fallback_level,
            )

    def process_actions(self, actions: torch.Tensor):
        # Let Isaac Lab own raw-action validation/copying, then replace its affine decoder with
        # the V15 contract below.
        super().process_actions(actions)
        q = self._select_asset_data(self._asset.data.joint_pos)
        qd = self._select_asset_data(self._asset.data.joint_vel)
        # This pre-physics pass owns only numerical fail-fast.  Physical hard-limit accounting is
        # post-physics in the termination manager so every joint-step is counted exactly once.
        finite = feasible_qdes_v3_dynamic_inputs_finite(
            torch,
            self._raw_actions,
            self._qdes_executed,
            q,
            qd,
        )
        if not bool(finite.all()):
            self._q_nonfinite_latched |= not bool(
                torch.isfinite(q).all() and torch.isfinite(qd).all()
            )
            self._qdes_nonfinite_latched |= not bool(
                torch.isfinite(self._qdes_executed).all()
            )
            self._latch_safety_fault(
                "numeric_nonfinite_input",
                ~finite,
                raw_action=self._raw_actions,
                q=q,
                qd=qd,
                previous_qdes=self._qdes_executed,
            )

        uninitialized = ~self._projector_initialized
        if bool(uninitialized.any()):
            seed = torch.clamp(q, min=self._safe_lo, max=self._safe_hi)
            self._qdes_executed[uninitialized] = seed[uninitialized]
            self._projector_initialized[uninitialized] = True

        contract = feasible_qdes_v3(
            torch,
            self._raw_actions,
            self._qdes_executed,
            q,
            qd,
            self._safe_lo,
            self._safe_hi,
            self._qdes_rate_limit,
            self._tracking_error_limit,
            self._projector_kp,
            self._projector_kd,
            self._effort_limit,
            self._torque_headroom_fraction,
            self._step_dt,
            self._actual_q_guard_enabled,
            self._actual_q_guard_horizon_s,
        )
        rate_delta = self._qdes_rate_limit * self._step_dt
        rate_lo, rate_hi = contract.rate_lo, contract.rate_hi
        tracking_lo, tracking_hi = contract.tracking_lo, contract.tracking_hi
        torque_lo, torque_hi = contract.torque_lo, contract.torque_hi
        safe_torque_lo, safe_torque_hi = (
            contract.safe_torque_lo,
            contract.safe_torque_hi,
        )
        base_lo, base_hi = contract.base_lo, contract.base_hi
        feasible = contract.full_feasible
        selected_lo, selected_hi = contract.selected_lo, contract.selected_hi
        self._selected_lo.copy_(selected_lo)
        self._selected_hi.copy_(selected_hi)
        self._fallback_level.copy_(contract.fallback_level)
        self._tightest_constraint.copy_(contract.tightest_constraint)
        self._actual_q_guard_active.copy_(contract.actual_q_guard_active)
        self._actual_q_guard_upper.copy_(contract.actual_q_guard_upper)
        self._actual_q_guard_lower.copy_(contract.actual_q_guard_lower)
        self._actual_q_guard_count.add_(
            contract.actual_q_guard_active.sum(dim=0)
        )
        self._actual_q_guard_upper_count.add_(
            contract.actual_q_guard_upper.sum(dim=0)
        )
        self._actual_q_guard_lower_count.add_(
            contract.actual_q_guard_lower.sum(dim=0)
        )

        previous_qdes = self._qdes_executed.clone()
        decoder_action = (
            self._raw_actions
            if self._action_contract == FEASIBLE_QDES_ACTION_CONTRACT
            else self._apply_upper_action_intervention(self._raw_actions)
        )
        if self._action_contract == FEASIBLE_QDES_ACTION_CONTRACT:
            self._qdes_nominal.copy_(contract.qdes)
            projected = self._qdes_nominal.clone()
            self._feasible_action_utilization.copy_(
                torch.abs(contract.normalized_action)
            )
            self._decode_utilization.zero_()
            projected = self._apply_upper_qdes_intervention(q, selected_lo, selected_hi)
        elif self._action_contract == ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT:
            room = torch.where(
                decoder_action >= 0.0, self._decode_room_pos, self._decode_room_neg
            )
            saturation = torch.tanh(self._decode_scale * decoder_action / room)
            self._qdes_nominal.copy_(self._default_q + room * saturation)
            projected = torch.minimum(
                torch.maximum(self._qdes_nominal, selected_lo), selected_hi
            )
            self._feasible_action_utilization.zero_()
            # Fraction of the per-side room consumed by the requested posture; the v5
            # saturation debt charges only the tail of this quantity.
            self._decode_utilization.copy_(torch.abs(saturation))
        else:
            self._qdes_nominal.copy_(
                self._safe_mid
                + self._safe_half
                * torch.tanh(self._tanh_input_gain * decoder_action + self._tanh_bias)
            )
            projected = torch.minimum(
                torch.maximum(self._qdes_nominal, selected_lo), selected_hi
            )
            self._feasible_action_utilization.zero_()
            self._decode_utilization.zero_()

        if self._passive_action_cols.numel() > 0:
            passive_default = self._default_q.index_select(-1, self._passive_action_cols)
            passive_lo = selected_lo.index_select(-1, self._passive_action_cols)
            passive_hi = selected_hi.index_select(-1, self._passive_action_cols)
            # The default is strictly position-safe, but can be outside a transient rate/tracking
            # interval after reset or disturbance.  The final plant target must obey the same
            # selected interval as every other joint.
            passive_target = torch.minimum(
                torch.maximum(passive_default, passive_lo), passive_hi
            )
            projected.index_copy_(-1, self._passive_action_cols, passive_target)

        eps = 1.0e-7
        if self._action_contract == FEASIBLE_QDES_ACTION_CONTRACT:
            # For V3 these booleans mean "this constraint reduced the available action interval",
            # not "a post-policy projector changed the command".  The latter must remain zero on
            # every feasible, non-intervened policy channel by construction.
            self._rate_active.zero_()
            self._tracking_active.zero_()
            self._torque_active.zero_()
            self._feasible_rate_bound.copy_(
                feasible & ((rate_lo > base_lo + eps) | (rate_hi < base_hi - eps))
            )
            self._feasible_tracking_bound.copy_(
                (tracking_lo > safe_torque_lo + eps)
                | (tracking_hi < safe_torque_hi - eps)
            )
            self._feasible_torque_bound.copy_(
                (torque_lo > self._safe_lo + eps) | (torque_hi < self._safe_hi - eps)
            )
        else:
            self._feasible_rate_bound.zero_()
            self._feasible_tracking_bound.zero_()
            self._feasible_torque_bound.zero_()
            self._rate_active.copy_(
                (self._qdes_nominal < rate_lo - eps)
                | (self._qdes_nominal > rate_hi + eps)
            )
            self._tracking_active.copy_(
                (self._qdes_nominal < tracking_lo - eps)
                | (self._qdes_nominal > tracking_hi + eps)
            )
            self._torque_active.copy_(
                (self._qdes_nominal < torque_lo - eps)
                | (self._qdes_nominal > torque_hi + eps)
            )
        self._infeasible.copy_(~feasible)
        self._projection_active.copy_(torch.abs(projected - self._qdes_nominal) > eps)
        self._projection_error_normalized.copy_(
            torch.abs(projected - self._qdes_nominal) / self._safe_half.clamp_min(1.0e-6)
        )
        self._feasible_interval_width_fraction.copy_(
            (selected_hi - selected_lo)
            / (self._safe_hi - self._safe_lo).clamp_min(1.0e-6)
        )
        self._qdes_rate_utilization.copy_(
            torch.where(
                rate_delta > eps,
                torch.abs(projected - previous_qdes) / rate_delta.clamp_min(eps),
                torch.zeros_like(projected),
            )
        )
        self._audit_final_qdes(
            self._raw_actions,
            previous_qdes,
            q,
            qd,
            projected,
            contract,
        )

        # Passive head channels have their own raw-action debt.  They are not part of the
        # executable q_des contract, so do not count their forced default targets as projector
        # intervention or infeasibility.
        if self._passive_action_cols.numel() > 0:
            self._projection_error_normalized.index_fill_(
                -1, self._passive_action_cols, 0.0
            )
            self._projection_active.index_fill_(-1, self._passive_action_cols, False)
            self._rate_active.index_fill_(-1, self._passive_action_cols, False)
            self._tracking_active.index_fill_(-1, self._passive_action_cols, False)
            self._torque_active.index_fill_(-1, self._passive_action_cols, False)
            self._infeasible.index_fill_(-1, self._passive_action_cols, False)
            self._feasible_rate_bound.index_fill_(
                -1, self._passive_action_cols, False
            )
            self._feasible_tracking_bound.index_fill_(
                -1, self._passive_action_cols, False
            )
            self._feasible_torque_bound.index_fill_(
                -1, self._passive_action_cols, False
            )
            self._feasible_action_utilization.index_fill_(
                -1, self._passive_action_cols, 0.0
            )
            self._feasible_interval_width_fraction.index_fill_(
                -1, self._passive_action_cols, 0.0
            )
            self._qdes_rate_utilization.index_fill_(
                -1, self._passive_action_cols, 0.0
            )
            self._decode_utilization.index_fill_(-1, self._passive_action_cols, 0.0)

        # The intervention group deliberately executes a target not selected by the actor.  Its
        # shoulder/elbow difference is therefore not policy projection debt.  Keep projector
        # diagnostics for every untouched joint and for all non-intervened environments.
        if self._upper_intervention_cols.numel() > 0 and bool(
            self._upper_intervention_effective.any()
        ):
            rows = torch.where(self._upper_intervention_effective)[0]
            cols = self._upper_intervention_cols
            self._projection_error_normalized[rows[:, None], cols[None, :]] = 0.0
            self._projection_active[rows[:, None], cols[None, :]] = False
            self._rate_active[rows[:, None], cols[None, :]] = False
            self._tracking_active[rows[:, None], cols[None, :]] = False
            self._torque_active[rows[:, None], cols[None, :]] = False
            self._infeasible[rows[:, None], cols[None, :]] = False
            self._feasible_rate_bound[rows[:, None], cols[None, :]] = False
            self._feasible_tracking_bound[rows[:, None], cols[None, :]] = False
            self._feasible_torque_bound[rows[:, None], cols[None, :]] = False
            self._feasible_action_utilization[rows[:, None], cols[None, :]] = 0.0
            self._qdes_rate_utilization[rows[:, None], cols[None, :]] = 0.0
            self._decode_utilization[rows[:, None], cols[None, :]] = 0.0

        executed_delta = projected - previous_qdes
        self._executed_qdes_delta.copy_(executed_delta)
        self._executed_qdes_second_difference.copy_(
            executed_delta - self._previous_executed_qdes_delta
        )
        self._executed_qdes_delta_normalized.copy_(
            executed_delta / self._feedback_scale.clamp_min(1.0e-6)
        )
        self._executed_qdes_second_difference_normalized.copy_(
            self._executed_qdes_second_difference
            / self._feedback_scale.clamp_min(1.0e-6)
        )
        self._previous_executed_qdes_delta.copy_(executed_delta)
        self._qdes_executed.copy_(projected)
        self._processed_actions = self._qdes_executed
        self._executed_feedback.copy_(
            ((self._qdes_executed - self._default_q) / self._feedback_scale).clamp(-1.0, 1.0)
        )

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if env_ids is None:
            self._projector_initialized.zero_()
            self._qdes_nominal.copy_(self._default_q)
            self._qdes_executed.copy_(self._default_q)
            self._executed_feedback.zero_()
            self._projection_error_normalized.zero_()
            self._projection_active.zero_()
            self._rate_active.zero_()
            self._tracking_active.zero_()
            self._torque_active.zero_()
            self._infeasible.zero_()
            self._feasible_rate_bound.zero_()
            self._feasible_tracking_bound.zero_()
            self._feasible_torque_bound.zero_()
            self._feasible_action_utilization.zero_()
            self._feasible_interval_width_fraction.zero_()
            self._qdes_rate_utilization.zero_()
            self._decode_utilization.zero_()
            self._actual_q_guard_active.zero_()
            self._actual_q_guard_upper.zero_()
            self._actual_q_guard_lower.zero_()
            self._executed_qdes_delta.zero_()
            self._previous_executed_qdes_delta.zero_()
            self._executed_qdes_second_difference.zero_()
            self._executed_qdes_delta_normalized.zero_()
            self._executed_qdes_second_difference_normalized.zero_()
            self._upper_intervention_mask.zero_()
            self._upper_intervention_effective.zero_()
            self._upper_intervention_prev_supervision.zero_()
            self._upper_intervention_age.zero_()
            self._upper_intervention_start_action.zero_()
            self._upper_intervention_target_action.zero_()
            self._upper_intervention_current_action.zero_()
            self._upper_intervention_target_q.copy_(
                self._default_q.index_select(-1, self._upper_intervention_cols)
            )
        else:
            self._projector_initialized[env_ids] = False
            self._qdes_nominal[env_ids] = self._default_q[env_ids]
            self._qdes_executed[env_ids] = self._default_q[env_ids]
            self._executed_feedback[env_ids] = 0.0
            self._projection_error_normalized[env_ids] = 0.0
            self._projection_active[env_ids] = False
            self._rate_active[env_ids] = False
            self._tracking_active[env_ids] = False
            self._torque_active[env_ids] = False
            self._infeasible[env_ids] = False
            self._feasible_rate_bound[env_ids] = False
            self._feasible_tracking_bound[env_ids] = False
            self._feasible_torque_bound[env_ids] = False
            self._feasible_action_utilization[env_ids] = 0.0
            self._feasible_interval_width_fraction[env_ids] = 0.0
            self._qdes_rate_utilization[env_ids] = 0.0
            self._decode_utilization[env_ids] = 0.0
            self._actual_q_guard_active[env_ids] = False
            self._actual_q_guard_upper[env_ids] = False
            self._actual_q_guard_lower[env_ids] = False
            self._executed_qdes_delta[env_ids] = 0.0
            self._previous_executed_qdes_delta[env_ids] = 0.0
            self._executed_qdes_second_difference[env_ids] = 0.0
            self._executed_qdes_delta_normalized[env_ids] = 0.0
            self._executed_qdes_second_difference_normalized[env_ids] = 0.0
            self._upper_intervention_mask[env_ids] = False
            self._upper_intervention_effective[env_ids] = False
            self._upper_intervention_prev_supervision[env_ids] = False
            self._upper_intervention_age[env_ids] = 0
            self._upper_intervention_start_action[env_ids] = 0.0
            self._upper_intervention_target_action[env_ids] = 0.0
            self._upper_intervention_current_action[env_ids] = 0.0
            self._upper_intervention_target_q[env_ids] = self._default_q[
                env_ids
            ].index_select(-1, self._upper_intervention_cols)

    @property
    def executed_qdes_feedback(self) -> torch.Tensor:
        # Before the first action after a reset, report the current measured posture in the same
        # normalized coordinates. This keeps post-swing RSI states consistent instead of pairing
        # a dynamic follow-through pose with an artificial zero action history.
        measured = self._select_asset_data(self._asset.data.joint_pos)
        measured_feedback = (
            (measured - self._default_q) / self._feedback_scale
        ).clamp(-1.0, 1.0)
        return torch.where(
            self._projector_initialized.unsqueeze(-1),
            self._executed_feedback,
            measured_feedback,
        )

    @property
    def applied_raw_actions(self) -> torch.Tensor:
        return self.executed_qdes_feedback

    @property
    def unclamped_processed_actions(self) -> torch.Tensor:
        return self._qdes_nominal

    @property
    def passive_joint_names(self) -> tuple[str, ...]:
        return self._passive_joint_names

    @property
    def projection_error_normalized(self) -> torch.Tensor:
        return self._projection_error_normalized

    @property
    def projection_active(self) -> torch.Tensor:
        return self._projection_active

    @property
    def rate_projection_active(self) -> torch.Tensor:
        return self._rate_active

    @property
    def tracking_projection_active(self) -> torch.Tensor:
        return self._tracking_active

    @property
    def torque_projection_active(self) -> torch.Tensor:
        return self._torque_active

    @property
    def projection_infeasible(self) -> torch.Tensor:
        return self._infeasible

    @property
    def feasible_action_utilization(self) -> torch.Tensor:
        return self._feasible_action_utilization

    @property
    def feasible_interval_width_fraction(self) -> torch.Tensor:
        return self._feasible_interval_width_fraction

    @property
    def qdes_rate_utilization(self) -> torch.Tensor:
        return self._qdes_rate_utilization

    @property
    def decode_utilization(self) -> torch.Tensor:
        """|tanh| of the v5 per-side decode: fraction of the room to the rail requested."""
        return self._decode_utilization

    @property
    def feasible_rate_bound_active(self) -> torch.Tensor:
        return self._feasible_rate_bound

    @property
    def feasible_tracking_bound_active(self) -> torch.Tensor:
        return self._feasible_tracking_bound

    @property
    def feasible_torque_bound_active(self) -> torch.Tensor:
        return self._feasible_torque_bound

    @property
    def qdes_debug_joints(self) -> tuple[tuple[str, int], ...]:
        """YAML-selected action joints whose projector state is logged at exact strike."""
        return tuple(zip(self._qdes_debug_joint_names, self._qdes_debug_cols))

    @property
    def upper_intervention_indicator(self) -> torch.Tensor:
        return self._upper_intervention_effective.float().unsqueeze(-1)

    @property
    def executed_qdes_delta(self) -> torch.Tensor:
        return self._executed_qdes_delta

    @property
    def executed_qdes_second_difference(self) -> torch.Tensor:
        return self._executed_qdes_second_difference

    @property
    def executed_qdes_delta_normalized(self) -> torch.Tensor:
        return self._executed_qdes_delta_normalized

    @property
    def executed_qdes_second_difference_normalized(self) -> torch.Tensor:
        return self._executed_qdes_second_difference_normalized

    def _utilization_quantile(
        self, histogram: torch.Tensor, probability: float
    ) -> torch.Tensor:
        total = histogram.sum(dim=1)
        target = torch.ceil(total.float() * float(probability)).long().clamp_min(1)
        index = (histogram.cumsum(dim=1) >= target.unsqueeze(1)).long().argmax(dim=1)
        value = index.float() / float(self._safety_histogram_bins - 1)
        return torch.where(total > 0, value, torch.zeros_like(value))

    def safety_instrumentation_scalars(self) -> dict[str, torch.Tensor]:
        """Cumulative all-step, all-joint q_des/actual-q safety evidence."""

        result: dict[str, torch.Tensor] = {}
        samples = self._safety_audit_samples.clamp_min(1)
        total_samples = samples.sum().clamp_min(1)
        result["qdes_isfinite"] = torch.tensor(
            0.0 if self._qdes_nonfinite_latched else 1.0, device=self.device
        )
        result["q_isfinite"] = torch.tensor(
            0.0 if self._q_nonfinite_latched else 1.0,
            device=self.device,
        )
        result["numeric_or_qdes_safety_fault_latched"] = torch.tensor(
            float(self._safety_fault_latched), device=self.device
        )
        result["physical_q_hard_fault_latched"] = torch.tensor(
            float(self._physical_safety_fault_latched), device=self.device
        )
        result["actual_q_hard_tolerance_rad"] = torch.tensor(
            self._actual_q_hard_tolerance_rad, device=self.device
        )
        result["interval_empty_count"] = self._interval_empty_count.sum()
        result["interval_empty_fraction"] = (
            self._interval_empty_count.sum().float() / total_samples.float()
        )
        result["actual_q_guard_count"] = self._actual_q_guard_count.sum()
        result["actual_q_guard_fraction"] = (
            self._actual_q_guard_count.sum().float() / total_samples.float()
        )
        result["actual_q_guard_upper_count"] = (
            self._actual_q_guard_upper_count.sum()
        )
        result["actual_q_guard_lower_count"] = (
            self._actual_q_guard_lower_count.sum()
        )
        result["safe_interval_width_min_rad"] = (
            self._safe_interval_width_min.min()
        )
        result["final_interval_width_min_rad"] = (
            self._final_interval_width_min.min()
        )
        for level in range(4):
            result[f"fallback_level_{level}_fraction"] = (
                self._fallback_level_count[:, level].sum().float()
                / total_samples.float()
            )
        for label in self._safety_violation_count:
            result[f"{label}_violation_count"] = self._safety_violation_count[
                label
            ].sum()
            result[f"{label}_violation_max_rad"] = self._safety_violation_max[
                label
            ].max()

        qdes_p99 = self._utilization_quantile(
            self._qdes_utilization_histogram, 0.99
        )
        qdes_p999 = self._utilization_quantile(
            self._qdes_utilization_histogram, 0.999
        )
        q_hard_p99 = self._utilization_quantile(
            self._q_hard_utilization_histogram, 0.99
        )
        q_hard_p999 = self._utilization_quantile(
            self._q_hard_utilization_histogram, 0.999
        )
        joint_labels = [
            str(name).removesuffix("_joint") for name in self._joint_names
        ]
        for joint, label in enumerate(joint_labels):
            prefix = f"joint/{label}"
            denom = samples[joint].float()
            result[f"{prefix}/sample_count"] = self._safety_audit_samples[joint]
            for violation in self._safety_violation_count:
                result[f"{prefix}/{violation}_violation_count"] = (
                    self._safety_violation_count[violation][joint]
                )
                result[f"{prefix}/{violation}_violation_max_rad"] = (
                    self._safety_violation_max[violation][joint]
                )
            result[f"{prefix}/qdes_utilization_p99"] = qdes_p99[joint]
            result[f"{prefix}/qdes_utilization_p99_9"] = qdes_p999[joint]
            result[f"{prefix}/q_hard_utilization_p99"] = q_hard_p99[joint]
            result[f"{prefix}/q_hard_utilization_p99_9"] = q_hard_p999[
                joint
            ]
            result[f"{prefix}/qdes_safe_margin_min_rad"] = (
                self._qdes_safe_margin_min[joint]
            )
            result[f"{prefix}/q_hard_margin_min_rad"] = self._q_hard_margin_min[
                joint
            ]
            result[f"{prefix}/qdes_near_limit_fraction"] = (
                self._qdes_near_limit_count[joint].float() / denom
            )
            result[f"{prefix}/q_hard_near_limit_fraction"] = (
                self._q_hard_near_limit_count[joint].float() / denom
            )
            result[f"{prefix}/interval_empty_fraction"] = (
                self._interval_empty_count[joint].float() / denom
            )
            result[f"{prefix}/actual_q_guard_count"] = (
                self._actual_q_guard_count[joint]
            )
            result[f"{prefix}/actual_q_guard_fraction"] = (
                self._actual_q_guard_count[joint].float() / denom
            )
            result[f"{prefix}/actual_q_guard_upper_count"] = (
                self._actual_q_guard_upper_count[joint]
            )
            result[f"{prefix}/actual_q_guard_lower_count"] = (
                self._actual_q_guard_lower_count[joint]
            )
            result[f"{prefix}/safe_interval_width_min_rad"] = (
                self._safe_interval_width_min[joint]
            )
            result[f"{prefix}/final_interval_width_min_rad"] = (
                self._final_interval_width_min[joint]
            )
            for category, category_name in enumerate(
                ("safe", "rate", "tracking", "torque")
            ):
                result[f"{prefix}/tightest_{category_name}_fraction"] = (
                    self._tightest_constraint_count[joint, category].float()
                    / denom
                )
            for level in range(4):
                result[f"{prefix}/fallback_level_{level}_fraction"] = (
                    self._fallback_level_count[joint, level].float() / denom
                )
        return result

    def resolved_contract_metadata(self) -> dict[str, list[float] | str | float]:
        """Resolved Isaac-order arrays for ONNX metadata and the C++ projector."""
        row = lambda value: value[0].detach().cpu().tolist()
        metadata = {
            "qdes_action_contract": self._action_contract,
            "qdes_actual_q_guard_contract": self._actual_q_guard_contract,
            "qdes_actual_q_guard_horizon_s": self._actual_q_guard_horizon_s,
            "qdes_actual_q_hard_tolerance_rad": (
                self._actual_q_hard_tolerance_rad
            ),
            "qdes_action_joint_names": list(self._joint_names),
            "qdes_safe_lo": row(self._safe_lo),
            "qdes_safe_hi": row(self._safe_hi),
            "qdes_rate_limit_rad_s": row(self._qdes_rate_limit),
            "qdes_tracking_error_limit_rad": row(self._tracking_error_limit),
            "qdes_torque_headroom_fraction": self._torque_headroom_fraction,
            "qdes_projector_dt_s": self._step_dt,
            "qdes_projector_kp": row(self._projector_kp),
            "qdes_projector_kd": row(self._projector_kd),
            "qdes_projector_effort_limit": row(self._effort_limit),
            "qdes_feedback_scale": row(self._feedback_scale),
        }
        if self._action_contract == BOUNDED_QDES_ACTION_CONTRACT:
            metadata["qdes_tanh_bias"] = row(self._tanh_bias)
            metadata["qdes_tanh_input_gain"] = row(self._tanh_input_gain)
        if self._action_contract == ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT:
            # v5 decode is fully determined by (default_q, action scale, safe_lo/hi); export the
            # two arrays not already present in the qdes block so C++/MuJoCo reconstruct it without
            # reaching into the legacy affine metadata.
            metadata["qdes_default_q"] = row(self._default_q)
            metadata["qdes_action_scale"] = row(self._decode_scale)
        return metadata


@configclass
class BoundedJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = BoundedJointPositionAction

    action_contract: str = FEASIBLE_QDES_ACTION_CONTRACT
    actual_q_guard_contract: str = ""
    actual_q_guard_horizon_s: float = 0.0
    actual_q_hard_tolerance_rad: float = 0.0
    passive_joint_names: tuple[str, ...] = ()
    # Empty/inert structural defaults are intentional: the selected task YAML supplies every
    # numerical joint map and the headroom value before the environment is instantiated.
    safe_margin_fraction: dict[str, float] = {}
    rate_limit_rad_s: dict[str, float] = {}
    tracking_error_limit_rad: dict[str, float] = {}
    torque_headroom_fraction: float = 0.0
    feedback_scale_floor: float = 1.0e-4
    upper_intervention_enabled: bool = False
    upper_intervention_command_name: str = "racket_target"
    upper_intervention_group_fraction: float = 0.0
    upper_intervention_switch_prob: float = 0.0
    upper_intervention_resample_steps: int = 1
    upper_intervention_action_range: tuple[float, float] = (0.0, 0.0)
    upper_intervention_radius_rad: float = 0.0
    upper_intervention_joint_names: tuple[str, ...] = ()
    qdes_debug_joint_names: tuple[str, ...] = ()


def actual_q_hard_limit_audit(
    env, action_name: str = "joint_pos"
) -> torch.Tensor:
    """Terminate only environments whose post-physics hard-limit excess exceeds tolerance."""

    action_term = env.action_manager.get_term(action_name)
    audit = getattr(action_term, "audit_actual_q_post_physics", None)
    if not callable(audit):
        raise RuntimeError(
            f"Action term {action_name!r} lacks the actual-q hard-limit audit"
        )
    terminated = audit()
    if (
        not torch.is_tensor(terminated)
        or terminated.shape != (env.num_envs,)
        or terminated.dtype != torch.bool
    ):
        raise RuntimeError(
            "actual-q hard-limit audit must return one boolean per environment"
        )
    return terminated


def actual_q_hard_limit_telemetry(
    env, action_name: str = "joint_pos"
) -> torch.Tensor:
    """Sample terminal actual-q before reset without making the excursion a PPO termination.

    This is intentionally registered as a ``DoneTerm``: Isaac Lab evaluates the termination
    manager after the final physics substep and before resetting any environment.  The physical
    fault mask is accumulated and snapshotted by :func:`actual_q_hard_limit_audit`, then replaced
    with an all-false return so actual-q remains telemetry-only for HitterPingPong.
    """

    action_term = env.action_manager.get_term(action_name)
    mode = str(
        getattr(action_term, "_actual_q_hard_audit_mode", "")
    ).strip().lower()
    if mode != "telemetry":
        raise RuntimeError(
            "non-terminating actual-q telemetry requires "
            f"actual_q_hard_audit_mode='telemetry', got {mode!r}"
        )
    observed_fault = actual_q_hard_limit_audit(env, action_name=action_name)
    return torch.zeros_like(observed_fault)


def phase_gated_action_rate_l2(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    strike_joint_names: tuple[str, ...] = (),
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """Unitree raw-action rate with a task-specific racket-arm strike release."""

    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("action-rate strike/follow-through windows must be non-negative")
    action_term = env.action_manager.get_term(action_name)
    value = env.action_manager.action - env.action_manager.prev_action
    if int(value.shape[-1]) != int(action_term.action_dim):
        raise RuntimeError(
            "phase-gated action rate requires one action term matching manager action order"
        )
    live = torch.ones_like(value, dtype=torch.bool)
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        live.index_fill_(-1, passive_cols, False)
    command = env.command_manager.get_term(command_name)
    strike_cols = resolve_action_columns_by_joint_name(
        action_term, strike_joint_names
    )
    if strike_cols:
        cols = torch.tensor(strike_cols, dtype=torch.long, device=value.device)
        protected = (
            (command.time_to_strike <= float(strike_free_pre_s))
            & (command.time_to_strike >= -float(follow_through_free_s))
        )
        rows = torch.where(protected)[0]
        if rows.numel() > 0:
            live[rows[:, None], cols[None, :]] = False
    rms = torch.sqrt(torch.mean(torch.square(value), dim=-1))
    command.metrics["raw_action_rate_rms"] = rms
    try:
        motion = env.command_manager.get_term("motion")
    except (KeyError, ValueError):
        motion = None
    if motion is not None and hasattr(motion, "in_hold"):
        key = "raw_action_rate_rms_hold"
        previous = command.metrics.get(key, torch.zeros_like(rms))
        command.metrics[key] = torch.where(motion.in_hold, rms, previous)
    return torch.sum(torch.square(value) * live.float(), dim=-1)


def executed_qdes_difference_l2(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    order: int = 1,
    upper_joint_names: tuple[str, ...] = (),
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """Smooth the published q_des while preserving the racket strike and follow-through.

    Lower-body and waist channels are always charged.  Selected upper-body channels are free from
    ``strike_free_pre_s`` before impact through ``follow_through_free_s`` after impact, then become
    quiet under the same policy again. Values are normalized into the action coordinate exposed by
    the selected q_des contract; for V17 this is exactly its affine action scale. Both histories
    originate from the final executable q_des in joint radians. Intervention-owned shoulder/elbow
    channels are masked because the actor cannot control them.
    """
    if order not in (1, 2):
        raise ValueError(f"executed q_des difference order must be 1 or 2, got {order}")
    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("executed q_des strike/follow-through windows must be non-negative")
    action_term = env.action_manager.get_term(action_name)
    attr = (
        "executed_qdes_delta_normalized"
        if order == 1
        else "executed_qdes_second_difference_normalized"
    )
    value = getattr(action_term, attr, None)
    if value is None:
        raise RuntimeError(
            f"Action term {action_name!r} does not expose {attr}"
        )
    live = torch.ones_like(value, dtype=torch.bool)
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        live.index_fill_(-1, passive_cols, False)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(action_term, "_upper_intervention_effective", None)
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        rows = torch.where(intervention_effective)[0]
        live[rows[:, None], intervention_cols[None, :]] = False

    command = env.command_manager.get_term(command_name)
    upper_cols = resolve_action_columns_by_joint_name(action_term, upper_joint_names)
    if upper_cols:
        upper = torch.tensor(upper_cols, dtype=torch.long, device=value.device)
        protected = (
            (command.time_to_strike <= float(strike_free_pre_s))
            & (command.time_to_strike >= -float(follow_through_free_s))
        )
        rows = torch.where(protected)[0]
        if rows.numel() > 0:
            live[rows[:, None], upper[None, :]] = False

    # Observability must not depend on whether the upper-body exemption list is configured:
    # any task paying this penalty gets the matching rad-RMS diagnostic.
    rad_value = (
        action_term.executed_qdes_delta
        if order == 1
        else action_term.executed_qdes_second_difference
    )
    rms = torch.sqrt(torch.mean(torch.square(rad_value), dim=-1))
    command.metrics[
        "executed_qdes_step_rms_rad"
        if order == 1
        else "executed_qdes_second_difference_rms_rad"
    ] = rms
    # HOLD-MASKED chatter observability (2026-07-24 audit): V14 died on hardware from motor
    # chatter in long policy-only holds, but the all-frame RMS above is dominated by legitimate
    # swing motion — hold quietness was unobservable.  Hold the metric at its last in-hold value
    # so eval/W&B can trend a pure quiet-wait command-noise number (no reward change here).
    motion_term = getattr(env.command_manager, "get_term", None)
    if motion_term is not None:
        try:
            _motion = env.command_manager.get_term("motion")
        except (KeyError, ValueError):
            _motion = None
        if _motion is not None and hasattr(_motion, "in_hold"):
            key = (
                "executed_qdes_step_rms_rad_hold"
                if order == 1
                else "executed_qdes_second_difference_rms_rad_hold"
            )
            previous = command.metrics.get(key)
            if previous is None:
                previous = torch.zeros_like(rms)
            command.metrics[key] = torch.where(_motion.in_hold, rms, previous)

    return torch.sum(torch.square(value) * live.float(), dim=-1)


def phase_gated_joint_acc_l2(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    strike_joint_names: tuple[str, ...] = (),
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """Unitree-style actual joint-acceleration cost with a racket-arm strike release.

    The lower body, waist and non-racket arm remain charged throughout the swing. Only explicitly
    listed racket-arm joints are released from ``strike_free_pre_s`` before contact through the
    follow-through window, so the regularizer quiets READY/recovery without removing racket speed.
    """

    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("joint-acc strike/follow-through windows must be non-negative")
    action_term = env.action_manager.get_term(action_name)
    select = getattr(action_term, "_select_action_joints", None)
    if not callable(select):
        raise RuntimeError(
            f"Action term {action_name!r} cannot map actual joint acceleration to action order"
        )
    asset = getattr(action_term, "_asset", None)
    if asset is None:
        raise RuntimeError(f"Action term {action_name!r} does not expose its articulation")
    value = select(asset.data.joint_acc)
    live = torch.ones_like(value, dtype=torch.bool)
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        live.index_fill_(-1, passive_cols, False)

    command = env.command_manager.get_term(command_name)
    strike_cols = resolve_action_columns_by_joint_name(
        action_term, strike_joint_names
    )
    if strike_cols:
        cols = torch.tensor(strike_cols, dtype=torch.long, device=value.device)
        protected = (
            (command.time_to_strike <= float(strike_free_pre_s))
            & (command.time_to_strike >= -float(follow_through_free_s))
        )
        rows = torch.where(protected)[0]
        if rows.numel() > 0:
            live[rows[:, None], cols[None, :]] = False

    live_count = live.sum(dim=-1).clamp_min(1)
    rms = torch.sqrt(
        (torch.square(value) * live.float()).sum(dim=-1) / live_count
    )
    command.metrics["actual_joint_acc_rms_rad_s2"] = rms
    try:
        motion = env.command_manager.get_term("motion")
    except (KeyError, ValueError):
        motion = None
    if motion is not None and hasattr(motion, "in_hold"):
        key = "actual_joint_acc_rms_rad_s2_hold"
        previous = command.metrics.get(key, torch.zeros_like(rms))
        command.metrics[key] = torch.where(motion.in_hold, rms, previous)
    return torch.sum(torch.square(value) * live.float(), dim=-1)


def qdes_feasible_rail_debt(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    free_fraction: float = 0.95,
    topk: int = 4,
    topk_blend: float = 0.75,
) -> torch.Tensor:
    """Discourage a V3 policy from hiding entropy on saturated feasible-action rails.

    The executable interval is part of the action parameterization, so ordinary action magnitude
    is not projection debt and remains free through ``free_fraction``.  Only the tanh-normalized
    tail is charged.  Interval width, realized rate use and each limiting constraint are logged
    separately so a narrow but valid interval cannot masquerade as an unconstrained policy.
    """
    if not 0.0 <= free_fraction < 1.0:
        raise ValueError(f"invalid feasible q_des free_fraction={free_fraction}")
    if topk <= 0 or not 0.0 <= topk_blend <= 1.0:
        raise ValueError(f"invalid feasible q_des topk/blend: {topk}/{topk_blend}")
    action_term = env.action_manager.get_term(action_name)
    if getattr(action_term, "_action_contract", None) != FEASIBLE_QDES_ACTION_CONTRACT:
        raise RuntimeError(
            "qdes_feasible_rail_debt requires action_contract="
            f"{FEASIBLE_QDES_ACTION_CONTRACT!r}"
        )
    utilization = action_term.feasible_action_utilization
    executable = torch.ones(
        utilization.shape[-1], dtype=torch.bool, device=utilization.device
    )
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        executable[passive_cols] = False
    utilization_live = utilization[:, executable]
    excess = torch.clamp(
        (utilization_live - float(free_fraction))
        / max(1.0 - float(free_fraction), 1.0e-6),
        min=0.0,
    )
    per_joint = 0.5 * torch.square(excess)
    k = min(int(topk), int(per_joint.shape[-1]))
    debt = (
        (1.0 - float(topk_blend)) * per_joint.mean(dim=-1)
        + float(topk_blend) * torch.topk(per_joint, k=k, dim=-1).values.mean(dim=-1)
    )

    cmd = env.command_manager.get_term(command_name)
    interval_width = action_term.feasible_interval_width_fraction
    rate_utilization = action_term.qdes_rate_utilization
    interval_width_live = interval_width[:, executable]
    rate_utilization_live = rate_utilization[:, executable]
    cmd.metrics["qdes_feasible_action_utilization_mean"] = utilization_live.mean(dim=-1)
    cmd.metrics["qdes_feasible_action_utilization_max"] = utilization_live.max(dim=-1).values
    cmd.metrics["qdes_feasible_interval_width_mean"] = interval_width_live.mean(dim=-1)
    cmd.metrics["qdes_feasible_interval_width_min"] = interval_width_live.min(dim=-1).values
    cmd.metrics["qdes_feasible_rate_utilization_mean"] = rate_utilization_live.mean(dim=-1)
    cmd.metrics["qdes_feasible_rate_utilization_max"] = rate_utilization_live.max(dim=-1).values
    cmd.metrics["qdes_feasible_rate_bound_frac"] = (
        action_term.feasible_rate_bound_active[:, executable].float().mean(dim=-1)
    )
    cmd.metrics["qdes_feasible_tracking_bound_frac"] = (
        action_term.feasible_tracking_bound_active[:, executable].float().mean(dim=-1)
    )
    cmd.metrics["qdes_feasible_torque_bound_frac"] = (
        action_term.feasible_torque_bound_active[:, executable].float().mean(dim=-1)
    )
    cmd.metrics["qdes_feasible_infeasible_frac"] = (
        action_term.projection_infeasible[:, executable].float().mean(dim=-1)
    )

    exact_strike = torch.abs(cmd.time_to_strike) <= (0.5 * float(env.step_dt) + 1.0e-6)
    for joint_name, col in action_term.qdes_debug_joints:
        label = joint_name.removesuffix("_joint")
        for suffix, values in (
            ("action_utilization", utilization),
            ("interval_width", interval_width),
            ("rate_utilization", rate_utilization),
        ):
            key = f"qdes_feasible_{suffix}_exact_strike_{label}"
            previous = cmd.metrics.get(key)
            if previous is None:
                previous = torch.zeros_like(cmd.time_to_strike)
            cmd.metrics[key] = torch.where(exact_strike, values[:, col], previous)
    return debt


def qdes_saturation_debt(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    free_fraction: float = 0.90,
    topk: int = 4,
    topk_blend: float = 0.75,
    upper_joint_names: tuple[str, ...] = (),
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """Charge deep v5 decode saturation (rail-pinned postures) that projection debt cannot see.

    The v5 nominal target is inside the safe range BY CONSTRUCTION, so a policy that parks a
    joint against the rail incurs zero projection debt (2026-07-23 audit finding).  This term
    charges the |tanh| tail of the per-side decode above ``free_fraction``; legitimate large
    swing excursions stay free both because they sit well inside the room (utilization < 0.6 for
    a full 1.4 rad abduction) and because the configured racket-arm joints are exempted through
    the strike/follow-through window, mirroring ``executed_qdes_difference_l2``.
    """
    if not 0.0 <= free_fraction < 1.0:
        raise ValueError(f"invalid q_des saturation free_fraction={free_fraction}")
    if topk <= 0 or not 0.0 <= topk_blend <= 1.0:
        raise ValueError(f"invalid q_des saturation topk/blend: {topk}/{topk_blend}")
    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("q_des saturation strike/follow-through windows must be non-negative")
    action_term = env.action_manager.get_term(action_name)
    if getattr(action_term, "_action_contract", None) != ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT:
        raise RuntimeError(
            "qdes_saturation_debt requires action_contract="
            f"{ABSOLUTE_FEASIBLE_QDES_ACTION_CONTRACT!r}"
        )
    utilization = action_term.decode_utilization
    live = torch.ones_like(utilization, dtype=torch.bool)
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        live.index_fill_(-1, passive_cols, False)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(action_term, "_upper_intervention_effective", None)
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        rows = torch.where(intervention_effective)[0]
        live[rows[:, None], intervention_cols[None, :]] = False

    cmd = env.command_manager.get_term(command_name)
    upper_cols = resolve_action_columns_by_joint_name(action_term, upper_joint_names)
    if upper_cols:
        upper = torch.tensor(upper_cols, dtype=torch.long, device=utilization.device)
        protected = (
            (cmd.time_to_strike <= float(strike_free_pre_s))
            & (cmd.time_to_strike >= -float(follow_through_free_s))
        )
        rows = torch.where(protected)[0]
        if rows.numel() > 0:
            live[rows[:, None], upper[None, :]] = False

    excess = torch.clamp(
        (utilization - float(free_fraction)) / max(1.0 - float(free_fraction), 1.0e-6),
        min=0.0,
    )
    per_joint = 0.5 * torch.square(excess) * live.float()
    k = min(int(topk), int(per_joint.shape[-1]))
    debt = (
        (1.0 - float(topk_blend)) * per_joint.mean(dim=-1)
        + float(topk_blend) * torch.topk(per_joint, k=k, dim=-1).values.mean(dim=-1)
    )
    cmd.metrics["qdes_decode_utilization_mean"] = utilization.mean(dim=-1)
    cmd.metrics["qdes_decode_utilization_max"] = utilization.max(dim=-1).values
    cmd.metrics["qdes_decode_saturated_frac"] = (
        (utilization > float(free_fraction)).float().mean(dim=-1)
    )
    return debt


def qdes_projection_debt(
    env,
    command_name: str,
    action_name: str = "joint_pos",
    topk: int = 4,
    topk_blend: float = 0.75,
    upper_joint_names: tuple[str, ...] = (),
    strike_free_pre_s: float = 0.12,
    follow_through_free_s: float = 0.30,
) -> torch.Tensor:
    """One unified V15 debt for every intervention made by the q_des projector.

    The configured racket-arm joints are exempted through the strike/follow-through window,
    mirroring ``qdes_saturation_debt`` / ``executed_qdes_difference_l2`` (2026-07-24
    momentum-risk audit): this was the ONLY debt of the three still charging the arm inside the
    window, so a swing riding the rate envelope — where the huber/top-4 shape concentrates on
    exactly the swinging joints — paid a quadratic in-window tax the projector already bounds
    physically.  The projector itself is identical in deploy, so an in-window over-request is
    executed identically everywhere; only the executed FK velocity earns racket reward.
    Passive and intervention-owned channels are masked like the sibling debts.
    """
    if topk <= 0 or not 0.0 <= topk_blend <= 1.0:
        raise ValueError(f"invalid q_des projection topk/blend: {topk}/{topk_blend}")
    if strike_free_pre_s < 0.0 or follow_through_free_s < 0.0:
        raise ValueError("q_des projection strike/follow-through windows must be non-negative")
    action_term = env.action_manager.get_term(action_name)
    error = getattr(action_term, "projection_error_normalized", None)
    if error is None:
        raise RuntimeError(
            f"Action term {action_name!r} does not expose V15 projection diagnostics"
        )
    live = torch.ones_like(error, dtype=torch.bool)
    passive_cols = getattr(action_term, "_passive_action_cols", None)
    if torch.is_tensor(passive_cols) and passive_cols.numel() > 0:
        live.index_fill_(-1, passive_cols, False)
    intervention_cols = getattr(action_term, "_upper_intervention_cols", None)
    intervention_effective = getattr(action_term, "_upper_intervention_effective", None)
    if (
        torch.is_tensor(intervention_cols)
        and intervention_cols.numel() > 0
        and torch.is_tensor(intervention_effective)
        and bool(intervention_effective.any())
    ):
        rows = torch.where(intervention_effective)[0]
        live[rows[:, None], intervention_cols[None, :]] = False

    cmd = env.command_manager.get_term(command_name)
    upper_cols = resolve_action_columns_by_joint_name(action_term, upper_joint_names)
    if upper_cols:
        upper = torch.tensor(upper_cols, dtype=torch.long, device=error.device)
        protected = (
            (cmd.time_to_strike <= float(strike_free_pre_s))
            & (cmd.time_to_strike >= -float(follow_through_free_s))
        )
        rows = torch.where(protected)[0]
        if rows.numel() > 0:
            live[rows[:, None], upper[None, :]] = False

    per_joint = torch.where(error <= 1.0, 0.5 * torch.square(error), error - 0.5)
    per_joint = per_joint * live.float()
    k = min(int(topk), int(per_joint.shape[-1]))
    mean_debt = per_joint.mean(dim=-1)
    topk_debt = torch.topk(per_joint, k=k, dim=-1).values.mean(dim=-1)
    debt = (1.0 - float(topk_blend)) * mean_debt + float(topk_blend) * topk_debt

    cmd.metrics["qdes_projection_joint_frac"] = action_term.projection_active.float().mean(dim=-1)
    cmd.metrics["qdes_projection_error_max"] = error.max(dim=-1).values
    cmd.metrics["qdes_projection_rate_frac"] = action_term.rate_projection_active.float().mean(dim=-1)
    cmd.metrics["qdes_projection_tracking_frac"] = (
        action_term.tracking_projection_active.float().mean(dim=-1)
    )
    cmd.metrics["qdes_projection_torque_frac"] = (
        action_term.torque_projection_active.float().mean(dim=-1)
    )
    cmd.metrics["qdes_projection_infeasible_frac"] = (
        action_term.projection_infeasible.float().mean(dim=-1)
    )
    # Aggregate fractions cannot reveal whether the deploy projector is specifically suppressing
    # the shoulder/elbow/wrist chain at contact.  Hold YAML-selected per-joint diagnostics on the
    # nearest exact-strike frame; they are logging-only and never enter policy observations.
    exact_strike = torch.abs(cmd.time_to_strike) <= (0.5 * float(env.step_dt) + 1.0e-6)
    for joint_name, col in action_term.qdes_debug_joints:
        label = joint_name.removesuffix("_joint")
        for suffix, values in (
            ("active", action_term.projection_active),
            ("rate", action_term.rate_projection_active),
            ("tracking", action_term.tracking_projection_active),
            ("torque", action_term.torque_projection_active),
            ("infeasible", action_term.projection_infeasible),
        ):
            key = f"qdes_projection_{suffix}_exact_strike_{label}"
            previous = cmd.metrics.get(key)
            if previous is None:
                previous = torch.zeros_like(cmd.time_to_strike)
            cmd.metrics[key] = torch.where(
                exact_strike, values[:, col].float(), previous
            )
        error_key = f"qdes_projection_error_exact_strike_{label}"
        previous_error = cmd.metrics.get(error_key)
        if previous_error is None:
            previous_error = torch.zeros_like(cmd.time_to_strike)
        cmd.metrics[error_key] = torch.where(
            exact_strike, error[:, col], previous_error
        )
    return debt
