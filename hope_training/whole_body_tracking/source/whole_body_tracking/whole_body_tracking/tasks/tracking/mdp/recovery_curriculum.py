"""Pure state machine for the RallyV17 monotonic recovery curriculum.

This module intentionally has no Isaac Lab or torch dependency.  The live command term uses the
same kernel as the host-only tests, so the reset-distribution schedule can be audited without
starting a simulator.

The curriculum has three one-way stages:

* stage 0: exact RallyV11/V16 task distribution (recovery scale 0);
* stage 1: acquire strict READY first, then expose half of the configured coverage;
* stage 2: the full configured replay/recovery difficulty.

Stage 1 deliberately owns two independent scales.  ``current_scale`` controls sampled READY
release and the bounded safe-set supervision.  During acquisition it can move through a configured
ladder of small supervision scales instead of jumping directly from zero to 0.5.
``coverage_scale`` controls replay, venue tuples, post-wrap hold extension and target-stream
robustness.  Coverage remains exactly zero until both sides have demonstrated the final strict
READY criterion at the last acquisition rung.

During READY acquisition, strike competence is guarded by *release-eligible* completion: READY
timeouts are removed from the swing-start denominator, while falls and every other aborted swing
remain failures.  Raw end-to-end completion is retained as telemetry.  Once exposure has been
earned it is never removed; failures are handled by the side/phase/severity adaptive replay
sampler instead of changing the task distribution back and forth.

Transitions are metric-gated and rate-limited, so crossing a gate cannot abruptly replace a large
fraction of the on-policy state distribution.  ``current_scale`` and ``coverage_scale`` are
monotonic invariants for an enabled run.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RecoveryCurriculumConfig:
    """Thresholds and rates for the monotonic recovery curriculum."""

    stage1_scale: float = 0.5
    minimum_environment_steps: int = 24_000
    # Every minimum and rate is applied independently to FH and BH.  No across-side average is
    # ever allowed to advance the state machine.
    stage1_minimum_exact_samples_per_side: float = 200.0
    stage1_minimum_swing_starts_per_side: float = 400.0
    stage2_minimum_exact_samples_per_side: float = 500.0
    stage2_minimum_swing_starts_per_side: float = 1_000.0
    stage2_minimum_virtual_samples_per_side: float = 200.0
    stage2_minimum_actual_q_window_starts_per_side: float = 1_000.0
    actual_q_window_steps: int = 500

    stage1_enter_completion: float = 0.60
    stage1_enter_position: float = 0.25
    stage1_enter_velocity: float = 0.20
    stage1_enter_normal: float = 0.35
    stage1_enter_composite: float = 0.10
    stage1_enter_ready: float = 0.50
    stage1_enter_post_fall: float = 0.10
    stage1_enter_actual_q_fault: float = 0.005
    stage1_enter_dwell_steps: int = 250

    stage1_exit_completion: float = 0.50
    stage1_exit_position: float = 0.15
    stage1_exit_velocity: float = 0.12
    stage1_exit_normal: float = 0.25
    stage1_exit_composite: float = 0.05
    stage1_exit_ready: float = 0.40
    stage1_exit_post_fall: float = 0.15
    stage1_exit_actual_q_fault: float = 0.010
    stage1_exit_dwell_steps: int = 150
    # READY is an acquisition target inside Stage 1, not a Stage-0 admission requirement.
    # Once readiness supervision reaches its 0.5 target, both sides must hold the strict READY
    # threshold for this dwell before venue/replay coverage can begin.
    stage1_ready_dwell_steps: int = 250
    # Optional Stage-1 acquisition ladder.  Empty tuples retain the legacy single-rung behavior:
    # one ramp to ``stage1_scale`` using ``stage0_to_1_ramp_steps`` and the final Stage-1 READY
    # threshold.  A configured ladder must end exactly at those same final values.
    stage1_acquisition_scales: tuple[float, ...] = ()
    stage1_acquisition_ready_thresholds: tuple[float, ...] = ()
    stage1_acquisition_ramp_steps: int = 2_000
    # Legacy r8 serialization field. R9 intentionally never uses a timeout to reduce exposure.
    stage1_acquisition_timeout_steps: int = 500

    stage2_enter_completion: float = 0.75
    stage2_enter_position: float = 0.70
    stage2_enter_velocity: float = 0.75
    stage2_enter_normal: float = 0.75
    stage2_enter_composite: float = 0.60
    stage2_enter_ready: float = 0.80
    stage2_enter_safe_recovery: float = 0.85
    stage2_enter_virtual_contact: float = 0.85
    stage2_enter_virtual_over_net: float = 0.75
    stage2_enter_virtual_legal: float = 0.65
    stage2_enter_post_fall: float = 0.03
    stage2_enter_actual_q_fault: float = 0.0
    stage2_enter_dwell_steps: int = 500

    stage2_exit_completion: float = 0.65
    stage2_exit_position: float = 0.55
    stage2_exit_velocity: float = 0.60
    stage2_exit_normal: float = 0.60
    stage2_exit_composite: float = 0.45
    stage2_exit_ready: float = 0.70
    stage2_exit_safe_recovery: float = 0.75
    stage2_exit_virtual_contact: float = 0.75
    stage2_exit_virtual_over_net: float = 0.65
    stage2_exit_virtual_legal: float = 0.50
    stage2_exit_post_fall: float = 0.05
    stage2_exit_actual_q_fault: float = 0.005
    stage2_exit_dwell_steps: int = 150

    stage0_to_1_ramp_steps: int = 8_000
    stage1_coverage_ramp_steps: int = 8_000
    stage1_to_0_ramp_steps: int = 4_000
    stage1_to_2_ramp_steps: int = 12_000
    stage2_to_1_ramp_steps: int = 6_000

    def acquisition_scales(self) -> tuple[float, ...]:
        """Return the normalized Stage-1 supervision ladder."""

        if self.stage1_acquisition_scales:
            return tuple(float(value) for value in self.stage1_acquisition_scales)
        return (float(self.stage1_scale),)

    def acquisition_ready_thresholds(self) -> tuple[float, ...]:
        """Return the per-rung, per-side READY success thresholds."""

        if self.stage1_acquisition_ready_thresholds:
            return tuple(
                float(value)
                for value in self.stage1_acquisition_ready_thresholds
            )
        return (float(self.stage1_enter_ready),)

    def validate(self) -> None:
        """Fail closed on thresholds that destroy hysteresis or boundedness."""

        unit_values = {
            name: float(value)
            for name, value in vars(self).items()
            if (
                isinstance(value, (int, float))
                and "minimum" not in name
                and not name.endswith("_steps")
                and (
                    "completion" in name
                    or "position" in name
                    or "velocity" in name
                    or "normal" in name
                    or "composite" in name
                    or "ready" in name
                    or "safe_recovery" in name
                    or "virtual_" in name
                    or "post_fall" in name
                    or "actual_q_fault" in name
                    or name == "stage1_scale"
                )
            )
        }
        for name, value in unit_values.items():
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0, 1], got {value}")
        if not 0.0 < self.stage1_scale < 1.0:
            raise ValueError("stage1_scale must lie strictly between 0 and 1")

        nonnegative = {
            "minimum_environment_steps": self.minimum_environment_steps,
            "stage1_minimum_exact_samples_per_side": self.stage1_minimum_exact_samples_per_side,
            "stage1_minimum_swing_starts_per_side": self.stage1_minimum_swing_starts_per_side,
            "stage2_minimum_exact_samples_per_side": self.stage2_minimum_exact_samples_per_side,
            "stage2_minimum_swing_starts_per_side": self.stage2_minimum_swing_starts_per_side,
            "stage2_minimum_virtual_samples_per_side": self.stage2_minimum_virtual_samples_per_side,
            "stage2_minimum_actual_q_window_starts_per_side": (
                self.stage2_minimum_actual_q_window_starts_per_side
            ),
        }
        for name, value in nonnegative.items():
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative, got {value}")
        for name, value in vars(self).items():
            if name.endswith("_dwell_steps") or name.endswith("_ramp_steps"):
                if int(value) < 1:
                    raise ValueError(f"{name} must be >= 1, got {value}")
        if int(self.actual_q_window_steps) < 1:
            raise ValueError("actual_q_window_steps must be >= 1")
        if int(self.stage1_acquisition_timeout_steps) < 1:
            raise ValueError("stage1_acquisition_timeout_steps must be >= 1")
        acquisition_scales = self.acquisition_scales()
        acquisition_ready = self.acquisition_ready_thresholds()
        if len(acquisition_scales) != len(acquisition_ready):
            raise ValueError(
                "stage1 acquisition scales and READY thresholds must have equal length"
            )
        if not acquisition_scales:
            raise ValueError("stage1 acquisition ladder must not be empty")
        if any(
            not math.isfinite(value) or not 0.0 < value <= self.stage1_scale
            for value in acquisition_scales
        ):
            raise ValueError(
                "stage1 acquisition scales must be finite and lie in (0, stage1_scale]"
            )
        if any(
            right <= left
            for left, right in zip(
                acquisition_scales[:-1], acquisition_scales[1:]
            )
        ):
            raise ValueError("stage1 acquisition scales must be strictly increasing")
        if not math.isclose(
            acquisition_scales[-1],
            float(self.stage1_scale),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                "stage1 acquisition ladder must end at stage1_scale"
            )
        if any(
            not math.isfinite(value)
            or not 0.0 < value <= self.stage1_enter_ready
            for value in acquisition_ready
        ):
            raise ValueError(
                "stage1 acquisition READY thresholds must be finite and lie in "
                "(0, stage1_enter_ready]"
            )
        if any(
            right <= left
            for left, right in zip(
                acquisition_ready[:-1], acquisition_ready[1:]
            )
        ):
            raise ValueError(
                "stage1 acquisition READY thresholds must be strictly increasing"
            )
        if not math.isclose(
            acquisition_ready[-1],
            float(self.stage1_enter_ready),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                "stage1 acquisition READY ladder must end at stage1_enter_ready"
            )

        # Exit thresholds deliberately form a wider safe set than their matching enter set.
        for prefix in (
            "completion",
            "position",
            "velocity",
            "normal",
            "composite",
            "ready",
        ):
            if not getattr(self, f"stage1_exit_{prefix}") < getattr(
                self, f"stage1_enter_{prefix}"
            ):
                raise ValueError(f"stage1 {prefix} thresholds do not provide hysteresis")
            if not getattr(self, f"stage2_exit_{prefix}") < getattr(
                self, f"stage2_enter_{prefix}"
            ):
                raise ValueError(f"stage2 {prefix} thresholds do not provide hysteresis")
        for prefix in (
            "safe_recovery",
            "virtual_contact",
            "virtual_over_net",
            "virtual_legal",
        ):
            if not getattr(self, f"stage2_exit_{prefix}") < getattr(
                self, f"stage2_enter_{prefix}"
            ):
                raise ValueError(f"stage2 {prefix} thresholds do not provide hysteresis")
        if not self.stage1_exit_post_fall > self.stage1_enter_post_fall:
            raise ValueError("stage1 post-fall thresholds do not provide hysteresis")
        if not self.stage2_exit_post_fall > self.stage2_enter_post_fall:
            raise ValueError("stage2 post-fall thresholds do not provide hysteresis")
        if not self.stage1_exit_actual_q_fault > self.stage1_enter_actual_q_fault:
            raise ValueError("stage1 actual-q thresholds do not provide hysteresis")
        if not self.stage2_exit_actual_q_fault > self.stage2_enter_actual_q_fault:
            raise ValueError("stage2 actual-q thresholds do not provide hysteresis")

        # Stage 2 must be a strict refinement of Stage 1, not an easier parallel gate.
        for prefix in (
            "completion",
            "position",
            "velocity",
            "normal",
            "composite",
            "ready",
        ):
            if not getattr(self, f"stage2_enter_{prefix}") > getattr(
                self, f"stage1_enter_{prefix}"
            ):
                raise ValueError(f"stage2 {prefix} entry must be stricter than stage1")
        if not self.stage2_enter_post_fall < self.stage1_enter_post_fall:
            raise ValueError("stage2 post-fall entry must be stricter than stage1")
        if not self.stage2_enter_actual_q_fault < self.stage1_enter_actual_q_fault:
            raise ValueError("stage2 actual-q entry must be stricter than stage1")
        if not (
            self.stage2_minimum_exact_samples_per_side
            > self.stage1_minimum_exact_samples_per_side
            and self.stage2_minimum_swing_starts_per_side
            > self.stage1_minimum_swing_starts_per_side
        ):
            raise ValueError("stage2 sample minima must be stricter than stage1")


@dataclass(frozen=True)
class RecoveryCurriculumMetrics:
    """Bounded performance summary consumed by the curriculum."""

    environment_steps: int
    # Raw completion is end-to-end exact-strike arrivals / all swing starts.  The
    # release-eligible sibling removes only sampled READY questions that explicitly timed out.
    completion_fh: float
    completion_bh: float
    release_eligible_completion_fh: float
    release_eligible_completion_bh: float
    position_fh: float
    position_bh: float
    velocity_fh: float
    velocity_bh: float
    normal_fh: float
    normal_bh: float
    composite_fh: float
    composite_bh: float
    ready_fh: float
    ready_bh: float
    safe_recovery_fh: float
    safe_recovery_bh: float
    virtual_contact_fh: float
    virtual_contact_bh: float
    virtual_over_net_fh: float
    virtual_over_net_bh: float
    virtual_legal_fh: float
    virtual_legal_bh: float
    post_fall_fh: float
    post_fall_bh: float
    actual_q_fault_fh: float
    actual_q_fault_bh: float
    exact_samples_fh: float
    exact_samples_bh: float
    swing_starts_fh: float
    swing_starts_bh: float
    virtual_samples_fh: float
    virtual_samples_bh: float
    actual_q_fault_events_fh: float
    actual_q_fault_events_bh: float
    actual_q_window_steps: int
    actual_q_window_starts_fh: float
    actual_q_window_starts_bh: float


@dataclass(frozen=True)
class RecoveryCurriculumState:
    """Checkpointable state of the curriculum."""

    stage: int = 0
    current_scale: float = 0.0
    target_scale: float = 0.0
    ramp_rate_per_step: float = 0.0
    coverage_scale: float = 0.0
    coverage_target_scale: float = 0.0
    coverage_ramp_rate_per_step: float = 0.0
    stage1_coverage_unlocked: bool = False
    stage1_acquisition_rung: int = 0
    stage1_acquisition_failures: int = 0
    stage1_ready_dwell: int = 0
    stage1_acquisition_age: int = 0
    stage1_enter_dwell: int = 0
    stage1_exit_dwell: int = 0
    stage2_enter_dwell: int = 0
    stage2_exit_dwell: int = 0


@dataclass(frozen=True)
class RecoveryCurriculumConditions:
    """Auditable gate results for metrics/W&B."""

    enough_steps: bool
    enough_samples: bool
    enough_stage1_samples: bool
    enough_stage2_samples: bool
    actual_q_window_ready: bool
    stage1_enter_ok: bool
    stage1_ready_ok: bool
    stage1_safety_exit_bad: bool
    stage1_exit_bad: bool
    stage2_enter_ok: bool
    stage2_exit_bad: bool
    block_reason: int
    acquisition_block_reason: int
    stage2_block_reason: int


def _bounded_rate(value: float, name: str) -> float:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite, got {value}")
    return min(max(float(value), 0.0), 1.0)


def release_eligible_completion_rate(
    exact_samples: float,
    swing_starts: float,
    ready_timeouts: float,
) -> tuple[float, float]:
    """Return completion after excluding only READY-blocked starts.

    All three counters must share the same EMA decay.  Numerical ordering at a reset boundary can
    still make ``ready_timeouts`` exceed the currently visible number of non-completions by a tiny
    amount, so the exclusion is bounded by ``swing_starts - exact_samples``.  The resulting
    denominator can never fall below the numerator and the returned rate therefore stays in
    ``[0, 1]`` without hiding a true completed strike.
    """

    values = {
        "exact_samples": exact_samples,
        "swing_starts": swing_starts,
        "ready_timeouts": ready_timeouts,
    }
    for name, value in values.items():
        if not math.isfinite(float(value)) or float(value) < 0.0:
            raise ValueError(f"{name} must be finite and non-negative, got {value}")
    exact = float(exact_samples)
    starts = max(float(swing_starts), exact)
    removable = min(float(ready_timeouts), max(starts - exact, 0.0))
    eligible_starts = max(starts - removable, exact, 1.0e-6)
    return min(max(exact / eligible_starts, 0.0), 1.0), eligible_starts


def _conditions(
    metrics: RecoveryCurriculumMetrics,
    cfg: RecoveryCurriculumConfig,
    *,
    acquisition_ready_threshold: float | None = None,
) -> RecoveryCurriculumConditions:
    rate_names = (
        "completion_fh",
        "completion_bh",
        "release_eligible_completion_fh",
        "release_eligible_completion_bh",
        "position_fh",
        "position_bh",
        "velocity_fh",
        "velocity_bh",
        "normal_fh",
        "normal_bh",
        "composite_fh",
        "composite_bh",
        "ready_fh",
        "ready_bh",
        "safe_recovery_fh",
        "safe_recovery_bh",
        "virtual_contact_fh",
        "virtual_contact_bh",
        "virtual_over_net_fh",
        "virtual_over_net_bh",
        "virtual_legal_fh",
        "virtual_legal_bh",
        "post_fall_fh",
        "post_fall_bh",
        "actual_q_fault_fh",
        "actual_q_fault_bh",
    )
    rates = {
        name: _bounded_rate(getattr(metrics, name), name) for name in rate_names
    }
    for side in ("fh", "bh"):
        completion = rates[f"completion_{side}"]
        eligible_completion = rates[f"release_eligible_completion_{side}"]
        if eligible_completion + 1.0e-9 < completion:
            raise ValueError(
                f"{side} release-eligible completion {eligible_completion} "
                f"cannot be below raw completion {completion}"
            )
        position = rates[f"position_{side}"]
        velocity = rates[f"velocity_{side}"]
        normal = rates[f"normal_{side}"]
        composite = rates[f"composite_{side}"]
        # A composite pass is the intersection of the three marginal pass events.  Violating this
        # invariant means the counters/denominators are inconsistent, so advancing recovery would
        # be unsafe even if every configured threshold happened to pass.
        marginal_min = min(position, velocity, normal)
        if composite > marginal_min + 1.0e-9:
            raise ValueError(
                f"{side} composite rate {composite} exceeds marginal minimum "
                f"{marginal_min}"
            )
        contact = rates[f"virtual_contact_{side}"]
        over_net = rates[f"virtual_over_net_{side}"]
        legal = rates[f"virtual_legal_{side}"]
        if over_net > contact + 1.0e-9 or legal > over_net + 1.0e-9:
            raise ValueError(
                f"{side} virtual outcome rates violate legal<=over_net<=contact: "
                f"{legal}, {over_net}, {contact}"
            )
    enough_steps = int(metrics.environment_steps) >= int(cfg.minimum_environment_steps)
    sample_values = {
        "exact_samples_fh": metrics.exact_samples_fh,
        "exact_samples_bh": metrics.exact_samples_bh,
        "swing_starts_fh": metrics.swing_starts_fh,
        "swing_starts_bh": metrics.swing_starts_bh,
        "virtual_samples_fh": metrics.virtual_samples_fh,
        "virtual_samples_bh": metrics.virtual_samples_bh,
        "actual_q_fault_events_fh": metrics.actual_q_fault_events_fh,
        "actual_q_fault_events_bh": metrics.actual_q_fault_events_bh,
        "actual_q_window_starts_fh": metrics.actual_q_window_starts_fh,
        "actual_q_window_starts_bh": metrics.actual_q_window_starts_bh,
    }
    for name, value in sample_values.items():
        if not math.isfinite(float(value)) or float(value) < 0.0:
            raise ValueError(f"{name} must be finite and non-negative, got {value}")
    enough_stage1_samples = (
        float(metrics.exact_samples_fh)
        >= cfg.stage1_minimum_exact_samples_per_side
        and float(metrics.exact_samples_bh)
        >= cfg.stage1_minimum_exact_samples_per_side
        and float(metrics.swing_starts_fh)
        >= cfg.stage1_minimum_swing_starts_per_side
        and float(metrics.swing_starts_bh)
        >= cfg.stage1_minimum_swing_starts_per_side
    )
    enough_stage2_samples = (
        float(metrics.exact_samples_fh)
        >= cfg.stage2_minimum_exact_samples_per_side
        and float(metrics.exact_samples_bh)
        >= cfg.stage2_minimum_exact_samples_per_side
        and float(metrics.swing_starts_fh)
        >= cfg.stage2_minimum_swing_starts_per_side
        and float(metrics.swing_starts_bh)
        >= cfg.stage2_minimum_swing_starts_per_side
        and float(metrics.virtual_samples_fh)
        >= cfg.stage2_minimum_virtual_samples_per_side
        and float(metrics.virtual_samples_bh)
        >= cfg.stage2_minimum_virtual_samples_per_side
    )
    if int(metrics.actual_q_window_steps) < 0:
        raise ValueError(
            "actual_q_window_steps must be non-negative, got "
            f"{metrics.actual_q_window_steps}"
        )
    actual_q_window_ready = (
        int(metrics.actual_q_window_steps) >= int(cfg.actual_q_window_steps)
        and float(metrics.actual_q_window_starts_fh)
        >= cfg.stage2_minimum_actual_q_window_starts_per_side
        and float(metrics.actual_q_window_starts_bh)
        >= cfg.stage2_minimum_actual_q_window_starts_per_side
    )
    enough_samples = enough_stage1_samples

    # Stage 0 proves strike bootstrap and physical survival only.  Strict READY is deliberately
    # absent: Stage 1 provides the sampled release questions and all-joint safe-set gradient needed
    # to learn it.
    stage1_enter_ok = bool(
        enough_steps
        and enough_stage1_samples
        and min(rates["completion_fh"], rates["completion_bh"])
        >= cfg.stage1_enter_completion
        and min(rates["position_fh"], rates["position_bh"])
        >= cfg.stage1_enter_position
        and min(rates["velocity_fh"], rates["velocity_bh"])
        >= cfg.stage1_enter_velocity
        and min(rates["normal_fh"], rates["normal_bh"])
        >= cfg.stage1_enter_normal
        and min(rates["composite_fh"], rates["composite_bh"])
        >= cfg.stage1_enter_composite
        and max(rates["post_fall_fh"], rates["post_fall_bh"])
        <= cfg.stage1_enter_post_fall
        and max(rates["actual_q_fault_fh"], rates["actual_q_fault_bh"])
        <= cfg.stage1_enter_actual_q_fault
    )
    ready_threshold = (
        float(cfg.stage1_enter_ready)
        if acquisition_ready_threshold is None
        else float(acquisition_ready_threshold)
    )
    if not math.isfinite(ready_threshold) or not 0.0 <= ready_threshold <= 1.0:
        raise ValueError(
            "acquisition_ready_threshold must be finite and lie in [0, 1]"
        )
    stage1_ready_ok = bool(
        min(rates["ready_fh"], rates["ready_bh"]) >= ready_threshold
    )
    stage1_strike_or_plant_exit_bad = bool(
        min(rates["position_fh"], rates["position_bh"])
        < cfg.stage1_exit_position
        or min(rates["velocity_fh"], rates["velocity_bh"])
        < cfg.stage1_exit_velocity
        or min(rates["normal_fh"], rates["normal_bh"])
        < cfg.stage1_exit_normal
        or min(rates["composite_fh"], rates["composite_bh"])
        < cfg.stage1_exit_composite
        or max(rates["post_fall_fh"], rates["post_fall_bh"])
        > cfg.stage1_exit_post_fall
        or max(rates["actual_q_fault_fh"], rates["actual_q_fault_bh"])
        > cfg.stage1_exit_actual_q_fault
    )
    # READY is the acquisition target in early Stage 1.  A sampled READY timeout must not make
    # that target look like a strike-safety regression, but true falls/aborts remain in the
    # release-eligible completion denominator.
    stage1_safety_exit_bad = bool(
        min(
            rates["release_eligible_completion_fh"],
            rates["release_eligible_completion_bh"],
        )
        < cfg.stage1_exit_completion
        or stage1_strike_or_plant_exit_bad
    )
    # Once READY coverage is unlocked, use the honest end-to-end completion again.  At that point
    # both readiness and the downstream strike must remain healthy.
    stage1_exit_bad = bool(
        min(rates["completion_fh"], rates["completion_bh"])
        < cfg.stage1_exit_completion
        or stage1_strike_or_plant_exit_bad
        or min(rates["ready_fh"], rates["ready_bh"]) < cfg.stage1_exit_ready
    )
    if cfg.stage2_enter_actual_q_fault <= 0.0:
        stage2_actual_q_ok = bool(
            actual_q_window_ready
            and max(
                float(metrics.actual_q_fault_events_fh),
                float(metrics.actual_q_fault_events_bh),
            )
            <= 0.0
        )
    else:
        stage2_actual_q_ok = bool(
            actual_q_window_ready
            and max(rates["actual_q_fault_fh"], rates["actual_q_fault_bh"])
            <= cfg.stage2_enter_actual_q_fault
        )
    stage2_enter_ok = bool(
        enough_steps
        and enough_stage2_samples
        and min(rates["completion_fh"], rates["completion_bh"])
        >= cfg.stage2_enter_completion
        and min(rates["position_fh"], rates["position_bh"])
        >= cfg.stage2_enter_position
        and min(rates["velocity_fh"], rates["velocity_bh"])
        >= cfg.stage2_enter_velocity
        and min(rates["normal_fh"], rates["normal_bh"])
        >= cfg.stage2_enter_normal
        and min(rates["composite_fh"], rates["composite_bh"])
        >= cfg.stage2_enter_composite
        and min(rates["ready_fh"], rates["ready_bh"]) >= cfg.stage2_enter_ready
        and min(rates["safe_recovery_fh"], rates["safe_recovery_bh"])
        >= cfg.stage2_enter_safe_recovery
        and min(rates["virtual_contact_fh"], rates["virtual_contact_bh"])
        >= cfg.stage2_enter_virtual_contact
        and min(rates["virtual_over_net_fh"], rates["virtual_over_net_bh"])
        >= cfg.stage2_enter_virtual_over_net
        and min(rates["virtual_legal_fh"], rates["virtual_legal_bh"])
        >= cfg.stage2_enter_virtual_legal
        and max(rates["post_fall_fh"], rates["post_fall_bh"])
        <= cfg.stage2_enter_post_fall
        and stage2_actual_q_ok
    )
    stage2_exit_bad = bool(
        min(rates["completion_fh"], rates["completion_bh"])
        < cfg.stage2_exit_completion
        or min(rates["position_fh"], rates["position_bh"])
        < cfg.stage2_exit_position
        or min(rates["velocity_fh"], rates["velocity_bh"])
        < cfg.stage2_exit_velocity
        or min(rates["normal_fh"], rates["normal_bh"])
        < cfg.stage2_exit_normal
        or min(rates["composite_fh"], rates["composite_bh"])
        < cfg.stage2_exit_composite
        or min(rates["ready_fh"], rates["ready_bh"]) < cfg.stage2_exit_ready
        or min(rates["safe_recovery_fh"], rates["safe_recovery_bh"])
        < cfg.stage2_exit_safe_recovery
        or min(rates["virtual_contact_fh"], rates["virtual_contact_bh"])
        < cfg.stage2_exit_virtual_contact
        or min(rates["virtual_over_net_fh"], rates["virtual_over_net_bh"])
        < cfg.stage2_exit_virtual_over_net
        or min(rates["virtual_legal_fh"], rates["virtual_legal_bh"])
        < cfg.stage2_exit_virtual_legal
        or max(rates["post_fall_fh"], rates["post_fall_bh"])
        > cfg.stage2_exit_post_fall
        or max(rates["actual_q_fault_fh"], rates["actual_q_fault_bh"])
        > cfg.stage2_exit_actual_q_fault
    )

    # Stable dashboard masks. Stage-0 bootstrap, Stage-1 READY acquisition and Stage-2 coverage
    # have separate masks so an expected future-stage failure is never mislabeled as the current
    # admission blocker.
    block_reason = 0
    block_reason |= 1 if not enough_steps else 0
    block_reason |= 2 if not enough_stage1_samples else 0
    bit_checks = (
        (4, rates["completion_fh"] < cfg.stage1_enter_completion),
        (8, rates["completion_bh"] < cfg.stage1_enter_completion),
        (16, rates["position_fh"] < cfg.stage1_enter_position),
        (32, rates["position_bh"] < cfg.stage1_enter_position),
        (64, rates["velocity_fh"] < cfg.stage1_enter_velocity),
        (128, rates["velocity_bh"] < cfg.stage1_enter_velocity),
        (256, rates["normal_fh"] < cfg.stage1_enter_normal),
        (512, rates["normal_bh"] < cfg.stage1_enter_normal),
        (1024, rates["composite_fh"] < cfg.stage1_enter_composite),
        (2048, rates["composite_bh"] < cfg.stage1_enter_composite),
        (
            4096,
            max(rates["post_fall_fh"], rates["post_fall_bh"])
            > cfg.stage1_enter_post_fall,
        ),
        (
            8192,
            max(rates["actual_q_fault_fh"], rates["actual_q_fault_bh"])
            > cfg.stage1_enter_actual_q_fault,
        ),
    )
    for bit, blocked in bit_checks:
        block_reason |= bit if blocked else 0
    acquisition_block_reason = 0
    acquisition_block_reason |= (
        1 if rates["ready_fh"] < ready_threshold else 0
    )
    acquisition_block_reason |= (
        2 if rates["ready_bh"] < ready_threshold else 0
    )
    stage2_block_reason = 0
    stage2_checks = (
        (1, not enough_stage2_samples),
        (2, not actual_q_window_ready),
        (4, rates["completion_fh"] < cfg.stage2_enter_completion),
        (8, rates["completion_bh"] < cfg.stage2_enter_completion),
        (16, rates["position_fh"] < cfg.stage2_enter_position),
        (32, rates["position_bh"] < cfg.stage2_enter_position),
        (64, rates["velocity_fh"] < cfg.stage2_enter_velocity),
        (128, rates["velocity_bh"] < cfg.stage2_enter_velocity),
        (256, rates["normal_fh"] < cfg.stage2_enter_normal),
        (512, rates["normal_bh"] < cfg.stage2_enter_normal),
        (1024, rates["composite_fh"] < cfg.stage2_enter_composite),
        (2048, rates["composite_bh"] < cfg.stage2_enter_composite),
        (4096, rates["ready_fh"] < cfg.stage2_enter_ready),
        (8192, rates["ready_bh"] < cfg.stage2_enter_ready),
        (16384, rates["safe_recovery_fh"] < cfg.stage2_enter_safe_recovery),
        (32768, rates["safe_recovery_bh"] < cfg.stage2_enter_safe_recovery),
        (65536, rates["virtual_contact_fh"] < cfg.stage2_enter_virtual_contact),
        (131072, rates["virtual_contact_bh"] < cfg.stage2_enter_virtual_contact),
        (262144, rates["virtual_over_net_fh"] < cfg.stage2_enter_virtual_over_net),
        (524288, rates["virtual_over_net_bh"] < cfg.stage2_enter_virtual_over_net),
        (1048576, rates["virtual_legal_fh"] < cfg.stage2_enter_virtual_legal),
        (2097152, rates["virtual_legal_bh"] < cfg.stage2_enter_virtual_legal),
        (
            4194304,
            max(rates["post_fall_fh"], rates["post_fall_bh"])
            > cfg.stage2_enter_post_fall,
        ),
        (8388608, not stage2_actual_q_ok),
    )
    for bit, blocked in stage2_checks:
        stage2_block_reason |= bit if blocked else 0
    return RecoveryCurriculumConditions(
        enough_steps=enough_steps,
        enough_samples=enough_samples,
        enough_stage1_samples=enough_stage1_samples,
        enough_stage2_samples=enough_stage2_samples,
        actual_q_window_ready=actual_q_window_ready,
        stage1_enter_ok=stage1_enter_ok,
        stage1_ready_ok=stage1_ready_ok,
        stage1_safety_exit_bad=stage1_safety_exit_bad,
        stage1_exit_bad=stage1_exit_bad,
        stage2_enter_ok=stage2_enter_ok,
        stage2_exit_bad=stage2_exit_bad,
        block_reason=block_reason,
        acquisition_block_reason=acquisition_block_reason,
        stage2_block_reason=stage2_block_reason,
    )


def _ramp_rate(start: float, target: float, steps: int) -> float:
    return abs(float(target) - float(start)) / float(steps)


def _move_toward(current: float, target: float, rate: float) -> float:
    if current < target:
        return min(target, current + rate)
    if current > target:
        return max(target, current - rate)
    return target


def advance_recovery_curriculum(
    state: RecoveryCurriculumState,
    metrics: RecoveryCurriculumMetrics,
    cfg: RecoveryCurriculumConfig,
    *,
    enabled: bool = True,
) -> tuple[RecoveryCurriculumState, RecoveryCurriculumConditions]:
    """Advance one control step while preserving ``0 <= scale <= 1``.

    Disabling the curriculum is an exact baseline contract: all state that can affect the task
    distribution is returned at stage 0 / scale 0.
    """

    cfg.validate()
    if state.stage not in (0, 1, 2):
        raise ValueError(f"recovery stage must be 0, 1 or 2, got {state.stage}")
    acquisition_scales = cfg.acquisition_scales()
    acquisition_ready_thresholds = cfg.acquisition_ready_thresholds()
    acquisition_rung = int(state.stage1_acquisition_rung)
    if not 0 <= acquisition_rung < len(acquisition_scales):
        raise ValueError(
            "stage1 acquisition rung is outside the configured ladder: "
            f"{acquisition_rung} not in [0, {len(acquisition_scales)})"
        )
    acquisition_failures = int(state.stage1_acquisition_failures)
    if acquisition_failures < 0:
        raise ValueError("stage1 acquisition failures must be non-negative")
    conditions = _conditions(
        metrics,
        cfg,
        acquisition_ready_threshold=(
            acquisition_ready_thresholds[acquisition_rung]
        ),
    )
    if not enabled:
        return RecoveryCurriculumState(), conditions

    stage = int(state.stage)
    current = _bounded_rate(state.current_scale, "current_scale")
    target = _bounded_rate(state.target_scale, "target_scale")
    ramp_rate = max(float(state.ramp_rate_per_step), 0.0)
    coverage = _bounded_rate(state.coverage_scale, "coverage_scale")
    coverage_target = _bounded_rate(
        state.coverage_target_scale, "coverage_target_scale"
    )
    coverage_ramp_rate = max(
        float(state.coverage_ramp_rate_per_step), 0.0
    )
    coverage_unlocked = bool(state.stage1_coverage_unlocked)
    ready_dwell = int(state.stage1_ready_dwell)
    acquisition_age = int(state.stage1_acquisition_age)
    s1_enter = int(state.stage1_enter_dwell)
    s1_exit = int(state.stage1_exit_dwell)
    s2_enter = int(state.stage2_enter_dwell)
    s2_exit = int(state.stage2_exit_dwell)

    if stage == 0:
        acquisition_rung = 0
        at_baseline = current <= 1.0e-12 and coverage <= 1.0e-12
        s1_enter = (
            s1_enter + 1
            if (conditions.stage1_enter_ok and at_baseline)
            else 0
        )
        s1_exit = s2_enter = s2_exit = 0
        ready_dwell = acquisition_age = 0
        coverage_unlocked = False
        if s1_enter >= cfg.stage1_enter_dwell_steps:
            stage = 1
            target = acquisition_scales[0]
            first_ramp_steps = (
                cfg.stage1_acquisition_ramp_steps
                if len(acquisition_scales) > 1
                else cfg.stage0_to_1_ramp_steps
            )
            ramp_rate = _ramp_rate(0.0, target, first_ramp_steps)
            coverage_target = 0.0
            coverage_ramp_rate = 0.0
            s1_enter = 0
    elif stage == 1:
        acquisition_target = acquisition_scales[acquisition_rung]
        if not coverage_unlocked and abs(target - acquisition_target) > 1.0e-12:
            target = max(current, acquisition_target)
            ramp_rate = _ramp_rate(
                current,
                target,
                cfg.stage1_acquisition_ramp_steps,
            )
        stage1_at_target = abs(current - acquisition_target) <= 1.0e-9
        coverage_at_target = (
            abs(coverage - cfg.stage1_scale) <= 1.0e-9
        )
        if coverage_unlocked:
            exit_bad = conditions.stage1_exit_bad
            ready_dwell = acquisition_age = 0
        else:
            # During acquisition, READY is the learning target rather than an exit condition.
            # Strike/fall/actual-q regressions remain fully active.
            exit_bad = conditions.stage1_safety_exit_bad
            if stage1_at_target:
                acquisition_age += 1
                ready_dwell = (
                    ready_dwell + 1 if conditions.stage1_ready_ok else 0
                )
            else:
                ready_dwell = acquisition_age = 0
        s1_exit = s1_exit + 1 if exit_bad else 0
        s2_enter = (
            s2_enter + 1
            if (
                coverage_unlocked
                and stage1_at_target
                and coverage_at_target
                and conditions.stage2_enter_ok
            )
            else 0
        )
        s1_enter = s2_exit = 0
        # R9 never removes learned exposure. Safety/quality regressions remain visible through
        # s1_exit and the live metrics, while adaptive replay increases the probability of the
        # failed side/phase/severity question. This removes the R8 learn-fail-backoff loop.
        if (
            not coverage_unlocked
            and ready_dwell >= cfg.stage1_ready_dwell_steps
        ):
            if acquisition_rung + 1 < len(acquisition_scales):
                acquisition_rung += 1
                target = acquisition_scales[acquisition_rung]
                ramp_rate = _ramp_rate(
                    current,
                    target,
                    cfg.stage1_acquisition_ramp_steps,
                )
            else:
                coverage_unlocked = True
                coverage_target = cfg.stage1_scale
                coverage_ramp_rate = _ramp_rate(
                    coverage,
                    cfg.stage1_scale,
                    cfg.stage1_coverage_ramp_steps,
                )
            ready_dwell = acquisition_age = 0
        elif s2_enter >= cfg.stage2_enter_dwell_steps:
            stage = 2
            acquisition_rung = len(acquisition_scales) - 1
            target = 1.0
            ramp_rate = _ramp_rate(cfg.stage1_scale, 1.0, cfg.stage1_to_2_ramp_steps)
            coverage_target = 1.0
            coverage_ramp_rate = _ramp_rate(
                cfg.stage1_scale, 1.0, cfg.stage1_to_2_ramp_steps
            )
            s2_enter = 0
    else:
        acquisition_rung = len(acquisition_scales) - 1
        s2_exit = s2_exit + 1 if conditions.stage2_exit_bad else 0
        s1_enter = s1_exit = s2_enter = 0
        coverage_unlocked = True
        ready_dwell = acquisition_age = 0
        # Stage 2 is also one-way. Keep the exit counter as a health alarm; never make the
        # on-policy distribution oscillate by silently removing robustness.

    target = max(target, current)
    coverage_target = max(coverage_target, coverage)
    current = _move_toward(current, target, ramp_rate)
    if abs(current - target) <= 1.0e-12:
        current = target
        ramp_rate = 0.0
    current = min(max(current, 0.0), 1.0)
    coverage = _move_toward(
        coverage, coverage_target, coverage_ramp_rate
    )
    if abs(coverage - coverage_target) <= 1.0e-12:
        coverage = coverage_target
        coverage_ramp_rate = 0.0
    coverage = min(max(coverage, 0.0), 1.0)
    return (
        RecoveryCurriculumState(
            stage=stage,
            current_scale=current,
            target_scale=target,
            ramp_rate_per_step=ramp_rate,
            coverage_scale=coverage,
            coverage_target_scale=coverage_target,
            coverage_ramp_rate_per_step=coverage_ramp_rate,
            stage1_coverage_unlocked=coverage_unlocked,
            stage1_acquisition_rung=acquisition_rung,
            stage1_acquisition_failures=acquisition_failures,
            stage1_ready_dwell=ready_dwell,
            stage1_acquisition_age=acquisition_age,
            stage1_enter_dwell=s1_enter,
            stage1_exit_dwell=s1_exit,
            stage2_enter_dwell=s2_enter,
            stage2_exit_dwell=s2_exit,
        ),
        conditions,
    )
