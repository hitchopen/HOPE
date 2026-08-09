"""Pure helpers for the strike-precision and velocity curricula.

This module deliberately has no Isaac Lab or torch dependency so the state transitions can be
tested on a host-only Python installation.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


VELOCITY_BLOCK_REASON_BITS = {
    "position_fh": 1,
    "position_bh": 2,
    "normal_fh": 4,
    "normal_bh": 8,
    "post_fall": 16,
    "ready": 32,
    "sample_count": 64,
    "velocity_fh": 128,
    "velocity_bh": 256,
}


def time_window_active(tts_s: float, window_s: float) -> bool:
    """Return the inclusive policy-tick gate used by strike-local reward terms."""
    if window_s < 0.0:
        raise ValueError(f"window_s must be non-negative, got {window_s}")
    return abs(float(tts_s)) <= float(window_s) + 1.0e-9


def moving_strike_target(
    strike_position: Sequence[float],
    target_velocity: Sequence[float],
    tts_s: float,
) -> tuple[float, ...]:
    """First-order strike trajectory: p_moving = p_strike - v_target * tts."""
    position = tuple(float(value) for value in strike_position)
    velocity = tuple(float(value) for value in target_velocity)
    if len(position) != len(velocity):
        raise ValueError(
            "strike_position and target_velocity must have equal length, got "
            f"{len(position)} and {len(velocity)}"
        )
    return tuple(
        position_value - velocity_value * float(tts_s)
        for position_value, velocity_value in zip(position, velocity)
    )


def position_guidance_reward_rate(
    error_m: float,
    sigma_m: float,
    tts_s: float,
    *,
    window_s: float = 0.04,
    temporal_scale: float = 2.0,
) -> float:
    """Unweighted moving-position guidance rate before RewardManager weight and dt."""
    if error_m < 0.0:
        raise ValueError(f"error_m must be non-negative, got {error_m}")
    if sigma_m <= 0.0:
        raise ValueError(f"sigma_m must be positive, got {sigma_m}")
    if temporal_scale < 0.0:
        raise ValueError(
            f"temporal_scale must be non-negative, got {temporal_scale}"
        )
    if not time_window_active(tts_s, window_s):
        return 0.0
    return float(temporal_scale) * math.exp(
        -float(error_m) ** 2 / float(sigma_m) ** 2
    )


def exact_position_debt_raw(
    error_m: float,
    *,
    margin_m: float = 0.075,
    huber_scale_m: float = 0.025,
) -> float:
    """Static-strike-point Huber debt before its negative RewardTerm weight."""
    return deadband_huber_debt_raw(
        error_m, margin=margin_m, huber_scale=huber_scale_m
    )


def deadband_huber_debt_raw(
    error: float,
    *,
    margin: float,
    huber_scale: float,
) -> float:
    """Dimensionless smooth-L1 debt with a physical-unit deadband and scale."""

    error = float(error)
    margin = float(margin)
    huber_scale = float(huber_scale)
    if not math.isfinite(error) or error < 0.0:
        raise ValueError(f"error must be finite and non-negative, got {error}")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError(f"margin must be finite and non-negative, got {margin}")
    if not math.isfinite(huber_scale) or huber_scale <= 0.0:
        raise ValueError(
            f"huber_scale must be finite and positive, got {huber_scale}"
        )
    scaled = max(error - margin, 0.0) / huber_scale
    if scaled <= 0.0:
        return 0.0
    if scaled <= 1.0:
        return 0.5 * scaled**2
    return scaled - 0.5


def staged_target_robustness(
    stage: int,
    *,
    current_weight: float,
    stage0_weight: float,
    stage1_weight: float,
    stage2_weight: float,
    max_delay_steps: int,
    stage1_scale: float,
    enabled: bool,
) -> tuple[float, int]:
    """Continuously couple target corruption to the live staged velocity weight.

    Stage transitions are discrete, but their velocity weights already ramp in both directions.
    Using that same live weight avoids applying half/full target corruption on a single control
    tick.  The piecewise-linear map is 0 at ``stage0_weight``, ``stage1_scale`` at
    ``stage1_weight``, and 1 at ``stage2_weight``.  It also gives a symmetric, gradual retreat
    after a safety-gated stage exit.
    """

    stage = int(stage)
    current_weight = float(current_weight)
    stage0_weight = float(stage0_weight)
    stage1_weight = float(stage1_weight)
    stage2_weight = float(stage2_weight)
    max_delay_steps = int(max_delay_steps)
    stage1_scale = float(stage1_scale)
    if stage not in (0, 1, 2):
        raise ValueError(f"target robustness stage must be 0, 1 or 2, got {stage}")
    if not all(
        math.isfinite(value)
        for value in (
            current_weight,
            stage0_weight,
            stage1_weight,
            stage2_weight,
        )
    ):
        raise ValueError("target robustness weights must all be finite")
    if not stage0_weight < stage1_weight < stage2_weight:
        raise ValueError(
            "target robustness weights must satisfy stage0 < stage1 < stage2, "
            f"got {stage0_weight}, {stage1_weight}, {stage2_weight}"
        )
    if max_delay_steps < 0:
        raise ValueError(
            f"max_delay_steps must be non-negative, got {max_delay_steps}"
        )
    if not math.isfinite(stage1_scale) or not 0.0 <= stage1_scale <= 1.0:
        raise ValueError(
            f"stage1_scale must be finite and in [0, 1], got {stage1_scale}"
        )
    if not enabled:
        scale = 1.0
    elif current_weight <= stage1_weight:
        progress = (
            min(max(current_weight, stage0_weight), stage1_weight)
            - stage0_weight
        ) / (stage1_weight - stage0_weight)
        scale = stage1_scale * progress
    else:
        progress = (
            min(max(current_weight, stage1_weight), stage2_weight)
            - stage1_weight
        ) / (stage2_weight - stage1_weight)
        scale = stage1_scale + (1.0 - stage1_scale) * progress
    scale = min(max(scale, 0.0), 1.0)
    delay_steps = min(
        max(int(round(float(max_delay_steps) * scale)), 0),
        max_delay_steps,
    )
    return scale, delay_steps


def staged_velocity_sampling(
    *,
    current_weight: float,
    stage0_weight: float,
    stage1_weight: float,
    stage2_weight: float,
    stage1_planner_mix: float,
    final_planner_mix: float,
) -> tuple[float, float]:
    """Map learned velocity competence to core-box expansion and planner mix.

    The bootstrap core expands fully by the Stage-1 weight. Planner targets ramp from zero to
    ``stage1_planner_mix`` over the same interval, then to the declared final mixture at Stage 2.
    This preserves the final deployment distribution without exposing it on a wall-clock schedule.
    """

    values = (
        current_weight,
        stage0_weight,
        stage1_weight,
        stage2_weight,
        stage1_planner_mix,
        final_planner_mix,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("staged velocity sampling values must be finite")
    if not stage0_weight < stage1_weight < stage2_weight:
        raise ValueError(
            "staged velocity sampling weights must satisfy stage0 < stage1 < stage2"
        )
    if not 0.0 <= stage1_planner_mix <= final_planner_mix <= 1.0:
        raise ValueError(
            "planner mixtures must satisfy 0 <= stage1 <= final <= 1"
        )
    bounded = min(max(float(current_weight), stage0_weight), stage2_weight)
    core_scale = min(
        max(
            (bounded - stage0_weight) / (stage1_weight - stage0_weight),
            0.0,
        ),
        1.0,
    )
    if bounded <= stage1_weight:
        planner_mix = stage1_planner_mix * core_scale
    else:
        stage2_progress = (bounded - stage1_weight) / (
            stage2_weight - stage1_weight
        )
        planner_mix = stage1_planner_mix + stage2_progress * (
            final_planner_mix - stage1_planner_mix
        )
    return core_scale, min(max(planner_mix, 0.0), final_planner_mix)


@dataclass(frozen=True)
class VelocityStageMetrics:
    """EMA metrics and effective sample masses consumed by the staged gate."""

    position_fh: float
    position_bh: float
    velocity_fh: float
    velocity_bh: float
    normal_fh: float
    normal_bh: float
    post_fall: float
    ready: float
    exact_samples_fh: float
    exact_samples_bh: float
    swing_start_samples: float


@dataclass(frozen=True)
class VelocityStageConfig:
    """Two-stage velocity-weight targets, metric hysteresis, dwell and slew rates."""

    stage0_weight: float = 14.0
    stage1_weight: float = 18.0
    stage2_weight: float = 24.0

    stage1_enter_position: float = 0.12
    stage1_enter_velocity: float = 0.005
    stage1_enter_normal: float = 0.45
    stage1_enter_post_fall: float = 0.10
    stage1_enter_ready: float = 0.20
    stage1_exit_position: float = 0.10
    stage1_exit_velocity: float = 0.0025
    stage1_exit_normal: float = 0.40
    stage1_exit_post_fall: float = 0.12
    stage1_exit_ready: float = 0.12

    stage2_enter_position: float = 0.15
    stage2_enter_velocity: float = 0.05
    stage2_enter_normal: float = 0.50
    stage2_enter_post_fall: float = 0.08
    stage2_enter_ready: float = 0.35
    stage2_exit_position: float = 0.13
    stage2_exit_velocity: float = 0.03
    stage2_exit_normal: float = 0.45
    stage2_exit_post_fall: float = 0.10
    stage2_exit_ready: float = 0.25

    # 2026-07-25 deadlock repair. The 7-way AND below never once evaluated True across
    # 172,056 control steps of the 2026-07-25_03-54-38 run (stage1_enter_dwell max == 0), and
    # the same normal/ready terms appear in the EXIT condition, so the sustained exit-bad path
    # kept clearing whatever partial enter dwell existed (that run's checkpoint carried
    # _velocity_stage1_exit_dwell == 97/100). Worse, the components never held simultaneously:
    # `ready` passed at iters 0-3000 and collapsed to 0 thereafter, while `post_fall` was 0
    # until iter 6000. The gate's stated intent (task YAML) is to stop the dense balance terms
    # from being traded away for premature racket speed -- that risk is measured by post_fall,
    # not by the normal/ready competence proxies. These switches drop normal and/or ready from
    # BOTH the enter and the exit condition; dropping them from only one side reintroduces the
    # deadlock. Defaults stay True so every pre-existing task keeps its exact behaviour.
    stage_gate_requires_normal: bool = True
    stage_gate_requires_ready: bool = True
    # Optional per-stage overrides let one run bootstrap contact at Stage 0/1 without recreating
    # the old seven-way deadlock, while still requiring face control and runner-equivalent READY
    # before Stage 2 exposes the full-speed/full-robustness task. ``None`` preserves the global
    # switches above for every existing recipe and frozen exact resume.
    stage1_gate_requires_normal: bool | None = None
    stage1_gate_requires_ready: bool | None = None
    stage2_gate_requires_normal: bool | None = None
    stage2_gate_requires_ready: bool | None = None

    min_exact_samples_per_side: float = 50.0
    min_swing_start_samples: float = 50.0
    stage1_enter_dwell_steps: int = 250
    stage1_exit_dwell_steps: int = 100
    stage2_enter_dwell_steps: int = 500
    stage2_exit_dwell_steps: int = 150
    stage0_to_1_ramp_steps: int = 8000
    stage1_to_0_ramp_steps: int = 4000
    stage1_to_2_ramp_steps: int = 12000
    stage2_to_1_ramp_steps: int = 6000

    def validate(self) -> None:
        weights = (self.stage0_weight, self.stage1_weight, self.stage2_weight)
        if not all(math.isfinite(value) for value in weights):
            raise ValueError(f"stage weights must be finite, got {weights}")
        if not (weights[0] < weights[1] < weights[2]):
            raise ValueError(
                "stage weights must be strictly increasing, got "
                f"{weights}"
            )
        rate_fields = (
            self.stage1_enter_position,
            self.stage1_enter_velocity,
            self.stage1_enter_normal,
            self.stage1_enter_post_fall,
            self.stage1_enter_ready,
            self.stage1_exit_position,
            self.stage1_exit_velocity,
            self.stage1_exit_normal,
            self.stage1_exit_post_fall,
            self.stage1_exit_ready,
            self.stage2_enter_position,
            self.stage2_enter_velocity,
            self.stage2_enter_normal,
            self.stage2_enter_post_fall,
            self.stage2_enter_ready,
            self.stage2_exit_position,
            self.stage2_exit_velocity,
            self.stage2_exit_normal,
            self.stage2_exit_post_fall,
            self.stage2_exit_ready,
        )
        if not all(0.0 <= value <= 1.0 for value in rate_fields):
            raise ValueError("all stage rate thresholds must be in [0, 1]")
        for stage in (1, 2):
            enter_position = getattr(self, f"stage{stage}_enter_position")
            exit_position = getattr(self, f"stage{stage}_exit_position")
            enter_velocity = getattr(self, f"stage{stage}_enter_velocity")
            exit_velocity = getattr(self, f"stage{stage}_exit_velocity")
            enter_normal = getattr(self, f"stage{stage}_enter_normal")
            exit_normal = getattr(self, f"stage{stage}_exit_normal")
            enter_fall = getattr(self, f"stage{stage}_enter_post_fall")
            exit_fall = getattr(self, f"stage{stage}_exit_post_fall")
            enter_ready = getattr(self, f"stage{stage}_enter_ready")
            exit_ready = getattr(self, f"stage{stage}_exit_ready")
            if not (
                enter_position > exit_position
                and enter_velocity > exit_velocity
                and enter_normal > exit_normal
                and enter_fall < exit_fall
                and enter_ready > exit_ready
            ):
                raise ValueError(
                    f"stage {stage} enter/exit thresholds do not define a "
                    "strict neutral hysteresis zone"
                )
        if (
            self.min_exact_samples_per_side < 0.0
            or self.min_swing_start_samples < 0.0
        ):
            raise ValueError("sample-count thresholds must be non-negative")
        step_fields = (
            self.stage1_enter_dwell_steps,
            self.stage1_exit_dwell_steps,
            self.stage2_enter_dwell_steps,
            self.stage2_exit_dwell_steps,
            self.stage0_to_1_ramp_steps,
            self.stage1_to_0_ramp_steps,
            self.stage1_to_2_ramp_steps,
            self.stage2_to_1_ramp_steps,
        )
        if any(int(value) < 1 for value in step_fields):
            raise ValueError("all dwell and ramp steps must be >= 1")


@dataclass(frozen=True)
class VelocityStageConditions:
    """Auditable enter/exit predicates and per-channel blockers."""

    sample_count_ok: bool
    stage1_position_fh_ok: bool
    stage1_position_bh_ok: bool
    stage1_velocity_fh_ok: bool
    stage1_velocity_bh_ok: bool
    stage1_normal_fh_ok: bool
    stage1_normal_bh_ok: bool
    stage1_post_fall_ok: bool
    stage1_ready_ok: bool
    stage1_enter_ok: bool
    stage1_exit_bad: bool
    stage2_position_fh_ok: bool
    stage2_position_bh_ok: bool
    stage2_velocity_fh_ok: bool
    stage2_velocity_bh_ok: bool
    stage2_normal_fh_ok: bool
    stage2_normal_bh_ok: bool
    stage2_post_fall_ok: bool
    stage2_ready_ok: bool
    stage2_enter_ok: bool
    stage2_exit_bad: bool
    stage1_blocked_reasons: tuple[str, ...]
    stage2_blocked_reasons: tuple[str, ...]

    def blocked_reason_mask(self, stage: int) -> int:
        reasons = (
            self.stage1_blocked_reasons
            if int(stage) <= 0
            else self.stage2_blocked_reasons
        )
        mask = 0
        for reason in reasons:
            mask |= VELOCITY_BLOCK_REASON_BITS[reason]
        return mask


@dataclass(frozen=True)
class VelocityStageState:
    """Checkpointable state for the staged curriculum."""

    stage: int = 0
    current_weight: float = 14.0
    target_weight: float = 14.0
    stage1_enter_dwell: int = 0
    stage1_exit_dwell: int = 0
    stage2_enter_dwell: int = 0
    stage2_exit_dwell: int = 0


def _blocked_reasons(
    metrics: VelocityStageMetrics,
    *,
    position_threshold: float,
    velocity_threshold: float,
    normal_threshold: float,
    post_fall_threshold: float,
    ready_threshold: float,
    sample_count_ok: bool,
    require_normal: bool,
    require_ready: bool,
) -> tuple[str, ...]:
    reasons = []
    if metrics.position_fh < position_threshold:
        reasons.append("position_fh")
    if metrics.position_bh < position_threshold:
        reasons.append("position_bh")
    if metrics.velocity_fh < velocity_threshold:
        reasons.append("velocity_fh")
    if metrics.velocity_bh < velocity_threshold:
        reasons.append("velocity_bh")
    if require_normal and metrics.normal_fh < normal_threshold:
        reasons.append("normal_fh")
    if require_normal and metrics.normal_bh < normal_threshold:
        reasons.append("normal_bh")
    if metrics.post_fall > post_fall_threshold:
        reasons.append("post_fall")
    if require_ready and metrics.ready < ready_threshold:
        reasons.append("ready")
    if not sample_count_ok:
        reasons.append("sample_count")
    return tuple(reasons)


def velocity_stage_conditions(
    metrics: VelocityStageMetrics, config: VelocityStageConfig
) -> VelocityStageConditions:
    """Evaluate both stage transitions without mutating curriculum state."""
    config.validate()
    metric_rates = (
        metrics.position_fh,
        metrics.position_bh,
        metrics.velocity_fh,
        metrics.velocity_bh,
        metrics.normal_fh,
        metrics.normal_bh,
        metrics.post_fall,
        metrics.ready,
    )
    if not all(math.isfinite(float(value)) for value in metric_rates):
        raise ValueError(f"velocity-stage metrics must be finite, got {metric_rates}")
    if not all(0.0 <= float(value) <= 1.0 for value in metric_rates):
        raise ValueError(f"velocity-stage rates must be in [0, 1], got {metric_rates}")
    counts = (
        metrics.exact_samples_fh,
        metrics.exact_samples_bh,
        metrics.swing_start_samples,
    )
    if not all(math.isfinite(float(value)) and float(value) >= 0.0 for value in counts):
        raise ValueError(
            f"velocity-stage sample counts must be finite and non-negative, got {counts}"
        )
    sample_count_ok = bool(
        metrics.exact_samples_fh >= config.min_exact_samples_per_side
        and metrics.exact_samples_bh >= config.min_exact_samples_per_side
        and metrics.swing_start_samples >= config.min_swing_start_samples
    )

    stage1_position_fh_ok = metrics.position_fh >= config.stage1_enter_position
    stage1_position_bh_ok = metrics.position_bh >= config.stage1_enter_position
    stage1_velocity_fh_ok = (
        metrics.velocity_fh >= config.stage1_enter_velocity
    )
    stage1_velocity_bh_ok = (
        metrics.velocity_bh >= config.stage1_enter_velocity
    )
    stage1_normal_fh_ok = metrics.normal_fh >= config.stage1_enter_normal
    stage1_normal_bh_ok = metrics.normal_bh >= config.stage1_enter_normal
    stage1_post_fall_ok = metrics.post_fall <= config.stage1_enter_post_fall
    stage1_ready_ok = metrics.ready >= config.stage1_enter_ready
    # The *_ok booleans above stay unconditional so the telemetry keeps reporting every gate
    # even when a gate is not part of the transition condition (that visibility is how the
    # 2026-07-25 deadlock was found). Only the AND/OR below honour the switches.
    stage1_require_normal = (
        bool(config.stage_gate_requires_normal)
        if config.stage1_gate_requires_normal is None
        else bool(config.stage1_gate_requires_normal)
    )
    stage1_require_ready = (
        bool(config.stage_gate_requires_ready)
        if config.stage1_gate_requires_ready is None
        else bool(config.stage1_gate_requires_ready)
    )
    stage1_enter_ok = bool(
        sample_count_ok
        and stage1_position_fh_ok
        and stage1_position_bh_ok
        and stage1_velocity_fh_ok
        and stage1_velocity_bh_ok
        and (stage1_normal_fh_ok or not stage1_require_normal)
        and (stage1_normal_bh_ok or not stage1_require_normal)
        and stage1_post_fall_ok
        and (stage1_ready_ok or not stage1_require_ready)
    )
    stage1_exit_bad = bool(
        metrics.position_fh < config.stage1_exit_position
        or metrics.position_bh < config.stage1_exit_position
        or metrics.velocity_fh < config.stage1_exit_velocity
        or metrics.velocity_bh < config.stage1_exit_velocity
        or (
            stage1_require_normal
            and metrics.normal_fh < config.stage1_exit_normal
        )
        or (
            stage1_require_normal
            and metrics.normal_bh < config.stage1_exit_normal
        )
        or metrics.post_fall > config.stage1_exit_post_fall
        or (
            stage1_require_ready
            and metrics.ready < config.stage1_exit_ready
        )
    )

    stage2_position_fh_ok = metrics.position_fh >= config.stage2_enter_position
    stage2_position_bh_ok = metrics.position_bh >= config.stage2_enter_position
    stage2_velocity_fh_ok = (
        metrics.velocity_fh >= config.stage2_enter_velocity
    )
    stage2_velocity_bh_ok = (
        metrics.velocity_bh >= config.stage2_enter_velocity
    )
    stage2_normal_fh_ok = metrics.normal_fh >= config.stage2_enter_normal
    stage2_normal_bh_ok = metrics.normal_bh >= config.stage2_enter_normal
    stage2_post_fall_ok = metrics.post_fall <= config.stage2_enter_post_fall
    stage2_ready_ok = metrics.ready >= config.stage2_enter_ready
    stage2_require_normal = (
        bool(config.stage_gate_requires_normal)
        if config.stage2_gate_requires_normal is None
        else bool(config.stage2_gate_requires_normal)
    )
    stage2_require_ready = (
        bool(config.stage_gate_requires_ready)
        if config.stage2_gate_requires_ready is None
        else bool(config.stage2_gate_requires_ready)
    )
    stage2_enter_ok = bool(
        sample_count_ok
        and stage2_position_fh_ok
        and stage2_position_bh_ok
        and stage2_velocity_fh_ok
        and stage2_velocity_bh_ok
        and (stage2_normal_fh_ok or not stage2_require_normal)
        and (stage2_normal_bh_ok or not stage2_require_normal)
        and stage2_post_fall_ok
        and (stage2_ready_ok or not stage2_require_ready)
    )
    stage2_exit_bad = bool(
        metrics.position_fh < config.stage2_exit_position
        or metrics.position_bh < config.stage2_exit_position
        or metrics.velocity_fh < config.stage2_exit_velocity
        or metrics.velocity_bh < config.stage2_exit_velocity
        or (
            stage2_require_normal
            and metrics.normal_fh < config.stage2_exit_normal
        )
        or (
            stage2_require_normal
            and metrics.normal_bh < config.stage2_exit_normal
        )
        or metrics.post_fall > config.stage2_exit_post_fall
        or (
            stage2_require_ready
            and metrics.ready < config.stage2_exit_ready
        )
    )

    return VelocityStageConditions(
        sample_count_ok=sample_count_ok,
        stage1_position_fh_ok=stage1_position_fh_ok,
        stage1_position_bh_ok=stage1_position_bh_ok,
        stage1_velocity_fh_ok=stage1_velocity_fh_ok,
        stage1_velocity_bh_ok=stage1_velocity_bh_ok,
        stage1_normal_fh_ok=stage1_normal_fh_ok,
        stage1_normal_bh_ok=stage1_normal_bh_ok,
        stage1_post_fall_ok=stage1_post_fall_ok,
        stage1_ready_ok=stage1_ready_ok,
        stage1_enter_ok=stage1_enter_ok,
        stage1_exit_bad=stage1_exit_bad,
        stage2_position_fh_ok=stage2_position_fh_ok,
        stage2_position_bh_ok=stage2_position_bh_ok,
        stage2_velocity_fh_ok=stage2_velocity_fh_ok,
        stage2_velocity_bh_ok=stage2_velocity_bh_ok,
        stage2_normal_fh_ok=stage2_normal_fh_ok,
        stage2_normal_bh_ok=stage2_normal_bh_ok,
        stage2_post_fall_ok=stage2_post_fall_ok,
        stage2_ready_ok=stage2_ready_ok,
        stage2_enter_ok=stage2_enter_ok,
        stage2_exit_bad=stage2_exit_bad,
        stage1_blocked_reasons=_blocked_reasons(
            metrics,
            position_threshold=config.stage1_enter_position,
            velocity_threshold=config.stage1_enter_velocity,
            normal_threshold=config.stage1_enter_normal,
            post_fall_threshold=config.stage1_enter_post_fall,
            ready_threshold=config.stage1_enter_ready,
            sample_count_ok=sample_count_ok,
            require_normal=stage1_require_normal,
            require_ready=stage1_require_ready,
        ),
        stage2_blocked_reasons=_blocked_reasons(
            metrics,
            position_threshold=config.stage2_enter_position,
            velocity_threshold=config.stage2_enter_velocity,
            normal_threshold=config.stage2_enter_normal,
            post_fall_threshold=config.stage2_enter_post_fall,
            ready_threshold=config.stage2_enter_ready,
            sample_count_ok=sample_count_ok,
            require_normal=stage2_require_normal,
            require_ready=stage2_require_ready,
        ),
    )


def _slew_velocity_weight(
    current_weight: float, target_weight: float, config: VelocityStageConfig
) -> float:
    if current_weight < target_weight:
        if target_weight == config.stage1_weight:
            increment = (
                config.stage1_weight - config.stage0_weight
            ) / config.stage0_to_1_ramp_steps
        else:
            increment = (
                config.stage2_weight - config.stage1_weight
            ) / config.stage1_to_2_ramp_steps
        return min(current_weight + increment, target_weight)
    if current_weight > target_weight:
        if target_weight == config.stage1_weight:
            decrement = (
                config.stage2_weight - config.stage1_weight
            ) / config.stage2_to_1_ramp_steps
        else:
            decrement = (
                config.stage1_weight - config.stage0_weight
            ) / config.stage1_to_0_ramp_steps
        return max(current_weight - decrement, target_weight)
    return current_weight


def advance_staged_velocity_curriculum(
    state: VelocityStageState,
    metrics: VelocityStageMetrics,
    config: VelocityStageConfig,
) -> tuple[VelocityStageState, VelocityStageConditions]:
    """Advance one control tick with metric hysteresis and one-level-only exits.

    Enter dwell is retained in the neutral band. A sustained exit-bad condition is required
    before it is cleared or a stage is lowered. Stage 2 dwell starts only after the Stage 0→1
    weight ramp has actually reached 18, so the intermediate-speed stage cannot be skipped while
    its target weight is still in flight.
    """
    conditions = velocity_stage_conditions(metrics, config)
    stage = int(state.stage)
    if stage not in (0, 1, 2):
        raise ValueError(f"velocity stage must be 0, 1 or 2, got {stage}")
    if not math.isfinite(float(state.current_weight)):
        raise ValueError(
            f"current velocity weight must be finite, got {state.current_weight}"
        )

    stage1_enter = max(int(state.stage1_enter_dwell), 0)
    stage1_exit = max(int(state.stage1_exit_dwell), 0)
    stage2_enter = max(int(state.stage2_enter_dwell), 0)
    stage2_exit = max(int(state.stage2_exit_dwell), 0)

    if stage == 0:
        if conditions.stage1_enter_ok:
            stage1_enter = min(
                stage1_enter + 1, config.stage1_enter_dwell_steps
            )
        if conditions.stage1_exit_bad:
            stage1_exit = min(
                stage1_exit + 1, config.stage1_exit_dwell_steps
            )
            if stage1_exit >= config.stage1_exit_dwell_steps:
                # Stage 0 cannot fall further; sustained exit-bad invalidates stale partial
                # enter dwell without letting one noisy frame erase it.
                stage1_enter = 0
                stage1_exit = 0
        else:
            stage1_exit = 0
        if stage1_enter >= config.stage1_enter_dwell_steps:
            stage = 1
            stage1_exit = 0

    elif stage == 1:
        # A true 18-weight intermediate stage is intentional. Do not count Stage 2 dwell while
        # Stage 1's target is still ramping up (or back down after a Stage 2 exit).
        stage1_weight_reached = (
            abs(float(state.current_weight) - config.stage1_weight) <= 1.0e-9
        )
        if stage1_weight_reached and conditions.stage2_enter_ok:
            stage2_enter = min(
                stage2_enter + 1, config.stage2_enter_dwell_steps
            )
        if conditions.stage1_exit_bad:
            stage1_exit = min(
                stage1_exit + 1, config.stage1_exit_dwell_steps
            )
            if stage1_exit >= config.stage1_exit_dwell_steps:
                stage = 0
                stage1_enter = 0
                stage1_exit = 0
                stage2_enter = 0
                stage2_exit = 0
        else:
            stage1_exit = 0
        if (
            stage == 1
            and stage2_enter >= config.stage2_enter_dwell_steps
        ):
            stage = 2
            stage2_exit = 0

    else:
        if conditions.stage2_exit_bad:
            stage2_exit = min(
                stage2_exit + 1, config.stage2_exit_dwell_steps
            )
            if stage2_exit >= config.stage2_exit_dwell_steps:
                # Never collapse Stage 2 directly to Stage 0.
                stage = 1
                stage2_enter = 0
                stage2_exit = 0
        else:
            stage2_exit = 0

    target_weight = (
        config.stage0_weight,
        config.stage1_weight,
        config.stage2_weight,
    )[stage]
    current_weight = _slew_velocity_weight(
        float(state.current_weight), target_weight, config
    )
    return (
        VelocityStageState(
            stage=stage,
            current_weight=current_weight,
            target_weight=target_weight,
            stage1_enter_dwell=stage1_enter,
            stage1_exit_dwell=stage1_exit,
            stage2_enter_dwell=stage2_enter,
            stage2_exit_dwell=stage2_exit,
        ),
        conditions,
    )


def constrained_tracking_sigma(
    previous: float,
    candidate: float,
    minimum: float,
    maximum: float,
    *,
    monotonic: bool,
) -> float:
    """Clamp an adaptive reward width and optionally forbid regression-driven widening."""
    if minimum <= 0.0 or maximum < minimum:
        raise ValueError(
            f"invalid sigma bounds: minimum={minimum}, maximum={maximum}"
        )
    bounded = min(max(float(candidate), float(minimum)), float(maximum))
    if monotonic:
        bounded = min(bounded, float(previous))
    return min(max(bounded, float(minimum)), float(maximum))


def advance_velocity_curriculum(
    progress: float,
    dwell: int,
    *,
    gates_ok: bool,
    dwell_steps: int,
    ramp_up_steps: int,
    ramp_down_steps: int,
) -> tuple[float, int]:
    """Advance or retreat a reversible velocity-weight curriculum.

    The curriculum must first observe ``dwell_steps`` consecutive healthy control steps. It then
    ramps toward one while the joint precision/stability gate remains healthy. Any regression
    clears the dwell and ramps back toward zero, preventing a transient early pass from permanently
    enabling the full strike-velocity reward.
    """
    if dwell_steps < 1:
        raise ValueError(f"dwell_steps must be >= 1, got {dwell_steps}")
    if ramp_up_steps < 1:
        raise ValueError(f"ramp_up_steps must be >= 1, got {ramp_up_steps}")
    if ramp_down_steps < 1:
        raise ValueError(f"ramp_down_steps must be >= 1, got {ramp_down_steps}")

    bounded_progress = min(max(float(progress), 0.0), 1.0)
    bounded_dwell = min(max(int(dwell), 0), int(dwell_steps))
    if gates_ok:
        bounded_dwell = min(bounded_dwell + 1, int(dwell_steps))
        if bounded_dwell >= int(dwell_steps):
            bounded_progress = min(
                bounded_progress + 1.0 / float(ramp_up_steps), 1.0
            )
    else:
        bounded_dwell = 0
        bounded_progress = max(
            bounded_progress - 1.0 / float(ramp_down_steps), 0.0
        )
    return bounded_progress, bounded_dwell


def interpolated_velocity_weight(
    bootstrap_weight: float, full_weight: float, progress: float
) -> float:
    """Linearly interpolate the live reward weight from bounded curriculum progress."""
    bounded_progress = min(max(float(progress), 0.0), 1.0)
    return float(bootstrap_weight) + bounded_progress * (
        float(full_weight) - float(bootstrap_weight)
    )


def velocity_curriculum_gate_status(
    *,
    qualified: bool,
    normal_rates: Sequence[float],
    position_rates: Sequence[float],
    normal_min: float,
    position_min: float,
    post_fall_rate: float,
    post_fall_max: float,
    ready_rate: float,
    ready_min: float,
) -> tuple[bool, bool, bool, bool, bool]:
    """Evaluate the precision/recovery gate without an Isaac or torch dependency.

    ``qualified`` means every required clip has accumulated enough exact-strike samples.
    Normal and position must pass for every clip; post-fall and READY are aggregate rates.
    The final item is the conjunction used by the reversible velocity curriculum.
    """
    normal_rates = tuple(float(value) for value in normal_rates)
    position_rates = tuple(float(value) for value in position_rates)
    if len(normal_rates) != len(position_rates):
        raise ValueError(
            "normal_rates and position_rates must have the same length, got "
            f"{len(normal_rates)} and {len(position_rates)}"
        )
    if qualified and not normal_rates:
        raise ValueError("qualified velocity gate requires at least one clip rate")

    normal_ok = bool(
        qualified and all(value >= float(normal_min) for value in normal_rates)
    )
    position_ok = bool(
        qualified and all(value >= float(position_min) for value in position_rates)
    )
    post_fall_ok = bool(float(post_fall_rate) <= float(post_fall_max))
    ready_ok = bool(float(ready_rate) >= float(ready_min))
    all_ok = bool(normal_ok and position_ok and post_fall_ok and ready_ok)
    return normal_ok, position_ok, post_fall_ok, ready_ok, all_ok
