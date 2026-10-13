"""Bounded recovery safe-set math used by RallyV17 recipe revision 3.

This module is deliberately independent of Isaac Lab.  The reward term assembles physical
signals, while the functions below own the two safety-critical pieces of mathematics:

* a velocity-aware stopping-distance margin against the *actual* hard joint limits; and
* a bounded max/top-k aggregation in which one dangerous channel cannot hide in a whole-body
  average.

Every returned violation lies in ``[0, 1)`` for finite plant/control signals.  Consequently a
reward term with weight ``-0.35`` still has an exact worst-case magnitude below ``0.35`` regardless
of the number of safety channels.  Unlike the old hard clip, the rational tail remains monotonic
outside the configured width, so a severely unsafe q_des still receives a corrective gradient.
"""

from __future__ import annotations

import torch


def normalized_upper_violation(
    value: torch.Tensor,
    safe: float | torch.Tensor,
    width: float | torch.Tensor,
) -> torch.Tensor:
    """Return ``excess / (excess + width)`` with fail-closed validation.

    ``width`` is the half-saturation distance: a violation exactly one width above ``safe`` maps
    to ``0.5``.  The value approaches one continuously but never develops the zero-gradient
    plateau created by the previous linear hard clip.
    """

    safe_t = torch.as_tensor(safe, dtype=value.dtype, device=value.device)
    width_t = torch.as_tensor(width, dtype=value.dtype, device=value.device)
    if not bool(torch.isfinite(value).all()):
        raise ValueError("safe-set signal contains NaN/Inf")
    if not bool(torch.isfinite(safe_t).all() and torch.isfinite(width_t).all()):
        raise ValueError("safe-set threshold/width contains NaN/Inf")
    if bool((width_t <= 0.0).any()):
        raise ValueError("safe-set normalization width must be strictly positive")
    excess = (value - safe_t).clamp_min(0.0)
    return (excess / (excess + width_t)).clamp(0.0, 1.0)


def actual_q_stopping_violation(
    q: torch.Tensor,
    qd: torch.Tensor,
    hard_lo: torch.Tensor,
    hard_hi: torch.Tensor,
    brake_accel: torch.Tensor,
    *,
    margin_fraction: float = 0.05,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Velocity-aware hard-limit violation for every environment and action joint.

    For a joint moving toward its upper/lower rail, the required distance is

    ``qd_toward**2 / (2*a_brake) + margin_fraction*(hard_hi-hard_lo)``.

    Both rails retain their static margin even when the joint is stationary or escaping a rail.
    The more violated rail owns the returned diagnostic distances. The normalized
    violation is zero when the available distance is at least the required stopping distance and
    rises linearly over one static margin before saturating at one.
    """

    tensors = {
        "q": q,
        "qd": qd,
        "hard_lo": hard_lo,
        "hard_hi": hard_hi,
        "brake_accel": brake_accel,
    }
    if q.shape != qd.shape:
        raise ValueError(f"q/qd shape mismatch: {tuple(q.shape)} vs {tuple(qd.shape)}")
    for name, value in tensors.items():
        if not torch.is_tensor(value) or not bool(torch.isfinite(value).all()):
            raise ValueError(f"{name} must be a finite tensor")
    try:
        hard_lo, hard_hi, brake_accel = torch.broadcast_tensors(
            hard_lo.to(device=q.device, dtype=q.dtype),
            hard_hi.to(device=q.device, dtype=q.dtype),
            brake_accel.to(device=q.device, dtype=q.dtype),
        )
    except RuntimeError as exc:
        raise ValueError("hard-limit/braking tensors are not broadcastable") from exc
    if not 0.0 < float(margin_fraction) < 0.5:
        raise ValueError("margin_fraction must lie in (0, 0.5)")
    span = hard_hi - hard_lo
    if bool((span <= 0.0).any()):
        raise ValueError("actual-q hard interval must be non-empty")
    if bool((brake_accel <= 0.0).any()):
        raise ValueError("brake_accel must be strictly positive")

    toward_upper_speed = qd.clamp_min(0.0)
    toward_lower_speed = (-qd).clamp_min(0.0)
    upper_distance = hard_hi - q
    lower_distance = q - hard_lo
    static_margin = float(margin_fraction) * span
    upper_stopping = toward_upper_speed.square() / (2.0 * brake_accel) + static_margin
    lower_stopping = toward_lower_speed.square() / (2.0 * brake_accel) + static_margin
    upper_risk = upper_stopping - upper_distance
    lower_risk = lower_stopping - lower_distance
    use_upper = upper_risk >= lower_risk
    distance = torch.where(use_upper, upper_distance, lower_distance)
    stopping_distance = torch.where(use_upper, upper_stopping, lower_stopping)
    violation = ((stopping_distance - distance).clamp_min(0.0) / static_margin).clamp(
        0.0, 1.0
    )
    return violation, stopping_distance, distance


def capture_point_support_excess(
    com_position: torch.Tensor,
    com_velocity: torch.Tensor,
    foot_position: torch.Tensor,
    foot_contact: torch.Tensor,
    *,
    foot_radius: float = 0.06,
    margin: float = 0.02,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Capture-point distance outside the current two-foot support capsule.

    The capsule is the convex hull of equal sole disks. A lifted foot contributes no support;
    single support uses the contacting sole only. This is a conservative flat-ground proxy,
    not a zero-velocity or double-contact objective, and permits corrective stepping.
    Returns excess distance and the no-contact mask. In flight the farther sole is used
    only as a distance diagnostic; the reward adds a separate flight cost. This prevents
    dropping a large distance debt merely by lifting the last loaded foot.
    """
    if foot_radius <= 0.0 or margin < 0.0:
        raise ValueError("support foot_radius must be positive and margin nonnegative")
    if foot_position.shape[-2:] != (2, 3) or foot_contact.shape != foot_position.shape[:-1]:
        raise ValueError("support requires two 3-D feet and a matching contact mask")
    contact = foot_contact.bool()
    flight = ~contact.any(dim=-1)
    both = contact.all(dim=-1)
    # Foot link heights differ from the sole by a small constant; clamp height for stability.
    ground_height = torch.where(contact, foot_position[..., 2], torch.inf).amin(dim=-1)
    ground_height = torch.where(flight, foot_position[..., 2].amin(dim=-1), ground_height)
    height = (com_position[..., 2] - ground_height).clamp_min(0.2)
    capture = com_position[..., :2] + com_velocity[..., :2] * torch.sqrt(height / 9.81).unsqueeze(-1)
    left, right = foot_position[..., 0, :2], foot_position[..., 1, :2]
    first = torch.where((contact[..., 0] | flight)[..., None], left, right)
    second = torch.where((both | flight)[..., None], right, first)
    segment = second - first
    projection = ((capture - first) * segment).sum(dim=-1) / segment.square().sum(dim=-1).clamp_min(1.0e-8)
    closest = first + projection.clamp(0.0, 1.0).unsqueeze(-1) * segment
    distance = torch.linalg.vector_norm(capture - closest, dim=-1)
    flight_distance = torch.maximum(
        torch.linalg.vector_norm(capture - left, dim=-1),
        torch.linalg.vector_norm(capture - right, dim=-1),
    )
    distance = torch.where(flight, flight_distance, distance)
    excess = (distance - float(foot_radius) - float(margin)).clamp_min(0.0)
    return excess, flight


def aggregate_recovery_violations(
    violations: torch.Tensor,
    *,
    topk: int = 3,
    max_blend: float = 0.5,
) -> torch.Tensor:
    """Combine normalized channels as ``blend*max + (1-blend)*mean(top-k)``.

    The function is total for fewer than ``topk`` channels and preserves the invariant
    ``0 <= aggregate <= 1``.  Inputs outside ``[0, 1]`` are rejected rather than silently clipped:
    the individual channel constructors own normalization, so an out-of-range value indicates a
    wiring error.
    """

    if violations.ndim < 2 or violations.shape[-1] < 1:
        raise ValueError("violations must have at least one channel on the final axis")
    if not bool(torch.isfinite(violations).all()):
        raise ValueError("recovery violations contain NaN/Inf")
    if bool(((violations < 0.0) | (violations > 1.0)).any()):
        raise ValueError("recovery violations must lie in [0, 1]")
    if int(topk) < 1:
        raise ValueError("topk must be >= 1")
    if not 0.0 <= float(max_blend) <= 1.0:
        raise ValueError("max_blend must lie in [0, 1]")
    k = min(int(topk), int(violations.shape[-1]))
    largest = torch.topk(violations, k=k, dim=-1).values
    maximum = largest[..., 0]
    top_mean = largest.mean(dim=-1)
    result = float(max_blend) * maximum + (1.0 - float(max_blend)) * top_mean
    return result.clamp(0.0, 1.0)
