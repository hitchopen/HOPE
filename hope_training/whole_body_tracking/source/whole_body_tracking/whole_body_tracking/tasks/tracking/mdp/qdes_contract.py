"""Backend-neutral math for the stateful ``feasible_qdes_v3`` decoder.

The function in this module deliberately uses only the small NumPy/PyTorch API
intersection (``maximum``, ``minimum``, ``where`` and ``tanh``).  The live
Isaac action term calls it with ``torch``; host-only property and C++ parity
tests call the exact same function with ``numpy``.

Fallback levels are part of the deploy contract:

0. full safe + torque + tracking + rate intersection;
1. drop only the q_des rate constraint;
2. additionally drop tracking, retaining safe + torque;
3. safe position interval only.

The position-safe interval is never dropped.  Its validity and all input
finiteness are fail-closed preconditions owned by the caller.
"""

from __future__ import annotations

from typing import NamedTuple


class FeasibleQdesV3Result(NamedTuple):
    qdes: object
    selected_lo: object
    selected_hi: object
    rate_lo: object
    rate_hi: object
    tracking_lo: object
    tracking_hi: object
    torque_lo: object
    torque_hi: object
    safe_torque_lo: object
    safe_torque_hi: object
    base_lo: object
    base_hi: object
    full_lo: object
    full_hi: object
    full_feasible: object
    base_feasible: object
    safe_torque_feasible: object
    fallback_level: object
    tightest_constraint: object
    normalized_action: object
    actual_q_guard_active: object
    actual_q_guard_upper: object
    actual_q_guard_lower: object


def feasible_qdes_v3_dynamic_inputs_finite(
    xp, raw_action, previous_qdes, q, qd
):
    """Return the elementwise finite mask enforced before decoding."""

    return (
        xp.isfinite(raw_action)
        & xp.isfinite(previous_qdes)
        & xp.isfinite(q)
        & xp.isfinite(qd)
    )


def feasible_qdes_v3(
    xp,
    raw_action,
    previous_qdes,
    q,
    qd,
    safe_lo,
    safe_hi,
    rate_limit,
    tracking_limit,
    kp,
    kd,
    effort_limit,
    torque_headroom,
    dt,
    actual_q_guard_enabled=False,
    actual_q_guard_horizon_s=None,
) -> FeasibleQdesV3Result:
    """Decode raw actions inside the selected feasible interval.

    ``tightest_constraint`` is encoded as safe=0, rate=1, tracking=2,
    torque=3.  If multiple bounds are active, the innermost intersection wins
    in that order (rate, then tracking, then torque), matching the existing
    runtime diagnostics.
    """

    rate_delta = rate_limit * dt
    rate_lo = previous_qdes - rate_delta
    rate_hi = previous_qdes + rate_delta
    tracking_lo = q - tracking_limit
    tracking_hi = q + tracking_limit
    torque_limit = effort_limit * torque_headroom
    torque_lo = q + (kd * qd - torque_limit) / kp
    torque_hi = q + (kd * qd + torque_limit) / kp

    safe_torque_lo = xp.maximum(safe_lo, torque_lo)
    safe_torque_hi = xp.minimum(safe_hi, torque_hi)
    safe_torque_feasible = safe_torque_lo <= safe_torque_hi
    base_lo = xp.maximum(safe_torque_lo, tracking_lo)
    base_hi = xp.minimum(safe_torque_hi, tracking_hi)
    base_feasible = base_lo <= base_hi
    full_lo = xp.maximum(base_lo, rate_lo)
    full_hi = xp.minimum(base_hi, rate_hi)
    full_feasible = full_lo <= full_hi

    selected_lo = xp.where(
        full_feasible,
        full_lo,
        xp.where(
            base_feasible,
            base_lo,
            xp.where(safe_torque_feasible, safe_torque_lo, safe_lo),
        ),
    )
    selected_hi = xp.where(
        full_feasible,
        full_hi,
        xp.where(
            base_feasible,
            base_hi,
            xp.where(safe_torque_feasible, safe_torque_hi, safe_hi),
        ),
    )
    fallback_level = xp.where(
        full_feasible,
        0,
        xp.where(base_feasible, 1, xp.where(safe_torque_feasible, 2, 3)),
    )

    # Predictive actual-q brake.  A safe q_des does not by itself guarantee a
    # hard-safe measured q under gravity/coupling: the production V15 smoke
    # observed waist joints reach their physical stops with a neutral actor.
    # If the configured Euler horizon predicts that q will cross its q_des safe
    # boundary (or it is already outside), collapse the executable interval
    # onto the strongest inward target that still respects safe position and
    # PD-effort headroom.  Rate and tracking deliberately yield (fallback 2)
    # before the never-dropped safe q_des interval.
    guard_horizon_s = (
        dt if actual_q_guard_horizon_s is None else actual_q_guard_horizon_s
    )
    predicted_q = q + qd * guard_horizon_s
    actual_q_guard_upper = actual_q_guard_enabled & (
        (q >= safe_hi) | ((qd > 0.0) & (predicted_q >= safe_hi))
    )
    actual_q_guard_lower = actual_q_guard_enabled & (
        (q <= safe_lo) | ((qd < 0.0) & (predicted_q <= safe_lo))
    )
    actual_q_guard_active = actual_q_guard_upper | actual_q_guard_lower
    emergency_lower = xp.where(
        safe_torque_feasible, safe_torque_lo, safe_lo
    )
    emergency_upper = xp.where(
        safe_torque_feasible, safe_torque_hi, safe_hi
    )
    emergency_target = xp.where(
        actual_q_guard_upper, emergency_lower, emergency_upper
    )
    selected_lo = xp.where(
        actual_q_guard_active, emergency_target, selected_lo
    )
    selected_hi = xp.where(
        actual_q_guard_active, emergency_target, selected_hi
    )
    fallback_level = xp.where(
        actual_q_guard_active,
        xp.where(
            safe_torque_feasible,
            fallback_level * 0 + 2,
            fallback_level * 0 + 3,
        ),
        fallback_level,
    )

    anchor = xp.minimum(xp.maximum(previous_qdes, selected_lo), selected_hi)
    normalized_action = xp.tanh(raw_action)
    qdes = anchor + xp.where(
        normalized_action >= 0.0,
        normalized_action * (selected_hi - anchor),
        normalized_action * (anchor - selected_lo),
    )

    eps = 1.0e-7
    torque_tight = (torque_lo > safe_lo + eps) | (torque_hi < safe_hi - eps)
    tracking_tight = (tracking_lo > safe_torque_lo + eps) | (
        tracking_hi < safe_torque_hi - eps
    )
    rate_tight = full_feasible & (
        (rate_lo > base_lo + eps) | (rate_hi < base_hi - eps)
    )
    zeros = fallback_level * 0
    tightest_constraint = xp.where(torque_tight, zeros + 3, zeros)
    tightest_constraint = xp.where(
        tracking_tight, zeros + 2, tightest_constraint
    )
    tightest_constraint = xp.where(rate_tight, zeros + 1, tightest_constraint)
    tightest_constraint = xp.where(
        actual_q_guard_active,
        xp.where(
            safe_torque_feasible,
            zeros + 3,
            zeros,
        ),
        tightest_constraint,
    )

    return FeasibleQdesV3Result(
        qdes=qdes,
        selected_lo=selected_lo,
        selected_hi=selected_hi,
        rate_lo=rate_lo,
        rate_hi=rate_hi,
        tracking_lo=tracking_lo,
        tracking_hi=tracking_hi,
        torque_lo=torque_lo,
        torque_hi=torque_hi,
        safe_torque_lo=safe_torque_lo,
        safe_torque_hi=safe_torque_hi,
        base_lo=base_lo,
        base_hi=base_hi,
        full_lo=full_lo,
        full_hi=full_hi,
        full_feasible=full_feasible,
        base_feasible=base_feasible,
        safe_torque_feasible=safe_torque_feasible,
        fallback_level=fallback_level,
        tightest_constraint=tightest_constraint,
        normalized_action=normalized_action,
        actual_q_guard_active=actual_q_guard_active,
        actual_q_guard_upper=actual_q_guard_upper,
        actual_q_guard_lower=actual_q_guard_lower,
    )
