"""Stable base-X selection for the operator-triggered fixed hit plane."""

from dataclasses import dataclass
import math
from statistics import median
from typing import Iterable, Tuple


@dataclass(frozen=True)
class StableBaseX:
    """Summary of the recent base samples accepted for an x-hit freeze."""

    x_m: float
    samples: int
    span_m: float
    newest_age_s: float


def select_stable_base_x(
    samples: Iterable[Tuple[float, float]],
    *,
    now_s: float,
    window_s: float,
    max_age_s: float,
    min_samples: int,
    max_span_m: float,
) -> StableBaseX:
    """Return the median X of a fresh, sufficiently stable receipt-time window.

    ``samples`` contains ``(receipt_time_s, corrected_base_x_m)`` pairs.  A
    ``ValueError`` is deliberately operator-readable because its text is
    returned directly by the ROS Trigger service.
    """

    if not math.isfinite(now_s):
        raise ValueError("node clock is non-finite")
    if not math.isfinite(window_s) or window_s <= 0.0:
        raise ValueError("calibration window must be > 0 s")
    if not math.isfinite(max_age_s) or max_age_s <= 0.0:
        raise ValueError("maximum sample age must be > 0 s")
    if min_samples < 1:
        raise ValueError("minimum sample count must be >= 1")
    if not math.isfinite(max_span_m) or max_span_m < 0.0:
        raise ValueError("maximum X span must be >= 0 m")

    cutoff_s = now_s - window_s
    recent = [
        (float(receipt_s), float(x_m))
        for receipt_s, x_m in samples
        if math.isfinite(receipt_s) and receipt_s >= cutoff_s
    ]
    if len(recent) < min_samples:
        raise ValueError(
            f"only {len(recent)} recent base samples; need at least {min_samples}"
        )

    recent.sort(key=lambda item: item[0])
    newest_age_s = max(0.0, now_s - recent[-1][0])
    if newest_age_s > max_age_s:
        raise ValueError(
            f"newest base sample is stale ({newest_age_s:.3f} s > {max_age_s:.3f} s)"
        )

    values = [x_m for _, x_m in recent]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("recent base X contains a non-finite value")
    span_m = max(values) - min(values)
    if span_m > max_span_m:
        raise ValueError(
            f"base X is not settled (span {span_m:.4f} m > {max_span_m:.4f} m)"
        )

    return StableBaseX(
        x_m=float(median(values)),
        samples=len(values),
        span_m=float(span_m),
        newest_age_s=float(newest_age_s),
    )
