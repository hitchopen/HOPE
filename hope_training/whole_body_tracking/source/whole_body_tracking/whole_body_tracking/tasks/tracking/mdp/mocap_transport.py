"""Consecutive packet loss at the control clock, independent of simulator state."""

import math

import torch


def outage_start_probability(active_fraction, short_range_s, long_probability, long_range_s, dt):
    """Convert a target time-in-outage fraction to an idle-tick start probability.

    This is a stress-test distribution, not a measured per-packet percentile.
    For mean duration L ticks, occupancy = p*L / (1+p*(L-1)).
    """
    validate_outage_config(active_fraction, short_range_s, long_probability, long_range_s)
    if not math.isfinite(dt) or dt <= 0 or active_fraction >= 1:
        raise ValueError("outage fraction must be <1 and control dt positive")
    def mean_ticks(bounds):
        return sum(max(1, math.ceil(x / dt - 1e-9)) for x in bounds) / 2
    length = (1-long_probability)*mean_ticks(short_range_s) + long_probability*mean_ticks(long_range_s)
    return active_fraction / (length*(1-active_fraction)+active_fraction)


def validate_outage_config(start_probability, short_range_s, long_probability, long_range_s):
    for probability in (start_probability, long_probability):
        if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError("mocap outage probabilities must be finite and in [0, 1]")
    for bounds in (short_range_s, long_range_s):
        if len(bounds) != 2 or not all(math.isfinite(x) for x in bounds) or not 0 < bounds[0] <= bounds[1]:
            raise ValueError("mocap outage duration requires finite 0 < lo <= hi")


def advance_packet_outage(
    remaining, reset_rows, *, dt, scale, start_probability, short_range_s,
    long_probability, long_range_s,
):
    """Return (remaining_after_this_tick, blocked_now, started_now).

    An N-tick burst blocks exactly N consecutive control ticks. New sessions
    cannot inherit a burst. Scale zero is clean and consumes no random numbers.
    Start probability is per control tick (not per physics substep/receive).
    """
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(scale) or not 0 <= scale <= 1:
        raise ValueError("invalid mocap outage clock or curriculum scale")
    remaining = remaining.clone()
    remaining[reset_rows] = 0
    started = torch.zeros_like(reset_rows)
    if scale <= 0:
        return torch.zeros_like(remaining), started, started.clone()
    eligible = (remaining == 0) & ~reset_rows
    if start_probability > 0:
        started = eligible & (torch.rand(remaining.shape, device=remaining.device) < start_probability * scale)
        indices = started.nonzero(as_tuple=True)[0]
        if indices.numel():
            def ticks(bounds):
                lo, hi = (max(1, int(math.ceil(x / dt - 1e-9))) for x in bounds)
                return torch.randint(lo, hi + 1, (indices.numel(),), device=remaining.device)
            duration = ticks(short_range_s)
            if long_probability > 0:
                long = torch.rand(indices.numel(), device=remaining.device) < long_probability
                duration = torch.where(long, ticks(long_range_s), duration)
            remaining[indices] = duration
    blocked = remaining > 0
    return (remaining - 1).clamp_min(0), blocked, started
