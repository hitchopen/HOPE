"""Positive task kernels with useful state-score slopes beyond the tight peak."""
import math
import torch


def tracking_kernel(squared_error, std, tail_blend=0.0):
    """Blend a Gaussian peak and rational tail, both maximized at the SAME goal.

    Maximum income stays one; no additional negative reward or auxiliary actor
    gradient is introduced. A nonzero state-score slope is not a guarantee of
    PPO policy-gradient magnitude.
    """
    if not math.isfinite(std) or std <= 0 or not 0 <= tail_blend <= 1:
        raise ValueError("tracking kernel needs std > 0 and tail_blend in [0, 1]")
    scaled = squared_error / std**2
    peak = torch.exp(-scaled)
    if tail_blend == 0:
        return peak
    return (1-tail_blend)*peak + tail_blend/(1+scaled)
