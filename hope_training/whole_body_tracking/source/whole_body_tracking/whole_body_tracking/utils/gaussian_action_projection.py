"""Differentiable cost of Gaussian action requests discarded by a position clamp."""

from __future__ import annotations

import math

import torch


def gaussian_projection_debt(
    mean: torch.Tensor,
    std: torch.Tensor,
    lower: torch.Tensor,
    upper: torch.Tensor,
) -> torch.Tensor:
    """Return E[(X - clamp(X, lower, upper))**2] for X ~ Normal(mean, std).

    Penalizing only a legal actor mean ignores exploration tails. This exact
    expectation supplies inward mean and decreasing-std gradients even when
    the mean is legal. It does not clip samples or alter their PPO likelihood.
    Inputs use the affine decoder's raw-action units and per-environment bounds.
    """
    variance = torch.square(std)

    def positive_part_second_moment(displacement):
        z = displacement / std
        # erfc avoids the cancellation of 1 + erf(z) in a small lower tail.
        probability = 0.5 * torch.erfc(-z / math.sqrt(2.0))
        density = torch.exp(-0.5 * torch.square(z)) / math.sqrt(2.0 * math.pi)
        moment = (variance + torch.square(displacement)) * probability
        moment = moment + displacement * std * density
        return moment.clamp_min(0.0)

    return positive_part_second_moment(lower - mean) + positive_part_second_moment(mean - upper)
