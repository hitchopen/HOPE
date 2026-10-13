"""Measured foot geometry in the fixed task heading frame (ordered left/right)."""
from __future__ import annotations

import math
import torch


def foot_placement_debt(positions, quaternions, loaded, *, heading=0.0,
                        width_min=0.24, width_max=0.34, width_std=0.05,
                        yaw_margin=0.15, yaw_std=0.25):
    """Return width/yaw costs and measurements; never constrain an airborne foot.

    Signed lateral separation cannot be satisfied by crossing the feet or by
    staggering them fore/aft. Heading refers to table-forward, not drifting pelvis
    yaw. Quaternions are wxyz; A3 ankle-roll links point forward along local +X.
    """
    if not (0 < width_min < width_max and width_std > 0 and yaw_std > 0
            and 0 <= yaw_margin < math.pi):
        raise ValueError("invalid foot placement bands/scales")
    if positions.shape[-2:] != (2, 3) or quaternions.shape != (*positions.shape[:-1], 4):
        raise ValueError("foot geometry requires ordered left/right positions and wxyz quaternions")
    if loaded.shape != positions.shape[:-1] or loaded.dtype != torch.bool:
        raise ValueError("loaded must be an ordered left/right boolean mask")
    delta = positions[..., 0, :2] - positions[..., 1, :2]
    width = -math.sin(heading) * delta[..., 0] + math.cos(heading) * delta[..., 1]
    w, x, y, z = quaternions.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y.square() + z.square()))
    error = torch.atan2(torch.sin(yaw - heading), torch.cos(yaw - heading))

    def huber(value):
        return torch.where(value < 1, 0.5 * value.square(), value - 0.5)

    width_cost = huber((torch.relu(width_min - width) + torch.relu(width - width_max)) / width_std)
    width_cost = width_cost * loaded.all(-1)
    yaw_cost = huber(torch.relu(error.abs() - yaw_margin) / yaw_std) * loaded
    yaw_cost = yaw_cost.sum(-1) / loaded.sum(-1).clamp_min(1)
    return width_cost, yaw_cost, width, error
