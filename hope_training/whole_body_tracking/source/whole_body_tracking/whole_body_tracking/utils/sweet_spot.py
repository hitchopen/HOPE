"""Pure rectangle reward and event identity checks; no ball-model coefficients."""

import math
import torch


def validate_region(x_range, y_range, scale_m, inside_bonus):
    values = (*x_range, *y_range, scale_m, inside_bonus)
    if (len(x_range) != 2 or len(y_range) != 2
            or not all(math.isfinite(float(v)) for v in values)
            or x_range[0] >= x_range[1] or y_range[0] >= y_range[1]
            or scale_m <= 0 or inside_bonus < 0):
        raise ValueError("sweet-spot bounds, positive distance scale and nonnegative bonus must be finite")


def rectangle_reward(landing_xy, legal, x_range, y_range, scale_m, inside_bonus):
    """Bounded nonnegative shaping, with inclusive rectangle edges.

    Invalid/nonlegal returns earn zero, including nonfinite landing coordinates.
    Distance is to the rectangle itself, so every point inside has equal value.
    """
    finite = torch.isfinite(landing_xy).all(dim=-1)
    xy = torch.where(finite[:, None], landing_xy, torch.zeros_like(landing_xy))
    lo = xy.new_tensor((x_range[0], y_range[0]))
    hi = xy.new_tensor((x_range[1], y_range[1]))
    outside = torch.maximum(lo - xy, xy - hi).clamp_min(0.0)
    distance = torch.linalg.vector_norm(outside, dim=-1)
    inside = finite & (outside == 0).all(dim=-1)
    shape = (1.0 - (distance / scale_m).pow(0.75)).clamp_min(0.0)
    raw = torch.where(legal & finite, shape + inside_bonus * inside, torch.zeros_like(shape))
    return raw, torch.where(finite, distance, torch.zeros_like(distance)), inside


def matching_command_event(event_generation, generation, event_clip, clip,
                           event_frame, frame):
    """A reward event survives only its own following command boundary.

    The motion command advances once before virtual telemetry consumes the
    cache; a held clock may remain unchanged. A reset/clip switch invalidates it.
    """
    return ((event_generation == generation) & (event_clip == clip)
            & ((frame == event_frame) | (frame == event_frame + 1)))
