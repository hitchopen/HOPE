"""Same-tick strike reductions with one compact device-to-host transfer."""

from __future__ import annotations

import math

import torch


def collect_exact_strike_statistics(
    *,
    exact_strike: torch.Tensor,
    pass_position: torch.Tensor,
    pass_velocity: torch.Tensor,
    pass_normal: torch.Tensor,
    pass_composite: torch.Tensor,
    position_error: torch.Tensor,
    velocity_error: torch.Tensor,
    normal_error_deg: torch.Tensor,
    in_window: torch.Tensor,
    clip_ids: torch.Tensor | None = None,
    clip_keys: tuple[int, ...] = (),
    venue_selected: torch.Tensor | None = None,
    cohort_names: tuple[str, ...] = (),
) -> dict[tuple, float]:
    """Return global/clip/cohort scalar increments, without mutating any accumulator.

    Every reduction uses its original dtype: boolean counts sum to int64 and error sums use
    the input floating dtype. Conversion to float64 occurs *after* summation and is only the
    host transport format, matching the old ``float(tensor.sum())`` Python values. Updating
    the EMAs and evaluating their gates remains the caller's responsibility at the old sites.
    """
    reductions = {
        ("global", "n"): exact_strike.sum(),
        ("global", "comp"): pass_composite.sum(),
        ("global", "pos"): pass_position.sum(),
        ("global", "vel"): pass_velocity.sum(),
        ("global", "5cm"): ((position_error < 0.05) & exact_strike).sum(),
        ("global", "10cm"): ((position_error < 0.10) & exact_strike).sum(),
        ("global", "normal"): pass_normal.sum(),
        ("global", "pos_err"): (position_error * exact_strike).sum(),
        ("global", "vel_err"): (velocity_error * exact_strike).sum(),
        ("global", "window_n"): in_window.sum(),
        ("global", "window_normal_err"): (normal_error_deg * (math.pi / 180.0) * in_window).sum(),
    }
    if clip_ids is not None:
        for clip in clip_keys:
            selected = exact_strike & (clip_ids == clip)
            selected_float = selected.float()
            reductions.update({
                ("clip", clip, "n"): selected.sum(),
                ("clip", clip, "pos"): (pass_position & selected).sum(),
                ("clip", clip, "vel"): (pass_velocity & selected).sum(),
                ("clip", clip, "normal"): (pass_normal & selected).sum(),
                ("clip", clip, "comp"): (pass_composite & selected).sum(),
                ("clip", clip, "pos_err"): (position_error * selected_float).sum(),
                ("clip", clip, "vel_err"): (velocity_error * selected_float).sum(),
                ("clip", clip, "normal_err"): (normal_error_deg * selected_float).sum(),
            })
            for cohort in cohort_names:
                cohort_selected = selected & (venue_selected if cohort == "tuple" else ~venue_selected)
                reductions.update({
                    ("cohort", cohort, clip, "n"): cohort_selected.sum(),
                    ("cohort", cohort, clip, "pos"): (pass_position & cohort_selected).sum(),
                    ("cohort", cohort, clip, "vel"): (pass_velocity & cohort_selected).sum(),
                    ("cohort", cohort, clip, "comp"): (pass_composite & cohort_selected).sum(),
                })

    # Group before conversion, avoiding one cast kernel per scalar and preserving integer
    # counts when an error tensor is float32. All signals belong to this command's device.
    groups = {}
    for key, value in reductions.items():
        groups.setdefault(value.dtype, []).append((key, value))
    keys = []
    packed_groups = []
    for items in groups.values():
        keys.extend(key for key, _ in items)
        packed_groups.append(torch.stack([value for _, value in items]).to(torch.float64))
    host_values = torch.cat(packed_groups).detach().cpu().tolist()
    return dict(zip(keys, host_values))
