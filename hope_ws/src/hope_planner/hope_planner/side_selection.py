"""Forehand/backhand selection: a binary split on the predicted lateral crossing.

The implemented convention (documented in ``docs/PLANNER_INTERFACE.md`` and the
planner YAML) is:

    crossing_y <  swing_side_split_y  -> FOREHAND (+1)
    crossing_y >= swing_side_split_y  -> BACKHAND (-1)

i.e. a ball arriving BELOW the split (toward the robot's paddle side, -y) is taken
forehand; a ball at or above the split is taken backhand.

With a nonzero hysteresis band the previous task's side is sticky near the split:

    previous FOREHAND: stay FOREHAND unless crossing_y >  split + hysteresis
    previous BACKHAND: stay BACKHAND unless crossing_y <  split - hysteresis

so a ball arriving near the boundary does not flip the choice between consecutive
rallies. Selection happens once per ``task_id`` and is locked for that task.
"""

from __future__ import annotations

FOREHAND: int = 1
BACKHAND: int = -1


def select_swing_side(
    crossing_y: float,
    split_y: float,
    hysteresis_y: float = 0.0,
    prev_side: int = 0,
) -> int:
    """Return ``FOREHAND`` (+1) or ``BACKHAND`` (-1) for a predicted lateral crossing.

    ``prev_side`` is the side of the previous task (0 = none yet); it only matters
    inside the hysteresis band around ``split_y``.
    """
    hysteresis_y = max(0.0, float(hysteresis_y))
    lo = split_y - hysteresis_y
    hi = split_y + hysteresis_y
    if prev_side == FOREHAND:
        return BACKHAND if crossing_y > hi else FOREHAND
    if prev_side == BACKHAND:
        return FOREHAND if crossing_y < lo else BACKHAND
    return FOREHAND if crossing_y < split_y else BACKHAND
