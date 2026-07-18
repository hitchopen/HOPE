"""Boundary-case tests pinning the forehand/backhand selection contract.

Finding: docs/PLANNER_INTERFACE.md stated ``crossing_y >= split -> FOREHAND`` while the
implementation selects FOREHAND below the split. These tests make the implemented
convention explicit (and the doc now matches):

    crossing_y <  split -> FOREHAND (+1)
    crossing_y >= split -> BACKHAND (-1)

plus the hysteresis stickiness relative to the previous task's side.
"""

import pytest

from hope_planner.side_selection import BACKHAND, FOREHAND, select_swing_side

SPLIT = -0.7625
EPS = 1e-9


# --- The documented mapping table (no previous side, no hysteresis) -----------------
@pytest.mark.parametrize(
    "crossing_y,expected",
    [
        (SPLIT - 0.30, FOREHAND),   # well below the split -> forehand
        (SPLIT - EPS, FOREHAND),    # just below the split -> forehand
        (SPLIT, BACKHAND),          # exactly AT the split -> backhand (>= rule)
        (SPLIT + EPS, BACKHAND),    # just above the split -> backhand
        (SPLIT + 0.30, BACKHAND),   # well above the split -> backhand
    ],
)
def test_fresh_selection_boundary_table(crossing_y, expected):
    assert select_swing_side(crossing_y, SPLIT) == expected


def test_matches_message_constants():
    # The int8 constants in RacketCommand.msg are FOREHAND=1 / BACKHAND=-1.
    assert FOREHAND == 1
    assert BACKHAND == -1


# --- Hysteresis: the previous side is sticky inside the band ------------------------
H = 0.04


@pytest.mark.parametrize(
    "prev,crossing_y,expected",
    [
        # Previous forehand: stays forehand up to and including split + H.
        (FOREHAND, SPLIT, FOREHAND),
        (FOREHAND, SPLIT + H, FOREHAND),
        (FOREHAND, SPLIT + H + EPS, BACKHAND),
        # Previous backhand: stays backhand down to and including split - H.
        (BACKHAND, SPLIT, BACKHAND),
        (BACKHAND, SPLIT - H, BACKHAND),
        (BACKHAND, SPLIT - H - EPS, FOREHAND),
    ],
)
def test_hysteresis_stickiness(prev, crossing_y, expected):
    assert select_swing_side(crossing_y, SPLIT, hysteresis_y=H, prev_side=prev) == expected


def test_zero_hysteresis_previous_side_only_breaks_exact_tie():
    # With h = 0 the previous side only matters exactly AT the split.
    assert select_swing_side(SPLIT, SPLIT, 0.0, prev_side=FOREHAND) == FOREHAND
    assert select_swing_side(SPLIT, SPLIT, 0.0, prev_side=BACKHAND) == BACKHAND
    assert select_swing_side(SPLIT + EPS, SPLIT, 0.0, prev_side=FOREHAND) == BACKHAND
    assert select_swing_side(SPLIT - EPS, SPLIT, 0.0, prev_side=BACKHAND) == FOREHAND


def test_negative_hysteresis_is_clamped_to_zero():
    assert select_swing_side(SPLIT + EPS, SPLIT, hysteresis_y=-1.0, prev_side=FOREHAND) == BACKHAND
