"""Unit tests for the planner_imitate fake-planner core (imitate_presets).

Pure-Python (no ROS); verifies the bring-up invariants from the task spec:
forehand y<0, backhand y>0, x~0.40, strike phases 0.36/0.50, command field
format, deterministic staging, safety clamps, and that level-0/estop stand.
"""

import math

import numpy as np

from hope_planner.imitate_presets import (
    BACKHAND_STRIKE_PHASE,
    FOREHAND_STRIKE_PHASE,
    ImitateCommand,
    ImitateConfig,
    ImitatePlanner,
    SafetyLimits,
)


def _first_swing(level, **cfg_kw):
    cfg = ImitateConfig(level=level, **cfg_kw)
    return ImitatePlanner(cfg).step(0.0)


def test_forehand_target_y_is_negative():
    cmd = _first_swing(2)  # level 2 = forehand
    assert cmd.swing_type == "forehand"
    assert cmd.position[1] < 0.0


def test_backhand_target_y_is_positive():
    cmd = _first_swing(4)  # level 4 = backhand
    assert cmd.swing_type == "backhand"
    assert cmd.position[1] > 0.0


def test_x_stays_near_040_by_default():
    for level in (1, 2, 3, 4):
        cmd = _first_swing(level)
        assert abs(cmd.position[0] - 0.40) < 1e-6, f"level {level} x={cmd.position[0]}"


def test_strike_phases_are_036_and_050():
    assert FOREHAND_STRIKE_PHASE == 0.36
    assert BACKHAND_STRIKE_PHASE == 0.50
    assert _first_swing(2).strike_phase == 0.36   # forehand
    assert _first_swing(4).strike_phase == 0.50   # backhand


def test_command_has_required_fields_and_shapes():
    cmd = _first_swing(2)
    assert isinstance(cmd, ImitateCommand)
    for vec in (cmd.position, cmd.velocity, cmd.normal):
        assert np.asarray(vec).shape == (3,)
    assert math.isclose(float(np.linalg.norm(cmd.normal)), 1.0, abs_tol=1e-6)
    # fields the RacketCommand msg / controller need
    for attr in ("swing_type", "frame_id", "time_to_strike", "strike_time", "valid"):
        assert hasattr(cmd, attr)


def test_default_frame_is_base_link():
    assert _first_swing(2).frame_id == "base_link"


def test_level0_is_stand_invalid():
    cmd = _first_swing(0)
    assert cmd.swing_type == "stand"
    assert cmd.valid is False


def test_estop_forces_stand():
    planner = ImitatePlanner(ImitateConfig(level=2))
    planner.set_estop(True)
    cmd = planner.step(0.0)
    assert cmd.valid is False and cmd.swing_type == "stand"


def test_backhand_disabled_stands():
    cmd = _first_swing(4, safety=SafetyLimits(backhand_disabled=True))
    assert cmd.swing_type == "stand"
    assert cmd.valid is False


def test_speed_is_clamped_to_max():
    # request a high normal speed, but clamp to a low max_speed
    cmd = _first_swing(2, normal_speed=10.0, safety=SafetyLimits(max_speed=2.0))
    assert float(np.linalg.norm(cmd.velocity)) <= 2.0 + 1e-6
    assert cmd.clamped is True


def test_normal_target_speed_in_bringup_band():
    cmd = _first_swing(2)  # normal forehand
    assert 2.3 <= float(np.linalg.norm(cmd.velocity)) <= 2.8


def test_time_to_strike_counts_down_to_zero():
    # tts decreases over a swing and approaches 0 just before the strike (elapsed -> period),
    # then wraps to the next swing at elapsed >= period.
    cfg = ImitateConfig(level=2, strike_period_s=2.0)
    planner = ImitatePlanner(cfg)
    tts0 = planner.step(0.0).time_to_strike
    tts_mid = planner.step(1.0).time_to_strike
    tts_late = planner.step(1.99).time_to_strike
    assert tts0 > tts_mid > tts_late
    assert tts_late < 0.05            # ~0 at the strike instant
    assert math.isclose(tts0, 2.0, abs_tol=1e-6)
    # the next step past the period wraps to a fresh swing (tts back up near period)
    assert planner.step(2.01).time_to_strike > 1.9


def test_level5_alternates_forehand_backhand():
    cfg = ImitateConfig(level=5, strike_period_s=1.0)
    planner = ImitatePlanner(cfg)
    s0 = planner.step(0.0).swing_type
    s1 = planner.step(1.01).swing_type   # next swing cycle
    s2 = planner.step(2.02).swing_type
    assert s0 == "forehand"
    assert s1 == "backhand"
    assert s2 == "forehand"


def test_out_of_training_range_flagged_and_clamped():
    # request 10 m/s (> model_15200 training speed 3.5) -> flagged out-of-range AND clamped to max_speed
    cmd = _first_swing(2, normal_speed=10.0)
    assert cmd.out_of_training_range is True
    assert float(np.linalg.norm(cmd.velocity)) <= 3.0 + 1e-6   # clamped to default max_speed
    assert cmd.clamped is True


def test_default_presets_are_in_training_range():
    for level in (1, 2, 3, 4):
        cmd = _first_swing(level)
        assert cmd.out_of_training_range is False, f"level {level} unexpectedly out of range"
