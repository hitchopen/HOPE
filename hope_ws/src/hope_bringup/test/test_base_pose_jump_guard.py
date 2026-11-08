"""Burst arrivals must not turn harmless pose noise into loss of localization."""
import importlib.util
import math
from pathlib import Path

import pytest


@pytest.fixture(params=["bringup"])
def guard_class(request):
    src = Path(__file__).resolve().parents[2]
    path = (src / "hope_bringup/scripts/base_pose_contract.py" if request.param == "bringup"
            else src / "hope_planner/hope_planner/base_pose_contract.py")
    spec = importlib.util.spec_from_file_location("jump_contract_" + request.param, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.BasePoseJumpGuard


def yaw(degrees):
    angle = math.radians(degrees) / 2
    return (math.cos(angle), 0., 0., math.sin(angle))


def test_field_burst_and_much_larger_jitter_remain_valid(guard_class):
    guard = guard_class()
    guard.observe(1_000_000_000, (0., 0., 0.), yaw(0))
    # Observed trigger: 0.0357 mm / 0.0137 deg over about 25 us.
    guard.observe(1_000_025_000, (.0000357, 0., 0.), yaw(.0137))
    # Deliberately much wider than the field noise, still within the budget.
    guard.observe(1_000_050_000, (.009, 0., 0.), yaw(2.9))


def test_normal_continuous_robot_motion_is_not_rejected(guard_class):
    guard = guard_class()
    for i in range(301):
        t = i / 300
        guard.observe(1_000_000_000 + round(t * 1e9), (t, 0., 0.), yaw(60 * t))


@pytest.mark.parametrize("position,angle,reason", [
    ((.1, 0., 0.), 0., "linear jump"),
    ((0., 0., 0.), 20., "angular jump"),
])
def test_real_pose_jump_still_rejected_and_does_not_poison_history(
        guard_class, position, angle, reason):
    guard = guard_class()
    guard.observe(1_000_000_000, (0., 0., 0.), yaw(0))
    with pytest.raises(ValueError, match=reason):
        guard.observe(1_000_025_000, position, yaw(angle))
    guard.observe(1_003_333_333, (.001, 0., 0.), yaw(.2))


@pytest.mark.parametrize("linear,angular", [(.004, 0.), (0., 1.)])
def test_small_steps_cannot_accumulate_unbounded_jump(guard_class, linear, angular):
    guard = guard_class()
    guard.observe(1_000_000_000, (0., 0., 0.), yaw(0))
    with pytest.raises(ValueError, match="jump"):
        for i in range(1, 21):
            guard.observe(1_000_000_000 + i * 25_000,
                          (i * linear, 0., 0.), yaw(i * angular))


def test_duplicate_time_invalid_pose_and_quaternion_remain_rejected(guard_class):
    guard = guard_class()
    guard.observe(1_000_000_000, (0., 0., 0.), yaw(0))
    for stamp, position, quaternion in [
        (1_000_000_000, (0., 0., 0.), yaw(0)),
        (999_999_999, (0., 0., 0.), yaw(0)),
        (1_001_000_000, (float("nan"), 0., 0.), yaw(0)),
        (1_001_000_000, (0., 0., 0.), (0., 0., 0., 0.)),
    ]:
        with pytest.raises(ValueError):
            guard.observe(stamp, position, quaternion)


def test_quaternion_sign_and_explicit_frame_reset(guard_class):
    guard = guard_class()
    guard.observe(1_000_000_000, (0., 0., 0.), yaw(0))
    guard.observe(1_000_025_000, (0., 0., 0.), (-1., 0., 0., 0.))
    guard.reset()
    guard.observe(1_000_050_000, (2., 0., 0.), yaw(180))


@pytest.mark.parametrize("kwargs", [
    {"linear_tolerance_m": -1.}, {"angular_tolerance_rad": float("nan")},
    {"max_linear_speed": float("inf")}, {"window_s": 0.},
])
def test_invalid_guard_parameters_rejected(guard_class, kwargs):
    with pytest.raises(ValueError):
        guard_class(**kwargs)
