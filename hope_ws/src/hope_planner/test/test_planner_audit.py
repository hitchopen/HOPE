"""Planner failure/timing telemetry must describe, never control, solves."""

import numpy as np

from hope_planner.ball_kalman_estimator import BallKalmanEstimator
from hope_planner.constants import PlannerConfig
from hope_planner.planner import HOPEPlanner


def _planner() -> HOPEPlanner:
    return HOPEPlanner(
        config=PlannerConfig(
            x_hit=0.0,
            fit_window=8,
            fit_window_max_span_s=1.0,
        )
    )


def test_audit_reports_estimator_not_ready():
    planner = _planner()
    assert planner.update(0.0, np.array([1.2, -0.7, 0.8])) is None
    assert planner.audit.reason == "estimator_not_ready"
    assert planner.audit.stage1_ms >= 0.0
    assert planner.audit.solve_total_ms >= planner.audit.stage1_ms


def test_audit_reports_not_incoming():
    planner = _planner()
    command = None
    for index in range(8):
        t = index * 0.01
        command = planner.update(t, np.array([1.0 + t, -0.7, 0.8]))
    assert command is None
    assert planner.audit.reason == "not_incoming"
    assert planner.audit.stage2_ms == 0.0
    assert planner.audit.stage3_ms == 0.0


def test_audit_timings_do_not_change_valid_command():
    planner = _planner()
    command = None
    for index in range(8):
        t = index * 0.01
        command = planner.update(t, np.array([1.0 - 2.0 * t, -0.7, 0.9]))
    assert command is not None and command.valid
    assert planner.audit.reason == "command_valid"
    assert planner.audit.stage1_ms > 0.0
    assert planner.audit.stage2_ms > 0.0
    assert planner.audit.stage3_ms > 0.0
    assert planner.audit.solve_total_ms >= (
        planner.audit.stage1_ms
        + planner.audit.stage2_ms
        + planner.audit.stage3_ms
    )


def test_use_kalman_selects_active_physics_estimator():
    planner = HOPEPlanner(config=PlannerConfig(use_kalman=True))
    assert isinstance(planner.estimator, BallKalmanEstimator)
