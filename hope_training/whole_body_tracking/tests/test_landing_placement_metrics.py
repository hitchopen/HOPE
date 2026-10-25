"""Placement rates use all attempts while keeping finite landing errors explicit."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('landing_placement_metrics_test',
    ROOT / 'source/whole_body_tracking/whole_body_tracking/utils/landing_placement_metrics.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def metrics():
    return module.LandingPlacementMetrics([2.24, 2.44], [-.575, -.425])


def outcome(xy=None, *, contacted=True, success=True):
    return NS(landing_xy=xy, contacted=contacted, success=success)


def test_region_boundaries_and_legal_gate():
    m = metrics()
    for corner in ((2.24, -.575), (2.44, -.425)):
        m.add(outcome(corner), 'forehand')
    m.add(outcome((2.34, -.5), success=False), 'forehand')
    m.add(outcome((2.44001, -.5)), 'backhand')
    result = m.as_dict()
    assert result['region_hits'] == 2 and result['attempts'] == 4
    assert result['region_hit_rate'] == .5
    assert result['by_side']['backhand']['region_hits'] == 0
    assert result['by_side']['forehand']['region_distance_m_p50_p95'] == [0., 0.]


def test_misses_falls_and_timeouts_are_not_dropped():
    m = metrics()
    m.add(outcome((2.34, -.5)), 'forehand')
    m.add(outcome(contacted=False, success=False), 'forehand')
    m.interrupt('backhand', fall=True)
    m.interrupt('backhand', fall=False)
    m.post_strike_fall('forehand')
    result = m.as_dict()
    assert result['attempts'] == 4 and result['region_hit_rate'] == .25
    assert result['pre_strike_falls'] == result['post_strike_falls'] == 1
    assert result['interrupted_before_strike'] == 2
    assert result['by_side']['backhand']['region_distance_m_p50_p95'] is None
    assert result['finite_contact_landing_count'] == 1


def test_nonfinite_landings_and_empty_population():
    m = metrics()
    assert m.as_dict()['region_distance_m_p50_p95'] is None
    m.add(outcome((np.nan, 0.), success=False), 'backhand')
    m.add(outcome((np.inf, 0.), success=False), 'backhand')
    result = m.as_dict()
    assert result['attempts'] == 2 and result['region_hits'] == 0
    assert result['region_distance_m_p50_p95'] is None


def test_bounds_and_side_validation():
    with pytest.raises(ValueError, match='bounds'):
        module.LandingPlacementMetrics([1, 0], [0, 1])
    with pytest.raises(ValueError, match='side'):
        metrics().interrupt('unknown', fall=True)


def test_attempt_timing_reset_crossing_wrap_and_post_strike_fall():
    m = metrics()
    tracker = module.PlacementAttemptTracker(m, [.02, .02], ['forehand', 'backhand'])
    # First side strikes; second falls exactly as the clock is reset. Only one hit event.
    event = tracker.advance([0., .5], ['forehand', 'forehand'], [False, True], [False, True])
    assert event.tolist() == [True, False]
    m.add(outcome((2.34, -.5)), 'forehand')
    assert m.as_dict()['attempts'] == 2
    assert m.as_dict()['by_side']['backhand']['pre_strike_falls'] == 1
    assert not tracker.advance([-.02, .48], ['forehand', 'forehand'], [False, False], [False, False]).any()
    # A post-strike fall does not add a duplicate attempt, but is still visible.
    tracker.advance([.5, .46], ['backhand', 'forehand'], [True, False], [True, False])
    assert m.as_dict()['attempts'] == 2 and m.as_dict()['post_strike_falls'] == 1
    assert tracker.advance([-.01, .44], ['backhand', 'forehand'], [False, False], [False, False]).tolist() == [True, False]


def test_partial_initial_swing_is_not_counted_and_timeout_is_not_fall():
    m = metrics()
    tracker = module.PlacementAttemptTracker(m, [-.1], ['backhand'])
    assert not tracker.advance([-.2], ['backhand'], [False], [False]).any()
    tracker.advance([.2], ['forehand'], [False], [False])
    tracker.advance([.5], ['backhand'], [True], [False])
    result = m.as_dict()
    assert result['attempts'] == result['interrupted_before_strike'] == 1
    assert result['pre_strike_falls'] == 0
    assert result['by_side']['forehand']['attempts'] == 1


def test_first_frame_zero_and_reset_to_zero_are_scored_once():
    m = metrics()
    tracker = module.PlacementAttemptTracker(m, [0.], ['forehand'])
    assert tracker.advance([0.], ['forehand'], [False], [False]).tolist() == [True]
    m.add(outcome((2.34, -.5)), 'forehand')
    assert not tracker.advance([0.], ['forehand'], [False], [False]).any()
    # Reset closes the old attempt and arms zero, without scoring the reset pose.
    assert not tracker.advance([0.], ['backhand'], [True], [True]).any()
    assert tracker.advance([0.], ['backhand'], [False], [False]).tolist() == [True]
    m.add(outcome((2.34, -.5)), 'backhand')
    assert not tracker.advance([0.], ['backhand'], [False], [False]).any()
    result = m.as_dict()
    assert result['attempts'] == result['region_hits'] == 2
    assert result['post_strike_falls'] == 1
