"""Check denominators, source causality and impact-aware scoring independently."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts' / 'flight_prediction_validation'))
from scoring import score_matched_events, interpolate_without_impact_bridge


def test_rejections_remain_in_coverage_and_missing_truth_is_not_physical_evidence():
    rows = [dict(spin_mode='POSITION_ONLY', target_valid=False, reason='uncertain')]
    assert score_matched_events(rows)['POSITION_ONLY']['valid_coverage'] == 0
    row = dict(spin_mode='MEASURED', target_valid=True, independent_truth=False)
    with pytest.raises(ValueError, match='independent'):
        score_matched_events([row])
    row.update(independent_truth=True, prediction_reference_time_ns=20,
               information_cutoff_ns=19)
    with pytest.raises(ValueError, match='cutoff'):
        score_matched_events([row])


def test_interpolation_never_bridges_a_contact():
    times = np.array([0., .1, .2])
    positions = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
    assert interpolate_without_impact_bridge(times, positions, .05, [.04]) is None
    assert np.allclose(interpolate_without_impact_bridge(times, positions, .15, [.04]), [1.5, 0., 0.])
