"""Validate matched datasets, one-shot trigger semantics, scoring and all plot families."""
import copy
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts/flight_prediction_validation'))
from accuracy import METHODS, validate, residuals, write_report


def rows():
    return [dict(session_id='synthetic',ball_id='test',flight_id=flight,event_id='plane',method=method,
                 spin_mode='POSITION_ONLY',information_cutoff_ns=ns,prediction_reference_time_ns=ns,
                 code_sha='0'*40,config_id='synthetic',frame_id='table',clock_mapping_id='synthetic',
                 independent_truth=False,target_valid=True,actual_trigger=True,attempted=True,
                 physical_available=True,position=[0,.01,.02],truth_position=[0,0,0],
                 velocity=[1,2,3],truth_velocity=[1,2,3],arrival_ns=1_001_000_000,truth_arrival_ns=1_000_000_000,
                 b1_ns=700_000_000,b2_ns=900_000_000,compute_ms=.5,reason='ready',event_kind='receiving_plane')
            for method in METHODS for flight in ['one','two'] for ns in [600_000_000,800_000_000]]


def test_matched_information_and_real_truth_cannot_be_fabricated():
    records=rows()
    with pytest.raises(ValueError,match='independent'):
        validate(records)
    validate(records,synthetic=True)
    changed=copy.deepcopy(records);changed[0]['prediction_reference_time_ns']+=1
    with pytest.raises(ValueError,match='future'):
        validate(changed,synthetic=True)
    with pytest.raises(ValueError,match='identical'):
        validate(records[1:],synthetic=True)
    changed=copy.deepcopy(records);changed[0]['actual_trigger']=False
    with pytest.raises(ValueError,match='one-shot'):
        validate(changed,synthetic=True)


def test_error_units_whole_shot_bootstrap_and_three_real_plot_families(tmp_path):
    records=rows()
    error=residuals(records)[0]
    assert error['error_y_m']==.01
    assert error['error_time_ms']==1
    report=write_report(records,tmp_path,synthetic=True,repetitions=10)
    assert report['gate_p']=='NOT_RUN_SYNTHETIC_TOOLING'
    assert report['paired_bootstrap']['groups']==2
    assert report['groups']['legacy_one_shot/POSITION_ONLY']['scored']==4
    assert len(list(tmp_path.glob('*/*.png')))==3
    assert all(p.stat().st_size>1000 for p in tmp_path.glob('*/*.png'))


def test_moving_plane_dataset_is_rejected_before_comparison():
    from compare_recordings import compare
    dataset = dict(frame_id='table', clock_mapping_id='fixture',
                   registration_receipt_sha256='0'*64, code_pins={}, flights=[],
                   plane_policy='FOLLOW_POSE')
    with pytest.raises(ValueError, match='fixed x_hit'):
        compare(dataset, 'not-run', 'not-run', synthetic=True)
