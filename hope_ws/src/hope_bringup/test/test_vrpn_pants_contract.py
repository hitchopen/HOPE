from pathlib import Path
import csv
import json
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from vrpn_pants_contract import Selection, load_profiles, normalize_pose, observation, update_selection, validate_observation, validate_vrpn_setup
from p1_marker_cad_registration_impl import V2_CAD_MARKERS_PELVIS_M


def test_assets_preserve_ground_truth_pelvis_origin():
    profiles = load_profiles()
    assert profiles['v2']['marker_positions_m'] == [list(p) for p in V2_CAD_MARKERS_PELVIS_M.values()]
    repo = Path(__file__).resolve().parents[4]
    csv_path = repo / 'agibot/mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv'
    rows = list(csv.DictReader(csv_path.open()))
    assert profiles['v4']['marker_positions_m'] == [[float(r[k]) for k in ('x_m','y_m','z_m')] for r in rows]
    for spec in profiles.values():
        # A nonzero centroid proves we did not recenter a pelvis-frame model.
        assert abs(sum(p[2] for p in spec['marker_positions_m']) / spec['marker_count']) > .05
        assert spec['marker_to_pelvis_xyz_m'] == [0, 0, 0]


def test_atomic_selection_requires_stopped_and_monotonic_revision():
    current = Selection('v2','P1',1)
    changed = dict(pants_profile='v4', table_side='P2', revision=2, state='STOPPED', busy=False)
    assert update_selection(current, changed) == Selection('v4','P2',2)
    for bad in ({**changed, 'revision':1}, {**changed,'state':'RUNNING'},
                {**changed,'busy':True}, {**changed,'pants_profile':'v3'}):
        with pytest.raises(ValueError):
            update_selection(current,bad)
    assert update_selection(current, dict(changed, pants_profile='v2',table_side='P1', revision=1, state='RUNNING')) == current


def test_mm_and_p2_conversion_apply_once_to_ball_and_robot():
    selected=Selection('v4','P2',7)
    robot,q=normalize_pose([2100,-1100,200],[0,0,0,1],.001,selected)
    ball,_=normalize_pose([2000,-1000,500],[0,0,0,1],.001,selected)
    assert robot == pytest.approx([.64,-.425,.2])
    assert ball == pytest.approx([.74,-.525,.5])
    assert q == pytest.approx([0,0,1,0])
    envelope=observation(selected,load_profiles(),(100,2),robot,q)
    sel,p,outq=validate_observation(envelope,load_profiles())
    assert sel==selected and p==robot and outq==q
    envelope['pants_profile']='v2'
    with pytest.raises(ValueError,match='mismatch'):
        validate_observation(envelope,load_profiles())


@pytest.mark.parametrize('position,q', [([float('nan'),0,0],[0,0,0,1]),([0,0,0],[0,0,0,0])])
def test_nonfinite_or_invalid_pose_is_rejected(position,q):
    with pytest.raises(ValueError):
        normalize_pose(position,q,.001,Selection())


def test_selected_source_endpoint_and_units_must_match_running_receiver():
    snapshot = dict(mocap_source="vrpn", vrpn_ip="192.168.100.99", vrpn_port="3883", vrpn_units="mm")
    validate_vrpn_setup(snapshot, server="192.168.100.99", port=3883, scale=.001)
    for changed in ({"mocap_source": "motive"}, {"mocap_source": "none"},
                    {"vrpn_ip": "192.168.100.98"}, {"vrpn_port": "3884"},
                    {"vrpn_units": "m"}, {"vrpn_units": "cm"}):
        with pytest.raises(ValueError):
            validate_vrpn_setup({**snapshot, **changed}, server="192.168.100.99", port=3883, scale=.001)
