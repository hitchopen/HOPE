"""Recipe 22 binds shared full PPQ004 physics to physical first-bounce labels."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

pytest.importorskip('numba')
ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import refit_stationary_reach_bank as refitter
from ball_physics_profile import load_profile, profile_source_hashes
from full_ball_model import (DT, FLIGHT_TIME, G, KD, KM, RADIUS,
                             first_bounce, full_contact, plan)
PROFILE = REPO / 'configs/ball_physics_ppq004_training.yaml'


def test_training_profile_selects_shared_ppq004_and_full_contact_yaml():
    wrapper = yaml.safe_load(PROFILE.read_text())
    assert 'flight' not in wrapper  # fitted coefficients remain references
    profile = load_profile(PROFILE)
    deployed = yaml.safe_load((REPO / 'hope_ws/src/hope_planner_cpp/config/ball_physics_ppq004_20261020.yaml').read_text())
    contact = yaml.safe_load((REPO / 'hope_ws/src/hope_planner_cpp/config/return_physics.yaml').read_text())['hope_planner']['ros__parameters']
    for node in ('hope_planner', 'hope_ball_flight_packetizer'):
        params = deployed[node]['ros__parameters']
        assert profile['flight']['k_d'] == params['drag_k']
        assert profile['flight']['k_m'] == params['magnus_k']
        for training, planner in (('e_eff', 'restitution_v'), ('a_t', 'table_tangential_gain'),
                                  ('mu_safety', 'table_friction_cap_mu')):
            assert profile['contact']['table'][training] == params[planner]
    for training, shared in (('e_exp_g1', 'restitution_exp_g1'), ('e_exp_g2', 'restitution_exp_g2'),
                             ('a_t', 'paddle_tangential_gain'), ('b_t', 'paddle_angle_gain'),
                             ('mu_safety', 'paddle_friction_cap_mu')):
        assert profile['contact']['paddle'][training] == contact[shared]
    assert profile['return_model']['id'] == 'full_ppq004_first_bounce_v1'
    assert profile['provenance']['model'] == 'PPQ004_UNIFIED'
    assert len(profile_source_hashes(PROFILE)) == 6


def test_reverse_and_outgoing_checks_use_center_contact_plane_not_surface():
    params = refitter.params_from_yaml(PROFILE)
    # Exact straight flight isolates contact-plane selection from model accuracy.
    params[G] = params[KD] = params[KM] = 0.
    x = np.array([.7, -.75, .2])
    up = np.array([-2., 0., 1.])
    xy, valid = refitter.incoming_bounce(x, up, np.zeros(3), params)
    t = (x[2]-params[RADIUS])/up[2]
    assert valid
    np.testing.assert_allclose(xy, x[:2]-up[:2]*t, rtol=0, atol=1e-12)
    down = -up
    bounce, _, time, _, _, valid = first_bounce(x, down, np.zeros(3), params)
    assert valid
    assert bounce[2] == pytest.approx(params[RADIUS], abs=1e-12)
    assert time == pytest.approx(t, abs=1e-12)
    np.testing.assert_allclose(bounce[:2], xy, rtol=0, atol=1e-12)
    profile = load_profile(PROFILE)
    frame = profile['training_frame']
    np.testing.assert_array_equal(refitter.table_offset_from_yaml(PROFILE),
        [frame['near_x'], frame['y_center']+profile['table']['width']/2, frame['surface_z']])


def source_sample(count=128):
    source = ROOT / 'assets/stationary_reach_v4_ppq004_generalization_20261022/train.npz'
    receipt = json.loads(source.with_suffix('.receipt.json').read_text())
    with np.load(source, allow_pickle=False) as bank:
        ids = np.linspace(0, len(bank['clip']) - 1, count, dtype=int)
        a = {key: bank[key][ids] if bank[key].ndim else bank[key] for key in bank.files}
    x = a['target_position_offset'].astype(np.float64)
    x[:, :2] += a['station_xy']
    return a, x, receipt


def test_refitter_uses_full_spin_inverse_and_requalifies_commands_without_relaxing_bounds():
    a, x, receipt = source_sample()
    incoming, spin = a['incoming_velocity'].astype(np.float64), a['incoming_spin'].astype(np.float64)
    target = a['intended_landing_xy'].astype(np.float64)
    params = refitter.params_from_yaml(PROFILE)
    offset = refitter.table_offset_from_yaml(PROFILE)
    velocity, normals, planned, planned_spin, valid = refitter.retarget(x, incoming, spin, target, params, offset)
    for i in (0, 21, 127):
        expected = plan(x[i]-offset, incoming[i], spin[i], target[i]-offset[:2], params[FLIGHT_TIME], params)
        for actual, wanted in zip((velocity[i], normals[i], planned[i], planned_spin[i]), expected[:4]):
            np.testing.assert_allclose(actual, wanted, rtol=1e-10, atol=1e-10)
    a['target_velocity'], a['target_normal'] = velocity.astype(np.float32), normals.astype(np.float32)
    result, outgoing, out_spin = refitter.check(x, incoming, spin, a['target_velocity'].astype(np.float64),
        a['target_normal'].astype(np.float64), target, params, offset)
    good, reasons = refitter.mask_valid(a, result, receipt, valid)
    assert good.sum() >= 75
    assert result[good, 8].max() <= .003
    assert np.all(a['station_xy'] == 0)
    # The stored command's actual full contact is re-evaluated, including spin.
    for i in (0, 21, 127):
        expected = full_contact(incoming[i], a['target_velocity'][i].astype(float),
                                a['target_normal'][i].astype(float), spin[i], params)
        np.testing.assert_allclose(outgoing[i], expected[0], rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(out_spin[i], expected[1], rtol=1e-12, atol=1e-12)
    assert not np.allclose(outgoing, velocity)
    changed, changed_out, changed_spin = refitter.check(x, incoming, -spin, a['target_velocity'].astype(np.float64),
        a['target_normal'].astype(np.float64), target, params, offset)
    assert np.max(np.abs(changed_out-outgoing)) > .1
    assert np.max(np.abs(changed_spin-out_spin)) > 1.
    assert np.max(np.abs(changed[:,:2]-result[:,:2])) > .01
    # Every original bound remains mandatory even on an otherwise accepted row.
    index = np.flatnonzero(good)[0]
    for column, value in ((0, 3.11), (1, .66), (2, .96), (3, 0), (4, .50),
                          (5, .75), (6, 0), (7, 7.01), (8, .004), (0, np.nan)):
        invalid = result.copy()
        invalid[index, column] = value
        assert not refitter.mask_valid(a, invalid, receipt, valid)[0][index]
    outside = {key:value.copy() for key,value in a.items()}
    outside['target_velocity'][index, 0] = 3.0
    assert not refitter.mask_valid(outside, result, receipt, valid)[0][index]
    invalid_plan = valid.copy()
    invalid_plan[index] = False
    assert not refitter.mask_valid(a, result, receipt, invalid_plan)[0][index]


def test_refit_preserves_published_input_provenance_and_split_rows():
    arrays, _, receipt = source_sample()
    assert receipt['contract']['recipe_revision'] == 21
    assert set(np.unique(arrays['generation_kind'])) == {0, 1}
    original = {key: value.copy() for key, value in arrays.items()}
    accepted = np.arange(len(arrays['clip'])) % 3 != 0
    output = refitter.retained_rows(arrays, accepted)
    np.testing.assert_array_equal(output['source_row_index'], np.flatnonzero(accepted))
    np.testing.assert_array_equal(output['input_source_row_index'], original['source_row_index'][accepted])
    for key, value in original.items():
        np.testing.assert_array_equal(arrays[key], value)
        if key != 'source_row_index':
            np.testing.assert_array_equal(output[key], value[accepted] if value.ndim else value)


def test_published_revision21_source_banks_match_receipts_and_disjoint_parents():
    base = ROOT / 'assets/stationary_reach_v4_ppq004_generalization_20261022'
    expected = {'train': 502053, 'dev': 61815, 'test': 62103}
    parents = []
    for split, count in expected.items():
        path = base / f'{split}.npz'
        receipt = json.loads(path.with_suffix('.receipt.json').read_text())
        assert hashlib.sha256(path.read_bytes()).hexdigest() == receipt['bank_sha256']
        assert receipt['rows'] == count
        assert receipt['contract']['recipe_revision'] == 21
        assert receipt['augmentation']['outgoing_landing_plane_z'] == .76
        with np.load(path, allow_pickle=False) as bank:
            assert len(bank['clip']) == count
            assert np.all(bank['station_xy'] == 0)
            parents.append(set(bank['parent_id'].tolist()))
            if split == 'train':
                assert receipt['bank_sha256'] == '02619e747333980a6f1d48aa3a9c7896e15727cf95cd3f51eb387d7687c4b419'
                assert int((bank['generation_kind'] == 0).sum()) == 183274
                assert int((bank['generation_kind'] == 1).sum()) == 318779
    assert not (parents[0] & parents[1] or parents[0] & parents[2] or parents[1] & parents[2])
