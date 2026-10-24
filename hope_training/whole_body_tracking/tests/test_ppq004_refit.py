"""Offline PPQ004 bank retargeting must use the Isaac training ball equations."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import torch
import yaml

from test_virtual_ball_cuda_graph import vb

pytest.importorskip('numba')
ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
spec = importlib.util.spec_from_file_location('ppq004_bank_refitter', ROOT / 'scripts/refit_stationary_reach_bank.py')
refitter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = refitter
spec.loader.exec_module(refitter)
PROFILE = REPO / 'configs/ball_physics_ppq004_training.yaml'


def test_training_profile_matches_ppq004_flight_and_table_and_retains_venue_paddle():
    profile = yaml.safe_load(PROFILE.read_text())
    venue = yaml.safe_load((REPO / 'configs/ball_physics_venue.yaml').read_text())
    deployed = yaml.safe_load((REPO / 'hope_ws/src/hope_planner_cpp/config/ball_physics_ppq004_20261020.yaml').read_text())
    for node in ('hope_planner', 'hope_ball_flight_packetizer'):
        params = deployed[node]['ros__parameters']
        assert profile['flight']['k_d'] == params['drag_k']
        assert profile['flight']['k_m'] == params['magnus_k']
        for training, planner in (('e_eff', 'restitution_v'), ('a_t', 'table_tangential_gain'),
                                  ('mu_safety', 'table_friction_cap_mu')):
            assert profile['contact']['table'][training] == params[planner]
    assert profile['contact']['paddle'] == venue['contact']['paddle']
    assert profile['ball']['inertia_coeff'] == pytest.approx(2 / 3)
    assert profile['provenance']['model'] == 'PPQ004_UNIFIED'


@pytest.mark.parametrize('h', [.005, .001, -.005])
def test_refitter_rk4_matches_training_forward_and_backward(h):
    rng = np.random.default_rng(20261025)
    x = rng.uniform([.44, -.68, .8], [.7, .28, 1.45], (17, 3))
    velocity = rng.uniform(-4, 4, (17, 3))
    spin = rng.uniform(-40, 40, (17, 3))
    params = refitter.params_from_yaml(PROFILE)
    actual = np.array([refitter.step(a, b, c, h, params) for a, b, c in zip(x, velocity, spin)])
    expected = vb.rk4_step(*(torch.from_numpy(v) for v in (x, velocity, spin)), h,
                           vb.load_venue_params(str(PROFILE)))
    np.testing.assert_allclose(actual[:, 0], expected[0].numpy(), rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(actual[:, 1], expected[1].numpy(), rtol=1e-12, atol=1e-12)


def test_refitter_paddle_matches_training_including_approach_orientation():
    rng = np.random.default_rng(5)
    incoming = rng.uniform(-3, 3, (64, 3))
    racket = rng.uniform([1, -.8, .3], [2.9, 1.2, 2], (64, 3))
    incoming[::2, 0] = 5  # Exercise face flipping as well as normal approach.
    spin = rng.uniform(-50, 50, (64, 3))
    params = refitter.params_from_yaml(PROFILE)
    actual = np.array([refitter.paddle(a, b, c, params) for a, b, c in zip(incoming, racket, spin)])
    expected = vb.predict_paddle_contact(torch.from_numpy(incoming), torch.from_numpy(racket),
                                         torch.from_numpy(racket), torch.from_numpy(spin),
                                         vb.load_venue_params(str(PROFILE)))
    np.testing.assert_allclose(actual[:, 0], expected[0].numpy(), rtol=1e-11, atol=1e-11)
    np.testing.assert_allclose(actual[:, 1], expected[1].numpy(), rtol=1e-11, atol=1e-11)


def test_refit_reduces_landing_error_without_relaxing_physical_bounds():
    source = ROOT / 'assets/stationary_reach_v4/train.npz'
    receipt = json.loads(source.with_suffix('.receipt.json').read_text())
    with np.load(source, allow_pickle=False) as bank:
        ids = np.linspace(0, len(bank['clip']) - 1, 64, dtype=int)
        a = {k: bank[k][ids] if bank[k].ndim else bank[k] for k in bank.files}
    x = a['target_position_offset'].astype(np.float64)
    x[:, :2] += a['station_xy']
    incoming = a['incoming_velocity'].astype(np.float64)
    spin = a['incoming_spin'].astype(np.float64)
    velocity = a['target_velocity'].astype(np.float64)
    target = a['intended_landing_xy'].astype(np.float64)
    params = refitter.params_from_yaml(PROFILE)
    boxes = np.array([[receipt['contract']['velocity_boxes'][str(side)][axis] for axis in 'xyz']
                      for side in (0, 1)])
    before = refitter.check(x, incoming, spin, velocity, target, params, .005)
    fitted = refitter.refit(x, incoming, spin, velocity, target, a['clip'], params, boxes).astype(np.float32)
    a['target_velocity'] = fitted
    coarse = refitter.check(x, incoming, spin, fitted.astype(np.float64), target, params, .005)
    fine = refitter.check(x, incoming, spin, fitted.astype(np.float64), target, params, .001)
    coarse_good, _ = refitter.mask_valid(a, coarse, receipt)
    fine_good, _ = refitter.mask_valid(a, fine, receipt, fit_tolerance=.003)
    good = coarse_good & fine_good
    assert good.sum() >= 48
    assert np.quantile(fine[good, 8], .95) < np.quantile(before[good, 8], .95) / 10
    assert fine[good, 8].max() <= .003
    assert np.all(a['station_xy'] == 0)
    for side in (0, 1):
        selected = a['clip'] == side
        assert np.all(fitted[selected] >= boxes[side, :, 0] - 1e-6)
        assert np.all(fitted[selected] <= boxes[side, :, 1] + 1e-6)
    contact = np.array([refitter.paddle(i, r, s, params)
                        for i, r, s in zip(incoming, fitted.astype(np.float64), spin)])
    out, omega = torch.from_numpy(contact[:, 0]), torch.from_numpy(contact[:, 1])
    training = vb.coarse_landing(torch.from_numpy(x), out, omega, vb.load_venue_params(str(PROFILE)),
                                 surface_z=.76, net_x=1.87, h=.005, n_steps=240)
    np.testing.assert_allclose(training['land_xy'].numpy()[good], coarse[good, :2], rtol=1e-10, atol=1e-10)
    np.testing.assert_allclose(training['net_z'].numpy()[good], coarse[good, 2], rtol=1e-10, atol=1e-10)
    # Each exclusion is enforced even for an otherwise qualified row.
    index = np.flatnonzero(good)[0]
    for column, value in ((0, 3.11), (1, .66), (2, .96), (3, 0), (4, .50),
                          (5, .75), (6, 0), (7, 7.01), (8, .004), (0, np.nan)):
        invalid = fine.copy()
        invalid[index, column] = value
        mask, _ = refitter.mask_valid(a, invalid, receipt, fit_tolerance=.003)
        assert not mask[index]
