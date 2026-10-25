"""The economical training grid resolves cases just inside/outside the target edges."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import torch

from test_online_full_ball_model import venue, vb

ROOT = Path(__file__).resolve().parents[1]


def test_region_edges_keep_full_model_landing_and_net_accuracy():
    numba = pytest.importorskip('numba')
    spec = importlib.util.spec_from_file_location('sweet_objective_parity', ROOT / 'scripts/sweet_spot_profile.py')
    objective = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(objective)
    region = objective.load_objective(ROOT.parents[1] / 'configs/return_placement_sweet_spot.yaml')
    bounds = np.array([region['x_range'], region['y_range']])
    center = bounds.mean(axis=1)
    plan = numba.njit(venue._full.plan)
    bounce = numba.njit(venue._full.first_bounce)
    raw = venue._profile.load_profile(ROOT.parents[1] / 'configs/ball_physics_ppq004_training.yaml')
    params = venue._full.params_from_profile(raw)
    positions, outgoing, spins, expected, expected_net = [], [], [], [], []
    for axis in (0, 1):
        for edge in bounds[axis]:
            for offset in (-.002, .002):
                target = center.copy()
                target[axis] = edge + offset
                for spin in ([20., -35., 15.], [-15., 30., -25.]):
                    position = np.array([.15, -.75, .32])
                    result = plan(position, np.array([-3., .15, -.2]), np.array(spin), target, .5, params)
                    assert result[6] and result[7]
                    fine = bounce(position, result[2], result[3], params)
                    positions.append(position); outgoing.append(result[2]); spins.append(result[3])
                    expected.append(fine[0][:2]); expected_net.append(fine[3][2])
    tensors = [torch.tensor(np.asarray(a), dtype=torch.float32) for a in (positions, outgoing, spins)]
    coarse = vb.coarse_landing(*tensors, vb.load_venue_params(ROOT.parents[1] / 'configs/ball_physics_ppq004_training.yaml'),
                              surface_z=0., net_x=params[venue._full.NET_X], h=.01, n_steps=200)
    assert coarse['land_valid'].all() and coarse['net_valid'].all()
    predicted = coarse['land_xy'].numpy()
    fine_xy = np.asarray(expected)
    assert np.linalg.norm(predicted - fine_xy, axis=1).max() < .001
    assert np.abs(coarse['net_z'].numpy() - expected_net).max() < .001
    inside = lambda xy: ((xy >= bounds[:, 0]) & (xy <= bounds[:, 1])).all(axis=1)
    np.testing.assert_array_equal(inside(predicted), inside(fine_xy))
    assert inside(predicted).any() and (~inside(predicted)).any()
