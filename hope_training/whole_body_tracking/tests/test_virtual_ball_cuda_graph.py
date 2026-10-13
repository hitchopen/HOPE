"""CUDA graph must preserve the existing landing physics exactly."""
import importlib.util
import os
from pathlib import Path
import sys

import pytest
import torch

PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/virtual_ball.py"
spec = importlib.util.spec_from_file_location("hope_virtual_ball_graph_test", PATH)
vb = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = vb
spec.loader.exec_module(vb)


def inputs(n, device):
    generator = torch.Generator(device=device).manual_seed(712)
    p = torch.rand(n, 3, device=device, generator=generator)
    p[:, 0] -= .5
    p[:, 2] += .7
    v = torch.rand(n, 3, device=device, generator=generator) * 12 - 6
    omega = torch.rand(n, 3, device=device, generator=generator) * 180 - 90
    return p, v, omega


def same(actual, expected):
    assert actual.keys() == expected.keys()
    for key in actual:
        torch.testing.assert_close(actual[key], expected[key], rtol=0, atol=0, equal_nan=True)


def test_cpu_uses_original_predictor():
    predictor = vb.CoarseLandingPredictor()
    args = (*inputs(17, "cpu"), vb.load_venue_params(), .76, 1.37)
    same(predictor(*args), vb.coarse_landing(*args))
    assert predictor._graph is None


@pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA parity needs GPU")
def test_cuda_replay_parameter_changes_and_output_ownership():
    device = os.environ.get("HOPE_TEST_CUDA_DEVICE", "cuda:0")
    predictor = vb.CoarseLandingPredictor()
    prm = vb.load_venue_params()
    # Full training batch, different inputs, and rebuilds after shape/horizon changes.
    with torch.inference_mode():
        for n, steps in ((6144, 100), (6144, 100), (31, 100), (31, 80)):
            p, v, w = inputs(n, device)
            original = predictor(p, v, w, prm, .76, 1.37, n_steps=steps)
            expected = vb.coarse_landing(p, v, w, prm, .76, 1.37, n_steps=steps)
            same(original, expected)
            v.mul_(-.5)
            changed = predictor(p, v, w, prm, .76, 1.37, n_steps=steps)
            same(changed, vb.coarse_landing(p, v, w, prm, .76, 1.37, n_steps=steps))
            same(original, expected)
