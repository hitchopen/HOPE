"""MuJoCo command prediction uses incoming spin and the full racket inverse."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def test_full_command_and_interpolated_incoming_state(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    import full_ball_model as model
    import mujoco_eval_onnx as evaluator
    metric = evaluator._load_success_metric(ROOT.parents[1])
    physics = metric.BallPhysics()
    scene = SimpleNamespace(near_edge_x=.5, table_height=.76, length=2.74, width=1.525,
                            offset=np.array([.5, .7625, .76]))
    args = SimpleNamespace(strike_plane_x=.15)
    p = np.array([.6513, -.2, 1.12]); v = np.array([-3.7, .42, -.9]); w = np.array([3., 24., -10.])
    command = evaluator._predict_command(p, v, -1, scene, physics, args, 17, 3,
                                          SimpleNamespace, ball_spin=w)
    assert command is not None
    dt = physics.return_parameters[model.DT]
    endpoint, velocity = model.rk4_step(p, v, w, dt, physics.return_parameters)
    fraction = (.65-p[0])/(endpoint[0]-p[0])
    expected_position = p+fraction*(endpoint-p)
    incoming = v+fraction*(velocity-v)
    np.testing.assert_allclose(command.position, expected_position, atol=1e-14)
    np.testing.assert_allclose(command.time_to_strike, fraction*dt, atol=1e-14)
    outgoing, spin = model.full_contact(incoming, command.velocity, command.velocity, w, physics.return_parameters)
    bounce, _, _, _, _, valid = model.first_bounce(command.position-scene.offset, outgoing, spin, physics.return_parameters)
    assert valid
    np.testing.assert_allclose(bounce[:2], physics.profile['return_model']['target_land'][:2], atol=1e-4)
    assert np.linalg.norm(outgoing-command.velocity) > 1
    assert (command.task_id, command.task_revision, command.swing_sign) == (17,3,-1)


def test_no_intercept_emits_no_command(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    import mujoco_eval_onnx as evaluator
    metric = evaluator._load_success_metric(ROOT.parents[1])
    scene = SimpleNamespace(near_edge_x=.5, table_height=.76, length=2.74, width=1.525,
                            offset=np.array([.5, .7625, .76]))
    command = evaluator._predict_command([.6, 0., 1.], [-2.,0.,0.], 1, scene,
                metric.BallPhysics(), SimpleNamespace(strike_plane_x=.15), 1, 0,
                SimpleNamespace, ball_spin=np.zeros(3))
    assert command is None
