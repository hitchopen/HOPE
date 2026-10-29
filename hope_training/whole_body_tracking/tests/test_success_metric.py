"""Full PPQ004 achieved-contact scoring and first-bounce regression tests."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('success_metric', ROOT / 'source/whole_body_tracking/whole_body_tracking/utils/success_metric.py')
sm = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = sm
spec.loader.exec_module(sm)

@pytest.fixture(scope='module')
def scenario():
    physics = sm.BallPhysics()
    table = sm.TableGeometry.from_config(physics.profile)
    p = np.array([.15, -.45, .32])
    incoming = np.array([-2.2, .18, .18])
    spin = np.array([12., 28., -8.])
    target = np.asarray(physics.profile['return_model']['target_land'])
    from numba import njit
    result = njit(sm._model.plan)(p, incoming, spin, target, .5, physics.return_parameters)
    assert result[6] and result[7]
    return physics, table, p, incoming, spin, target, result


def score(scenario, position=None, physics=None, spin=None):
    original, table, p, incoming, omega, _, result = scenario
    return sm.evaluate_return(p, p if position is None else position, result[0],
                              original if physics is None else physics, table,
                              incoming_ball_velocity=incoming,
                              incoming_ball_spin=omega if spin is None else spin,
                              racket_normal=result[1])


def test_default_profile_and_actual_achieved_contact(scenario):
    physics, table, p, incoming, spin, target, result = scenario
    assert physics.profile['provenance']['model'] == 'PPQ004_UNIFIED'
    outcome = score(scenario)
    assert outcome.success
    np.testing.assert_allclose(outcome.landing_xy, target[:2], atol=1e-4)
    wrong = sm.integrate_outgoing_ball(p, result[0], physics, table, ball_spin=spin)
    assert np.linalg.norm(np.asarray(outcome.landing_xy) - wrong.landing_xy) > .5


def test_achieved_position_and_contact_gate(scenario):
    physics, table, p, _, _, _, result = scenario
    displaced = p + [.035, -.015, .025]
    out = score(scenario, position=displaced)
    expected = sm.integrate_outgoing_ball(displaced, result[2], physics, table, ball_spin=result[3])
    np.testing.assert_allclose(out.landing_xy, expected.landing_xy, atol=1e-12)
    assert not score(scenario, position=p + [.3, 0, 0]).contacted


def test_spin_and_yaml_coefficients_affect_return(scenario):
    physics = scenario[0]
    baseline = score(scenario)
    without_spin = score(scenario, spin=np.zeros(3))
    assert np.linalg.norm(np.asarray(baseline.landing_xy) - without_spin.landing_xy) > .001
    cfg = deepcopy(physics.profile)
    cfg['flight']['k_d'] *= 1.5
    cfg['flight']['k_m'] *= .5
    cfg['contact']['paddle']['a_t'] *= .7
    changed = score(scenario, physics=sm.BallPhysics.from_config(cfg))
    assert np.linalg.norm(np.asarray(changed.landing_xy) - baseline.landing_xy) > .01


def test_radius_plane_matches_analytic_ballistic_limit(scenario):
    cfg = deepcopy(scenario[0].profile)
    cfg['flight'].update(k_d=0., k_m=0.)
    cfg['ball']['radius'] = .08
    physics = sm.BallPhysics.from_config(cfg)
    p, v = np.array([.1, -.7, .3]), np.array([3., 0., 1.])
    t = (v[2] + np.sqrt(v[2]**2 + 2*physics.gravity*(p[2] - physics.ball_radius))) / physics.gravity
    out = sm.integrate_outgoing_ball(p, v, physics, scenario[1], ball_spin=np.zeros(3))
    np.testing.assert_allclose(out.landing_xy, (p + v*t)[:2], atol=2e-6)


def test_short_flight_cannot_cross_net_after_bounce(scenario):
    physics, table = scenario[:2]
    out = sm.integrate_outgoing_ball([.1, -.7, .2], [1., 0., -.2], physics, table, ball_spin=np.zeros(3))
    assert out.landing_xy is not None and not out.net_crossed and not out.on_opponent


@pytest.mark.parametrize('normal', [np.zeros(3), np.array([np.nan, 0, 1])])
def test_invalid_face_normal_rejected(scenario, normal):
    physics, table, p, incoming, spin, _, result = scenario
    with pytest.raises(ValueError, match='achieved racket normal'):
        sm.evaluate_return(p, p, result[0], physics, table,
                           incoming_ball_velocity=incoming, incoming_ball_spin=spin, racket_normal=normal)


def test_required_inputs_and_accumulation(scenario):
    physics, table, p = scenario[:3]
    with pytest.raises(TypeError):
        sm.evaluate_return(p, p, np.zeros(3), physics, table)
    with pytest.raises(ValueError, match='does not clip speed'):
        sm.BallPhysics(velocity_clip=50.)
    acc = sm.SuccessRate()
    acc.add(score(scenario)); acc.add(sm.ReturnOutcome())
    assert acc.as_dict() == {'success_rate': .5}

@pytest.mark.parametrize('bad', [np.array([np.nan, 0., 0.]), np.zeros(2), np.array([0., np.inf, 0.])])
def test_invalid_target_cannot_pass_contact_gate(scenario, bad):
    physics, table, p, incoming, spin, _, result = scenario
    with pytest.raises(ValueError, match='finite 3D contact states'):
        sm.evaluate_return(bad, p, result[0], physics, table,
                           incoming_ball_velocity=incoming, incoming_ball_spin=spin,
                           racket_normal=result[1])
