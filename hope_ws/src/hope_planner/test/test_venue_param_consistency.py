"""Guard against drift between configs/ball_physics_venue.yaml (single source of truth,
loaded at runtime by the TRAINING side virtual_ball.py) and hope_planner/constants.py
(which MIRRORS the values because the deploy planner must not depend on repo-root paths).

If this test fails, someone re-fitted the venue yaml without updating the planner mirror
(or vice versa). Update BOTH, in one commit. Ownership note (franco 2026-07-04): claude
updates the yaml after adjudicated refits and is responsible for keeping this green.
"""

from __future__ import annotations

import pathlib

import pytest

yaml = pytest.importorskip("yaml")

from hope_planner.constants import BallPhysics, PlannerConfig


def _venue_yaml():
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "configs" / "ball_physics_venue.yaml"
        if cand.exists():
            with open(cand) as fh:
                return yaml.safe_load(fh)
    pytest.skip("ball_physics_venue.yaml not found (packaged deploy checkout without repo root)")


def test_flight_constants_match_venue_fit():
    raw = _venue_yaml()
    cfg = PlannerConfig()
    assert cfg.k_m == pytest.approx(float(raw["flight"]["k_m"]), rel=1e-9)
    # drag: the planner predictor's drag constant must equal the venue fit
    assert BallPhysics().k == pytest.approx(float(raw["flight"]["k_d"]), rel=1e-9)


def test_capture_noise_model_matches_venue_fit():
    raw = _venue_yaml()
    cap = raw["capture"]["position_noise"]
    cfg = PlannerConfig()
    assert cfg.sigma_white_m == pytest.approx(float(cap["white_mm"]) / 1000.0, rel=1e-9)
    assert cfg.sigma_ar1_m == pytest.approx(float(cap["ar1_marginal_mm"]) / 1000.0, rel=1e-9)
    assert cfg.ar1_tau_s == pytest.approx(float(cap["ar1_tau_ms"]) / 1000.0, rel=1e-9)


def test_paddle_contact_matches_venue_fit():
    raw = _venue_yaml()
    pad = raw["contact"]["paddle"]
    cfg = PlannerConfig()
    assert cfg.paddle_a_t == pytest.approx(float(pad["a_t"]), rel=1e-9)
    assert cfg.paddle_b_t == pytest.approx(float(pad["b_t"]), rel=1e-9)
    assert cfg.paddle_mu == pytest.approx(float(pad["mu_safety"]), rel=1e-9)
    assert cfg.e_exp_g1 == pytest.approx(float(pad["e_exp_g1"]), rel=1e-9)
    assert cfg.e_exp_g2 == pytest.approx(float(pad["e_exp_g2"]), rel=1e-9)


def test_ball_geometry_matches_venue_fit():
    raw = _venue_yaml()
    cfg = PlannerConfig()
    assert BallPhysics().radius == pytest.approx(float(raw["ball"]["radius"]), rel=1e-9)
    assert BallPhysics().mass == pytest.approx(float(raw["ball"]["mass"]), rel=1e-9)
    assert cfg.ball_inertia_coeff == pytest.approx(float(raw["ball"]["inertia_coeff"]), rel=1e-9)
