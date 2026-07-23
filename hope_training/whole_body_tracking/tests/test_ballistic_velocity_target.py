"""Tests for ballistic racket target velocities used by the ping-pong command."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import torch


def _load_ballistics():
    root = Path(__file__).resolve().parents[1]
    path = root / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/ballistics.py"
    spec = importlib.util.spec_from_file_location("hope_ballistics_test_module", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


ballistics = _load_ballistics()


def test_ballistic_velocity_lands_on_requested_xy_and_table_height():
    p0 = torch.tensor([[0.55, -0.10, 1.30], [0.80, 0.20, 1.45]], dtype=torch.float32)
    land_xy = torch.tensor([[2.40, 0.10], [2.85, -0.25]], dtype=torch.float32)
    flight_time = torch.tensor([0.60, 0.70], dtype=torch.float32)
    surface_z = 0.76

    vel = ballistics.ballistic_velocity_from_landing(p0, land_xy, flight_time, surface_z)
    landed_xy = p0[:, :2] + vel[:, :2] * flight_time[:, None]
    landed_z = p0[:, 2] + vel[:, 2] * flight_time - 0.5 * ballistics.GRAVITY * flight_time**2

    assert torch.allclose(landed_xy, land_xy, atol=1.0e-5)
    assert torch.allclose(landed_z, torch.full_like(landed_z, surface_z), atol=1.0e-5)


def test_ballistic_velocity_clears_nominal_net_for_mid_table_landings():
    p0 = torch.tensor([[0.60, 0.00, 1.30]], dtype=torch.float32)
    land_xy = torch.tensor([[2.55, 0.00]], dtype=torch.float32)
    flight_time = torch.tensor([0.62], dtype=torch.float32)
    vel = ballistics.ballistic_velocity_from_landing(p0, land_xy, flight_time, 0.76)

    net_top = 0.76 + 0.1525 + 0.02
    assert vel[0, 0] > 0.3
    assert ballistics.ballistic_z_at_x(p0, vel, 0.5 + 1.37)[0] > net_top
