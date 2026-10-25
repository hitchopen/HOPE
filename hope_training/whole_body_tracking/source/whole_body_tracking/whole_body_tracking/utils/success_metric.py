# Copyright (c) 2025, Intelligent Racing Inc. (dba Hitch Interactive).
# SPDX-License-Identifier: Apache-2.0
"""Analytic return scoring under the YAML-selected runtime planner model.

The frame is the planner's table-surface frame: x forward, y in [-width, 0],
z=0 at the playing surface. A return is evaluated from the incoming ball,
achieved racket velocity and achieved face normal. Racket velocity is never
substituted for outgoing ball velocity. The full paddle impulse transfers spin; RK4 flight includes gravity, drag and Magnus.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import importlib.util
from pathlib import Path
import sys

import numpy as np

_SCRIPTS = Path(__file__).resolve().parents[4] / 'scripts'


def _script(name):
    key = f'hope_metric_{name}'
    if key not in sys.modules:
        spec = importlib.util.spec_from_file_location(key, _SCRIPTS / f'{name}.py')
        module = importlib.util.module_from_spec(spec)
        sys.modules[key] = module
        spec.loader.exec_module(module)
    return sys.modules[key]


_profile = _script('ball_physics_profile')
_model = _script('full_ball_model')


def load_ball_physics_config() -> dict:
    """Return the expanded PPQ004 profile, honoring the shared environment override."""
    return _profile.load_profile()


@dataclass
class BallPhysics:
    """YAML-derived return parameters; explicit overrides support controlled tests."""
    gravity: float | None = None
    drag_k: float | None = None
    velocity_clip: float | None = None
    ball_radius: float | None = None
    profile: dict | None = field(default=None, repr=False)

    def __post_init__(self):
        self.profile = deepcopy(self.profile if self.profile is not None else load_ball_physics_config())
        if self.profile.get('return_model', {}).get('id') != 'full_ppq004_first_bounce_v1':
            raise ValueError('Analytic return scoring requires a full_ppq004_first_bounce_v1 YAML profile')
        if self.gravity is None:
            self.gravity = float(self.profile['flight']['g'])
        if self.drag_k is None:
            self.drag_k = float(self.profile['flight']['k_d'])
        if self.ball_radius is None:
            self.ball_radius = float(self.profile['ball']['radius'])
        if self.velocity_clip is not None:
            raise ValueError('Runtime planner return flight does not clip speed')
        self.profile['flight']['g'] = self.gravity
        self.profile['flight']['k_d'] = self.drag_k
        self.profile['ball']['radius'] = self.ball_radius
        self.return_parameters = _model.params_from_profile(self.profile)

    @classmethod
    def from_config(cls, cfg: dict | None = None) -> 'BallPhysics':
        return cls(profile=cfg if cfg is not None else load_ball_physics_config())

    def acceleration(self, velocity: np.ndarray, spin: np.ndarray | None = None) -> np.ndarray:
        return _model.flight_accel(np.asarray(velocity), np.zeros(3) if spin is None else
                                   np.asarray(spin), self.return_parameters)


@dataclass
class TableGeometry:
    """Bounds in the planner table frame, loaded from the selected YAML."""
    length: float | None = None
    width: float | None = None
    net_height: float | None = None
    net_position: float | None = None
    net_overhang: float | None = None

    def __post_init__(self):
        if all(value is not None for value in (self.length, self.width, self.net_height,
                                               self.net_position, self.net_overhang)):
            return
        cfg = load_ball_physics_config()
        if self.length is None:
            self.length = float(cfg['table']['length'])
        if self.width is None:
            self.width = float(cfg['table']['width'])
        if self.net_height is None:
            self.net_height = float(cfg['net']['height'])
        if self.net_position is None:
            self.net_position = float(cfg['net']['x_position'])
        if self.net_overhang is None:
            self.net_overhang = float(cfg['net']['overhang'])

    @property
    def net_x(self) -> float:
        return self.net_position

    @classmethod
    def from_config(cls, cfg: dict | None = None) -> 'TableGeometry':
        cfg = cfg if cfg is not None else load_ball_physics_config()
        return cls(length=float(cfg['table']['length']), width=float(cfg['table']['width']),
                   net_height=float(cfg['net']['height']), net_position=float(cfg['net']['x_position']),
                   net_overhang=float(cfg['net']['overhang']))

    def on_opponent_half(self, x: float, y: float) -> bool:
        return (self.net_x < x <= self.length) and (-self.width <= y <= 0)


@dataclass
class ReturnOutcome:
    contacted: bool = False
    net_crossed: bool = False
    net_clear: bool = False
    on_opponent: bool = False
    success: bool = False
    landing_xy: tuple[float, float] | None = None


def integrate_outgoing_ball(start_pos_w, ball_vel_w, physics, table, dt=None, max_time=None,
                            *, ball_spin):
    """Full PPQ004 RK4 flight to first descending center crossing at ball radius."""
    params = physics.return_parameters.copy()
    params[_model.NET_X] = table.net_x
    params[_model.NET_HEIGHT] = table.net_height
    params[_model.WIDTH] = table.width
    params[_model.LENGTH] = table.length
    params[_model.OVERHANG] = table.net_overhang
    if dt is not None:
        params[_model.DT] = float(dt)
    if max_time is not None:
        params[_model.HORIZON] = float(max_time)
    if params[_model.DT] <= 0 or params[_model.HORIZON] <= 0:
        raise ValueError('Flight step and horizon must be positive')
    bounce, _, _, net, crossed, landed = _model.first_bounce(
        np.asarray(start_pos_w, dtype=np.float64), np.asarray(ball_vel_w, dtype=np.float64),
        np.asarray(ball_spin, dtype=np.float64), params)
    clears, _ = _model.net_clearance(net, crossed, params)
    return ReturnOutcome(net_crossed=bool(crossed), net_clear=bool(clears),
                         on_opponent=bool(landed and table.on_opponent_half(*bounce[:2])),
                         landing_xy=tuple(map(float, bounce[:2])) if landed else None)


def evaluate_return(target_pos_w, achieved_racket_pos_w, racket_vel_w, physics, table,
                    contact_radius=0.10, *, incoming_ball_velocity, incoming_ball_spin,
                    racket_normal):
    """Score achieved paddle contact, spin transfer and the full first-bounce flight."""
    target = np.asarray(target_pos_w, dtype=np.float64)
    achieved = np.asarray(achieved_racket_pos_w, dtype=np.float64)
    incoming = np.asarray(incoming_ball_velocity, dtype=np.float64)
    spin = np.asarray(incoming_ball_spin, dtype=np.float64)
    racket = np.asarray(racket_vel_w, dtype=np.float64)
    if (not np.isfinite(contact_radius) or contact_radius < 0
            or any(value.shape != (3,) or not np.isfinite(value).all()
                   for value in (target, achieved, incoming, spin, racket))):
        raise ValueError('Scoring requires finite 3D contact states and a nonnegative contact radius')
    if np.linalg.norm(achieved - target) > contact_radius:
        return ReturnOutcome()
    normal = np.asarray(racket_normal, dtype=np.float64)
    norm = np.linalg.norm(normal)
    if normal.shape != (3,) or not np.isfinite(norm) or norm <= 0:
        raise ValueError('Scoring requires a finite nonzero achieved racket normal')
    outgoing, spin = _model.full_contact(np.asarray(incoming_ball_velocity, dtype=np.float64),
                                        np.asarray(racket_vel_w, dtype=np.float64), normal,
                                        np.asarray(incoming_ball_spin, dtype=np.float64),
                                        physics.return_parameters)
    outcome = integrate_outgoing_ball(achieved, outgoing, physics, table, ball_spin=spin)
    outcome.contacted = True
    outcome.success = outcome.net_clear and outcome.on_opponent
    return outcome


class SuccessRate:
    """Accumulate successful returns over strike attempts."""
    def __init__(self):
        self.attempts = 0
        self.successes = 0

    def add(self, outcome):
        self.attempts += 1
        self.successes += int(outcome.success)

    def add_bool(self, success):
        self.attempts += 1
        self.successes += int(success)

    @property
    def value(self):
        return self.successes / self.attempts if self.attempts else 0.0

    def as_dict(self):
        return {'success_rate': self.value}
