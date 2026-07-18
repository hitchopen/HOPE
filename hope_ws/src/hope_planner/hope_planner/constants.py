"""Physical constants and tuning parameters for the HOPE PingPong planner.

Frame convention (matching ``configs/ball_physics.yaml``): +x forward (toward
the opponent), +y left, +z up (right-handed); z = 0 is the table surface and
the world origin is the near-side left table corner, so the table occupies
x in [0, length] and y in [-width, 0]. Ball state is no-spin only:
[x, y, z, vx, vy, vz].

The fitted no-spin ball physics (drag, table/paddle restitution, gravity, ball
geometry, table/net geometry) are read from ``configs/ball_physics.yaml`` when
it is found; see :func:`load_ball_physics`, :func:`load_paddle_params`, and
:func:`load_table_params`. The dataclass defaults below mirror that file and are
used only as a fallback when it is absent, so the pure modules import and run
without any config on disk.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

import numpy as np


@dataclass
class TableParams:
    """Table / net geometry, expressed in the play frame (table at y in [-width, 0])."""

    length: float = 2.74          # m, along +x
    width: float = 1.525          # m, table occupies y in [-width, 0]
    height: float = 0.76          # m, table surface above the floor
    y_max: float = 0.0            # m, table's +y edge in the frame
    net_x: float = 1.37           # m, net plane along x
    net_height: float = 0.1525    # m, net top above the table surface
    net_overhang: float = 0.15    # m, net extends past each table edge in y

    @classmethod
    def from_mapping(cls, data: Dict, y_max: Optional[float] = None) -> "TableParams":
        """Build from a parsed ``ball_physics.yaml`` mapping (missing keys -> defaults)."""
        d = cls()
        if y_max is not None:
            d.y_max = float(y_max)
        geom = data.get("geometry", {}) if isinstance(data, dict) else {}
        geom = geom if isinstance(geom, dict) else {}
        # Prefer top-level `table`/`net` (canonical ball_physics.yaml schema);
        # fall back to the older `geometry.table`/`geometry.net` nesting.
        table = data.get("table", geom.get("table", {})) if isinstance(data, dict) else {}
        net = data.get("net", geom.get("net", {})) if isinstance(data, dict) else {}
        if isinstance(table, dict):
            if table.get("length") is not None:
                d.length = float(table["length"])
            if table.get("width") is not None:
                d.width = float(table["width"])
            if table.get("height") is not None:
                d.height = float(table["height"])
        if isinstance(net, dict):
            if net.get("x_position") is not None:
                d.net_x = float(net["x_position"])
            if net.get("height") is not None:
                d.net_height = float(net["height"])
            if net.get("overhang") is not None:
                d.net_overhang = float(net["overhang"])
        return d


@dataclass
class BallPhysics:
    """No-spin aerodynamic and restitution parameters."""

    k: float = 0.1261             # 1/m, quadratic drag: a = -k |v| v
    C_h: float = 0.631            # tangential retention at a table bounce (= 1 - a_t)
    C_v: float = 0.9215           # normal restitution at a table bounce
    g: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    radius: float = 0.02          # m, ball radius (40 mm diameter)
    mass: float = 0.0027          # kg, ball mass (drag folds mass in; kept for reference)

    @classmethod
    def from_mapping(cls, data: Dict) -> "BallPhysics":
        """Build from a parsed ``ball_physics.yaml`` mapping (missing keys -> defaults)."""
        d = cls()
        if not isinstance(data, dict):
            return d
        ball = data.get("ball", {})
        drag = data.get("drag", data.get("flight", {}))
        contact = data.get("contact", {})
        table = contact.get("table", {}) if isinstance(contact, dict) else {}

        k = _first(drag, ("k_d", "k", "coefficient"))
        if k is not None:
            d.k = float(k)
        gravity = _first(data, ("gravity",)) or _first(drag, ("g", "gravity"))
        if gravity is not None:
            d.g = np.array([0.0, 0.0, -abs(float(gravity))])
        if isinstance(ball, dict):
            if ball.get("radius") is not None:
                d.radius = float(ball["radius"])
            if ball.get("mass") is not None:
                d.mass = float(ball["mass"])
        c_v = _first(table, ("restitution", "e_n", "e_eff", "restitution_normal"))
        if c_v is not None:
            d.C_v = float(c_v)
        c_h = _first(table, ("restitution_tangential", "tangential_retention"))
        if c_h is not None:
            d.C_h = float(c_h)
        else:
            a_t = _first(table, ("tangential_damping", "a_t"))
            if a_t is not None:
                d.C_h = 1.0 - float(a_t)
        return d


@dataclass
class PlannerConfig:
    """Planner tuning parameters."""

    # State estimation
    poly_order: int = 2           # polynomial fit order
    fit_window: int = 31          # number of position samples in the velocity fit
    mocap_hz: float = 300.0       # nominal motion-capture sample rate

    # Trajectory prediction
    dt_integrate: float = 0.001   # integration time step (s)
    max_predict_time: float = 2.0  # forward prediction horizon (s)
    bounce_z_tol: float = 0.005   # z threshold for a point-ball bounce dip (m)
    bounce_center_z_max: float = 0.05  # local-minimum height for a centre-tracked bounce (m)

    # Racket planning
    x_hit: float = 0.0            # fixed virtual hitting-plane x coordinate (m)
    target_land: np.ndarray = field(
        default_factory=lambda: np.array([2.055, -0.7625, 0.02])
    )                             # fixed landing target (opponent-half centre); z = ball radius,
                                  # the CENTROID height at table contact (same convention as the
                                  # bounce planes everywhere else)
    delta_t_flight: float = 0.5   # desired post-strike flight time (s)
    C_r: float = 0.654            # paddle normal restitution
    racket_radius: float = 0.075  # m, paddle radius

    # Simplified paddle tangential contact (used by ball_contact.py)
    paddle_a_t: float = 0.52      # tangential damping fraction
    paddle_b_t: float = 0.0       # impact-angle coupling for the tangential term
    paddle_mu: float = 0.5        # friction-cone cap


def _first(mapping: Dict, keys) -> Optional[float]:
    """Return the first present key's value from a mapping, else None."""
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key]
    return None


def find_ball_physics_config(start: Optional[Path] = None) -> Optional[Path]:
    """Search upward from ``start`` (or this file) for ``configs/ball_physics.yaml``."""
    base = Path(start) if start is not None else Path(__file__).resolve()
    for parent in [base, *base.parents]:
        candidate = parent / "configs" / "ball_physics.yaml"
        if candidate.is_file():
            return candidate
    return None


def _read_physics_yaml(path: Optional[str] = None) -> Dict:
    """Load the ball-physics YAML as a dict; return {} if missing or unreadable."""
    resolved = Path(path) if path else find_ball_physics_config()
    if not resolved or not Path(resolved).is_file():
        return {}
    try:
        import yaml  # lazy import so the pure modules import without PyYAML
        with open(resolved, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def load_ball_physics(path: Optional[str] = None) -> BallPhysics:
    """Load :class:`BallPhysics` from ``configs/ball_physics.yaml`` (or defaults)."""
    return BallPhysics.from_mapping(_read_physics_yaml(path))


def load_table_params(path: Optional[str] = None, y_max: Optional[float] = None) -> TableParams:
    """Load :class:`TableParams` geometry from the physics YAML (or defaults)."""
    return TableParams.from_mapping(_read_physics_yaml(path), y_max=y_max)


def load_paddle_params(path: Optional[str] = None) -> Dict[str, float]:
    """Load paddle restitution / tangential params from the physics YAML.

    Returns a dict with keys ``C_r``, ``paddle_a_t``, ``paddle_b_t``,
    ``paddle_mu``. Missing entries fall back to the :class:`PlannerConfig`
    defaults so callers can splat the result into a config unconditionally.
    """
    defaults = PlannerConfig()
    out = {
        "C_r": defaults.C_r,
        "paddle_a_t": defaults.paddle_a_t,
        "paddle_b_t": defaults.paddle_b_t,
        "paddle_mu": defaults.paddle_mu,
    }
    data = _read_physics_yaml(path)
    contact = data.get("contact", {}) if isinstance(data, dict) else {}
    paddle = contact.get("paddle", {}) if isinstance(contact, dict) else {}
    if isinstance(paddle, dict):
        c_r = _first(paddle, ("restitution", "e_n", "e_eff", "restitution_normal"))
        if c_r is not None:
            out["C_r"] = float(c_r)
        a_t = _first(paddle, ("tangential_damping", "a_t"))
        if a_t is not None:
            out["paddle_a_t"] = float(a_t)
        if paddle.get("b_t") is not None:
            out["paddle_b_t"] = float(paddle["b_t"])
        mu = _first(paddle, ("tangential_cap", "mu"))
        if mu is not None:
            out["paddle_mu"] = float(mu)
    return out
