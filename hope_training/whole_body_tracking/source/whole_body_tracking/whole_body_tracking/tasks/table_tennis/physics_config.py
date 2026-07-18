"""Loader for the shared no-spin ball-physics configuration.

Every physical constant used to build the table-tennis scene (ball, air drag, table / net geometry,
bounce materials) is sourced from a single repo-level file, ``configs/ball_physics.yaml``, so that
training, the planner, and the MuJoCo evaluator all agree on one world. This module locates and parses
that file and exposes it as a plain nested ``dict``.

Resolution order for the config file:

1. the ``HOPE_BALL_PHYSICS_CONFIG`` environment variable, if set (absolute path), else
2. the first ``configs/ball_physics.yaml`` found while walking up from this file toward the repo root.

The values below in :data:`DEFAULTS` are the **clean no-spin** subset of the real fitted ball-capture
values and double as documentation of the schema this package reads. Whenever ``configs/ball_physics.yaml``
is present it is authoritative and overrides these defaults key-by-key (a shallow section is deep-merged),
so the file — not this module — is the single source of truth. The defaults only keep the package
importable and unit-testable on their own.

Units: SI (meters, seconds, kilograms, radians). Coordinate frame: +x forward, +y left, +z up
(right-handed), with z = 0 at the table surface. There is **no spin anywhere** in this model: the ball
state is fully described by position and linear velocity.
"""

from __future__ import annotations

import copy
import os
import warnings
from functools import lru_cache

# Environment variable that, if set, points directly at the config file (overrides the search).
CONFIG_ENV_VAR: str = "HOPE_BALL_PHYSICS_CONFIG"
# Path of the config relative to the repository root.
CONFIG_REL_PATH: str = os.path.join("configs", "ball_physics.yaml")


# ---------------------------------------------------------------------------------------------------
# Clean no-spin defaults == the schema this package reads. Values are the real-capture fit; the shipped
# ``configs/ball_physics.yaml`` overrides them and is the single source of truth.
# ---------------------------------------------------------------------------------------------------
DEFAULTS: dict = {
    # Ball (40 mm competition ball).
    "ball": {
        "mass": 0.0034,          # kg
        "radius": 0.020,         # m (40 mm diameter)
    },
    # Air (reference only; the drag coefficient below already folds in air density).
    "air": {
        "density": 1.20,         # kg/m^3
    },
    # Aerodynamic drag. No-spin quadratic model: a_drag = -k * |v| * v (world frame). No Magnus/lift.
    "drag": {
        "k": 0.1261,             # 1/m
        "velocity_clip": 50.0,   # m/s, numerical safety clip on |v| when computing drag
    },
    # Gravity magnitude (acts along -z).
    "gravity": 9.81,             # m/s^2
    # ITTF regulation table geometry.
    "table": {
        "length": 2.74,          # m, along +x
        "width": 1.525,          # m, along y
        "height": 0.76,          # m, playing surface above the floor
        "thickness": 0.05,       # m, collision/visual slab thickness of the table top
    },
    # Net geometry.
    "net": {
        "height": 0.1525,        # m, above the table surface
        "overhang": 0.15,        # m, net extends this far past each table edge in y
        "thickness": 0.01,       # m, slab thickness along x
    },
    # Painted boundary / center line (visual only).
    "lines": {
        "width": 0.02,           # m
        "thickness": 0.001,      # m, sits just above the surface
    },
    # Contact/bounce materials. Restitution values are the effective ball<->surface normal restitution;
    # the ball material is neutral (restitution/friction = 1.0) and combined multiplicatively with each
    # surface, so the surface value is the effective one (see geometry.BounceMaterials). Friction is the
    # PhysX Coulomb coefficient used by the simulator.
    "contact": {
        "ball": {
            "restitution": 1.0,
            "static_friction": 1.0,
            "dynamic_friction": 1.0,
        },
        "table": {
            "restitution": 0.9215,      # measured effective ball<->table normal restitution
            "static_friction": 0.40,
            "dynamic_friction": 0.40,
        },
        "floor": {
            "restitution": 0.40,
            "static_friction": 0.80,
            "dynamic_friction": 0.80,
        },
        "net": {
            "restitution": 0.10,        # low, so the ball dies on a net touch (match play)
            "static_friction": 0.50,
            "dynamic_friction": 0.50,
        },
        "paddle": {
            "restitution": 0.654,       # measured ball<->paddle normal restitution (used by the
                                        # racket-link material, set on the robot asset, and by the
                                        # planner / evaluator; not applied to a scene prim here)
        },
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge ``override`` into a copy of ``base`` (dict-typed values merged, others replaced)."""
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def find_config_path() -> str | None:
    """Return the path to ``configs/ball_physics.yaml`` (env override, else walk up), or ``None``."""
    env_path = os.environ.get(CONFIG_ENV_VAR)
    if env_path:
        return env_path if os.path.isfile(env_path) else None

    here = os.path.abspath(os.path.dirname(__file__))
    prev = None
    while here != prev:
        candidate = os.path.join(here, CONFIG_REL_PATH)
        if os.path.isfile(candidate):
            return candidate
        prev, here = here, os.path.dirname(here)
    return None


@lru_cache(maxsize=1)
def load_ball_physics() -> dict:
    """Load the merged ball-physics config (``configs/ball_physics.yaml`` over :data:`DEFAULTS`), cached.

    Missing file or missing PyYAML falls back to :data:`DEFAULTS` (with a one-time warning), so the
    package stays importable on its own.
    """
    path = find_config_path()
    if path is None:
        warnings.warn(
            f"{CONFIG_REL_PATH} not found (searched parents of the task package and the "
            f"${CONFIG_ENV_VAR} override); using built-in no-spin defaults.",
            stacklevel=2,
        )
        return copy.deepcopy(DEFAULTS)

    try:
        import yaml
    except ImportError:
        warnings.warn(
            f"PyYAML is unavailable; cannot read {path}. Using built-in no-spin defaults.",
            stacklevel=2,
        )
        return copy.deepcopy(DEFAULTS)

    with open(path, "r", encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh) or {}
    if not isinstance(loaded, dict):
        warnings.warn(f"{path} did not parse to a mapping; using built-in no-spin defaults.", stacklevel=2)
        return copy.deepcopy(DEFAULTS)
    return _deep_merge(DEFAULTS, loaded)
