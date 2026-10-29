"""Pure YAML identity and frame conversion for a table landing objective.

The objective contains geometry and target selection only. Reward weights and
switches do not affect its hash, so reward-on/off runs can share the same bank.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import yaml


_FIELDS = {"schema_version", "id", "frame", "units", "x_range", "y_range",
           "surface_z", "target_policy", "boundary"}


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Landing objective {name} must be a finite number")
    return float(value)


def _range(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"Landing objective {name} must contain two bounds")
    bounds = [_number(v, name) for v in value]
    if bounds[0] >= bounds[1]:
        raise ValueError(f"Landing objective {name} must have increasing bounds")
    return bounds


def _canonical(raw, *, descriptor=False):
    if not isinstance(raw, dict):
        raise ValueError("Landing objective must be a mapping")
    allowed = _FIELDS | ({"target_xy"} if descriptor else set())
    if set(raw) != allowed:
        raise ValueError(f"Landing objective fields differ: missing={sorted(allowed-set(raw))}, "
                         f"unknown={sorted(set(raw)-allowed)}")
    expected = {"schema_version": 1, "id": "table_return_sweet_spot_v1",
                "frame": "table_near_left", "units": "m", "surface_z": 0.0,
                "target_policy": "fixed_center", "boundary": "inclusive"}
    if type(raw["schema_version"]) is not int:
        raise ValueError("Landing objective schema_version must be integer 1")
    _number(raw["surface_z"], "surface_z")
    for key, value in expected.items():
        if raw[key] != value:
            raise ValueError(f"Unsupported landing objective {key}: {raw[key]!r}; expected {value!r}")
    xr, yr = _range(raw["x_range"], "x_range"), _range(raw["y_range"], "y_range")
    target = [(xr[0]+xr[1])/2, (yr[0]+yr[1])/2]
    if descriptor:
        supplied = raw["target_xy"]
        if not isinstance(supplied, (list, tuple)) or len(supplied) != 2:
            raise ValueError("Landing objective target_xy must contain two coordinates")
        if [_number(v, "target_xy") for v in supplied] != target:
            raise ValueError("Landing objective target_xy must equal the rectangle centre")
    return dict(schema_version=1, id=expected["id"], frame=expected["frame"], units="m",
                x_range=xr, y_range=yr, surface_z=0.0, target_policy="fixed_center",
                boundary="inclusive", target_xy=target)


def load_objective(path: str | Path) -> dict:
    """Read the canonical table-frame descriptor, deriving its fixed centre."""
    return _canonical(yaml.safe_load(Path(path).expanduser().read_bytes()))


def objective_sha256(path: str | Path) -> str:
    """Hash the exact selected YAML, independently of reward settings."""
    return hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()


def objective_in_training_frame(descriptor: dict, physics_profile: dict) -> dict:
    """Validate the opponent-table region and translate it into env-local XY.

    The returned surface_z is the tabletop, not the ball-centre bounce plane.
    The full physics model independently adds the selected physical ball radius.
    """
    canonical = _canonical(descriptor, descriptor=True)
    table, net, frame = (physics_profile[key] for key in ("table", "net", "training_frame"))
    length, width = (_number(table[key], f"table.{key}") for key in ("length", "width"))
    net_x = _number(net["x_position"], "net.x_position")
    near_x = _number(frame["near_x"], "training_frame.near_x")
    y_center = _number(frame["y_center"], "training_frame.y_center")
    surface = _number(frame["surface_z"], "training_frame.surface_z")
    if length <= 0 or width <= 0 or not 0 < net_x < length:
        raise ValueError("Landing objective requires valid table and net geometry")
    xr, yr = canonical["x_range"], canonical["y_range"]
    if not net_x < xr[0] < xr[1] <= length or not -width <= yr[0] < yr[1] <= 0:
        raise ValueError("Landing objective rectangle must lie on the opponent tabletop")
    y_offset = y_center + width/2
    return {"frame": "training_env_local", "x_range": [x+near_x for x in xr],
            "y_range": [y+y_offset for y in yr],
            "target_xy": [canonical["target_xy"][0]+near_x, canonical["target_xy"][1]+y_offset],
            "surface_z": surface}
