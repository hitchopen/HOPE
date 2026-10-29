"""Load training physics from YAML, including the runtime planner's selected profile.

This module is pure Python: importing it never registers or starts a simulator.
The PPQ004 wrapper references reviewed YAML files instead of copying fitted values.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import os
from pathlib import Path

import yaml


DEFAULT_PROFILE = "configs/ball_physics_ppq004_training.yaml"


def default_profile_path() -> Path:
    selected = os.environ.get("HOPE_BALL_PHYSICS_YAML") or os.environ.get("HOPE_BALL_PHYSICS_CONFIG")
    if selected:
        return Path(selected).expanduser().resolve(strict=True)
    for parent in Path(__file__).resolve().parents:
        candidate = parent / DEFAULT_PROFILE
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"Cannot find {DEFAULT_PROFILE}; set HOPE_BALL_PHYSICS_YAML")


def _merge(base: dict, overlay: dict) -> dict:
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _read(path: Path, files: dict[str, str]) -> dict:
    path = path.resolve(strict=True)
    content = path.read_bytes()
    files[str(path)] = hashlib.sha256(content).hexdigest()
    document = yaml.safe_load(content)
    if not isinstance(document, dict):
        raise ValueError(f"Physics YAML must contain a mapping: {path}")
    return document


def _load(path: Path, files: dict[str, str], active: set[Path]) -> dict:
    path = path.resolve(strict=True)
    if path in active:
        raise ValueError(f"Cyclic physics YAML reference: {path}")
    active.add(path)
    try:
        raw = _read(path, files)
        if "extends" in raw:
            raw = _merge(_load(path.parent / raw["extends"], files, active), raw)
        if "geometry_yaml" in raw:
            geometry = _read(path.parent / raw["geometry_yaml"], files)
            raw = _merge(geometry, raw)
            for key in ("table", "net", "lines"):
                raw[key] = deepcopy(geometry[key])
        if "planner_profile_yaml" in raw:
            planner = _read(path.parent / raw["planner_profile_yaml"], files)
            params = planner["hope_planner"]["ros__parameters"]
            if params != planner["hope_ball_flight_packetizer"]["ros__parameters"]:
                raise ValueError("Planner and packetizer profile coefficients differ")
            raw["flight"].update(k_d=params["drag_k"], k_m=params["magnus_k"])
            raw["contact"]["table"].update(
                e_eff=params["restitution_v"], a_t=params["table_tangential_gain"],
                mu_safety=params["table_friction_cap_mu"])
        if "planner_settings_yaml" in raw:
            settings = _read(path.parent / raw["planner_settings_yaml"], files)["hope_planner"]["ros__parameters"]
            contract = _read(path.parent / raw["return_physics_yaml"], files)["hope_planner"]["ros__parameters"]
            settings = _merge(settings, contract)
            raw["flight"]["g"] = settings["gravity_mps2"]
            raw["flight"]["rk4_h"] = settings["return_dt_integrate"]
            raw["ball"].update(radius=settings["ball_radius"], inertia_coeff=settings["ball_inertia_coeff"])
            raw["contact"]["paddle"].update(
                e_eff=settings["restitution_racket"], e_exp_g1=settings["restitution_exp_g1"],
                e_exp_g2=settings["restitution_exp_g2"], a_t=settings["paddle_tangential_gain"],
                b_t=settings["paddle_angle_gain"], mu_safety=settings["paddle_friction_cap_mu"])
            model = raw["return_model"]
            mapping = {
                "integrate_dt_s": "return_dt_integrate", "max_predict_time_s": "max_predict_time",
                "flight_time_s": "delta_t_flight", "restitution_racket": "restitution_racket",
                "restitution_min": "return_restitution_min", "restitution_max": "return_restitution_max",
                "newton_iterations": "return_newton_iterations", "fd_relative_step": "return_fd_relative_step",
                "position_tolerance_m": "return_position_tolerance_m", "time_tolerance_s": "return_time_tolerance_s",
                "max_step_mps": "return_max_step_mps", "line_search_iterations": "return_line_search_iterations",
                "seed_iterations": "return_seed_iterations", "numeric_epsilon": "return_numeric_epsilon",
                "net_clearance_margin": "return_net_clearance_margin",
                "alternative_flight_times": "return_alternative_flight_times",
            }
            model.update({key: settings[source] for key, source in mapping.items()})
            model["target_land"] = [settings["target_land_x"], settings["target_land_y"], 0.0]
            raw["training_frame"]["surface_z"] = raw["table"]["height"]
        if "geometry_yaml" in raw:
            # Legacy scene consumers use these aliases. They receive the same
            # selected values, never the historical coefficients from geometry.
            raw["gravity"] = raw["flight"]["g"]
            raw["drag"]["k"] = raw["flight"]["k_d"]
            for name in ("table", "paddle"):
                contact = raw["contact"][name]
                contact["restitution"] = contact["e_eff"]
                contact["tangential_damping"] = contact["a_t"]
                contact["tangential_cap"] = contact["mu_safety"]
            if raw.get("return_model", {}).get("id") == "full_ppq004_first_bounce_v1":
                raw["drag"]["velocity_clip"] = None
        for key in ("extends", "geometry_yaml", "planner_profile_yaml", "planner_settings_yaml", "return_physics_yaml"):
            raw.pop(key, None)
        return raw
    finally:
        active.remove(path)


def load_profile(path: str | Path | None = None) -> dict:
    """Expand the selected YAML profile; missing files/keys never fall back to constants."""
    selected = Path(path).expanduser() if path is not None else default_profile_path()
    return _load(selected, {}, set())


def profile_source_hashes(path: str | Path | None = None) -> dict[str, str]:
    """Hash the wrapper and every referenced YAML for refit/launch provenance."""
    selected = Path(path).expanduser() if path is not None else default_profile_path()
    files: dict[str, str] = {}
    _load(selected, files, set())
    return dict(sorted(files.items()))
