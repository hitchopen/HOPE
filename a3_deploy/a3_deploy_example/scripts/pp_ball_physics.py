#!/usr/bin/env python3
"""Resolve Gate3 ball coefficients from the same ROS YAMLs as its planner."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex

import yaml


KEYS = {
    "drag_k": ("PP_BALL_DRAG_K", 10.0),
    "magnus_k": ("PP_BALL_MAGNUS_K", 1.0),
    "table_friction_cap_mu": ("PP_BALL_TABLE_FRICTION_CAP_MU", 10.0),
    "spin_window_s": ("PP_SPIN_WINDOW_S", 1.0),
    "restitution_h": ("PP_BALL_RESTITUTION_H", 1.0),
    "restitution_v": ("PP_BALL_RESTITUTION_V", 1.0),
    "table_tangential_gain": ("PP_BALL_OBSERVED_BOUNCE_GAIN", 1.0),
    "post_net_future_bounce_tangential_gain": ("PP_BALL_FUTURE_BOUNCE_GAIN", 1.0),
    "post_net_future_bounce_restitution_v": ("PP_BALL_FUTURE_BOUNCE_RESTITUTION_V", 1.0),
}

CONTACT_DEFAULTS = {
    "future_contact_spin_enabled": False,
    "future_contact_magnus_enabled": False,
    "future_contact_spin_tangential_gain": 0.38744443855065164,
    "future_contact_spin_omega_scale": 1.0,
    "future_contact_spin_min_coherence": 0.4,
}


def resolve(paths: list[Path]) -> dict:
    parameters: dict = {}
    sources = []
    for path in paths:
        data = path.read_bytes()
        document = yaml.safe_load(data)
        # Same node name and file ordering as the production launchers.
        for node in ("/**", "hope_planner", "/hope_planner"):
            parameters.update(document.get(node, {}).get("ros__parameters", {}))
        sources.append({"path": str(path.resolve()), "sha256": hashlib.sha256(data).hexdigest()})
    parameters.setdefault("post_net_future_bounce_restitution_v", parameters["restitution_v"])
    parameters.setdefault("table_tangential_gain", 0.369)
    parameters.setdefault("magnus_k", 0.00379260504480258)
    parameters.setdefault("table_friction_cap_mu", 0.16706204822993975)
    parameters.setdefault("spin_window_s", 0.18)
    coefficients = {}
    for key, (_, maximum) in KEYS.items():
        raw = parameters[key]
        if isinstance(raw, bool):
            raise ValueError(f"{key} must be numeric")
        value = float(raw)
        if not math.isfinite(value) or not 0.0 <= value <= maximum:
            raise ValueError(f"invalid {key}: {raw!r}")
        coefficients[key] = value
    # MuJoCo now uses the same slip/friction-cap and angular impulse law.
    # restitution_h remains a legacy zero-spin, inactive-cap approximation.
    horizontal = 1.0 - coefficients["post_net_future_bounce_tangential_gain"]
    vertical = coefficients["post_net_future_bounce_restitution_v"]
    comparison = {
        "scope": "capped table-slip model with world spin; not a hardware accuracy verdict",
        "planner_horizontal_retention": horizontal,
        "simulator_horizontal_retention": 1.0 - coefficients["table_tangential_gain"],
        "planner_vertical_restitution": vertical,
        "simulator_vertical_restitution": coefficients["restitution_v"],
        "matched": math.isclose(horizontal, 1.0 - coefficients["table_tangential_gain"], abs_tol=1e-9)
        and math.isclose(vertical, coefficients["restitution_v"], abs_tol=1e-9),
    }
    contact = {key: parameters.get(key, default) for key, default in CONTACT_DEFAULTS.items()}
    booleans = {"future_contact_spin_enabled", "future_contact_magnus_enabled"}
    for key in booleans:
        if not isinstance(contact[key], bool):
            raise ValueError(f"{key} must be boolean")
    for key in CONTACT_DEFAULTS.keys() - booleans:
        value = contact[key]
        if isinstance(value, bool) or not math.isfinite(float(value)) or not 0 <= float(value) <= 1:
            raise ValueError(f"invalid {key}: {value!r}")
        contact[key] = float(value)
    comparison["matched"] = comparison["matched"] and contact["future_contact_magnus_enabled"] and (
        not contact["future_contact_spin_enabled"] or (
            math.isclose(contact["future_contact_spin_tangential_gain"], coefficients["table_tangential_gain"], abs_tol=1e-9)
            and math.isclose(contact["future_contact_spin_omega_scale"], 1.0, abs_tol=1e-9)))
    return {"schema_version": 2, "sources": sources, "parameters": coefficients,
            "contact_prediction": contact,
            "prediction_vs_simulated_contact": comparison,
            "scope": "drag, Magnus and capped spin table contact; paddle parameters unchanged"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params-file", type=Path, action="append", required=True)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--shell", action="store_true")
    args = parser.parse_args()
    receipt = resolve(args.params_file)
    if args.receipt:
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        args.receipt.write_text(json.dumps(receipt, indent=2) + "\n")
    if args.shell:
        for key, (env, _) in KEYS.items():
            print(f"export {env}={shlex.quote(str(receipt['parameters'][key]))}")
        for key, value in receipt["contact_prediction"].items():
            print(f"export PP_{key.upper()}={shlex.quote(str(value).lower())}")
    else:
        print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
