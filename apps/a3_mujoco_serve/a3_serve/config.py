"""Strict JSON configuration loading for the serve workflow."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .constants import (
    APP_ROOT,
    DEFAULT_MODEL_XML,
    VALIDATED_FRAME_COUNT,
    VALIDATED_READY_FRAME,
    VALIDATED_SOURCE_HZ,
    VALIDATED_STRIKE_FRAME,
    VALIDATED_STROKE_START_FRAME,
)


class ConfigError(ValueError):
    """Raised when a workflow configuration violates the application contract."""


def _require(mapping: dict[str, Any], key: str, kind: type) -> Any:
    value = mapping.get(key)
    if not isinstance(value, kind):
        raise ConfigError(f"{key!r} must be {kind.__name__}")
    return value


def _resolve(config_path: Path, raw: str) -> Path:
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (config_path.parent / path).resolve()


def load_config(path: str | Path) -> dict[str, Any]:
    """Load and validate a serve JSON file, resolving its file references."""

    config_path = Path(path).expanduser().resolve()
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"cannot load configuration {config_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ConfigError("configuration root must be an object")
    if payload.get("schema_version") != 1:
        raise ConfigError("schema_version must be 1")

    model = _require(payload, "model", dict)
    model.setdefault("xml", str(DEFAULT_MODEL_XML))
    model["xml"] = str(_resolve(config_path, str(model["xml"])))
    if model.get("racket_site") != "right_racket":
        raise ConfigError("model.racket_site must be 'right_racket' for this A3 asset")
    if model.get("racket_geom") != "right_racket_collision":
        raise ConfigError(
            "model.racket_geom must be 'right_racket_collision' for this A3 asset"
        )
    if model.get("racket_normal_axis") not in (0, 1, 2):
        raise ConfigError("model.racket_normal_axis must be 0, 1, or 2")

    source = _require(payload, "source", dict)
    source["template_csv"] = str(
        _resolve(config_path, str(_require(source, "template_csv", str)))
    )
    timing = _require(payload, "timing", dict)
    expected = {
        "source_hz": VALIDATED_SOURCE_HZ,
        "frame_count": VALIDATED_FRAME_COUNT,
        "ready_frame": VALIDATED_READY_FRAME,
        "stroke_start_frame": VALIDATED_STROKE_START_FRAME,
        "strike_frame": VALIDATED_STRIKE_FRAME,
    }
    for key, required in expected.items():
        if timing.get(key) != required:
            raise ConfigError(
                f"timing.{key} must remain {required!r} for the validated runtime contract"
            )
    if not (
        0 < timing["ready_frame"] < timing["stroke_start_frame"]
        < timing["strike_frame"] < timing["frame_count"]
    ):
        raise ConfigError("timing frames are not strictly ordered")

    physics = _require(payload, "physics", dict)
    physics["source_reference"] = str(
        _resolve(
            config_path,
            str(_require(physics, "source_reference", str)),
        )
    )
    if not Path(physics["source_reference"]).is_file():
        raise ConfigError(
            "physics.source_reference does not exist: "
            f"{physics['source_reference']}"
        )
    for section in ("ball", "drag", "contact", "table", "net"):
        _require(physics, section, dict)
    planner = _require(payload, "planner", dict)
    for vector in (
        "incoming_ball_velocity_world",
        "first_bounce_target_table",
        "second_bounce_target_table",
    ):
        value = planner.get(vector)
        if not isinstance(value, list) or len(value) != 3:
            raise ConfigError(f"planner.{vector} must contain three numbers")
        planner[vector] = [float(item) for item in value]
    for grid in ("speed_m_s", "elevation_deg", "azimuth_deg"):
        value = planner.get(grid)
        if not isinstance(value, list) or not value:
            raise ConfigError(f"planner.{grid} must be a non-empty list")
        planner[grid] = [float(item) for item in value]

    ik = _require(payload, "ik", dict)
    if float(ik.get("damping", 0.0)) <= 0.0:
        raise ConfigError("ik.damping must be positive")
    if int(ik.get("max_iterations", 0)) <= 0:
        raise ConfigError("ik.max_iterations must be positive")

    payload["_config_path"] = str(config_path)
    return payload


def default_config_path() -> Path:
    return APP_ROOT / "config/serve.json"
