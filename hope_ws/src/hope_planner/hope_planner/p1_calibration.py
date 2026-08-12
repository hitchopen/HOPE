"""Load the approved P1 -> pelvis_link calibration receipt."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from .base_pose_contract import receipt_id_u52


@dataclass(frozen=True)
class P1Calibration:
    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    receipt_sha256: str
    receipt_id_u52: int


def _finite_vector(value: object, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must be a JSON array of length {length}")
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} contains a non-number") from exc
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{field} contains a non-finite value")
    return result


def load_p1_calibration(path: Path) -> P1Calibration:
    """Read either supported receipt schema and derive its wire receipt ID."""

    encoded = path.read_bytes()
    document = json.loads(encoded.decode("utf-8"))
    if not isinstance(document, dict):
        raise ValueError("P1 calibration document must be a JSON object")
    if document.get("approved") is not True:
        raise ValueError("P1 calibration receipt is not approved")

    try:
        if "p1_to_pelvis" in document:
            transform = document["p1_to_pelvis"]
            translation_value = transform["translation_m"]
        else:
            transform = document["p1_to_pelvis_link"]
            translation_value = transform["xyz_m"]
        parent = transform["parent_frame"]
        child = transform["child_frame"]
        translation = _finite_vector(translation_value, 3, "translation_m")
        quaternion = _finite_vector(
            transform["quaternion_xyzw"], 4, "quaternion_xyzw"
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(f"invalid P1 calibration document: missing {exc}") from exc

    if not isinstance(parent, str) or not parent:
        raise ValueError("parent_frame must be a non-empty string")
    if not isinstance(child, str) or not child:
        raise ValueError("child_frame must be a non-empty string")
    if parent == child:
        raise ValueError("parent_frame and child_frame must differ")

    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 0.5 or norm > 1.5:
        raise ValueError("quaternion_xyzw norm is outside [0.5,1.5]")
    normalized = tuple(component / norm for component in quaternion)
    receipt_sha256 = hashlib.sha256(encoded).hexdigest()
    return P1Calibration(
        parent_frame=parent,
        child_frame=child,
        translation_m=translation,  # type: ignore[arg-type]
        quaternion_xyzw=normalized,  # type: ignore[arg-type]
        receipt_sha256=receipt_sha256,
        receipt_id_u52=receipt_id_u52(receipt_sha256),
    )
