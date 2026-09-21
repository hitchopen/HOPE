"""Load the approved selected-UCB -> pelvis_link calibration receipt.

The module and schema retain ``p1`` identifiers for receipt compatibility; the
runtime parent frame must be the canonical ``UCB_P1`` or ``UCB_P2`` asset.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from p1_marker_layout import validate_receipt_layout
from pathlib import Path

from base_pose_contract import receipt_id_u52


@dataclass(frozen=True)
class P1Calibration:
    parent_frame: str
    child_frame: str
    rigid_body_id: int | None
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

    return decode_p1_calibration(path.read_bytes())


def decode_p1_calibration(encoded: bytes) -> P1Calibration:
    """Validate an uploaded receipt without changing its bytes or wire ID."""
    document = json.loads(encoded.decode("utf-8"))
    if not isinstance(document, dict):
        raise ValueError("UCB robot calibration document must be a JSON object")
    if document.get("approved") is not True:
        raise ValueError("UCB robot calibration receipt is not approved")

    layout_id = document.get("cad", {}).get("marker_layout")
    if layout_id not in {"v2", "v3"}:
        validate_receipt_layout(document)

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
        raise ValueError(f"invalid UCB robot calibration document: missing {exc}") from exc

    if not isinstance(parent, str) or not parent:
        raise ValueError("parent_frame must be a non-empty string")
    if not isinstance(child, str) or not child:
        raise ValueError("child_frame must be a non-empty string")
    if parent == child:
        raise ValueError("parent_frame and child_frame must differ")

    source = document.get("source")
    rigid_body_id: int | None = None
    if isinstance(source, dict) and "rigid_body_id" in source:
        try:
            rigid_body_id = int(source["rigid_body_id"])
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("source.rigid_body_id must be an integer") from exc
        if rigid_body_id <= 0:
            raise ValueError("source.rigid_body_id must be positive")

    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 0.5 or norm > 1.5:
        raise ValueError("quaternion_xyzw norm is outside [0.5,1.5]")
    normalized = tuple(component / norm for component in quaternion)
    receipt_sha256 = hashlib.sha256(encoded).hexdigest()
    return P1Calibration(
        parent_frame=parent,
        child_frame=child,
        rigid_body_id=rigid_body_id,
        translation_m=translation,  # type: ignore[arg-type]
        quaternion_xyzw=normalized,  # type: ignore[arg-type]
        receipt_sha256=receipt_sha256,
        receipt_id_u52=receipt_id_u52(receipt_sha256),
    )


def validate_calibration_source_identity(
    calibration: P1Calibration,
    *,
    live_rigid_body_name: str,
    live_rigid_body_id: int,
) -> None:
    """Reject a receipt captured from a different Motive rigid-body asset."""

    if str(live_rigid_body_name) != calibration.parent_frame:
        raise ValueError(
            f"live rigid body {live_rigid_body_name!r} != calibration parent "
            f"{calibration.parent_frame!r}"
        )
    if calibration.rigid_body_id is None:
        raise ValueError("calibration receipt has no source.rigid_body_id")
    live_id = int(live_rigid_body_id)
    if live_id <= 0:
        raise ValueError("live rigid body ID must be positive")
    if live_id != calibration.rigid_body_id:
        raise ValueError(
            f"live rigid body ID {live_id} != calibration source ID "
            f"{calibration.rigid_body_id}"
        )
