"""Accept live Motive identity or opt in to a pinned nominal asset check.

Only pinned mode proves agreement with supplied nominal asset coordinates.
Neither mode measures the physical shell installation or the table origin.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from base_pose_contract import receipt_id_u52

ASSET_FILENAME = "A3_v3_17_stickers.motive"
ASSET_SHA256 = "3f9f7feb6f4613108a2e38eb151556a8c07ed54d5e36520c3883d9ab05dca0b4"
ASSET_REVISION = "pelvis_link_aligned_native_y_up_rear17_20261011"
EXPECTED_MARKERS = 17
# This compares serialized asset coordinates, not camera measurements.
MODELDEF_TOLERANCE_M = 0.00005


def accept_live_modeldef(message, expected_name: str) -> tuple[str, int]:
    """Use the selected body's nominal pelvis frame without an asset-file pin.

    Marker count and geometry are not compared with any reference. The caller
    assumes the imported body's pivot/axes already describe pelvis_link.
    The digest is schema-2 provenance, not a file or marker-layout checksum.
    """
    if expected_name not in {"UCB_P1", "UCB_P2"}:
        raise ValueError("unsupported selected robot asset")
    if message.rigid_body_name != expected_name:
        raise ValueError(f"waiting for {expected_name}")
    body_id = int(message.rigid_body_id)
    if body_id <= 0:
        raise ValueError("invalid Motive rigid-body ID")
    markers = list(message.markers)
    if not markers:
        raise ValueError("waiting for live Motive ModelDef markers")
    members = [int(marker.member_id) for marker in markers]
    if len(set(members)) != len(members) or min(members) < 0:
        raise ValueError("duplicate or invalid ModelDef member IDs")
    for marker in markers:
        if not all(math.isfinite(float(getattr(marker.model_position, axis)))
                   for axis in ("x", "y", "z")):
            raise ValueError("non-finite ModelDef coordinate")
    provenance = dict(source="motive_live_nominal_identity", parent=expected_name,
                      rigid_body_id=body_id, child="pelvis_link", xyz_m=[0, 0, 0],
                      quaternion_xyzw=[0, 0, 0, 1])
    digest = hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest()
    return digest, receipt_id_u52(digest)


def asset_path() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "agibot/mocap_sticker_shell_v3/motive_asset" / ASSET_FILENAME
        if candidate.is_file():
            return candidate
    installed = here.parents[2] / "share/hope_bringup/config/motive_assets" / ASSET_FILENAME
    if installed.is_file():
        return installed
    raise FileNotFoundError(f"missing {ASSET_FILENAME}; rebuild hope_bringup")


def load_asset_points(path: Path | None = None) -> tuple[tuple[float, float, float], ...]:
    encoded = (path or asset_path()).read_bytes()
    if hashlib.sha256(encoded).hexdigest() != ASSET_SHA256:
        raise ValueError("pelvis-aligned Motive asset SHA-256 mismatch")
    body = ET.fromstring(encoded).find("./node_assets/rigid_body")
    points = []
    for marker in body.find("constraints"):
        x, y, z = map(float, marker.findtext("offset").split(","))
        # Exactly the existing driver's ModelDef native Y-up -> ROS Z-up map.
        points.append((x, -z, y))
    if len(points) != EXPECTED_MARKERS:
        raise ValueError(f"pelvis-aligned asset must define {EXPECTED_MARKERS} markers")
    return tuple(points)


def verify_modeldef(message, expected_name: str, points) -> tuple[str, int]:
    """Accept only this asset's local origin/axes, regardless of member order."""
    if expected_name not in {"UCB_P1", "UCB_P2"}:
        raise ValueError("unsupported selected robot asset")
    if message.rigid_body_name != expected_name:
        raise ValueError(f"waiting for {expected_name}")
    body_id = int(message.rigid_body_id)
    if body_id <= 0:
        raise ValueError("invalid Motive rigid-body ID")
    markers = list(message.markers)
    if len(markers) != EXPECTED_MARKERS:
        raise ValueError(f"asset defines {len(markers)} markers; import {ASSET_FILENAME}")
    members = [int(marker.member_id) for marker in markers]
    # The vendored NatNet parser exposes zero-based ModelDef indices. They
    # differ from the native .motive constraint member IDs (usually one-based).
    if len(set(members)) != EXPECTED_MARKERS or min(members) < 0:
        raise ValueError("duplicate or invalid ModelDef member IDs")
    unmatched = set(range(EXPECTED_MARKERS))
    for marker in markers:
        xyz = tuple(float(getattr(marker.model_position, axis)) for axis in ("x", "y", "z"))
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("non-finite ModelDef coordinate")
        matches = [index for index in unmatched if math.dist(xyz, points[index]) <= MODELDEF_TOLERANCE_M]
        if len(matches) != 1:
            raise ValueError("asset origin/axes or marker geometry differ from the pelvis-aligned import")
        unmatched.remove(matches[0])
    # Distinct provenance from a legacy measured calibration receipt. Preserve
    # the schema-2 ABI while binding nominal extrinsics to this live asset ID.
    provenance = dict(source="motive_asset_modeldef", asset_sha256=ASSET_SHA256,
                      revision=ASSET_REVISION, parent=expected_name, rigid_body_id=body_id,
                      child="pelvis_link", xyz_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1])
    digest = hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest()
    return digest, receipt_id_u52(digest)
