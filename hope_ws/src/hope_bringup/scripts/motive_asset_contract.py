"""Accept live Motive identity or opt in to a pinned nominal asset check.

Only pinned mode proves agreement with supplied nominal asset coordinates.
Neither mode measures the physical shell installation or the table origin.
"""
from __future__ import annotations

import hashlib
from functools import lru_cache
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from base_pose_contract import receipt_id_u52

ASSETS = {
    "UCB_P1": ("P1.motive", "26bd10809e5d8a2cf6ea17257e842e15d6792a33b5439dce833e94cb72304b78"),
    "UCB_P2": ("P2.motive", "c263ad41264a8a4728f6633b9033ba520a874489e082158871e3d10899fc16a0"),
}
ASSET_REVISION = "pelvis_link_nominal_player12_20261027"
EXPECTED_MARKERS = 12
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


def asset_spec(expected_name: str) -> tuple[str, str]:
    if expected_name not in ASSETS:
        raise ValueError("unsupported selected robot asset")
    return ASSETS[expected_name]


def asset_path(expected_name: str = "UCB_P1") -> Path:
    filename, _ = asset_spec(expected_name)
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "agibot/mocap_sticker_shell_v3/motive_asset" / filename
        if candidate.is_file():
            return candidate
    installed = here.parents[2] / "share/hope_bringup/config/motive_assets" / filename
    if installed.is_file():
        return installed
    raise FileNotFoundError(f"missing {filename}; rebuild hope_bringup")


def load_asset_points(path: Path | None = None, *, expected_name: str = "UCB_P1") -> tuple[tuple[float, float, float], ...]:
    _, digest = asset_spec(expected_name)
    encoded = (path or asset_path(expected_name)).read_bytes()
    if hashlib.sha256(encoded).hexdigest() != digest:
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


@lru_cache(maxsize=2)
def _reference_points(expected_name):
    # MODELDEF callbacks need no repeated disk reads. Each selected reference
    # is SHA-checked once; only an explicit asset release changes this contract.
    return load_asset_points(expected_name=expected_name)


def verify_modeldef(message, expected_name: str, points) -> tuple[str, int]:
    """Accept only this asset's local origin/axes, regardless of member order."""
    filename, asset_sha256 = asset_spec(expected_name)
    if tuple(tuple(p) for p in points) != _reference_points(expected_name):
        raise ValueError("reference points do not match the selected player's asset")
    if message.rigid_body_name != expected_name:
        raise ValueError(f"waiting for {expected_name}")
    body_id = int(message.rigid_body_id)
    if body_id <= 0:
        raise ValueError("invalid Motive rigid-body ID")
    markers = list(message.markers)
    if len(markers) != EXPECTED_MARKERS:
        raise ValueError(f"asset defines {len(markers)} markers; import {filename}")
    members = [int(marker.member_id) for marker in markers]
    # Match geometry bijectively; do not assume native export/member ordering.
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
    provenance = dict(source="motive_asset_modeldef", asset_sha256=asset_sha256,
                      revision=ASSET_REVISION, parent=expected_name, rigid_body_id=body_id,
                      child="pelvis_link", xyz_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1])
    digest = hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest()
    return digest, receipt_id_u52(digest)
