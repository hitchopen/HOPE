"""Single-source v3 sticker optical-centre / pelvis_link CAD contract.

The CSV is already in ROS metres with X forward, Y left, Z up and includes
the 0.20 mm film thickness. Never apply CAD-axis conversion or thickness twice.
Round stickers do not optically measure the mounting quaternion stored here.
"""

from __future__ import annotations

import csv
import hashlib
import io
import math
from pathlib import Path

LAYOUT_ID = "A3_marker_shell_v3_stickers_12mm"
# Retained v3 station identities; never renumber after removing occluded pads.
MARKER_NAMES = (
    "S04", "S05", "S11", "S12", "S13", "S14",
    "S16", "S18", "S19", "S20", "S21", "S22",
)
SOURCE_RELATIVE = (
    "agibot/mocap_sticker_shell_v3/documents/"
    "marker_transforms_pelvis_link_stickers.csv"
)
SOURCE_SHA256 = "8b1bf1eba661e2a07ae5591aa5d57b9057b496c3b57489b6d206f4855b7ada9e"
POINT_DEFINITION = "nominal_sticker_surface_0.20mm"
# The measured asset was rebased without changing the physical marker table.
# Receipts from its old arbitrary pivot must not survive this frame change.
ASSET_FRAME_REVISION = "pelvis_link_aligned_native_y_up_20261004"


def table_path() -> Path:
    """Resolve the original checkout table or its colcon-installed copy."""
    here = Path(__file__).resolve()
    # Source / symlink-install execution, without depending on the working dir.
    for parent in here.parents:
        candidate = parent / SOURCE_RELATIVE
        if candidate.is_file():
            return candidate
    # Standard install: <prefix>/lib/hope_bringup/p1_marker_layout.py.
    installed = (
        here.parents[2] / "share/hope_bringup/config/marker_layouts"
        / Path(SOURCE_RELATIVE).name
    )
    if installed.is_file():
        return installed
    raise FileNotFoundError(
        "v3 sticker CAD table missing; rebuild/install hope_bringup with its "
        "marker_layouts data (the ten-marker table is not a fallback)"
    )


def load_marker_transforms(path: Path | None = None) -> dict[str, dict]:
    encoded = (path or table_path()).read_bytes()
    if hashlib.sha256(encoded).hexdigest() != SOURCE_SHA256:
        raise ValueError("v3 sticker CAD table SHA-256 mismatch; revalidate the layout")
    result = {}
    for row in csv.DictReader(io.StringIO(encoded.decode("utf-8"))):
        name = row["marker_id"]
        if name not in MARKER_NAMES or name in result:
            raise ValueError(f"invalid/duplicate v3 sticker station: {name}")
        if (
            row["parent_frame"] != "pelvis_link"
            or row["child_frame"] != f"{name}_sticker"
            or row["point_definition"] != POINT_DEFINITION
        ):
            raise ValueError(f"invalid sticker optical-centre frame contract: {name}")
        xyz = tuple(float(row[k]) for k in ("x_m", "y_m", "z_m"))
        quat = tuple(float(row[k]) for k in ("qx", "qy", "qz", "qw"))
        normal = tuple(float(row[k]) for k in ("normal_x", "normal_y", "normal_z"))
        if not all(math.isfinite(v) for v in xyz + quat + normal):
            raise ValueError(f"non-finite v3 marker transform: {name}")
        if abs(sum(v * v for v in quat) - 1.0) > 1e-8:
            raise ValueError(f"non-unit v3 marker quaternion: {name}")
        if abs(sum(v * v for v in normal) - 1.0) > 1e-8:
            raise ValueError(f"non-unit v3 marker normal: {name}")
        result[name] = {
            "translation_m": xyz,
            "quaternion_xyzw": quat,
            "outward_normal": normal,
        }
    if tuple(result) != MARKER_NAMES:
        raise ValueError("v3 sticker table must contain the 12 retained station IDs in canonical order")
    return result


def layout_metadata() -> dict:
    return {
        "id": LAYOUT_ID,
        "coordinate_source": SOURCE_RELATIVE,
        "source_sha256": SOURCE_SHA256,
        "asset_frame_revision": ASSET_FRAME_REVISION,
        "marker_mode": "12mm_round_reflective_stickers",
        "point_definition": POINT_DEFINITION,
        "sticker_total_thickness_m": 0.0002,
        "parent_frame": "pelvis_link",
        "axes": {"x": "forward", "y": "left", "z": "up"},
        "units": "metres",
        "marker_names": list(MARKER_NAMES),
        "physical_status": "CAD_NOMINAL_REQUIRES_INSTALLED_LIVE_VALIDATION",
    }


def validate_receipt_layout(document: dict) -> None:
    """Reject legacy / wrong-mode receipts, even if they say approved=true."""
    if document.get("marker_layout") != layout_metadata():
        raise ValueError(
            "P1 receipt does not match the current v3 sticker layout/TF table; "
            "perform a new 12-station v3 live calibration (pre-alignment, old 24-station, ten-marker, mount-seat "
            "and ball-mode receipts cannot be reused)"
        )
