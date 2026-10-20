"""Offline ChingMu asset checks; these do not certify a CMTracker import."""
import csv
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[4]
SHELL = REPO / "agibot/mocap_sticker_shell_v3"
ASSETS = SHELL / "vrpn_asset"
RUNTIME = REPO / "hope_ws/src/hope_bringup/config/pants_profiles"
TABLE = SHELL / "documents/marker_transforms_pelvis_link_stickers.csv"
STATIONS = (
    "S04", "S05", "S06", "S07", "S08", "S11", "S12", "S13", "S14",
    "S15", "S16", "S17", "S18", "S19", "S20", "S21", "S22",
)
NATIVE_FIELDS = {
    "Rbc", "active", "beta", "bodyColor", "enable", "filterEnable", "fixAxis",
    "groupBelongto", "groupName", "id", "isMerged", "lock", "markerPose",
    "minCutoff", "minMarker", "parentId", "type",
}
IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def release():
    return json.loads((ASSETS / "asset_manifest.json").read_text())


def table_rows():
    with TABLE.open(newline="") as stream:
        return list(csv.DictReader(stream))


def points_m(body):
    flat = body["markerPose"]
    return [tuple(value / 1000 for value in flat[i:i + 3])
            for i in range(0, len(flat), 3)]


def asset():
    document = json.loads(
        (ASSETS / "A3_v3_17_stickers.json").read_text()
    )
    assert set(document) == {"UCB_P1"}
    return document["UCB_P1"]


def test_release_manifest_preserves_pelvis_frame_and_canonical_sources():
    document = release()
    assert document["format"] == "HOPE_CMTRACKER_ASSET_RELEASE"
    assert document["schema_version"] == 1
    assert document["body_frame"] == "pelvis_link"
    assert document["units"] == "millimetres"
    assert document["axes"] == {
        "x": "forward", "y": "left", "z": "up", "handedness": "right",
    }
    assert document["centroid_subtracted"] is False
    assert document["pivot_m"] == [0, 0, 0]
    assert document["body_to_pelvis_quaternion_xyzw"] == [0, 0, 0, 1]
    assert document["marker_order"] == list(STATIONS)
    assert "assets" not in document
    assert document["asset"]["file"] == "A3_v3_17_stickers.json"
    source = document["source_table"]
    assert (ASSETS / source["path"]).resolve() == TABLE
    assert source["sha256"] == hashlib.sha256(TABLE.read_bytes()).hexdigest()
    catalog = document["runtime_catalog"]
    assert (ASSETS / catalog["path"]).resolve() == RUNTIME / "profiles.json"
    assert catalog["profile"] == "v4"  # Runtime alias, not the old ten-ball v3.


def test_only_one_import_asset_is_released_with_the_motive_filename_stem():
    assert {path.name for path in ASSETS.glob("*.json")} == {
        "A3_v3_17_stickers.json", "asset_manifest.json",
    }
    assert (SHELL / "motive_asset/A3_v3_17_stickers.motive").is_file()


def test_published_asset_is_byte_identical_to_runtime_p1_definition():
    document = release()
    spec = json.loads((RUNTIME / "profiles.json").read_text())["profiles"]["v4"]
    entry = document["asset"]
    filename = "A3_v3_17_stickers.json"
    assert entry["file"] == filename
    assert entry["body_name"] == "UCB_P1"
    assert entry["id"] == 1
    assert (ASSETS / entry["runtime_file"]).resolve() == RUNTIME / spec["assets"]["P1"]["file"]
    published = (ASSETS / filename).read_bytes()
    runtime = (RUNTIME / spec["assets"]["P1"]["file"]).read_bytes()
    assert published == runtime
    digest = hashlib.sha256(published).hexdigest()
    assert digest == entry["sha256"] == spec["assets"]["P1"]["sha256"]


def test_native_schema_and_optical_centres_match_cad_station_order():
    body = asset()
    assert set(body) == NATIVE_FIELDS
    assert body["Rbc"] == IDENTITY
    assert body["id"] == 1
    flat = body["markerPose"]
    assert len(flat) == 51
    assert all(type(value) in (int, float) and math.isfinite(value) for value in flat)
    rows = table_rows()
    assert tuple(row["marker_id"] for row in rows) == STATIONS
    assert all(row["parent_frame"] == "pelvis_link" for row in rows)
    assert all(row["point_definition"] == "nominal_sticker_surface_0.20mm" for row in rows)
    expected_mm = [float(Decimal(row[key]) * 1000) for row in rows
                   for key in ("x_m", "y_m", "z_m")]
    assert flat == expected_mm
    assert len(set(points_m(body))) == 17


def test_origin_is_not_centroid_and_axes_are_ros_not_motive_native():
    points = points_m(asset())
    centroid = tuple(sum(point[axis] for point in points) / 17 for axis in range(3))
    assert centroid == pytest.approx(
        (-0.007841290176470587, 0.00045418094117647184, -0.0922557295882353),
        rel=0, abs=1e-12,
    )
    assert math.dist(centroid, (0, 0, 0)) > 0.09
    by_station = dict(zip(STATIONS, points))
    front = {"S11", "S12", "S13", "S19", "S20", "S21", "S22"}
    assert all((point[0] > 0) == (name in front) for name, point in by_station.items())
    assert by_station["S13"][1] > 0  # Robot left, not Motive's native up axis.
    assert by_station["S18"][1] < 0
    assert all(point[2] < 0 for point in points)  # Stations below the pelvis datum.
    native_y_up = [(x, z, -y) for x, y, z in points]
    assert any(math.dist(ros, native) > 0.05 for ros, native in zip(points, native_y_up))
    # The CAD authoring data uses the same pelvis frame; Motive storage does not.
    authoring = json.loads((SHELL / "motive_asset/P1_stickers_definition.json").read_text())
    assert [m["station"] for m in authoring["markers"]] == list(STATIONS)
    for point, marker in zip(points, authoring["markers"]):
        assert point == pytest.approx(marker["position_pelvis_link_m"], rel=0, abs=1e-12)


def test_single_asset_geometry_matches_both_existing_runtime_sides():
    body = asset()
    spec = json.loads((RUNTIME / "profiles.json").read_text())["profiles"]["v4"]
    for side in ("P1", "P2"):
        runtime = json.loads((RUNTIME / spec["assets"][side]["file"]).read_text())
        assert body["markerPose"] == runtime[f"UCB_{side}"]["markerPose"]
        assert body["Rbc"] == runtime[f"UCB_{side}"]["Rbc"] == IDENTITY
