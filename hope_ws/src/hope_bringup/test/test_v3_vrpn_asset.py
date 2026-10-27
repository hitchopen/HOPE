"""CAD/CMTracker consistency and partial-pattern checks, not import certification."""
import csv
from decimal import Decimal
import hashlib
import itertools
import json
import math
from pathlib import Path
import runpy

import pytest

REPO = Path(__file__).resolve().parents[4]
SHELL = REPO / "agibot/mocap_sticker_shell_v3"
ASSETS = SHELL / "vrpn_asset"
RUNTIME = REPO / "hope_ws/src/hope_bringup/config/pants_profiles"
TABLE = SHELL / "documents/marker_transforms_pelvis_link_stickers.csv"
ORDERS = {
    "P1": "S19 S20 S11 S12 S22 S21 S13 S14 S04 S05 S16 S18".split(),
    "P2": "S04 S05 S06 S07 S08 S11 S12 S15 S17 S18 S20 S22".split(),
}
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
        return {row["marker_id"]: row for row in csv.DictReader(stream)}


def body(side):
    document = json.loads((ASSETS / (side + ".json")).read_text())
    assert set(document) == {"UCB_" + side}
    return document["UCB_" + side]


def test_release_has_exactly_two_player_assets_and_keeps_the_cad_unchanged():
    manifest = release()
    assert manifest["schema_version"] == 2
    assert manifest["station_count"] == 17
    assert manifest["marker_count_per_player"] == 12
    assert set(p.name for p in ASSETS.glob("*.json")) == {"P1.json", "P2.json", "asset_manifest.json"}
    assert manifest["source_table"]["sha256"] == hashlib.sha256(TABLE.read_bytes()).hexdigest()
    # These hashes predate the player-pattern change, not regenerated expectations.
    assert hashlib.sha256(TABLE.read_bytes()).hexdigest() == "5c0e9ed20497453026d07e877711ef2a184c7b27098525a2b6c3fc6468b150a0"
    assert manifest["unchanged_printable_sha256"]["a3_marker_shell_v3_half_a.stl"] == "e8cd73bd04adb59edfde3f05ccf3fce47396bda512b51b44cf4963239eede935"
    assert manifest["unchanged_printable_sha256"]["a3_marker_shell_v3_half_b.stl"] == "5bf8bf09862e22431e35d023a467c17fb298eaa71dec8f7f31ac97fe169cb9f4"
    for name, digest in manifest["unchanged_printable_sha256"].items():
        assert hashlib.sha256((SHELL / "printable" / name).read_bytes()).hexdigest() == digest
    assert (SHELL / "motive_asset/P1.motive").is_file()
    assert (SHELL / "motive_asset/P2.motive").is_file()


@pytest.mark.parametrize("side", ["P1", "P2"])
def test_native_asset_matches_selected_cad_points_and_runtime_bytes(side):
    manifest = release()
    entry = manifest["assets"][side]
    native = body(side)
    rows = table_rows()
    assert entry["marker_order"] == ORDERS[side]
    assert len(set(entry["marker_order"])) == entry["marker_count"] == 12
    assert set(entry["unmarked_stations"]) == set(rows) - set(ORDERS[side])
    assert set(native) == NATIVE_FIELDS
    assert native["id"] == (1 if side == "P1" else 2)
    assert native["minMarker"] == 6  # Retained, explicitly not an identity guarantee.
    expected = [float(Decimal(rows[n][k]) * 1000) for n in ORDERS[side] for k in ("x_m", "y_m", "z_m")]
    assert len(native["markerPose"]) == 36
    assert native["markerPose"] == expected
    assert all(math.isfinite(v) for v in expected)
    spec = json.loads((RUNTIME / "profiles.json").read_text())["profiles"]["v4"]
    runtime = spec["assets"][side]
    assert runtime["marker_names"] == ORDERS[side]
    encoded = (ASSETS / entry["file"]).read_bytes()
    assert encoded == (RUNTIME / runtime["file"]).read_bytes()
    assert entry["sha256"] == runtime["sha256"] == hashlib.sha256(encoded).hexdigest()


def test_both_assets_preserve_pelvis_origin_ros_axes_and_identity():
    manifest = release()
    assert manifest["body_frame"] == "pelvis_link"
    assert manifest["units"] == "millimetres"
    assert manifest["axes"] == dict(x="forward", y="left", z="up", handedness="right")
    assert manifest["pivot_m"] == [0, 0, 0]
    assert manifest["body_to_pelvis_quaternion_xyzw"] == [0, 0, 0, 1]
    assert manifest["centroid_subtracted"] is False
    for side in ORDERS:
        native = body(side)
        assert native["Rbc"] == IDENTITY
        points = list(zip(*[iter(native["markerPose"])] * 3))
        assert all(z < 0 for x, y, z in points)
        assert abs(sum(z for x, y, z in points) / 12) > 50
        station = dict(zip(ORDERS[side], points))
        assert station["S12"][0] > 0 and station["S12"][1] > 0
        assert station["S18"][0] < 0 and station["S18"][1] < 0
    assert body("P1")["bodyColor"] != body("P2")["bodyColor"]


def test_shared_only_partial_patterns_are_explicitly_identical():
    p1, p2 = map(set, ORDERS.values())
    assert len(p1 & p2) == 12 + 12 - 17 == 7
    assert p2-p1 == {"S06", "S07", "S08", "S15", "S17"}
    assert p1-p2 == {"S13", "S14", "S16", "S19", "S21"}
    a = json.loads((SHELL / "documents/player_pattern_analysis.json").read_text())
    assert a["maximum_separation_candidates_evaluated"] == math.comb(12, 7) == 792
    assert a["identical_shared_subsets"]["6"] == math.comb(7, 6)
    assert a["view_model"]["includes_shell_or_torso_occlusion"] is False
    assert a["p2_order"] == ORDERS["P2"]


def test_exhaustive_search_reproduces_published_selection_and_report():
    pytest.importorskip("numpy")
    authoring = runpy.run_path(str(SHELL / "tools/generate_player_patterns.py"))
    computed = authoring["choose_pattern"]()
    assert computed == json.loads((SHELL / "documents/player_pattern_analysis.json").read_text())
    assert computed["p2_order"] == ORDERS["P2"]


def test_selected_pattern_has_no_extra_four_point_distance_candidates_at_two_mm():
    rows = table_rows()
    positions = {n: tuple(float(row[k])*1000 for k in ("x_m", "y_m", "z_m")) for n, row in rows.items()}
    def signature(subset):
        return sorted(math.dist(positions[a], positions[b]) for a, b in itertools.combinations(subset, 2))
    p1_signatures = [signature(s) for s in itertools.combinations(ORDERS["P1"], 4)]
    shared = set(ORDERS["P1"]) & set(ORDERS["P2"])
    for subset in itertools.combinations(ORDERS["P2"], 4):
        if set(subset) <= shared:
            continue
        candidate = signature(subset)
        assert all(max(abs(a-b) for a,b in zip(candidate, reference)) > 2 for reference in p1_signatures)
