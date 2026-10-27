"""Static asset provenance/correspondence checks; not a Motive import test."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import p1_marker_layout as layout
import p1_marker_cad_registration_impl as registration
from p1_calibration import load_p1_calibration
from motive_asset_contract import load_asset_points
import motive_asset_contract as asset_contract

PROJECT = layout.table_path().parents[1]
ASSETS = PROJECT / "motive_asset"
REPO = PROJECT.parents[1]
SOURCE = PROJECT / "documents/source_evidence/optitrack"
FIXTURES = Path(__file__).parent / "fixtures/motive_legacy_17"
EXPECTED = {
    1: "S19", 2: "S20", 4: "S11", 5: "S12", 6: "S22", 7: "S21",
    8: "S16", 10: "S04", 11: "S18", 12: "S14", 14: "S05", 16: "S13",
}
RESTORED = ("S06", "S07", "S08", "S15", "S17")
MEMBERS = {i: station for i, station in enumerate([*EXPECTED.values(), *RESTORED], 1)}
ASSET = FIXTURES / "A3_v3_17_stickers.motive"


def body(path):
    return ET.parse(path).getroot().find("./node_assets/rigid_body")


def offsets(constraints):
    return [tuple(map(float, c.findtext("offset").split(","))) for c in constraints]


def ros_from_native(point):
    return (point[0], -point[2], point[1])


def native_from_ros(point):
    return (point[0], point[2], -point[1])


def test_source_files_are_preserved_byte_for_byte():
    for name, digest in (
        ("A3.motive", "7e5423ae1943761380eb834653011dfbf1a52cd6961bdac45504c0741ba7840a"),
        ("A3.csv", "5c0f478c12f93e212fdda6478e3ee1413e7f6de5323d464a97e2816cbcc6c7ab"),
    ):
        assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == digest


def test_aligned_native_asset_preserves_measured_shape_without_inventing_graph_statistics():
    source = body(SOURCE / "A3.motive")
    subset = body(ASSET)
    old = list(source.find("constraints"))
    new = list(subset.find("constraints"))
    indices = [i - 1 for i in EXPECTED]
    assert len(new) == len(layout.MARKER_NAMES) == 17
    assert len({c.get("id") for c in new}) == 17
    assert [int(c.get("member_id")) for c in new] == list(range(1, 18))
    assert subset.attrib == source.attrib  # UUID retained: test-import separately.
    expected_properties = [dict(p.attrib) for p in source.find("properties")]
    for prop in expected_properties:
        if prop["name"] in ("NodeName", "AssetName", "JointName"):
            prop["value"] = "UCB_P1"
            if "default_value" in prop:
                prop["default_value"] = "UCB_P1"
    assert [p.attrib for p in subset.find("properties")] == expected_properties
    for member, (constraint, index) in enumerate(zip(new, indices), 1):
        assert int(constraint.get("member_id")) == member
        assert constraint.get("joint_name") == "UCB_P1"
        assert {k: v for k, v in constraint.attrib.items() if k not in ("member_id", "joint_name")} == {
            k: v for k, v in old[index].attrib.items() if k not in ("member_id", "joint_name")
        }
        assert [(c.tag, c.text) for c in constraint if c.tag != "offset"] == [
            (c.tag, c.text) for c in old[index] if c.tag != "offset"
        ]
    old_xyz = offsets([old[i] for i in indices])
    new_xyz = offsets(new)
    for i in range(12):
        for j in range(i):
            assert math.dist(old_xyz[i], old_xyz[j]) == pytest.approx(
                math.dist(new_xyz[i], new_xyz[j]), abs=2e-12
            )
    # The source's learned pair statistics do not exist for the five new points.
    # This offline candidate omits the cache; native import remains unverified.
    assert subset.find("constraint_graph") is None
    names = {c.get("name") for c in new}
    sticks = list(subset.find("marker_sticks"))
    assert len(sticks) == 136
    assert all(s.get("origin") in names and s.get("end") in names for s in sticks)
    assert len({frozenset((s.get("origin"), s.get("end"))) for s in sticks}) == 136
    table = layout.load_marker_transforms()
    for constraint, station in zip(new[12:], RESTORED):
        assert constraint.get("name") == station
        assert ros_from_native(offsets([constraint])[0]) == pytest.approx(table[station]["translation_m"], abs=1e-12)
        assert float(constraint.findtext("diameter")) == .012


def test_mixed_asset_recovers_all_stations_without_assuming_member_order():
    constraints = list(body(ASSET).find("constraints"))
    markers = [registration.ModelMarker(
        int(c.get("member_id")), c.get("name"),
        ros_from_native(tuple(float(v) for v in c.findtext("offset").split(","))),
    ) for c in constraints]
    result = registration.resolve_correspondence(
        markers, layout.MARKER_NAMES,
        cad_markers_pelvis_m={name: row["translation_m"] for name, row in layout.load_marker_transforms().items()},
    )
    assert result.mapping == MEMBERS
    assert result.registration.rms_m == pytest.approx(0.001395144770534123 * math.sqrt(12 / 17), abs=1e-10)
    assert result.registration.transform.translation == pytest.approx((0., 0., 0.), abs=1e-10)
    assert result.registration.transform.quaternion == pytest.approx((0., 0., 0., 1.), abs=1e-9)
    report = json.loads((SOURCE / "asset_correspondence_20261011.json").read_text())
    assert report["motive_import_reexport"] == report["live_calibration"] == "NOT_PERFORMED"
    assert report["retained_station_ids"] == list(layout.MARKER_NAMES)
    assert report["asset_frame_revision"] == layout.ASSET_FRAME_REVISION
    # Do not dilute measurement accuracy by counting five exact nominal points.
    assert report["fit_rms_mm"] == pytest.approx(1.395144770534123)
    assert report["restored_station_ids"] == list(RESTORED)
    assert all(row["provenance"] == "MEASURED_SOURCE_RIGIDLY_ALIGNED" for row in report["mapping"][:12])
    assert all(row["provenance"] == "CAD_NOMINAL_NOT_OBSERVED" and row["residual_mm"] is None and row["original_asset_point"] is None for row in report["mapping"][12:])
    assert report["path_base"] == "REPOSITORY_ROOT"
    for artifact in report["source_assets"] + report["filtered_assets"]:
        assert hashlib.sha256((REPO / artifact["path"]).read_bytes()).hexdigest() == artifact["sha256"]
    csv_points = [r for r in csv.reader((FIXTURES / "A3_v3_17_stickers.csv").open()) if r[0] == "Point"]
    assert [int(r[1].split()[-1]) for r in csv_points[:12]] == list(EXPECTED)
    assert [r[1] for r in csv_points[12:]] == list(RESTORED)
    for row, marker in zip(csv_points, markers):
        assert ros_from_native(tuple(map(float, row[2:5]))) == pytest.approx(marker.position, abs=1e-12)


def test_frame_rebase_is_proper_and_not_a_centroid_reset():
    report = json.loads((SOURCE / "asset_correspondence_20261011.json").read_text())
    alignment = report["alignment"]
    r = alignment["R_new_native_from_original_asset"]
    t = alignment["t_new_native_from_original_asset_m"]
    assert alignment["scale_factor"] == 1 and alignment["centroid_subtracted"] is False
    for i in range(3):
        for j in range(3):
            assert sum(r[i][k] * r[j][k] for k in range(3)) == pytest.approx(float(i == j), abs=1e-12)
    determinant = sum(r[0][i] * (r[1][(i+1)%3]*r[2][(i+2)%3] - r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    assert determinant == pytest.approx(1., abs=1e-12)
    new = offsets(body(ASSET).find("constraints"))[:12]
    source = offsets(body(SOURCE / "A3.motive").find("constraints"))
    for point, original_id, row in zip(new, EXPECTED, report["mapping"]):
        original = source[original_id - 1]
        predicted = tuple(sum(r[i][k]*original[k] for k in range(3)) + t[i] for i in range(3))
        assert point == pytest.approx(predicted, abs=1e-12)
        assert ros_from_native(point) == pytest.approx(row["aligned_measured_pelvis_link_m"], abs=1e-12)
    centroid = tuple(sum(ros_from_native(p)[axis] for p in new)/12 for axis in range(3))
    assert centroid == pytest.approx((0.01435789875, 0.012864007916666665, -0.0999334205), abs=1e-12)


def test_missing_or_double_axis_conversion_is_not_identity():
    constraints = list(body(ASSET).find("constraints"))
    native = offsets(constraints)
    for points in (native, [ros_from_native(ros_from_native(p)) for p in native]):
        markers = [registration.ModelMarker(i+1, c.get("name"), p) for i, (c, p) in enumerate(zip(constraints, points))]
        fit = registration.resolve_correspondence(
            markers, layout.MARKER_NAMES,
            cad_markers_pelvis_m={name: row["translation_m"] for name, row in layout.load_marker_transforms().items()},
        )
        assert math.sqrt(sum(v*v for v in fit.registration.transform.quaternion[:3])) > .5


def test_identity_default_is_reference_only_not_an_approved_runtime_receipt():
    path = FIXTURES / "nominal_17_identity_default.json"
    default = json.loads(path.read_text())
    assert default["p1_to_pelvis_link"]["xyz_m"] == [0., 0., 0.]
    assert default["p1_to_pelvis_link"]["quaternion_xyzw"] == [0., 0., 0., 1.]
    assert default["marker_layout"] == layout.layout_metadata()
    assert default["asset_sha256"] == hashlib.sha256((FIXTURES / default["asset_file"]).read_bytes()).hexdigest()
    assert default["required_motive_streaming_up_axis"] == "Z"
    assert default["modeldef_y_up_to_z_up"] is True
    with pytest.raises(ValueError, match="not approved"):
        load_p1_calibration(path)


@pytest.mark.parametrize("rotation", [(0., 0., 0., 1.), (.2, -.3, .4, .8), (.7, .1, -.2, .4)])
def test_reframed_constellation_preserves_nonidentity_world_poses(rotation):
    # Exercise the declared two-basis conversion, not Motive itself. This must
    # work for tilted poses as well as a level/identity demonstration.
    world_ros = registration.Transform(
        (1.3, -.8, .45), registration.normalize_quaternion(rotation)
    )
    c_quat = (math.sqrt(.5), 0., 0., math.sqrt(.5))
    q_native = registration.quaternion_multiply(
        registration.quaternion_multiply(
            registration.quaternion_conjugate(c_quat), world_ros.quaternion
        ), c_quat
    )
    world_native = registration.Transform(native_from_ros(world_ros.translation), q_native)
    for point in offsets(body(ASSET).find("constraints")):
        via_native = ros_from_native(registration.transform_point(world_native, point))
        via_ros = registration.transform_point(world_ros, ros_from_native(point))
        assert via_native == pytest.approx(via_ros, abs=1e-12)


def test_unfiltered_16_point_source_is_rejected_for_new_shell():
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        load_asset_points(SOURCE / "A3.motive")


def test_only_rear_stations_were_restored_and_front_mesh_is_unchanged():
    report = json.loads((PROJECT / "documents/printability_validation.json").read_text())
    assert report["front_station_ids"] == ["S11", "S12", "S13", "S19", "S20", "S21", "S22"]
    assert report["rear_upper_row"] == ["S04", "S05", "S06", "S07", "S08"]
    assert report["rear_middle_row"] == ["S14", "S15", "S16", "S17", "S18"]
    assert report["restored_station_ids"] == list(RESTORED)
    for suffix, key in (("a", "front"), ("b", "rear")):
        path = PROJECT / f"printable/a3_marker_shell_v3_half_{suffix}.stl"
        assert hashlib.sha256(path.read_bytes()).hexdigest() == report[f"{key}_stl_sha256_after"]
    assert report["front_stl_sha256_after"] == report["front_stl_sha256_before"] == "e8cd73bd04adb59edfde3f05ccf3fce47396bda512b51b44cf4963239eede935"


PLAYER_ORDERS = {
    "P1": "S19 S20 S11 S12 S22 S21 S13 S14 S04 S05 S16 S18".split(),
    "P2": "S04 S05 S06 S07 S08 S11 S12 S15 S17 S18 S20 S22".split(),
}


@pytest.mark.parametrize("side", ["P1", "P2"])
def test_player_native_asset_matches_cad_chingmu_and_runtime_pin(side):
    manifest = json.loads((ASSETS / "asset_manifest.json").read_text())
    spec = manifest["assets"][side]
    name = "UCB_" + side
    path = ASSETS / spec["file"]
    assert spec["marker_order"] == PLAYER_ORDERS[side]
    assert spec["marker_count"] == 12
    assert spec["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert asset_contract.ASSETS[name] == (path.name, spec["sha256"])
    assert manifest["revision"] == asset_contract.ASSET_REVISION
    assert manifest["source_table"]["sha256"] == hashlib.sha256(layout.table_path().read_bytes()).hexdigest()
    root = ET.parse(path).getroot()
    assert len(root.findall("./node_assets/rigid_body")) == 1
    b = body(path)
    props = {p.get("name"): p.get("value") for p in b.find("properties")}
    assert all(props[key] == name for key in ("NodeName", "AssetName", "JointName"))
    assert int(props["UserData"]) == spec["streaming_id"] == (9 if side == "P1" else 10)
    constraints = list(b.find("constraints"))
    assert [c.get("name") for c in constraints] == PLAYER_ORDERS[side]
    assert [int(c.get("member_id")) for c in constraints] == list(range(1, 13))
    assert all(c.get("joint_name") == name and float(c.findtext("diameter")) == .012 for c in constraints)
    assert b.find("constraint_graph") is None
    sticks = list(b.find("marker_sticks"))
    pairs = {frozenset((s.get("origin"), s.get("end"))) for s in sticks}
    assert len(sticks) == len(pairs) == 66
    assert all(len(pair) == 2 and pair <= set(PLAYER_ORDERS[side]) for pair in pairs)
    table = layout.load_marker_transforms()
    cm = json.loads((PROJECT / "vrpn_asset" / (side + ".json")).read_text())[name]["markerPose"]
    native_points = offsets(constraints)
    reference = load_asset_points(expected_name=name)
    for i, (station, point, record) in enumerate(zip(PLAYER_ORDERS[side], native_points, spec["markers"])):
        xyz = ros_from_native(point)
        assert xyz == pytest.approx(table[station]["translation_m"], abs=1e-12)
        assert xyz == pytest.approx(tuple(v / 1000 for v in cm[3*i:3*i+3]), abs=1e-12)
        assert xyz == reference[i] == tuple(record["position_pelvis_link_m"])
        assert point == tuple(record["position_motive_native_y_up_m"])
        assert record["station"] == station and record["member_id"] == i + 1
    assert math.dist(tuple(sum(p[k] for p in reference)/12 for k in range(3)), (0,0,0)) > .05
    assert manifest["centroid_subtracted"] is False
    assert manifest["body_to_pelvis_xyz_m"] == [0,0,0]
    assert manifest["body_to_pelvis_quaternion_xyzw"] == [0,0,0,1]
    assert manifest["motive_import_reexport"] == manifest["live_tracking"] == "NOT_PERFORMED"
    for filename, digest in manifest["historical_reference_sha256"].items():
        assert hashlib.sha256((ASSETS / filename).read_bytes()).hexdigest() == digest
    for filename, digest in manifest["unchanged_printable_sha256"].items():
        assert hashlib.sha256((PROJECT / "printable" / filename).read_bytes()).hexdigest() == digest


def test_player_uuids_are_distinct_from_each_other_and_historical_sources():
    existing = {node.get("id") for path in (SOURCE / "A3.motive", ASSET)
                for node in ET.parse(path).iter() if node.get("id")}
    seen = set()
    for side in PLAYER_ORDERS:
        ids = [node.get("id") for node in ET.parse(ASSETS / (side + ".motive")).iter() if node.get("id")]
        assert len(ids) == len(set(ids)) == 13
        assert not set(ids) & (existing | seen)
        seen.update(ids)


def test_release_import_folders_have_only_current_player_pair_and_manifest():
    for directory, extension in ((ASSETS, ".motive"), (PROJECT / "vrpn_asset", ".json")):
        visible = {p.name for p in directory.iterdir() if not p.name.startswith(".")}
        assert visible == {"P1" + extension, "P2" + extension, "README.md", "asset_manifest.json"}
    cmake = (Path(__file__).parents[1] / "CMakeLists.txt").read_text()
    assert "/motive_asset/P1.motive" in cmake
    assert "/motive_asset/P2.motive" in cmake
    assert "A3_v3_17_stickers" not in cmake
    assert "fixtures/motive_legacy_17" not in cmake


@pytest.mark.parametrize("side", ["P1", "P2"])
@pytest.mark.parametrize("rotation", [(0.,0.,0.,1.), (.2,-.3,.4,.8), (.7,.1,-.2,.4)])
def test_player_frames_preserve_tilted_world_poses(side, rotation):
    world_ros = registration.Transform((1.3,-.8,.45), registration.normalize_quaternion(rotation))
    q = (math.sqrt(.5),0.,0.,math.sqrt(.5))
    native_q = registration.quaternion_multiply(registration.quaternion_multiply(
        registration.quaternion_conjugate(q), world_ros.quaternion), q)
    world_native = registration.Transform(native_from_ros(world_ros.translation), native_q)
    for point in offsets(body(ASSETS / (side + ".motive")).find("constraints")):
        assert ros_from_native(registration.transform_point(world_native, point)) == pytest.approx(
            registration.transform_point(world_ros, ros_from_native(point)), abs=1e-12)
