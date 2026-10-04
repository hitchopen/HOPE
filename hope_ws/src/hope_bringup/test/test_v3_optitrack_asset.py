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

PROJECT = layout.table_path().parents[1]
ASSETS = PROJECT / "motive_asset"
EXPECTED = {
    1: "S19", 2: "S20", 4: "S11", 5: "S12", 6: "S22", 7: "S21",
    8: "S16", 10: "S04", 11: "S18", 12: "S14", 14: "S05", 16: "S13",
}


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
        assert hashlib.sha256((ASSETS / "source" / name).read_bytes()).hexdigest() == digest


def test_aligned_native_asset_preserves_measured_shape_and_graph_pairs():
    source = body(ASSETS / "source/A3.motive")
    subset = body(ASSETS / "A3_v3_12_stickers.motive")
    old = list(source.find("constraints"))
    new = list(subset.find("constraints"))
    indices = [i - 1 for i in EXPECTED]
    assert len(new) == len(layout.MARKER_NAMES) == 12
    assert subset.attrib == source.attrib  # UUID retained: test-import separately.
    expected_properties = [dict(p.attrib) for p in source.find("properties")]
    for prop in expected_properties:
        if prop["name"] in ("NodeName", "AssetName", "JointName"):
            prop["value"] = "P1"
            if "default_value" in prop:
                prop["default_value"] = "P1"
    assert [p.attrib for p in subset.find("properties")] == expected_properties
    for member, (constraint, index) in enumerate(zip(new, indices), 1):
        assert int(constraint.get("member_id")) == member
        assert constraint.get("joint_name") == "P1"
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
    remap = {old_i: new_i for new_i, old_i in enumerate(indices)}
    old_pairs = {
        (remap[int(row.get("index"))], remap[int(item.get("index"))]): item.text
        for row in source.find("constraint_graph") if int(row.get("index")) in remap
        for item in row if int(item.get("index")) in remap
    }
    new_pairs = {
        (int(row.get("index")), int(item.get("index"))): item.text
        for row in subset.find("constraint_graph") for item in row
    }
    assert new_pairs == old_pairs and len(new_pairs) == 66
    names = {c.get("name") for c in new}
    sticks = list(subset.find("marker_sticks"))
    assert len(sticks) == 66
    assert all(s.get("origin") in names and s.get("end") in names for s in sticks)
    assert {(int(r.get("index")), r.get("id")) for r in subset.find("constraint_graph")} == {
        (i, c.get("id")) for i, c in enumerate(new)
    }


def test_measured_subset_recovers_retained_stations_without_assuming_member_order():
    constraints = list(body(ASSETS / "A3_v3_12_stickers.motive").find("constraints"))
    markers = [registration.ModelMarker(
        int(c.get("member_id")), c.get("name"),
        ros_from_native(tuple(float(v) for v in c.findtext("offset").split(","))),
    ) for c in constraints]
    result = registration.resolve_correspondence(markers, layout.MARKER_NAMES)
    assert result.mapping == {i: station for i, station in enumerate(EXPECTED.values(), 1)}
    assert result.registration.rms_m == pytest.approx(0.001395144770534123, abs=1e-10)
    assert result.registration.transform.translation == pytest.approx((0., 0., 0.), abs=1e-10)
    assert result.registration.transform.quaternion == pytest.approx((0., 0., 0., 1.), abs=1e-9)
    report = json.loads((PROJECT / "documents/optitrack_asset_correspondence.json").read_text())
    assert report["motive_import_reexport"] == report["live_calibration"] == "NOT_PERFORMED"
    assert report["retained_station_ids"] == list(layout.MARKER_NAMES)
    assert report["asset_frame_revision"] == layout.ASSET_FRAME_REVISION
    assert result.registration.rms_m * 1000 == pytest.approx(report["fit_rms_mm"])
    for artifact in report["filtered_assets"]:
        assert hashlib.sha256((ASSETS / artifact["path"]).read_bytes()).hexdigest() == artifact["sha256"]
    csv_points = [r for r in csv.reader((ASSETS / "A3_v3_12_stickers.csv").open()) if r[0] == "Point"]
    assert [int(r[1].split()[-1]) for r in csv_points] == list(EXPECTED)
    for row, marker in zip(csv_points, markers):
        assert ros_from_native(tuple(map(float, row[2:5]))) == pytest.approx(marker.position, abs=1e-12)


def test_frame_rebase_is_proper_and_not_a_centroid_reset():
    report = json.loads((PROJECT / "documents/optitrack_asset_correspondence.json").read_text())
    alignment = report["alignment"]
    r = alignment["R_new_native_from_original_asset"]
    t = alignment["t_new_native_from_original_asset_m"]
    assert alignment["scale_factor"] == 1 and alignment["centroid_subtracted"] is False
    for i in range(3):
        for j in range(3):
            assert sum(r[i][k] * r[j][k] for k in range(3)) == pytest.approx(float(i == j), abs=1e-12)
    determinant = sum(r[0][i] * (r[1][(i+1)%3]*r[2][(i+2)%3] - r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    assert determinant == pytest.approx(1., abs=1e-12)
    new = offsets(body(ASSETS / "A3_v3_12_stickers.motive").find("constraints"))
    source = offsets(body(ASSETS / "source/A3.motive").find("constraints"))
    for point, original_id, row in zip(new, EXPECTED, report["mapping"]):
        original = source[original_id - 1]
        predicted = tuple(sum(r[i][k]*original[k] for k in range(3)) + t[i] for i in range(3))
        assert point == pytest.approx(predicted, abs=1e-12)
        assert ros_from_native(point) == pytest.approx(row["aligned_measured_pelvis_link_m"], abs=1e-12)
    centroid = tuple(sum(ros_from_native(p)[axis] for p in new)/12 for axis in range(3))
    assert centroid == pytest.approx((0.01435789875, 0.012864007916666665, -0.0999334205), abs=1e-12)


def test_missing_or_double_axis_conversion_is_not_identity():
    constraints = list(body(ASSETS / "A3_v3_12_stickers.motive").find("constraints"))
    native = offsets(constraints)
    for points in (native, [ros_from_native(ros_from_native(p)) for p in native]):
        markers = [registration.ModelMarker(i+1, c.get("name"), p) for i, (c, p) in enumerate(zip(constraints, points))]
        fit = registration.resolve_correspondence(markers, layout.MARKER_NAMES)
        assert math.sqrt(sum(v*v for v in fit.registration.transform.quaternion[:3])) > .5


def test_identity_default_is_reference_only_not_an_approved_runtime_receipt():
    path = ASSETS / "P1_to_pelvis_link_default.json"
    default = json.loads(path.read_text())
    assert default["p1_to_pelvis_link"]["xyz_m"] == [0., 0., 0.]
    assert default["p1_to_pelvis_link"]["quaternion_xyzw"] == [0., 0., 0., 1.]
    assert default["marker_layout"] == layout.layout_metadata()
    assert default["asset_sha256"] == hashlib.sha256((ASSETS / default["asset_file"]).read_bytes()).hexdigest()
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
    for point in offsets(body(ASSETS / "A3_v3_12_stickers.motive").find("constraints")):
        via_native = ros_from_native(registration.transform_point(world_native, point))
        via_ros = registration.transform_point(world_ros, ros_from_native(point))
        assert via_native == pytest.approx(via_ros, abs=1e-12)


def test_unfiltered_16_point_source_is_rejected_for_new_shell():
    with pytest.raises(ValueError, match="requires all 12"):
        registration.cad_names_for_markers(list(body(ASSETS / "source/A3.motive").find("constraints")))
