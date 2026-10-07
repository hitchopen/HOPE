"""The current public optical-center table survives the multi-layout port."""
import json
from pathlib import Path
import sys
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import p1_marker_cad_registration_impl as cad
from p1_marker_layout import MARKER_NAMES, layout_metadata
from p1_calibration import decode_p1_calibration


def test_public_sticker_capture_and_receipt_roundtrip():
    layout = cad.STICKER_MARKER_LAYOUT
    transform = cad.Transform((.02, -.01, .15), cad.normalize_quaternion((.1, -.2, .3, .9)))
    markers = [cad.ModelMarker(i+1, name, cad.transform_point(transform, layout.markers_pelvis_m[name])) for i,name in enumerate(MARKER_NAMES)]
    capture = cad.Capture('UCB_P2', 7, 'world', markers, frames_received=200)
    capture.frames_with_physical_samples = 200
    capture.poses = [cad.Transform((0,0,1), (0,0,0,1))] * 200
    for marker in markers:
        capture.live_errors_m[marker.member_id] = [.0007] * 200
        capture.live_residuals_m[marker.member_id] = [.0004] * 200
    document, blockers = cad.analyze_capture(
        capture, MARKER_NAMES, {}, max_registration_rms_m=.003,
        max_registration_max_m=.006, max_pairwise_rms_m=.003,
        minimum_mapping_margin_m=.0015, minimum_live_samples_per_marker=30,
        max_live_rms_m=.004, max_live_max_m=.005, minimum_rotation_span_deg=0,
        operator_attested_installed_layout=True, allow_nominal_only_markers=False,
        layout=layout, table_side='P2')
    assert blockers == []
    assert document['marker_layout'] == layout_metadata()
    assert document['runtime_world']['table_side'] == 'P2'
    receipt = decode_p1_calibration(json.dumps(document).encode())
    assert receipt.parent_frame == 'UCB_P2'
    assert receipt.rigid_body_id == 7
    document['marker_layout']['source_sha256'] = '0' * 64
    with pytest.raises(ValueError):
        decode_p1_calibration(json.dumps(document).encode())


def test_layouts_do_not_substitute_for_each_other():
    assert set(cad.MARKER_LAYOUTS) == {'stickers_v3', 'v2', 'v3'}
    assert len(cad.STICKER_MARKER_LAYOUT.markers_pelvis_m) == 12
    assert len(cad.V2_MARKER_LAYOUT.markers_pelvis_m) == 10
    assert len(cad.V3_MARKER_LAYOUT.markers_pelvis_m) == 10


@pytest.mark.parametrize("count", [8, 10, 11, 16, 24])
def test_sticker_layout_rejects_superseded_or_incomplete_modeldef(count):
    with pytest.raises(ValueError, match="requires all 12"):
        cad.cad_names_for_markers([None] * count, layout=cad.STICKER_MARKER_LAYOUT)


def test_sticker_selection_requires_retained_stations():
    markers = [None] * len(MARKER_NAMES)
    assert cad.cad_names_for_markers(markers, layout=cad.STICKER_MARKER_LAYOUT) == MARKER_NAMES
    with pytest.raises(ValueError, match="complete retained 12-station"):
        cad.cad_names_for_markers(markers, ["S07"] + list(MARKER_NAMES[1:]), cad.STICKER_MARKER_LAYOUT)
    for name in MARKER_NAMES:
        assert cad.canonical_marker_name("P1_" + name + "_sticker") == name
    for label in ("S07", "S08", "S24", "S00", "S25", "S010", "S04_S05", "XS04"):
        assert cad.canonical_marker_name(label) is None
