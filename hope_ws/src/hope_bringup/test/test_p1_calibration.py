import hashlib
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from p1_calibration import load_p1_calibration
from p1_marker_layout import layout_metadata


def test_loads_marker_cad_receipt_and_derives_file_identity(tmp_path):
    path = tmp_path / "p1_to_pelvis.json"
    encoded = (
        json.dumps(
            {
                "approved": True,
                "marker_layout": layout_metadata(),
                "p1_to_pelvis_link": {
                    "parent_frame": "P1",
                    "child_frame": "pelvis_link",
                    "xyz_m": [0.018, 0.0, 0.148],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.2],
                },
            },
            sort_keys=True,
        )
        + "\n"
    ).encode()
    path.write_bytes(encoded)

    calibration = load_p1_calibration(path)

    expected_sha = hashlib.sha256(encoded).hexdigest()
    assert calibration.translation_m == pytest.approx((0.018, 0.0, 0.148))
    assert calibration.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert calibration.receipt_sha256 == expected_sha
    assert calibration.receipt_id_u52 == int(expected_sha[:13], 16)


def test_rejects_unapproved_receipt(tmp_path):
    path = tmp_path / "p1_to_pelvis.json"
    path.write_text(json.dumps({"approved": False}), encoding="utf-8")
    with pytest.raises(ValueError, match="not approved"):
        load_p1_calibration(path)


@pytest.mark.parametrize("change", ["missing", "old_id", "wrong_hash", "ball_mode", "missing_station"])
def test_rejects_approved_receipt_from_another_layout(tmp_path, change):
    metadata = layout_metadata()
    if change == "old_id":
        metadata["id"] = "A3_hip_shell_P1_0702"
    elif change == "wrong_hash":
        metadata["source_sha256"] = "0" * 64
    elif change == "ball_mode":
        metadata["marker_mode"] = "12mm_balls"
    elif change == "missing_station":
        metadata["marker_names"].pop()
    document = {"approved": True, "marker_layout": metadata}
    if change == "missing":
        del document["marker_layout"]
    path = tmp_path / "old.json"
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="current v3 sticker layout"):
        load_p1_calibration(path)


def test_rejects_receipt_without_explicit_approval(tmp_path):
    path = tmp_path / "p1_to_pelvis.json"
    path.write_text(
        json.dumps(
            {
                "p1_to_pelvis": {
                    "parent_frame": "P1",
                    "child_frame": "pelvis_link",
                    "translation_m": [0.0, 0.0, 0.15],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="not approved"):
        load_p1_calibration(path)
