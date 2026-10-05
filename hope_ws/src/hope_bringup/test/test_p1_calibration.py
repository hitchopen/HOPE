import hashlib
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from p1_calibration import (
    load_p1_calibration,
    validate_calibration_source_identity,
)


def test_loads_marker_cad_receipt_and_derives_file_identity(tmp_path):
    path = tmp_path / "ucb_robot_to_pelvis.json"
    encoded = (
        json.dumps(
            {
                "approved": True,
                "cad": {"marker_layout": "v2"},
                "source": {"rigid_body_id": 34},
                "p1_to_pelvis_link": {
                    "parent_frame": "UCB_P1",
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
    assert calibration.rigid_body_id == 34
    assert calibration.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert calibration.receipt_sha256 == expected_sha
    assert calibration.receipt_id_u52 == int(expected_sha[:13], 16)
    validate_calibration_source_identity(
        calibration, live_rigid_body_name="UCB_P1", live_rigid_body_id=34
    )


def test_rejects_live_rigid_body_id_that_differs_from_receipt(tmp_path):
    path = tmp_path / "ucb_robot_to_pelvis.json"
    path.write_text(
        json.dumps(
            {
                "approved": True,
                "cad": {"marker_layout": "v2"},
                "source": {"rigid_body_id": 34},
                "p1_to_pelvis": {
                    "parent_frame": "UCB_P2",
                    "child_frame": "pelvis_link",
                    "translation_m": [0.0, 0.0, 0.15],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            }
        ),
        encoding="utf-8",
    )
    calibration = load_p1_calibration(path)
    with pytest.raises(ValueError, match="live rigid body ID 15.*source ID 34"):
        validate_calibration_source_identity(
            calibration, live_rigid_body_name="UCB_P2", live_rigid_body_id=15
        )


def test_identity_check_rejects_legacy_receipt_without_source_id(tmp_path):
    path = tmp_path / "ucb_robot_to_pelvis.json"
    path.write_text(
        json.dumps(
            {
                "approved": True,
                "cad": {"marker_layout": "v2"},
                "p1_to_pelvis": {
                    "parent_frame": "UCB_P1",
                    "child_frame": "pelvis_link",
                    "translation_m": [0.0, 0.0, 0.15],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            }
        ),
        encoding="utf-8",
    )
    calibration = load_p1_calibration(path)
    assert calibration.rigid_body_id is None
    with pytest.raises(ValueError, match="no source.rigid_body_id"):
        validate_calibration_source_identity(
            calibration, live_rigid_body_name="UCB_P1", live_rigid_body_id=7
        )


def test_rejects_unapproved_receipt(tmp_path):
    path = tmp_path / "ucb_robot_to_pelvis.json"
    path.write_text(json.dumps({"approved": False}), encoding="utf-8")
    with pytest.raises(ValueError, match="not approved"):
        load_p1_calibration(path)


def test_rejects_receipt_without_explicit_approval(tmp_path):
    path = tmp_path / "ucb_robot_to_pelvis.json"
    path.write_text(
        json.dumps(
            {
                "p1_to_pelvis": {
                    "parent_frame": "UCB_P1",
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
