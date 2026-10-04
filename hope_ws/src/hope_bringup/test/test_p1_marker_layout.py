"""The v3 CAD table is shared, installed, and never confused with ball centres."""

import importlib.util
import json
from pathlib import Path
import shutil
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import p1_marker_layout as layout


def test_optical_centres_are_used_without_a_second_conversion():
    transforms = layout.load_marker_transforms()
    assert tuple(transforms) == (
        "S04", "S05", "S11", "S12", "S13", "S14",
        "S16", "S18", "S19", "S20", "S21", "S22",
    )
    assert transforms["S04"]["translation_m"] == pytest.approx(
        (-0.023680990, 0.072053090, -0.061630517), abs=1e-12
    )
    assert layout.layout_metadata()["axes"] == {"x": "forward", "y": "left", "z": "up"}
    # The authoring asset must describe the same centres, not a centroid-shifted body.
    asset = layout.table_path().parents[1] / "motive_asset/P1_stickers_definition.json"
    document = json.loads(asset.read_text())
    assert document["source"]["sha256"] == layout.SOURCE_SHA256
    assert document["physical_calibration_status"] == "NOT_PERFORMED"
    for marker in document["markers"]:
        assert marker["position_pelvis_link_m"] == pytest.approx(
            transforms[marker["station"]]["translation_m"], abs=1e-12
        )


def test_old_24_station_receipt_is_rejected_even_with_same_v3_name():
    old = layout.layout_metadata()
    old["marker_names"] = [f"S{i:02d}" for i in range(1, 25)]
    old["source_sha256"] = "3b304f521b85ebac8c8595d1666322ef3c64f9dcd40ffa0d339da25b431bdb90"
    with pytest.raises(ValueError, match="12-station v3"):
        layout.validate_receipt_layout({"approved": True, "marker_layout": old})


def test_pre_alignment_12_station_receipt_is_rejected():
    old = layout.layout_metadata()
    del old["asset_frame_revision"]
    with pytest.raises(ValueError, match="pre-alignment"):
        layout.validate_receipt_layout({"approved": True, "marker_layout": old})


def test_corrupted_table_and_mount_seats_are_rejected(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_bytes(layout.table_path().read_bytes() + b"\n")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        layout.load_marker_transforms(bad)
    seats = layout.table_path().with_name("marker_transforms_pelvis_link_mount_seats.csv")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        layout.load_marker_transforms(seats)


def test_normal_install_works_without_checkout_or_original_cwd(tmp_path, monkeypatch):
    prefix = tmp_path / "isolated_install/hope_bringup"
    module_path = prefix / "lib/hope_bringup/p1_marker_layout.py"
    module_path.parent.mkdir(parents=True)
    shutil.copyfile(SCRIPTS / "p1_marker_layout.py", module_path)
    table = prefix / "share/hope_bringup/config/marker_layouts" / layout.table_path().name
    table.parent.mkdir(parents=True)
    shutil.copyfile(layout.table_path(), table)
    monkeypatch.chdir(tmp_path)
    spec = importlib.util.spec_from_file_location("installed_marker_layout", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.table_path() == table
    assert module.load_marker_transforms() == layout.load_marker_transforms()
    table.unlink()
    with pytest.raises(FileNotFoundError, match="ten-marker table is not a fallback"):
        module.load_marker_transforms()


def test_cmake_installs_the_canonical_table_not_a_numeric_twin():
    cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text()
    assert "scripts/p1_marker_layout.py" in cmake
    assert "${CMAKE_CURRENT_SOURCE_DIR}/../../../" + layout.SOURCE_RELATIVE in cmake
    assert "DESTINATION share/${PROJECT_NAME}/config/marker_layouts" in cmake
