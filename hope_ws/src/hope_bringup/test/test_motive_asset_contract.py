from pathlib import Path
import importlib.util
import shutil
import sys
from types import SimpleNamespace as NS

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import motive_asset_contract as asset


def message(points=None, name="UCB_P1", body_id=17):
    points = asset.load_asset_points() if points is None else points
    return NS(rigid_body_name=name, rigid_body_id=body_id, markers=[
        NS(member_id=index, model_position=NS(x=x, y=y, z=z), has_live_sample=False)
        for index, (x, y, z) in enumerate(points)
    ])


def test_imported_asset_needs_no_live_fit_or_marker_visibility():
    msg = message()
    points = asset.load_asset_points()
    msg.markers.reverse()
    digest, identity = asset.verify_modeldef(msg, "UCB_P1", points)
    assert len(digest) == 64 and 0 < identity < 2**52
    # A rigid-body ID/table-side change must produce different provenance.
    assert asset.verify_modeldef(message(body_id=18), "UCB_P1", points)[0] != digest
    assert asset.verify_modeldef(message(name="UCB_P2"), "UCB_P2", points)[0] != digest


@pytest.mark.parametrize("kind", ["old_body", "missing", "duplicate", "translated", "rotated", "nan", "wrong_side"])
def test_wrong_definition_cannot_silently_use_identity(kind):
    points = asset.load_asset_points()
    msg = message()
    if kind == "old_body":
        msg.markers = msg.markers[:10]
    elif kind == "missing":
        msg.markers.pop()
    elif kind == "duplicate":
        msg.markers[-1].member_id = msg.markers[0].member_id
    elif kind == "translated":
        for marker in msg.markers:
            marker.model_position.z += 0.001
    elif kind == "rotated":
        for marker in msg.markers:
            p = marker.model_position
            p.y, p.z = -p.z, p.y
    elif kind == "nan":
        msg.markers[0].model_position.x = float("nan")
    else:
        msg.rigid_body_name = "UCB_P2"
    with pytest.raises(ValueError):
        asset.verify_modeldef(msg, "UCB_P1", points)


def test_standard_install_resolves_asset_without_source_tree(tmp_path, monkeypatch):
    prefix = tmp_path / "install/hope_bringup"
    module_path = prefix / "lib/hope_bringup/motive_asset_contract.py"
    module_path.parent.mkdir(parents=True)
    shutil.copyfile(SCRIPTS / "motive_asset_contract.py", module_path)
    target = prefix / "share/hope_bringup/config/motive_assets" / asset.ASSET_FILENAME
    target.parent.mkdir(parents=True)
    shutil.copyfile(asset.asset_path(), target)
    spec = importlib.util.spec_from_file_location("installed_asset_contract", module_path)
    installed = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(installed)
    monkeypatch.chdir(tmp_path)
    assert installed.load_asset_points() == asset.load_asset_points()
    target.write_bytes(target.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="SHA-256"):
        installed.load_asset_points()


def test_default_launch_selects_asset_without_legacy_capture():
    source = (SCRIPTS.parent / "launch/optitrack_hope_bridge.launch.py").read_text()
    assert '"localization_source", default_value="motive_live"' in source
    assert '"start_calibration", default_value="false"' in source
    assert '"publish_ucb_markers", default_value="true"' in source


@pytest.mark.parametrize("count", [3, 10, 12, 17, 24])
def test_live_definition_accepts_variable_layout_without_loading_a_file(count, monkeypatch):
    def forbid_file_load(*args):
        pytest.fail("live mode must not read a pinned asset")
    monkeypatch.setattr(asset, "load_asset_points", forbid_file_load)
    msg = message(points=[(i * 0.01, 0.02, 0.03) for i in range(count)])
    digest, identity = asset.accept_live_modeldef(msg, "UCB_P1")
    assert len(digest) == 64 and 0 < identity < 2**52
    msg.markers[0].model_position.x += 0.1
    msg.markers.reverse()
    assert asset.accept_live_modeldef(msg, "UCB_P1") == (digest, identity)
    msg.rigid_body_id += 1
    assert asset.accept_live_modeldef(msg, "UCB_P1")[0] != digest


@pytest.mark.parametrize("kind", ["empty", "duplicate", "negative_member", "nan", "bad_id", "wrong_side"])
def test_live_definition_rejects_malformed_identity(kind):
    msg = message(points=[(0.01, 0.02, i * 0.01) for i in range(17)])
    if kind == "empty":
        msg.markers = []
    elif kind == "duplicate":
        msg.markers[-1].member_id = msg.markers[0].member_id
    elif kind == "negative_member":
        msg.markers[0].member_id = -1
    elif kind == "nan":
        msg.markers[0].model_position.x = float("nan")
    elif kind == "bad_id":
        msg.rigid_body_id = 0
    else:
        msg.rigid_body_name = "UCB_P2"
    with pytest.raises(ValueError):
        asset.accept_live_modeldef(msg, "UCB_P1")
