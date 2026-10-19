import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "foxglove/a3"))
from hope_ball_models import PROFILES, compare_parameters, handle, profile, select, selected
from hope_field_operations import FieldOperations

CONFIG = ROOT / "hope_ws/src/hope_planner_cpp/config"


@pytest.mark.parametrize("model", PROFILES)
def test_profile_matches_original_fit_receipt_and_round_trips(tmp_path, model):
    document = profile(CONFIG, model)
    receipt = json.loads((CONFIG / PROFILES[model].replace(".yaml", ".receipt.json")).read_text())
    # Both archived receipts use the exact adopted parameter dictionary.
    expected = receipt.get("parameters", receipt.get("adopted_parameters"))
    if expected is None:
        expected = receipt["parameters_adopted"]
    assert document["hope_planner"]["ros__parameters"] == expected
    path = tmp_path / "physics.json"
    select(path, CONFIG, model)
    assert selected(path, CONFIG) == model
    assert handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "model": model},
                  read_live=lambda *_: expected)["runtime"] == "VERIFIED"


def test_mismatch_missing_wrong_type_nan_and_unreachable_are_not_verified(tmp_path):
    path = tmp_path / "physics.json"
    select(path, CONFIG, "ppq002")
    expected = profile(CONFIG, "ppq002")["hope_planner"]["ros__parameters"]
    for bad in ({}, {**expected, "drag_k": float("nan")}, {**expected, "drag_k": .2},
                {**expected, "future_contact_spin_enabled": 1}):
        assert compare_parameters(expected, bad)
        with pytest.raises(ValueError, match="different physics"):
            handle(path, CONFIG, "laptop", {"op": "verify_ball_model"}, read_live=lambda *_: bad)
    assert handle(path, CONFIG, "laptop", {"op": "verify_ball_model"}, read_live=lambda *_: None)["runtime"] == "NOT_RUNNING"
    with pytest.raises(ValueError, match="not running"):
        handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "require_running": True}, read_live=lambda *_: None)
    with pytest.raises(ValueError, match="differs"):
        handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "model": "berkeley"})
    path.write_text('{}')
    with pytest.raises(ValueError, match="does not match"):
        selected(path, CONFIG)


def field(tmp_path, fail=None):
    states = {"hdu": "berkeley", "laptop": "berkeley"}
    calls = []
    def exchange(host, config, request):
        calls.append((host, dict(request)))
        if request["op"] == "select_ball_model":
            states[host] = request["model"]
            if fail == host and request["model"] == "ppq002":
                raise TimeoutError("write receipt lost")
        if request["op"] == "verify_ball_model" and request.get("model") != states[host]:
            raise ValueError("configured model mismatch")
        return {"model": states[host], "runtime": "VERIFIED"}
    manager = FieldOperations(tmp_path, exchange, recorder=Mock(status=lambda: {}))
    return manager, states, calls


def test_switch_requires_stopped_and_commits_both_hosts(tmp_path):
    manager, states, calls = field(tmp_path)
    config = SimpleNamespace()
    with pytest.raises(ValueError, match="stop the Runner"):
        manager.command({"op": "set_ball_model", "model": "ppq002"}, config, stopped=False)
    assert calls == []
    for invalid in ("ppq003", "../../etc/passwd", None):
        with pytest.raises(ValueError):
            manager.command({"op": "set_ball_model", "model": invalid}, config, stopped=True)
    manager.command({"op": "set_ball_model", "model": "ppq002"}, config, stopped=True)
    assert states == {"hdu": "ppq002", "laptop": "ppq002"}
    assert manager.status()["ball_model"] == "ppq002"
    assert FieldOperations(tmp_path, manager.exchange).status()["ball_model"] == "ppq002"
    manager.runner_selection(config)
    assert [host for host, r in calls if r["op"] == "verify_ball_model"] == ["laptop", "hdu"]


def test_lost_write_receipt_rolls_back_both_hosts_and_selection(tmp_path):
    manager, states, _ = field(tmp_path, fail="hdu")
    with pytest.raises(TimeoutError):
        manager.command({"op": "set_ball_model", "model": "ppq002"}, SimpleNamespace(), stopped=True)
    assert states == {"hdu": "berkeley", "laptop": "berkeley"}
    assert manager.status()["ball_model"] is None
    assert not (tmp_path / "selection.json").exists()


def test_failed_rollback_is_visible_and_never_adopts_new_selection(tmp_path):
    manager, states, _ = field(tmp_path, fail="hdu")
    original = manager.exchange
    def exchange(host, config, request):
        if request["op"] == "select_ball_model" and request.get("model") == "berkeley":
            raise OSError("offline during rollback")
        return original(host, config, request)
    manager.exchange = exchange
    with pytest.raises(ValueError, match="rollback unconfirmed"):
        manager.command({"op": "set_ball_model", "model": "ppq002"}, SimpleNamespace(), stopped=True)
    assert manager.status()["ball_model"] is None

    with pytest.raises(ValueError, match="configured model mismatch"):
        manager.runner_selection(SimpleNamespace())


def test_absent_selection_preserves_site_defaults_and_kernel_does_not_require_ball_physics(tmp_path):
    path = tmp_path / "physics.json"
    assert handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "model": None},
                  read_live=lambda *_: pytest.fail("no explicit ball override")) == {
                      "model": None, "runtime": "UNSELECTED"}
    assert not path.exists()
    manager, _, calls = field(tmp_path)
    manager.command({"op": "set_mode", "mode": "KERNEL"}, SimpleNamespace(), stopped=True)
    assert manager.runner_selection(SimpleNamespace()) == ("KERNEL", "DEFAULT")
    assert calls == []


def test_berkeley_base_configs_keep_original_physics_until_selection():
    import yaml
    expected = profile(CONFIG, "berkeley")["hope_planner"]["ros__parameters"]
    for filename, node in (("model21800_hardware.yaml", "hope_planner"),
                           ("model21800_flight_packetizer.yaml", "hope_ball_flight_packetizer")):
        base = yaml.safe_load((CONFIG / filename).read_text())[node]["ros__parameters"]
        assert {name: base[name] for name in expected} == expected
