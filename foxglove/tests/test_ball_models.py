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
from hope_ball_models import BallModelInspectionCache, inspection_cache_path, inspection_identity, live_parameters

CONFIG = ROOT / "hope_ws/src/hope_planner_cpp/config"


def test_inspection_cache_path_respects_hdu_service_write_allowlist(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "read-only-home")
    assert inspection_cache_path("hdu") == Path("/var/lib/hope-lifecycle/ball-model-cache/hdu.json")
    assert inspection_cache_path("laptop") == tmp_path / "read-only-home/.cache/hope-ball-models/laptop.json"
    with pytest.raises(ValueError, match="unsupported ball model role"):
        inspection_cache_path("unknown")


def test_parameter_check_limits_blas_before_importing_ros(monkeypatch):
    import builtins
    import os
    from hope_ball_models import live_parameters

    variables = ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")
    for variable in variables:
        monkeypatch.setenv(variable, "8")
    original_import = builtins.__import__

    class BeforeRosImport(Exception):
        pass

    def checked_import(name, *args, **kwargs):
        if name == "rclpy":
            assert {key: os.environ[key] for key in variables} == dict.fromkeys(variables, "1")
            raise BeforeRosImport
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", checked_import)
    with pytest.raises(BeforeRosImport):
        live_parameters("hope_planner", ["drag_k"])


@pytest.mark.parametrize("model", PROFILES)
def test_profile_matches_original_fit_receipt_and_round_trips(tmp_path, model):
    document = profile(CONFIG, model)
    receipt = json.loads((CONFIG / PROFILES[model].replace(".yaml", ".receipt.json")).read_text())
    # Archived receipts use the exact adopted parameter dictionary.
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


def field(tmp_path, fail=None, model="ppq002"):
    states = {"hdu": "berkeley", "laptop": "berkeley"}
    calls = []
    def exchange(host, config, request):
        calls.append((host, dict(request)))
        if request["op"] == "read_policy_model":
            return {"catalog_installed": False, "model": None}
        if request["op"] == "select_ball_model":
            states[host] = request["model"]
            if fail == host and request["model"] == model:
                raise TimeoutError("write receipt lost")
        if request["op"] == "verify_ball_model" and request.get("model") != states[host]:
            raise ValueError("configured model mismatch")
        return {"model": states[host], "configured_model": states[host], "runtime": "VERIFIED"}
    manager = FieldOperations(tmp_path, exchange, recorder=Mock(status=lambda: {}))
    return manager, states, calls


@pytest.mark.parametrize("model", ("ppq002", "ppq004"))
def test_switch_requires_stopped_and_commits_both_hosts(tmp_path, model):
    manager, states, calls = field(tmp_path)
    config = SimpleNamespace()
    with pytest.raises(ValueError, match="stop the Runner"):
        manager.command({"op": "set_ball_model", "model": model}, config, stopped=False)
    assert calls == []
    for invalid in ("ppq003", "../../etc/passwd", None):
        with pytest.raises(ValueError):
            manager.command({"op": "set_ball_model", "model": invalid}, config, stopped=True)
    manager.command({"op": "set_ball_model", "model": model}, config, stopped=True)
    assert states == {"hdu": model, "laptop": model}
    assert manager.status()["ball_model"] == model
    assert FieldOperations(tmp_path, manager.exchange).status()["ball_model"] == model
    manager.runner_selection(config)
    assert [host for host, r in calls if r["op"] == "verify_ball_model"] == ["laptop", "hdu"]


@pytest.mark.parametrize("model", ("ppq002", "ppq004"))
def test_lost_write_receipt_rolls_back_both_hosts_and_selection(tmp_path, model):
    manager, states, _ = field(tmp_path, fail="hdu", model=model)
    with pytest.raises(TimeoutError):
        manager.command({"op": "set_ball_model", "model": model}, SimpleNamespace(), stopped=True)
    assert states == {"hdu": "berkeley", "laptop": "berkeley"}
    assert manager.status()["ball_model"] is None
    assert not (tmp_path / "selection.json").exists()


@pytest.mark.parametrize("model", ("ppq002", "ppq004"))
def test_failed_rollback_is_visible_and_never_adopts_new_selection(tmp_path, model):
    manager, states, _ = field(tmp_path, fail="hdu", model=model)
    original = manager.exchange
    def exchange(host, config, request):
        if request["op"] == "select_ball_model" and request.get("model") == "berkeley":
            raise OSError("offline during rollback")
        return original(host, config, request)
    manager.exchange = exchange
    with pytest.raises(ValueError, match="rollback unconfirmed"):
        manager.command({"op": "set_ball_model", "model": model}, SimpleNamespace(), stopped=True)
    assert manager.status()["ball_model"] is None

    with pytest.raises(ValueError, match="configured model mismatch"):
        manager.runner_selection(SimpleNamespace())


def test_legacy_direct_config_is_identified_without_creating_an_override(tmp_path):
    path = tmp_path / "missing.json"
    expected = profile(CONFIG, "ppq004")["hope_planner"]["ros__parameters"]
    base = tmp_path / "base.yaml"
    base.write_text(json.dumps({"hope_planner": {"ros__parameters": expected}}))
    result = handle(path, CONFIG, "hdu", {"op": "inspect_ball_model"}, base_config=base,
                    read_live=lambda *_: expected)
    assert result["model"] is None
    assert result["configured_model"] == result["runtime_model"] == "ppq004"
    assert result["runtime"] == "VERIFIED"
    assert not path.exists()
    stopped = handle(path, CONFIG, "hdu", {"op": "inspect_ball_model"}, base_config=base,
                     read_live=lambda *_: None)
    assert stopped["configured_model"] == "ppq004" and stopped["runtime"] == "NOT_RUNNING"
    old = profile(CONFIG, "ppq002")["hope_planner"]["ros__parameters"]
    mismatch = handle(path, CONFIG, "hdu", {"op": "inspect_ball_model"}, base_config=base,
                      read_live=lambda *_: old)
    assert mismatch["runtime"] == "MISMATCH" and mismatch["runtime_model"] == "ppq002"
    with pytest.raises(ValueError, match="different physics"):
        handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "model": None},
               base_config=base, read_live=lambda *_: old)


def test_status_distinguishes_configuration_loaded_mismatch_and_unavailable(tmp_path):
    endpoints = {h: {"configured_model": "ppq004", "runtime": "NOT_RUNNING"} for h in ("hdu", "laptop")}
    def exchange(host, config, request):
        if request["op"] == "read_policy_model":
            return {"catalog_installed": False, "model": None}
        if isinstance(endpoints[host], Exception):
            raise endpoints[host]
        return endpoints[host]
    manager = FieldOperations(tmp_path, exchange, recorder=Mock(status=lambda: {}))
    read = lambda: manager.refresh_ball_models(SimpleNamespace())["ball_model_status"]
    assert read()["state"] == "CONFIGURED"
    for e in endpoints.values(): e["runtime"] = "VERIFIED"
    assert read()["state"] == "LOADED"
    endpoints["hdu"] = {"configured_model": "ppq002", "runtime": "VERIFIED"}
    assert read()["state"] == "MISMATCH" and read()["configured_model"] is None
    with pytest.raises(ValueError, match="do not match"):
        manager.runner_selection(SimpleNamespace())
    endpoints["hdu"] = TimeoutError("offline")
    status = read()
    assert status["state"] == "UNAVAILABLE"
    assert status["configured_model"] is None and status["endpoints"]["hdu"]["error"] == "offline"


@pytest.fixture
def cached_inspection(tmp_path):
    path = tmp_path / "selection.json"
    select(path, CONFIG, "ppq004")
    clock = SimpleNamespace(wall=1793718000.0, monotonic=100.0)
    identity = {"boot_id": "boot-a", "process": {"pid": 123, "starttime": "10"},
                "config": "config-a", "peer": "192.0.2.1"}
    cache_path = tmp_path / "cache" / "hdu.json"
    def cache():
        return BallModelInspectionCache(cache_path, lambda: dict(identity),
                                        wall_time=lambda: clock.wall, monotonic=lambda: clock.monotonic)
    expected = profile(CONFIG, "ppq004")["hope_planner"]["ros__parameters"]
    read = Mock(return_value=expected)
    def invoke(operation="inspect_ball_model", *, reader=read, store=None):
        return handle(path, CONFIG, "hdu", {"op": operation, "require_running": True},
                      read_live=reader, inspection_cache=store or cache())
    def advance(seconds):
        clock.wall += seconds
        clock.monotonic += seconds
    return SimpleNamespace(path=path, cache_path=cache_path, cache=cache, identity=identity,
                           clock=clock, advance=advance, read=read, invoke=invoke, expected=expected)


def test_inspect_cache_survives_new_helper_without_importing_ros(cached_inspection, monkeypatch):
    import builtins
    c = cached_inspection
    first = c.invoke()
    assert first["runtime"] == "VERIFIED" and not first["verification_cached"]
    c.advance(15)
    original_import = builtins.__import__
    def no_ros(name, *args, **kwargs):
        assert not name.startswith(("rclpy", "rcl_interfaces")), "cache hit imported ROS"
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", no_ros)
    cached = c.invoke(reader=live_parameters)
    assert cached["runtime"] == "CACHED_VERIFIED"
    assert cached["verification_scope"] == "process_identity"
    assert cached["verified_at_unix_s"] == first["verified_at_unix_s"]
    assert cached["verification_age_s"] == 15
    c.read.assert_called_once()
    assert c.cache_path.stat().st_mode & 0o777 == 0o600
    assert c.cache_path.parent.stat().st_mode & 0o777 == 0o700


def test_cache_expires_at_sixty_seconds_without_refreshing_measurement_on_hits(cached_inspection):
    c = cached_inspection
    first = c.invoke()
    for _ in range(3):
        c.advance(15)
        assert c.invoke()["verified_at_unix_s"] == first["verified_at_unix_s"]
    c.advance(15)
    refreshed = c.invoke()
    assert refreshed["runtime"] == "VERIFIED"
    assert refreshed["verified_at_unix_s"] == first["verified_at_unix_s"] + 60
    assert c.read.call_count == 2


@pytest.mark.parametrize("key,value", [("boot_id", "boot-b"), ("config", "config-b"),
    ("peer", "192.0.2.2"), ("process", {"pid": 123, "starttime": "11"}),
    ("process", {"pid": 124, "starttime": "10"})])
def test_changed_identity_or_configuration_bypasses_cache(cached_inspection, key, value):
    c = cached_inspection
    c.invoke()
    c.identity[key] = value
    assert c.invoke()["runtime"] == "VERIFIED"
    assert c.read.call_count == 2


def test_ambiguous_or_unreadable_identity_never_caches(cached_inspection):
    c = cached_inspection
    for identify in (lambda: None, Mock(side_effect=OSError("proc unavailable"))):
        c.invoke()
        store = BallModelInspectionCache(c.cache_path, identify)
        assert c.invoke(store=store)["runtime"] == "VERIFIED"
        assert not c.cache_path.exists()


def test_runtime_change_during_real_query_does_not_cache(cached_inspection):
    c = cached_inspection
    def restart(*_):
        c.identity["process"] = {"pid": 124, "starttime": "20"}
        return c.expected
    assert c.invoke(reader=restart)["runtime"] == "VERIFIED"
    assert not c.cache_path.exists()


def test_runtime_change_while_reading_cache_forces_new_query(cached_inspection):
    c = cached_inspection
    c.invoke()
    before = dict(c.identity)
    after = {**before, "boot_id": "boot-b"}
    store = BallModelInspectionCache(c.cache_path, Mock(side_effect=[before, after, after]),
                                     wall_time=lambda: c.clock.wall, monotonic=lambda: c.clock.monotonic)
    assert c.invoke(store=store)["runtime"] == "VERIFIED"
    assert c.read.call_count == 2 and not c.cache_path.exists()


def test_verify_always_reads_live_and_failed_verification_discards_old_success(cached_inspection):
    c = cached_inspection
    c.invoke()
    assert c.invoke("verify_ball_model")["runtime"] == "VERIFIED"
    assert c.read.call_count == 2
    c.read.return_value = {**c.expected, "drag_k": .2}
    with pytest.raises(ValueError, match="different physics"):
        c.invoke("verify_ball_model")
    assert not c.cache_path.exists()
    assert c.invoke()["runtime"] == "MISMATCH"
    assert not c.cache_path.exists()


def test_failed_or_stopped_live_query_cannot_leave_cached_success(cached_inspection):
    c = cached_inspection
    c.invoke()
    with pytest.raises(OSError, match="service unavailable"):
        c.invoke("verify_ball_model", reader=Mock(side_effect=OSError("service unavailable")))
    assert not c.cache_path.exists()
    assert c.invoke(reader=lambda *_: None)["runtime"] == "NOT_RUNNING"
    assert not c.cache_path.exists()


@pytest.mark.parametrize("change", ["json", "schema", "parameters", "permissions", "future", "wall_jump"])
def test_corrupt_or_unsafe_cache_falls_back_to_original_query(cached_inspection, change):
    c = cached_inspection
    c.invoke()
    record = json.loads(c.cache_path.read_text())
    if change == "json":
        c.cache_path.write_text("{")
    elif change == "permissions":
        c.cache_path.chmod(0o644)
    elif change == "future":
        c.clock.monotonic -= 1
    elif change == "wall_jump":
        c.clock.wall += 10
    else:
        if change == "schema": record["schema"] = -1
        if change == "parameters": record["result"]["configured_parameters"] = {}
        c.cache_path.write_text(json.dumps(record))
    assert c.invoke()["runtime"] == "VERIFIED"
    assert c.read.call_count == 2


def test_cache_symlink_is_not_followed_and_write_failure_does_not_break_verify(cached_inspection, monkeypatch):
    c = cached_inspection
    c.invoke()
    protected = c.cache_path.parent / "original.json"
    c.cache_path.rename(protected)
    original = protected.read_bytes()
    c.cache_path.symlink_to(protected)
    assert c.invoke()["runtime"] == "VERIFIED"
    assert protected.read_bytes() == original and not c.cache_path.is_symlink()
    monkeypatch.setattr("hope_ball_models.atomic_write", Mock(side_effect=OSError("disk full")))
    assert c.invoke("verify_ball_model")["runtime"] == "VERIFIED"
    assert not c.cache_path.exists()


def test_select_clears_diagnostic_cache(cached_inspection):
    c = cached_inspection
    c.invoke()
    handle(c.path, CONFIG, "hdu", {"op": "select_ball_model", "model": "ppq002"}, inspection_cache=c.cache())
    assert not c.cache_path.exists()


def test_real_identity_checks_process_generation_duplicates_config_and_network(tmp_path, monkeypatch):
    import shutil
    proc, config = tmp_path / "proc", tmp_path / "config"
    config.mkdir()
    for name in [*PROFILES.values(), "model21800_hardware.yaml"]:
        shutil.copy2(CONFIG / name, config / name)
    boot = proc / "sys/kernel/random/boot_id"
    boot.parent.mkdir(parents=True)
    boot.write_text("boot-a")
    binary = tmp_path / "planner-binary"
    binary.write_bytes(b"original")
    def process(pid, starttime):
        directory = proc / str(pid)
        directory.mkdir(exist_ok=True)
        (directory / "cmdline").write_bytes(b"/install/hope_planner_cpp_node\0--ros-args\0")
        (directory / "stat").write_text(f"{pid} (name with spaces) S " + "0 " * 18 + str(starttime))
        if not (directory / "exe").exists(): (directory / "exe").symlink_to(binary)
    process(123, 10)
    override = tmp_path / "override.json"
    key = lambda: inspection_identity(override, config, "hdu", proc=proc)
    original = key()
    assert original["process"]["starttime"] == "10"
    process(123, 11)
    assert key() != original
    process(124, 10)
    assert key() is None
    shutil.rmtree(proc / "124")
    before = key()
    override.write_text("{}")
    assert key() != before
    before = key()
    with (config / PROFILES["ppq004"]).open("a") as stream: stream.write("\n# changed\n")
    assert key() != before
    before = key()
    monkeypatch.setenv("ROS_STATIC_PEERS", "192.0.2.2")
    assert key() != before
    before = key()
    monkeypatch.setenv("ROS_DOMAIN_ID", "218")
    assert key() != before
    before = key()
    binary.write_bytes(b"replacement")
    assert key() != before
    before = key()
    boot.write_text("boot-b")
    assert key() != before


def test_probe_disables_unnecessary_services_before_node_creation(monkeypatch):
    from types import ModuleType
    import os
    rclpy, parameters, services = ModuleType("rclpy"), ModuleType("rclpy.parameter"), ModuleType("rcl_interfaces.srv")
    node, client = Mock(), Mock()
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(values=[.12])
    client.call_async.return_value = future
    client.wait_for_service.return_value = True
    node.create_client.return_value = client
    node.get_node_names_and_namespaces.return_value = [("hope_planner", "/")]
    parameters.Parameter = lambda name, *, value: SimpleNamespace(name=name, value=value)
    parameters.parameter_value_to_python = lambda value: value
    services.GetParameters = SimpleNamespace(Request=lambda **kwargs: SimpleNamespace(**kwargs))
    def create_node(name, **kwargs):
        assert name == "hope_ball_model_check"
        assert kwargs["start_parameter_services"] is False and kwargs["enable_rosout"] is False
        assert [(p.name, p.value) for p in kwargs["parameter_overrides"]] == [("start_type_description_service", False)]
        assert all(os.environ[k] == "1" for k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"))
        rclpy.init.assert_called_once_with(args=[])
        return node
    rclpy.init, rclpy.shutdown, rclpy.spin_until_future_complete = Mock(), Mock(), Mock()
    rclpy.create_node = create_node
    for name, module in [("rclpy", rclpy), ("rclpy.parameter", parameters), ("rcl_interfaces.srv", services)]:
        monkeypatch.setitem(sys.modules, name, module)
    assert live_parameters("hope_planner", ["drag_k"]) == {"drag_k": .12}
    node.destroy_node.assert_called_once()
    rclpy.shutdown.assert_called_once()
