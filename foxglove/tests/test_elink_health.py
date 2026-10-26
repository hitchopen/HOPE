"""ELink passive observation and fault-latch regression tests (no ROS/robot)."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_elink_health import (  # noqa: E402
    ElinkHealthLatch, PassiveSnapshot, ProbeError, ProcessIdentity,
    collect_passive_snapshot, main, passive_probe_source, run_passive_probe,
)

BOOT = "12345678-1234-1234-1234-123456789abc"


def snapshot():
    return PassiveSnapshot(BOOT, (ProcessIdentity(BOOT, 101, 200),),
                           (ProcessIdentity(BOOT, 102, 300),), True)


def process(root, pid, argv, *, comm="python3", ticks=200, state="S"):
    directory = root / str(pid)
    directory.mkdir()
    # stat fields after comm: state is field 3, starttime is field 22.
    rest = [state] + ["0"] * 18 + [str(ticks)] + ["0"] * 5
    (directory / "stat").write_text(f"{pid} ({comm}) " + " ".join(rest))
    (directory / "cmdline").write_bytes(b"\0".join(arg.encode() for arg in argv) + b"\0")
    (directory / "comm").write_text(comm + "\n")


@pytest.fixture
def proc(tmp_path):
    (tmp_path / "sys/kernel/random").mkdir(parents=True)
    (tmp_path / "sys/kernel/random/boot_id").write_text(BOOT + "\n")
    (tmp_path / "net").mkdir()
    (tmp_path / "net/unix").write_text(
        "Num RefCount Protocol Flags Type St Inode Path\n"
        "0000: 00000002 00000000 00010000 0001 01 123 @elink_stack\n")
    process(tmp_path, 101, ["/usr/bin/python3", "-m", "elink.core"])
    process(tmp_path, 102, ["/agibot/aimrt_main", "--cfg_file_path=/opt/hal_elink_a3_cfg.yaml"],
            comm="hal_elink", ticks=300)
    return tmp_path


def test_real_proc_layout_is_observed_without_claiming_functional_health(proc):
    value = collect_passive_snapshot(proc)
    assert value == snapshot()
    status = ElinkHealthLatch().observe(value, 10)
    assert status["state"] == "OBSERVED"
    assert status["functional_ready"] is False


def test_non_listener_established_socket_is_not_accepted(proc):
    (proc / "net/unix").write_text(
        "Num RefCount Protocol Flags Type St Inode Path\n"
        "0000: 00000003 00000000 00000000 0001 03 123 @elink_stack\n")
    assert collect_passive_snapshot(proc).ipc_listening is False


def test_broad_command_mentions_do_not_count_as_elink_processes(proc):
    process(proc, 103, ["/bin/sh", "-c", "python3 -m elink.core"], comm="sh")
    process(proc, 104, ["/usr/bin/python3", "diagnostic.py", "elink.core"], comm="python3")
    assert len(collect_passive_snapshot(proc).transfer) == 1


def test_aimrt_hal_match_when_comm_not_renamed(proc):
    (proc / "102/comm").write_text("aimrt_main\n")
    assert len(collect_passive_snapshot(proc).hal) == 1


def test_zombie_hal_is_known_failed(proc):
    text = (proc / "102/stat").read_text().replace(") S ", ") Z ")
    (proc / "102/stat").write_text(text)
    value = collect_passive_snapshot(proc)
    status = ElinkHealthLatch().observe(value, 0)
    assert status["state"] == "FAULT"
    assert status["reason"] == "ELINK_HAL_NOT_LIVE"


def test_missing_proc_or_partial_table_is_unknown_not_healthy(tmp_path, proc):
    unknown = collect_passive_snapshot(tmp_path / "does-not-exist")
    assert unknown.errors
    assert ElinkHealthLatch().observe(unknown, 0)["state"] == "UNKNOWN"
    (proc / "102/stat").write_text("truncated")
    partial = collect_passive_snapshot(proc)
    assert partial.errors
    assert ElinkHealthLatch().observe(partial, 0)["state"] == "UNKNOWN"


@pytest.mark.parametrize("change,reason", [
    ({"transfer": ()}, "ELINK_TRANSFER_MISSING"),
    ({"hal": ()}, "ELINK_HAL_MISSING"),
    ({"ipc_listening": False}, "ELINK_IPC_NOT_LISTENING"),
    ({"transfer": (ProcessIdentity(BOOT, 101, 200), ProcessIdentity(BOOT, 103, 400))},
     "ELINK_TRANSFER_AMBIGUOUS"),
])
def test_known_loss_latches_and_late_success_cannot_clear(change, reason):
    latch = ElinkHealthLatch()
    assert latch.observe(snapshot(), 1)["state"] == "OBSERVED"
    assert latch.observe(replace(snapshot(), **change), 2)["reason"] == reason
    for late in (3, 4, 100):
        result = latch.observe(snapshot(), late)
        assert result["state"] == "FAULT" and result["fault_latched"]
        assert result["reason"] == reason
    assert latch.unavailable("timeout", 101)["state"] == "FAULT"


@pytest.mark.parametrize("identity", [
    ProcessIdentity(BOOT, 101, 999),  # PID reused
    ProcessIdentity("87654321-1234-1234-1234-123456789abc", 101, 200),  # reboot
    ProcessIdentity(BOOT, 999, 200),
])
def test_process_generation_change_latches(identity):
    latch = ElinkHealthLatch()
    latch.observe(snapshot(), 0)
    changed = replace(snapshot(), transfer=(identity,))
    assert latch.observe(changed, 1)["reason"] == "ELINK_PROCESS_IDENTITY_CHANGED"


def test_unknown_stale_and_clock_errors_never_report_healthy():
    latch = ElinkHealthLatch(stale_after_s=3)
    assert latch.status(0)["state"] == "UNKNOWN"
    latch.observe(snapshot(), 10)
    assert latch.status(12.999)["state"] == "OBSERVED"
    assert latch.status(13)["state"] == "UNKNOWN"
    assert latch.status(9)["state"] == "UNKNOWN"
    assert latch.status(float("nan"))["state"] == "UNKNOWN"
    assert latch.unavailable("ELINK_OBSERVATION_TIMEOUT", 14)["state"] == "UNKNOWN"
    assert latch.status(14)["functional_ready"] is False


def test_wire_contract_rejects_malformed_boolean_and_identity():
    wire = snapshot().to_dict()
    assert PassiveSnapshot.from_dict(wire) == snapshot()
    for key, value in (("ipc_listening", 1), ("schema", 2), ("errors", "none")):
        with pytest.raises(ProbeError):
            PassiveSnapshot.from_dict({**wire, key: value})
    wire["hal"][0]["pid"] = True
    with pytest.raises(ProbeError):
        PassiveSnapshot.from_dict(wire)


def test_subprocess_timeout_is_bounded_without_vendor_rpc():
    captured = {}
    def run(argv, **kwargs):
        captured.update(kwargs)
        raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
    with pytest.raises(ProbeError, match="TIMEOUT"):
        run_passive_probe(["ssh", "host", "python3", "-"], timeout_s=.2, run=run)
    assert captured["timeout"] == .2
    assert "shell" not in captured
    assert "multiprocessing" not in captured["input"]
    assert "socket(" not in captured["input"]


def test_probe_source_executes_standalone_and_echoes_nonce():
    result = subprocess.run([sys.executable, "-"], input=passive_probe_source("test-nonce"),
                            capture_output=True, text=True, timeout=2, check=True)
    payload = json.loads(result.stdout)
    assert payload["nonce"] == "test-nonce"
    assert payload["snapshot"]["functional_ready"] is False
    PassiveSnapshot.from_dict(payload["snapshot"])


def test_probe_token_and_response_validation():
    def wrong_token(argv, **kwargs):
        return SimpleNamespace(returncode=0, stdout=json.dumps({"nonce": "old", "snapshot": snapshot().to_dict()}))
    with pytest.raises(ProbeError, match="TOKEN_MISMATCH"):
        run_passive_probe(["python3", "-"], run=wrong_token)
    def invalid_output(argv, **kwargs):
        return SimpleNamespace(returncode=0, stdout="x" * 65537)
    with pytest.raises(ProbeError, match="OUTPUT_LIMIT"):
        run_passive_probe(["python3", "-"], run=invalid_output)


def test_actual_local_passive_probe_has_whole_command_deadline():
    # Runs a real child, but only reads local proc (macOS yields UNKNOWN).
    result = run_passive_probe([sys.executable, "-"], timeout_s=2)
    assert result.to_dict()["functional_ready"] is False


def test_no_reset_api_or_robot_actions_in_latch():
    latch = ElinkHealthLatch()
    assert not hasattr(latch, "reset")
    latch.observe(replace(snapshot(), ipc_listening=False), 0)
    assert latch.status(1000)["fault_latched"]


def test_cli_default_preserves_snapshot_json(capsys):
    assert main([], collect=snapshot) == 0
    assert PassiveSnapshot.from_dict(json.loads(capsys.readouterr().out)) == snapshot()


@pytest.mark.parametrize("value,expected_state,expected_exit", [
    (snapshot(), "OBSERVED", 0),
    (replace(snapshot(), ipc_listening=False), "FAULT", 1),
    (replace(snapshot(), errors=("PROCESS_TABLE_PARTIAL",)), "UNKNOWN", 1),
    (replace(snapshot(), hal=()), "FAULT", 1),
])
def test_cli_requires_current_observation_without_functional_claim(capsys, value, expected_state, expected_exit):
    assert main(["--require-observed"], collect=lambda: value, clock=lambda: 10) == expected_exit
    status = json.loads(capsys.readouterr().out)
    assert status["state"] == expected_state
    assert status["functional_ready"] is False


def test_cli_does_not_accept_alternate_proc_root():
    with pytest.raises(SystemExit) as exit_info:
        main(["--proc-root", "/tmp/fake-proc"], collect=snapshot)
    assert exit_info.value.code == 2


@pytest.mark.parametrize("value,expected_exit", [
    (snapshot(), 0),
    (replace(snapshot(), errors=("PROCESS_TABLE_PARTIAL",)), 10),
    (replace(snapshot(), ipc_listening=False), 11),
])
def test_cli_structured_exit_preserves_observation_class(value, expected_exit, capsys):
    assert main(["--require-observed", "--structured-exit"], collect=lambda: value, clock=lambda: 0) == expected_exit
    assert not json.loads(capsys.readouterr().out)["functional_ready"]


def test_cli_tool_error_is_not_a_definite_hardware_fault(capsys):
    def unavailable():
        raise OSError("cannot inspect proc")
    assert main(["--require-observed", "--structured-exit"], collect=unavailable, clock=lambda: 0) == 12
    status = json.loads(capsys.readouterr().out)
    assert status["state"] == "UNKNOWN" and not status["fault_latched"]


def test_vendor_restore_wait_is_bounded_and_can_settle(capsys):
    now, samples = [0.0], iter([replace(snapshot(), hal=()), snapshot()])
    result = main(["--require-observed", "--structured-exit", "--wait-observed-seconds", "1"],
                  collect=lambda: next(samples), clock=lambda: now[0], sleep=lambda delay: now.__setitem__(0, now[0] + delay))
    assert result == 0 and now[0] == 0.25
    capsys.readouterr()
    now[0] = 0
    result = main(["--require-observed", "--structured-exit", "--wait-observed-seconds", "1"],
                  collect=lambda: replace(snapshot(), hal=()), clock=lambda: now[0],
                  sleep=lambda delay: now.__setitem__(0, now[0] + delay))
    assert result == 11 and now[0] == 1
