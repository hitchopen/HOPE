"""Read-only historical records and verified startup failure regressions."""
import json
from pathlib import Path
import sys
import pytest
import subprocess

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_elink_fault import PersistentElinkFault, startup_log_fault


def test_empty_history_creates_no_files_or_mutation_api(tmp_path):
    reader = PersistentElinkFault(tmp_path / "fault.json")
    assert reader.reason == ""
    assert not list(tmp_path.iterdir())
    for name in ("arm", "latch", "disarm"):
        assert not hasattr(reader, name)


def test_legacy_fault_remains_readable_and_byte_identical(tmp_path):
    path = tmp_path / "fault.json"
    original = json.dumps({"schema": 1, "reason": "ELINK_HAL_MISSING"}).encode()
    path.write_bytes(original)
    for _ in range(2):
        assert PersistentElinkFault(path).reason == "ELINK_HAL_MISSING"
        assert path.read_bytes() == original


def test_legacy_active_record_is_preserved_not_adopted(tmp_path):
    path = tmp_path / "fault.json"
    active = path.with_name(path.name + ".active")
    original = b'{"schema":1,"session_id":"prior-session"}'
    active.write_bytes(original)
    reader = PersistentElinkFault(path)
    assert reader.reason == "ELINK_RECOVERY_REQUIRED_AFTER_SUPERVISOR_RESTART"
    assert active.read_bytes() == original
    assert not path.exists()


@pytest.mark.parametrize("contents", ["not json", "[]", '{}', "x" * 4097])
def test_invalid_history_is_diagnostic_and_unchanged(tmp_path, contents):
    path = tmp_path / "fault.json"
    path.write_text(contents)
    assert PersistentElinkFault(path).reason == "ELINK_FAULT_RECORD_UNREADABLE"
    assert path.read_text() == contents


def test_symlink_record_is_not_followed_or_changed(tmp_path):
    target = tmp_path / "target"
    target.write_text('{"schema":1,"reason":"other-file"}')
    link = tmp_path / "fault.json"
    link.symlink_to(target)
    assert PersistentElinkFault(link).reason == "ELINK_FAULT_RECORD_UNREADABLE"
    assert link.is_symlink()
    assert json.loads(target.read_text())["reason"] == "other-file"


def test_old_startup_markers_do_not_hide_reset_or_crash():
    good = "load config file: session\nEnter ElinkKeepAliveLoop\nsubscriber success\n"
    assert startup_log_fault(good) == ""
    for failure in ("is not alive,will reset", "Invalid session", "DestoryElink",
                    "SIGSEGV", "Segmentation fault"):
        assert startup_log_fault(good + failure) == "ELINK_FAILURE_AFTER_READY"
        assert startup_log_fault(good + failure + "\nEnter ElinkKeepAliveLoop")


def test_initial_startup_log_only_is_not_misclassified_as_later_reset():
    assert startup_log_fault("is not alive,will reset\nEnter ElinkKeepAliveLoop\n") == ""


def test_actual_shell_readiness_rejects_later_failure(tmp_path):
    root = Path(__file__).resolve().parents[2]
    source = (root / "foxglove/helpers/hope-lifecycle").read_text()
    function = source.split("check_t3_elink_ready() {", 1)[1].split("\n}\n", 1)[0]
    function = "check_t3_elink_ready() {" + function + "\n}\n"
    function = function.replace('/tmp/hope_real/$SESSION_ID/mdu/system', str(tmp_path))
    function = function.replace('/usr/local/lib/hope-foxglove', str(root / 'foxglove/a3'))
    log = tmp_path / "hal_elink.log"
    good = (f"load config file: {tmp_path}/hal_elink_a3_gripper.yaml\n"
            "Enter ElinkKeepAliveLoop\n"
            "subscribe topic '/body_drive/hand_joint_command' success\n"
            "/body_drive/hand_joint_command | pb:aimdk.protocol.HandCommandChannel | ElinkModule | ros2\n")
    script = ('set -euo pipefail\nSESSION_ID=test\n'
              'die() { printf "%s\\n" "$2"; exit 1; }\n'
              'command -v timeout >/dev/null || timeout() { shift; "$@"; }\n'
              'check_elink_process_presence() { :; }\n' + function + '\ncheck_t3_elink_ready TEST\n')
    log.write_text(good)
    assert subprocess.run(['bash', '-c', script], capture_output=True, text=True).returncode == 0
    log.write_text(good + "Invalid session\nSIGSEGV\n")
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == 1
    assert "ELINK_FAILURE_AFTER_READY" in result.stdout


@pytest.mark.parametrize("problem", ["missing_module", "missing_log", "oversized_log", "timeout"])
def test_log_inspection_error_never_claims_observed_hardware_failure(tmp_path, problem):
    root = Path(__file__).resolve().parents[2]
    source = (root / "foxglove/helpers/hope-lifecycle").read_text()
    function = "check_t3_elink_ready() {" + source.split("check_t3_elink_ready() {", 1)[1].split("\n}\n", 1)[0] + "\n}\n"
    function = function.replace('/tmp/hope_real/$SESSION_ID/mdu/system', str(tmp_path))
    module_dir = tmp_path / "missing-module" if problem == "missing_module" else root / "foxglove/a3"
    function = function.replace('/usr/local/lib/hope-foxglove', str(module_dir))
    if problem != "missing_log":
        (tmp_path / "hal_elink.log").write_text("x" * (8 * 1024 * 1024 + 1) if problem == "oversized_log" else "Enter ElinkKeepAliveLoop\n")
    timeout_stub = 'timeout() { return 124; }' if problem == "timeout" else 'timeout() { shift; "$@"; }'
    script = ('set -euo pipefail\nSESSION_ID=test\n'
              'die() { printf "%s\\n" "$2"; exit 1; }\n'
              'check_elink_process_presence() { :; }\n' + timeout_stub + '\n' + function + '\ncheck_t3_elink_ready TEST\n')
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert "ELINK_LOG_INSPECTION_UNAVAILABLE" in result.stdout
    assert "ELINK_FAILURE_AFTER_READY" not in result.stdout


@pytest.mark.parametrize("step,exit_code,reason", [
    ("PREFLIGHT", 10, "ELINK_OBSERVATION_UNKNOWN"),
    ("PREFLIGHT", 11, "ELINK_VENDOR_STACK_NOT_READY"),
    ("HAL", 11, "ELINK_PROCESS_FAULT"),
    ("RUNNER_HAL", 11, "ELINK_VENDOR_STACK_NOT_READY"),
    ("HAL", 1, "ELINK_OBSERVATION_TOOL_UNAVAILABLE"),
    ("HAL", 124, "ELINK_OBSERVATION_TOOL_UNAVAILABLE"),
])
def test_helper_preserves_unknown_fault_and_tool_failure(tmp_path, step, exit_code, reason):
    root = Path(__file__).resolve().parents[2]
    source = (root / "foxglove/helpers/hope-lifecycle").read_text()
    function = "check_elink_process_presence() {" + source.split("check_elink_process_presence() {", 1)[1].split("\n}\n", 1)[0] + "\n}\n"
    script = ('set -euo pipefail\n'
              'die() { printf "%s\\n" "$2"; exit 1; }\n'
              f'timeout() {{ return {exit_code}; }}\n' + function + f'\ncheck_elink_process_presence {step}\n')
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1 and reason in result.stdout
