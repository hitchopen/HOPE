"""Execute the fixed MDU verification shell with isolated process/unit probes.

No SSH, sudo, systemd or hardware command is run. Only the real helper's clock
guard functions execute; their external probes and receipt paths are replaced.
"""
import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "foxglove/helpers/hope-lifecycle"
SOURCE = HELPER.read_text()
GUARDS = SOURCE.split("time_calibration_require_root() {", 1)[1].split(
    "time_calibration_check_mdu() {", 1)[0]
GUARDS = "time_calibration_require_root() {" + GUARDS
PATTERNS = (
    ("[a]3_deploy_onnx_ref_pingpong", "RUNNER_REMAINS"),
    ("[a]imrt_main_hal", "HAL_REMAINS"),
    ("[a]imrt_main .*hal_elink_a3(_ultra)?_(cfg|gripper)[.]yaml", "ELINK_REMAINS"),
    ("(^|/)[p]tp4l([[:space:]]|$)", "MDU_PTP4L_REMAINS"),
    ("(^|/)[p]hc2sys([[:space:]]|$)", "MDU_PHC2SYS_REMAINS"),
)
TRANSFER_PATTERN = "[e]link[.]core|/elink/[c]ore[.]py"


def run_guard(tmp_path, *, process_pattern="", process_rc=1, unit_state="inactive",
              unit_load="loaded", unit_rc=0, boot_matches=True, receipt_state="STOPPED",
              missing_receipt=False, action="time_calibration_verify_stopped_mdu",
              sudo_user="agi", process_owned=True, unit_target=""):
    receipt = tmp_path / "receipt"
    receipt.mkdir()
    (receipt / "mdu-active-services").write_text("agibot_pm.service\n")
    (receipt / "mdu-boot-id").write_text("test-boot\n" if boot_matches else "other-boot\n")
    if not missing_receipt:
        (receipt / "mdu-state").write_text(receipt_state + "\n")
    boot = tmp_path / "boot"
    boot.write_text("test-boot\n")
    before = {p.name: p.read_bytes() for p in receipt.iterdir()}
    guards = (GUARDS.replace("/usr/bin/pgrep", "mock_pgrep")
              .replace("/usr/bin/systemctl", "mock_systemctl")
              .replace("/run/hope-time-calibration", str(receipt))
              .replace("/proc/sys/kernel/random/boot_id", str(boot)))
    script = r'''
set -euo pipefail
id() { printf '0\n'; }
emit() { printf '%s %s %s\n' "$@"; }
die() { emit "$1" FAILED "$2"; exit 1; }
mock_pgrep() {
  [[ "$1" == -f && "$2" == -- ]] || exit 90
  if [[ "$3" == "$TEST_PROCESS_PATTERN" ]]; then
    [[ "$TEST_PROCESS_RC" != 0 ]] || printf '123\n'
    return "$TEST_PROCESS_RC"
  fi
  return 1
}
mock_systemctl() {
  # Read-only contract: any mutation or different command fails the test.
  [[ "$1" == show && "$2" == --property=LoadState && "$3" == --property=ActiveState ]] || exit 91
  if [[ -n "$TEST_UNIT_TARGET" && "$4" != "$TEST_UNIT_TARGET" ]]; then
    printf 'LoadState=loaded\nActiveState=inactive\n'
    return
  fi
  [[ "$TEST_UNIT_RC" == 0 ]] || return "$TEST_UNIT_RC"
  printf 'LoadState=%s\nActiveState=%s\n' "$TEST_UNIT_LOAD" "$TEST_UNIT_STATE"
}
pid_in_systemd_unit() { [[ "$TEST_PROCESS_OWNED" == yes ]]; }
'''
    environment = dict(os.environ, SUDO_USER=sudo_user,
        TEST_PROCESS_PATTERN=process_pattern, TEST_PROCESS_RC=str(process_rc),
        TEST_UNIT_STATE=unit_state, TEST_UNIT_LOAD=unit_load, TEST_UNIT_RC=str(unit_rc),
        TEST_UNIT_TARGET=unit_target,
        TEST_PROCESS_OWNED="yes" if process_owned else "no")
    result = subprocess.run(["bash", "-c", script + guards + "\n" + action],
                            env=environment, capture_output=True, text=True, timeout=5)
    assert before == {p.name: p.read_bytes() for p in receipt.iterdir()}
    return result


@pytest.mark.parametrize("unit_state", ["inactive", "failed"])
def test_verified_stopped_state_is_read_only(tmp_path, unit_state):
    result = run_guard(tmp_path, unit_state=unit_state)
    assert result.returncode == 0, result.stderr
    assert "MDU_STOPPED_STATE_VERIFIED" in result.stdout


def test_rc_local_transfer_does_not_block_stopped_verification(tmp_path):
    # Real A3 topology: PM/HAL/PTP are stopped, but rc.local's transfer remains.
    # Clock calibration neither owns nor restarts that transfer process.
    result = run_guard(tmp_path, process_pattern=TRANSFER_PATTERN, process_rc=0)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MDU_STOPPED_STATE_VERIFIED" in result.stdout


def test_rc_local_transfer_survives_full_mdu_stop_verify_restore(tmp_path):
    """Execute the actual helper transaction with fake probes/services only.

    Unlike a canned remote success, this exercises STOPPING -> STOPPED ->
    RESTORED with a transfer that always stays alive outside PM ownership.
    All receipt paths and privileged/host commands are replaced in the shell.
    """
    receipt = tmp_path / "receipt"
    receipt.mkdir()
    boot = tmp_path / "boot"
    boot.write_text("test-boot\n")
    functions = "time_calibration_require_root() {" + SOURCE.split(
        "time_calibration_require_root() {", 1)[1].split("prepare_laptop() {", 1)[0]
    for original, replacement in (
            ("/usr/bin/pgrep", "mock_pgrep"),
            ("/usr/bin/systemctl", "mock_systemctl"),
            ("/usr/bin/install", "mock_install"),
            ("/usr/bin/flock", "mock_flock"),
            ("/run/hope-time-calibration", str(receipt)),
            ("/proc/sys/kernel/random/boot_id", str(boot))):
        functions = functions.replace(original, replacement)
    script = r'''
set -euo pipefail
id() { printf '0\n'; }
emit() { printf '%s %s %s\n' "$@"; }
die() { emit "$1" FAILED "$2"; exit 1; }
sleep() { :; }
mock_install() { :; }
mock_flock() { :; }
# macOS Bash 3 lacks mapfile; emulate only this builtin for portable tests.
mapfile() {
  [[ "$1" == -t && "$2" == active_services ]] || exit 93
  local line index=0
  while IFS= read -r line; do
    active_services[$index]="$line"
    ((index += 1))
  done
}
active_units=' agibot_roudi.service agibot_top.service agibot_ui.service agibot_pm.service agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service '
is_active() { [[ "$active_units" == *" $1 "* ]]; }
mock_systemctl() {
  local verb="$1" unit state
  shift
  case "$verb" in
    cat) return 0 ;;
    is-active) is_active "$2" ;;
    show)
      for unit; do :; done
      state=inactive
      if is_active "$unit"; then state=active; fi
      printf 'LoadState=loaded\nActiveState=%s\n' "$state"
      ;;
    stop)
      for unit; do active_units="${active_units/ $unit / }"; done
      ;;
    start)
      for unit; do active_units+=" $unit "; done
      ;;
    *) exit 91 ;;
  esac
}
mock_pgrep() {
  [[ "$1" == -f && "$2" == -- ]] || exit 90
  case "$3" in
    '[e]link[.]core|/elink/[c]ore[.]py') printf '2107\n'; return 0 ;;
    '[a]3_deploy_onnx_ref_pingpong') return 1 ;;
    '[a]imrt_main_hal'|'[a]imrt_main .*hal_elink_a3(_ultra)?_(cfg|gripper)[.]yaml')
      if is_active agibot_pm.service; then printf '123\n'; else return 1; fi ;;
    '(^|/)[p]tp4l([[:space:]]|$)') is_active agibot-mdu-ptp4l.service ;;
    '(^|/)[p]hc2sys([[:space:]]|$)') is_active agibot-mdu-phc2sys.service ;;
    *) exit 92 ;;
  esac
}
pid_in_systemd_unit() { [[ "$2" == agibot_pm.service ]] && is_active agibot_pm.service; }
'''
    transaction = r'''
time_calibration_stop_mdu
[[ "$(< "$TEST_RECEIPT/mdu-state")" == STOPPED ]]
mock_pgrep -f -- '[e]link[.]core|/elink/[c]ore[.]py' >/dev/null
time_calibration_verify_stopped_mdu
time_calibration_restore_mdu
[[ "$(< "$TEST_RECEIPT/mdu-state")" == RESTORED ]]
mock_pgrep -f -- '[e]link[.]core|/elink/[c]ore[.]py' >/dev/null
is_active agibot_pm.service
is_active agibot-mdu-ptp4l.service
is_active agibot-mdu-phc2sys.service
'''
    result = subprocess.run(["bash", "-c", script + functions + transaction],
                            env=dict(os.environ, SUDO_USER="agi", TEST_RECEIPT=str(receipt)),
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MDU_CLOCK_CONSUMERS_STOPPED" in result.stdout
    assert "MDU_STOPPED_STATE_VERIFIED" in result.stdout
    assert "MDU_PTP_AND_VENDOR_RESTORED" in result.stdout


@pytest.mark.parametrize("pattern,reason", PATTERNS)
def test_any_runner_hal_elink_or_ptp_process_blocks_step(tmp_path, pattern, reason):
    result = run_guard(tmp_path, process_pattern=pattern, process_rc=0)
    assert result.returncode == 1 and reason in result.stdout


@pytest.mark.parametrize("pattern,_reason", PATTERNS)
@pytest.mark.parametrize("returncode", [2, 3, 127])
def test_process_observation_error_is_not_absence(tmp_path, pattern, _reason, returncode):
    result = run_guard(tmp_path, process_pattern=pattern, process_rc=returncode)
    assert result.returncode == 1 and "MDU_PROCESS_OBSERVATION_UNKNOWN" in result.stdout


@pytest.mark.parametrize("state", ["active", "activating", "deactivating", "reloading", "refreshing"])
def test_non_stopped_unit_blocks_step(tmp_path, state):
    result = run_guard(tmp_path, unit_state=state)
    assert result.returncode == 1 and "MDU_CLOCK_CONSUMER_UNIT_NOT_STOPPED" in result.stdout


@pytest.mark.parametrize("kwargs", [
    {"unit_state": ""}, {"unit_state": "unknown"}, {"unit_load": "not-found"},
    {"unit_load": "error"}, {"unit_rc": 1}, {"unit_rc": 4},
    {"unit_state": "inactive\nActiveState=inactive"},
])
def test_missing_or_unreadable_units_fail_closed(tmp_path, kwargs):
    result = run_guard(tmp_path, **kwargs)
    assert result.returncode == 1 and "MDU_UNIT_OBSERVATION_UNKNOWN" in result.stdout


@pytest.mark.parametrize("unit", ["agibot_ui.service", "agibot_top.service", "agibot_roudi.service"])
def test_known_optional_vendor_units_may_be_explicitly_absent(tmp_path, unit):
    result = run_guard(tmp_path, unit_load="not-found", unit_target=unit)
    assert result.returncode == 0 and "MDU_STOPPED_STATE_VERIFIED" in result.stdout


@pytest.mark.parametrize("unit", ["agibot_pm.service", "agibot-mdu-phc2sys.service", "agibot-mdu-ptp4l.service"])
def test_required_pm_and_ptp_units_must_exist(tmp_path, unit):
    result = run_guard(tmp_path, unit_load="not-found", unit_target=unit)
    assert result.returncode == 1 and "MDU_UNIT_OBSERVATION_UNKNOWN" in result.stdout


@pytest.mark.parametrize("kwargs", [
    {"unit_rc": 4}, {"unit_load": "error"}, {"unit_load": ""},
    {"unit_load": "not-found", "unit_state": "failed"},
])
def test_optional_unit_query_error_is_not_explicit_absence(tmp_path, kwargs):
    result = run_guard(tmp_path, unit_target="agibot_ui.service", **kwargs)
    assert result.returncode == 1 and "MDU_UNIT_OBSERVATION_UNKNOWN" in result.stdout


@pytest.mark.parametrize("kwargs,reason", [
    ({"missing_receipt": True}, "MDU_STOPPED_RECEIPT_MISSING"),
    ({"boot_matches": False}, "MDU_STOPPED_BOOT_CHANGED"),
    ({"receipt_state": "STOPPING"}, "MDU_NOT_IN_STOPPED_STATE"),
    ({"receipt_state": "RESTORED"}, "MDU_NOT_IN_STOPPED_STATE"),
])
def test_receipt_must_be_same_boot_and_stopped(tmp_path, kwargs, reason):
    result = run_guard(tmp_path, **kwargs)
    assert result.returncode == 1 and reason in result.stdout


def test_verification_rejects_extra_arguments(tmp_path):
    result = run_guard(tmp_path, action="time_calibration_verify_stopped_mdu ignored")
    assert result.returncode == 1 and "INVALID_ARGUMENTS" in result.stdout


def test_verification_requires_sudo_account(tmp_path):
    result = run_guard(tmp_path, sudo_user="")
    assert result.returncode == 1 and "INVALID_SUDO_USER" in result.stdout


@pytest.mark.parametrize("pattern,reason", [
    (PATTERNS[0][0], "RUNNER_PRESENT"),
    (PATTERNS[1][0], "NON_VENDOR_HAL_PRESENT"),
    (PATTERNS[2][0], "NON_VENDOR_ELINK_PRESENT"),
])
def test_idle_preflight_rejects_unmanaged_consumers(tmp_path, pattern, reason):
    result = run_guard(tmp_path, action="time_calibration_require_idle_mdu",
                       process_pattern=pattern, process_rc=0, process_owned=False)
    assert result.returncode == 1 and reason in result.stdout


@pytest.mark.parametrize("pattern,_reason", PATTERNS[:3])
def test_idle_preflight_does_not_treat_pgrep_error_as_stopped(tmp_path, pattern, _reason):
    result = run_guard(tmp_path, action="time_calibration_require_idle_mdu",
                       process_pattern=pattern, process_rc=2)
    assert result.returncode == 1 and "MDU_PROCESS_OBSERVATION_UNKNOWN" in result.stdout


def test_stop_records_success_only_after_strict_verification():
    stop = SOURCE.split("time_calibration_stop_mdu() {", 1)[1].split("time_calibration_restore_mdu() {", 1)[0]
    assert stop.index("time_calibration_require_idle_mdu") < stop.index("systemctl stop")
    assert stop.index("time_calibration_assert_mdu_consumers_stopped") < stop.index("printf 'STOPPED")
    dispatch = SOURCE.split("time-calibration-verify-stopped-mdu)", 1)[1].split(";;", 1)[0]
    assert 'time_calibration_verify_stopped_mdu "$@"' in dispatch


def test_runbook_grants_only_exact_fixed_clock_actions():
    runbook = (ROOT / "docs/operations/foxglove_first_hardware_test.md").read_text()
    for action in ("preflight", "recover", "stop", "verify-stopped", "restore"):
        command = f"/usr/local/libexec/hope-lifecycle time-calibration-{action}-mdu"
        assert shlex.quote(command) in runbook
        assert command + " *" not in runbook
