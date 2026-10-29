from pathlib import Path
import subprocess
import tempfile
import unittest


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(FOXGLOVE_DIR / "a3"))

from hope_lifecycle_core import LifecycleConfig  # noqa: E402
from hope_monitor_core import NtpProbeResult  # noqa: E402
from hope_time_calibration_core import (  # noqa: E402
    CHRONYC,
    CHRONY_BURST_UNIT,
    HDU_CONSOLIDATED_CONTROL_GROUP,
    HDU_CONSOLIDATED_CONTROL_SCRIPT,
    HDU_CONSOLIDATED_CONTROL_UNIT,
    HDU_CONTROL_PROCESS_PATTERNS,
    HDU_FOXGLOVE_UNITS,
    HDU_REQUIRED_CONTROL_UNITS,
    HDU_SPLIT_CONTROL_UNITS,
    HDU_PTP_UNITS,
    HDU_RUNTIME_PATTERNS,
    HDU_VENDOR_PATTERNS,
    HDU_VENDOR_STOP_UNITS,
    HDU_VENDOR_UNITS,
    LIFECYCLE_HELPER,
    SS,
    SYSTEMCTL,
    CalibrationFailure,
    CalibrationRejected,
    CalibrationStatus,
    TimeCalibrationBackend,
    load_status,
    save_status_atomic,
    status_for_current_boot,
    status_from_document,
    status_to_document,
)


class FakeMachine:
    def __init__(
        self,
        *,
        fail_waitsync=False,
        remote_stop_reason=None,
        remote_reasons=None,
        legacy_ptp=False,
        consolidated=False,
        split_units_loaded=True,
    ):
        self.calls: list[tuple[str, ...]] = []
        self.active = set(HDU_FOXGLOVE_UNITS) | set(HDU_VENDOR_UNITS) | set(HDU_PTP_UNITS) | {
            "chrony.service"
        }
        self.loaded = self.active | {
            "agibot-clock-bootstrap.service",
            CHRONY_BURST_UNIT,
        }
        self.legacy_ptp = legacy_ptp
        if legacy_ptp:
            self.active.difference_update(HDU_PTP_UNITS)
            self.loaded.difference_update(HDU_PTP_UNITS)
        if consolidated:
            self.active.difference_update(HDU_SPLIT_CONTROL_UNITS)
            if not split_units_loaded:
                self.loaded.difference_update(HDU_SPLIT_CONTROL_UNITS)
        self.owner_properties = {
            "ExecStart": "{ path=/bin/bash ; argv[]=/bin/bash -lc exec python3 "
                         + (HDU_CONSOLIDATED_CONTROL_SCRIPT if consolidated
                            else "/usr/local/bin/hope_lifecycle_supervisor.py") + " ; }",
            "MainPID": "2910",
            "ControlGroup": HDU_CONSOLIDATED_CONTROL_GROUP,
            "KillMode": "control-group",
        }
        self.file_overrides = {}
        self.processes = {"ptp4l", "phc2sys", "chronyd"}
        self.fail_waitsync = fail_waitsync
        self.remote_reasons = dict(remote_reasons or {})
        if remote_stop_reason is not None:
            self.remote_reasons["time-calibration-stop-mdu"] = remote_stop_reason

    def read_text(self, path):
        key = str(path)
        if key in self.file_overrides:
            value = self.file_overrides[key]
            if isinstance(value, Exception):
                raise value
            return value
        group_root = "/sys/fs/cgroup" + HDU_CONSOLIDATED_CONTROL_GROUP
        if key == group_root + "/cgroup.procs":
            return "2910\n3566\n545145\n"  # Shell MainPID, Python child, exited helper.
        if key == group_root + "/cgroup.events":
            return "populated " + ("1" if HDU_CONSOLIDATED_CONTROL_UNIT in self.active else "0") + "\nfrozen 0\n"
        if key == "/proc/2910/cmdline":
            return "/bin/bash\0wrapper\0python3 " + HDU_CONSOLIDATED_CONTROL_SCRIPT + "\0"
        if key == "/proc/3566/cmdline":
            return "python3\0" + HDU_CONSOLIDATED_CONTROL_SCRIPT + "\0"
        if key == "/proc/3566/cgroup":
            return "0::" + HDU_CONSOLIDATED_CONTROL_GROUP + "\n"
        if key == "/proc/3566/stat":
            return "3566 (python3) " + " ".join(["S"] + ["0"] * 18 + ["12345"] + ["0"] * 5)
        raise FileNotFoundError(key)

    def __call__(self, argv, **_kwargs):
        command = tuple(argv)
        self.calls.append(command)
        returncode = 0
        stdout = ""
        stderr = ""

        if command[0] == SYSTEMCTL:
            verb = command[1]
            if verb == "cat":
                returncode = 0 if command[2] in self.loaded else 1
            elif verb == "show":
                if "--property=ExecStart" in command:
                    stdout = "".join(f"{key}={value}\n" for key, value in self.owner_properties.items())
                else:
                    stdout = ("LoadState=" + ("loaded" if command[-1] in self.loaded else "not-found")
                              + "\nActiveState=" + ("active" if command[-1] in self.active else "inactive") + "\n")
            elif verb == "is-active":
                returncode = 0 if command[-1] in self.active else 3
            elif verb == "stop":
                units = command[2:]
                self.active.difference_update(units)
                # The current PTP units are independent of the vendor PM.
                for unit, process in zip(HDU_PTP_UNITS, ("ptp4l", "phc2sys")):
                    if unit in units:
                        self.processes.discard(process)
                if "chrony.service" in units:
                    self.processes.discard("chronyd")
                if self.legacy_ptp and "agibot_pm.service" in units:
                    self.processes.difference_update(("ptp4l", "phc2sys"))
            elif verb == "start":
                units = command[2:]
                if any(unit not in self.loaded for unit in units):
                    return subprocess.CompletedProcess(list(command), 5, stdout="", stderr="Unit not found")
                self.active.update(units)
                for unit, process in zip(HDU_PTP_UNITS, ("ptp4l", "phc2sys")):
                    if unit in units:
                        self.processes.add(process)
                if "chrony.service" in units:
                    self.processes.add("chronyd")
                if self.legacy_ptp and "agibot_pm.service" in units:
                    self.processes.update(("ptp4l", "phc2sys"))
            elif verb in {"reset-failed", "restart", "status"}:
                if verb == "reset-failed" and any(unit not in self.loaded for unit in command[2:]):
                    returncode, stderr = 5, "Unit not found"
            else:
                raise AssertionError(f"unexpected systemctl command: {command}")
        elif command[0] == "/usr/bin/pgrep":
            if command[1] == "-x":
                returncode = 0 if command[2] in self.processes else 1
            elif command[1] == "-f":
                returncode = 1
            else:
                raise AssertionError(f"unexpected pgrep command: {command}")
        elif LIFECYCLE_HELPER in command:
            remote_reason = self.remote_reasons.get(command[-1])
            if remote_reason is not None:
                returncode = 1
                stdout = (
                    "HOPE_LIFECYCLE_V1 step=TIME_CALIBRATION state=FAILED "
                    f"reason={remote_reason}\n"
                )
            else:
                stdout = "HOPE_LIFECYCLE_V1 step=TIME_CALIBRATION state=COMPLETE reason=OK\n"
        elif command[:2] == (CHRONYC, "waitsync"):
            if self.fail_waitsync:
                returncode = 1
                stderr = "waitsync failed"
        elif command[:2] == (CHRONYC, "tracking"):
            stdout = "Leap status     : Normal\n"
        elif command[0] == "/usr/bin/test":
            pass
        elif command == (SS, "-lnt"):
            stdout = "LISTEN 0 128 0.0.0.0:8766 0.0.0.0:*\n"
        else:
            raise AssertionError(f"unexpected command: {command}")

        return subprocess.CompletedProcess(
            list(command), returncode, stdout=stdout, stderr=stderr
        )


class ProbeSequence:
    def __init__(self, *results):
        self._results = list(results)

    def __call__(self, **_kwargs):
        if not self._results:
            raise AssertionError("unexpected extra NTP probe")
        return self._results.pop(0)


def failing_ntp():
    return NtpProbeResult(
        offset_ms=168.0,
        skew_ppm=8.0,
        root_dispersion_ms=1.0,
        utc_qualified=True,
        gate_pass=False,
    )


def passing_ntp():
    return NtpProbeResult(
        offset_ms=0.5,
        skew_ppm=0.8,
        root_dispersion_ms=0.7,
        utc_qualified=True,
        gate_pass=True,
    )


class TimeCalibrationStatusTests(unittest.TestCase):
    def test_status_round_trip_and_atomic_mode(self):
        status = CalibrationStatus(
            state="RUNNING",
            step="HARD_STEP",
            result="WAITING_FOR_CHRONY_10MS",
            operation_id="timecal_20260817T120000Z",
            boot_id="e2e-test-boot",
            hard_step_attempted=True,
            active_hdu_vendor=("agibot_pm.service",),
            active_hdu_foxglove=HDU_FOXGLOVE_UNITS,
        )
        self.assertEqual(status_from_document(status_to_document(status)), status)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested/status.json"
            save_status_atomic(path, status)
            self.assertEqual(load_status(path), status)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_status_rejects_unknown_restore_units(self):
        document = status_to_document(
            CalibrationStatus(boot_id="e2e-test-boot")
        )
        document["active_hdu_vendor"] = ["arbitrary.service"]
        with self.assertRaisesRegex(ValueError, "unsupported"):
            status_from_document(document)

    def test_cross_boot_running_operation_remains_visible_as_rejected(self):
        saved = CalibrationStatus(
            state="RUNNING",
            step="MDU_STOP",
            result="STOPPING_MDU_TIME_CONSUMERS",
            operation_id="timecal_20260820T150623Z",
            boot_id="previous-boot",
        )

        current = status_for_current_boot(saved, "current-boot")

        self.assertEqual(current.state, "REJECTED")
        self.assertEqual(current.step, "REBOOT_INTERRUPTED")
        self.assertIn(
            "PREVIOUS_CALIBRATION_INTERRUPTED_BY_HDU_REBOOT", current.result
        )
        self.assertEqual(current.operation_id, saved.operation_id)
        self.assertEqual(current.boot_id, "current-boot")
        self.assertFalse(current.hard_step_attempted)
        document = status_to_document(
            CalibrationStatus(boot_id="e2e-test-boot")
        )
        document["state"] = "ARBITRARY"
        with self.assertRaisesRegex(ValueError, "unsupported"):
            status_from_document(document)


class TimeCalibrationOrderingTests(unittest.TestCase):
    def test_consolidated_receipt_restores_only_prior_active_operations_services(self):
        old_units = HDU_REQUIRED_CONTROL_UNITS
        saved = CalibrationStatus(boot_id="same-boot", active_hdu_foxglove=old_units)
        restored = status_from_document(status_to_document(saved))
        self.assertEqual(restored.active_hdu_foxglove, old_units)
        machine = FakeMachine(consolidated=True)
        backend = self.backend(machine)
        backend._restore_control_plane(restored.active_hdu_foxglove)
        starts = [c for c in machine.calls if c[:2] == (SYSTEMCTL, "start")]
        self.assertEqual(len(starts), 1)
        self.assertEqual(starts[0].count("hope-lifecycle-supervisor.service"), 1)
        self.assertNotIn("hope-observer.service", starts[0])
        self.assertNotIn("hope-command-proxy.service", starts[0])

    def test_active_transport_rejects_clock_step_before_mutation(self):
        for process, reason in (("hope-transport-gateway", "HDU_TRANSPORT_GATEWAY_IS_RUNNING"),
                                ("hope-base-pose-transport-relay", "HDU_BASE_RELAY_IS_RUNNING")):
            with self.subTest(process=process):
                machine = FakeMachine()

                def run(argv, **kwargs):
                    if argv[:2] == ["/usr/bin/pgrep", "-f"] and process in argv[-1]:
                        machine.calls.append(tuple(argv))
                        return subprocess.CompletedProcess(argv, 0, "123\n", "")
                    return machine(argv, **kwargs)

                backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=run)
                with self.assertRaisesRegex(CalibrationRejected, reason):
                    backend.preflight(self.config())
                self.assertFalse(any(c[:2] in ((SYSTEMCTL, "stop"), (SYSTEMCTL, "start"),
                                              (SYSTEMCTL, "restart")) for c in machine.calls))

    @staticmethod
    def config():
        return LifecycleConfig(
            laptop_wifi_ip="192.168.10.2",
            hdu_wifi_ip="192.168.10.3",
            mdu_internal_ip="10.42.10.12",
            motive_ip="192.168.100.111",
            revision=1,
        )

    @staticmethod
    def index_of(calls, predicate):
        return next(index for index, call in enumerate(calls) if predicate(call))

    def test_exact_stop_step_restore_order(self):
        machine = FakeMachine()
        progress = []
        backend = TimeCalibrationBackend(
            "agi",
            robot_home="/home/agi",
            run=machine,
            sleep=lambda _seconds: None,
            ntp_probe=ProbeSequence(failing_ntp(), passing_ntp(), passing_ntp()),
        )
        backend.calibrate(self.config(), progress.append, handoff_delay_s=0.0)

        calls = machine.calls
        mdu_recover = self.index_of(
            calls, lambda call: call[-1] == "time-calibration-recover-mdu"
        )
        mdu_preflight = self.index_of(
            calls, lambda call: call[-1] == "time-calibration-preflight-mdu"
        )
        mdu_stop = self.index_of(
            calls, lambda call: call[-1] == "time-calibration-stop-mdu"
        )
        hdu_control_stop = self.index_of(
            calls,
            lambda call: call[:2] == (SYSTEMCTL, "stop")
            and "hope-lifecycle-supervisor.service" in call,
        )
        chrony_stop = self.index_of(
            calls,
            lambda call: call == (SYSTEMCTL, "stop", "chrony.service"),
        )
        bootstrap = self.index_of(
            calls,
            lambda call: call
            == (SYSTEMCTL, "restart", "agibot-clock-bootstrap.service"),
        )
        burst = self.index_of(
            calls,
            lambda call: call
            == (SYSTEMCTL, "start", CHRONY_BURST_UNIT),
        )
        self.assertNotIn((CHRONYC, "burst", "8/8"), calls)
        waitsync = self.index_of(
            calls, lambda call: call[:2] == (CHRONYC, "waitsync")
        )
        hdu_vendor_restore = self.index_of(
            calls,
            lambda call: call[:2] == (SYSTEMCTL, "start")
            and "agibot_pm.service" in call,
        )
        mdu_restore = self.index_of(
            calls, lambda call: call[-1] == "time-calibration-restore-mdu"
        )
        control_restore = self.index_of(
            calls,
            lambda call: call[:2] == (SYSTEMCTL, "start")
            and "hope-lifecycle-supervisor.service" in call,
        )

        self.assertLess(mdu_recover, mdu_preflight)
        self.assertLess(mdu_preflight, mdu_stop)
        self.assertLess(mdu_stop, hdu_control_stop)
        hdu_vendor_stop = next(
            call
            for call in calls
            if call[:2] == (SYSTEMCTL, "stop")
            and "agibot_pm.service" in call
        )
        self.assertEqual(hdu_vendor_stop[2:], HDU_VENDOR_STOP_UNITS)
        self.assertLess(hdu_control_stop, chrony_stop)
        hdu_ptp_stop = self.index_of(calls, lambda call: call ==
            (SYSTEMCTL, "stop", *reversed(HDU_PTP_UNITS)))
        self.assertLess(hdu_control_stop, hdu_ptp_stop)
        self.assertLess(hdu_ptp_stop, chrony_stop)
        final_mdu_check = self.index_of(calls, lambda call: call[-1] == "time-calibration-verify-stopped-mdu")
        self.assertLess(chrony_stop, final_mdu_check)
        self.assertLess(final_mdu_check, bootstrap)
        # A real zero-consumer snapshot, not just successful stop commands.
        final_hdu_checks = calls[final_mdu_check + 1:bootstrap]
        self.assertIn(("/usr/bin/pgrep", "-x", "chronyd"), final_hdu_checks)
        self.assertIn(("/usr/bin/pgrep", "-x", "ptp4l"), final_hdu_checks)
        self.assertEqual(sum(call == (SYSTEMCTL, "restart", "agibot-clock-bootstrap.service") for call in calls), 1)
        self.assertLess(chrony_stop, bootstrap)
        self.assertLess(bootstrap, burst)
        self.assertLess(burst, waitsync)
        self.assertLess(bootstrap, waitsync)
        self.assertLess(waitsync, hdu_vendor_restore)
        hdu_ptp_restore = self.index_of(calls, lambda call: call ==
            (SYSTEMCTL, "start", *HDU_PTP_UNITS))
        self.assertLess(waitsync, hdu_ptp_restore)
        self.assertLess(hdu_ptp_restore, hdu_vendor_restore)
        self.assertLess(hdu_vendor_restore, mdu_restore)
        self.assertLess(mdu_restore, control_restore)
        self.assertEqual(progress[-1].step, "COMPLETE")
        self.assertTrue(progress[-1].hard_step_attempted)
        self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(HDU_FOXGLOVE_UNITS))

    def test_mdu_ptp_preflight_rejection_surfaces_structured_reason(self):
        machine = FakeMachine(
            remote_reasons={
                "time-calibration-preflight-mdu": "MDU_PTP4L_NOT_ACTIVE",
            }
        )
        backend = TimeCalibrationBackend(
            "agi",
            robot_home="/home/agi",
            run=machine,
            sleep=lambda _seconds: None,
            ntp_probe=ProbeSequence(failing_ntp()),
        )

        with self.assertRaisesRegex(CalibrationRejected, "MDU_PTP4L_NOT_ACTIVE"):
            backend.calibrate(
                self.config(), lambda _progress: None, handoff_delay_s=0.0
            )

    def test_unmanaged_transfer_is_outside_clock_stop_and_recovery_set(self):
        # Mirrors rc.local-owned transfer topology: it survives PM shutdown.
        # A host with that process must still complete the attended operation;
        # no process kill, serial reinitialization or transfer restart is added.
        machine = FakeMachine()
        transfer_pattern = "([e]link[.]core|/[e]link/core[.]py)"
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv) == ("/usr/bin/pgrep", "-f", transfer_pattern):
                result.returncode = 0
                result.stdout = "2107\n"
            return result
        progress = []
        self.backend(run, failing_ntp(), passing_ntp(), passing_ntp()).calibrate(
            self.config(), progress.append, handoff_delay_s=0)
        self.assertEqual(progress[-1].step, "COMPLETE")
        self.assertTrue(progress[-1].hard_step_attempted)
        self.assertEqual(machine.active.intersection(HDU_VENDOR_UNITS), set(HDU_VENDOR_UNITS))
        self.assertIn((SYSTEMCTL, "start", *HDU_PTP_UNITS), machine.calls)
        self.assertTrue(any(c[-1] == "time-calibration-restore-mdu" for c in machine.calls))
        self.assertFalse(any("kill" in c[0] or "abox" in " ".join(c) for c in machine.calls))

    def assert_control_operations(self, machine, expected):
        calls = [call for call in machine.calls
                 if call[:2] in ((SYSTEMCTL, "stop"), (SYSTEMCTL, "start"), (SYSTEMCTL, "reset-failed"))
                 and any(unit in call[2:] for unit in HDU_FOXGLOVE_UNITS)]
        self.assertTrue(calls)
        for call in calls:
            self.assertEqual(call[2:], expected, call)

    def test_consolidated_owner_with_inactive_or_absent_stubs_completes_and_restores_only_workload(self):
        for loaded in (False, True):
            for legacy_ptp in (False, True):
                with self.subTest(split_units_loaded=loaded, legacy_ptp=legacy_ptp):
                    machine = FakeMachine(consolidated=True, split_units_loaded=loaded, legacy_ptp=legacy_ptp)
                    progress = []
                    self.backend(machine, failing_ntp(), passing_ntp(), passing_ntp()).calibrate(
                        self.config(), progress.append, handoff_delay_s=0)
                    self.assertEqual(progress[-1].step, "COMPLETE")
                    self.assertEqual(progress[-1].active_hdu_foxglove, HDU_REQUIRED_CONTROL_UNITS)
                    self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(HDU_REQUIRED_CONTROL_UNITS))
                    self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)
                    receipt = CalibrationStatus(boot_id="test-boot", active_hdu_foxglove=progress[-1].active_hdu_foxglove)
                    self.assertEqual(status_from_document(status_to_document(receipt)), receipt)

    def test_consolidated_failure_and_interrupted_restore_do_not_start_or_reset_inactive_stubs(self):
        for loaded in (False, True):
            with self.subTest(loaded=loaded):
                machine = FakeMachine(consolidated=True, split_units_loaded=loaded, fail_waitsync=True)
                with self.assertRaisesRegex(CalibrationFailure, "waitsync failed"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)
                self.assertFalse(machine.active.intersection(HDU_SPLIT_CONTROL_UNITS))
                # The coordinator only persists the prior-active list; it can
                # replay that exact subset after interruption without discovery.
                machine.calls.clear()
                errors = self.backend(machine).fail_safe(
                    self.config(), active_hdu_foxglove=HDU_REQUIRED_CONTROL_UNITS)
                self.assertEqual(errors, ())
                self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_consolidated_restore_waits_for_real_python_child_after_shell_starts(self):
        machine = FakeMachine(consolidated=True)
        restoring = [False]
        missing_reads = [0]
        clock = iter(range(1_000))
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv[:2]) == (SYSTEMCTL, "start") and HDU_CONSOLIDATED_CONTROL_UNIT in argv:
                restoring[0] = True
            return result
        def read_text(path):
            if restoring[0] and str(path) == "/proc/3566/cmdline" and missing_reads[0] < 2:
                missing_reads[0] += 1
                raise FileNotFoundError("child has not started yet")
            return machine.read_text(path)
        progress = []
        backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=run, read_text=read_text,
            sleep=lambda _s: None, monotonic=lambda: next(clock),
            ntp_probe=ProbeSequence(failing_ntp(), passing_ntp(), passing_ntp()))
        backend.calibrate(self.config(), progress.append, handoff_delay_s=0)
        self.assertEqual(missing_reads[0], 2)
        self.assertEqual(progress[-1].step, "COMPLETE")
        self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_missing_or_changed_consolidated_owner_after_restore_is_failure_not_rejection(self):
        for changed in ("child", "owner"):
            with self.subTest(changed=changed):
                machine = FakeMachine(consolidated=True)
                clock = iter(range(0, 1_000, 5))
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if tuple(argv[:2]) == (SYSTEMCTL, "start") and HDU_CONSOLIDATED_CONTROL_UNIT in argv:
                        if changed == "child":
                            machine.file_overrides["/proc/3566/cmdline"] = "python3\0/unrelated.py\0"
                        else:
                            machine.owner_properties["KillMode"] = "process"
                    return result
                progress = []
                backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=run, read_text=machine.read_text,
                    sleep=lambda _s: None, monotonic=lambda: next(clock),
                    ntp_probe=ProbeSequence(failing_ntp(), passing_ntp()))
                with self.assertRaisesRegex(CalibrationFailure, "HDU_CONSOLIDATED_OWNER_NOT_RESTORED"):
                    backend.calibrate(self.config(), progress.append, handoff_delay_s=0)
                self.assertNotIn("COMPLETE", [item.step for item in progress])
                self.assertEqual(machine.calls.count((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)
                self.assertEqual(sum(c[-1] == "time-calibration-stop-mdu" for c in machine.calls), 2)
                self.assertFalse(machine.active.intersection((*HDU_VENDOR_UNITS, *HDU_PTP_UNITS)))
                self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_interrupted_consolidated_restore_rechecks_owner_from_saved_active_subset(self):
        machine = FakeMachine(consolidated=True, split_units_loaded=False)
        machine.file_overrides["/proc/3566/cmdline"] = "python3\0/unrelated.py\0"
        clock = iter(range(0, 1_000, 5))
        backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=machine, read_text=machine.read_text,
            sleep=lambda _s: None, monotonic=lambda: next(clock))
        errors = backend.fail_safe(self.config(), active_hdu_foxglove=HDU_REQUIRED_CONTROL_UNITS)
        self.assertTrue(any("CONTROL_RESTORE=HDU_CONSOLIDATED_OWNER_NOT_RESTORED" in error for error in errors))
        self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)
        self.assert_no_step(machine)

    def test_disappeared_captured_unit_does_not_suppress_surviving_control_restore(self):
        machine = FakeMachine()
        missing = HDU_SPLIT_CONTROL_UNITS[0]
        machine.active.difference_update(HDU_FOXGLOVE_UNITS)
        machine.loaded.remove(missing)
        with self.assertRaisesRegex(CalibrationFailure, "HDU_CONTROL_RESTORE_UNIT_MISSING"):
            self.backend(machine)._restore_control_plane(HDU_FOXGLOVE_UNITS)
        surviving = tuple(unit for unit in HDU_FOXGLOVE_UNITS if unit != missing)
        self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(surviving))
        self.assert_control_operations(machine, surviving)

    def test_captured_unit_disappearing_after_step_restores_other_controls_but_never_completes(self):
        machine = FakeMachine()
        missing = HDU_SPLIT_CONTROL_UNITS[0]
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv) == (SYSTEMCTL, "restart", "agibot-clock-bootstrap.service"):
                machine.loaded.remove(missing)
            return result
        progress = []
        with self.assertRaisesRegex(CalibrationFailure, "HDU_CONTROL_RESTORE_UNIT_MISSING"):
            self.backend(run, failing_ntp(), passing_ntp()).calibrate(self.config(), progress.append, handoff_delay_s=0)
        self.assertNotIn("COMPLETE", [item.step for item in progress])
        self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(HDU_FOXGLOVE_UNITS) - {missing})
        self.assertFalse(machine.active.intersection((*HDU_VENDOR_UNITS, *HDU_PTP_UNITS)))
        for call in machine.calls:
            if call[:2] in ((SYSTEMCTL, "start"), (SYSTEMCTL, "reset-failed")):
                self.assertNotIn(missing, call)

    def test_missing_required_control_role_rejects_before_stop_even_with_integrated_owner(self):
        for unit in HDU_REQUIRED_CONTROL_UNITS:
            with self.subTest(unit=unit):
                machine = FakeMachine(consolidated=True)
                machine.active.remove(unit)
                machine.loaded.remove(unit)
                with self.assertRaisesRegex(CalibrationRejected, "CONTROL_PLANE_NOT_FULLY_ACTIVE"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)
                self.assertFalse(any(c[:2] == (SYSTEMCTL, "stop") for c in machine.calls))
                self.assertFalse(any(c[-1] == "time-calibration-stop-mdu" for c in machine.calls))

    def test_incomplete_split_layout_requires_a_verified_integrated_owner(self):
        for missing in (HDU_SPLIT_CONTROL_UNITS[:1], HDU_SPLIT_CONTROL_UNITS):
            with self.subTest(missing=missing):
                machine = FakeMachine()
                machine.active.difference_update(missing)
                with self.assertRaisesRegex(CalibrationRejected, "HDU_CONSOLIDATED_OWNER_UNVERIFIED"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assertFalse(any(c[:2] == (SYSTEMCTL, "stop") for c in machine.calls))

    def test_verified_integrated_owner_preserves_any_prior_active_compatibility_unit(self):
        machine = FakeMachine(consolidated=True)
        machine.active.add(HDU_SPLIT_CONTROL_UNITS[0])
        expected = tuple(unit for unit in HDU_FOXGLOVE_UNITS if unit in machine.active)
        self.backend(machine, failing_ntp(), passing_ntp(), passing_ntp()).calibrate(
            self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_control_operations(machine, expected)
        self.assertNotIn(HDU_SPLIT_CONTROL_UNITS[1], machine.active)

    def test_failed_inactive_compatibility_stubs_are_not_reset_or_started(self):
        machine = FakeMachine(consolidated=True)
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv[:2]) == (SYSTEMCTL, "show") and argv[-1] in HDU_SPLIT_CONTROL_UNITS:
                result.stdout = "LoadState=loaded\nActiveState=failed\n"
            return result
        self.backend(run, failing_ntp(), passing_ntp(), passing_ntp(), read_text=machine.read_text).calibrate(
            self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_optional_unknown_or_transitioning_layout_rejects_before_stop(self):
        for output, rc in (("LoadState=loaded\nActiveState=activating\n", 0),
                           ("LoadState=loaded\nActiveState=unknown\n", 0), ("", 1)):
            with self.subTest(output=output, rc=rc):
                machine = FakeMachine(consolidated=True)
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if tuple(argv[:2]) == (SYSTEMCTL, "show") and argv[-1] == HDU_SPLIT_CONTROL_UNITS[0]:
                        result.stdout, result.returncode = output, rc
                    return result
                with self.assertRaises(CalibrationRejected):
                    self.backend(run, failing_ntp(), read_text=machine.read_text).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assertFalse(any(c[:2] == (SYSTEMCTL, "stop") for c in machine.calls))
                self.assert_no_step(machine)

    def test_consolidated_owner_requires_real_owned_python_child_not_shell_or_static_claim(self):
        overrides = (
            {"/proc/3566/cmdline": "/bin/bash\0-c\0echo python3 " + HDU_CONSOLIDATED_CONTROL_SCRIPT + "\0"},
            {"/proc/3566/cmdline": "python3\0-c\0" + HDU_CONSOLIDATED_CONTROL_SCRIPT + "\0"},
            {"/proc/3566/cmdline": "python3\0" + HDU_CONSOLIDATED_CONTROL_SCRIPT + ".old\0"},
            {"/proc/3566/cgroup": "0::/user.slice/unmanaged.scope\n"},
            {"/proc/3566/stat": "not a proc stat"},
            {"/proc/3566/cmdline": PermissionError("unreadable matching child")},
        )
        for files in overrides:
            with self.subTest(files=files):
                machine = FakeMachine(consolidated=True)
                machine.file_overrides.update(files)
                with self.assertRaisesRegex(CalibrationRejected, "HDU_CONSOLIDATED_OWNER_PROCESS_MISSING"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_consolidated_owner_process_identity_change_is_not_accepted(self):
        machine = FakeMachine(consolidated=True)
        reads = [0]
        def read_text(path):
            value = machine.read_text(path)
            if str(path) == "/proc/3566/stat":
                reads[0] += 1
                if reads[0] > 1:
                    return value.replace("12345", "54321")
            return value
        with self.assertRaisesRegex(CalibrationRejected, "HDU_CONSOLIDATED_OWNER_PROCESS_MISSING"):
            self.backend(machine, failing_ntp(), read_text=read_text).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_no_step(machine)

    def test_consolidated_owner_metadata_must_be_known_and_control_group_owned(self):
        for key, value in (("MainPID", "0"), ("ControlGroup", "/user.slice/other.scope"),
                           ("KillMode", "process"), ("ExecStart", "/usr/bin/true")):
            with self.subTest(key=key, value=value):
                machine = FakeMachine(consolidated=True)
                machine.owner_properties[key] = value
                with self.assertRaisesRegex(CalibrationRejected, "HDU_CONSOLIDATED_OWNER_UNVERIFIED"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_harmless_optional_installation_changes_do_not_block_or_expand_restore(self):
        for initially_loaded in (False, True):
            with self.subTest(initially_loaded=initially_loaded):
                machine = FakeMachine(consolidated=True, split_units_loaded=initially_loaded)
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if tuple(argv) == (SYSTEMCTL, "stop", "chrony.service"):
                        if initially_loaded:
                            machine.loaded.difference_update(HDU_SPLIT_CONTROL_UNITS)
                        else:
                            machine.loaded.update(HDU_SPLIT_CONTROL_UNITS)
                    return result
                self.backend(run, failing_ntp(), passing_ntp(), passing_ntp(), read_text=machine.read_text).calibrate(
                    self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_optional_active_transitioning_or_unknown_state_prevents_step(self):
        for state, rc in (("active", 0), ("activating", 0), ("deactivating", 0), ("reloading", 0), ("unknown", 0), ("inactive", 1)):
            with self.subTest(state=state, rc=rc):
                machine = FakeMachine(consolidated=True, split_units_loaded=False)
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if ("chrony.service" not in machine.active and tuple(argv[:2]) == (SYSTEMCTL, "show")
                            and argv[-1] == HDU_SPLIT_CONTROL_UNITS[0]):
                        result.stdout, result.returncode = f"LoadState=loaded\nActiveState={state}\n", rc
                    return result
                with self.assertRaises(CalibrationFailure):
                    self.backend(run, failing_ntp(), read_text=machine.read_text).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)
                self.assert_control_operations(machine, HDU_REQUIRED_CONTROL_UNITS)

    def test_previously_active_control_unit_disappearing_prevents_step_and_restore_success(self):
        for unit in (HDU_CONSOLIDATED_CONTROL_UNIT, HDU_SPLIT_CONTROL_UNITS[0]):
            with self.subTest(unit=unit):
                machine = FakeMachine(consolidated=unit == HDU_CONSOLIDATED_CONTROL_UNIT)
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if tuple(argv) == (SYSTEMCTL, "stop", "chrony.service"):
                        machine.loaded.remove(unit)
                    return result
                with self.assertRaisesRegex(CalibrationFailure, "HDU_CLOCK_CONSUMER_NOT_STOPPED.*HDU_CONTROL_RESTORE_UNIT_MISSING"):
                    self.backend(run, failing_ntp(), read_text=machine.read_text).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_consolidated_descendants_or_unknown_group_state_prevent_step(self):
        group_events = "/sys/fs/cgroup" + HDU_CONSOLIDATED_CONTROL_GROUP + "/cgroup.events"
        for value in ("populated 1\nfrozen 0\n", "frozen 0\n", "malformed\n", PermissionError("denied")):
            with self.subTest(value=value):
                machine = FakeMachine(consolidated=True)
                machine.file_overrides[group_events] = value
                with self.assertRaisesRegex(CalibrationFailure, "HDU_CONSOLIDATED_GROUP"):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_removed_empty_consolidated_cgroup_is_accepted_after_stop(self):
        machine = FakeMachine(consolidated=True)
        machine.file_overrides["/sys/fs/cgroup" + HDU_CONSOLIDATED_CONTROL_GROUP + "/cgroup.events"] = FileNotFoundError()
        self.backend(machine, failing_ntp(), passing_ntp(), passing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)

    def test_escaped_role_process_or_unknown_probe_prevents_step(self):
        for _label, pattern in HDU_CONTROL_PROCESS_PATTERNS:
            for rc in (0, 2):
                with self.subTest(pattern=pattern, rc=rc):
                    machine = FakeMachine(consolidated=True)
                    def run(argv, **kwargs):
                        result = machine(argv, **kwargs)
                        if "chrony.service" not in machine.active and tuple(argv) == ("/usr/bin/pgrep", "-f", pattern):
                            result.returncode = rc
                        return result
                    with self.assertRaises(CalibrationFailure):
                        self.backend(run, failing_ntp(), read_text=machine.read_text).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                    self.assert_no_step(machine)

    def test_missing_mdu_recovery_receipt_rejects_without_local_mutation(self):
        machine = FakeMachine(
            remote_reasons={
                "time-calibration-recover-mdu": "MDU_BASELINE_RECEIPT_MISSING",
            }
        )
        backend = TimeCalibrationBackend(
            "agi",
            robot_home="/home/agi",
            run=machine,
            sleep=lambda _seconds: None,
            ntp_probe=ProbeSequence(failing_ntp()),
        )

        with self.assertRaisesRegex(
            CalibrationRejected, "MDU_BASELINE_RECEIPT_MISSING"
        ):
            backend.calibrate(
                self.config(), lambda _progress: None, handoff_delay_s=0.0
            )

        mutating_local_calls = [
            call
            for call in machine.calls
            if call[0] == SYSTEMCTL and call[1] in {"stop", "start", "restart"}
        ]
        self.assertEqual(mutating_local_calls, [])
        self.assertEqual(machine.active.intersection(HDU_VENDOR_UNITS), set(HDU_VENDOR_UNITS))

    def test_hard_step_failure_keeps_robot_services_stopped(self):
        machine = FakeMachine(fail_waitsync=True)
        progress = []
        backend = TimeCalibrationBackend(
            "agi",
            robot_home="/home/agi",
            run=machine,
            sleep=lambda _seconds: None,
            ntp_probe=ProbeSequence(failing_ntp()),
        )
        with self.assertRaisesRegex(CalibrationFailure, "robot services stopped"):
            backend.calibrate(self.config(), progress.append, handoff_delay_s=0.0)

        remote_stop_calls = [
            call for call in machine.calls
            if call[-1] == "time-calibration-stop-mdu"
        ]
        self.assertEqual(len(remote_stop_calls), 2)
        self.assertTrue(progress[-1].hard_step_attempted)
        self.assertIn("chrony.service", machine.active)
        self.assertEqual(machine.active.intersection(HDU_VENDOR_UNITS), set())
        self.assertEqual(machine.active.intersection(HDU_PTP_UNITS), set())
        self.assertEqual(
            machine.active.intersection(HDU_FOXGLOVE_UNITS),
            set(HDU_FOXGLOVE_UNITS),
        )

    def test_mdu_runner_rejection_changes_no_machine_state(self):
        machine = FakeMachine(remote_stop_reason="RUNNER_PRESENT")
        backend = TimeCalibrationBackend(
            "agi",
            robot_home="/home/agi",
            run=machine,
            sleep=lambda _seconds: None,
            ntp_probe=ProbeSequence(failing_ntp()),
        )
        with self.assertRaisesRegex(CalibrationRejected, "MDU_RUNNER_PRESENT"):
            backend.calibrate(self.config(), lambda _progress: None, handoff_delay_s=0.0)

        mutating_local_calls = [
            call for call in machine.calls
            if call[0] == SYSTEMCTL and call[1] in {"stop", "start", "restart"}
        ]
        self.assertEqual(mutating_local_calls, [])
        self.assertEqual(machine.active.intersection(HDU_VENDOR_UNITS), set(HDU_VENDOR_UNITS))
        self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(HDU_FOXGLOVE_UNITS))

    def backend(self, machine, *probes, read_text=None):
        return TimeCalibrationBackend("agi", robot_home="/home/agi", run=machine,
            sleep=lambda _seconds: None, ntp_probe=ProbeSequence(*probes),
            read_text=read_text if read_text is not None else getattr(machine, "read_text", None))

    def assert_no_step(self, machine):
        self.assertNotIn((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service"), machine.calls)

    def test_runtime_process_or_unknown_process_probe_rejects_before_mutation(self):
        for _label, pattern in HDU_RUNTIME_PATTERNS:
            for rc in (0, 2):
                with self.subTest(pattern=pattern, rc=rc):
                    machine = FakeMachine()
                    def run(argv, **kwargs):
                        result = machine(argv, **kwargs)
                        if tuple(argv) == ("/usr/bin/pgrep", "-f", pattern):
                            result.returncode = rc
                        return result
                    with self.assertRaises(CalibrationRejected):
                        self.backend(run).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                    self.assert_no_step(machine)
                    self.assertFalse(any(c[0] == SYSTEMCTL and c[1] in {"stop", "start", "restart"}
                                         for c in machine.calls))

    def test_required_ptp_units_must_be_verified_active(self):
        for unit in HDU_PTP_UNITS:
            with self.subTest(unit=unit):
                machine = FakeMachine()
                machine.active.remove(unit)
                with self.assertRaisesRegex(CalibrationRejected, "HDU_PTP_UNIT_NOT_ACTIVE"):
                    self.backend(machine).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_unknown_unit_state_is_not_treated_as_inactive(self):
        for output, rc in (("", 1), ("", 0), ("LoadState=loaded\nActiveState=unknown\n", 0)):
            with self.subTest(output=output, rc=rc):
                machine = FakeMachine()
                def run(argv, **kwargs):
                    result = machine(argv, **kwargs)
                    if tuple(argv[:2]) == (SYSTEMCTL, "show"):
                        result.stdout, result.returncode = output, rc
                    return result
                with self.assertRaises(CalibrationRejected):
                    self.backend(run).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                self.assert_no_step(machine)

    def test_final_mdu_recheck_failure_prevents_step_and_keeps_safe_stop(self):
        # Even a reason normally classified as pre-mutation rejection must be
        # a failure here: both boards have already stopped their consumers.
        for reason in ("RUNNER_PRESENT", "MDU_VERIFY_BOOT_CHANGED", "PROCESS_PROBE_UNKNOWN"):
            with self.subTest(reason=reason):
                machine = FakeMachine(remote_reasons={"time-calibration-verify-stopped-mdu": reason})
                progress = []
                with self.assertRaisesRegex(CalibrationFailure, reason):
                    self.backend(machine, failing_ntp()).calibrate(self.config(), progress.append, handoff_delay_s=0)
                self.assert_no_step(machine)
                self.assertFalse(progress[-1].hard_step_attempted)
                self.assertFalse(machine.active.intersection((*HDU_VENDOR_UNITS, *HDU_PTP_UNITS)))

    def test_final_local_check_detects_reappearing_consumers_and_probe_errors(self):
        for _label, pattern in (*HDU_RUNTIME_PATTERNS, *HDU_VENDOR_PATTERNS):
            for rc in (0, 2):
                with self.subTest(pattern=pattern, rc=rc):
                    machine = FakeMachine()
                    def run(argv, **kwargs):
                        result = machine(argv, **kwargs)
                        if "chrony.service" not in machine.active and tuple(argv) == ("/usr/bin/pgrep", "-f", pattern):
                            result.returncode = rc
                        return result
                    with self.assertRaises(CalibrationFailure):
                        self.backend(run, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
                    self.assert_no_step(machine)

    def test_final_check_rejects_reactivating_unit_without_a_live_process(self):
        machine = FakeMachine()
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if ("chrony.service" not in machine.active and tuple(argv[:2]) == (SYSTEMCTL, "show")
                    and argv[-1] == HDU_PTP_UNITS[0]):
                result.stdout = "LoadState=loaded\nActiveState=activating\n"
            return result
        with self.assertRaisesRegex(CalibrationFailure, "HDU_CLOCK_CONSUMER_NOT_STOPPED"):
            self.backend(run, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_no_step(machine)

    def test_explicit_resync_of_healthy_clock_steps_once_and_restores_services(self):
        machine = FakeMachine()
        progress = []
        self.backend(machine, passing_ntp(), passing_ntp(), passing_ntp()).calibrate(
            self.config(), progress.append, handoff_delay_s=0)
        self.assertEqual(progress[-1].step, "COMPLETE")
        self.assertTrue(progress[-1].hard_step_attempted)
        self.assertEqual(machine.calls.count(
            (SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)


    def test_bootstrap_failure_does_not_retry_or_restart_robot_services(self):
        machine = FakeMachine()
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv) == (SYSTEMCTL, "restart", "agibot-clock-bootstrap.service"):
                result.returncode = 1
            return result
        with self.assertRaises(CalibrationFailure):
            self.backend(run, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assertEqual(machine.calls.count((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)
        self.assertFalse(machine.active.intersection((*HDU_VENDOR_UNITS, *HDU_PTP_UNITS)))

    def test_restore_rejection_after_step_is_failure_not_unlocking_rejection(self):
        for reason in ("ROOT_REQUIRED", "INVALID_SUDO_USER", "ALREADY_RUNNING"):
            with self.subTest(reason=reason):
                machine = FakeMachine(remote_reasons={"time-calibration-restore-mdu": reason})
                with self.assertRaisesRegex(CalibrationFailure, reason):
                    self.backend(machine, failing_ntp(), passing_ntp()).calibrate(
                        self.config(), lambda _p: None, handoff_delay_s=0)
                self.assertEqual(machine.calls.count((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)
                self.assertFalse(machine.active.intersection((*HDU_VENDOR_UNITS, *HDU_PTP_UNITS)))

    def test_post_step_unqualified_clock_prevents_robot_restore(self):
        machine = FakeMachine()
        with self.assertRaisesRegex(CalibrationFailure, "CLOCK_NOT_QUALIFIED_AFTER_HARD_STEP"):
            self.backend(machine, failing_ntp(), failing_ntp()).calibrate(
                self.config(), lambda _p: None, handoff_delay_s=0)
        self.assertFalse(any(c == (SYSTEMCTL, "start", *HDU_PTP_UNITS) for c in machine.calls))
        self.assertFalse(any(c[-1] == "time-calibration-restore-mdu" for c in machine.calls))

    def test_fail_safe_attempts_ptp_stop_even_when_vendor_query_fails(self):
        machine = FakeMachine()
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv[:2]) == (SYSTEMCTL, "show") and argv[-1] == HDU_VENDOR_UNITS[0]:
                result.returncode = 1
            return result
        errors = self.backend(run).fail_safe(self.config(), active_hdu_foxglove=HDU_FOXGLOVE_UNITS)
        self.assertTrue(any(e.startswith("HDU_VENDOR_STOP=") for e in errors))
        for unit in HDU_PTP_UNITS:
            self.assertIn((SYSTEMCTL, "stop", unit), machine.calls)
        self.assertFalse(machine.active.intersection(HDU_PTP_UNITS))

    def test_verified_legacy_vendor_owned_ptp_still_supports_one_step(self):
        machine = FakeMachine(legacy_ptp=True)
        self.backend(machine, failing_ntp(), passing_ntp(), passing_ntp()).calibrate(
            self.config(), lambda _p: None, handoff_delay_s=0)
        self.assertEqual(machine.calls.count((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)
        self.assertFalse(any(c[0] == SYSTEMCTL and c[1] in {"stop", "start"}
                             and any(unit in c for unit in HDU_PTP_UNITS) for c in machine.calls))
        self.assertTrue({"ptp4l", "phc2sys"}.issubset(machine.processes))

    def test_partial_ptp_install_is_not_misclassified_as_legacy(self):
        machine = FakeMachine()
        machine.active.remove(HDU_PTP_UNITS[0])
        machine.loaded.remove(HDU_PTP_UNITS[0])
        with self.assertRaisesRegex(CalibrationRejected, "unverified PTP topology"):
            self.backend(machine).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_no_step(machine)

    def test_legacy_topology_change_at_final_check_prevents_step(self):
        machine = FakeMachine(legacy_ptp=True)
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if ("chrony.service" not in machine.active and tuple(argv[:2]) == (SYSTEMCTL, "show")
                    and argv[-1] in HDU_PTP_UNITS):
                result.stdout = "LoadState=loaded\nActiveState=inactive\n"
            return result
        with self.assertRaisesRegex(CalibrationFailure, "HDU_PTP_TOPOLOGY_CHANGED"):
            self.backend(run, failing_ntp()).calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_no_step(machine)

    def test_legacy_orphan_ptp_process_prevents_step(self):
        machine = FakeMachine(legacy_ptp=True)
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv[:2]) == ("/usr/bin/pgrep", "-x") and argv[-1] == "ptp4l":
                result.returncode = 0  # Orphan survives the vendor PM stop.
            return result
        clock = [0]
        def advance():
            clock[0] += 11
            return clock[0]
        backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=run,
            sleep=lambda _s: None, monotonic=advance, ntp_probe=ProbeSequence(failing_ntp()))
        with self.assertRaisesRegex(CalibrationFailure, "PTP workers did not exit"):
            backend.calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assert_no_step(machine)

    def test_legacy_vendor_restore_without_ptp_fails_closed(self):
        machine = FakeMachine(legacy_ptp=True)
        def run(argv, **kwargs):
            result = machine(argv, **kwargs)
            if tuple(argv[:2]) == (SYSTEMCTL, "start") and "agibot_pm.service" in argv:
                machine.processes.difference_update(("ptp4l", "phc2sys"))
            return result
        clock = [0]
        def advance():
            clock[0] += 100
            return clock[0]
        backend = TimeCalibrationBackend("agi", robot_home="/home/agi", run=run,
            sleep=lambda _s: None, monotonic=advance, ntp_probe=ProbeSequence(failing_ntp(), passing_ntp()))
        with self.assertRaisesRegex(CalibrationFailure, "PTP workers did not appear"):
            backend.calibrate(self.config(), lambda _p: None, handoff_delay_s=0)
        self.assertEqual(machine.calls.count((SYSTEMCTL, "restart", "agibot-clock-bootstrap.service")), 1)
        self.assertFalse(machine.active.intersection(HDU_VENDOR_UNITS))
        self.assertFalse(any(c[-1] == "time-calibration-restore-mdu" for c in machine.calls))


if __name__ == "__main__":
    unittest.main()
