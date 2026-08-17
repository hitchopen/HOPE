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
    HDU_FOXGLOVE_UNITS,
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
    status_from_document,
    status_to_document,
)


class FakeMachine:
    def __init__(self, *, fail_waitsync=False, remote_stop_reason=None):
        self.calls: list[tuple[str, ...]] = []
        self.active = set(HDU_FOXGLOVE_UNITS) | set(HDU_VENDOR_UNITS) | {
            "chrony.service"
        }
        self.loaded = self.active | {
            "agibot-clock-bootstrap.service",
        }
        self.processes = {"ptp4l", "phc2sys"}
        self.fail_waitsync = fail_waitsync
        self.remote_stop_reason = remote_stop_reason

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
            elif verb == "is-active":
                returncode = 0 if command[-1] in self.active else 3
            elif verb == "stop":
                units = command[2:]
                self.active.difference_update(units)
                if any(unit in HDU_VENDOR_UNITS for unit in units):
                    self.processes.difference_update({"ptp4l", "phc2sys"})
            elif verb == "start":
                units = command[2:]
                self.active.update(units)
                if any(unit in HDU_VENDOR_UNITS for unit in units):
                    self.processes.update({"ptp4l", "phc2sys"})
            elif verb in {"reset-failed", "restart", "status"}:
                pass
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
            if (
                command[-1] == "time-calibration-stop-mdu"
                and self.remote_stop_reason is not None
            ):
                returncode = 1
                stdout = (
                    "HOPE_LIFECYCLE_V1 step=TIME_CALIBRATION state=FAILED "
                    f"reason={self.remote_stop_reason}\n"
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
        offset_ms=125.0,
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
            result="WAITING_FOR_CHRONY_10MS_5PPM",
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
        document = status_to_document(
            CalibrationStatus(boot_id="e2e-test-boot")
        )
        document["state"] = "ARBITRARY"
        with self.assertRaisesRegex(ValueError, "unsupported"):
            status_from_document(document)


class TimeCalibrationOrderingTests(unittest.TestCase):
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

        self.assertLess(mdu_preflight, mdu_stop)
        self.assertLess(mdu_stop, hdu_control_stop)
        self.assertLess(hdu_control_stop, chrony_stop)
        self.assertLess(chrony_stop, bootstrap)
        self.assertLess(bootstrap, waitsync)
        self.assertLess(waitsync, hdu_vendor_restore)
        self.assertLess(hdu_vendor_restore, mdu_restore)
        self.assertLess(mdu_restore, control_restore)
        self.assertEqual(progress[-1].step, "COMPLETE")
        self.assertTrue(progress[-1].hard_step_attempted)
        self.assertEqual(machine.active.intersection(HDU_FOXGLOVE_UNITS), set(HDU_FOXGLOVE_UNITS))
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


if __name__ == "__main__":
    unittest.main()
