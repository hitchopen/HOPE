"""ROS-free orchestration for the attended HDU clock-calibration action.

The browser can request one fixed operation only.  This module owns the exact
service ordering from the hardware runbook and accepts no shell text, unit
name, path, or command from Foxglove.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import pwd
import re
import subprocess
import tempfile
import time
from typing import Callable, Mapping, Sequence

from hope_lifecycle_core import LifecycleConfig
from hope_monitor_core import NtpProbeResult, probe_ntp


STATUS_SCHEMA_VERSION = 1
OPERATION_PATTERN = re.compile(r"timecal_[0-9]{8}T[0-9]{6}Z")
SAFE_ACCOUNT_PATTERN = re.compile(r"[a-z_][a-z0-9_-]{0,31}")
SAFE_STATUS_TOKEN = re.compile(r"[A-Z0-9_]+")
STATUS_STATES = {
    "IDLE",
    "RUNNING",
    "INTERRUPTED",
    "REJECTED",
    "COMPLETE",
    "FAILED_SAFE_STOP",
}

HDU_VENDOR_UNITS = (
    "agibot_roudi.service",
    "agibot_top.service",
    "agibot_ui.service",
    "agibot_pm.service",
)
# Stop dependants before their shared transport/process manager. The vendor
# clock installer uses this exact order; stopping RouDi first can strand the
# process-manager shutdown and leave the transaction in MDU_STOP.
HDU_VENDOR_STOP_UNITS = tuple(reversed(HDU_VENDOR_UNITS))
HDU_PTP_UNITS = ("agibot-hdu-ptp4l.service", "agibot-hdu-phc2sys.service")
HDU_FOXGLOVE_UNITS = (
    "hope-monitor.service",
    "hope-foxglove-bridge.service",
    "hope-observer.service",
    "hope-command-proxy.service",
    "hope-foxglove-control-bridge.service",
    "hope-lifecycle-supervisor.service",
)
HDU_SPLIT_CONTROL_UNITS = ("hope-observer.service", "hope-command-proxy.service")
HDU_REQUIRED_CONTROL_UNITS = tuple(
    unit for unit in HDU_FOXGLOVE_UNITS if unit not in HDU_SPLIT_CONTROL_UNITS
)
HDU_CONSOLIDATED_CONTROL_UNIT = "hope-lifecycle-supervisor.service"
HDU_CONSOLIDATED_CONTROL_SCRIPT = "/usr/local/bin/hope_operations.py"
HDU_CONSOLIDATED_CONTROL_GROUP = "/system.slice/hope-lifecycle-supervisor.service"
HDU_CONTROL_PROCESS_PATTERNS = (
    ("OBSERVER", "(^|/)[h]ope_observer[.]py([[:space:]]|$)"),
    ("COMMAND_PROXY", "(^|/)[h]ope_command_proxy[.]py([[:space:]]|$)"),
    ("OPERATIONS", "(^|/)[h]ope_operations[.]py([[:space:]]|$)"),
)
PTP_PROCESSES = ("ptp4l", "phc2sys")
HDU_RUNTIME_PATTERNS = (
    ("RUNNER", "[a]3_deploy_onnx_ref_pingpong"),
    ("PLANNER", "(^|/)hope_planner_cpp_node([[:space:]]|$)"),
    ("BASE_RELAY", "[h]ope_base_pose_transport_relay"),
)
HDU_VENDOR_PATTERNS = (
    ("HAL", "[a]imrt_main_hal"),
    ("ELINK_HAL", "[a]imrt_main .*hal_elink"),
)
# ELink's rc.local-owned transfer is not part of the clock-consumer stop set.
# Its continuing presence must not strand an otherwise stopped calibration;
# this operation never restarts the transfer or reinitializes its UART/IPC.

SYSTEMCTL = "/usr/bin/systemctl"
PGREP = "/usr/bin/pgrep"
SSH = "/usr/bin/ssh"
RUNUSER = "/usr/sbin/runuser"
ENV = "/usr/bin/env"
REMOTE_SUDO = "/usr/bin/sudo"
LIFECYCLE_HELPER = "/usr/local/libexec/hope-lifecycle"
CHRONYC = "/usr/bin/chronyc"
CHRONY_BURST_UNIT = "hope-chrony-burst.service"
TEST = "/usr/bin/test"
SS = "/usr/bin/ss"

SSH_OPTIONS = (
    "-T",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=3",
    "-o", "ConnectionAttempts=1",
    "-o", "ServerAliveInterval=2",
    "-o", "ServerAliveCountMax=2",
)


class CalibrationRejected(RuntimeError):
    """The operation was rejected before any machine state was changed."""


class CalibrationFailure(RuntimeError):
    """The operation failed after state may have changed."""


@dataclass(frozen=True)
class ControlPlaneSnapshot:
    """Only the verified pre-operation workload is stopped and restored."""

    active_units: tuple[str, ...]
    consolidated_group: str = ""


@dataclass(frozen=True)
class CalibrationStatus:
    state: str = "IDLE"
    step: str = "IDLE"
    result: str = "WAITING_FOR_REQUEST"
    operation_id: str = ""
    boot_id: str = ""
    hard_step_attempted: bool = False
    active_hdu_vendor: tuple[str, ...] = ()
    active_hdu_foxglove: tuple[str, ...] = ()


@dataclass(frozen=True)
class CalibrationProgress:
    step: str
    result: str
    hard_step_attempted: bool = False
    active_hdu_vendor: tuple[str, ...] = ()
    active_hdu_foxglove: tuple[str, ...] = ()


def status_for_current_boot(
    saved: CalibrationStatus | None, current_boot_id: str
) -> CalibrationStatus:
    """Keep an interrupted cross-boot operation visible without stale receipts."""

    if saved is None:
        return CalibrationStatus(boot_id=current_boot_id)
    if saved.boot_id == current_boot_id:
        return saved
    if saved.state in {"RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP"}:
        return CalibrationStatus(
            state="REJECTED",
            step="REBOOT_INTERRUPTED",
            result=(
                "PREVIOUS_CALIBRATION_INTERRUPTED_BY_HDU_REBOOT "
                f"previous_state={saved.state} previous_step={saved.step}"
            ),
            operation_id=saved.operation_id,
            boot_id=current_boot_id,
        )
    return CalibrationStatus(boot_id=current_boot_id)


def _status_text(value: object, *, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    cleaned = " ".join(value.split())
    if not cleaned or len(cleaned) > 800:
        raise ValueError(f"{field} must contain 1..800 printable characters")
    return cleaned


def _status_token(value: object, *, field: str) -> str:
    cleaned = _status_text(value, field=field)
    if SAFE_STATUS_TOKEN.fullmatch(cleaned) is None:
        raise ValueError(f"{field} is not a safe status token")
    return cleaned


def _unit_list(value: object, *, allowed: Sequence[str], field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise ValueError(f"{field} must be a list of unit names")
    if len(value) != len(set(value)) or any(item not in allowed for item in value):
        raise ValueError(f"{field} contains an unsupported or duplicate unit")
    return tuple(value)


def status_to_document(status: CalibrationStatus) -> dict[str, object]:
    document = asdict(status)
    document["schema_version"] = STATUS_SCHEMA_VERSION
    document["active_hdu_vendor"] = list(status.active_hdu_vendor)
    document["active_hdu_foxglove"] = list(status.active_hdu_foxglove)
    return document


def status_from_document(document: Mapping[str, object]) -> CalibrationStatus:
    if document.get("schema_version") != STATUS_SCHEMA_VERSION:
        raise ValueError("unsupported time-calibration status schema")
    state = _status_token(document.get("state"), field="state")
    if state not in STATUS_STATES:
        raise ValueError("unsupported time-calibration state")
    step = _status_token(document.get("step"), field="step")
    result = _status_text(document.get("result"), field="result")
    operation_id = document.get("operation_id")
    if not isinstance(operation_id, str):
        raise ValueError("operation_id must be a string")
    if operation_id and OPERATION_PATTERN.fullmatch(operation_id) is None:
        raise ValueError("invalid time-calibration operation id")
    boot_id = document.get("boot_id")
    if not isinstance(boot_id, str) or not boot_id or len(boot_id) > 80:
        raise ValueError("invalid boot id")
    hard_step_attempted = document.get("hard_step_attempted")
    if not isinstance(hard_step_attempted, bool):
        raise ValueError("hard_step_attempted must be boolean")
    return CalibrationStatus(
        state=state,
        step=step,
        result=result,
        operation_id=operation_id,
        boot_id=boot_id,
        hard_step_attempted=hard_step_attempted,
        active_hdu_vendor=_unit_list(
            document.get("active_hdu_vendor"),
            allowed=HDU_VENDOR_UNITS,
            field="active_hdu_vendor",
        ),
        active_hdu_foxglove=_unit_list(
            document.get("active_hdu_foxglove"),
            allowed=HDU_FOXGLOVE_UNITS,
            field="active_hdu_foxglove",
        ),
    )


def load_status(path: Path) -> CalibrationStatus | None:
    if not path.exists():
        return None
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("time-calibration status must be a JSON object")
    return status_from_document(document)


def save_status_atomic(path: Path, status: CalibrationStatus) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent), text=True
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(status_to_document(status), stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def read_boot_id(path: Path = Path("/proc/sys/kernel/random/boot_id")) -> str:
    value = path.read_text(encoding="ascii").strip()
    if not value or len(value) > 80 or any(character.isspace() for character in value):
        raise ValueError("invalid kernel boot id")
    return value


class TimeCalibrationBackend:
    """Run the exact 10.4 stop, hard-step, and restore sequence."""

    def __init__(
        self,
        robot_user: str,
        *,
        robot_home: str | None = None,
        run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        ntp_probe: Callable[..., NtpProbeResult] = probe_ntp,
        audit: Callable[[str], None] | None = None,
        read_text: Callable[[Path], str] | None = None,
    ):
        if SAFE_ACCOUNT_PATTERN.fullmatch(robot_user) is None:
            raise ValueError("HOPE_ROBOT_USER is not a safe POSIX account name")
        if robot_home is None:
            robot_home = pwd.getpwnam(robot_user).pw_dir
        if not robot_home.startswith("/") or any(character.isspace() for character in robot_home):
            raise ValueError("robot account home must be an absolute path without whitespace")
        self._robot_user = robot_user
        self._robot_home = robot_home
        self._run = run
        self._sleep = sleep
        self._monotonic = monotonic
        self._ntp_probe = ntp_probe
        self._audit = audit if audit is not None else lambda _message: None
        self._read_text = read_text if read_text is not None else (
            lambda path: path.read_text(encoding="utf-8")
        )

    def _record_command(
        self, label: str, completed: subprocess.CompletedProcess[str]
    ) -> None:
        detail = [f"{label} rc={completed.returncode}"]
        if completed.stdout.strip():
            detail.append("stdout=" + completed.stdout.strip()[:4_000])
        if completed.stderr.strip():
            detail.append("stderr=" + completed.stderr.strip()[:4_000])
        self._audit(" | ".join(detail))

    def _call(self, argv: Sequence[str], *, timeout_s: float) -> subprocess.CompletedProcess[str]:
        return self._run(
            list(argv),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_s,
            check=False,
        )

    def _checked(self, argv: Sequence[str], *, timeout_s: float, label: str) -> str:
        try:
            completed = self._call(argv, timeout_s=timeout_s)
        except (OSError, subprocess.SubprocessError) as exc:
            raise CalibrationFailure(f"{label}: {exc}") from exc
        self._record_command(label, completed)
        if completed.returncode != 0:
            detail = completed.stderr.strip().splitlines()
            reason = detail[-1] if detail else f"exit code {completed.returncode}"
            raise CalibrationFailure(f"{label}: {reason[:300]}")
        return completed.stdout

    def _unit_state(self, unit: str) -> tuple[str, str]:
        output = self._checked(
            [SYSTEMCTL, "show", "--property=LoadState", "--property=ActiveState", unit],
            timeout_s=5.0, label=f"query unit {unit}",
        )
        fields = {}
        for line in output.splitlines():
            key, separator, value = line.partition("=")
            if not separator or key in fields:
                raise CalibrationFailure(f"HDU_UNIT_STATE_UNKNOWN: {unit}")
            fields[key] = value
        if (set(fields) != {"LoadState", "ActiveState"}
                or fields["LoadState"] not in {"loaded", "not-found"}
                or fields["ActiveState"] not in {
                    "active", "inactive", "failed", "activating", "deactivating", "reloading"}):
            raise CalibrationFailure(f"HDU_UNIT_STATE_UNKNOWN: {unit}")
        return fields["LoadState"], fields["ActiveState"]

    def _is_active(self, unit: str) -> bool:
        return self._unit_state(unit) == ("loaded", "active")

    def _active_loaded_units(self, units: Sequence[str]) -> tuple[str, ...]:
        active: list[str] = []
        for unit in units:
            loaded, state = self._unit_state(unit)
            if state in {"activating", "deactivating", "reloading"}:
                raise CalibrationFailure(f"HDU_UNIT_TRANSITIONING: {unit}")
            if loaded == "loaded" and state == "active":
                active.append(unit)
        return tuple(active)

    def _consolidated_owner_properties(self) -> dict[str, str]:
        properties = ("ExecStart", "MainPID", "ControlGroup", "KillMode")
        output = self._checked(
            [SYSTEMCTL, "show", *(f"--property={name}" for name in properties),
             HDU_CONSOLIDATED_CONTROL_UNIT],
            timeout_s=5.0, label="query consolidated control owner",
        )
        fields: dict[str, str] = {}
        for line in output.splitlines():
            key, separator, value = line.partition("=")
            if not separator or key in fields:
                raise CalibrationRejected("HDU_CONSOLIDATED_OWNER_UNKNOWN")
            fields[key] = value
        if (set(fields) != set(properties)
                or not fields["MainPID"].isdigit() or int(fields["MainPID"]) <= 0
                or fields["ControlGroup"] != HDU_CONSOLIDATED_CONTROL_GROUP
                or fields["KillMode"] != "control-group"
                or re.search(r"(?<![\w./-])" + re.escape(HDU_CONSOLIDATED_CONTROL_SCRIPT)
                             + r"(?=$|[\s;'\"])", fields["ExecStart"]) is None):
            raise CalibrationRejected("HDU_CONSOLIDATED_OWNER_UNVERIFIED")
        return fields

    def _verified_consolidated_group(self) -> str:
        # On consolidated HDUs the systemd MainPID is the DDS shell wrapper.
        # Prove the real Python child, not just a description/ExecStart token,
        # belongs to the one service that will be stopped and restored.
        before = self._consolidated_owner_properties()
        group = before["ControlGroup"]
        root = Path("/sys/fs/cgroup") / group.lstrip("/")
        try:
            pids = self._read_text(root / "cgroup.procs").split()
            if not pids or len(pids) > 256 or any(not pid.isdigit() or int(pid) <= 0 for pid in pids):
                raise CalibrationRejected("HDU_CONSOLIDATED_OWNER_PIDS_UNKNOWN")
            for pid in pids:
                process = Path("/proc") / pid
                try:
                    argv = self._read_text(process / "cmdline").rstrip("\0").split("\0")
                    if (len(argv) < 2 or argv[1] != HDU_CONSOLIDATED_CONTROL_SCRIPT
                            or re.fullmatch(r"python(?:3(?:\.\d+)?)?", Path(argv[0]).name) is None):
                        continue
                    first_stat = self._read_text(process / "stat")
                    first_start = first_stat.rsplit(")", 1)[1].split()[19]
                    if (not first_stat.startswith(f"{pid} (") or not first_start.isdigit()
                            or f"0::{group}" not in self._read_text(process / "cgroup").splitlines()):
                        continue
                    second_stat = self._read_text(process / "stat")
                    if (not second_stat.startswith(f"{pid} (")
                            or second_stat.rsplit(")", 1)[1].split()[19] != first_start
                            or self._read_text(process / "cmdline").rstrip("\0").split("\0") != argv
                            or f"0::{group}" not in self._read_text(process / "cgroup").splitlines()):
                        continue
                except (OSError, UnicodeError, IndexError):
                    continue  # A short-lived child can exit during enumeration.
                if (self._consolidated_owner_properties() != before
                        or self._unit_state(HDU_CONSOLIDATED_CONTROL_UNIT) != ("loaded", "active")):
                    raise CalibrationRejected("HDU_CONSOLIDATED_OWNER_CHANGED")
                return group
        except (OSError, UnicodeError) as exc:
            raise CalibrationRejected(f"HDU_CONSOLIDATED_OWNER_UNREADABLE: {type(exc).__name__}") from exc
        raise CalibrationRejected("HDU_CONSOLIDATED_OWNER_PROCESS_MISSING")

    def _snapshot_control_plane(self) -> ControlPlaneSnapshot:
        states = {unit: self._unit_state(unit) for unit in HDU_FOXGLOVE_UNITS}
        for unit, (loaded, state) in states.items():
            if state in {"activating", "deactivating", "reloading"}:
                raise CalibrationRejected(f"HDU_CONTROL_UNIT_TRANSITIONING: {unit}")
            if loaded == "not-found" and state != "inactive":
                raise CalibrationRejected(f"HDU_CONTROL_UNIT_STATE_UNKNOWN: {unit}")
        missing = [unit for unit in HDU_REQUIRED_CONTROL_UNITS if states[unit] != ("loaded", "active")]
        if missing:
            raise CalibrationRejected("CONTROL_PLANE_NOT_FULLY_ACTIVE: " + ",".join(missing))
        active = tuple(unit for unit in HDU_FOXGLOVE_UNITS if states[unit] == ("loaded", "active"))
        if active == HDU_FOXGLOVE_UNITS:
            return ControlPlaneSnapshot(active)
        # An absent/inactive compatibility stub is not a missing role when the
        # verified operations process owns observer, proxy and lifecycle.
        return ControlPlaneSnapshot(active, self._verified_consolidated_group())

    def _remote_mdu(self, config: LifecycleConfig, action: str, *, timeout_s: float) -> str:
        if action not in {
            "time-calibration-preflight-mdu",
            "time-calibration-recover-mdu",
            "time-calibration-stop-mdu",
            "time-calibration-verify-stopped-mdu",
            "time-calibration-restore-mdu",
        }:
            raise ValueError("unsupported MDU time-calibration action")
        host = f"{self._robot_user}@{config.mdu_internal_ip}"
        argv = [
            RUNUSER,
            "--user", self._robot_user,
            "--",
            ENV,
            f"HOME={self._robot_home}",
            f"USER={self._robot_user}",
            f"LOGNAME={self._robot_user}",
            SSH,
            *SSH_OPTIONS,
            host,
            REMOTE_SUDO,
            "-n",
            LIFECYCLE_HELPER,
            action,
        ]
        try:
            completed = self._call(argv, timeout_s=timeout_s)
        except (OSError, subprocess.SubprocessError) as exc:
            raise CalibrationFailure(f"MDU {action}: {exc}") from exc
        self._record_command(f"MDU {action}", completed)
        if completed.returncode != 0:
            output = "\n".join((completed.stdout, completed.stderr))
            structured = re.findall(r"\breason=([A-Z0-9_]+)\b", output)
            if structured:
                reason = structured[-1]
                pre_mutation_reasons = {
                    "RUNNER_PRESENT",
                    "NON_VENDOR_HAL_PRESENT",
                    "NON_VENDOR_ELINK_PRESENT",
                    "ALREADY_RUNNING",
                    "ROOT_REQUIRED",
                    "INVALID_SUDO_USER",
                    "INVALID_ARGUMENTS",
                    "MDU_BASELINE_RECEIPT_MISSING",
                    "MDU_BASELINE_BOOT_CHANGED",
                    "MDU_BASELINE_NOT_STOPPED",
                }
                if (
                    action == "time-calibration-preflight-mdu"
                    or (action in {"time-calibration-recover-mdu", "time-calibration-stop-mdu"}
                        and reason in pre_mutation_reasons)
                ):
                    normalized = reason if reason.startswith("MDU_") else f"MDU_{reason}"
                    raise CalibrationRejected(normalized)
                raise CalibrationFailure(f"MDU {action}: {reason}")
            detail = completed.stderr.strip().splitlines()
            reason = detail[-1] if detail else f"exit code {completed.returncode}"
            raise CalibrationFailure(f"MDU {action}: {reason[:300]}")
        return completed.stdout

    def _stop_units(self, units: Sequence[str], *, label: str) -> None:
        if units:
            self._checked(
                [SYSTEMCTL, "stop", *units], timeout_s=120.0, label=label
            )

    def _start_units(self, units: Sequence[str], *, label: str) -> None:
        if units:
            self._checked(
                [SYSTEMCTL, "start", *units], timeout_s=120.0, label=label
            )

    def _wait_processes(self, *, present: bool, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while True:
            matches = tuple(
                self._process_present("-x", process)
                for process in PTP_PROCESSES
            )
            if (all(matches) if present else not any(matches)):
                return
            if self._monotonic() >= deadline:
                state = "appear" if present else "exit"
                raise CalibrationFailure(
                    f"HDU PTP workers did not {state} within {timeout_s:.0f}s"
                )
            self._sleep(0.5 if not present else 1.0)

    def _wait_units_active(self, units: Sequence[str], *, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while True:
            inactive = [unit for unit in units if not self._is_active(unit)]
            if not inactive:
                return
            if self._monotonic() >= deadline:
                raise CalibrationFailure(
                    "units did not become active: " + ",".join(inactive)
                )
            self._sleep(0.5)

    def _wait_control_port(self, *, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while True:
            completed = self._call([SS, "-lnt"], timeout_s=5.0)
            if completed.returncode == 0 and re.search(r":8766\s", completed.stdout):
                return
            if self._monotonic() >= deadline:
                raise CalibrationFailure("Foxglove control port 8766 did not return")
            self._sleep(0.5)

    def _process_present(self, mode: str, pattern: str) -> bool:
        completed = self._call([PGREP, mode, pattern], timeout_s=3.0)
        if completed.returncode not in (0, 1):
            raise CalibrationFailure(f"HDU_PROCESS_PROBE_UNKNOWN: {pattern}")
        return completed.returncode == 0

    def _preflight_ptp_units(self) -> tuple[str, ...]:
        states = [self._unit_state(unit) for unit in HDU_PTP_UNITS]
        if all(state == ("loaded", "active") for state in states):
            return HDU_PTP_UNITS
        if all(state == ("not-found", "inactive") for state in states):
            # Existing field installations can still have vendor-owned PTP.
            # A failed query or a partially installed service pair is NOT
            # evidence of that topology and must never select this fallback.
            if self._is_active("agibot_pm.service"):
                return ()
        raise CalibrationRejected("HDU_PTP_UNIT_NOT_ACTIVE: incomplete or unverified PTP topology")

    def _require_hdu_consumers_stopped(
        self, ptp_units: Sequence[str], control: ControlPlaneSnapshot
    ) -> None:
        # A successful stop command alone is not proof that consumers exited.
        # Check immediately before the single step, after the MDU recheck.
        for unit in (*HDU_VENDOR_UNITS, *HDU_FOXGLOVE_UNITS, *HDU_PTP_UNITS, "chrony.service"):
            loaded, state = self._unit_state(unit)
            if unit in HDU_PTP_UNITS and unit not in ptp_units:
                if (loaded, state) != ("not-found", "inactive"):
                    raise CalibrationFailure(f"HDU_PTP_TOPOLOGY_CHANGED: {unit}")
                continue
            optional_control = unit in HDU_SPLIT_CONTROL_UNITS and unit not in control.active_units
            if ((loaded != "loaded" and unit not in HDU_VENDOR_UNITS and not optional_control)
                    or state not in {"inactive", "failed"}):
                raise CalibrationFailure(f"HDU_CLOCK_CONSUMER_NOT_STOPPED: {unit}")
            if loaded == "not-found" and state != "inactive":
                raise CalibrationFailure(f"HDU_CLOCK_CONSUMER_STATE_UNKNOWN: {unit}")
        if control.consolidated_group:
            events_path = Path("/sys/fs/cgroup") / control.consolidated_group.lstrip("/") / "cgroup.events"
            try:
                events = dict(line.split() for line in self._read_text(events_path).splitlines())
            except FileNotFoundError:
                events = {"populated": "0"}  # systemd removed the stopped service's cgroup.
            except (OSError, UnicodeError, ValueError) as exc:
                raise CalibrationFailure("HDU_CONSOLIDATED_GROUP_STATE_UNKNOWN") from exc
            if events.get("populated") != "0":
                raise CalibrationFailure("HDU_CONSOLIDATED_GROUP_NOT_EMPTY")
        for label, pattern in (*HDU_RUNTIME_PATTERNS, *HDU_VENDOR_PATTERNS, *HDU_CONTROL_PROCESS_PATTERNS):
            if self._process_present("-f", pattern):
                raise CalibrationFailure(f"HDU_{label}_REMAINS")
        for process in (*PTP_PROCESSES, "chronyd"):
            if self._process_present("-x", process):
                raise CalibrationFailure(f"HDU_CLOCK_PROCESS_REMAINS: {process}")

    def preflight(self, config: LifecycleConfig) -> tuple[str, ...]:
        if config.revision < 1:
            raise CalibrationRejected("CONFIG_NOT_CONFIRMED")
        if not self._is_active("chrony.service"):
            raise CalibrationRejected("CHRONY_NOT_ACTIVE")
        if self._call(
            [SYSTEMCTL, "cat", CHRONY_BURST_UNIT], timeout_s=5.0
        ).returncode != 0:
            raise CalibrationRejected("CHRONY_BURST_UNIT_MISSING")
        for label, pattern in HDU_RUNTIME_PATTERNS:
            if self._process_present("-f", pattern):
                raise CalibrationRejected(f"HDU_{label}_IS_RUNNING")
        ptp_units = self._preflight_ptp_units()
        missing_ptp = [
            process
            for process in PTP_PROCESSES
            if not self._process_present("-x", process)
        ]
        if missing_ptp:
            raise CalibrationRejected(
                "HDU_PTP_WORKERS_NOT_RUNNING: " + ",".join(missing_ptp)
            )
        result = self._ntp_probe(max_offset_ms=10.0, max_skew_ppm=0.0)
        if result.error:
            raise CalibrationRejected(f"NTP_PROBE_UNAVAILABLE: {result.error[:240]}")
        if result.gate_pass:
            raise CalibrationRejected(
                f"CLOCK_ALREADY_QUALIFIED: offset={result.offset_ms:+.3f}ms "
                f"skew_audit={result.skew_ppm:.3f}ppm"
            )
        self._remote_mdu(
            config, "time-calibration-recover-mdu", timeout_s=180.0
        )
        self._remote_mdu(
            config, "time-calibration-preflight-mdu", timeout_s=20.0
        )
        return ptp_units

    def _restore_control_plane(self, units: Sequence[str]) -> None:
        available: list[str] = []
        errors: list[str] = []
        for unit in units:
            try:
                if self._unit_state(unit)[0] != "loaded":
                    errors.append(f"HDU_CONTROL_RESTORE_UNIT_MISSING: {unit}")
                else:
                    available.append(unit)
            except (CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
                errors.append(f"HDU_CONTROL_RESTORE_UNIT_UNKNOWN: {unit}: {exc}")
        # Do not let a disappeared captured unit suppress restoration of the
        # surviving diagnostics. Still report failure, never COMPLETE, and do
        # not reset/start any absent or previously inactive compatibility unit.
        if available:
            try:
                self._checked(
                    [SYSTEMCTL, "reset-failed", *available],
                    timeout_s=20.0,
                    label="reset Foxglove control units",
                )
            except (CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
                errors.append(str(exc))
            try:
                self._start_units(available, label="restore Foxglove control plane")
                self._wait_units_active(available, timeout_s=30.0)
            except (CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
                errors.append(str(exc))
        needs_consolidated = (
            set(HDU_REQUIRED_CONTROL_UNITS).issubset(units)
            and not set(HDU_SPLIT_CONTROL_UNITS).issubset(units)
        )
        if needs_consolidated and HDU_CONSOLIDATED_CONTROL_UNIT in available:
            try:
                self._wait_consolidated_owner(timeout_s=30.0)
            except CalibrationFailure as exc:
                errors.append(str(exc))
        try:
            self._wait_control_port(timeout_s=30.0)
        except (CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
            errors.append(str(exc))
        if errors:
            raise CalibrationFailure("; ".join(errors))

    def _wait_consolidated_owner(self, *, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while True:
            try:
                self._verified_consolidated_group()
                return
            except (CalibrationRejected, CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
                # Active shell wrappers can precede the actual Python child.
                # This is readiness after mutation, never an admission rejection.
                if self._monotonic() >= deadline:
                    raise CalibrationFailure(f"HDU_CONSOLIDATED_OWNER_NOT_RESTORED: {exc}") from exc
            self._sleep(0.5)

    def fail_safe(
        self,
        config: LifecycleConfig,
        *,
        active_hdu_foxglove: Sequence[str],
    ) -> tuple[str, ...]:
        """Best-effort safe stop: keep robot services down, restore diagnostics."""

        errors: list[str] = []
        try:
            self._remote_mdu(config, "time-calibration-stop-mdu", timeout_s=120.0)
        except Exception as exc:  # noqa: BLE001 - collect every recovery failure
            errors.append(f"MDU_STOP={exc}")
        try:
            active_vendor = self._active_loaded_units(HDU_VENDOR_UNITS)
            active_vendor_set = set(active_vendor)
            stop_order = tuple(
                unit for unit in HDU_VENDOR_STOP_UNITS if unit in active_vendor_set
            )
            self._stop_units(stop_order, label="fail-safe stop HDU vendor services")
        except Exception as exc:  # noqa: BLE001
            errors.append(f"HDU_VENDOR_STOP={exc}")
        for unit in reversed(HDU_PTP_UNITS):
            try:
                if self._unit_state(unit) == ("not-found", "inactive"):
                    continue  # Explicitly absent on the legacy vendor topology.
                self._stop_units((unit,), label="fail-safe stop HDU PTP service")
            except Exception as exc:  # noqa: BLE001
                errors.append(f"HDU_PTP_STOP={unit}:{exc}")
        try:
            self._wait_processes(present=False, timeout_s=10.0)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"HDU_PTP_STOP={exc}")
        try:
            self._checked(
                [SYSTEMCTL, "start", "chrony.service"],
                timeout_s=20.0,
                label="fail-safe restore chrony",
            )
        except Exception as exc:  # noqa: BLE001
            errors.append(f"CHRONY_RESTORE={exc}")
        for label, argv in (
            (
                "clock bootstrap diagnostic",
                [
                    SYSTEMCTL,
                    "status",
                    "agibot-clock-bootstrap.service",
                    "--no-pager",
                ],
            ),
            ("chrony diagnostic", [CHRONYC, "tracking"]),
        ):
            try:
                completed = self._call(argv, timeout_s=10.0)
                self._record_command(label, completed)
            except Exception as exc:  # noqa: BLE001 - diagnostics are best effort
                errors.append(f"DIAGNOSTIC={label}:{exc}")
        try:
            self._restore_control_plane(active_hdu_foxglove)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"CONTROL_RESTORE={exc}")
        return tuple(errors)

    def calibrate(
        self,
        config: LifecycleConfig,
        progress_callback: Callable[[CalibrationProgress], None],
        *,
        handoff_delay_s: float = 3.0,
    ) -> None:
        """Perform one calibration attempt and restore the pre-operation state."""

        active_vendor: tuple[str, ...] = ()
        active_foxglove: tuple[str, ...] = ()
        active_ptp: tuple[str, ...] = ()
        control = ControlPlaneSnapshot(())
        hard_step_attempted = False
        mutated = False

        def report(step: str, result: str) -> None:
            progress_callback(
                CalibrationProgress(
                    step=step,
                    result=result,
                    hard_step_attempted=hard_step_attempted,
                    active_hdu_vendor=active_vendor,
                    active_hdu_foxglove=active_foxglove,
                )
            )

        try:
            self._sleep(handoff_delay_s)
            report("PREFLIGHT", "VERIFYING_STOPPED_SYSTEM_AND_BAD_CLOCK")
            active_ptp = self.preflight(config)
            active_vendor = self._active_loaded_units(HDU_VENDOR_UNITS)
            control = self._snapshot_control_plane()
            active_foxglove = control.active_units
            report("PREFLIGHT", "PRECONDITIONS_CONFIRMED")
        except CalibrationRejected:
            raise
        except Exception as exc:
            raise CalibrationRejected(
                f"PREFLIGHT_ERROR: {type(exc).__name__}: {exc}"
            ) from exc

        try:
            mutated = True
            report("MDU_STOP", "STOPPING_MDU_TIME_CONSUMERS")
            self._remote_mdu(config, "time-calibration-stop-mdu", timeout_s=120.0)

            report("HDU_STOP", "STOPPING_HDU_CONTROL_AND_VENDOR_SERVICES")
            self._stop_units(active_foxglove, label="stop Foxglove control plane")
            active_vendor_set = set(active_vendor)
            stop_order = tuple(
                unit for unit in HDU_VENDOR_STOP_UNITS if unit in active_vendor_set
            )
            self._stop_units(stop_order, label="stop HDU vendor services")
            # PTP workers are independently supervised; stopping agibot_pm
            # does not stop them. Otherwise this path waits forever/fails,
            # or a restarted PTP worker distributes the HDU clock step live.
            self._stop_units(tuple(reversed(active_ptp)), label="stop HDU PTP services")
            self._wait_processes(present=False, timeout_s=10.0)
            self._checked(
                [SYSTEMCTL, "stop", "chrony.service"],
                timeout_s=20.0,
                label="stop chrony",
            )

            self._checked(
                [SYSTEMCTL, "reset-failed", "agibot-clock-bootstrap.service"],
                timeout_s=20.0,
                label="reset clock bootstrap",
            )
            report("VERIFY_STOPPED", "RECHECKING_HDU_AND_MDU_CLOCK_CONSUMERS")
            self._remote_mdu(config, "time-calibration-verify-stopped-mdu", timeout_s=20.0)
            self._require_hdu_consumers_stopped(active_ptp, control)
            hard_step_attempted = True
            report("HARD_STEP", "STARTING_SINGLE_HDU_UTC_HARD_STEP")
            self._checked(
                [SYSTEMCTL, "restart", "agibot-clock-bootstrap.service"],
                timeout_s=120.0,
                label="run clock bootstrap",
            )
            self._checked(
                [TEST, "-e", "/run/agibot-time/bootstrap-qualified"],
                timeout_s=5.0,
                label="verify clock bootstrap receipt",
            )
            self._checked(
                [SYSTEMCTL, "start", "chrony.service"],
                timeout_s=20.0,
                label="start chrony",
            )
            report("HARD_STEP", "REQUESTING_CHRONY_BURST")
            self._checked(
                [SYSTEMCTL, "start", CHRONY_BURST_UNIT],
                timeout_s=10.0,
                label="chrony burst",
            )
            report("HARD_STEP", "WAITING_FOR_CHRONY_10MS")
            self._checked(
                [CHRONYC, "waitsync", "120", "0.010", "0", "1"],
                timeout_s=140.0,
                label="chrony waitsync",
            )
            qualified = self._ntp_probe(max_offset_ms=10.0, max_skew_ppm=0.0)
            if qualified.error or not qualified.gate_pass:
                detail = qualified.error or (
                    f"offset={qualified.offset_ms:+.3f}ms skew={qualified.skew_ppm:.3f}ppm"
                )
                raise CalibrationFailure(f"CLOCK_NOT_QUALIFIED_AFTER_HARD_STEP: {detail}")

            report("HDU_RESTORE", "RESTORING_HDU_VENDOR_PTP_CHAIN")
            if active_ptp:
                self._start_units(active_ptp, label="restore HDU PTP services")
                self._wait_units_active(active_ptp, timeout_s=30.0)
            else:
                self._start_units(active_vendor, label="restore HDU vendor-owned PTP services")
            self._wait_processes(present=True, timeout_s=90.0)
            if active_ptp:
                self._start_units(active_vendor, label="restore HDU vendor services")

            report("MDU_RESTORE", "RESTORING_MDU_PTP_AND_VENDOR_SERVICES")
            self._remote_mdu(config, "time-calibration-restore-mdu", timeout_s=180.0)

            report("CONTROL_RESTORE", "RESTORING_FOXGLOVE_CONTROL_PLANE")
            self._restore_control_plane(active_foxglove)
            final = self._ntp_probe(max_offset_ms=10.0, max_skew_ppm=0.0)
            if final.error or not final.gate_pass:
                detail = final.error or (
                    f"offset={final.offset_ms:+.3f}ms skew={final.skew_ppm:.3f}ppm"
                )
                raise CalibrationFailure(f"FINAL_CLOCK_CHECK_FAILED: {detail}")
            report(
                "COMPLETE",
                f"CLOCK_QUALIFIED offset={final.offset_ms:+.3f}ms "
                f"skew_audit={final.skew_ppm:.3f}ppm SERVICES_RESTORED",
            )
        except CalibrationRejected:
            raise
        except Exception as exc:
            if not mutated:
                raise
            recovery_errors = self.fail_safe(
                config, active_hdu_foxglove=active_foxglove
            )
            suffix = (
                " | fail-safe errors: " + "; ".join(recovery_errors)
                if recovery_errors
                else " | robot services stopped; chrony and Foxglove control restored"
            )
            raise CalibrationFailure(f"{exc}{suffix}") from exc
