"""ROS-free control contract for the validated model21800 TTY runner."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import subprocess
import time
from typing import Callable, Sequence


WIRE_SCHEMA_VERSION = 1
COMMAND_PASSIVE = 0
COMMAND_PREPARE = 1
COMMAND_POLICY = 2
COMMAND_NAMES = {
    COMMAND_PASSIVE: "passive",
    COMMAND_PREPARE: "prepare",
    COMMAND_POLICY: "policy",
}

MODE_PASSIVE = 0
MODE_PD_STAND = 1
MODE_MOTION = 3
MODE_IDLE = 6
MODE_STOPPED = 7
MODE_STARTING = 8

STATE_TO_MODE = {
    "PASSIVE": MODE_PASSIVE,
    "PD_RAMP": MODE_PD_STAND,
    "PD_READY": MODE_PD_STAND,
    "MOTION": MODE_MOTION,
    "IDLE": MODE_IDLE,
    "STOPPED": MODE_STOPPED,
    "STARTING": MODE_STARTING,
    "FAILED": MODE_STOPPED,
    "UNKNOWN": MODE_STOPPED,
}
VALID_STATES = frozenset(STATE_TO_MODE)
MAX_EXACT_SEQUENCE = 1 << 52


@dataclass(frozen=True)
class RunnerStatus:
    state: str
    run_id: str
    pid: int
    start_ticks: int
    pd_ticks: int
    request_seq: int
    applied_seq: int
    result: int
    fault: bool
    reason: str

    @property
    def mode(self) -> int:
        return STATE_TO_MODE[self.state]

    @property
    def pd_ready(self) -> bool:
        return self.state == "PD_READY"

    def wire_data(self) -> list[float]:
        return [
            float(WIRE_SCHEMA_VERSION),
            float(self.mode),
            float(self.pd_ready),
            float(self.request_seq),
            float(self.applied_seq),
            float(self.result),
            0.0,  # Stock model21800 does not consume /a3/base_pose_flat.
            float(self.fault),
        ]


def _parse_uint(name: str, value: str, *, maximum: int = MAX_EXACT_SEQUENCE) -> int:
    if not value.isascii() or not value.isdecimal():
        raise ValueError(f"{name} is not an unsigned integer")
    parsed = int(value)
    if parsed < 0 or parsed > maximum:
        raise ValueError(f"{name} is out of range")
    return parsed


def parse_helper_status(output: str) -> RunnerStatus:
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    matches = [line for line in lines if line.startswith("A3CTL_V1 ")]
    if len(matches) != 1:
        raise ValueError("helper must return exactly one A3CTL_V1 status line")
    fields: dict[str, str] = {}
    for token in matches[0].split()[1:]:
        if token.count("=") != 1:
            raise ValueError("malformed helper status token")
        key, value = token.split("=", 1)
        if key in fields:
            raise ValueError(f"duplicate helper status field: {key}")
        fields[key] = value
    expected = {
        "state",
        "run_id",
        "pid",
        "start_ticks",
        "pd_ticks",
        "request_seq",
        "applied_seq",
        "result",
        "fault",
        "reason",
    }
    if set(fields) != expected:
        raise ValueError("helper status fields do not match A3CTL_V1")
    if fields["state"] not in VALID_STATES:
        raise ValueError("unknown helper state")
    run_id = fields["run_id"]
    if run_id != "none" and (
        len(run_id) != 32 or any(ch not in "0123456789abcdef" for ch in run_id)
    ):
        raise ValueError("invalid run_id")
    reason = fields["reason"]
    if not reason or any(ch not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for ch in reason):
        raise ValueError("invalid helper reason")
    try:
        result = int(fields["result"])
    except ValueError as exc:
        raise ValueError("invalid helper result") from exc
    if result < -3 or result > 1:
        raise ValueError("helper result is out of range")
    fault = _parse_uint("fault", fields["fault"], maximum=1)
    return RunnerStatus(
        state=fields["state"],
        run_id=run_id,
        pid=_parse_uint("pid", fields["pid"], maximum=(1 << 31) - 1),
        start_ticks=_parse_uint(
            "start_ticks", fields["start_ticks"], maximum=(1 << 63) - 1
        ),
        pd_ticks=_parse_uint(
            "pd_ticks", fields["pd_ticks"], maximum=(1 << 63) - 1
        ),
        request_seq=_parse_uint("request_seq", fields["request_seq"]),
        applied_seq=_parse_uint("applied_seq", fields["applied_seq"]),
        result=result,
        fault=bool(fault),
        reason=reason,
    )


def decode_mode_command(values: Sequence[float]) -> tuple[int, int]:
    if len(values) != 3 or float(values[0]) != float(WIRE_SCHEMA_VERSION):
        raise ValueError("mode command schema/size mismatch")
    sequence_float = float(values[1])
    code_float = float(values[2])
    if not math.isfinite(sequence_float) or not sequence_float.is_integer():
        raise ValueError("mode command sequence is not an exact integer")
    sequence = int(sequence_float)
    if sequence <= 0 or sequence > MAX_EXACT_SEQUENCE:
        raise ValueError("mode command sequence is out of range")
    if not math.isfinite(code_float) or not code_float.is_integer():
        raise ValueError("mode command code is not an exact integer")
    code = int(code_float)
    if code not in COMMAND_NAMES:
        raise ValueError("unsupported mode command")
    return sequence, code


class SshRunnerClient:
    """Invoke only the fixed helper and its fixed command vocabulary."""

    _ALLOWED_ACTIONS = frozenset({"status", "claim", "start", "key", "stop"})

    def __init__(
        self,
        *,
        host: str = "agi@10.42.10.12",
        helper_path: str = "/agibot/a3_deploy_model21800/hope_model21800_runner.sh",
        timeout_s: float = 3.0,
        run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    ):
        if host != "agi@10.42.10.12":
            raise ValueError("runner SSH host is pinned to the A3 HDU-MDU link")
        if helper_path != "/agibot/a3_deploy_model21800/hope_model21800_runner.sh":
            raise ValueError("runner helper path is pinned to model21800")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self._host = host
        self._helper_path = helper_path
        self._timeout_s = timeout_s
        self._run = run

    @staticmethod
    def _validate_args(args: tuple[str, ...]) -> None:
        if not args or args[0] not in SshRunnerClient._ALLOWED_ACTIONS:
            raise ValueError("unsupported runner helper action")
        action = args[0]
        if action in {"status", "stop"}:
            valid = len(args) == 1
        elif action == "start":
            valid = len(args) == 2 and args[1].isdecimal()
        elif action == "claim":
            valid = (
                len(args) == 3
                and args[1].isdecimal()
                and args[2] in COMMAND_NAMES.values()
            )
        else:
            valid = (
                len(args) == 3
                and args[1] in COMMAND_NAMES.values()
                and args[2].isdecimal()
            )
        if not valid:
            raise ValueError("invalid runner helper arguments")

    def invoke(self, *args: str, timeout_s: float | None = None) -> RunnerStatus:
        self._validate_args(args)
        argv = [
            "/usr/bin/ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=1",
            "-o",
            "ConnectionAttempts=1",
            "-o",
            "ServerAliveInterval=1",
            "-o",
            "ServerAliveCountMax=1",
            self._host,
            self._helper_path,
            *args,
        ]
        completed = self._run(
            argv,
            capture_output=True,
            text=True,
            timeout=self._timeout_s if timeout_s is None else timeout_s,
            check=False,
            env={**os.environ, "LC_ALL": "C"},
        )
        status = parse_helper_status(completed.stdout)
        if completed.returncode != 0 and status.state not in {"STOPPED", "UNKNOWN"}:
            raise RuntimeError(
                f"runner helper failed rc={completed.returncode} reason={status.reason}"
            )
        return status


def execute_mode_command(
    client: SshRunnerClient,
    sequence: int,
    code: int,
    *,
    startup_timeout_s: float = 16.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> RunnerStatus:
    """Claim and execute one command; remote sequence checks resolve races."""

    action = COMMAND_NAMES[code]
    status = client.invoke("claim", str(sequence), action)
    if status.request_seq != sequence or status.result < 0:
        return status

    if action == "prepare":
        deadline = monotonic() + startup_timeout_s
        while status.state in {"STOPPED", "STARTING"}:
            if status.request_seq != sequence or status.result < 0:
                return status
            if status.state == "STOPPED":
                status = client.invoke("start", str(sequence))
            else:
                if monotonic() >= deadline:
                    return status
                sleep(0.2)
                status = client.invoke("status")
        if status.request_seq != sequence:
            return status
        if status.state not in {"IDLE", "PASSIVE", "PD_RAMP", "PD_READY", "MOTION"}:
            return status
        return client.invoke("key", "prepare", str(sequence))

    if action == "passive":
        deadline = monotonic() + startup_timeout_s
        while status.state == "STARTING":
            if status.request_seq != sequence or status.result < 0:
                return status
            if monotonic() >= deadline:
                return status
            sleep(0.2)
            status = client.invoke("status")
        if status.state == "STOPPED":
            return status
    return client.invoke("key", action, str(sequence))


def unavailable_status(reason: str = "ADAPTER_UNAVAILABLE") -> RunnerStatus:
    return RunnerStatus(
        state="UNKNOWN",
        run_id="none",
        pid=0,
        start_ticks=0,
        pd_ticks=0,
        request_seq=0,
        applied_seq=0,
        result=-3,
        fault=True,
        reason=reason,
    )
