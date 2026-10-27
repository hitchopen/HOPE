"""Read-only ELink process observation; this is not functional/safety health.

The shipped vendor IPC listener can fail when a peer disconnects during its
authentication handshake.  This module therefore never connects to that
listener, imports the vendor package, opens a serial port, or restarts anything.
An OBSERVED result only records necessary process/listener conditions.  It does
not prove UART activity, fresh BMS/E-stop/hand telemetry, or safe native reset.
"""

from __future__ import annotations

from dataclasses import dataclass
import inspect
import json
import math
from pathlib import Path
import re
import subprocess
import time
from typing import Sequence
import uuid


SCHEMA = 1
KIND = "passive_elink_process_observation"
MAX_OUTPUT_BYTES = 65536
BOOT_ID = re.compile(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}")


class ProbeError(RuntimeError):
    """The observer could not obtain a complete, bounded snapshot."""


@dataclass(frozen=True)
class ProcessIdentity:
    boot_id: str
    pid: int
    start_ticks: int

    def to_dict(self) -> dict:
        return {"boot_id": self.boot_id, "pid": self.pid,
                "start_ticks": self.start_ticks}


@dataclass(frozen=True)
class PassiveSnapshot:
    boot_id: str
    transfer: tuple[ProcessIdentity, ...]
    hal: tuple[ProcessIdentity, ...]
    ipc_listening: bool | None
    errors: tuple[str, ...] = ()

    def to_dict(self) -> dict:
        return {"schema": SCHEMA, "kind": KIND, "boot_id": self.boot_id,
                "transfer": [item.to_dict() for item in self.transfer],
                "hal": [item.to_dict() for item in self.hal],
                "ipc_listening": self.ipc_listening, "errors": list(self.errors),
                "functional_ready": False}

    @classmethod
    def from_dict(cls, value: object) -> PassiveSnapshot:
        if (not isinstance(value, dict) or type(value.get("schema")) is not int or
                value.get("schema") != SCHEMA or value.get("kind") != KIND):
            raise ProbeError("ELINK_SNAPSHOT_SCHEMA_INVALID")
        boot = value.get("boot_id")
        errors = value.get("errors")
        if (not isinstance(errors, list) or len(errors) > 16 or
                any(not isinstance(e, str) or not e or len(e) > 128 for e in errors)):
            raise ProbeError("ELINK_SNAPSHOT_ERRORS_INVALID")
        if not isinstance(boot, str) or not (BOOT_ID.fullmatch(boot) or (boot == "" and errors)):
            raise ProbeError("ELINK_BOOT_ID_INVALID")
        ipc = value.get("ipc_listening")
        if ipc is not None and type(ipc) is not bool:
            raise ProbeError("ELINK_IPC_OBSERVATION_INVALID")
        processes = []
        for name in ("transfer", "hal"):
            items = value.get(name)
            if not isinstance(items, list) or len(items) > 16:
                raise ProbeError("ELINK_PROCESS_LIST_INVALID")
            decoded = []
            for item in items:
                if (not isinstance(item, dict) or item.get("boot_id") != boot or
                        type(item.get("pid")) is not int or item["pid"] <= 0 or
                        type(item.get("start_ticks")) is not int or item["start_ticks"] < 0):
                    raise ProbeError("ELINK_PROCESS_IDENTITY_INVALID")
                decoded.append(ProcessIdentity(boot, item["pid"], item["start_ticks"]))
            if len({p.pid for p in decoded}) != len(decoded):
                raise ProbeError("ELINK_PROCESS_IDENTITY_DUPLICATED")
            processes.append(tuple(decoded))
        if not errors and ipc is None:
            raise ProbeError("ELINK_IPC_OBSERVATION_MISSING")
        return cls(boot, processes[0], processes[1], ipc, tuple(errors))


def _collect_passive_data(proc_root="/proc"):
    """Self-contained proc reader, also sent over SSH as source without install."""
    from pathlib import Path
    import re

    root = Path(proc_root)
    errors = []

    def error(code):
        if code not in errors and len(errors) < 16:
            errors.append(code)

    result = {"schema": 1, "kind": "passive_elink_process_observation",
              "boot_id": "", "transfer": [], "hal": [],
              "ipc_listening": None, "errors": errors,
              "functional_ready": False}
    try:
        boot = (root / "sys/kernel/random/boot_id").read_text().strip()
        if not re.fullmatch(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", boot):
            raise ValueError("invalid boot id")
        result["boot_id"] = boot
    except (OSError, ValueError):
        error("BOOT_ID_UNAVAILABLE")
    try:
        entries = list(root.iterdir())
    except OSError:
        entries = []
        error("PROCESS_TABLE_UNAVAILABLE")
    candidates = [entry for entry in entries if entry.name.isdecimal()]
    if len(candidates) > 32768:
        error("PROCESS_TABLE_LIMIT_EXCEEDED")
        candidates = candidates[:32768]
    for entry in candidates:
        candidate_matched = False
        try:
            # Read stat on both sides of argv/comm to detect exit/PID reuse.
            first = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            argv = (entry / "cmdline").read_bytes().split(b"\0")
            argv = [arg.decode("utf-8", "replace") for arg in argv if arg]
            comm = (entry / "comm").read_text().strip()
            # Match actual vendor executable/module forms, not a broad grep.
            transfer = any(argv[i:i + 2] == ["-m", "elink.core"]
                           for i in range(max(0, len(argv) - 1)))
            if argv and Path(argv[0]).name.startswith("python"):
                transfer = transfer or any(arg.endswith("/elink/core.py") for arg in argv[1:2])
            else:
                transfer = False
            hal = comm == "hal_elink"
            if argv and Path(argv[0]).name == "aimrt_main":
                hal = hal or any(re.search(r"(?:^|/)hal_elink_a3(?:_ultra)?_(?:cfg|gripper)\.yaml$", arg)
                                 for arg in argv[1:])
            candidate_matched = bool(transfer or hal)
            if not candidate_matched:
                continue
            last = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if first[19] != last[19]:
                error("PROCESS_TABLE_CHANGED")
                continue
            for name, matched in (("transfer", transfer), ("hal", hal)):
                if not matched:
                    continue
                if first[0] in {"Z", "X", "x"}:
                    error(name.upper() + "_PROCESS_NOT_LIVE")
                    continue
                if len(result[name]) >= 16:
                    error(name.upper() + "_PROCESS_LIMIT_EXCEEDED")
                    continue
                result[name].append({"boot_id": result["boot_id"],
                                     "pid": int(entry.name), "start_ticks": int(last[19])})
        except (FileNotFoundError, ProcessLookupError):
            # Routine unrelated process churn does not invalidate the sample.
            # A positively identified ELink process disappearing mid-read does.
            if candidate_matched:
                error("PROCESS_TABLE_CHANGED")
        except (OSError, ValueError, IndexError):
            error("PROCESS_TABLE_PARTIAL")
    try:
        lines = (root / "net/unix").read_text().splitlines()
        if not lines or not lines[0].startswith("Num"):
            raise ValueError("invalid unix table")
        result["ipc_listening"] = False
        for line in lines[1:]:
            fields = line.split()
            if not fields or len(fields) < 7:
                raise ValueError("invalid unix table row")
            if len(fields) == 8 and fields[7] == "@elink_stack":
                # Socket flags SO_ACCEPTCON, type STREAM, unconnected listener.
                if int(fields[3], 16) & 0x10000 and fields[4:6] == ["0001", "01"]:
                    result["ipc_listening"] = True
    except (OSError, ValueError):
        result["ipc_listening"] = None
        error("IPC_TABLE_UNAVAILABLE")
    return result


def collect_passive_snapshot(proc_root: str | Path = "/proc") -> PassiveSnapshot:
    return PassiveSnapshot.from_dict(_collect_passive_data(proc_root))


def passive_probe_source(nonce: str = "") -> str:
    """Source for `python3 -`; reads proc only and emits one JSON object."""
    return ("import json\n" + inspect.getsource(_collect_passive_data) +
            "\nprint(json.dumps({'nonce': " + repr(nonce) +
            ", 'snapshot': _collect_passive_data()}, sort_keys=True))\n")


def run_passive_probe(argv: Sequence[str], *, timeout_s: float = 2.0,
                      run=subprocess.run) -> PassiveSnapshot:
    """Bound the SSH/child invocation; never invoke a shell or vendor client.

    `argv` is a trusted fixed command assembled by the caller, ending in
    `python3 -`. A random echo token binds the response to this invocation.
    The remote script opens no IPC socket and sends no actuator commands.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and positive")
    if not argv or isinstance(argv, (str, bytes)) or any(not isinstance(a, str) for a in argv):
        raise ValueError("argv must be a nonempty sequence of strings")
    nonce = uuid.uuid4().hex
    try:
        completed = run(list(argv), input=passive_probe_source(nonce), text=True,
                        capture_output=True, timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired as exc:
        raise ProbeError("ELINK_OBSERVATION_TIMEOUT") from exc
    except OSError as exc:
        raise ProbeError("ELINK_OBSERVATION_UNAVAILABLE") from exc
    if completed.returncode:
        raise ProbeError("ELINK_OBSERVATION_COMMAND_FAILED")
    if not isinstance(completed.stdout, str) or len(completed.stdout.encode("utf-8")) > MAX_OUTPUT_BYTES:
        raise ProbeError("ELINK_OBSERVATION_OUTPUT_LIMIT")
    try:
        payload = json.loads(completed.stdout)
    except (ValueError, TypeError) as exc:
        raise ProbeError("ELINK_OBSERVATION_INVALID_JSON") from exc
    if not isinstance(payload, dict) or payload.get("nonce") != nonce:
        raise ProbeError("ELINK_OBSERVATION_TOKEN_MISMATCH")
    return PassiveSnapshot.from_dict(payload.get("snapshot"))


class ElinkHealthLatch:
    """Monotonic passive evidence with a fault retained within one instance.

    The supervisor and settling CLI use a fresh instance per observation, so
    later recovery is visible without a permanent lifecycle latch. Reusing an
    instance retains its earlier fault as evidence, not motion authority.
    OBSERVED is deliberately not HEALTHY; functional_ready is always false
    because this module cannot establish functional or safety-channel health.
    """

    def __init__(self, *, stale_after_s: float = 3.0):
        if not math.isfinite(stale_after_s) or stale_after_s <= 0:
            raise ValueError("stale_after_s must be finite and positive")
        self.stale_after_s = stale_after_s
        self._identity = None
        self._last_time = None
        self._state = "UNKNOWN"
        self._reason = "ELINK_NOT_OBSERVED"
        self._fault = ""

    def _time(self, now: float) -> bool:
        if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
            self._state = "UNKNOWN"
            self._reason = "ELINK_OBSERVATION_CLOCK_INVALID"
            return False
        return True

    def observe(self, snapshot: PassiveSnapshot, now: float) -> dict:
        if not self._time(now):
            return self.status(now)
        if self._fault:
            return self.status(now)
        self._last_time = now
        for role in ("TRANSFER", "HAL"):
            if role + "_PROCESS_NOT_LIVE" in snapshot.errors:
                self._fault = "ELINK_" + role + "_NOT_LIVE"
                return self.status(now)
        if snapshot.errors or snapshot.ipc_listening is None:
            self._state = "UNKNOWN"
            self._reason = "ELINK_OBSERVATION_INCOMPLETE"
            return self.status(now)
        reason = ""
        if len(snapshot.transfer) != 1:
            reason = "ELINK_TRANSFER_MISSING" if not snapshot.transfer else "ELINK_TRANSFER_AMBIGUOUS"
        elif len(snapshot.hal) != 1:
            reason = "ELINK_HAL_MISSING" if not snapshot.hal else "ELINK_HAL_AMBIGUOUS"
        elif not snapshot.ipc_listening:
            reason = "ELINK_IPC_NOT_LISTENING"
        identity = (snapshot.transfer, snapshot.hal)
        if not reason and self._identity is not None and identity != self._identity:
            reason = "ELINK_PROCESS_IDENTITY_CHANGED"
        if reason:
            self._fault = reason
        else:
            self._identity = identity
            self._state = "OBSERVED"
            self._reason = "ELINK_STRUCTURE_OBSERVED_FUNCTIONAL_UNVERIFIED"
        return self.status(now)

    def unavailable(self, reason: str, now: float) -> dict:
        if self._time(now) and not self._fault:
            self._last_time = now
            self._state = "UNKNOWN"
            self._reason = reason[:128] or "ELINK_OBSERVATION_UNAVAILABLE"
        return self.status(now)

    def status(self, now: float) -> dict:
        if self._fault:
            state, reason = "FAULT", self._fault
        elif (not math.isfinite(now) or
              (self._last_time is not None and now < self._last_time)):
            state, reason = "UNKNOWN", "ELINK_OBSERVATION_CLOCK_INVALID"
        elif self._last_time is None:
            state, reason = "UNKNOWN", self._reason
        elif now - self._last_time >= self.stale_after_s:
            state, reason = "UNKNOWN", "ELINK_OBSERVATION_STALE"
        else:
            state, reason = self._state, self._reason
        age = None if self._last_time is None or not math.isfinite(now) else max(0.0, now - self._last_time)
        return {"state": state, "reason": reason, "fault_latched": bool(self._fault),
                "functional_ready": False, "observation_age_s": age}


def main(argv=None, *, collect=collect_passive_snapshot, clock=time.monotonic,
         sleep=time.sleep) -> int:
    """CLI for a caller with an external whole-command timeout.

    --require-observed is a necessary structural check, never a claim that
    functional/safety channels are fresh. No alternate process roots or remote
    commands are accepted from command-line arguments.
    """
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-observed", action="store_true")
    parser.add_argument("--structured-exit", action="store_true",
                        help="0 observed, 10 unknown, 11 definite fault, 12 inspection error")
    parser.add_argument("--wait-observed-seconds", type=int, default=0, choices=range(0, 11),
                        help="bounded passive vendor-startup settling; no RPC/recovery")
    args = parser.parse_args(argv)
    if args.require_observed:
        deadline = clock() + args.wait_observed_seconds
        # This CLI is a startup/restore observation, not a live-session latch.
        # Each sample may settle from an absent vendor child to observed.
        for attempt in range(4 * args.wait_observed_seconds + 1):
            try:
                status = ElinkHealthLatch().observe(collect(), clock())
            except (OSError, ValueError, ProbeError) as exc:
                status = {"state": "UNKNOWN", "reason": "ELINK_OBSERVATION_TOOL_UNAVAILABLE",
                          "fault_latched": False, "functional_ready": False}
                print(json.dumps(status, sort_keys=True))
                return 12 if args.structured_exit else 1
            if status["state"] == "OBSERVED" or clock() >= deadline:
                break
            sleep(min(0.25, max(0.0, deadline - clock())))
        print(json.dumps(status, sort_keys=True))
        if args.structured_exit:
            return {"OBSERVED": 0, "UNKNOWN": 10, "FAULT": 11}[status["state"]]
        return 0 if status["state"] == "OBSERVED" else 1
    if args.structured_exit or args.wait_observed_seconds:
        parser.error("observation options require --require-observed")
    snapshot = collect()
    print(json.dumps(snapshot.to_dict(), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
