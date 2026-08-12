#!/usr/bin/env python3
"""Pure helpers for the HOPE A3 Foxglove monitor.

This module intentionally has no ROS dependency so the parsing and health rules
can be unit-tested on a development machine.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import subprocess
from typing import Mapping, Sequence


CALIBRATION_SHA_RE = re.compile(r"[0-9a-f]{64}")


def combine_estop_results(
    *,
    vendor_accepted: bool,
    vendor_detail: str,
    runner_stopped: bool,
    runner_detail: str,
) -> tuple[bool, str]:
    """Combine the two independent E-stop actions without overstating proof."""

    vendor_text = vendor_detail.strip() or "no vendor detail"
    runner_text = runner_detail.strip() or "no runner detail"
    if vendor_accepted and runner_stopped:
        return (
            True,
            "vendor software E-stop request accepted and managed model21800 "
            "runner stopped; actuator stop is not independently confirmed, "
            "verify the physical E-stop",
        )
    if vendor_accepted:
        return (
            True,
            "PARTIAL E-STOP: vendor request accepted, but managed runner stop "
            f"is unconfirmed ({runner_text}); use the physical E-stop",
        )
    if runner_stopped:
        return (
            True,
            "PARTIAL E-STOP: managed runner stopped, but vendor E-stop request "
            f"was not accepted ({vendor_text}); use the physical E-stop",
        )
    return (
        False,
        "E-STOP UNCONFIRMED: vendor path failed "
        f"({vendor_text}); runner path failed ({runner_text}); "
        "use the physical E-stop",
    )


@dataclass(frozen=True)
class EstopBackendStatus:
    """Operator-facing availability without overstating a degraded stop path."""

    action_ready: bool
    full_ready: bool
    detail: str


def estop_backend_status(
    *,
    vendor_ready: bool,
    runner_ready: bool,
    latched: bool,
    vendor_protocol_available: bool,
) -> EstopBackendStatus:
    """Describe whether at least one and whether both E-stop paths are ready.

    A live native Runner emergency-PASSIVE service is still useful when the
    vendor application manager is intentionally stopped for the managed HAL,
    but it must remain visibly degraded and can never be reported as a full
    dual-path E-stop.
    """

    if latched:
        return EstopBackendStatus(
            action_ready=False,
            full_ready=False,
            detail="E-STOP LATCHED | inspect robot and use approved local recovery",
        )
    if vendor_ready and runner_ready:
        return EstopBackendStatus(
            action_ready=True,
            full_ready=True,
            detail="DUAL-PATH E-STOP READY | VENDOR + RUNNER EMERGENCY PASSIVE",
        )
    if vendor_ready:
        return EstopBackendStatus(
            action_ready=True,
            full_ready=False,
            detail="VENDOR E-STOP READY | RUNNER STOP DEGRADED | PARTIAL ONLY",
        )
    if runner_ready:
        return EstopBackendStatus(
            action_ready=True,
            full_ready=False,
            detail=(
                "RUNNER EMERGENCY PASSIVE READY | VENDOR E-STOP UNAVAILABLE | "
                "PARTIAL ONLY | USE PHYSICAL E-STOP"
            ),
        )
    if not vendor_protocol_available:
        detail = "E-STOP UNAVAILABLE | ros2_plugin_proto is not installed"
    else:
        detail = "E-STOP UNAVAILABLE | no live vendor or Runner emergency backend"
    return EstopBackendStatus(
        action_ready=False,
        full_ready=False,
        detail=detail,
    )


def parse_calibration_service_sha(message: str) -> str:
    """Validate the laptop calibration service's exact SHA response."""

    value = str(message).strip()
    if CALIBRATION_SHA_RE.fullmatch(value) is None:
        raise ValueError("calibration service did not return a 64-hex SHA")
    return value


@dataclass(frozen=True)
class NtpProbeResult:
    offset_ms: float = math.nan
    skew_ppm: float = math.nan
    root_dispersion_ms: float = math.nan
    utc_qualified: bool = False
    gate_pass: bool = False
    error: str = ""


@dataclass(frozen=True)
class ServiceProbeResult:
    active: bool = False
    state: str = "unknown"
    error: str = ""


@dataclass(frozen=True)
class CpuTimes:
    """Aggregate Linux CPU counters read from the first ``/proc/stat`` row."""

    user: int
    nice: int
    system: int
    idle: int
    iowait: int
    irq: int
    softirq: int
    steal: int

    @property
    def total(self) -> int:
        return sum(
            (
                self.user,
                self.nice,
                self.system,
                self.idle,
                self.iowait,
                self.irq,
                self.softirq,
                self.steal,
            )
        )

    @property
    def idle_total(self) -> int:
        return self.idle + self.iowait


@dataclass(frozen=True)
class ProcessCpuTimes:
    """One Linux process CPU counter snapshot from ``/proc/<pid>/stat``."""

    pid: int
    name: str
    ticks: int


@dataclass(frozen=True)
class ProcessCpuLoad:
    """Top process load in both whole-machine and one-core percentage units."""

    pid: int
    name: str
    system_percent: float
    core_percent: float


def parse_chrony_status(
    tracking_csv: str,
    sources_text: str,
    *,
    max_offset_ms: float,
    max_skew_ppm: float,
) -> NtpProbeResult:
    """Parse chrony output using the same source-selection rule as preflight."""

    fields = tracking_csv.strip().split(",")
    if len(fields) < 14:
        raise ValueError(f"chronyc tracking returned {len(fields)} fields; expected at least 14")

    offset_ms = float(fields[4]) * 1000.0
    skew_ppm = float(fields[9])
    root_dispersion_ms = float(fields[11]) * 1000.0
    leap_normal = fields[13].strip() == "Normal"
    selected_source = any(line.startswith("^*") for line in sources_text.splitlines())
    utc_qualified = leap_normal and selected_source
    finite = all(math.isfinite(value) for value in (offset_ms, skew_ppm))
    gate_pass = (
        utc_qualified
        and finite
        and abs(offset_ms) <= max_offset_ms
        and skew_ppm <= max_skew_ppm
    )
    return NtpProbeResult(
        offset_ms=offset_ms,
        skew_ppm=skew_ppm,
        root_dispersion_ms=root_dispersion_ms,
        utc_qualified=utc_qualified,
        gate_pass=gate_pass,
    )


def probe_ntp(*, max_offset_ms: float, max_skew_ppm: float) -> NtpProbeResult:
    """Read chrony without throwing; failures become an unqualified result."""

    try:
        tracking = subprocess.run(
            ["chronyc", "-c", "tracking"],
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        ).stdout
        sources = subprocess.run(
            ["chronyc", "-n", "sources"],
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        ).stdout
        return parse_chrony_status(
            tracking,
            sources,
            max_offset_ms=max_offset_ms,
            max_skew_ppm=max_skew_ppm,
        )
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        return NtpProbeResult(error=str(exc))


def probe_systemd_service(unit: str) -> ServiceProbeResult:
    """Read a local systemd unit's state without changing it."""

    try:
        completed = subprocess.run(
            ["systemctl", "is-active", unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        state = completed.stdout.strip() or "unknown"
        if completed.returncode == 0 and state == "active":
            return ServiceProbeResult(active=True, state=state)
        # systemctl returns 3 for normal non-active states such as inactive or
        # failed. Surface stderr only for a real probe/configuration failure.
        error = ""
        if completed.returncode not in (0, 3):
            error = completed.stderr.strip() or (
                f"systemctl exited with status {completed.returncode}"
            )
        return ServiceProbeResult(active=False, state=state, error=error)
    except (OSError, subprocess.SubprocessError) as exc:
        return ServiceProbeResult(error=str(exc))


def parse_proc_stat_cpu(proc_stat: str) -> CpuTimes:
    """Parse the aggregate CPU counters from Linux ``/proc/stat``."""

    line = next(
        (candidate for candidate in proc_stat.splitlines() if candidate.startswith("cpu ")),
        None,
    )
    if line is None:
        raise ValueError("/proc/stat has no aggregate 'cpu' row")
    fields = line.split()[1:]
    if len(fields) < 8:
        raise ValueError(
            f"aggregate CPU row has {len(fields)} counters; expected at least 8"
        )
    try:
        counters = [int(value) for value in fields[:8]]
    except ValueError as exc:
        raise ValueError("aggregate CPU row contains a non-integer counter") from exc
    if any(value < 0 for value in counters):
        raise ValueError("aggregate CPU row contains a negative counter")
    return CpuTimes(*counters)


def cpu_load_percent(previous: CpuTimes, current: CpuTimes) -> float:
    """Return aggregate CPU busy time between two samples, bounded to 0..100."""

    total_delta = current.total - previous.total
    idle_delta = current.idle_total - previous.idle_total
    if total_delta <= 0 or idle_delta < 0:
        raise ValueError("CPU counters did not advance monotonically")
    busy_delta = total_delta - idle_delta
    return max(0.0, min(100.0, 100.0 * busy_delta / total_delta))


def parse_process_stat(process_stat: str) -> ProcessCpuTimes:
    """Parse PID, command name, and user+system ticks from procfs stat text."""

    text = str(process_stat).strip()
    left = text.find("(")
    right = text.rfind(")")
    if left <= 0 or right <= left:
        raise ValueError("process stat has no valid command field")
    try:
        pid = int(text[:left].strip())
    except ValueError as exc:
        raise ValueError("process stat PID is invalid") from exc
    name = text[left + 1:right]
    fields = text[right + 1:].split()
    # fields[0] is kernel field 3 (state); utime/stime are fields 14/15.
    if pid <= 0 or not name or len(fields) < 13:
        raise ValueError("process stat is truncated")
    try:
        user_ticks = int(fields[11])
        system_ticks = int(fields[12])
    except ValueError as exc:
        raise ValueError("process CPU counters are invalid") from exc
    if user_ticks < 0 or system_ticks < 0:
        raise ValueError("process CPU counters are negative")
    return ProcessCpuTimes(pid=pid, name=name, ticks=user_ticks + system_ticks)


def top_process_cpu_load(
    previous: Mapping[int, ProcessCpuTimes],
    current: Mapping[int, ProcessCpuTimes],
    *,
    total_cpu_delta: int,
    cpu_count: int,
) -> ProcessCpuLoad | None:
    """Find the process responsible for the largest CPU delta.

    ``system_percent`` is its share of all machine CPU time. ``core_percent``
    uses the familiar top(1) scale where one fully occupied core is 100%.
    """

    if total_cpu_delta <= 0 or cpu_count <= 0:
        raise ValueError("CPU delta and CPU count must be positive")
    candidates: list[tuple[int, ProcessCpuTimes]] = []
    for pid, sample in current.items():
        old = previous.get(pid)
        if old is None or old.name != sample.name:
            continue
        delta = sample.ticks - old.ticks
        if delta >= 0:
            candidates.append((delta, sample))
    if not candidates:
        return None
    delta, sample = max(candidates, key=lambda item: (item[0], -item[1].pid))
    system_percent = max(0.0, min(100.0, 100.0 * delta / total_cpu_delta))
    return ProcessCpuLoad(
        pid=sample.pid,
        name=sample.name,
        system_percent=system_percent,
        core_percent=system_percent * cpu_count,
    )


def message_latency_ms(now_ns: int, stamp_sec: int, stamp_nanosec: int) -> float:
    """Return local ROS time minus a ROS message header timestamp in ms."""

    stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
    if stamp_ns <= 0:
        raise ValueError("message header timestamp is zero")
    return (int(now_ns) - stamp_ns) / 1_000_000.0


def timestamp_age_s(now_ns: int, stamp_sec: int, stamp_nanosec: int) -> float:
    """Return local ROS time minus a positive ROS timestamp in seconds."""

    return message_latency_ms(now_ns, stamp_sec, stamp_nanosec) / 1000.0


def build_software_estop_request(timestamp_ns: int, trace_id: str) -> bytes:
    """Encode aimdk.protocol.EmergencyCommandReq without generated protobufs.

    The request contains a current RequestHeader and only the vendor software
    E-stop bit. It intentionally has no representation for clearing E-stop.
    Field numbers come from the A3 vendor's emergency_state.proto and
    common/header.proto schemas.
    """

    if timestamp_ns < 0:
        raise ValueError("timestamp_ns must be non-negative")
    if not trace_id:
        raise ValueError("trace_id must not be empty")

    seconds, nanos = divmod(int(timestamp_ns), 1_000_000_000)
    timestamp = b"".join(
        (
            _pb_uint(1, seconds),
            _pb_uint(2, nanos),
            _pb_uint(3, timestamp_ns // 1_000_000),
        )
    )
    header = b"".join(
        (
            _pb_bytes(1, timestamp),
            _pb_bytes(4, trace_id.encode("utf-8")),
            _pb_bytes(5, b"hope-foxglove"),
        )
    )
    # EmergencyCommand field 2 is software_emergency_stop.
    command = _pb_uint(2, 1)
    return _pb_bytes(1, header) + _pb_bytes(2, command)


def decode_software_estop_response(payload: bytes) -> tuple[int, str]:
    """Decode ``EmergencyCommandRsp.header`` and return its code and message.

    The vendor response schema places ``ResponseHeader`` in top-level field 1;
    `ResponseHeader.code` and `.msg` are fields 1 and 2. In proto3, an omitted
    scalar code inside a present header has its specified default value zero.
    A missing response header is rejected because message presence is
    semantically distinct from a present header containing default scalars.
    """

    headers = []
    for field_number, wire_type, value in _pb_fields(bytes(payload)):
        if field_number != 1:
            continue
        if wire_type != 2:
            raise ValueError("EmergencyCommandRsp.header has the wrong wire type")
        headers.append(value)
    if len(headers) != 1:
        raise ValueError(
            f"EmergencyCommandRsp has {len(headers)} response headers; expected 1"
        )

    code = 0
    message = ""
    code_seen = False
    message_seen = False
    for field_number, wire_type, value in _pb_fields(headers[0]):
        if field_number == 1:
            if wire_type != 0 or code_seen:
                raise ValueError("ResponseHeader.code is malformed or duplicated")
            code = int(value)
            code_seen = True
        elif field_number == 2:
            if wire_type != 2 or message_seen:
                raise ValueError("ResponseHeader.msg is malformed or duplicated")
            try:
                message = value.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise ValueError("ResponseHeader.msg is not valid UTF-8") from exc
            message_seen = True
    return code, message


def _pb_uint(field_number: int, value: int) -> bytes:
    return _pb_varint((field_number << 3) | 0) + _pb_varint(value)


def _pb_bytes(field_number: int, value: bytes) -> bytes:
    return _pb_varint((field_number << 3) | 2) + _pb_varint(len(value)) + value


def _pb_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("protobuf varint value must be non-negative")
    encoded = bytearray()
    while value > 0x7F:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _pb_fields(payload: bytes):
    """Yield protobuf fields while rejecting truncation and unsupported groups."""

    offset = 0
    while offset < len(payload):
        key, offset = _pb_read_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0:
            raise ValueError("protobuf field number zero is invalid")
        if wire_type == 0:
            value, offset = _pb_read_varint(payload, offset)
        elif wire_type == 1:
            end = offset + 8
            if end > len(payload):
                raise ValueError("truncated protobuf fixed64 field")
            value = payload[offset:end]
            offset = end
        elif wire_type == 2:
            length, offset = _pb_read_varint(payload, offset)
            end = offset + length
            if end > len(payload):
                raise ValueError("truncated protobuf length-delimited field")
            value = payload[offset:end]
            offset = end
        elif wire_type == 5:
            end = offset + 4
            if end > len(payload):
                raise ValueError("truncated protobuf fixed32 field")
            value = payload[offset:end]
            offset = end
        else:
            raise ValueError(f"unsupported protobuf wire type {wire_type}")
        yield field_number, wire_type, value


def _pb_read_varint(payload: bytes, offset: int) -> tuple[int, int]:
    value = 0
    for shift in range(0, 70, 7):
        if offset >= len(payload):
            raise ValueError("truncated protobuf varint")
        byte = payload[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, offset
    raise ValueError("protobuf varint exceeds 10 bytes")


def stale_sources(
    last_received_monotonic: Mapping[str, float],
    expected_sources: Sequence[str],
    *,
    now_monotonic: float,
    stale_after_s: float,
) -> list[str]:
    """Return missing or stale sources in stable configured order."""

    return [
        source
        for source in expected_sources
        if source not in last_received_monotonic
        or now_monotonic - last_received_monotonic[source] > stale_after_s
    ]
