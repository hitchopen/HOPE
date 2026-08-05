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


PING_RTT_RE = re.compile(r"time=([0-9.]+) ms")


@dataclass(frozen=True)
class NtpProbeResult:
    offset_ms: float = math.nan
    skew_ppm: float = math.nan
    root_dispersion_ms: float = math.nan
    utc_qualified: bool = False
    gate_pass: bool = False
    error: str = ""


@dataclass(frozen=True)
class MocapProbeResult:
    rtt_ms: float = math.nan
    reachable: bool = False
    error: str = ""


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


def probe_mocap(host: str) -> MocapProbeResult:
    """Measure one ICMP round trip without throwing."""

    if not host or host.startswith("REPLACE"):
        return MocapProbeResult(error="mocap_host is not configured")
    try:
        completed = subprocess.run(
            ["ping", "-c", "1", "-W", "1", host],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        match = PING_RTT_RE.search(completed.stdout)
        if match:
            return MocapProbeResult(rtt_ms=float(match.group(1)), reachable=True)
        detail = completed.stderr.strip() or f"ping exited with status {completed.returncode}"
        return MocapProbeResult(error=detail)
    except (OSError, subprocess.SubprocessError) as exc:
        return MocapProbeResult(error=str(exc))


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
