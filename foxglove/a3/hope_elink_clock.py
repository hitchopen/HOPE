"""Same-HDU chrony qualification; DDS receipt must not renew probe age.

This is passive diagnostic evidence, not an admission or authenticated clock service. Both
producer and consumer must use the same Linux boot and monotonic clock domain.
"""
from __future__ import annotations

import json
import math
import uuid


def _uuid(value: object) -> str | None:
    if not isinstance(value, str) or len(value) not in (32, 36):
        return None
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError):
        return None
    return parsed.hex if value in (parsed.hex, str(parsed)) else None


def _number(value: object) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value) and value >= 0
    except OverflowError:
        return False


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate field")
        result[key] = value
    return result


class ClockQualificationGuard:
    """Accept progressing completed probes and expire from their start time.

    ``update(raw_json, now)`` returns whether a new snapshot was accepted.
    ``status(now)`` returns ``(qualified, original_probe_start_or_None)``.
    Invalid/replayed input revokes qualification until a genuinely newer probe
    arrives. No input can extend the source deadline by being redelivered.
    """

    def __init__(self, expected_clock_boot_id: str, stale_after_s: float = 3.0):
        if not _number(stale_after_s) or stale_after_s <= 0:
            raise ValueError("stale_after_s must be finite and positive")
        self.stale_after_s = float(stale_after_s)
        self._boot = _uuid(expected_clock_boot_id)
        self._monitor: str | None = None
        self._retired: set[str] = set()
        self._sequence = 0
        self._observed: float | None = None
        self._last_now: float | None = None
        self._qualified = False

    def _check_now(self, now: float) -> bool:
        if not _number(now) or (self._last_now is not None and now < self._last_now):
            self._qualified = False
            return False
        self._last_now = float(now)
        return True

    def update(self, raw_json: str, now: float) -> bool:
        self._qualified = False
        if not self._check_now(now) or self._boot is None:
            return False
        if not isinstance(raw_json, str) or len(raw_json) > 4096:
            return False
        try:
            value = json.loads(raw_json, object_pairs_hook=_unique_object)
        except (ValueError, TypeError, RecursionError):
            return False
        if not isinstance(value, dict) or set(value) != {
            "schema_version", "clock_boot_id", "monitor_id", "sequence",
            "observed_monotonic_s", "gate_pass",
        }:
            return False
        sequence = value["sequence"]
        observed = value["observed_monotonic_s"]
        monitor = _uuid(value["monitor_id"])
        if (type(value["schema_version"]) is not int or value["schema_version"] != 1
                or _uuid(value["clock_boot_id"]) != self._boot
                or monitor is None or monitor in self._retired
                or type(sequence) is not int or not 1 <= sequence <= (1 << 53)
                or type(value["gate_pass"]) is not bool
                or not _number(observed)
                or not 0 <= now - observed < self.stale_after_s):
            return False
        if self._observed is not None and observed <= self._observed:
            return False
        if monitor == self._monitor and sequence <= self._sequence:
            return False
        if self._monitor is not None and monitor != self._monitor:
            # Never evict a retired identity and thereby make its replay fresh.
            # Exhaustion is fail-closed and requires supervisor maintenance.
            if len(self._retired) >= 128:
                return False
            self._retired.add(self._monitor)
        self._monitor = monitor
        self._sequence = sequence
        self._observed = float(observed)
        self._qualified = value["gate_pass"]
        return True

    def status(self, now: float) -> tuple[bool, float | None]:
        if not self._check_now(now):
            return False, self._observed
        fresh = (self._observed is not None
                 and 0 <= now - self._observed < self.stale_after_s)
        return self._qualified and fresh, self._observed
