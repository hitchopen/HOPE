#!/usr/bin/env python3
"""Zero-buffer rate gate driven by message source timestamps."""

from __future__ import annotations


class SourceTimeRateGate:
    """Select source samples at an average maximum rate without adding a timer queue."""

    def __init__(self, rate_hz: float) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self._period_ns = 1_000_000_000.0 / float(rate_hz)
        self._next_source_ns: float | None = None
        self._last_source_ns: int | None = None

    def accept(self, source_ns: int) -> bool:
        """Return true for the newest sample that reaches the next source-time slot."""
        source_ns = int(source_ns)
        if self._last_source_ns is None or source_ns < self._last_source_ns:
            self._last_source_ns = source_ns
            self._next_source_ns = source_ns + self._period_ns
            return True

        self._last_source_ns = source_ns
        assert self._next_source_ns is not None
        if source_ns + 1.0 < self._next_source_ns:
            return False

        elapsed_periods = max(
            1,
            int((source_ns - self._next_source_ns) // self._period_ns) + 1,
        )
        self._next_source_ns += elapsed_periods * self._period_ns
        return True
