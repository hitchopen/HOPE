#!/usr/bin/env python3
"""Pure fail-closed recovery-dwell state used by the Gate3 conductor."""

from __future__ import annotations


def update_recovery_dwell(
    row: dict,
    *,
    now: float,
    settled_now: bool,
    required_dwell_s: float,
) -> None:
    """Update a reversible continuous-dwell qualification.

    Reaching HOME once is not a lifetime certificate.  Any later violation
    revokes qualification; a fresh uninterrupted dwell is then required.
    ``recovery_max_dwell_observed_s`` remains diagnostic only.
    """

    if settled_now:
        if row.get("_recovery_dwell_start") is None:
            row["_recovery_dwell_start"] = float(now)
        dwell = max(0.0, float(now) - float(row["_recovery_dwell_start"]))
        row["recovery_dwell_observed_s"] = dwell
        row["recovery_max_dwell_observed_s"] = max(
            float(row.get("recovery_max_dwell_observed_s", 0.0)), dwell
        )
        row["recovery_settled"] = dwell >= float(required_dwell_s)
        return

    if row.get("recovery_settled", False):
        row["recovery_revocation_count"] = int(
            row.get("recovery_revocation_count", 0)
        ) + 1
    row["_recovery_dwell_start"] = None
    row["recovery_dwell_observed_s"] = 0.0
    row["recovery_settled"] = False
