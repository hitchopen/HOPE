#!/usr/bin/env python3
"""ROS-free rules for the live P1 marker-count operator interface."""

from __future__ import annotations

import math
from typing import Iterable


EXPECTED_P1_MARKERS = 10


def marker_has_physical_sample(marker) -> bool:
    """Match the physical-sample rule used by the P1 CAD calibrator.

    NatNet params bit 0 means occluded and bit 1 means point-cloud solved.
    Model-filled positions are not counted as markers received by the laptop.
    """

    params = int(marker.params)
    position = marker.position
    finite_position = all(
        math.isfinite(float(value))
        for value in (position.x, position.y, position.z)
    )
    return (
        bool(marker.has_live_sample)
        and (params & 0x01) == 0
        and (params & 0x02) != 0
        and finite_position
    )


def count_physical_markers(
    markers: Iterable[object], *, expected_count: int = EXPECTED_P1_MARKERS
) -> tuple[int, int]:
    """Return ``(bounded_count, raw_unique_count)`` using unique member IDs."""

    if expected_count <= 0:
        raise ValueError("expected_count must be positive")
    member_ids = {
        int(marker.member_id)
        for marker in markers
        if marker_has_physical_sample(marker)
    }
    raw_count = len(member_ids)
    return min(raw_count, expected_count), raw_count


def marker_count_text(
    count: int,
    *,
    fresh: bool,
    expected_count: int = EXPECTED_P1_MARKERS,
    raw_count: int | None = None,
) -> str:
    if not fresh:
        return f"P1 live markers = 0/{expected_count} | NO FRESH LAPTOP DATA"
    suffix = ""
    if raw_count is not None and raw_count > expected_count:
        suffix = f" | raw_unique={raw_count} bounded_for_UI={expected_count}"
    return f"P1 live markers = {int(count)}/{expected_count}{suffix}"
