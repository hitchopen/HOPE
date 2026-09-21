from __future__ import annotations

import pathlib
import sys

import pytest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from source_time_rate_gate import SourceTimeRateGate


def _accepted(rate_in_hz: int, seconds: int = 2) -> list[int]:
    gate = SourceTimeRateGate(150.0)
    return [
        index
        for index in range(rate_in_hz * seconds)
        if gate.accept(round(index * 1_000_000_000 / rate_in_hz))
    ]


def test_300_hz_selects_every_second_sample():
    assert _accepted(300, seconds=1) == list(range(0, 300, 2))


def test_150_hz_passes_every_sample():
    assert _accepted(150, seconds=1) == list(range(150))


def test_360_hz_preserves_average_150_hz_without_buffering():
    accepted = _accepted(360)
    assert len(accepted) == 300
    assert set(b - a for a, b in zip(accepted, accepted[1:])) == {2, 3}


def test_backward_source_time_starts_a_new_epoch_immediately():
    gate = SourceTimeRateGate(150.0)
    assert gate.accept(1_000_000_000)
    assert not gate.accept(1_001_000_000)
    assert gate.accept(900_000_000)


def test_non_positive_rate_is_rejected():
    with pytest.raises(ValueError, match="positive"):
        SourceTimeRateGate(0.0)
