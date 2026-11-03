"""Missing offers and bad decoding must not disappear from useful-flight coverage."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / 'scripts/flight_prediction_validation')
)
import coverage_outcome_audit as audit


@pytest.mark.parametrize(
    'fault', ['none', 'missing_flight', 'decoder_mismatch', 'late', 'wrong_contact']
)
def test_independent_denominator_and_consumer_usability(tmp_path, monkeypatch, fault):
    monkeypatch.setattr(
        audit,
        'flight',
        lambda *args: (None, SimpleNamespace(sol=lambda t: np.array([0.15, -0.7, 0.2])), 0.1),
    )
    raw = dict(
        source=dict(
            command_epoch_contract='first_2_flights', plane_time_s=0.3, clock_origin_ns=0, rows=[]
        ),
        wire=[],
        decoded=[],
        traces=[],
    )
    for index in range(2):
        stamp = (index + 1) * 1000000000
        raw['source']['rows'].append(dict(stamp_ns=stamp, state_elapsed_s=0.9 + index))
        if fault == 'missing_flight' and index == 1:
            continue
        wire = [0.0] * 20
        wire[1] = 1.0
        wire[3:9] = [0.15, -0.7, 0.96, 1.0, 2.0, 3.0]
        wire[10] = 1.1 + index
        wire[14] = index + 1
        decoded = [0.0] * 15
        decoded[0] = index + 1
        decoded[3:9] = wire[3:9]
        decoded[9] = 0.1
        if index == 1:
            if fault == 'decoder_mismatch':
                decoded[3] += 0.01
            if fault == 'late':
                decoded[9] = 0.079
            if fault == 'wrong_contact':
                wire[10] += 0.02
        raw['wire'].append(wire)
        raw['decoded'].append(decoded)
        raw['traces'].append(dict(source_time_ns=stamp, command_sequence=index + 1))
    path = tmp_path / 'raw.json'
    path.write_text(json.dumps(raw))
    result = audit.audit('review_chain_300_10_unknown', path)
    assert result['expected_flights'] == 2
    assert len(result['flights']) == 2
    assert (result['assessment'] == 'CONFIRMED_DEFECT') == (fault != 'none')
