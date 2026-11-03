"""Failure receipts keep source exposures and distinguish observer loss."""
import json
from types import SimpleNamespace

import pytest
from qualification_accounting import SourceCadence, validate_source_epoch, write_receipt


@pytest.mark.parametrize('rate', [150, 300, 360])
def test_source_pause_does_not_manufacture_catch_up_observations(rate):
    """A delayed camera resumes acquisition instead of creating a false burst."""
    cadence = SourceCadence(rate, 1_000_000_000)
    acquisitions = []
    for index in range(1000):
        acquisition = cadence.deadline_ns + (34_000_000 if index == 305 else 50_000)
        acquisitions.append(acquisition)
        cadence.acquired(acquisition)
        if index == 305:
            assert cadence.deadline_ns == acquisition + cadence.period_ns
    receipt = cadence.receipt()
    assert receipt['valid']
    assert receipt['resynchronizations'] == 1
    assert receipt['missed_nominal_ticks'] == 34_000_000 // cadence.period_ns
    assert min(b - a for a, b in zip(acquisitions, acquisitions[1:])) >= cadence.period_ns // 2
    assert receipt['samples'] == len(acquisitions)  # no offered observation is removed


def test_source_health_rejects_slow_or_burst_acquisition():
    """Coverage cannot conceal a source that misses its declared cadence."""
    slow = SourceCadence(150, 0)
    for index in range(100):
        slow.acquired(index * 10_000_000)
    assert not slow.receipt()['valid']
    burst = SourceCadence(150, 0)
    for instant in [0, 0, 100_000, 6_666_667]:
        burst.acquired(instant)
    assert not burst.receipt()['valid']


def test_delayed_stamp_cannot_describe_an_earlier_physical_state():
    # A 2 ms construction pause at 10 m/s would silently relabel a state 2 cm
    # behind its advertised acquisition time. Waiting to publish is allowed;
    # changing the acquisition identity of the already sampled state is not.
    source = {'clock_origin_ns': 1792000000000000000, 'rows': [
        {'index': 1, 'stamp_ns': 1792000000102000000, 'state_elapsed_s': .1}]}
    with pytest.raises(AssertionError, match='state/acquisition timestamp mismatch'):
        validate_source_epoch(source)
    source['rows'][0]['state_elapsed_s'] = .102
    validate_source_epoch(source)


def test_failed_trial_keeps_independent_denominator_and_resources(tmp_path):
    args = SimpleNamespace(output=tmp_path / 'trial.json', stage_accounting=True)
    (tmp_path / 'trial-hope_planner_cpp_node.stages').write_text(
        '# capacity=10 total=2 overwritten=0\n'
        'stage\tsource_ns\tsequence\tgeneration\twall_ns\tsteady_ns\tduration_ns\treason\n'
        'published\t20\t4\t2\t25\t26\t0\t30\n'
        'planner_reject\t30\t5\t2\t35\t36\t0\tSOURCE_UNQUALIFIED\n')
    raw = {'source': {'eligible_stamps': [10, 20, 30]},
           'traces': [{'source_time_ns': '10'}]}
    resources = {'planner': {'VmHWM': '1000 kB'}}
    result = write_receipt(args, raw, resources, 'coverage failed')
    assert result['independent_denominator'] == 3
    assert result['observed_covered'] == 1
    assert result['missing_epochs'][0]['first_observed_loss'] == (
        'TRACE_OBSERVER_MISSING_PRODUCTION_PUBLICATION_PRESENT')
    assert result['missing_epochs'][1]['first_observed_loss'] == 'PLANNER_REJECTED'
    assert json.loads(args.output.read_text())['status'] == 'FAIL'
    assert json.loads(args.output.with_suffix('.raw.json').read_text())['resources'] == resources
