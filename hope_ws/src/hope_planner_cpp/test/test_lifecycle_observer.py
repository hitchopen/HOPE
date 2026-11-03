"""Positive/negative independent controls for the lifecycle oracle."""
import copy
import pytest
from lifecycle_observer import verify_lifecycle


def scenario():
    source = [dict(stamp_ns=t, round=0, opportunity=label, fault=None)
              for t, label in [(10, 'A'), (20, 'A_REJECT'),
                               (30, 'A_CONTINUED_REJECTED'), (40, 'B')]]
    rows = [dict(source_ns=t, steady_ns=t, wall_ns=t, stage=state,
                 generation=gen, reason=reason)
            for t, state, gen, reason in [(10, 'RALLY', 2, 'ballistic_acquisition_incomplete_history'),
                                         (20, 'INVALID', 3, 'observed_under_table'),
                                         (30, 'INVALID', 3, 'observed_under_table'),
                                         (40, 'RALLY', 4, 'ballistic_acquisition_incomplete_history')]]
    commands = [dict(source_time_ns=40, physical_shot=4, command_sequence=1,
                     publication_begin_ns=45)]
    return dict(source_rows=source, metadata={'overwritten': 0, 'total': 4},
                transitions=rows, commands=commands, decoded_sequences={1}, rounds=1)


def test_acquire_reject_between_status_polls():
    # Polls at 0 and 50 never see acquired A. The actual trace must still prove it.
    assert verify_lifecycle(**scenario())['rounds'][0]['A_generation'] == 2


def test_injected_wire_timestamp_remains_attributable_but_is_not_clean_evidence():
    case = scenario()
    case['source_rows'].append(dict(stamp_ns=35, wire_stamp_ns=34, round=0,
                                    opportunity='A_CONTINUED_REJECTED', fault='reorder'))
    case['transitions'].insert(3, dict(source_ns=34, steady_ns=35, wall_ns=35,
                                      stage='INVALID', generation=3, reason='observed_under_table'))
    case['metadata']['total'] += 1
    assert verify_lifecycle(**case)['rounds'][0]['quarantined_observations'] == 1


@pytest.mark.parametrize('failure', ['no_A', 'wrong_round', 'wrong_generation', 'no_reject',
                                    'quarantine_resurrection', 'obsolete_command', 'no_B',
                                    'no_command', 'no_decode', 'overflow', 'missing', 'duplicate'])
def test_genuine_lifecycle_failures_are_not_hidden(failure):
    case = copy.deepcopy(scenario())
    rows = case['transitions']
    if failure == 'no_A': rows[0]['stage'] = 'INVALID'
    elif failure == 'wrong_round': case['source_rows'][0]['round'] = 1
    elif failure == 'wrong_generation': rows[1]['generation'] = 1
    elif failure == 'no_reject':
        rows[1]['reason'] = rows[2]['reason'] = 'startup'
    elif failure == 'quarantine_resurrection': rows[2]['stage'] = 'RALLY'
    elif failure == 'obsolete_command':
        case['commands'].append(dict(source_time_ns=10, physical_shot=2,
                                     command_sequence=2, publication_begin_ns=25))
    elif failure == 'no_B': rows[3]['stage'] = 'INVALID'
    elif failure == 'no_command': case['commands'] = []
    elif failure == 'no_decode': case['decoded_sequences'] = set()
    elif failure == 'overflow': case['metadata']['overwritten'] = 1
    elif failure == 'missing': rows.pop()
    elif failure == 'duplicate': rows[1]['source_ns'] = 10
    with pytest.raises(AssertionError):
        verify_lifecycle(**case)
