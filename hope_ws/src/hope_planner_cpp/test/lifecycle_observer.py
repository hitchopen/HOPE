"""Check actual flight transitions against independent A/B source windows."""

ACTIVE = {'OPPONENT_RALLY', 'RALLY', 'OPPONENT_SERVE', 'SERVE'}


def verify_lifecycle(source_rows, metadata, transitions, commands, decoded_sequences, rounds=10):
    """A latest-status sample cannot prove that a brief transition never occurred."""
    assert int(metadata.get('overwritten', -1)) == 0, 'incomplete lifecycle trace'
    assert int(metadata.get('total', -1)) == len(transitions) > 0, 'missing lifecycle records'
    stamps = [int(row['source_ns']) for row in transitions]
    assert len(stamps) == len(set(stamps)), 'duplicate lifecycle source'
    assert all(row['stage'] in ACTIVE | {'INVALID'} for row in transitions), 'unknown state'
    source = {row['stamp_ns']: row for row in source_rows}
    # Fault injection can intentionally put a different timestamp on the wire.
    # Keep those receipts attributable, without counting them as clean evidence.
    for row in source_rows:
        if row.get('fault') and 'wire_stamp_ns' in row:
            source.setdefault(row['wire_stamp_ns'], row)
    assert all(stamp in source for stamp in stamps), 'lifecycle source absent from producer'
    assert all(b > a for a, b in zip(stamps, stamps[1:])), 'reordered lifecycle evidence'
    generations = [int(row['generation']) for row in transitions]
    assert all(b >= a for a, b in zip(generations, generations[1:])), 'reordered flight identity'
    evidence = []
    previous_b = 0
    for ordinal in range(rounds):
        def samples(*labels):
            return [row for row in transitions
                    if source[int(row['source_ns'])]['round'] == ordinal
                    and source[int(row['source_ns'])]['opportunity'] in labels
                    and not source[int(row['source_ns'])].get('fault')]

        acquired = [row for row in samples('A') if row['stage'] in ACTIVE]
        assert acquired, ('A never acquired', ordinal)
        a_generations = {int(row['generation']) for row in acquired}
        assert len(a_generations) == 1, ('A changed generation', ordinal)
        a_generation = a_generations.pop()
        assert a_generation > previous_b, ('A reused earlier flight', ordinal)
        rejected = [row for row in samples('A_REJECT', 'A_CONTINUED_REJECTED')
                    if row['stage'] == 'INVALID' and row['reason'] == 'observed_under_table'
                    and int(row['generation']) == a_generation + 1]
        assert rejected, ('A never physically rejected', ordinal)
        first_rejection = min(int(row['steady_ns']) for row in rejected)
        assert max(int(row['steady_ns']) for row in acquired) < first_rejection
        quarantine = samples('A_CONTINUED_REJECTED')
        assert quarantine and all(row['stage'] == 'INVALID'
                                  and int(row['generation']) == a_generation + 1
                                  for row in quarantine), ('A resurrected', ordinal)
        b_active = [row for row in samples('B') if row['stage'] in ACTIVE]
        assert b_active, ('B never acquired', ordinal)
        b_generations = {int(row['generation']) for row in b_active}
        assert len(b_generations) == 1, ('B changed generation', ordinal)
        b_generation = b_generations.pop()
        assert b_generation > a_generation + 1, ('B reused retired A generation', ordinal)
        assert min(int(row['steady_ns']) for row in b_active) > first_rejection
        b_sources = {row['stamp_ns'] for row in source_rows
                     if row['round'] == ordinal and row['opportunity'] == 'B'}
        b_commands = [row for row in commands if int(row['source_time_ns']) in b_sources]
        assert b_commands and all(int(row['physical_shot']) == b_generation
                                  for row in b_commands), ('B commands absent/wrong flight', ordinal)
        assert any(int(row['command_sequence']) in decoded_sequences
                   for row in b_commands), ('B never decoded', ordinal)
        rejection_wall = min(int(row['wall_ns']) for row in rejected)
        assert not [row for row in commands
                    if int(row['physical_shot']) == a_generation
                    and int(row['publication_begin_ns']) >= rejection_wall], (
                        'obsolete A command after rejection', ordinal)
        evidence.append({'round': ordinal, 'A_generation': a_generation,
                         'A_retired_generation': a_generation + 1,
                         'B_generation': b_generation,
                         'acquisition_sources': [int(row['source_ns']) for row in acquired],
                         'first_rejection_steady_ns': first_rejection,
                         'quarantined_observations': len(quarantine),
                         'B_decoded_commands': sum(int(row['command_sequence']) in decoded_sequences
                                                   for row in b_commands)})
        previous_b = b_generation
    return {'status': 'PASS', 'rounds': evidence, 'records': len(transitions)}
