"""Local, failure-safe receipts; coverage always uses the independent source set."""
from collections import Counter
import csv
import json
import os
from pathlib import Path


class SourceCadence:
    """Resume paced acquisition after a stall without inventing catch-up frames."""

    def __init__(self, rate_hz, origin_ns):
        self.rate_hz = rate_hz
        self.period_ns = round(1e9 / rate_hz)
        self.deadline_ns = origin_ns
        self.first_ns = self.last_ns = None
        self.minimum_interval_ns = None
        self.samples = self.resynchronizations = self.missed_ticks = 0
        self.maximum_lateness_ns = 0

    def acquired(self, now_ns):
        """Keep actual acquisition time and account for every scheduling overrun."""
        lateness = max(0, now_ns - self.deadline_ns)
        self.maximum_lateness_ns = max(self.maximum_lateness_ns, lateness)
        if self.last_ns is not None:
            interval = now_ns - self.last_ns
            self.minimum_interval_ns = (interval if self.minimum_interval_ns is None
                                        else min(interval, self.minimum_interval_ns))
        else:
            self.first_ns = now_ns
        self.last_ns = now_ns
        self.samples += 1
        if lateness > self.period_ns // 2:
            self.resynchronizations += 1
            self.missed_ticks += lateness // self.period_ns
            self.deadline_ns = now_ns + self.period_ns
        else:
            self.deadline_ns += self.period_ns

    def receipt(self):
        """Expose source health independently of forecast/command coverage."""
        span = (self.last_ns or 0) - (self.first_ns or 0)
        actual_rate = (self.samples - 1) * 1e9 / span if span > 0 else 0
        minimum = self.minimum_interval_ns or 0
        return {
            'policy': 'actual acquisition; resynchronize after half-period lateness; no catch-up',
            'requested_rate_hz': self.rate_hz,
            'actual_rate_hz': actual_rate,
            'minimum_interval_ns': minimum,
            'maximum_lateness_ns': self.maximum_lateness_ns,
            'resynchronizations': self.resynchronizations,
            'missed_nominal_ticks': self.missed_ticks,
            'samples': self.samples,
            'valid': actual_rate >= .95 * self.rate_hz
            and minimum >= self.period_ns // 2,
        }


def stage_parameters(args, executable):
    if not getattr(args, 'stage_accounting', False):
        return []
    if executable.name not in ('optitrack_mct_relay', 'hope_planner_cpp_node',
                               'hope_integrated_runner_probe'):
        return []
    path = args.output.with_name(args.output.stem + '-' + executable.name + '.stages')
    return [('test.stage_trace_path', str(path))]


def resources(processes):
    result = {}
    for process in processes:
        row = {'pid': process.pid, 'exit': process.poll()}
        try:
            for line in Path(f'/proc/{process.pid}/status').read_text().splitlines():
                if line.startswith(('VmRSS:', 'VmHWM:', 'Threads:', 'Cpus_allowed_list:')):
                    key, value = line.split(':', 1)
                    row[key] = value.strip()
            fields = Path(f'/proc/{process.pid}/stat').read_text().rsplit(')', 1)[1].split()
            row['cpu_s'] = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
        except (OSError, ValueError) as error:
            row['unavailable'] = str(error)
        result[Path(process.args[0]).name] = row
    return result


def validate_source_epoch(source):
    """Reject a source that labels a pre-construction state with a later clock."""
    origin = source['clock_origin_ns']
    rows = source['rows']
    assert rows, 'no source exposures'
    for row in rows:
        assert abs(row['state_elapsed_s'] - (row['stamp_ns'] - origin) * 1e-9) < 1e-12, (
            'state/acquisition timestamp mismatch', row['index'])


def read_stages(path):
    if not path.exists():
        return {'missing': str(path)}, []
    try:
        with path.open() as stream:
            first = stream.readline()
            if first.startswith('#'):
                metadata = dict(item.split('=') for item in first[1:].split())
                return metadata, list(csv.DictReader(stream, delimiter='\t'))
            metadata = json.loads(first)
            return metadata, [dict(zip(('stage', 'source_ns', 'wall_ns', 'steady_ns', 'sequence'),
                                       json.loads(line))) for line in stream]
    except (OSError, ValueError) as error:
        return {'unreadable': str(path), 'error': str(error)}, []


def write_receipt(args, raw, resource, error):
    raw['resources'] = resource
    raw['exception'] = error
    args.output.with_suffix('.raw.json').write_text(json.dumps(raw, indent=2) + '\n')
    expected = set(raw.get('source', {}).get('eligible_stamps', []))
    observed = {int(row['source_time_ns']) for row in raw.get('traces', [])}
    by_source = {}
    stages = {}
    decoder_receipts = {}
    for executable in ('optitrack_mct_relay', 'hope_planner_cpp_node', 'hope_integrated_runner_probe'):
        path = args.output.with_name(args.output.stem + '-' + executable + '.stages')
        if not getattr(args, 'stage_accounting', False):
            continue
        metadata, rows = read_stages(path)
        stages[executable] = {'metadata': metadata, 'counts': dict(Counter(r['stage'] for r in rows))}
        for row in rows:
            by_source.setdefault(int(row['source_ns']), []).append(row)
            if row['stage'] == 'decoder_receipt':
                decoder_receipts.setdefault(int(row['sequence']), []).append(row)
    complete = bool(stages) and all(
        not {'missing', 'unreadable'} & stage['metadata'].keys() and
        int(stage['metadata'].get('overwritten', -1)) == 0
        for stage in stages.values())
    joined = []
    for stamp in sorted(expected):
        rows = by_source.get(stamp, [])
        publications = [r for r in rows if r['stage'] == 'published']
        decoder = [d for p in publications for d in decoder_receipts.get(int(p['sequence']), [])]
        joined.append({'source_ns': stamp, 'trace_observer_present': stamp in observed,
                       'production_stages': rows, 'actual_decoder_receipts': decoder})
    missing = []
    for stamp in sorted(expected - observed):
        rows = by_source.get(stamp, [])
        names = {row['stage'] for row in rows}
        if not getattr(args, 'stage_accounting', False):
            first = 'UNRESOLVED_ACCOUNTING_DISABLED'
        elif 'published' in names:
            first = 'TRACE_OBSERVER_MISSING_PRODUCTION_PUBLICATION_PRESENT'
        elif 'worker_reject' in names or 'worker_exception' in names:
            first = 'WORKER_REJECTED'
        elif 'worker_start' in names:
            first = 'WORKER_STARTED_NO_TERMINAL_RECEIPT'
        elif 'queued' in names:
            first = 'QUEUED_NOT_STARTED_SUPERSEDED_OR_CLOSED'
        elif 'planner_reject' in names:
            first = 'PLANNER_REJECTED'
        elif 'planner_receipt' in names:
            first = 'PLANNER_RECEIVED_NO_TERMINAL_RECEIPT'
        elif 'typed_published' in names:
            first = 'NO_ACTUAL_PLANNER_RECEIPT'
        elif 'relay_receipt' in names:
            first = 'RELAY_RECEIVED_NO_TYPED_PUBLICATION'
        else:
            first = 'NO_RELAY_RECEIPT_OR_ACCOUNTING_INCOMPLETE'
        missing.append({'source_ns': stamp, 'first_observed_loss': first, 'stages': rows,
                        'accounting_complete': complete})
    summary = {'status': 'FAIL' if error else 'PASS', 'exception': error,
               'independent_denominator': len(expected), 'observed_covered': len(expected & observed),
               'observed_coverage': len(expected & observed) / len(expected) if expected else None,
               'missing_epochs': missing, 'resources': resource, 'stages': stages,
               'joined_eligible_epochs': joined, 'accounting_complete': complete,
               'stage_accounting_enabled': getattr(args, 'stage_accounting', False),
               'note': 'Observer absence alone does not establish production loss; every source opportunity remains in coverage.'}
    args.output.with_suffix('.accounting.json').write_text(json.dumps(summary, indent=2) + '\n')
    if error:
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
    return summary
