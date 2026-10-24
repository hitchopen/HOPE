#!/usr/bin/env python3
"""An actual assertion must leave receipts and no test publishers running."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('planner', 'decoder', 'bringup', 'output'):
        parser.add_argument('--' + key, required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    child_output = args.output.with_name(args.output.stem + '-expected-failure.json')
    command = [sys.executable, str(Path(__file__).with_name('integrated_producer_chain.py')),
               '--planner', str(args.planner), '--decoder', str(args.decoder),
               '--bringup', str(args.bringup), '--output', str(child_output), '--rate', '150',
               '--stage-accounting', '--test-fail-after-collection']
    result = subprocess.run(command, capture_output=True, text=True, timeout=40)
    child_output.with_suffix('.log').write_text(result.stdout + result.stderr)
    assert result.returncode != 0, 'the deliberate assertion did not fire'
    receipt = json.loads(child_output.read_text())
    raw = json.loads(child_output.with_suffix('.raw.json').read_text())
    assert receipt['status'] == 'FAIL'
    assert receipt['exception'] == 'AssertionError: injected failure after source collection'
    assert receipt['independent_denominator'] > 0 and raw['source']['rows']
    assert receipt['resources'] and receipt['accounting_complete']
    for name, resource in receipt['resources'].items():
        cmdline = Path(f"/proc/{resource['pid']}/cmdline")
        assert not cmdline.exists() or name.encode() not in cmdline.read_bytes(), (name, resource)
    # A launch failure after other nodes started must also enter cleanup.
    startup_output = args.output.with_name(args.output.stem + '-expected-startup-failure.json')
    startup = command.copy()
    startup.remove('--test-fail-after-collection')
    startup[startup.index('--output') + 1] = str(startup_output)
    startup[startup.index('--decoder') + 1] = str(args.output.with_suffix('.missing-decoder'))
    setup = subprocess.run(startup, capture_output=True, text=True, timeout=20)
    startup_output.with_suffix('.log').write_text(setup.stdout + setup.stderr)
    assert setup.returncode != 0
    setup_receipt = json.loads(startup_output.read_text())
    assert setup_receipt['status'] == 'FAIL'
    assert setup_receipt['exception'].startswith('FileNotFoundError:')
    assert len(setup_receipt['resources']) == 3
    assert startup_output.with_suffix('.raw.json').exists()
    for name, resource in setup_receipt['resources'].items():
        cmdline = Path(f"/proc/{resource['pid']}/cmdline")
        assert not cmdline.exists() or name.encode() not in cmdline.read_bytes(), (name, resource)
    args.output.write_text(json.dumps({'status': 'PASS', 'injected_failure_retained': True,
                                      'partial_startup_failure_retained': True,
                                      'independent_denominator': receipt['independent_denominator'],
                                      'children_reaped': True, 'accounting_complete': True}, indent=2) + '\n')


if __name__ == '__main__':
    main()
