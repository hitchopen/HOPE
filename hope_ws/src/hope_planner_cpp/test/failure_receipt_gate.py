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
    args.output.write_text(json.dumps({'status': 'PASS', 'injected_failure_retained': True,
                                      'independent_denominator': receipt['independent_denominator'],
                                      'children_reaped': True, 'accounting_complete': True}, indent=2) + '\n')


if __name__ == '__main__':
    main()
