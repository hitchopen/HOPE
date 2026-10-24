#!/usr/bin/env python3
"""Inspect installed V2 libraries/node and evaluation adapter for command dependencies."""
import argparse
import json
from pathlib import Path
import re
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--executable', required=True)
args = parser.parse_args()
executable = Path(args.executable)
forbidden = re.compile(r'RacketTargetPlanner::|Schema[23]Packer::|pack_schema[23]|planner_locked')
# Colcon's executable may be in build/ or install/<package>/lib/<package>/.
if executable.parent.name == 'hope_planner_cpp' and executable.parent.parent.name == 'lib':
    libraries = executable.parent.parent
else:
    libraries = executable.parent
paths = [executable] + sorted(libraries.glob('libhope_v2*'))
baseline = executable.with_name('hope_ball_flight_baseline_replay')
if baseline.exists():
    paths.append(baseline)
for path in paths:
    symbols = subprocess.check_output(['nm', '-C', str(path)], text=True)
    assert not forbidden.search(symbols), path
    if path == executable:
        dependencies = subprocess.check_output(['ldd', str(path)], text=True)
        assert 'libhope_planner_core' not in dependencies
print(json.dumps(dict(status='PASS', inspected=[str(p) for p in paths],
                      inverse_and_command_symbols_absent=True)))
