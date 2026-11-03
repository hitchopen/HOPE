#!/usr/bin/env python3
"""Shadow integration must never acquire live publication authority."""
import argparse
import json
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--node', required=True)
args = p.parse_args()
cases = [('output_mode', 'live', 'use shadow'),
         ('runner_contract', 'unregistered', 'runner/source contract'),
         ('return_backend', 'legacy', 'shared_physics')]
for key, value, reason in cases:
    run = subprocess.run([args.node, '--ros-args', '-p', 'prediction_backend:=v2',
                          '-p', f'{key}:={value}'], capture_output=True, text=True, timeout=10)
    assert run.returncode != 0 and reason.lower() in (run.stdout+run.stderr).lower(), (key, run.returncode, run.stderr)
print(json.dumps({'status': 'PASS', 'rejected_startups': [key for key, _, _ in cases]}))
