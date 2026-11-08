#!/usr/bin/env python3
"""Derive and simulate a Fusion-specific v23 handoff, without hardware access."""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path
import subprocess
import sys

import numpy as np
from validate_serve_segments_mujoco import run


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-dir', type=Path, required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--model', type=Path, help='MuJoCo robot XML used for all adaptation and validation cases')
    parser.add_argument('--csv-name', default='v23-fusion')
    args = parser.parse_args()
    if args.model:
        import validate_spin001_serve_mujoco as sim
        sim.DEFAULT_MODEL = args.model.resolve()
    if not re.fullmatch(r"v23-[a-z0-9-]+", args.csv_name):
        parser.error("invalid CSV name")
    if args.cycles < 3:
        parser.error('at least three repeat cycles required')
    args.output.mkdir(parents=True, exist_ok=False)
    motions = Path(__file__).resolve().parents[1]/'assets/a3_runtime/serve/motions'
    recipe = json.loads((motions/'v23-13800.recipe.json').read_text())
    source = (motions/recipe['source_csv']).resolve()
    shutil.copy2(source, args.output/source.name)
    trace = args.output/'ready_reference.npz'
    capture = run(args.library, args.policy_dir, source, False, 0., 15.,
                  cycles=1, stand_offset=.03, trace_path=trace,
                  segmented=False, blend=.5)
    (args.output/'ready_reference.json').write_text(json.dumps(capture, indent=2)+'\n')
    data = np.load(trace)
    selected = data['t'] > data['t'][-1]-1
    command = data['command'][selected]
    q = data['qpos'][selected][:, data['qa']]
    dq = data['qvel'][selected][:, data['va']]
    target = command.mean(axis=0)
    target[0] = q.mean(axis=0)
    target[1] = 0
    target[2] = (command[:, 3]*(command[:, 0]-q) +
                 command[:, 4]*(command[:, 1]-dq) + command[:, 2]).mean(axis=0)
    target[0, [23, 29]] += .04
    identity = {'policy_sha256': sha(args.policy_dir/'exported/policy.onnx'),
                'deploy_sha256': sha(args.policy_dir/'params/deploy.yaml')}
    recipe.update(**identity, name=args.csv_name, source_csv=source.name,
        policy_ready_command=target.tolist(),
        target_derivation='Selected policy native 15 s WAIT rollout: final 1 s mean measured SDK31 posture and support torque, +0.04 rad ankle pitch; unchanged MuJoCo contact physics.',
        selection='Candidate; continuous-ball Gate3 and physical qualification are separate.')
    recipe_path = args.output/(args.csv_name+'.recipe.json')
    recipe_path.write_text(json.dumps(recipe, indent=2)+'\n')
    csv = args.output/(args.csv_name+'.csv')
    subprocess.run([sys.executable, str(Path(__file__).with_name('generate_serve_segments.py')),
                    '--recipe', str(recipe_path), '--output', str(csv)], check=True)
    subprocess.run([sys.executable, str(Path(__file__).with_name('validate_serve_segments_mujoco.py')),
                    '--library', str(args.library), '--policy-dir', str(args.policy_dir),
                    '--csv', str(csv), '--cycles', str(args.cycles),
                    '--output-dir', str(args.output/'segments'),
                    *(['--model', str(args.model.resolve())] if args.model else [])], check=True)
    receipt = {**identity, 'csv_sha256': sha(csv),
               'cases': json.loads((args.output/'segments/results.json').read_text()),
               'model': str(args.model.resolve()) if args.model else None,
               'model_sha256': sha(args.model.resolve()) if args.model else None,
               'scope': 'Serve transitions and WAIT only; no continuous-ball or hardware qualification'}
    (args.output/'simulation-receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')


if __name__ == '__main__':
    main()
