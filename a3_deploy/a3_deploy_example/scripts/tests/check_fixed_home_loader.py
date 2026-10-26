#!/usr/bin/env python3
"""Run strict metadata rejection checks against a real native Runner, without HAL."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import onnx


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner-dist', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--legacy-policy', type=Path, required=True)
    args = parser.parse_args()
    dist = args.runner_dist.resolve()
    env = {**os.environ, 'LD_LIBRARY_PATH': f'{dist}:{dist}/lib:/opt/ros/jazzy/lib:'
           + os.environ.get('LD_LIBRARY_PATH', '')}

    def inspect(policy):
        return subprocess.run(
            [str(dist / 'a3_deploy_onnx_ref_pingpong'), '--inspect-policy',
             '--policy-dir', str(policy)], env=env, capture_output=True, text=True, timeout=30)

    results = []
    for name, path in [('model17400', args.policy), ('legacy', args.legacy_policy)]:
        result = inspect(path.resolve())
        if result.returncode:
            raise RuntimeError(f'{name} unexpectedly rejected: {result.stderr}')
        results.append({'case': name, 'accepted': True})

    mutations = {
        'unknown_station': ('hitter_pure_station_contract', 'unknown_v99'),
        'moving_station': ('hitter_pure_base_target_range', '-0.1,0.1,0,0'),
        'wrong_hold': ('hitter_pure_hold_pose_contract', 'other_hold'),
        'wrong_recovery': ('hitter_pure_recovery_target_contract', 'per_ball_anchor'),
        'added_delay': ('hitter_pingpong_deploy_delay_contract', 'added_delay_v1'),
        'wrong_sample': ('hitter_pure_target_sample_contract', 'old_sample'),
    }
    with tempfile.TemporaryDirectory() as temporary:
        target = Path(temporary) / 'policy'
        shutil.copytree(args.policy, target)
        original = onnx.load(args.policy / 'exported/policy.onnx')
        for name, (key, value) in mutations.items():
            model = onnx.ModelProto()
            model.CopyFrom(original)
            metadata = {p.key: p.value for p in model.metadata_props}
            if key not in metadata:
                raise RuntimeError(f'mutation key absent: {key}')
            metadata[key] = value
            onnx.helper.set_model_props(model, metadata)
            onnx.save(model, target / 'exported/policy.onnx')
            result = inspect(target)
            if result.returncode == 0:
                raise RuntimeError(f'unsafe contract accepted: {name}')
            results.append({'case': name, 'accepted': False,
                            'diagnostic': (result.stderr or result.stdout)[-1000:]})
    print(json.dumps({'passed': True, 'checks': results}, indent=2))


if __name__ == '__main__':
    main()
