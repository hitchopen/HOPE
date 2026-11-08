#!/usr/bin/env python3
"""Stage a model-specific serve bundle with native and simulation evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import yaml
from pp_fixed_home_profile import planner_profile
from prepare_model12000_bundle import architecture, digest, restore_hardware_runtime, write

CSV_NAMES = {
    'model14600': 'a3p_op3_serve025_v23_branch110_model14600.csv',
    'model12600': 'v23-12600.csv',
    'model17400': 'v23-17400.csv',
    'model12000': 'v23-12000.csv',
}
LEGACY_14600_PROFILE_SHA = 'ec7e2a5a288e3b573bb1c67456701ac43f34bbbf8490d62e885b38bc032cf955'


def validate_evidence(recipe, evidence, policy, csv):
    identity = {'policy_sha256': digest(policy/'exported/policy.onnx'),
                'deploy_sha256': digest(policy/'params/deploy.yaml'), 'csv_sha256': digest(csv)}
    if any(recipe.get(k) != identity[k] for k in ('policy_sha256', 'deploy_sha256')):
        raise ValueError('recipe does not match the policy/deploy bytes')
    if any(evidence.get(k) != v for k, v in identity.items()):
        raise ValueError('simulation evidence does not match the policy/deploy/CSV bytes')
    for name in ('normal', 'kernel', 'normal_yaw', 'pitch_plus', 'pitch_minus'):
        case = evidence.get('cases', {}).get(name, {})
        if case.get('status') != 'PASS' or case.get('clock') != 'simulation':
            raise ValueError('missing successful simulation case: ' + name)
        if name in ('normal', 'kernel') and len(case.get('cycles', [])) < 3:
            raise ValueError('repeated-cycle validation required: ' + name)
        if not 0 <= case.get('transition_torque_ratio', float('inf')) <= 1:
            raise ValueError('transition exceeds simulated torque limits: ' + name)
        boundaries = case.get('boundaries', [])
        if not boundaries or any(len(b.get('q_dq_tau_kp_kd', [])) != 5 or
                any(not math.isfinite(v) or abs(v) > 1e-9 for v in b['q_dq_tau_kp_kd']) for b in boundaries):
            raise ValueError('native handoff command continuity not established: ' + name)
    return identity


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=CSV_NAMES, required=True)
    for name in ('policy-dir', 'runner-dist', 'inspection-runner-dist', 'csv', 'recipe', 'simulation-receipt', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--slot-suffix', required=True)
    p.add_argument('--legacy-planner-profile', type=Path)
    a = p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_]+', a.slot_suffix):
        p.error('invalid deployment slot suffix')
    if a.csv.name != CSV_NAMES[a.model]:
        p.error('model-specific CSV name required')
    recipe = json.loads(a.recipe.read_text())
    evidence = json.loads(a.simulation_receipt.read_text())
    identity = validate_evidence(recipe, evidence, a.policy_dir, a.csv)
    with tempfile.TemporaryDirectory() as temp:
        reproduced = Path(temp)/'reproduced.csv'
        subprocess.run([os.sys.executable, str(Path(__file__).with_name('generate_serve_segments.py')),
                        '--recipe', str(a.recipe), '--output', str(reproduced)], check=True, capture_output=True)
        if digest(reproduced) != identity['csv_sha256']:
            raise ValueError('CSV is not reproducible from its recipe')
    runner = a.runner_dist/'a3_deploy_onnx_ref_pingpong'
    machine = architecture(runner)
    for item in a.runner_dist.rglob('*'):
        if item.is_file():
            with item.open('rb') as stream:
                header = stream.read(4)
            if header == b'\x7fELF' and architecture(item) != machine:
                raise ValueError('mixed-architecture Runner distribution')
    env = {**os.environ, 'LD_LIBRARY_PATH': f'{a.inspection_runner_dist}:{a.inspection_runner_dist}/lib:/opt/ros/jazzy/lib:' + os.environ.get('LD_LIBRARY_PATH', '')}
    inspector = a.inspection_runner_dist/runner.name
    def inspect(*flags):
        result = subprocess.run([str(inspector), *flags, '--policy-dir', str(a.policy_dir)], env=env,
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise ValueError('native validation failed: ' + result.stdout + result.stderr)
        return result.stdout
    inspection = yaml.safe_load(inspect('--inspect-policy'))
    if a.model == 'model14600':
        if (not a.legacy_planner_profile or digest(a.legacy_planner_profile) != LEGACY_14600_PROFILE_SHA
                or inspection.get('policy_abi') != 'ball_clock_110_v1'):
            raise ValueError('model14600 requires its verified legacy Planner profile')
        profile = json.loads(a.legacy_planner_profile.read_text())
    else:
        profile, _ = planner_profile(inspection)
    validation = inspect('--validate-serve-timeline', str(a.csv), '--validate-serve-stand')
    if 'SERVE_SEGMENTS_VALID' not in validation or 'SERVE_STAND_VALID' not in validation:
        raise ValueError('native segmented serve validation required')
    a.output.mkdir(parents=True, exist_ok=False)
    deploy = a.output/'deployment'
    shutil.copytree(a.runner_dist, deploy, symlinks=False, ignore=shutil.ignore_patterns('logs', 'policy'))
    shutil.copytree(a.policy_dir, deploy/'policy')
    (deploy/'receive_only.policy_sha256').unlink(missing_ok=True)
    (deploy/'motions').mkdir(exist_ok=True)
    shutil.copy2(a.csv, deploy/'motions'/a.csv.name)
    shutil.copy2(a.recipe, deploy/'motions'/a.recipe.name)
    if machine == 183:
        restore_hardware_runtime(deploy)
    assets = a.output/'field-assets'; assets.mkdir()
    shutil.copy2(a.csv, assets/(identity['csv_sha256']+'.csv'))
    write(a.output/(a.model+'.json'), profile)
    shutil.copy2(a.simulation_receipt, a.output/'simulation-receipt.json')
    destination = '/agibot/a3_deploy_'+a.model+'_'+a.slot_suffix
    hdu = '/var/lib/hope-lifecycle/policy-models/'+a.model+'.json'
    entry = {**identity, 'receive_only': False, 'required_ball_model': 'ppq004',
             'serve_csv': {'name': a.csv.name, 'sha256': identity['csv_sha256']}}
    files = {destination+'/'+str(f.relative_to(deploy)): digest(f) for f in deploy.rglob('*') if f.is_file()}
    files['/agibot/hope-field-assets/'+identity['csv_sha256']+'.csv'] = identity['csv_sha256']
    write(a.output/'mdu_entry.json', {**entry, 'deployment': destination, 'files': files})
    write(a.output/'hdu_entry.json', {**entry, 'planner_profile': hdu, 'files': {hdu: digest(a.output/(a.model+'.json'))}})
    write(a.output/'receipt.json', {'model': a.model, 'identity': identity, 'inspection': inspection,
          'native_serve_validation': validation, 'runner_sha256': digest(runner),
          'inspection_runner_sha256': digest(inspector), 'recipe_sha256': digest(a.recipe),
          'simulation_receipt_sha256': digest(a.simulation_receipt),
          'architecture': 'aarch64' if machine == 183 else 'x86_64',
          'status': 'STAGED_NOT_INSTALLED', 'physical_serve_qualified': False})
    print(a.output)


if __name__ == '__main__':
    main()
