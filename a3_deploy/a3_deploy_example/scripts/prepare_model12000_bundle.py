#!/usr/bin/env python3
"""Stage model12000 and its validated serve CSV; never install or start hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import yaml
from pp_fixed_home_profile import planner_profile

POLICY_SHA = '6df3556eda0fdf01fb7d7a8f3d5a95f8d57a5df29af6082b3c5ee4a796529a45'
DEPLOY_SHA = '207cf4bf39441374a4af473ec28884cca66bca395cb6faa7389382633f0eccd5'
CSV_NAME = 'v23-12000.csv'
MODEL = 'model12000'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def validate_receipt(receipt, csv):
    expected = {'policy_sha256': POLICY_SHA, 'deploy_sha256': DEPLOY_SHA, 'csv_sha256': digest(csv)}
    if any(receipt.get(key) != value for key, value in expected.items()):
        raise ValueError('simulation receipt does not match the policy/deploy/CSV bytes')
    for name in ('normal', 'kernel', 'normal_yaw', 'pitch_plus', 'pitch_minus'):
        case = receipt.get('cases', {}).get(name, {})
        if case.get('status') != 'PASS' or case.get('clock') != 'simulation':
            raise ValueError('missing successful simulation case: ' + name)
        if name in ('normal', 'kernel') and len(case.get('cycles', [])) < 3:
            raise ValueError('repeated-cycle validation required: ' + name)
    return expected


def architecture(path):
    with path.open('rb') as stream:
        header = stream.read(20)
    if header[:4] != b'\x7fELF' or header[5] != 1:
        raise ValueError('expected little-endian ELF Runner')
    machine = int.from_bytes(header[18:20], 'little')
    if machine not in (62, 183):
        raise ValueError('unsupported Runner architecture')
    return machine


def restore_hardware_runtime(deployment):
    config_root = Path(__file__).resolve().parents[1] / 'src/a3/a3_deploy_onnx_ref/config'
    hardware = yaml.safe_load((config_root / 'a3_runtime_config.pingpong.hitter_pingpong.rockchip.yaml').read_text())
    path = deployment / 'config/a3_runtime_config.pingpong.hitter_pingpong.yaml'
    config = yaml.safe_load(path.read_text())
    for key in ('background_cpu', 'driver_cpu', 'serve_release_lead_frames'):
        config[key] = hardware[key]
    config['backend']['sync_cpu'] = hardware['backend']['sync_cpu']
    path.write_text(yaml.safe_dump(config, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-dir', type=Path, required=True)
    parser.add_argument('--runner-dist', type=Path, required=True)
    parser.add_argument('--inspection-runner-dist', type=Path)
    parser.add_argument('--csv', type=Path, required=True)
    parser.add_argument('--simulation-receipt', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    policy, dist, csv, out = (p.resolve() for p in (args.policy_dir, args.runner_dist, args.csv, args.output))
    for relative, expected in [('exported/policy.onnx', POLICY_SHA), ('params/deploy.yaml', DEPLOY_SHA)]:
        if digest(policy / relative) != expected:
            raise ValueError('model12000 identity mismatch: ' + relative)
    evidence = json.loads(args.simulation_receipt.read_text())
    identity = validate_receipt(evidence, csv)
    runner = dist / 'a3_deploy_onnx_ref_pingpong'
    machine = architecture(runner)
    for relative in ('run_a3_pingpong.sh', 'config/a3_runtime_config.pingpong.hitter_pingpong.yaml',
                     'config/a3_aimrt_config.pingpong_iceoryx.yaml', 'tools/a3p_gripper_bridge.py'):
        if not (dist / relative).is_file():
            raise ValueError('incomplete Runner distribution: ' + relative)
    for path in dist.rglob('*'):
        if path.is_file():
            with path.open('rb') as stream:
                header = stream.read(20)
            if header[:4] == b'\x7fELF' and architecture(path) != machine:
                raise ValueError('mixed architecture distribution: ' + str(path))
    inspection_dist = (args.inspection_runner_dist or dist).resolve()
    inspector = inspection_dist / runner.name
    env = {**os.environ, 'LD_LIBRARY_PATH': f'{inspection_dist}:{inspection_dist}/lib:/opt/ros/jazzy/lib:' + os.environ.get('LD_LIBRARY_PATH', '')}
    def inspect(*flags):
        result = subprocess.run([str(inspector), *flags, '--policy-dir', str(policy)], env=env,
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise ValueError('native validation failed: ' + result.stdout + result.stderr)
        return result.stdout
    inspection = yaml.safe_load(inspect('--inspect-policy'))
    profile, plane = planner_profile(inspection)
    serve_validation = inspect('--validate-serve-timeline', str(csv), '--validate-serve-stand')
    if 'SERVE_SEGMENTS_VALID' not in serve_validation or 'SERVE_STAND_VALID' not in serve_validation:
        raise ValueError('native segmented serve validation required')
    out.mkdir(parents=True, exist_ok=False)
    deployment = out / 'deployment'
    shutil.copytree(dist, deployment, symlinks=False, ignore=shutil.ignore_patterns('logs', 'policy'))
    shutil.copytree(policy, deployment / 'policy')
    (deployment / 'receive_only.policy_sha256').unlink(missing_ok=True)
    (deployment / 'motions').mkdir(exist_ok=True)
    shutil.copy2(csv, deployment / 'motions' / CSV_NAME)
    source_config = Path(__file__).resolve().parents[1] / 'src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.pingpong_ros2body.yaml'
    shutil.copy2(source_config, deployment / 'config' / source_config.name)
    if machine == 183:
        restore_hardware_runtime(deployment)
    assets = out / 'field-assets'
    assets.mkdir()
    asset_name = identity['csv_sha256'] + '.csv'
    shutil.copy2(csv, assets / asset_name)
    write(out / 'model12000.json', profile)
    shutil.copy2(args.simulation_receipt, out / 'simulation-receipt.json')
    destination = '/agibot/a3_deploy_model12000'
    hdu = '/var/lib/hope-lifecycle/policy-models/model12000.json'
    entry = {**identity, 'receive_only': False, 'required_ball_model': 'ppq004',
             'serve_csv': {'name': CSV_NAME, 'sha256': identity['csv_sha256']}}
    files = {destination + '/' + str(p.relative_to(deployment)): digest(p)
             for p in deployment.rglob('*') if p.is_file()}
    files['/agibot/hope-field-assets/' + asset_name] = identity['csv_sha256']
    write(out / 'mdu_entry.json', {**entry, 'deployment': destination, 'files': files})
    write(out / 'hdu_entry.json', {**entry, 'planner_profile': hdu, 'files': {hdu: digest(out / 'model12000.json')}})
    write(out / 'receipt.json', {'model': MODEL, 'identity': identity, 'inspection': inspection,
        'native_serve_validation': serve_validation, 'selected_plane_x_relative_home': plane,
        'runner_sha256': digest(runner), 'inspection_runner_sha256': digest(inspector),
        'policy_inspected_with_staged_runner': runner == inspector,
        'architecture': 'aarch64' if machine == 183 else 'x86_64',
        'simulation_receipt_sha256': digest(out / 'simulation-receipt.json'),
        'status': 'STAGED_NOT_INSTALLED' if machine == 183 else 'SIMULATION_ONLY',
        'physical_serve_qualified': False})
    print(out)


if __name__ == '__main__':
    main()
