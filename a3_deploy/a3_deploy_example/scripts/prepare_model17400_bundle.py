#!/usr/bin/env python3
"""Prepare an immutable receive-only bundle locally; never install or start hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import yaml
from pp_fixed_home_profile import planner_profile

POLICY_SHA = '7d81d3250933cba72623de3544dff1abd9ba983aae77f3581f22fdd375dab854'
DEPLOY_SHA = '1e2be028e503c19954d9b0414075a6698dc1e9db93adca73e5d9ea6026baea40'

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,indent=2)+'\n')

def restore_hardware_runtime(deployment):
    """Restore MDU settings that generic Runner distributions may omit."""
    source = Path(__file__).resolve().parents[1]/'src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.rockchip.yaml'
    hardware = yaml.safe_load(source.read_text())
    runtime = deployment/'config/a3_runtime_config.pingpong.hitter_pingpong.yaml'
    config = yaml.safe_load(runtime.read_text())
    for key in ('background_cpu', 'driver_cpu', 'serve_release_lead_frames'):
        config[key] = hardware[key]
    config['backend']['sync_cpu'] = hardware['backend']['sync_cpu']
    runtime.write_text(yaml.safe_dump(config, sort_keys=False))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--runner-dist',type=Path,required=True)
    parser.add_argument('--inspection-runner-dist', type=Path, help='Native build of the same source when staging ARM64 on x86')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); package=args.package.resolve();dist=args.runner_dist.resolve();out=args.output.resolve()
    for rel,expected in [('policy/exported/policy.onnx',POLICY_SHA),('policy/params/deploy.yaml',DEPLOY_SHA)]:
        if digest(package/rel)!=expected:raise ValueError('model17400 identity mismatch: '+rel)
    runner=dist/'a3_deploy_onnx_ref_pingpong'
    header = runner.read_bytes()[:20]
    if header[:4] != b'\x7fELF' or header[5] != 1:
        raise ValueError('expected little-endian ELF Runner')
    machine = int.from_bytes(header[18:20], 'little')
    if machine not in (62, 183):
        raise ValueError('unsupported Runner architecture')
    required = ('run_a3_pingpong.sh', 'config/a3_runtime_config.pingpong.hitter_pingpong.yaml',
                'config/a3_aimrt_config.pingpong_iceoryx.yaml', 'tools/a3p_gripper_bridge.py')
    for relative in required:
        if not (dist / relative).is_file():
            raise ValueError('incomplete Runner distribution: ' + relative)
    for artifact in dist.rglob('*'):
        if artifact.is_file():
            with artifact.open('rb') as stream:
                elf = stream.read(20)
            if elf[:4] == b'\x7fELF' and (elf[5] != 1 or int.from_bytes(elf[18:20], 'little') != machine):
                raise ValueError('mixed architecture distribution: ' + str(artifact))
    inspection_dist = (args.inspection_runner_dist or dist).resolve()
    inspector = inspection_dist/'a3_deploy_onnx_ref_pingpong'
    env={**os.environ,'LD_LIBRARY_PATH':f'{inspection_dist}:{inspection_dist}/lib:/opt/ros/jazzy/lib:'+os.environ.get('LD_LIBRARY_PATH','')}
    native=subprocess.run([str(inspector),'--inspect-policy','--policy-dir',str(package/'policy')],
                          env=env,capture_output=True,text=True,check=True,timeout=60)
    inspection=yaml.safe_load(native.stdout);profile,plane=planner_profile(inspection)
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(dist,out/'deployment',symlinks=False)
    policy_target = out/'deployment/policy'
    if policy_target.exists():
        shutil.rmtree(policy_target)  # Only the newly created, unpublished staging directory.
    shutil.copytree(package/'policy', policy_target)
    # HOME publication was absent from some older distributions. Use this source's config.
    source_config = Path(__file__).resolve().parents[1]/'src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.pingpong_ros2body.yaml'
    shutil.copy2(source_config, out/'deployment/config'/source_config.name)
    # ARM bundles target the Rockchip MDU. Do not inherit an unbound simulation
    # runtime from runner-dist; restore hardware defaults before hashing files.
    if machine == 183:
        restore_hardware_runtime(out/'deployment')
    (out/'deployment/receive_only.policy_sha256').write_text(POLICY_SHA+'\n')
    write(out/'model17400.json',profile)
    identity={'policy_sha256':POLICY_SHA,'deploy_sha256':DEPLOY_SHA,'receive_only':True,'required_ball_model':'ppq004'}
    destination='/agibot/a3_deploy_model17400';hdu='/var/lib/hope-lifecycle/policy-models/model17400.json'
    files={destination+'/'+str(p.relative_to(out/'deployment')):digest(p) for p in (out/'deployment').rglob('*') if p.is_file()}
    write(out/'mdu_entry.json',{**identity,'deployment':destination,'files':files})
    write(out/'hdu_entry.json',{**identity,'planner_profile':hdu,'files':{hdu:digest(out/'model17400.json')}})
    write(out/'receipt.json',{'inspection':inspection,'selected_plane_x_relative_home':plane,'identity':identity,
        'runner_sha256':digest(runner), 'inspection_runner_sha256':digest(inspector),
        'architecture': 'aarch64' if machine == 183 else 'x86_64',
        'policy_inspected_with_staged_runner': runner == inspector,
        'status': 'STAGED_NOT_INSTALLED' if machine == 183 else 'SIMULATION_ONLY', 'serve_qualified':False})
    print(out)
if __name__=='__main__':main()
