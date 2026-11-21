#!/usr/bin/env python3
"""Resumable, fail-closed native build -> parity -> inspect -> reference Gate3.

Run from the normal desktop. No dependency installation or hardware launch.
All builds, logs and receipts stay outside Git; no historical install is changed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
GEAR = ROOT / 'a3_deploy/a3_deploy_example'
DEFAULT = ROOT.parent / 'hope-stationary-gru-evaluations/20261108-model19000-native-v2'

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((HERE / 'manifest.json').read_text())
    actor = HERE / 'policy/exported/policy.onnx'
    deploy = HERE / 'policy/params/deploy.yaml'
    assert digest(actor) == manifest['candidate_onnx_sha256']
    assert digest(deploy) == manifest['deploy_yaml_sha256']
    # Original inventories are immutable historical evidence, never overwritten here.
    preserved = json.loads((DEFAULT / 'preserved-files-before.json').read_text())
    for relative, expected in preserved.items():
        assert digest(ROOT / relative) == expected, f'Original changed: {relative}'
    checkpoint = ROOT.parent / ('hope-stationary-gru-evaluations/'
        '20261106-eight-checkpoint-comparison/model_19000/model_19000.pt')
    assert digest(checkpoint) == manifest['checkpoint_sha256']
    for item in manifest['reference_assets']:
        assert digest(ROOT / item['path']) == item['sha256']

    # Content binding survives a commit but refuses stale binaries after source edits.
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).split(b'\0')
    prefixes = ('a3_deploy/', 'hope_ws/', 'NatNet2ROS2/', 'run_gate3.sh')
    sources = {p.decode(): digest(ROOT / p.decode()) for p in tracked if p and
               p.decode().startswith(prefixes) and (ROOT / p.decode()).is_file()}
    for path in HERE.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            sources[str(path.relative_to(ROOT))] = digest(path)
    header = GEAR / 'src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_explicit_gru.hpp'
    sources[str(header.relative_to(ROOT))] = digest(header)
    for name in ('obs.f32', 'actions.f32', 'hidden.f32', 'initial_h.f32', 'refs.f32'):
        sources[str(DEFAULT / name)] = digest(DEFAULT / name)
    binding = hashlib.sha256(json.dumps(sources, sort_keys=True).encode()).hexdigest()
    lock = out / 'source-binding.json'
    if lock.exists():
        assert json.loads(lock.read_text())['binding'] == binding, (
            'Sources changed since this build. Select a new --output; previous evidence retained.')
    else:
        write(lock, {'binding': binding, 'sources': sources,
                     'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()})
    print(f'Checkpoint: model_19000; run: {manifest["run_id"]}', flush=True)
    print(f'ONNX SHA256: {digest(actor)}\nDeployment SHA256: {digest(deploy)}', flush=True)
    print(f'Scenario: fixed-home-100, unchanged; output: {out}', flush=True)

    def stage(name, command, outputs=(), container=True, terminal_result=False, extra_roots=()):
        receipt = out / f'{name}.receipt.json'
        if receipt.exists():
            saved = json.loads(receipt.read_text())
            if saved['binding'] == binding and saved['completed'] and all(
                    Path(p).is_file() and digest(p) == h for p, h in saved['outputs'].items()):
                print(f'SKIP verified {name} ({saved["seconds"]:.1f}s)', flush=True)
                return saved['exit_code']
        prefix = 'set -eo pipefail\nsource /opt/ros/jazzy/setup.bash\n'
        argv = ['distrobox', 'enter', 'hope', '--', 'bash', '-lc', prefix + command] if container else command
        write(out / f'{name}.command.json', {'argv': argv})
        print(f'START {name}', flush=True)
        start = time.monotonic()
        with (out / f'{name}.log').open('a') as log:
            result = subprocess.run(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        missing = [str(p) for p in outputs if not Path(p).is_file()]
        terminal = False
        if terminal_result and not missing:
            final = json.loads(Path(outputs[0]).read_text())
            terminal = (final.get('status') in ('finished', 'failed') and
                        isinstance(final.get('certification_pass'), bool) and
                        'exit_code' in final)
        complete = not missing and (terminal if terminal_result else result.returncode == 0)
        artifacts = set(map(Path, outputs))
        for directory in extra_roots:
            artifacts.update(p for p in Path(directory).rglob('*') if p.is_file())
        write(receipt, {'binding': binding, 'completed': complete,
                       'exit_code': result.returncode, 'seconds': time.monotonic()-start,
                       'outputs': {str(p): digest(p) for p in sorted(artifacts) if p.is_file()},
                       'missing': missing})
        print(f'END {name}: exit={result.returncode}, {time.monotonic()-start:.1f}s; log={out / (name+".log")}', flush=True)
        if not complete:
            raise RuntimeError(f'{name} blocked/failed; no later stage started (see log)')
        return result.returncode

    stage('container-access', ['podman', 'inspect', 'hope', '--format', '{{.State.Status}}'], container=False)
    q = shlex.quote
    ws = out / 'ros-ws'
    ws.mkdir(exist_ok=True)
    if not (ws / 'src').exists():
        (ws / 'src').symlink_to(ROOT / 'hope_ws/src', target_is_directory=True)
    install = ws / 'install'
    stage('ros-build', f'''colcon --log-base {q(str(out / 'ros-log'))} build \
 --base-paths {q(str(ROOT / 'hope_ws/src'))} {q(str(ROOT / 'NatNet2ROS2/src'))} \
 --packages-up-to hope_msgs hope_bringup hope_planner_cpp motion_capture_tracking \
 --build-base {q(str(out / 'ros-build'))} --install-base {q(str(install))} \
 --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source {q(str(install / 'local_setup.bash'))}
/usr/bin/python3 -c 'from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2'
''', [install / 'local_setup.bash', install / 'hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node'], extra_roots=[install])
    build, dist = out / 'runner-build', out / 'runner-dist'
    thirdparty = Path(os.environ.get('HOPE_NATIVE_THIRDPARTY', str(ROOT.parent / 'nohope/a3_deploy/a3_deploy_example/thirdparty')))
    sdk = thirdparty / 'onnxruntime/onnxruntime-linux-x64-1.19.2'
    # Reuse source downloads by COPY, since the package builder patches its AimRT sources.
    # Never symlink old sources or old compiled binaries into this new build.
    cache = ROOT.parent / 'nohope/agibot/code_deployment/a3_deploy_example/build/a3_pkg_x86_64/_deps'
    env = f'''source {q(str(install / 'local_setup.bash'))}
export A3_PACKAGE_BUILD_DIR={q(str(build))} A3_PACKAGE_OUTPUT_DIR={q(str(dist))}
export A3_THIRDPARTY_ROOT={q(str(thirdparty))} onnxruntime_ROOT={q(str(sdk))}
export A3_CMAKE_PYTHON=/usr/bin/python3 HAS_ROS2=1
'''
    stage('runner-build', env + f'''
mkdir -p {q(str(build / '_deps'))}
for saved_source in {q(str(cache))}/*-src; do
  [[ -d "$saved_source" ]] || continue
  destination={q(str(build / '_deps'))}/"$(basename "$saved_source")"
  [[ -e "$destination" ]] || cp -a "$saved_source" "$destination"
done
cd {q(str(GEAR))}
test -f {q(str(thirdparty / 'unitree_sdk2/CMakeLists.txt'))}
test -f {q(str(sdk / 'include/onnxruntime_cxx_api.h'))}
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --without-teleop --jobs 4 \
 --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml \
 --policy-dir {q(str(HERE / 'policy'))}
cmake --build {q(str(build))} --target model19000_native_parity --parallel 4
''', [dist / 'a3_deploy_onnx_ref_pingpong', dist / 'model19000_native_parity', dist / 'run_a3_pingpong.sh'], extra_roots=[dist])
    sim_build = out / 'sim-build'
    sim_install = sim_build / 'install'
    stage('sim-build', env + f'''cmake -S {q(str(ROOT / 'a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim'))} \
 -B {q(str(sim_build))} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX={q(str(sim_install))} \
 -DAIMRT_MUJOCO_SIM_INSTALL=ON -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=ON \
 -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=OFF \
 -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build {q(str(sim_build))} --target install --parallel 4
''', [sim_install / 'bin/aimrt_main'], extra_roots=[sim_install])
    runtime = out / 'runtime.yaml'
    # JSON is also YAML; keep the repository runtime configuration untouched.
    write(runtime, {'runner_dist': str(dist), 'sim_install': str(sim_install), 'hope_ws': str(ws)})
    runtime_hashes = {}
    for directory in (dist, install, sim_install):
        for path in directory.rglob('*'):
            if path.is_file():
                runtime_hashes[str(path)] = digest(path)
    write(out / 'runtime-files-sha256.json', runtime_hashes)
    stage('build-versions', env + '''uname -a
cmake --version
c++ --version
/usr/bin/python3 --version
printenv ROS_DISTRO
''')
    library = f'export LD_LIBRARY_PATH={q(str(dist))}:"${{LD_LIBRARY_PATH:-}}"\n'
    original = HERE.with_name('model_19000') / 'policy/exported/actor.onnx'
    ff = GEAR / 'models/model_21800/policy/exported/policy.onnx'
    stage('full-native-parity', env + library + ' '.join(map(q, map(str, [
        dist / 'model19000_native_parity', original, actor, DEFAULT,
        out / 'full-native-parity.json', ff]))), [out / 'full-native-parity.json'])
    stage('native-inspection', env + library + f'''{q(str(dist / 'a3_deploy_onnx_ref_pingpong'))} \
 --inspect-policy --policy-dir {q(str(HERE / 'policy'))} > {q(str(out / 'inspection.yaml'))}
/usr/bin/python3 - {q(str(out / 'inspection.yaml'))} <<'PY'
import sys,yaml
r=yaml.safe_load(open(sys.argv[1]));assert r['recurrent_explicit_state'] is True
assert r['recurrent_advances_during_inspection']==0
assert r['observation_dim']==110 and r['action_dim']==31
PY
''', [out / 'inspection.yaml'])
    scenario = GEAR / 'config/gate3/scenarios/fixed-home-100.yaml'
    base_command = ' '.join(map(q, map(str, ['bash', ROOT / 'run_gate3.sh', '--runtime', runtime,
        '--policy-dir', HERE / 'policy', '--scenario', scenario, '--headless'])))
    def fresh(label):
        # Keep failed partial attempts: every restart gets a new directory.
        return out / (label + '-' + time.strftime('%Y%m%d-%H%M%S'))
    check = fresh('check')
    stage('gate3-check', env + library + base_command + f' --check --output {q(str(check))}', [check / 'run.json'])
    formal = fresh('formal')
    rc = stage('formal-gate3', env + library + 'timeout --signal=INT --kill-after=45s 900s ' + base_command + f' --output {q(str(formal))}',
               [formal / 'run.json'], terminal_result=True)
    # Formal failure is a measured terminal result, never an automatic retry/tuning loop.
    formal_receipt = json.loads((out / 'formal-gate3.receipt.json').read_text())
    report_path = next(Path(p) for p in formal_receipt['outputs'] if p.endswith('/run.json'))
    report = json.loads(report_path.read_text())
    print(json.dumps({'exit_code': rc, 'status': report.get('status'),
                      'certification_pass': report.get('certification_pass'),
                      'evidence': str(report_path.parent)}, indent=2))
    return 0 if rc == 0 and report.get('certification_pass') is True else 2

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as error:
        print(f'NATIVE_GATE3_STOPPED: {error}', file=sys.stderr)
        sys.exit(2)
