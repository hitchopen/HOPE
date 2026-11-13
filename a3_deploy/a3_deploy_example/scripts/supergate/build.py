#!/usr/bin/env python3
"""Build Supergate against the current checkout and explicit native dependencies."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from common import HERE, REPOSITORY, sha256


def offline_clock_header(source):
    result = source.replace('namespace a3_pingpong {', 'namespace a3_pingpong {\ninline thread_local double simulation_time_s = 0.;', 1)
    for name, epoch in [('PpNowWallSec', '1800000000.0'), ('PpNowSteadySec', '1000.0')]:
        result, count = re.subn(r'inline double '+name+r'\(\) \{.*?\n\}', f'inline double {name}() {{ return {epoch} + simulation_time_s; }}', result, flags=re.S)
        if count != 1:
            raise ValueError(f'cannot identify production clock function {name}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ort-root', required=True, type=Path, help='ONNX Runtime C++ package with include/ and lib/')
    parser.add_argument('--eigen-include', type=Path, default=Path('/usr/include/eigen3'))
    parser.add_argument('--cxx', default='g++')
    args = parser.parse_args()
    import mujoco
    root, ort, output = REPOSITORY, args.ort_root.resolve(), args.output.resolve()
    mj = Path(mujoco.__file__).resolve().parent
    runner = root/'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref'
    planner = HERE/'native/planner'
    module = root/'a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module'
    mj_lib = mj/f'libmujoco.so.{mujoco.__version__}'
    for path in [ort/'include/onnxruntime_cxx_api.h', ort/'lib/libonnxruntime.so', mj/'include/mujoco/mjbatch.h', mj_lib, args.eigen_include/'Eigen/Core']:
        if not path.is_file():
            parser.error(f'missing dependency: {path}')
    output.mkdir(parents=True, exist_ok=True)
    overlay = output/'include/a3_pingpong'
    overlay.mkdir(parents=True, exist_ok=True)
    header = runner/'include/a3_pingpong/pp_planner_input.hpp'
    (overlay/'pp_planner_input.hpp').write_text(offline_clock_header(header.read_text()))
    physics_source = module/'mujoco_sim_module/mujoco_sim_module.cc'
    source = physics_source.read_text()
    begin, end = source.index('void MujocoSimModule::ApplyGate3BallDrag'), source.index('void MujocoSimModule::InitializeDebugCsv')
    methods = source[begin:end].replace('MujocoSimModule::', 'BallPhysicsRuntime::')
    (output/'physics.hpp').write_text((HERE/'native/physics_prefix.hpp').read_text()+methods)
    sources = [HERE/'native/native_batch.cpp', runner/'src/a3_deploy/pp_humanlike_policy.cpp', runner/'src/a3_deploy/pp_hybrid_lower_policy.cpp']
    sources += [planner/'src'/f'{name}.cpp' for name in ['batch_physics_estimator', 'spin_estimator', 'trajectory_predictor', 'racket_target_planner', 'incoming_trajectory', 'adaptive_flight_commit', 'post_net_one_shot', 'schema2_packer']]
    includes = [output, output/'include', HERE/'native', runner/'include', args.eigen_include, ort/'include', mj/'include', planner/'include', module]
    command = [args.cxx, '-std=c++20', '-O2', '-shared', '-fPIC', '-pthread', *[f'-I{p}' for p in includes], *map(str, sources), str(ort/'lib/libonnxruntime.so'), str(mj_lib), f'-Wl,-rpath,{ort / "lib"}', f'-Wl,-rpath,{mj}', '-lyaml-cpp', '-o', str(output/'native_batch.so')]
    subprocess.run(command, check=True)
    try:
        revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    except subprocess.CalledProcessError:
        revision = None
    inputs = [*sources, header, physics_source, HERE/'native/physics_prefix.hpp', *sorted((runner/'include').rglob('*.hpp')), *sorted((planner/'include').rglob('*.hpp')), *sorted((module/'mujoco_sim_module/common').glob('gate3*.h'))]
    receipt = {'repository_commit': revision, 'planner_source': 'bundled Supergate v2 offline dependency; see native/planner/SOURCE_MANIFEST.json', 'mujoco': mujoco.__version__, 'command': command, 'library_sha256': sha256(output/'native_batch.so'), 'source_sha256': {str(p.relative_to(root)): sha256(p) for p in inputs}, 'clock_overlay': 'thread-local simulation clocks; production header unmodified'}
    (output/'build.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(output/'native_batch.so')


if __name__ == '__main__':
    main()
