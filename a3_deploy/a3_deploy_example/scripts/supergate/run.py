#!/usr/bin/env python3
"""Evaluate supplied native policies on six frozen Supergate shards, offline."""
import argparse
import ctypes
import json
from pathlib import Path
import sys
import time
from common import HERE, REPOSITORY, load_dataset, sha256, summarize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--model-xml', type=Path, required=True)
    parser.add_argument('--ready-state', type=Path, required=True, help='NPZ from validate_serve_segments_mujoco or adapt_fusion_serve')
    parser.add_argument('--policy', action='append', required=True, metavar='LABEL=DIRECTORY', help='repeat to compare policies on identical shards')
    parser.add_argument('--dataset', type=Path, default=HERE/'data/main')
    parser.add_argument('--config-dir', type=Path, default=HERE/'config')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', type=int, help='per-shard prefix for a smoke test; omit for the complete set')
    parser.add_argument('--threads', type=int, default=6)
    parser.add_argument('--raw-traces', action='store_true', help='opt in to per-step ball and posture CSVs')
    args = parser.parse_args()
    import numpy as np
    import mujoco
    sys.path.insert(0, str(HERE.parent))
    from validate_spin001_serve_mujoco import _compile_model
    from runner_sim_profile import apply_hope_contact_profile
    if args.threads < 1:
        parser.error('--threads must be positive')
    groups = []
    for item in args.policy:
        if '=' not in item:
            parser.error('--policy must be LABEL=DIRECTORY')
        label, directory = item.split('=', 1)
        path = Path(directory).resolve()
        if not label or any(g['label'] == label for g in groups) or '\n' in str(path):
            parser.error('policy labels must be nonempty and unique; paths cannot contain newlines')
        for name in ['exported/policy.onnx', 'params/deploy.yaml']:
            if not (path/name).is_file():
                parser.error(f'missing policy input: {path/name}')
        groups.append({'label': label, 'path': str(path), 'policy_sha256': sha256(path/'exported/policy.onnx'), 'deploy_sha256': sha256(path/'params/deploy.yaml')})
    cases = load_dataset(args.dataset, args.cases)
    model, bindings = _compile_model(args.model_xml.resolve())
    apply_hope_contact_profile(model)
    if model.nuserdata < 9*7:
        parser.error('XML must have the Gate3 nine-slot flight pool and at least 63 userdata entries')
    for kind, names in [(mujoco.mjtObj.mjOBJ_JOINT, ['gate3_ball_free_joint']+[f'gate3_ball_free_joint_return{i}' for i in range(1,9)]), (mujoco.mjtObj.mjOBJ_GEOM, ['right_racket_collision','gate3_table_collision','gate3_net_collision']), (mujoco.mjtObj.mjOBJ_BODY, ['pelvis_link','torso_Link']), (mujoco.mjtObj.mjOBJ_SITE, ['right_racket'])]:
        for name in names:
            if mujoco.mj_name2id(model, kind, name) < 0:
                parser.error(f'XML missing required Gate3 object: {name}')
    bindings = np.ascontiguousarray(bindings, dtype=np.int32)
    with np.load(args.ready_state, allow_pickle=False) as ref:
        q = np.ascontiguousarray(ref['qpos'][-1], dtype=np.float64)
        v = np.ascontiguousarray(ref['qvel'][-1], dtype=np.float64)
        command = np.ascontiguousarray(ref['command'][-1], dtype=np.float64)
        if not np.array_equal(bindings[:,0], ref['qa']):
            parser.error('ready-state SDK joint bindings do not match the XML')
    if q.shape != (model.nq,) or v.shape != (model.nv,) or command.shape != (5,31) or not all(np.isfinite(x).all() for x in (q,v,command)):
        parser.error('ready-state must contain finite model-sized qpos/qvel and a (5,31) command')
    q[:2] = [-.5, -.7625]
    config = args.config_dir.resolve()
    configs = [config/name for name in ['return_physics.yaml','ball_physics_ppq004_20261020.yaml','model21800_flight_packetizer.yaml','model21800_hardware.yaml']]
    for path in configs:
        if not path.is_file():
            parser.error(f'missing protocol config: {path}')
    library = args.library.resolve()
    build = json.loads(library.with_name('build.json').read_text())
    if build['library_sha256'] != sha256(library) or build['mujoco'] != mujoco.__version__:
        parser.error('native library/build receipt or MuJoCo version mismatch; rebuild')
    lib = ctypes.CDLL(str(library))
    floating = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    integer = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
    lib.batch_run.argtypes = [ctypes.c_void_p,floating,floating,floating,integer,floating,ctypes.c_int,ctypes.c_int,ctypes.c_int]+[ctypes.c_char_p]*7+[ctypes.c_int]
    lib.batch_run.restype = ctypes.c_int
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    start = time.perf_counter()
    policies = '\n'.join(g['path'] for g in groups)
    rc = lib.batch_run(model._address,q,v,command,bindings,cases,cases.shape[1],6*len(groups),args.threads,policies.encode(),groups[0]['path'].encode(),str(output).encode(),*[str(p).encode() for p in configs],int(args.raw_traces))
    receipt = {'backend':'MuJoCo internal batch; production native policy, pinned Supergate v2 offline Planner, production ball physics', 'mujoco':mujoco.__version__, 'policy_groups':groups, 'cases_per_environment':cases.shape[1], 'environments':6*len(groups), 'threads':args.threads, 'return_code':rc, 'elapsed_s':time.perf_counter()-start, 'dataset_manifest_sha256':sha256(args.dataset/'SHA256.json'), 'model_xml_sha256':sha256(args.model_xml), 'ready_state_sha256':sha256(args.ready_state), 'library_sha256':sha256(library), 'build_receipt_sha256':sha256(library.with_name('build.json')), 'config_sha256':{p.name:sha256(p) for p in configs}, 'raw_traces':args.raw_traces, 'protocol':'nominal HOME (-0.5,-0.7625), 20 s WAIT warmup, 1 kHz physics, 50 Hz policy, 250 Hz observations, fixed 8 ms delivery; continuous per shard', 'scope':'Offline core simulation only; excludes outer Runner, DDS, HAL, managed lifecycle and hardware qualification.'}
    (output/'run.json').write_text(json.dumps(receipt,indent=2)+'\n')
    if rc:
        raise RuntimeError(f'native evaluation failed ({rc}); inspect env-*-error.txt')
    summary = summarize(output,[g['label'] for g in groups],cases.shape[1])
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
