#!/usr/bin/env python3
"""Verify local Foxglove policy assets without selecting or deploying a policy."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def verify(root, check_onnx=False):
    root = Path(root).resolve()
    catalog = json.loads((root / 'catalog.json').read_text())
    models = catalog['console_models']
    if len(models) != 12 or len(set(models)) != len(models):
        raise ValueError('expected the 12 distinct Console model IDs')
    verified = {}
    for name, record in catalog['files'].items():
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ValueError(f'asset path escapes package: {name}')
        data = path.read_bytes()
        if len(data) != record['bytes'] or hashlib.sha256(data).hexdigest() != record['sha256']:
            raise ValueError(f'asset integrity failure: {name}')
        verified[name] = record['sha256']
    if check_onnx:
        import numpy as np
        import onnx
        import onnxruntime as ort
        import yaml
    seen, referenced, rows = set(), set(), []
    for package in catalog['packages']:
        identity = (package['model'], package['variant'])
        if identity in seen:
            raise ValueError(f'duplicate model/variant: {identity}')
        seen.add(identity)
        assets = package['assets']
        for record in assets.values():
            if verified.get(record['path']) != record['sha256']:
                raise ValueError(f'unbound package asset: {record}')
            referenced.add(record['path'])
        policy, deploy = assets['policy'], assets['deploy']
        result = {'model': identity[0], 'variant': identity[1], 'hashes': 'PASS'}
        if 'serve_csv' in assets:
            lines = (root / assets['serve_csv']['path']).read_text().splitlines()
            metadata = [next(csv.reader([line])) for line in lines
                        if line.startswith('#HOPE_SERVE_SEGMENTS_V1,')]
            if len(metadata) != 1 or metadata[0][3:5] != [policy['sha256'], deploy['sha256']]:
                raise ValueError(f'serve/policy/deploy identity mismatch: {identity}')
            result['serve_pairing'] = 'PASS'
        if 'planner_profile' in assets:
            profile = json.loads((root / assets['planner_profile']['path']).read_text())
            if not isinstance(profile['hope_planner']['ros__parameters'], dict):
                raise ValueError(f'invalid Planner profile: {identity}')
        if check_onnx:
            path = root / policy['path']
            model = onnx.load(path, load_external_data=False)
            if any(t.data_location == onnx.TensorProto.EXTERNAL for t in model.graph.initializer):
                raise ValueError(f'unpackaged external ONNX weights: {identity}')
            onnx.checker.check_model(model)
            config = yaml.safe_load((root / deploy['path']).read_text())
            if not isinstance(config, dict):
                raise ValueError(f'invalid deploy YAML: {identity}')
            claimed = config.get('provenance', {}).get('policy_sha256')
            if claimed is not None and claimed != policy['sha256']:
                raise ValueError(f'deploy provenance refers to another policy: {identity}')
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            session = ort.InferenceSession(str(path), sess_options=options,
                                           providers=['CPUExecutionProvider'])
            feed = {}
            for item in session.get_inputs():
                if item.type != 'tensor(float)':
                    raise ValueError(f'unexpected input type: {identity}: {item.name} {item.type}')
                shape = [d if isinstance(d, int) and d > 0 else 1 for d in item.shape]
                feed[item.name] = np.zeros(shape, dtype=np.float32)
            outputs = session.run(None, feed)
            if not all(np.isfinite(value).all() for value in outputs):
                raise ValueError(f'nonfinite zero-input output: {identity}')
            names = [o.name for o in session.get_outputs()]
            if 'actions' not in names or outputs[names.index('actions')].shape != (1, 31):
                raise ValueError(f'unexpected action ABI: {identity}')
            result.update(onnx='PASS', cpu_zero_input_smoke='PASS',
                          inputs={k:list(v.shape) for k,v in feed.items()})
        rows.append(result)
    if {model for model, _ in seen} != set(models) or referenced != set(verified):
        raise ValueError('Console coverage or asset reference mismatch')
    return {'console_models': len(models), 'packages': len(rows),
            'files': len(verified), 'results': rows,
            'scope': 'Asset integrity, pairing and optional CPU load/smoke only; no deployment or physical qualification.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--onnx', action='store_true', help='also run ONNX checker and CPU smoke; requires onnx, onnxruntime, numpy, PyYAML')
    args = parser.parse_args()
    print(json.dumps(verify(args.root, args.onnx), indent=2))


if __name__ == '__main__':
    main()
