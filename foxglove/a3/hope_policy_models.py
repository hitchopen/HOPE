#!/usr/bin/env python3
"""Select preinstalled policy/deploy/Planner bundles; never accept caller paths."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from hope_field_assets import atomic_write

LEGACY_MODELS = {'model14600', 'model11999', 'model12600'}
MODELS = LEGACY_MODELS | {'model17400'}
MOCAP_MODELS = MODELS - {'model14600'}

def inspect(root, model=None):
    catalog = json.loads((root / 'catalog.json').read_text())
    if catalog.get('version') != 1 or not LEGACY_MODELS <= set(catalog['models']) <= MODELS:
        raise ValueError('invalid policy catalog')
    selected = json.loads((root / 'selected.json').read_text())['model'] if (root / 'selected.json').exists() else 'model14600'
    if selected not in MODELS or (model is not None and model not in MODELS):
        raise ValueError('unknown policy model')
    name = selected if model is None else model
    if name not in catalog['models']:
        raise ValueError(f'policy bundle not installed: {name}')
    entry = catalog['models'][name]
    if name == 'model17400' and (entry.get('required_ball_model') != 'ppq004' or entry.get('receive_only') is not True):
        raise ValueError('model17400 requires PPQ004 and receive-only bundle')
    for path, expected in entry['files'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise ValueError(f'policy bundle changed: {path}')
    return {'model': name, 'selected': selected, **{k:v for k,v in entry.items() if k != 'files'}}

def require_stopped():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            name = (proc / 'exe').resolve(strict=True).name
            if name in {'a3_deploy_onnx_ref_pingpong', 'hope_planner_cpp_node'}:
                raise ValueError('stop Runner and Planner before selecting policy')
        except (FileNotFoundError, PermissionError, ProcessLookupError): pass

def handle(root, request):
    op = request.get('op')
    if op == 'read_policy_model': return inspect(root)
    if op == 'verify_policy_model':
        result = inspect(root)
        if result['model'] != request.get('model'): raise ValueError('selected policy differs across endpoints')
        return result
    if op == 'select_policy_model':
        require_stopped()
        result = inspect(root, request.get('model'))
        if request.get('model') not in MODELS: raise ValueError('unknown policy model')
        atomic_write(root / 'selected.json', json.dumps({'model': result['model']}).encode())
        return inspect(root)
    raise ValueError('unknown policy operation')

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--field',choices=['deployment','planner_profile']);a=p.parse_args()
    try:
        result = inspect(a.root) if a.field else handle(a.root,json.load(sys.stdin))
        print(result[a.field] if a.field else json.dumps(result))
    except (OSError,ValueError,KeyError,TypeError) as e:
        print(str(e),file=sys.stderr);return 1
    return 0
if __name__ == '__main__':sys.exit(main())
