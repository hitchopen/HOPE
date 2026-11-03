#!/usr/bin/env python3
"""Reproduce whole-body 12000/13800 research candidates; never deploy hardware."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper
import yaml

BASE_SHA = '8a11ebe5d997a115a9f3a55ea57339c4d47773a2978083e2e6b6aeb346d4d79d'
BASE_DEPLOY_SHA = '64b3c681a5f0ad25020b3f9e3c2ce32771aebfd914c13a1aacb800cc8d590442'
TEACHER_SHA = '6df3556eda0fdf01fb7d7a8f3d5a95f8d57a5df29af6082b3c5ee4a796529a45'
TEACHER_DEPLOY_SHA = '207cf4bf39441374a4af473ec28884cca66bca395cb6faa7389382633f0eccd5'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(wrapped_usb, source12000, output, share13800=.25, method='actions'):
    if not 0 <= share13800 <= 1 or method not in ('actions', 'weights'):
        raise ValueError('finite share in [0,1] and actions/weights method required')
    if output.exists():
        raise ValueError('output must be new')
    if (sha(wrapped_usb/'exported/policy.onnx') != BASE_SHA or
            sha(wrapped_usb/'params/deploy.yaml') != BASE_DEPLOY_SHA or
            sha(source12000/'exported/policy.onnx') != TEACHER_SHA or
            sha(source12000/'params/deploy.yaml') != TEACHER_DEPLOY_SHA):
        raise ValueError('unreviewed source bytes')
    base = onnx.load(wrapped_usb/'exported/policy.onnx')
    config = yaml.safe_load((wrapped_usb/'params/deploy.yaml').read_text())
    teacher = onnx.load(source12000/'exported/policy.onnx')
    weights = {v.name: numpy_helper.to_array(v).copy() for v in teacher.graph.initializer
               if v.name.startswith('actor.')}
    md = {x.key: x.value for x in teacher.metadata_props}
    bm = {x.key: x.value for x in base.metadata_props}
    for key in ['joint_names', 'action_scale', 'default_joint_pos', 'joint_stiffness',
                'joint_damping', 'actor_obs_term_dims', 'qdes_safe_lower_rad',
                'qdes_safe_upper_rad', 'qdes_policy_feedback_contract']:
        if md[key] != bm[key]:
            raise ValueError('incompatible teacher runtime contract: '+key)
    candidate = copy.deepcopy(base)
    mask = np.ones((1, 31), np.float32); mask[:, [11, 16]] = 0
    updates = {}
    for value in candidate.graph.initializer:
        if value.name.startswith('fusion/upper/actor.'):
            updates[value.name] = weights[value.name.split('upper/')[1]]
        elif value.name.startswith('fusion/lower/actor.') and method == 'weights':
            updates[value.name] = share13800*numpy_helper.to_array(value) + (
                1-share13800)*weights[value.name.split('lower/')[1]]
    updates['fusion/lower_mask'] = mask*(share13800 if method == 'actions' else 1)
    updates['fusion/upper_mask'] = mask*((1-share13800) if method == 'actions' else 0)
    for i, value in enumerate(candidate.graph.initializer):
        if value.name in updates:
            candidate.graph.initializer[i].CopyFrom(numpy_helper.from_array(
                updates[value.name].astype(np.float32), value.name))
    name = 'teacher_12000' if share13800 == 0 else '12000_13800_'+method+'_'+str(share13800)
    meta = {x.key: x.value for x in candidate.metadata_props}
    meta.update(fusion_variant=name, fusion_method=method,
                fusion_13800_fraction=str(share13800), fusion_other_teacher='12000',
                fusion_other_teacher_sha256=TEACHER_SHA)
    helper.set_model_props(candidate, meta)
    onnx.checker.check_model(candidate)
    (output/'exported').mkdir(parents=True); (output/'params').mkdir()
    onnx.save(candidate, output/'exported/policy.onnx')
    config['provenance']['policy_sha256'] = sha(output/'exported/policy.onnx')
    config['provenance']['fusion_variant'] = {'method': method, 'alpha13800': share13800,
        'other_teacher': '12000', 'source12000': yaml.safe_load(
            (source12000/'params/deploy.yaml').read_text())['provenance']}
    (output/'params/deploy.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    receipt = {'model': 'modelfusion', 'variant': name,
        'policy_sha256': sha(output/'exported/policy.onnx'),
        'deploy_sha256': sha(output/'params/deploy.yaml'),
        'status': 'UNTRAINED_CANDIDATE_NOT_INSTALLED', 'physical_qualified': False}
    (output/'fusion-candidate.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('wrapped-usb', 'source12000', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--share13800', type=float, default=.25)
    parser.add_argument('--method', choices=['actions', 'weights'], default='actions')
    print(json.dumps(prepare(**vars(parser.parse_args())), indent=2))
