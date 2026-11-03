#!/usr/bin/env python3
"""Smooth whole-body strike/recovery fusion; untrained research, no deployment.

At WAIT (tts=0.82), the actor is about 99% model13800. During approach and
contact, model12000 contributes up to 90%. Both observe the same applied fused
action. This is a phase-conditioned ensemble, not a trained network.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, output, recovery='default'):
    if output.exists():
        raise ValueError('output must be new')
    if recovery not in ('default', 'early'):
        raise ValueError('unknown recovery variant')
    identities = {
        'exported/policy.onnx': 'b2bbec73305d3a18c38358e3402568df183e75caa076f721dfb6119e66569dc6',
        'params/deploy.yaml': '1f539d5ceaf70bbd18d1c5cef2dd899f75c00981b45ec22dd889bf1f7ab0b9c7',
    }
    for relative, expected in identities.items():
        if sha(source/relative) != expected:
            raise ValueError('unreviewed source bytes: '+relative)
    model = onnx.load(source/'exported/policy.onnx')
    config = yaml.safe_load((source/'params/deploy.yaml').read_text())
    mask = np.ones((1, 31), np.float32)
    mask[:, [11, 16]] = 0
    constants = {'phase/after': 0. if recovery == 'early' else .12,
                 'phase/before': .60, 'phase/slope': 20., 'phase/amplitude': .9}
    initializers = [numpy_helper.from_array(np.array([[v]], np.float32), k)
                    for k, v in constants.items()]
    initializers += [numpy_helper.from_array(np.array([109], np.int64), 'phase/index'),
                     numpy_helper.from_array(mask, 'phase/active')]
    specs = [
        ('Add', ['phase/tts', 'phase/after'], 'phase/d1'),
        ('Sub', ['phase/before', 'phase/tts'], 'phase/d2'),
        ('Mul', ['phase/d1', 'phase/slope'], 'phase/z1'),
        ('Mul', ['phase/d2', 'phase/slope'], 'phase/z2'),
        ('Sigmoid', ['phase/z1'], 'phase/g1'),
        ('Sigmoid', ['phase/z2'], 'phase/g2'),
        ('Mul', ['phase/g1', 'phase/g2'], 'phase/window'),
        ('Mul', ['phase/window', 'phase/amplitude'], 'phase/share12000'),
        ('Mul', ['phase/share12000', 'phase/active'], 'phase/mask12000'),
        ('Sub', ['phase/active', 'phase/mask12000'], 'phase/mask13800'),
    ]
    operations = [helper.make_node('Gather', ['obs', 'phase/index'], ['phase/tts'], axis=1)]
    operations += [helper.make_node(op, inputs, [output_name]) for op, inputs, output_name in specs]
    replacements = {'fusion/lower_mask': 'phase/mask13800',
                    'fusion/upper_mask': 'phase/mask12000'}
    for node in model.graph.node:
        for i, name in enumerate(node.input):
            node.input[i] = replacements.get(name, name)
    original = list(model.graph.node)
    del model.graph.node[:]
    model.graph.node.extend(operations+original)
    kept = [v for v in model.graph.initializer if v.name not in replacements]
    del model.graph.initializer[:]
    model.graph.initializer.extend(kept+initializers)
    name = ('phase_early_recovery' if recovery == 'early'
            else 'phase_12000_strike_13800_recover')
    first_factor = 'sigmoid(20*tts)' if recovery == 'early' else 'sigmoid(20*(tts+0.12))'
    formula = '0.9*'+first_factor+'*sigmoid(20*(0.60-tts))'
    meta = {x.key: x.value for x in model.metadata_props}
    meta.update(fusion_variant=name, fusion_method='whole_body_sigmoid_tts_window',
                fusion_12000_weight=formula)
    helper.set_model_props(model, meta)
    onnx.checker.check_model(model)
    (output/'exported').mkdir(parents=True)
    (output/'params').mkdir()
    onnx.save(model, output/'exported/policy.onnx')
    result = copy.deepcopy(config)
    result['provenance']['policy_sha256'] = sha(output/'exported/policy.onnx')
    result['provenance']['fusion_variant'] = {
        'method': meta['fusion_method'], 'formula': formula,
        'parent_policy_sha256': config['provenance']['policy_sha256']}
    (output/'params/deploy.yaml').write_text(yaml.safe_dump(result, sort_keys=False))
    receipt = {'variant': name, 'formula': formula,
               'policy_sha256': sha(output/'exported/policy.onnx'),
               'deploy_sha256': sha(output/'params/deploy.yaml'),
               'status': 'UNTRAINED_CANDIDATE_NOT_INSTALLED', 'physical_qualified': False}
    (output/'fusion-candidate.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--whole-body-policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--recovery', choices=['default', 'early'], default='default')
    args = parser.parse_args()
    print(json.dumps(prepare(args.whole_body_policy, args.output, args.recovery), indent=2))
