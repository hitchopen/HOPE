#!/usr/bin/env python3
"""Wrap the reviewed USB Fusion actor in the existing Runner export interface.

Preserves the actor bytes/operations and reference outputs. Does not install,
select or qualify hardware. Requires onnx, onnxruntime, numpy and PyYAML.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import compose, helper, utils
import onnxruntime as ort
import yaml


FUSION_SHA = 'c3c5ae5a2261f46ddf74d25d42ce016a258744ff849ee04dffc0c126fd92cf1b'
SOURCE_HASHES = {
    '13800': ('24b021194ae5fd4c7d3ef59d3930c917feadb8f4242df9ada80fe1fb42ad6922',
              '5ff5695ad771cef67f0a2b6d40e5e0d1a6b8775ac16b573008aaf27b0fc36a93'),
    '17400': ('7d81d3250933cba72623de3544dff1abd9ba983aae77f3581f22fdd375dab854',
              '1e2be028e503c19954d9b0414075a6698dc1e9db93adca73e5d9ea6026baea40'),
}
LEGS = [0, 1, 3, 4, 6, 7, 9, 10, 14, 15, 19, 20]
TRAINING_KEYS = {'run_path', 'hitter_pure_checkpoint_sha256',
                 'hitter_pure_resolved_task_sha256', 'hitter_pure_resolved_task_json',
                 'hitter_pure_reward_revision'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def metadata(model):
    return {item.key: item.value for item in model.metadata_props}


def prefix_internal(model, prefix):
    """Namespace internal tensors, preserving the public input/output names."""
    public = {v.name for v in [*model.graph.input, *model.graph.output]}
    result = compose.add_prefix(model, prefix)
    names = {prefix + name: name for name in public}
    for node in result.graph.node:
        for values in (node.input, node.output):
            for i, value in enumerate(values):
                values[i] = names.get(value, value)
    for value in [*result.graph.input, *result.graph.output, *result.graph.value_info,
                  *result.graph.initializer]:
        value.name = names.get(value.name, value.name)
    return result


def prepare(fusion, source13800, source17400, output):
    if output.exists():
        raise ValueError('output must be a new directory')
    if digest(fusion) != FUSION_SHA:
        raise ValueError('unreviewed Fusion actor bytes')
    models, configs = {}, {}
    for name, root in [('13800', source13800), ('17400', source17400)]:
        paths = [root/'exported/policy.onnx', root/'params/deploy.yaml']
        if tuple(digest(p) for p in paths) != SOURCE_HASHES[name]:
            raise ValueError('source policy/deploy identity mismatch: ' + name)
        models[name] = onnx.load(paths[0])
        configs[name] = yaml.safe_load(paths[1].read_text())
    meta = metadata(models['13800'])
    other = metadata(models['17400'])
    if any(meta.get(k) != other.get(k) for k in (meta.keys() | other.keys()) - TRAINING_KEYS):
        raise ValueError('source runtime metadata differs')
    actor = onnx.load(fusion)
    if metadata(actor)['joint_names'] != meta['joint_names']:
        raise ValueError('Fusion joint order differs')
    refs = [v.name for v in models['13800'].graph.output if v.name != 'actions']
    reference = utils.Extractor(models['13800']).extract_model(['time_step'], refs)
    reference = prefix_internal(reference, 'reference/')
    actor = prefix_internal(actor, 'fusion/')
    graph = helper.make_graph(
        [*actor.graph.node, *reference.graph.node], 'fusion_17400_13800_runner',
        [*actor.graph.input, *reference.graph.input],
        [*actor.graph.output, *reference.graph.output],
        initializer=[*actor.graph.initializer, *reference.graph.initializer])
    wrapped = helper.make_model(graph, opset_imports=actor.opset_import,
                                producer_name='HOPE Fusion Runner adapter')
    wrapped.ir_version = actor.ir_version
    # Retain both original training histories as provenance, never attribute the
    # untrained composite to a single checkpoint or a new training run.
    for key in TRAINING_KEYS:
        meta.pop(key, None)
    meta.update(metadata(actor))
    meta.update(run_path='fusion_17400_13800',
                fusion_actor_sha256=FUSION_SHA,
                fusion_source_provenance_json=json.dumps({name: configs[name]['provenance']
                    for name in configs}, sort_keys=True),
                fusion_reference_source='model13800',
                fusion_training_status='untrained_composite')
    helper.set_model_props(wrapped, meta)
    onnx.checker.check_model(wrapped)
    # Numerical checks include nonzero shared previous-action feedback. Both
    # source actors see exactly the same obs, including the applied fused action.
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    def session(value):
        return ort.InferenceSession(value, sess_options=options, providers=['CPUExecutionProvider'])
    fused = session(str(fusion))
    candidate = session(wrapped.SerializeToString())
    sources = {name: session(str(root/'exported/policy.onnx')) for name, root in
               [('13800', source13800), ('17400', source17400)]}
    rng = np.random.default_rng(1740013800)
    errors = {'wrapper_actions': 0., 'source_composition': 0., 'reference_outputs': 0.}
    for i in range(128):
        obs = rng.normal(0., .5, (1, 110)).astype(np.float32)
        feed = {'obs': obs, 'time_step': np.array([[i % 216]], dtype=np.float32)}
        a = sources['13800'].run(None, feed)
        b = sources['17400'].run(['actions'], feed)[0]
        expected = b.copy(); expected[:, LEGS] = a[0][:, LEGS]
        expected[:, [11, 16]] = 0.  # Supplied Fusion keeps the passive head zero.
        raw = fused.run(None, {'obs': obs})[0]
        result = candidate.run(None, feed)
        errors['wrapper_actions'] = max(errors['wrapper_actions'], float(np.max(abs(result[0]-raw))))
        errors['source_composition'] = max(errors['source_composition'], float(np.max(abs(raw-expected))))
        errors['reference_outputs'] = max(errors['reference_outputs'],
            max(float(np.max(abs(x-y))) for x, y in zip(result[1:], a[1:])))
    if max(errors.values()) > 1e-4:
        raise ValueError('Fusion parity failed: ' + str(errors))
    (output/'exported').mkdir(parents=True)
    (output/'params').mkdir()
    onnx.save(wrapped, output/'exported/policy.onnx')
    config = copy.deepcopy(configs['13800'])
    config['provenance'] = {
        'policy_sha256': digest(output/'exported/policy.onnx'),
        'fusion_actor_sha256': FUSION_SHA,
        'source_models': {name: configs[name]['provenance'] for name in configs},
        'export_source': 'untrained Fusion actor with unchanged model13800 reference outputs',
    }
    (output/'params/deploy.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    receipt = {'model': 'modelfusion', 'fusion_actor_sha256': FUSION_SHA,
        'policy_sha256': digest(output/'exported/policy.onnx'),
        'deploy_sha256': digest(output/'params/deploy.yaml'),
        'sources': SOURCE_HASHES, 'leg_indices_policy_order': LEGS,
        'shared_previous_applied_action': True, 'parity_samples': 128,
        'max_abs_errors': errors, 'status': 'PARITY_ONLY_PENDING_DYNAMICS',
        'trained_composite': False, 'physical_qualified': False}
    (output/'fusion-adaptation.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('fusion', 'source13800', 'source17400', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(**vars(args)), indent=2))
