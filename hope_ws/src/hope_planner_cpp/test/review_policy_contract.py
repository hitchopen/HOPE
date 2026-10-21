#!/usr/bin/env python3
"""Two replaceable artifacts: metadata/110-D ABI and finite CPU inference only.

Real decoder and observation-builder semantics are exercised by the C++ suite.
This is not achieved paddle tracking or a native Runner --inspect-policy run.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
args = p.parse_args()
paths = ['a3_deploy/a3_deploy_example/models/model_21800/policy/exported/policy.onnx',
         'hope_training/whole_body_tracking/checkpoints/policy.onnx']
expected_terms = ['base_ang_vel', 'joint_pos', 'joint_vel', 'actions', 'projected_gravity',
                  'base_forward_xy', 'base_target_delta_xy', 'racket_target_rel_base',
                  'racket_target_vel_w', 'time_to_strike']
results = []
for relative in paths:
    path = args.repo/relative
    session = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
    metadata = session.get_modelmeta().custom_metadata_map
    assert metadata['observation_names'].split(',') == expected_terms
    assert int(metadata['actor_obs_total_dim']) == 110
    assert metadata['world_frame_contract'] == 'table_p1_to_p2_v1'
    inputs = {i.name: i for i in session.get_inputs()}
    assert inputs['obs'].shape == [1, 110]
    cases = []
    for velocity, lead in [([1.5, -.2, .8], .2), ([2., .1, 1.2], .08), ([1., 0., .5], .4)]:
        obs = np.zeros((1, 110), dtype=np.float32)
        obs[0, 98], obs[0, 99] = -1., 1.
        obs[0, 103:106] = [.58, -.2, .3]
        obs[0, 106:109], obs[0, 109] = velocity, lead
        feeds = {'obs': obs}
        if 'time_step' in inputs: feeds['time_step'] = np.array([[0.]], dtype=np.float32)
        outputs = session.run(None, feeds)
        assert outputs and all(np.isfinite(o).all() for o in outputs)
        assert any(o.shape == (1, 31) for o in outputs)
        cases.append({'lead_s': lead, 'velocity': velocity, 'output_shapes': [list(o.shape) for o in outputs]})
    results.append({'artifact': relative, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                    'recipe': metadata.get('hitter_pure_training_recipe'), 'cases': cases,
                    'contract': {k: metadata.get(k) for k in ['observation_names', 'actor_obs_total_dim',
                                 'world_frame_contract', 'actor_obs_contract']}})
assert results[0]['sha256'] != results[1]['sha256']
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps({'status': 'PASS', 'scope': __doc__, 'results': results}, indent=2)+'\n')
print(json.dumps({'status': 'PASS', 'distinct_artifacts': len(results), 'inference_cases': 6}))
