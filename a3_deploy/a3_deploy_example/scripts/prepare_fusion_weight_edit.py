#!/usr/bin/env python3
"""Reproduce a bounded, interpretable edit inside model13800's single MLP.

No installation or hardware access. Inherited checkpoint metadata describes the
source training run; provenance.weight_edit identifies the subsequent edit.
The offset is internal to the actor: Planner targets and evaluation targets stay
unchanged. This is experimental compensation, not a qualified new capability.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper
import onnxruntime as ort
import yaml

POLICY_SHA = '24b021194ae5fd4c7d3ef59d3930c917feadb8f4242df9ada80fe1fb42ad6922'
DEPLOY_SHA = '5ff5695ad771cef67f0a2b6d40e5e0d1a6b8775ac16b573008aaf27b0fc36a93'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, output, method, dx, gain, position_delta=None, velocity_delta=None):
    policy, deploy = source/'exported/policy.onnx', source/'params/deploy.yaml'
    if (digest(policy), digest(deploy)) != (POLICY_SHA, DEPLOY_SHA):
        raise ValueError('model13800 source policy/deploy hash mismatch')
    if output.exists():
        raise ValueError('output must be a new directory')
    if not np.isfinite(dx) or not 0 < dx <= .06:
        raise ValueError('dx must be finite and within (0, 0.06] m')
    if not np.isfinite(gain) or not 1 <= gain <= 2:
        raise ValueError('heading gain must be finite and within [1, 2]')
    joint_delta = None
    if method == 'joint':
        if position_delta is None or velocity_delta is None:
            raise ValueError('joint edit requires both position and velocity XYZ deltas')
        joint_delta = np.r_[position_delta, velocity_delta].astype(np.float32)
        if (joint_delta.shape != (6,) or not np.isfinite(joint_delta).all()
                or np.max(abs(joint_delta[:3])) > .08 or np.max(abs(joint_delta[3:])) > .5):
            raise ValueError('joint XYZ deltas must be finite, position <= 0.08 m and velocity <= 0.5 m/s')
    elif position_delta is not None or velocity_delta is not None:
        raise ValueError('XYZ deltas require --method joint')
    names = {'constant': '13800_dx'+str(dx), 'clock': '13800_clock_dx'+str(dx),
             'forward': '13800_forward'+str(dx),
             'heading': '13800_centered_heading'+str(gain), 'joint': '13800_joint_pvn'}
    name = names[method]
    model = onnx.load(policy)
    arrays = {v.name: numpy_helper.to_array(v).copy() for v in model.graph.initializer}
    original = {k: v.copy() for k, v in arrays.items()}
    weight, bias = arrays['actor.0.weight'], arrays['actor.0.bias']
    if weight.shape != (512, 110) or arrays['actor.6.weight'].shape != (31, 128):
        raise ValueError('unexpected actor shape')
    # 99:101 = world-frame base forward; 103:106 = world-frame target
    # relative to base; 109 = seconds to strike, WAIT uses 0.82.
    if method == 'joint':
        direction = weight[:, 103:109] @ joint_delta
        bias += direction
        weight[:, 109] -= direction/.82
    elif method != 'forward':
        bias += weight[:, 103]*dx
    if method == 'clock':
        weight[:, 109] -= (dx/.82)*weight[:, 103]
    elif method == 'heading':
        bias += (1-gain)*weight[:, 99]
        weight[:, 99:101] *= gain
    elif method == 'forward':
        weight[:, 99] += dx*weight[:, 103]
        weight[:, 100] += dx*weight[:, 104]
    for value in model.graph.initializer:
        value.CopyFrom(numpy_helper.from_array(arrays[value.name], value.name))
    metadata = {v.key: v.value for v in model.metadata_props}
    metadata.update(weight_edit=name, weight_edit_status='experimental_not_qualified')
    helper.set_model_props(model, metadata)
    onnx.checker.check_model(model)
    (output/'exported').mkdir(parents=True)
    (output/'params').mkdir()
    onnx.save(model, output/'exported/policy.onnx')
    config = yaml.safe_load(deploy.read_text())
    edit = dict(name=name, parent_sha256=POLICY_SHA,
                dx=0. if method == 'forward' else dx, dt=0., heading=1., waist=1., velocity=1.)
    if method == 'clock':
        edit.update(method='clock_conditioned_input_direction', wait_tts=.82)
    elif method == 'heading':
        edit['centered_heading_gain'] = gain
    elif method == 'forward':
        edit.update(method='base_heading_aligned_target_direction', forward_m=dx)
    elif method == 'joint':
        edit = dict(name=name, parent_sha256=POLICY_SHA,
                    method='clock_conditioned_position_velocity_direction', wait_tts=.82,
                    position_delta_xyz_m=list(map(float, position_delta)),
                    velocity_delta_xyz_mps=list(map(float, velocity_delta)),
                    normal_supervision='Evaluate actual face plane jointly; current Planner n=unit(v), no independent actor normal input')
    config['provenance']['policy_sha256'] = digest(output/'exported/policy.onnx')
    config['provenance']['weight_edit'] = edit
    (output/'params/deploy.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    base = ort.InferenceSession(str(policy), options, providers=['CPUExecutionProvider'])
    changed = ort.InferenceSession(str(output/'exported/policy.onnx'), options,
                                   providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(20261104)
    action_error = reference_error = wait_error = 0.
    for i in range(128):
        obs = rng.normal(0, .15, (1, 110)).astype(np.float32)
        yaw = rng.uniform(-1, 1)
        obs[0, 99:101] = [np.cos(yaw), np.sin(yaw)]
        obs[0, 109] = .82 if i % 2 == 0 else rng.uniform(-.2, .6)
        transformed = obs.copy()
        if method == 'joint':
            transformed[0, 103:109] += joint_delta*(1-float(obs[0, 109])/.82)
        elif method == 'clock':
            transformed[0, 103] += dx*(1-float(obs[0, 109])/.82)
        elif method == 'forward':
            transformed[0, 103:105] += dx*obs[0, 99:101]
        else:
            transformed[0, 103] += dx
            if method == 'heading':
                transformed[0, 99:101] = [1., 0.] + gain*(obs[0, 99:101]-[1., 0.])
        time = np.array([[float(i)]], np.float32)
        expected = base.run(None, {'obs': transformed, 'time_step': time})
        actual = changed.run(None, {'obs': obs, 'time_step': time})
        action_error = max(action_error, float(np.max(abs(expected[0]-actual[0]))))
        reference_error = max(reference_error, max(float(np.max(abs(a-b)))
                              for a, b in zip(expected[1:], actual[1:])))
        if method in ('clock', 'joint') and i % 2 == 0:
            old = base.run(['actions'], {'obs': obs, 'time_step': time})[0]
            wait_error = max(wait_error, float(np.max(abs(old-actual[0]))))
    if action_error > 1e-5 or reference_error != 0 or wait_error > 1e-5:
        raise ValueError('numerical equivalence check failed')
    receipt = dict(parameters=edit, policy_sha256=digest(output/'exported/policy.onnx'),
                   deploy_sha256=digest(output/'params/deploy.yaml'), samples=128,
                   action_equivalence_max_abs=action_error,
                   reference_max_abs=reference_error, wait_neutral_max_abs=wait_error,
                   changed_values={k: int(np.count_nonzero(v != original[k]))
                                   for k, v in arrays.items() if np.any(v != original[k])},
                   formal_gate3_pass=False, installed=False)
    (output/'weight-edit-receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source13800', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--method', choices=['constant', 'clock', 'heading', 'forward', 'joint'],
                        default='clock')
    parser.add_argument('--dx', type=float, default=.045)
    parser.add_argument('--gain', type=float, default=1.3)
    parser.add_argument('--position-delta', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--velocity-delta', type=float, nargs=3, metavar=('VX', 'VY', 'VZ'))
    args = parser.parse_args()
    print(json.dumps(prepare(args.source13800, args.output, args.method, args.dx, args.gain,
                             args.position_delta, args.velocity_delta), indent=2))


if __name__ == '__main__':
    main()
