"""CPU-only, reset-aware GRU export. No Isaac/SimulationApp imports.

The installed RSL-RL policy is the independent stateful inference oracle. The
export graph calls the very same normalizer, GRU and MLP, with explicit memory.
Only trusted, locally snapshotted checkpoints may be deserialized by this tool.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import torch
from torch import nn
from tensordict import TensorDict
import yaml
from rsl_rl.modules import ActorCriticRecurrent
from portable_model_config import portable_motion_config


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ConfigLoader(yaml.SafeLoader):
    """Read Isaac's data-only YAML without executing Python YAML constructors."""


def python_tag(loader, suffix, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    return loader.construct_scalar(node)


ConfigLoader.add_multi_constructor('tag:yaml.org,2002:python/', python_tag)


class ExplicitActor(nn.Module):
    def __init__(self, policy, passive):
        super().__init__()
        self.normalizer = policy.actor_obs_normalizer
        self.gru = policy.memory_a.rnn
        self.actor = policy.actor
        mask = torch.ones(31)
        mask[passive] = 0
        self.register_buffer('mask', mask)

    def forward(self, obs, h_in):
        features, h_out = self.gru(self.normalizer(obs).unsqueeze(0), h_in)
        return self.actor(features.squeeze(0)) * self.mask, h_out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--expected-sha256', required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    args = p.parse_args()
    torch.set_num_threads(2)
    digest = sha256(args.checkpoint)
    if digest != args.expected_sha256:
        raise ValueError(f'checkpoint SHA mismatch: {digest}')
    saved = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    params = args.checkpoint.parent / 'params'
    agent = yaml.safe_load((params/'agent.yaml').read_text())
    recipe = yaml.safe_load((params/'task_resolved.yaml').read_text())
    task=recipe['task']
    if (task.get('actor_obs_contract')!='hitter_pure'
            or task.get('build_contract',{}).get('reward_revision')!='actual_complete_hold_v11_stationary_reach_v4'
            or not task['racket']['virtual_ball'] or task['racket']['physical_ball']):
        raise ValueError('Expected the saved PPQ004 StationaryRally virtual-ball recipe')
    env_cfg = yaml.load((params/'env.yaml').read_text(), Loader=ConfigLoader)
    policy_cfg = dict(agent['policy'])
    if policy_cfg.pop('class_name') != 'BoundedStdActorCriticRecurrent':
        raise ValueError('Only the bounded StationaryRally GRU policy is supported')
    assert policy_cfg['rnn_type'] == 'gru' and policy_cfg['rnn_num_layers'] == 1
    assert policy_cfg['rnn_hidden_dim'] == 110
    state = saved['hope_exact_resume_state']
    assert state['schema_version'] == 3
    env = state['environment_resume_state']
    fields = env['managers']['action_manager']['terms']['joint_pos']['fields']
    names = fields['_joint_names']
    ids = fields['_action_joint_ids'].tolist()
    assert ids == list(range(31)), 'This exporter requires full-articulation action order'
    passive = fields['_passive_action_cols'].tolist()
    assert {names[i] for i in passive} == {'head_yaw_joint', 'head_pitch_joint'}
    obs = env['initial_observations']
    observations = TensorDict({k: obs[k][:1].clone() for k in ('policy','critic')}, batch_size=[1])
    # Bounded exploration affects stochastic training only; deterministic inference
    # differs from the installed policy solely by the passive-action mask below.
    for key in ('min_noise_std','max_noise_std','entropy_excluded_action_indices','entropy_action_transform'):
        policy_cfg.pop(key, None)
    policy = ActorCriticRecurrent(observations, {'policy':['policy'],'critic':['critic']}, 31, **policy_cfg)
    policy.load_state_dict(saved['model_state_dict'], strict=True)
    policy.eval()
    actor = ExplicitActor(policy, passive).eval()
    normalizer_keys = [k for k in saved['model_state_dict'] if k.startswith('actor_obs_normalizer.')]
    normalization = 'empirical_embedded' if bool(policy_cfg.get('actor_obs_normalization')) else 'identity'
    assert bool(normalizer_keys) == (normalization != 'identity')
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out/'manifest.json').unlink(missing_ok=True)
    with torch.inference_mode():
        # Real checkpoint observations, starting from ZERO fresh-episode memory;
        # arbitrary nonzero memory is tested separately. No saved training memory.
        samples = obs['policy'][:48].clone()
        samples = torch.cat([samples, samples[:16]], dim=0)
        generator = torch.Generator().manual_seed(43)
        hidden = torch.randn(1,1,110, generator=generator) * .1
        initial = hidden.clone()
        policy.memory_a.reset(hidden_state=hidden.clone())
        actions, memories = [], []
        reset_at = 32
        max_oracle_error = 0.0
        for i, sample in enumerate(samples):
            if i == reset_at:
                hidden = torch.zeros_like(hidden)
                policy.memory_a.reset(hidden_state=hidden.clone())
            action, hidden = actor(sample[None], hidden)
            oracle = policy.act_inference(TensorDict({'policy':sample[None]},batch_size=[1])) * actor.mask
            max_oracle_error = max(max_oracle_error, float((action-oracle).abs().max()))
            torch.testing.assert_close(action, oracle, atol=1e-6, rtol=1e-6)
            torch.testing.assert_close(hidden, policy.memory_a.hidden_state, atol=1e-6, rtol=1e-6)
            actions.append(action.numpy().copy()); memories.append(hidden.numpy().copy())
        np.savez(out/'parity_vectors.npz', obs=samples.numpy(), initial_h=initial.numpy(),
                 reset_at=reset_at, actions=np.stack(actions), hidden=np.stack(memories))
        torch.onnx.export(actor, (samples[:1], torch.zeros(1,1,110)), str(out/'actor.onnx'),
            input_names=['obs','h_in'], output_names=['actions','h_out'],
            opset_version=17, dynamo=False)
    assert 'omni.kit.app' not in sys.modules and 'isaacsim' not in sys.modules
    physical = env['physics_properties']['robot']
    data = physical['data']
    def row(value):
        return (value[0] if value.ndim > 1 else value).tolist()
    action = {label:row(fields[key]) for label,key in [
        ('offset','_offset'),('scale','_scale'),('lower','_safe_lo'),('upper','_safe_hi')]}
    action.update(default_q=row(data['default_joint_pos']), default_qd=row(data['default_joint_vel']),
        passive_columns=passive, contract=fields['_action_contract'],
        delay_min=fields['_qdes_delay_min_steps'], delay_max=fields['_qdes_delay_max_steps'],
        delay_nominal_fraction=fields['_qdes_delay_nominal_fraction'])
    actuator = {k:[0.0]*31 for k in ('kp','kd_total','effort_limit','armature','friction')}
    actuator['kp'] = row(data['default_joint_stiffness'])
    actuator['kd_total'] = row(data['default_joint_damping'])
    for entry in physical['actuators'].values():
        f = entry['fields']
        for label, field in [('effort_limit','effort_limit_sim'),('armature','armature'),('friction','friction')]:
            values = row(f[field])
            for index, value in zip(f['_joint_indices'].tolist(), values):
                actuator[label][index] = value
    task = recipe['task']
    required_hashes = {task['racket']['venue_tuple_bank_path']:task['racket']['venue_tuple_bank_sha256'],
        task['racket']['venue_tuple_bank_receipt_path']:task['racket']['venue_tuple_bank_receipt_sha256']}
    for path, expected in required_hashes.items():
        assert sha256(args.repo_root/'hope_training/whole_body_tracking'/path) == expected
    assert sha256(args.repo_root/task['ball_physics']['path']) == task['ball_physics']['sha256']
    physics=yaml.safe_load((args.repo_root/task['ball_physics']['path']).read_text())
    from mujoco_eval_onnx import _load_success_metric
    geometry=_load_success_metric(args.repo_root).TableGeometry.from_config(physics)
    racket_cfg=env_cfg['commands']['racket_target']
    obs_terms = [(k,v) for k,v in env_cfg['observations']['policy'].items()
                 if isinstance(v,dict) and v.get('func')]
    dims = [3,31,31,31,3,2,2,3,3,1]
    assert [k for k,v in obs_terms] == ['base_ang_vel','joint_pos','joint_vel','actions',
        'projected_gravity','base_forward_xy','base_target_delta_xy','racket_target_rel_base',
        'racket_target_vel_w','time_to_strike']
    layout=[]; cursor=0
    for (name,cfg),dim in zip(obs_terms,dims):
        assert cfg.get('scale') is None and cfg.get('clip') is None
        layout.append({'name':name,'slice':[cursor,cursor+dim],'source':cfg['func']})
        cursor += dim
    manifest = {'schema':'hope_stationary_gru_mujoco_v1','checkpoint':str(args.checkpoint.resolve()),
        'checkpoint_sha256':digest,'checkpoint_iteration':saved['iter'],
        'wandb_run_id':state.get('wandb_run_id'),
        'onnx_sha256':sha256(out/'actor.onnx'),'source_runtime_manifest':state['runtime_manifest'],
        'normalization':normalization,'normalization_keys':normalizer_keys,
        'joint_names':names,'observation_layout':layout,'hidden_shape':[1,1,110],
        'action':action,'actuator':actuator,'root_default':row(data['default_root_state']),
        'physics_dt':env['physics_dt'],'decimation':env['decimation'],'control_dt':env['step_dt'],
        'task':task,'motion_cfg':portable_motion_config(env_cfg['commands']['motion']),
        'visual_table':{'near_x':racket_cfg['vb_table_near_x'],
            'surface_z':racket_cfg['vb_table_surface_z'],
            'length':geometry.length,'width':geometry.width,
            'robot_table_collisions':False},
        'racket_site':{'body':racket_cfg['wrist_body_name'],'offset':racket_cfg['mount_offset'],
            'position_tolerance_m':2e-6},
        'export_oracle_action_max_error':max_oracle_error,'export_launched_isaac':False,
        'training_hidden_state_used':False,'plant':'nominal MJCF with saved nominal implicit actuator gains',
        'limitations':['MuJoCo replaces PhysX, including different contact/friction and drive solver semantics.',
            'Nominal mass/COM/PD cohort; training startup randomization and disturbances are not replayed.',
            'Deterministic target-bank alternation/clock replaces adaptive curriculum, stochastic motion resets, replay buffer and interruptible recovery scheduler.',
            'Perfect-state mocap is used; training mocap noise/delay/dropout/stress is not reproduced.',
            'Only target/reference markers, no physical ball, no contact or legal-landing qualification.',
            'Training reward and reference-body termination managers and drive velocity-limit solver are not recreated.']}
    temporary=out/'manifest.json.tmp'
    temporary.write_text(json.dumps(manifest,indent=2)+'\n')
    temporary.replace(out/'manifest.json')
    shutil.copytree(params,out/'params',dirs_exist_ok=True)
    print(json.dumps({'export':'PASS','checkpoint_sha256':digest,'normalization':normalization,
        'pytorch_stateful_oracle_error':max_oracle_error,'output':str(out)},indent=2))


if __name__ == '__main__':
    main()
