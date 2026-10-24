"""v16 bounded, question-compatible joint teachers and persistent Home recovery.

The adapter changes only the seven striking-arm reference joints. Root, legs,
feet and the other arm keep the selected complete demonstration. It is a reward
reference, not a changed action ABI, PD target injection or PPO auxiliary loss.
"""
import hashlib
import json
import numpy as np
import torch

from .full_body_hold import GROUPS, rpy, wrap_angle
from .ready_recovery import _log

ADAPTER_FIELDS = ('_stroke_adapter_delta_q', '_stroke_adapter_delta_dq', '_stroke_adapter_valid')


def enabled(motion):
    return bool(getattr(motion.cfg, 'stroke_adapter_path', None))


def load_adapter(command, motion_cfg, bank_arrays, routed_side):
    """Attach an immutable row-aligned sidecar to the in-memory bank only."""
    path = command._resolve_venue_tuple_bank_path(motion_cfg.stroke_adapter_path)
    receipt_path = command._resolve_venue_tuple_bank_path(motion_cfg.stroke_adapter_receipt_path)
    receipt = json.loads(receipt_path.read_text())
    if (hashlib.sha256(path.read_bytes()).hexdigest() != motion_cfg.stroke_adapter_sha256
            or hashlib.sha256(receipt_path.read_bytes()).hexdigest() != motion_cfg.stroke_adapter_receipt_sha256
            or receipt['adapter_sha256'] != motion_cfg.stroke_adapter_sha256
            or receipt['bank_sha256'] != command.cfg.venue_tuple_bank_sha256):
        raise ValueError('stroke adapter does not match the intact question bank')
    with np.load(path, allow_pickle=False) as source:
        fields = {k: source[k].copy() for k in ('delta_q','delta_dq','qualified','family')}
    n = len(routed_side)
    if receipt['rows'] != n or any(v.shape[0] != n for v in fields.values()):
        raise ValueError('stroke adapter row count mismatch')
    from .four_stroke import geometric_step
    side = torch.as_tensor(routed_side, dtype=torch.long)
    step = geometric_step(torch.as_tensor(bank_arrays['target_position_offset']), side,
                          motion_cfg.stand_contact_xyz, motion_cfg.step_contact_xyz)
    if not np.array_equal(fields['family'], (side+2*step).numpy()):
        raise ValueError('stroke adapter family mapping differs from live selector')
    if (fields['delta_q'].shape != (n,7) or fields['delta_dq'].shape != (n,7)
            or not np.isfinite(fields['delta_q']).all() or not np.isfinite(fields['delta_dq']).all()):
        raise ValueError('invalid stroke adapter correction arrays')
    for key in ('delta_q','delta_dq','qualified'):
        bank_arrays['adapter_'+key] = fields[key]
    command._stroke_adapter_delta_q = torch.zeros(command.num_envs,7,device=command.device)
    command._stroke_adapter_delta_dq = torch.zeros_like(command._stroke_adapter_delta_q)
    command._stroke_adapter_valid = torch.zeros(command.num_envs,dtype=torch.bool,device=command.device)
    print(f'[stroke-adapter] {n} original questions; {int(fields["qualified"].sum())} bounded arm paths; unqualified rows retained',flush=True)
    return ('adapter_delta_q','adapter_delta_dq','adapter_qualified')


def envelope(t, before, after=1.0):
    pre=t<0
    duration=torch.where(pre,before,torch.full_like(before,after))
    u=torch.where(pre,(t+before)/before,t/after).clamp(0,1)
    a=torch.where(pre,3*u.square()-2*u.pow(3),1-3*u.square()+2*u.pow(3))
    b=torch.where(pre,u.pow(3)-u.square(),u.pow(3)-2*u.square()+u)*duration
    da=torch.where(pre,6*u-6*u.square(),6*u.square()-6*u)/duration
    db=torch.where(pre,3*u.square()-2*u,3*u.square()-4*u+1)
    inside=(t>=-before)&(t<=after)
    return tuple(x*inside for x in (a,b,da,db))


def joint_reference(motion, command, arm_ids):
    """Current published question only; startup HOLD never sees the hidden row."""
    from .four_stroke import complete_reference
    complete=complete_reference(motion,command)
    q=motion.joint_pos.clone();dq=motion.joint_vel.clone()
    # An urgent packet cannot demand a skipped partial step. Home still owns
    # lower-body recovery; right-arm task freedom is handled separately below.
    hold=motion._explicit_hold_q
    q=torch.where(complete[:,None]|motion.in_hold[:,None],q,hold)
    dq=torch.where(complete[:,None]&~motion.in_hold[:,None],dq,torch.zeros_like(dq))
    valid=command._stroke_adapter_valid&complete&~motion.in_hold
    lead=torch.where(command._stroke_step,
                     q.new_tensor(motion.cfg.step_minimum_lead_s)[motion.clip_id],
                     q.new_tensor(motion.cfg.stand_minimum_lead_s)[motion.clip_id])
    a,b,da,db=envelope(-command.time_to_strike,lead)
    delta=command._stroke_adapter_delta_q*valid[:,None]
    delta_v=command._stroke_adapter_delta_dq*valid[:,None]
    q[:,arm_ids]+=a[:,None]*delta+b[:,None]*delta_v
    dq[:,arm_ids]+=da[:,None]*delta+db[:,None]*delta_v
    return q,dq,complete,valid


def tracking_score(error,scale,margin=0.,tail=.08):
    """Dense peak plus a small linear-tail debt; no direct actor gradients."""
    x=(error.abs()-margin).clamp_min(0)/scale
    cost=torch.sqrt(1+x.square())-1
    # One bad wrist/ankle must not disappear in the group average.
    cost=.5*cost.mean(-1)+.5*cost.amax(-1)
    return 1/(1+cost)-tail*cost


def full_stroke_tracking(env, worst_group_blend=0., foot_blend=0.):
    if not 0 <= worst_group_blend <= 1 or not 0 <= foot_blend <= .30:
        raise ValueError('invalid full-stroke group/foot reward blend')
    motion=env.command_manager.get_term('motion');cmd=env.command_manager.get_term('racket_target')
    if not enabled(motion):
        raise ValueError('full_stroke_tracking requires its qualified adapter')
    robot=cmd.robot
    cache=getattr(env,'_stroke_tracking_ids',None)
    if cache is None:
        cache={group:robot.find_joints(names,preserve_order=True)[0] for group,names in GROUPS.items()}
        env._stroke_tracking_ids=cache
    target,target_vel,complete,valid=joint_reference(motion,cmd,cache['right_arm'])
    from .reference_recovery import static_gate
    gate=1-static_gate(motion,cmd.time_to_strike)
    # On unsupported challenge rows, do not force an incompatible arm during
    # contact. Recovery pressure returns continuously after the strike.
    recovery=((-cmd.time_to_strike-.15)/.70).clamp(0,1)
    arm_factor=torch.where(valid,torch.ones_like(gate),recovery)
    pose=[];velocity=[];rms=[]
    for group,ids in cache.items():
        error=robot.data.joint_pos[:,ids]-target[:,ids]
        rate=robot.data.joint_vel[:,ids]-target_vel[:,ids]
        factor=arm_factor if group=='right_arm' else torch.ones_like(gate)
        pose.append(tracking_score(error,.18 if group=='waist' else .28,.025)*factor)
        velocity.append(tracking_score(rate,1.5,.15,tail=.03)*factor*complete)
        rms.append(error.square().mean(-1).sqrt())
        _log(cmd,'stroke_tracking_'+group+'_rms_rad',rms[-1])
        _log(cmd,'stroke_tracking_'+group+'_velocity_rms',rate.square().mean(-1).sqrt())
        _log(cmd,'imitation_'+group+'_rms_rad',rms[-1])
    actual=rpy(motion.robot_anchor_quat_w)[:,:2]
    desired=rpy(motion.anchor_quat_w)[:,:2]
    desired=torch.where(complete[:,None],desired,torch.zeros_like(desired))
    torso_error=wrap_angle(actual-desired)
    pose_scores = torch.stack(pose,-1)
    pose_mean = pose_scores.mean(-1)
    # Exclude the deliberately unconstrained striking arm on unsupported rows.
    # A zero arm factor must not become a fake worst limb and erase all gradients.
    pose_score = pose_mean
    if worst_group_blend:
        eligible_scores = pose_scores.clone()
        eligible_scores[:, -1] += 1-arm_factor
        pose_score = (1-worst_group_blend)*pose_mean + worst_group_blend*eligible_scores.amin(-1)
    score=(.70*pose_score+.20*torch.stack(velocity,-1).mean(-1)
           +.10*tracking_score(torso_error,.12,.015))*gate
    feet=getattr(env,'_fixed_station_foot_indices',None)
    if foot_blend:
        if feet is None:
            # Reward manager order must not postpone the first foot teacher.
            names=['left_ankle_roll_Link','right_ankle_roll_Link']
            actual_ids, _ = robot.find_bodies(names, preserve_order=True)
            feet=env._fixed_station_foot_indices=(actual_ids,[motion.cfg.body_names.index(n) for n in names])
        from .fixed_station import foot_reference_debt
        home=motion.motion.body_pos_w[0,feet[1]]+env.scene.env_origins[:,None,:]
        reference=torch.where(complete[:,None,None],motion.body_pos_w[:,feet[1]],home)
        foot_error=robot.data.body_pos_w[:,feet[0]]-reference
        debt=foot_reference_debt(foot_error)
        foot_score=1/(1+debt)-.08*debt
        score=(1-foot_blend)*score+foot_blend*foot_score*gate
        _log(cmd,'stroke_foot_tracking_score',foot_score*gate)
    _log(cmd,'stroke_adapter_valid',valid.float())
    _log(cmd,'stroke_tracking_score',score)
    _log(cmd,'imitation_score',score)
    _log(cmd,'imitation_torso_rp_error_rad',torso_error.norm(dim=-1))
    # Curriculum advances only when every complete family actually tracks.
    if feet is not None:
        actual_feet=robot.data.body_pos_w[:,feet[0]];ref_feet=motion.body_pos_w[:,feet[1]]
        error=(actual_feet-ref_feet).norm(dim=-1).amax(-1)
        rms=torch.stack(rms,-1).amax(-1)
        success=(error<.04)&(rms<.30)&(torso_error.norm(dim=-1)<.15)
        selected=valid&(gate>.5)&(cmd.time_to_strike.abs()<.5)
        family=motion.clip_id+2*cmd._stroke_step
        ema=getattr(env,'_stroke_family_quality',None)
        if ema is None:
            ema=env._stroke_family_quality=torch.zeros(4,device=cmd.device)
        for i in range(4):
            mask=selected&(family==i);count=mask.sum()
            ratio=(success&mask).float().sum()/count.clamp_min(1)
            ema[i]=torch.where(count>0,.995*ema[i]+.005*ratio,ema[i])
            _log(cmd,f'stroke_family_{i}_quality',ema[i].expand_as(gate))
            from .motion_quality_curriculum import observe
            observe(cmd,i,mask,success)
    _log(cmd,'stroke_core_probability',torch.ones_like(gate)*core_probability(cmd))
    return score


def core_probability(command):
    cfg=command._motion().cfg
    from .motion_quality_curriculum import enabled
    if enabled(command):
        # Both difficulty knobs use the same admitted evidence clock. Good
        # strokes alone cannot broaden questions while Home/HOLD still fails.
        steps=command._motion_quality_progress_steps
        quality=command._motion_quality_ema[:4]
    else:
        steps=command._env.common_step_counter
        quality=getattr(command._env,'_stroke_family_quality',None)
    elapsed=max(0.,min(1.,steps/max(1,cfg.stroke_core_ramp_steps)))
    if quality is None:
        progress=0.
    else:
        progress=((quality.amin()-.5)/.3).clamp(0,1)*elapsed
    return cfg.stroke_core_start_probability+(cfg.stroke_core_final_probability-cfg.stroke_core_start_probability)*progress


def sample_pool(command,bank,pool,size):
    """All rows retain nonzero support; compatible rows teach the full motion."""
    if not len(pool):
        raise ValueError('empty stroke pool')
    selected=pool[torch.randint(len(pool),(size,),device=command.device)]
    key=(pool.data_ptr(),len(pool))
    cache=getattr(command,'_stroke_compatible_pools',None)
    if cache is None:cache=command._stroke_compatible_pools={}
    if key not in cache:cache[key]=pool[bank['adapter_qualified'][pool]>.5]
    core=cache[key]
    if len(core):
        draw=core[torch.randint(len(core),(size,),device=command.device)]
        selected=torch.where(torch.rand(size,device=command.device)<core_probability(command),draw,selected)
    return selected
