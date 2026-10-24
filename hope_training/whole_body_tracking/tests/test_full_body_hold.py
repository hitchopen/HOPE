"""Full-body imitation cannot hide a bad limb, force PD offsets away or block a ball."""
import math
import sys
import types
from types import SimpleNamespace as NS
import pytest
import torch
from test_gaussian_action_projection import load_module, UTILS
from test_gaussian_action_projection import policy_modules

package = types.ModuleType("full_hold_test")
package.__path__ = [str(UTILS)]
sys.modules[package.__name__] = package
full = load_module("full_hold_test.full_body_hold", UTILS / "full_body_hold.py")
smooth = load_module("full_hold_test.settled_policy_refinement", UTILS / "settled_policy_refinement.py")


def quaternion(rpy):
    r, p, y = rpy/2
    cr,sr,cp,sp,cy,sy = r.cos(),r.sin(),p.cos(),p.sin(),y.cos(),y.sin()
    return torch.stack((cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                        cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy))


def environment(n=4):
    pose=full.load_reference();m=pose['metrics']
    # Deliberately shuffled articulation names, including passive head slots.
    names=list(reversed(pose['joint_order']))
    q=full.reference_joints(names,torch.zeros(1)).repeat(n,1)
    pos=torch.zeros(n,5,3);pos[:,1,1]=m['foot_center_width_cm']/200;pos[:,2,1]=-pos[:,1,1]
    pos[:,3,2]=m['left_wrist_height_cm']/100;pos[:,4,2]=m['right_wrist_height_cm']/100
    quats=torch.zeros(n,5,4);quats[:,:,0]=1
    for i,ref in enumerate([m['torso_rpy_deg'],*m['foot_rpy_deg']]):
        quats[:,i]=quaternion(torch.tensor(ref)*math.pi/180)
    data=NS(joint_pos=q,joint_vel=torch.zeros_like(q),body_pos_w=pos,body_quat_w=quats,
            root_pos_w=torch.tensor([[0.,0.,m['pelvis_height_cm']/100]]).repeat(n,1),
            root_lin_vel_w=torch.zeros(n,3),applied_torque=torch.ones_like(q)*20)
    robot=NS(data=data,find_joints=lambda wanted,**kw:([names.index(x) for x in wanted],wanted))
    cmd=NS(robot=robot,metrics={},time_to_strike=torch.ones(n),base_target_pos_w=torch.zeros(n,2),
           base_pos_w=data.root_pos_w,_resolve_body_index=lambda x:full.BODY_NAMES.index(x))
    motion=NS(in_hold=torch.ones(n,dtype=torch.bool))
    action=NS(_joint_names=names,_qdes_executed=q+.4,_asset=robot,
              _action_contract='v11_affine_safe_qdes_v1',_offset=torch.zeros_like(q),
              _scale=torch.ones_like(q),_safe_lo=torch.full_like(q,-3.),_safe_hi=torch.full_like(q,3.))
    action._decoder_parameter_rows=lambda value,ids,name:value[ids]
    action._select_action_joints=lambda value:value
    sensor=NS(data=NS(net_forces_w=torch.ones(n,2,3)*100),find_bodies=lambda *a,**kw:([0,1],[]))
    terms={'motion':motion,'racket_target':cmd}
    env=NS(common_step_counter=576000,command_manager=NS(get_term=terms.__getitem__),
           action_manager=NS(get_term=lambda _:action),episode_length_buf=torch.arange(n),
           scene=NS(env_origins=torch.zeros(n,3),sensors={'contact_forces':sensor}))
    return env,cmd,motion,action


def test_reference_pose_pd_bias_head_and_metrics_do_not_change_physical_goal():
    env,cmd,_,action=environment()
    torch.testing.assert_close(full.full_body_hold(env),torch.ones(4))
    action._qdes_executed+=.5;cmd.robot.data.applied_torque*=4
    for name in ['head_pitch_joint','head_yaw_joint']:
        cmd.robot.data.joint_pos[:,action._joint_names.index(name)]+=1
    torch.testing.assert_close(full.full_body_hold(env),torch.ones(4))
    before=cmd.robot.data.joint_pos.clone()
    for value in cmd.metrics.values():value.zero_()
    torch.testing.assert_close(before,cmd.robot.data.joint_pos)


@pytest.mark.parametrize('joint',['left_hip_yaw_joint','right_ankle_roll_joint','waist_pitch_joint',
                                 'left_shoulder_pitch_joint','right_elbow_joint','right_wrist_pitch_joint'])
def test_each_limb_and_small_waist_error_has_a_signal(joint):
    env,cmd,_,action=environment()
    cmd.robot.data.joint_pos[:,action._joint_names.index(joint)]+=math.radians(10)
    value=full.full_body_hold(env)
    assert (value<.99).all()
    assert cmd.metrics['full_body_hold_pose_pass_fraction'].eq(0).all()


def test_pose_cannot_escape_by_retreating_or_unloading_a_foot():
    env,cmd,_,action=environment()
    cmd.robot.data.joint_pos[:,action._joint_names.index('left_knee_joint')]+=.4
    near=full.full_body_hold(env)
    cmd.base_target_pos_w[:,0]=.3
    assert (full.full_body_hold(env)<near).all()
    env.scene.sensors['contact_forces'].data.net_forces_w.zero_()
    assert (full.full_body_hold(env)<near).all()


def test_translation_invariance_and_geometry_catches_whole_body_rotation_and_hands():
    env,cmd,_,_=environment()
    before=full.full_body_hold(env)
    offset=torch.tensor([10.,-4.,2.])
    env.scene.env_origins+=offset
    cmd.robot.data.root_pos_w+=offset;cmd.robot.data.body_pos_w+=offset
    cmd.base_target_pos_w+=offset[:2]
    torch.testing.assert_close(full.full_body_hold(env),before)
    cmd.robot.data.body_pos_w[:,3:5,2]-=.15
    assert (full.full_body_hold(env)<before-.1).all()
    cmd.robot.data.body_quat_w[:,1]=quaternion(torch.tensor([0.,0.,.4]))
    full.full_body_hold(env)
    assert cmd.metrics['full_body_foot_angle_error_max_rad'].min()>.39


def test_no_pose_or_quiet_objective_during_strike_and_incoming_ball_preempts():
    env,cmd,motion,_=environment()
    motion.in_hold.zero_();cmd.time_to_strike[:]=torch.tensor([.85,0.,-.5,-.8])
    torch.testing.assert_close(full.full_body_hold(env),torch.tensor([0.,0.,0.,1.]))
    cmd.robot.data.joint_vel.fill_(.3)
    assert full.full_body_quiet(env)[:3].eq(0).all()
    motion.in_hold.fill_(True);assert full.full_body_hold(env).eq(1).all()
    assert full.full_body_quiet(env).gt(0).all()
    env.scene.sensors['contact_forces'].data.net_forces_w.zero_()
    assert full.full_body_quiet(env).eq(0).all()
    motion.in_hold.zero_();cmd.time_to_strike.fill_(.35)
    assert full.full_body_hold(env).eq(0).all()


def test_curriculum_preserves_upper_only_before_7000_and_replaces_by_9000():
    env,cmd,_,action=environment()
    cmd.robot.data.joint_pos[:,action._joint_names.index('left_knee_joint')]+=.3
    new=full.full_body_hold(env)
    env.common_step_counter=448000
    torch.testing.assert_close(full.full_body_hold(env),torch.ones(4))
    env.common_step_counter=512000
    torch.testing.assert_close(full.full_body_hold(env),(1+new)/2)


def test_quieting_does_not_brake_recovery_and_local_chatter_not_diluted():
    env,cmd,_,action=environment()
    ctx=smooth.SettledPolicyRefinement(env,NS(num_transitions_per_env=4,num_envs=4,device='cpu'),
        start_steps=0,ramp_steps=1,include_upper=True,ready_only=True,worst_joint_blend=.5)
    cmd.robot.data.joint_pos[1,action._joint_names.index('right_elbow_joint')]+=.4
    cmd.base_target_pos_w[2,0]=.3;cmd.robot.data.root_lin_vel_w[3,0]=.3
    ctx.capture(0)
    torch.testing.assert_close(ctx.gate[0],torch.tensor([1.,0.,0.,0.]))
    current=torch.zeros(2,31,requires_grad=True);previous=torch.zeros_like(current)
    previous[:,3]=.006
    args=(current,previous,torch.zeros(2),torch.ones(2),torch.ones(2),list(range(29)),14)
    old,_,_=smooth.refinement_losses(*args)
    new,tracking,_=smooth.refinement_losses(*args,worst_joint_blend=.5)
    assert new>10*old
    (new+tracking).backward();assert current.grad[:,3].abs().sum()>0
    assert current.grad[:,29:].eq(0).all()


def test_v8_config_preserves_plant_and_interruptible_clock():
    from hydra import compose,initialize_config_dir
    from omegaconf import OmegaConf
    with initialize_config_dir(version_base=None,config_dir=str(UTILS.parents[3]/'cfg')):
        old=compose(config_name='train',overrides=['task=HOPEPingPongReadyRecovery','algo=ppo_ready'])
        new=compose(config_name='train',overrides=['task=HOPEPingPongFullBodyHold','algo=ppo_full_body'])
    for key in ('env','sim','domain_rand','action','racket','motion','actor_obs_contract'):
        assert OmegaConf.to_container(old.task,resolve=True)[key]==OmegaConf.to_container(new.task,resolve=True)[key]
    assert new.task.rewards.full_body_hold_start_steps==448000
    assert new.task.rewards.full_body_hold_ramp_steps==128000
    assert new.task.rewards.actual_joint_limit_weight==-1
    assert new.task.rewards.actual_task_pose_weight==new.task.rewards.actual_ready_legs_weight==0
    assert new.algo.algorithm.settled_aux_tracking_coef==0
    assert new.algo.algorithm.settled_aux_gradient_ratio_max==.05


def test_wandb_keeps_full_pose_rewards_and_phase_metrics():
    logging=load_module('full_body_wandb_test',UTILS/'wandb_logging.py')
    for tag in ('Episode_Reward/full_body_hold','Episode_Reward/full_body_quiet',
                'Live/racket_target/full_body_hold_pose_pass_fraction',
                'Live/racket_target/full_body_hold_right_arm_error_rms_rad',
                'Live/racket_target/full_body_recover_left_leg_error_rms_rad'):
        assert logging.wandb_metric_allowed(tag,'hitter_pingpong_v1')
    env,cmd,motion,_=environment()
    full.full_body_hold(env)
    stats=full.phase_summaries(cmd.metrics)
    assert math.isnan(stats['full_body_recover_right_arm_error_rms_rad'])
    assert stats['full_body_hold_right_arm_error_rms_rad']==0
    assert 'full_body_strike_right_arm_error_rms_rad' not in stats


def test_real_ppo_update_v8_worst_joint_gradient_stays_capped(policy_modules):
    from test_settled_policy_refinement import build_algorithm
    obs,policy,alg=build_algorithm(policy_modules)
    env,_,_,_=environment()
    alg.settled_aux_include_upper=True
    alg.settled_aux_worst_joint_blend=.5
    alg.settled_aux_ready_only=True
    alg.bind_settled_refinement(env)
    alg.settled_aux_smooth_coef=100
    with torch.no_grad():
        for _ in range(4):
            action=alg.act(obs)
            obs=obs.clone();obs['policy']=torch.randn(4,6)
            alg.process_env_step(obs,-action.square().mean(-1),torch.zeros(4,dtype=torch.bool),{})
        alg.compute_returns(obs)
    stats=alg.update()
    assert all(math.isfinite(x) for x in stats.values())
    assert 0<stats['settled_mean_gradient_ratio']<=.0500001
    assert stats['settled_mean_gradient_multiplier']<1
    assert stats['settled_mean_smooth']>0
