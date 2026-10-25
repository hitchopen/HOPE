"""Performance work must preserve reset semantics, snapshots and metric means."""
import sys
from types import SimpleNamespace as NS
import pytest
import torch
from test_gaussian_action_projection import load_module, UTILS
from test_rally_timing import method

perf=load_module('training_performance_test',UTILS/'training_performance.py')
sys.modules['whole_body_tracking.utils.training_performance']=perf


def fixture():
    n=4
    target=NS(racket_target_pos_w=torch.tensor([[.45,-.5,1.]]*n),base_target_pos_w=torch.zeros(n,2))
    data=lambda: NS(joint_pos=torch.randn(10,31),body_quat_w=torch.nn.functional.normalize(torch.randn(10,3,4),dim=-1))
    m=NS(step_motion=data(),motion=data(),num_envs=n,device='cpu',time_steps=torch.zeros(n,dtype=torch.long),clip_id=torch.tensor([0,1,0,1]),
         cfg=NS(step_reference_mode='target_reach',stand_contact_xyz=[[.4,-.5,1.],[.4,-.05,1.]],step_contact_xyz=[[.4,-.7,.9],[.4,-.1,.9]]),
         _env=NS(scene=NS(env_origins=torch.zeros(n,3)),command_manager=NS(get_term=lambda _:target)))
    fixed=load_module('training_perf_fixed',UTILS/'fixed_station.py');sys.modules['whole_body_tracking.utils.fixed_station']=fixed
    weight=method('commands.py','MotionCommand','step_reference_weight')
    m.step_reference_weight=lambda:weight(m)
    return m,target


def test_reference_cache_invalidates_within_one_tick_after_target_reset_and_restore():
    m,target=fixture();read=method('commands.py','MotionCommand','reference_at')
    a=read(m,'joint_pos');assert read(m,'joint_pos') is a
    for change in (lambda:target.racket_target_pos_w[:,1].sub_(.12),lambda:m.time_steps.add_(1),
                   lambda:m.clip_id.copy_(1-m.clip_id),lambda:target.base_target_pos_w.add_(.1),
                   lambda:m._env.scene.env_origins[:,2].add_(.1)):
        change();b=read(m,'joint_pos');assert b is not a
        w=m.step_reference_weight()[:,None]
        torch.testing.assert_close(b,torch.lerp(m.motion.joint_pos[m.time_steps],m.step_motion.joint_pos[m.time_steps],w),rtol=0,atol=0)
        a=b
    q=read(m,'body_quat_w')
    torch.testing.assert_close(q.norm(dim=-1),torch.ones(4,3))


def test_inference_tensors_and_legacy_drift_style_bypass_cache():
    m,t=fixture()
    with torch.inference_mode():m.time_steps=torch.zeros(4,dtype=torch.long)
    assert perf.reference_cache(m) is None
    m,t=fixture();m.cfg.step_reference_mode='station_error'
    assert perf.reference_cache(m) is None


def test_real_hold_countdown_keeps_reference_cache_live_during_inference():
    from test_rally_timing import motion, advance
    m = motion(count=4)
    m.cfg.step_reference_mode = 'discrete_four_stroke'
    m.cfg.stand_contact_xyz = [[.4,-.5,1.],[.4,-.05,1.]]
    m.cfg.step_contact_xyz = [[.4,-.7,.9],[.4,-.1,.9]]
    m.step_motion = NS()
    target = NS(racket_target_pos_w=torch.zeros(4,3),base_target_pos_w=torch.zeros(4,2),
                _stroke_step=torch.zeros(4,dtype=torch.bool),_stroke_initial_tts=torch.ones(4))
    m._env.scene=NS(env_origins=torch.zeros(4,3))
    m._env.command_manager=NS(get_term=lambda _:target)
    m.time_steps[:]=torch.tensor([10,120,10,120]);m.hold_counter.fill_(3)
    m.rally_wrap_step[:]=torch.tensor([70,180,70,180]);m._resample_command=lambda ids:None
    counter=m.hold_counter
    with torch.inference_mode():
        first=perf.reference_cache(m);first['sentinel']=1
        advance(m)
        assert m.hold_counter is counter and not counter.is_inference()
        assert counter.tolist()==[2]*4
        second=perf.reference_cache(m)
        assert second is not None and 'sentinel' not in second
        assert perf.reference_cache(m) is second


def test_episode_means_are_element_weighted_and_do_not_mutate_inputs():
    original=torch.tensor([1.,3.,9.])
    info=[{'x':original,'scalar':2.,'missing':torch.tensor(5.)},
          {'x':torch.tensor([11.]),'scalar':4.,'later_only':99}]
    out=perf.aggregate_episode_infos(info,'cpu')[0]
    assert set(out)==set(info[0])
    assert out['x'].item()==6 and out['scalar'].item()==3 and out['missing'].item()==5
    assert info[0]['x'] is original and info[0]['scalar']==2.
    assert perf.aggregate_episode_infos([],'cpu')==[]


def test_markov_snapshot_is_independent_without_redundant_clone():
    capture=method('hope_commands.py','RacketTargetCommand','capture_markov_replay_state')
    m=NS(device='cpu',num_envs=4,field=torch.arange(12.).reshape(4,3),_delay_steps=1,_delay_ptr=1,
         _delay_buf_pos=torch.arange(24.).reshape(2,4,3),_delay_buf_vel=torch.randn(2,4,3),
         _delay_buf_sign=torch.randn(2,4),_markov_replay_tensor_fields=lambda:('field',))
    ids=torch.tensor([3,1]);state=capture(m,ids);saved={k:v.clone() for k,v in state.items()}
    m.field.zero_();m._delay_buf_pos.zero_();m._delay_buf_vel.zero_();m._delay_buf_sign.zero_()
    for key in saved:torch.testing.assert_close(state[key],saved[key],rtol=0,atol=0)
    for value in state.values():value.add_(999)
    assert m.field.eq(0).all() and m._delay_buf_pos.eq(0).all()


def test_scalar_batch_preserves_keys_steps_and_local_events():
    local=[];remote=[];outside=[]
    raw=NS(add_scalar=lambda *args:outside.append(args),sentinel=42)
    writer=perf.BatchedScalarWriter(raw,lambda *args:local.append(args),lambda payload,step:remote.append((dict(payload),step)))
    with writer.scalar_batch(17):
        writer.add_scalar('Loss/policy',torch.tensor(2.),17)
        writer.add_scalar('Live/tilt',3.,17)
        writer.add_scalar('Live/tilt',4.,17)
    assert remote==[({'Loss/policy':2.,'Live/tilt':4.},17)]
    assert len(local)==3 and not outside and writer.sentinel==42
    writer.add_scalar('outside',5.,18)
    assert len(outside)==1
    with pytest.raises(ValueError):
        with writer.scalar_batch(19):
            writer.add_scalar('partial',6.,19)
            raise ValueError('interrupt')
    assert remote[-1]==({'partial':6.},19) and writer._pending is None
