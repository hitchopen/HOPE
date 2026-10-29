"""Online target integration, frame handling and economical full-physics reward grid."""
import ast
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]
MDP = ROOT/'source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp'


def _load(name):
    spec=importlib.util.spec_from_file_location(f'test_online_{name}',MDP/(name+'.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    return module


venue=_load('venue_target_tuple')
vb=_load('virtual_ball')


def test_online_inverse_uses_incoming_spin_and_shared_frame():
    raw=venue._profile.load_profile()
    position=torch.tensor([[.65,-.2,1.08],[.65,-.2,1.08]],dtype=torch.float64)
    incoming=torch.tensor([[-2.2,.18,.18],[-2.2,.18,.18]],dtype=torch.float64)
    spin=torch.tensor([[0.,0.,0.],[25.,-40.,35.]],dtype=torch.float64)
    target=torch.tensor([[2.555,0.],[2.555,0.]],dtype=torch.float64)
    solved=venue.full_model_racket_target(position,incoming,spin,target,profile=raw)
    assert solved['model_valid'].all() and solved['model_net_clear'].all()
    assert torch.linalg.norm(solved['racket_velocity_w'][0]-solved['racket_velocity_w'][1])>.01
    assert torch.max(torch.linalg.norm(solved['predicted_landing_xy_w']-target,dim=-1))<=raw['return_model']['position_tolerance_m']
    assert torch.max(torch.abs(solved['return_flight_time_s']-.5))<=raw['return_model']['time_tolerance_s']
    malformed=position.clone();malformed[:,2]=.76
    rejected=venue.full_model_racket_target(malformed,incoming,spin,target,profile=raw)
    assert not rejected['model_valid'].any()
    legacy=venue._profile.load_profile(ROOT.parents[1]/'configs/ball_physics_venue.yaml')
    with pytest.raises(ValueError,match='published bank mode'):
        venue.full_model_racket_target(position,incoming,spin,target,profile=legacy)


def test_correlated_target_translation_does_not_change_local_ball_solve(monkeypatch):
    calls=[]
    def sample(clip,contact_x_w,**kwargs):
        calls.append(contact_x_w.clone())
        position=torch.stack([contact_x_w,torch.full_like(contact_x_w,-.2),torch.full_like(contact_x_w,1.08)],dim=-1)
        incoming=torch.tensor([[-2.2,.18,.18]],dtype=position.dtype).repeat(2,1)
        spin=torch.tensor([[0.,-30.,5.]],dtype=position.dtype).repeat(2,1)
        target=torch.tensor([[2.555,0.]],dtype=position.dtype).repeat(2,1)
        result=venue.full_model_racket_target(position,incoming,spin,target,
                                             table_surface_z=kwargs['table_surface_z'],table_near_x=kwargs['table_near_x'])
        return dict(result,contact_pos_w=position,incoming_velocity_w=incoming,incoming_spin_w=spin,intended_landing_xy_w=target)
    monkeypatch.setitem(sys.modules,'whole_body_tracking.tasks.tracking.mdp.venue_target_tuple',
                        SimpleNamespace(sample_correlated_venue_tuple=sample,
                                        align_normal_to_reference_hemisphere=venue.align_normal_to_reference_hemisphere))
    tree=ast.parse((MDP/'hope_commands.py').read_text())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='RacketTargetCommand')
    method=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='_apply_correlated_venue_tuple')
    namespace={'torch':torch}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<online-frame-test>','exec'),namespace)
    cfg=SimpleNamespace(venue_tuple_final_mix_prob=1.,venue_tuple_max_resample_attempts=1,
                        vb_table_surface_z=.76,vb_table_near_x=.5,venue_tuple_landing_x_range=(2.17,3.04),
                        venue_tuple_landing_y_range=(-.5,.5),base_target_x_range=(-1,1),base_target_y_range=(-1,1),
                        venue_tuple_speed_limit_mps=10.,normal_mode='velocity')
    shape=lambda n:torch.zeros((2,n),dtype=torch.float64)
    state=SimpleNamespace(cfg=cfg,device='cpu',_venue_tuple_mix_mode='recovery_scaled_online_v1',
                          _recovery_coverage_scale=1.,_resample_is_wrap=False,
                          _vel_range_per_clip_t=torch.tensor([[[-10.,10.]]*3]*2,dtype=torch.float64),
                          _ref_normal_per_clip=torch.tensor([[1.,0.,0.]]*2,dtype=torch.float64),
                          _motion=lambda:SimpleNamespace(),
                          _normal_from_target_velocity=lambda v:v/(torch.linalg.norm(v,dim=-1,keepdim=True)+1e-6),
                          _venue_tuple_selected=torch.zeros(2,dtype=torch.bool),
                          metrics={name:torch.zeros(2) for name in ['venue_tuple_mix_effective','venue_tuple_selected',
                              'station_y_step_command','station_y_step_signed_command','station_y_step_class']})
    for name,size in [('base_target_pos_w',2),('racket_target_pos_w',3),('racket_target_vel_w',3),
                      ('racket_target_normal_w',3),('_venue_planner_contact_normal_w',3),('vb_vel_in_w',3),
                      ('vb_spin_in_w',3),('_venue_intended_landing_xy',2),('_venue_outgoing_velocity_seed',3)]:
        setattr(state,name,shape(size))
    origins=torch.tensor([[0.,0.,0.],[20.,-15.,0.]],dtype=torch.float64)
    boxes=torch.tensor([[[.6,.7],[-.21,-.19],[1.,1.1]]]*2,dtype=torch.float64)
    namespace['_apply_correlated_venue_tuple'](state,torch.arange(2),origins,torch.zeros(2,dtype=torch.long),boxes,torch.zeros(2))
    assert state._venue_tuple_selected.all()
    torch.testing.assert_close(calls[0],torch.tensor([.65,.65],dtype=torch.float64))
    torch.testing.assert_close(state.racket_target_pos_w[1]-state.racket_target_pos_w[0],origins[1]-origins[0])
    torch.testing.assert_close(state.racket_target_vel_w[0],state.racket_target_vel_w[1])
    torch.testing.assert_close(state._venue_intended_landing_xy[0],state._venue_intended_landing_xy[1])


def test_ten_ms_reward_grid_is_within_one_mm_of_full_qualification():
    numba=pytest.importorskip('numba')
    plan=numba.njit(venue._full.plan)
    raw=venue._profile.load_profile();params=venue._full.params_from_profile(raw)
    positions=[];outgoing=[];spins=[];expected=[];expected_net=[]
    for lateral in (-1.2,-.75,-.3):
        for spin in ([0.,0.,0.],[20.,-35.,15.],[-15.,30.,-25.]):
            position=np.array([.15,lateral,.32]);incoming=np.array([-3.,.15,-.2])
            result=plan(position,incoming,np.array(spin),np.array([2.1,-.75]),.5,params)
            assert result[6] and result[7]
            positions.append(position);outgoing.append(result[2]);spins.append(result[3]);expected.append(result[4][:2])
            expected_net.append(venue._full.first_bounce(position,result[2],result[3],params)[3][2])
    prm=vb.load_venue_params()
    tensors=[torch.tensor(np.asarray(array),dtype=torch.float64) for array in (positions,outgoing,spins)]
    coarse=vb.coarse_landing(*tensors,prm,surface_z=0.,net_x=params[venue._full.NET_X],h=.01,n_steps=200)
    assert coarse['land_valid'].all() and coarse['net_valid'].all()
    error=torch.linalg.norm(coarse['land_xy']-torch.tensor(np.asarray(expected)),dim=-1)
    assert float(error.max())<.001
    assert float(torch.max(torch.abs(coarse['net_z']-torch.tensor(expected_net))))<.001
