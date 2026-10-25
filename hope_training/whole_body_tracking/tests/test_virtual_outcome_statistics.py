"""Batched host transfers must preserve contact/landing denominators and EMA."""
import ast
from collections import defaultdict
import math
import sys
from types import SimpleNamespace as NS, ModuleType
import pytest
import torch
from test_virtual_ball_cuda_graph import vb, PATH


def method(path=PATH.with_name('hope_commands.py')):
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_vb_evaluate')
    scope = {'torch': torch, 'math': math}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
    return scope['_vb_evaluate']


def fixture(device, unconditional):
    n = 128
    gen = torch.Generator(device=device).manual_seed(34)
    pos = torch.rand(n, 3, device=device, generator=gen)
    pos[:, 0] = .58
    pos[:, 1] = pos[:, 1] * .8 - .5
    pos[:, 2] = pos[:, 2] * .6 + .8
    velocity = torch.rand(n, 3, device=device, generator=gen)
    velocity[:, 0] += 1.2
    normal = velocity / velocity.norm(dim=-1, keepdim=True)
    incoming = torch.zeros_like(pos); incoming[:, 0] = -2.
    clip = torch.arange(n, device=device) % 2
    command = NS(cfg=NS(vb_capture_radius=.1, vb_min_approach_speed=.05,
                        vb_table_surface_z=.76, vb_rollout_h=.01, vb_rollout_steps=100,
                        vb_min_landing_depth=.05, exact_success_decay=.9,
                        exact_success_min_count=1., venue_tuple_unconditional_outcomes=unconditional),
                 _vb_params=vb.load_venue_params(), racket_pos_w=pos,
                 racket_lin_vel_w=velocity, racket_normal_w=normal,
                 vb_vel_in_w=incoming, vb_spin_in_w=torch.zeros_like(pos),
                 _env=NS(scene=NS(env_origins=torch.zeros_like(pos))),
                 _vb_net_x=1.87, _vb_far_x=3.24, _vb_half_w=.7625,
                 _vb_net_top_z=.9125, _vb_ball_r=.02, _vb_y_center=0., _vb_net_overhang=.1525,
                 _vb_target_xy=torch.tensor([2.7, 0.], device=device),
                 _venue_tuple_selected=(torch.arange(n, device=device) % 3 != 0),
                 _venue_intended_landing_xy=torch.tensor([2.6, .1],device=device).expand(n, 2),
                 _venue_tuple_outcome_pending=torch.ones(n,device=device,dtype=torch.bool),
                 _venue_tuple_outcome_clip=clip.clone(),
                 _clip_names={0:'forehand', 1:'backhand'}, _motion=lambda: NS(clip_id=clip),
                 metrics=defaultdict(lambda:torch.zeros(n,device=device)),
                 vb_fired=torch.zeros(n,device=device,dtype=torch.bool))
    for name in ('exact', 'hit', 'net', 'land_valid', 'inb'):
        setattr(command, f'_vb_{name}_acc', 2.)
    for name in ('exact', 'hit', 'net', 'legal', 'land_err_sum', 'land_err_n'):
        setattr(command, f'_vb_{name}_acc_c' if name not in ('land_err_sum', 'land_err_n')
                else f'_vb_{name}_c', {0:2.,1:3.})
    command._venue_swing_starts_acc_c={0:5.,1:7.}
    exact=torch.arange(n,device=device)%4!=0
    err=torch.rand(n,device=device,generator=gen)*.2
    return command, exact, err


@pytest.mark.parametrize('device', ['cpu','cuda'])
@pytest.mark.parametrize('unconditional', [False,True])
def test_batched_statistics_keep_physical_outcomes_and_sample_denominators(monkeypatch, device, unconditional):
    if device == 'cuda' and not torch.cuda.is_available():
        pytest.skip('CUDA unavailable')
    module=ModuleType('whole_body_tracking.tasks.tracking.mdp');module.virtual_ball=vb
    monkeypatch.setitem(sys.modules,module.__name__,module)
    cmd, exact, err=fixture(device,unconditional)
    expected_pending=cmd._venue_tuple_outcome_pending.clone()
    with torch.inference_mode():
        method()(cmd,exact,err)
    gate=exact & (err<.1)
    torch.testing.assert_close(cmd.vb_fired,gate,rtol=0,atol=0)
    assert cmd._vb_exact_acc == 1.8+float(exact.sum())
    assert cmd._vb_hit_acc == 1.8+float(gate.sum())
    intended=torch.where(cmd._venue_tuple_selected[:,None],cmd._venue_intended_landing_xy,cmd._vb_target_xy)
    error=(cmd.vb_landing_xy-intended).norm(dim=-1)
    for side in (0,1):
        selected=exact & cmd._venue_tuple_selected & (cmd._motion().clip_id==side)
        base=(2.+side)*(1. if unconditional else .9)
        assert cmd._vb_exact_acc_c[side] == base+float(selected.sum())
        assert cmd._vb_hit_acc_c[side] == base+float((selected & gate).sum())
        landed=selected & gate & cmd.vb_landing_valid
        assert cmd._vb_land_err_sum_c[side] == base+float((error*landed).sum())
        if unconditional:
            assert cmd._venue_swing_starts_acc_c[side] == (5.+2*side)+float(selected.sum())
    expected_pending[exact & cmd._venue_tuple_selected]=False
    torch.testing.assert_close(cmd._venue_tuple_outcome_pending,expected_pending)
