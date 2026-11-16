import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from planner_rally_protocol import make_protocol,decode_command,physical_outcome,summarize,valid_delivery
from mujoco_eval_planner_rally import blank,spawn


def event(kind,t,p=(2.5,0,.78),**kw):return {'kind':kind,'episode_t':t,'position':list(p),**kw}


def test_exact_balanced_mixed_protocol_reproduces_existing_clean_serves():
    a=make_protocol(0);assert a==make_protocol(0)
    assert len(a['serves'])==100
    assert sum(s['side']==0 for s in a['serves'])==50
    assert any(a['serves'][i]['side']==a['serves'][i+1]['side'] for i in range(99))
    for s in a['serves']:
        assert s['launch_at_s']==2+5*s['id']
        assert 1<np.linalg.norm(s['velocity_w'])<7
        assert s['position_w'][0]==pytest.approx(2.9)
        assert s['position_w'][2]==pytest.approx(1.25)
        assert (s['position_w'][1]<-.25)==(s['side']==0)


def packet(now=100.):
    return [2,1,1,.08,-1.0225,.76+.3,2,0,1,.5,now+.5,0,int(now),0,1,1,1,24,.08]


def test_schema2_frame_z_only_once_and_deadline():
    d,reason=decode_command(packet(),99_000_000_000,100_000_000_000)
    assert reason=='accepted'
    np.testing.assert_allclose(d['position_w'],[.58,-.26,1.06])
    np.testing.assert_allclose(d['velocity_w'],[2,0,1])
    assert d['deadline_unix_s']==100.5


@pytest.mark.parametrize('change,reason',[(lambda x:x.__setitem__(10,99),'timestamp_contract'),(lambda x:x.__setitem__(1,0),'planner_invalid'),(lambda x:x.__setitem__(3,float('nan')),'malformed')])
def test_bad_packets_rejected(change,reason):
    p=packet();change(p)
    assert decode_command(p,99_000_000_000,100_000_000_000)[1]==reason


def test_stale_wrong_flight_and_late_rejected():
    assert decode_command(packet(),101_000_000_000,101_000_000_000)[1]=='wrong_flight_or_clock'
    assert decode_command(packet(),99_000_000_000,100_490_000_000)[1]=='late_command'


def test_legal_requires_force_contact_net_and_first_actual_table_landing():
    e=[event('racket_contact',1),event('net_crossing_outgoing',1.2,clear=True,after_racket=True),event('first_return_landing',1.5),event('table_contact',1.501)]
    assert physical_outcome(e)['legal_return']
    assert not physical_outcome(e[:-1])['legal_return']
    assert not physical_outcome(e+[event('net_contact',1.3)])['legal_return']
    assert not physical_outcome(e+[event('racket_contact',1.1)])['legal_return']
    e[2]['position']=[3.5,0,.78];assert not physical_outcome(e)['legal_return']


def test_delivery_is_not_spawn_or_reference_strike():
    e=[event('launch',0),event('net_crossing_incoming',.4,clear=True),event('table_contact',.7,(1.1,0,.78)),event('hit_plane_incoming',.9,(.58,0,1.))]
    assert valid_delivery(e)
    assert not valid_delivery(e[:1])
    assert not valid_delivery(e+[event('net_contact',.4)])


def test_denominators_include_all_100_not_just_completed():
    rows=[blank(s) for s in make_protocol()['serves']]
    rows[0].update(started=True,valid_delivery=True,planner_on_time=True,contact=True,legal_return=True,interrupted=False,end_to_end_success=True)
    r=summarize(rows)
    assert r['all']['end_to_end']=={'numerator':1,'denominator':100,'rate':.01}
    assert r['all']['valid_delivery_conditioned']['denominator']==1
    assert r['all']['contact_conditioned_physical_legal']['denominator']==1
    assert r['all']['unresolved_interrupted']==99
    assert r['longest_success_streak']==1
    assert r['longest_reset_free_segment']==1


def test_spawn_does_not_reset_robot_or_gru():
    from types import SimpleNamespace
    calls=[]
    pb=SimpleNamespace(active=False,park=lambda:calls.append('ball_park'),contacts=set(),history=[],plant=SimpleNamespace(episode_time=7.))
    serve=make_protocol()['serves'][0];spawn(pb,serve,3)
    assert calls==['ball_park']
    assert pb.pending[0]==7.
    np.testing.assert_array_equal(pb.pending[1],serve['position_w'])
    assert pb.current['episode']==3
    assert not hasattr(pb,'reset_episode')


def test_real_plant_and_gru_memory_unchanged_by_ball_spawn():
    import os,json
    from mujoco_eval_recurrent import Plant,RecurrentOnnxPolicy
    path=os.environ.get('HOPE_GRU_EVAL_BUNDLE')
    if not path:pytest.skip('set frozen GRU bundle for real-plant memory test')
    policy=RecurrentOnnxPolicy(path);plant=Plant(policy.manifest,physical=True)
    fixture=np.load(Path(path)/'parity_vectors.npz')
    for obs in fixture['obs'][:8]:policy.infer(obs)
    hidden=policy.hidden.copy();qpos=plant.data.qpos[plant.qadr].copy();root=plant.state().base_pos_w.copy()
    for serve in make_protocol()['serves'][:3]:
        spawn(plant.ball_physics,serve,0)
        np.testing.assert_array_equal(policy.hidden,hidden)
        np.testing.assert_array_equal(plant.data.qpos[plant.qadr],qpos)
        np.testing.assert_array_equal(plant.state().base_pos_w,root)
        assert policy.reset_count==0


def test_fall_after_resolved_legal_ball_keeps_physical_truth_but_fails_end_to_end():
    from planner_rally_protocol import finalize_row
    row=blank(make_protocol()['serves'][0]);row.update(planner_on_time=True,fall_reset=True)
    e=[event('net_crossing_incoming',.2,clear=True),event('table_contact',.4,(1.,0,.78)),event('hit_plane_incoming',.5),
       event('racket_contact',.6),event('net_crossing_outgoing',.8,clear=True,after_racket=True),
       event('first_return_landing',1.),event('table_contact',1.001)]
    finalize_row(row,e,'fall',1.)
    assert row['legal_return'] and not row['interrupted']
    assert not row['end_to_end_success'] and not row['ready_proxy']
    row=blank(make_protocol()['serves'][0]);row['planner_on_time']=True
    finalize_row(row,e[:4],'slot_deadline_unresolved',0.)
    assert row['interrupted'] and not row['end_to_end_success']
