import os,sys,json
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'hope_training/whole_body_tracking/scripts'))
from mujoco_eval_recurrent import Plant,RecurrentOnnxPolicy
from concurrent_physical_balls import ConcurrentBalls,PoolFull
from reference_gru_diagnostic import reference_ledger,advance_without_actor,end_rally

@pytest.fixture
def bundle():
    p=os.environ.get('HOPE_GRU_EVAL_BUNDLE')
    if not p:pytest.skip('requires existing frozen recurrent bundle')
    return Path(p)

@pytest.fixture
def setup(bundle):
    policy=RecurrentOnnxPolicy(bundle);plant=Plant(policy.manifest,physical=True,ball_slots=2)
    pool=ConcurrentBalls(plant,[1,2,3],slots=2)
    return plant,pool,policy


def serve(i,x=-1.,z=.4):
    return {'shot_id':i,'position_w':[x,0,z],'velocity_w':[0.,0.,-2.],'spin_w':[0.,0.,0.]}


def test_reference_ledger_is_exact_official_sequence_and_cadence():
    ledger,env=reference_ledger(ROOT/'a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home-100.yaml')
    assert len(ledger)==100
    assert [r['flight_s'] for r in ledger]==[2.5]*6+[1.15]*93+[2.5]
    assert [r['pause_s'] for r in ledger]==[1.]*6+[0.]*94
    assert ledger[0]['nominal_launch_at_s']==20.
    assert ledger[6]['nominal_launch_at_s']==41.
    assert ledger[-1]['nominal_launch_at_s']==147.95
    assert all('side' not in r for r in ledger)
    np.testing.assert_allclose(ledger[0]['position_w'],[2.9,-.26,1.25])


def test_handoff_preserves_both_bodies_robot_and_memory(setup,bundle):
    plant,pool,policy=setup
    fixture=np.load(bundle/'parity_vectors.npz')
    for obs in fixture['obs'][:8]:policy.infer(obs)
    hidden=policy.hidden.copy();robot=plant.data.qpos[plant.qadr].copy()
    pool.launch(serve(1,z=1.),0);pool.before(0)
    first=pool.incoming;pos=plant.data.qpos[first.q:first.q+7].copy();vel=plant.data.qvel[first.v:first.v+6].copy()
    pool.release(.1);pool.launch(serve(2,x=-2.,z=1.1),.1)
    np.testing.assert_array_equal(plant.data.qpos[first.q:first.q+7],pos)
    np.testing.assert_array_equal(plant.data.qvel[first.v:first.v+6],vel)
    np.testing.assert_array_equal(plant.data.qpos[plant.qadr],robot)
    np.testing.assert_array_equal(policy.hidden,hidden)
    assert pool.live==[1,2] and pool.planner_ball().shot_id==2
    pool.before(.1);assert pool.planner_ball().shot_id==2
    assert first.active and pool.records[1]['released_at_s']==.1
    assert policy.reset_count==0


def test_full_pool_waits_without_truncating_and_reuses_only_natural_endpoints(setup):
    plant,pool,_=setup
    pool.launch(serve(1),0);pool.release(0);pool.launch(serve(2,x=-2.),0);pool.release(0)
    with pytest.raises(PoolFull):pool.launch(serve(3),0)
    with pytest.raises(RuntimeError):pool.park()
    for _ in range(30):advance_without_actor(plant)
    assert not pool.live
    before=json.dumps(pool.records[1]['events']);history=list(pool.records[1]['history'])
    report=pool.score.report();assert all(r['trajectory_complete'] for r in report['rows'][:2])
    pool.launch(serve(3),plant.episode_time)
    for _ in range(30):advance_without_actor(plant)
    assert json.dumps(pool.records[1]['events'])==before
    assert len(pool.records[1]['history'])==len(history)
    assert pool.score.report()['trajectory_complete_count']==3


def test_contacts_are_independent_for_table_and_floor(setup):
    plant,pool,_=setup
    pool.launch(serve(1,x=2.5,z=1.05),0);pool.release(0)
    pool.launch(serve(2,x=-2.,z=.3),0)
    for _ in range(25):advance_without_actor(plant)
    a=pool.records[1]['events'];b=pool.records[2]['events']
    assert any(e['kind']=='table_contact' for e in a)
    assert not any(e['kind']=='table_contact' for e in b)
    assert any(e['kind']=='floor_contact' for e in b)
    assert pool.records[2]['terminal_kind']=='ground_crossing'
    assert pool.records[1]['terminal_at_s'] is None
    assert pool.peak_concurrent==2


def test_end_on_fall_stops_planner_ownership_without_reset_or_deleting_flights(setup,bundle):
    plant,pool,policy=setup
    policy.infer(np.load(bundle/'parity_vectors.npz')['obs'][0]);h=policy.hidden.copy()
    pool.launch(serve(1),0);pool.release(0);pool.launch(serve(2,x=-2.),0)
    end_rally(pool,0)
    assert pool.incoming is None and pool.live==[1,2]
    for _ in range(30):advance_without_actor(plant)
    assert not pool.live and pool.score.report()['trajectory_complete_count']==2
    np.testing.assert_array_equal(policy.hidden,h)
    assert policy.reset_count==0
