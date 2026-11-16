"""PPQ004 impulse regression and actual-contact-only lifecycle tests."""
import json,os,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mujoco_eval_recurrent import Plant
from recurrent_physical_ball import table_impact
from concurrent_physical_balls import ConcurrentBalls

@pytest.fixture
def setup():
    bundle=os.environ.get('HOPE_GRU_EVAL_BUNDLE')
    if not bundle:pytest.skip('requires frozen policy manifest')
    m=json.loads((Path(bundle)/'manifest.json').read_text())
    p=Plant(m,physical=True,ball_slots=2,table_impact_model='ppq004')
    return p,ConcurrentBalls(p,[1,2],slots=2)


def test_archived_map_matches_actual_cpp_predictor_impact(setup):
    _,pool=setup
    v,w=table_impact([-2.3566721207709076,0,-3.498080971289681],[0,0,0],pool.cfg)
    np.testing.assert_allclose(v,[-1.3753032994175594,0,3.2788629675231586],atol=1e-14)
    np.testing.assert_allclose(w,[0,-73.602661601501126,0],atol=1e-12)


@pytest.mark.parametrize('v,w',[([-3,1,-.2],[0,0,0]),([-2,0,-3],[0,-100,0]),([0,0,-2],[0,0,0]),([-2,.5,-3],[3,-20,11])])
def test_spin_slip_friction_cap_and_normal_restitution(setup,v,w):
    _,pool=setup;c=pool.cfg;v=np.array(v);w=np.array(w);out,spin=table_impact(v,w,c)
    radius=c['ball']['radius'];mass=c['ball']['mass'];inertia=c['ball']['inertia_coeff']*mass*radius**2
    assert out[2]==pytest.approx(-c['contact']['table']['e_eff']*v[2])
    assert np.linalg.norm(out[:2]-v[:2])<=c['contact']['table']['mu_safety']*(1+c['contact']['table']['e_eff'])*abs(v[2])+1e-12
    assert .5*mass*np.dot(out,out)+.5*inertia*np.dot(spin,spin)<=.5*mass*np.dot(v,v)+.5*inertia*np.dot(w,w)+1e-12


def test_actual_independent_contacts_only_no_position_or_robot_velocity_change(setup):
    p,pool=setup
    for sid,y in [(1,-.26),(2,.05)]:
        pool.launch({'shot_id':sid,'position_w':[2.9,y,1.25],'velocity_w':[-3,0,2.2],'spin_w':[0,0,0]},0)
        pool.release(0)
    for b in pool.slots:
        original=b.after
        def wrapped(t,b=b,original=original):
            q=p.data.qpos.copy();robot_v=p.data.qvel[:b.v].copy()
            original(t)
            np.testing.assert_array_equal(p.data.qpos,q)
            np.testing.assert_array_equal(p.data.qvel[:b.v],robot_v)
        b.after=wrapped
    for i in range(40):
        p.step(np.zeros(31))
        if i<29:assert all(not b.table_suppressed for b in pool.slots)
    for b in pool.slots:
        events=[e for e in b.events if e['kind']=='table_contact']
        assert len(events)==1;e=events[0]
        assert e['impact_model']=='ppq004_capped_slip_actual_contact_v1'
        assert e['normal_force']>0 and e['penetration']>0
        assert not e['position_modified']
        expected=table_impact(e['velocity_before'],e['angular_velocity_before_w'],b.cfg)
        np.testing.assert_allclose(e['velocity_after'],expected[0],atol=1e-12)
        np.testing.assert_allclose(e['angular_velocity_after_w'],expected[1],atol=1e-12)
        assert not b.table_suppressed
        assert p.model.pair_margin[b.table_pair]==b.table_pair_margin
    assert not p.data.warning.number.any()


def test_no_impulse_for_absent_table_contact_or_separating_map(setup):
    p,pool=setup
    pool.launch({'shot_id':1,'position_w':[-1,0,1.25],'velocity_w':[-3,0,2.2],'spin_w':[0,0,0]},0)
    for _ in range(40):p.step(np.zeros(31))
    b=pool.slots[0]
    assert not any(e.get('impact_model') for e in b.events)
    with pytest.raises(ValueError):table_impact([0,0,1],[0,0,0],b.cfg)
