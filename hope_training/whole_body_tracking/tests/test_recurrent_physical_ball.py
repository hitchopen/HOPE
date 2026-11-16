"""Independent collision/flight checks against the frozen recurrent bundle."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
SCRIPTS=ROOT/'hope_training/whole_body_tracking/scripts'
sys.path.insert(0,str(SCRIPTS))
from mujoco_eval_recurrent import Plant
from recurrent_physical_ball import incoming_serve,flight_step

@pytest.fixture
def setup():
    bundle=os.environ.get('HOPE_GRU_EVAL_BUNDLE')
    if not bundle:pytest.skip('requires frozen bundle')
    manifest=json.loads((Path(bundle)/'manifest.json').read_text())
    plant=Plant(manifest,physical=True)
    bank=np.load(ROOT/'hope_training/whole_body_tracking'/manifest['task']['racket']['venue_tuple_bank_path'])
    return plant,bank


def test_reverse_serve_roundtrip_both_sides_and_truncation(setup):
    plant,bank=setup;pb=plant.ball_physics
    truncated=0
    for side in [0,1]:
        for row in np.flatnonzero(bank['clip']==side)[::10000]:
            p=bank['target_position_offset'][row].astype(float);v=bank['incoming_velocity'][row].astype(float);w=bank['incoming_spin'][row].astype(float)
            launch,vel,horizon,path=incoming_serve(p,v,w,.6,pb.cfg,.76)
            assert np.min(path[:,2])>.785
            assert np.linalg.norm(vel)<40
            truncated+=horizon<.599
            for _ in range(round(horizon/.001)):launch,vel=flight_step(launch,vel,w,.001,pb.cfg)
            np.testing.assert_allclose(launch,p,atol=1e-7)
            np.testing.assert_allclose(vel,v,atol=1e-7)
    assert truncated


def test_physical_mode_preserves_policy_io_and_control_contract(setup):
    plant,_=setup;other=Plant(plant.cfg)
    from mujoco_eval_recurrent import ObsTarget
    target=ObsTarget(np.array([.58,-.44,1.1]),np.array([2.,.2,1.]),.5)
    np.testing.assert_array_equal(plant.observe(target),other.observe(target))
    assert plant.dt==other.dt==.02
    assert plant.substeps*plant.model.opt.timestep==.02
    for a in [np.linspace(-1,1,31),np.zeros(31),np.ones(31)]:
        for actual,expected in zip(plant.decode(a),other.decode(a)):np.testing.assert_array_equal(actual,expected)
    pb=plant.ball_physics;m=plant.model
    assert m.geom_contype[pb.gid]==m.geom_conaffinity[pb.gid]==0
    assert m.geom_contype[pb.table_gid]==m.geom_conaffinity[pb.table_gid]==0
    assert m.body_mass[pb.bid]==pytest.approx(.0034)
    np.testing.assert_allclose(m.body_inertia[pb.bid],(2/3)*.0034*.02**2)


@pytest.mark.parametrize('surface',['table','net','floor','racket'])
def test_actual_collision_and_finite_response(setup,surface):
    plant,bank=setup;pb=plant.ball_physics;m=plant.model;d=plant.data
    pb.schedule(bank,0,10,0,0)
    if surface=='table':p=[2.5,0,1.05];v=[0,0,-2]
    elif surface=='floor':p=[-1,0,.3];v=[0,0,-2]
    elif surface=='net':p=[1.7,0,.84];v=[4,0,0]
    else:
        normal=d.site_xmat[plant.racket].reshape(3,3)[:,1]
        p=d.site_xpos[plant.racket]+.12*normal;v=-4*normal
    pb.pending=(0,np.array(p,dtype=float),np.array(v,dtype=float),np.zeros(3))
    velocities=[]
    for _ in range(20):
        plant.step(np.zeros(31))
        velocities.append(pb.state()[1])
    assert any(e['kind']==surface+'_contact' and e['normal_force']>0 for e in pb.events)
    assert np.max(np.linalg.norm(velocities,axis=1))<15
    assert not d.warning.number.any()
    if surface=='table':assert max(v[2] for v in velocities)>1


def test_drawing_does_not_move_ball_or_change_actor_state(setup):
    from contextlib import nullcontext
    from types import SimpleNamespace
    from mujoco_eval_recurrent import ObsTarget
    plant,bank=setup;pb=plant.ball_physics
    pb.schedule(bank,0,.65,0,0)
    scene=plant.mj.MjvScene(plant.model,maxgeom=2000)
    plant.viewer=SimpleNamespace(user_scn=scene,lock=nullcontext,sync=lambda:None,set_texts=lambda x:None)
    qpos=plant.data.qpos.copy();action=plant.previous_action.copy()
    pb.draw(ObsTarget(np.array([.58,-.44,1.1]),np.zeros(3),.5),0)
    np.testing.assert_array_equal(plant.data.qpos,qpos)
    np.testing.assert_array_equal(plant.previous_action,action)


def test_legal_return_requires_contact_and_clearance_before_first_landing(setup):
    plant,bank=setup;pb=plant.ball_physics
    pb.schedule(bank,0,.65,0,0);s=pb.current
    np.testing.assert_array_equal(bank['station_xy'][0],np.zeros(2))
    s['racket_contacts']=1;s['net_clear']=True
    pb.landing([2.5,0,.78],1.2,'surface_plane')
    assert not s['legal_return']  # geometric plane crossing alone is insufficient
    s['table_contacts']=1;pb.update_legal();assert s['legal_return']
    s['racket_contacts']=2;pb.update_legal();assert not s['legal_return']
    s['racket_contacts']=1;s['net_contacts']=1;pb.update_legal();assert not s['legal_return']
    s['net_contacts']=0;s['first_landing']['position']=[1.,0,.78]
    pb.update_legal();assert not s['legal_return']
    s['first_landing']['position']=[2.5,1.,.78]
    pb.update_legal();assert not s['legal_return']


def test_unresolved_shots_are_censored_and_floor_miss_is_recorded(setup):
    plant,bank=setup;pb=plant.ball_physics
    pb.schedule(bank,0,.65,0,0)
    pb.finish('evaluation_end')
    assert not pb.shots[-1]['complete']
    pb.schedule(bank,0,.65,1,0);pb.current['floor_contacts']=1
    pb.finish('evaluation_end')
    assert pb.shots[-1]['complete'] and pb.shots[-1]['outcome']=='miss'
