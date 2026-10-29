"""Independent analytic checks and compiled production C++ full-model parity."""
from copy import deepcopy
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]


def _module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name+'.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


model = _module('full_ball_model')
profiles = _module('ball_physics_profile')


@pytest.fixture(scope='module')
def profile():
    return profiles.load_profile(REPO / 'configs/ball_physics_ppq004_training.yaml')


@pytest.fixture(scope='module')
def params(profile):
    return model.params_from_profile(profile)


@pytest.fixture(scope='module')
def compiled():
    numba = pytest.importorskip('numba')
    return {name: numba.njit(getattr(model, name)) for name in
            ('integrate_flight', 'first_bounce', 'evaluate_contact', 'plan', 'solve_racket')}


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    compiler = os.environ.get('CXX') or shutil.which('c++') or shutil.which('clang++')
    candidates = [os.environ.get('EIGEN3_INCLUDE_DIR'), Path(sys.prefix)/'include/eigen3',
                  '/usr/include/eigen3','/usr/local/include/eigen3','/opt/homebrew/include/eigen3',
                  '/opt/miniconda3/include/eigen3']
    eigen = next((Path(path) for path in candidates if path and (Path(path)/'Eigen/Core').is_file()),None)
    if not compiler or eigen is None:
        pytest.skip('Production C++ oracle requires C++17 and Eigen (EIGEN3_INCLUDE_DIR)')
    package=REPO/'hope_ws/src/hope_planner_cpp'
    executable=tmp_path_factory.mktemp('full-ball-native')/'oracle'
    subprocess.run([str(compiler),'-std=c++17','-O2','-I',str(eigen),'-I',str(package/'include'),
                    str(Path(__file__).parent/'fixtures/full_ball_oracle.cpp'),
                    str(package/'src/racket_target_planner.cpp'),'-o',str(executable)],
                   capture_output=True,text=True,check=True)
    return executable


def _native_rows(native, mode, params, cases):
    lines=[]
    for case in cases:
        values=[len(params),*params]
        for value in case:
            values.extend(np.asarray(value).reshape(-1).tolist())
        lines.append(mode+' '+' '.join(map(str,values)))
    result=subprocess.run([str(native)],input='\n'.join(lines)+'\n',text=True,
                          capture_output=True,check=True)
    return np.asarray([[float(item) for item in line.split()] for line in result.stdout.splitlines()])


def test_rk4_matches_analytic_magnus_rotation_and_drag(compiled, params):
    p=params.copy(); p[model.KD]=0; p[model.G]=0
    position=np.array([.1,-.4,.3]); velocity=np.array([3.,0,0]); spin=np.array([0.,0,50.])
    time=.17373; rate=p[model.KM]*spin[2]
    x,v,valid=compiled['integrate_flight'](position,velocity,spin,time,p)
    assert valid
    np.testing.assert_allclose(v,[3*np.cos(rate*time),3*np.sin(rate*time),0],atol=1e-11)
    np.testing.assert_allclose(x,position+[3*np.sin(rate*time)/rate,3*(1-np.cos(rate*time))/rate,0],atol=1e-11)
    p[model.KD]=params[model.KD]; p[model.KM]=0
    x,v,valid=compiled['integrate_flight'](position,velocity,spin,time,p)
    assert valid
    np.testing.assert_allclose(v,velocity/(1+p[model.KD]*3*time),atol=1e-11)
    np.testing.assert_allclose(x,position+[np.log1p(p[model.KD]*3*time)/p[model.KD],0,0],atol=1e-11)


def test_first_bounce_is_ball_center_radius_and_precedes_later_net(compiled, params):
    p=params.copy(); p[model.KD]=0; p[model.KM]=0
    position=np.array([.15,-.4,.3]); velocity=np.array([.5,0,-.2])
    result=compiled['first_bounce'](position,velocity,np.zeros(3),p)
    bounce,bounce_v,time,_,net_valid,valid=result
    analytic_time=(velocity[2]+np.sqrt(velocity[2]**2+2*p[model.G]*(position[2]-p[model.RADIUS])))/p[model.G]
    assert valid and not net_valid
    assert abs(time-analytic_time)<2e-7
    assert bounce[2]==pytest.approx(p[model.RADIUS],abs=1e-15)
    np.testing.assert_allclose(bounce[:2],position[:2]+velocity[:2]*analytic_time,atol=2e-7)
    assert bounce_v[2]<0
    # No second bounce is integrated; table restitution cannot change this first event.
    changed=p.copy(); changed[model.TAB_E]=.1; changed[model.TAB_AT]=.99
    other=compiled['first_bounce'](position,velocity,np.zeros(3),changed)
    np.testing.assert_array_equal(other[0],bounce)
    before=np.array([1.,.3,-2.]); omega=np.array([2.,-5.,1.])
    assert not np.allclose(model.table_contact(before,omega,p)[0],model.table_contact(before,omega,changed)[0])


def test_full_paddle_impulse_transfers_spin_and_obeys_friction_cap(params):
    p=params.copy(); p[model.PAD_AT]=10; p[model.PAD_BT]=0; p[model.PAD_MU]=.02
    incoming=np.array([-2.,3.,0]); racket=np.array([1.,0,0]); normal=np.array([1.,0,0]); spin=np.zeros(3)
    outgoing,omega=model.full_contact(incoming,racket,normal,spin,p)
    e=np.clip(p[model.PAD_G1]*np.exp(p[model.PAD_G2]*3),p[model.EMIN],p[model.EMAX])
    impulse=p[model.PAD_MU]*(1+e)*3
    np.testing.assert_allclose(outgoing,[-2+(1+e)*3,3-impulse,0],atol=2e-8)
    np.testing.assert_allclose(omega,[0,0,impulse/(p[model.INERTIA]*p[model.RADIUS])],atol=3e-8)
    spun=model.full_contact(incoming,racket,normal,np.array([0.,0.,50.]),p)
    assert not np.allclose(spun[1],omega)


@pytest.mark.parametrize('changed',[False,True])
def test_full_forward_matches_production_cpp(native,profile,compiled,changed):
    raw=deepcopy(profile)
    if changed:
        raw['flight'].update(k_d=.21,k_m=.008,g=8.4)
        raw['ball'].update(radius=.025,inertia_coeff=.55)
        raw['contact']['paddle'].update(a_t=.31,b_t=.19,mu_safety=.14,e_exp_g1=.88,e_exp_g2=-.08)
        raw['return_model'].update(integrate_dt_s=.00073,restitution_racket=.61)
    p=model.params_from_profile(raw)
    cases=[([.15,-.45,.32],[-2.2,.18,.18],[0,-30,5],[1.97,-.3,.49]),
           ([.15,-1.1,.25],[-4,.5,-.7],[30,10,-40],[2.3,.3,.7]),
           ([.15,.7,.3],[-3,0,-.2],[-10,25,15],[2.1,.2,.6]),
           ([.15,-.5,.3],[-3,0,-.2],[0,0,0],[0,0,0])]
    expected=_native_rows(native,'forward',p,cases)
    for i,(position,incoming,spin,racket) in enumerate(cases):
        arrays=[np.asarray(v,dtype=float) for v in (position,incoming,spin,racket)]
        n,out,w,bounce,time,net,crossed,valid=compiled['evaluate_contact'](*arrays,p)
        bounce_v=np.zeros(3)
        if valid: bounce_v=compiled['first_bounce'](arrays[0],out,w,p)[1]
        actual=np.array([valid,crossed,*n,*out,*w,*bounce,*bounce_v,*net,time])
        np.testing.assert_array_equal(actual[:2],expected[i,:2])
        np.testing.assert_allclose(actual[2:],expected[i,2:],atol=2e-10,rtol=2e-10)


def test_inverse_matches_cpp_with_spin_net_retries_and_bypass(native,params,compiled):
    cases=[([.15,-.45,.32],[-2.2,.18,.18],[0,-30,5],[2.055,-.7625],.5),
           ([.15,-1.1,.25],[-4,.5,-.7],[30,10,-40],[2.3,-.4],.50037),
           ([.15,-.65,.1],[-4,0,-1],[3,-20,5],[2.4,-.8],.05),
           ([.15,.7,.3],[-3,0,-.2],[-10,25,15],[2.1,.8],.5),
           ([.15,-.45,.32],[-2.2,.18,.18],[0,-30,5],[2.055,-.7625],0.)]
    expected=_native_rows(native,'plan',params,cases)
    for i,(position,incoming,spin,target,time) in enumerate(cases):
        arrays=[np.asarray(v,dtype=float) for v in (position,incoming,spin,target)]
        vr,n,out,w,bounce,duration,valid,clear,bypass=compiled['plan'](*arrays,time,params)
        actual=np.array([valid,clear,bypass,*vr,*n,*out,*w,*bounce,duration])
        np.testing.assert_array_equal(actual[:3],expected[i,:3])
        np.testing.assert_allclose(actual[3:],expected[i,3:],atol=2e-8,rtol=2e-8)
        if valid:
            np.testing.assert_allclose(n,vr/(np.linalg.norm(vr)+params[model.EPS]),atol=1e-15)
            assert np.linalg.norm(bounce[:2]-arrays[3])<=params[model.POS_TOL]
    assert expected[2,0] and expected[2,1]  # Original short arc needs configured retry.
    assert tuple(expected[3,:3])==(1,0,1)
    assert not expected[4,0]


def test_nonconvergence_is_rejected_not_reported_as_finite_success(params,compiled):
    p=params.copy(); p[model.NEWTON_ITER]=1; p[model.POS_TOL]=1e-12; p[model.TIME_TOL]=1e-12
    result=compiled['solve_racket'](np.array([.15,-.45,.32]),np.array([-2.2,.18,.18]),
                                    np.array([10.,-30.,5.]),np.array([2.055,-.7625]),.5,p)
    assert np.isfinite(result[0]).all() and not result[-1]


def test_spin_changes_the_required_command_and_qualified_first_bounce(params,compiled):
    args=[np.array([.15,-.45,.32]),np.array([-2.2,.18,.18]),None,np.array([2.055,-.7625]),.5,params]
    args[2]=np.zeros(3); zero=compiled['plan'](*args)
    args[2]=np.array([25.,-40.,35.]); spinning=compiled['plan'](*args)
    assert zero[6] and spinning[6]
    assert np.linalg.norm(zero[0]-spinning[0])>.01
    assert np.linalg.norm(zero[3]-spinning[3])>10


@pytest.mark.parametrize('block,key,value', [
    ('return_model', 'net_clearance_margin', -.001),
    ('table', 'width', np.nan),
    ('net', 'x_position', np.nan),
    ('net', 'overhang', np.inf),
])
def test_profile_rejects_invalid_clearance_and_nonfinite_geometry(profile, block, key, value):
    changed = deepcopy(profile)
    changed[block][key] = value
    with pytest.raises(ValueError, match='finite|clearance'):
        model.params_from_profile(changed)
