"""Retarget an existing stationary bank under an explicitly selected ball profile.

Keeps at-strike incoming states, positions, intended landings, parent IDs and
split membership. Fits a minimum-norm three-axis racket adjustment, then
rechecks physical bounds.
This does not identify pre-bounce states, change a policy, or start training.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

# Isaac's optional coverage package is incompatible with its Numba release.
# Coverage instrumentation is not used by this offline numeric executable.
sys.modules['coverage'] = None
from numba import njit, prange, set_num_threads


@njit
def accel(v, w, p):
    a = -p[0] * np.linalg.norm(v) * v + p[1] * np.cross(w, v)
    a[2] -= p[2]
    return a


@njit
def step(x, v, w, h, p):
    a = accel(v, w, p)
    b = accel(v + .5*h*a, w, p)
    c = accel(v + .5*h*b, w, p)
    d = accel(v + h*c, w, p)
    return x+h*(v+2*(v+.5*h*a)+2*(v+.5*h*b)+(v+h*c))/6, v+h*(a+2*b+2*c+d)/6


@njit
def landing(x, v, w, p, h=.005, horizon=1.2):
    x = x.copy()
    v = v.copy()
    net = -1e9
    for _ in range(int(round(horizon/abs(h)))):
        y, u = step(x, v, w, h, p)
        if x[0] < 1.87 <= y[0] and net == -1e9:
            f = (1.87-x[0])/(y[0]-x[0])
            net = x[2]+f*(y[2]-x[2])
        if x[2] > .76 >= y[2]:
            f = (x[2]-.76)/(x[2]-y[2])
            return x[:2]+f*(y[:2]-x[:2]), net, True
        x, v = y, u
    return np.zeros(2), net, False


@njit
def paddle(incoming, racket, spin, p):
    normal = racket / (np.linalg.norm(racket)+1e-9)
    if np.dot(incoming-racket, normal)>0:
        normal = -normal
    r = -p[3]*normal
    u = incoming+np.cross(spin, r)-racket
    un = np.dot(u, normal)
    ut = u-un*normal
    speed = np.linalg.norm(ut)
    e = min(.95, max(.05, p[8]*np.exp(p[9]*abs(un))))
    raw = (p[5]+p[6]*abs(un)/(np.hypot(speed, un)+1e-9))*speed
    impulse = min(max(0.,raw), p[7]*(1+e)*abs(un))
    dv = -impulse*ut/(speed+1e-9)
    out = incoming-(1+e)*un*normal+dv
    omega = spin-np.cross(normal,dv)/(p[4]*p[3])
    return out, omega


@njit
def evaluate(x, incoming, spin, racket, p, h=.005):
    out, omega = paddle(incoming, racket, spin, p)
    xy, net, valid = landing(x, out, omega, p, h)
    return xy, net, valid, out


@njit(parallel=True)
def refit(x, incoming, spin, racket, target, clip, p, vboxes):
    fitted = racket.copy()
    n = len(x)
    for i in prange(n):
        v = racket[i].copy()
        side = int(clip[i])
        for _ in range(8):
            xy, net, valid, out = evaluate(x[i], incoming[i], spin[i], v, p)
            if not valid:
                break
            residual = xy-target[i]
            if np.linalg.norm(residual)<2e-5:
                break
            jac = np.empty((2,3))
            for axis in range(3):
                perturbed=v.copy()
                perturbed[axis] += 1e-4
                y, _, ok, _ = evaluate(x[i], incoming[i], spin[i], perturbed, p)
                jac[:,axis]=(y-xy)/1e-4
            # Minimum-norm solution of J delta = residual. Damping avoids
            # magnifying near-singular geometry into an unrelated stroke.
            gram=jac@jac.T
            gram[0,0]+=1e-8
            gram[1,1]+=1e-8
            determinant=gram[0,0]*gram[1,1]-gram[0,1]*gram[1,0]
            if abs(determinant)<1e-10:
                break
            dual=np.array([(gram[1,1]*residual[0]-gram[0,1]*residual[1])/determinant,
                           (-gram[1,0]*residual[0]+gram[0,0]*residual[1])/determinant])
            delta=jac.T@dual
            for axis in range(3):
                v[axis] -= min(.1,max(-.1,delta[axis]))
                v[axis] = min(vboxes[side,axis,1],max(vboxes[side,axis,0],v[axis]))
        fitted[i]=v
    return fitted


@njit(parallel=True)
def check(x, incoming, spin, racket, target, p, h):
    result=np.empty((len(x),9))
    for i in prange(len(x)):
        xy, net, valid, out=evaluate(x[i],incoming[i],spin[i],racket[i],p,h)
        bounce, _, bounce_valid=landing(x[i],incoming[i],spin[i],p,-h,.8)
        result[i,:2]=xy
        result[i,2]=net
        result[i,3]=valid
        result[i,4:6]=bounce
        result[i,6]=bounce_valid
        result[i,7]=np.linalg.norm(out)
        result[i,8]=np.linalg.norm(xy-target[i])
    return result


def mask_valid(a, result, receipt, fit_tolerance=.002):
    pos=a['target_position_offset']; v=a['target_velocity']
    n=v/np.linalg.norm(v,axis=1,keepdims=True)
    speed=np.abs(np.sum((a['incoming_velocity']-v)*n,axis=1))
    criteria={
        'landing':(result[:,3]>0)&(result[:,0]>2.20)&(result[:,0]<3.10)&(np.abs(result[:,1])<.65),
        'net':result[:,2]>.9675,
        'incoming_bounce':(result[:,6]>0)&(result[:,4]>.52)&(result[:,4]<1.85)&(np.abs(result[:,5])<.7425),
        'speed':(np.linalg.norm(a['incoming_velocity'],axis=1)<=7)&(result[:,7]<=7)&(np.linalg.norm(v,axis=1)<=3.5),
        'spin':np.linalg.norm(a['incoming_spin'],axis=1)<=15*2*np.pi,
        'paddle_approach':(speed>=1.4)&(speed<=7.2),
        'landing_fit':result[:,8]<=fit_tolerance,
        'finite':np.isfinite(result).all(1)&np.isfinite(v).all(1),
        'reach':np.ones(len(v),dtype=bool),
    }
    for side in (0,1):
        ids=a['clip']==side
        for key,values in [('reach_boxes',pos),('velocity_boxes',v)]:
            box=np.array([receipt['contract'][key][str(side)][axis] for axis in 'xyz'])
            criteria['reach'][ids] &= ((values[ids]>=box[:,0]-1e-6)&(values[ids]<=box[:,1]+1e-6)).all(1)
    return np.logical_and.reduce(list(criteria.values())), {k:int((~v).sum()) for k,v in criteria.items()}


def params_from_yaml(path):
    raw=yaml.safe_load(Path(path).read_text()); pad=raw['contact']['paddle']
    return np.array([raw['flight']['k_d'],raw['flight']['k_m'],raw['flight']['g'],raw['ball']['radius'],
                     raw['ball']['inertia_coeff'],pad['a_t'],pad['b_t'],pad['mu_safety'],pad['e_exp_g1'],pad['e_exp_g2']],dtype=np.float64)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--physics',type=Path,required=True)
    parser.add_argument('--threads',type=int,default=12)
    args=parser.parse_args(); set_num_threads(args.threads)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    p=params_from_yaml(args.physics)
    summaries={}; parent_sets=[]
    for split in ('train','dev','test'):
        source=args.source/f'{split}.npz'; old_receipt=json.loads(source.with_suffix('.receipt.json').read_text())
        assert sha(source)==old_receipt['bank_sha256']
        a=dict(np.load(source,allow_pickle=False)); n=len(a['clip'])
        x=a['target_position_offset'].astype(np.float64); x[:,:2]+=a['station_xy']
        incoming=a['incoming_velocity'].astype(np.float64); spin=a['incoming_spin'].astype(np.float64)
        velocity=a['target_velocity'].astype(np.float64); target=a['intended_landing_xy'].astype(np.float64)
        boxes=np.array([[old_receipt['contract']['velocity_boxes'][str(s)][axis] for axis in 'xyz'] for s in (0,1)])
        print(f'{split}: refitting {n} rows',flush=True)
        before=check(x,incoming,spin,velocity,target,p,.005)
        v=refit(x,incoming,spin,velocity,target,a['clip'],p,boxes).astype(np.float32)
        a['target_velocity']=v; a['target_normal']=(v/np.linalg.norm(v,axis=1,keepdims=True)).astype(np.float32)
        after=check(x,incoming,spin,v.astype(np.float64),target,p,.005)
        # Every row is independently re-integrated at 1 ms, with original margins.
        fine=check(x,incoming,spin,v.astype(np.float64),target,p,.001)
        good,reasons=mask_valid(a,after,old_receipt)
        fine_good,fine_reasons=mask_valid(a,fine,old_receipt,fit_tolerance=.003)
        good &= fine_good
        delta=np.linalg.norm(v-velocity,axis=1)
        a['outgoing_velocity']=np.array([paddle(incoming[i],v[i].astype(np.float64),spin[i],p)[0] for i in range(n)],dtype=np.float32)
        a['predicted_landing_xy']=after[:,:2].astype(np.float32)
        a['source_row_index']=np.arange(n,dtype=np.int64)
        retained={k:val[good] if val.ndim else val for k,val in a.items()}
        assert np.all(retained['station_xy']==0)
        summary={'source_rows':n,'retained_rows':int(good.sum()),'rejected_rows':int((~good).sum()),
                 'failure_counts_overlapping':reasons,'fine_failure_counts_overlapping':fine_reasons,
                 'before_landing_error_m_p50_p95_max':np.percentile(before[:,8],[50,95,100]).tolist(),
                 'after_landing_error_m_p50_p95_max':np.percentile(after[good,8],[50,95,100]).tolist(),
                 'fine_landing_error_m_p50_p95_max':np.percentile(fine[good,8],[50,95,100]).tolist(),
                 'racket_velocity_change_mps_p50_p95_max':np.percentile(delta[good],[50,95,100]).tolist(),
                 'sides':{}}
        for side in (0,1):
            ids=retained['clip']==side; wide=ids&(retained['source_kind']==1)
            bounds=old_receipt['contract']['reach_boxes'][str(side)]['y']
            summary['sides'][str(side)]={'rows':int(ids.sum()),'core_rows':int((ids&~wide).sum()),'wide_rows':int(wide.sum()),
                'wide_lateral_histogram':np.histogram(retained['target_position_offset'][wide,1],np.linspace(*bounds,5))[0].tolist()}
            if split=='train' and ids.sum()<50000:
                raise RuntimeError('Refit does not meet original minimum 50000 train rows per side')
        path=args.output/f'{split}.npz'; np.savez_compressed(path,**retained)
        np.savez_compressed(args.output/f'{split}.refit_audit.npz',accepted=good,source_row_index=np.arange(n),
                            before=before,after=after,fine=fine,velocity_delta=delta)
        receipt={'schema_version':2,'split':split,'rows':int(good.sum()),'bank_sha256':sha(path),
                 'source':{'bank':str(source.resolve()),'bank_sha256':sha(source),'receipt_sha256':sha(source.with_suffix('.receipt.json')),
                           'physics_yaml':str(args.physics.resolve()),'physics_yaml_sha256':sha(args.physics),'refitter_sha256':sha(__file__)},
                 'contract':dict(old_receipt['contract'],recipe_revision=20,qualification='ppq004_at_strike_retarget_and_fine_requalification_v1'),
                 'summary':summary,'limitations':['At-strike incoming velocity/spin held fixed; no observed pre-bounce state is available.',
                   'PPQ004 table coefficients are supplied in the profile but do not change fixed post-bounce at-strike inputs.',
                   'Paddle model retained from venue fit; PPQ004 did not re-identify it.',
                   'Original bank table-plane convention (surface_z=0.76) retained; no claim of complete C++ Planner parity.',
                   'Ball geometry only: policy performance, long-horizon balance and hardware qualification NOT_PROVEN.']}
        path.with_suffix('.receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        parent_sets.append(set(retained['parent_id'].tolist())); summaries[split]=summary
        print(json.dumps(summary),flush=True)
    assert not(parent_sets[0]&parent_sets[1] or parent_sets[0]&parent_sets[2] or parent_sets[1]&parent_sets[2])
    (args.output/'coverage.json').write_text(json.dumps(summaries,indent=2)+'\n')
    train=args.output/'train.npz'; receipt=train.with_suffix('.receipt.json')
    overrides={'environment':{'HOPE_BALL_PHYSICS_YAML':str(args.physics.resolve())},'task':{'racket':{
        'venue_tuple_bank_path':str(train.resolve()),'venue_tuple_bank_receipt_path':str(receipt.resolve()),
        'venue_tuple_bank_sha256':sha(train),'venue_tuple_bank_receipt_sha256':sha(receipt),
        'venue_tuple_bank_recipe_revision':20}}}
    (args.output/'training_overrides.yaml').write_text(yaml.safe_dump(overrides,sort_keys=False))


if __name__=='__main__':
    main()
