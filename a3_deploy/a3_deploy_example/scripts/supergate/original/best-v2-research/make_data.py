from pathlib import Path
import sys,json,itertools,concurrent.futures,hashlib,time,copy
import numpy as np,yaml
W=Path(__file__).resolve().parent;T=W.parent
sys.path.insert(0,str(T));sys.path.append(str(T/'numerical-tools'))
from build_super_factorial import solve

def cell_job(spec):
    cell,n,offset=spec;ok=[];attempts=[]
    for variant in range(offset,offset+300):
        r=solve((*cell,variant));attempts.append(r)
        if r['feasible']:ok.append(r)
        if len(ok)==n:break
    assert len(ok)==n,(cell,len(ok),n)
    return ok,attempts

def freeze(label,rows,attempts):
    out=W/('data-'+label);out.mkdir(exist_ok=False)
    rng=np.random.default_rng(2026110901+(label=='holdout'));shards=[[] for _ in range(6)]
    for cell in sorted({r['cell_id'] for r in rows}):
        batch=[copy.deepcopy(r) for r in rows if r['cell_id']==cell];rng.shuffle(batch);assert len(batch)%6==0
        for k,r in enumerate(batch):shards[k%6].append(r)
    base=yaml.safe_load((T/'branch-gate3/fixed-home.yaml').read_text());allrows=[]
    for k,batch in enumerate(shards):
        rng.shuffle(batch)
        for i,r in enumerate(batch):r.update(sample_id=f'W4-{label}-{k+1:02d}-{i+1:03d}',shard_id=k+1,shot_id=i+1,aim=[2.055,-.7625]);allrows.append(r)
        scene=copy.deepcopy(base);scene.update(description=f'Best v2 {label}: outcome-independent fixed trajectories.',balls=len(batch),clean_serves=[r['launch'] for r in batch[:6]],rapid_cycle=[r['launch'] for r in batch[6:]],spin_rad_s_list=[r['initial_spin'] for r in batch]);scene['timing'].update(flight_s_list=[max(1.5,r['flight_s']+.95) for r in batch],pause_s_list=[.8]*6+[0.]*(len(batch)-6));scene['timing']['flight_s_list'][-1]=3.
        (out/f'shard-{k+1:02d}.yaml').write_text(yaml.safe_dump(scene));(out/f'shard-{k+1:02d}-ledger.json').write_text(json.dumps(batch,indent=2))
    (out/'samples.json').write_text(json.dumps(allrows,indent=2));(out/'attempts.json').write_text(json.dumps(attempts,indent=2))
    (out/'design.json').write_text(json.dumps({'label':label,'total':len(rows),'cells':81,'samples_per_shard':len(shards[0]),'scope':'Train reuses earlier training trajectories and adds high-fast/high-middle/middle-fast cases. Holdout is new balanced 81 x 6, frozen before fitting. No outcome-dependent rejection.'},indent=2))
    (out/'SHA256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()},indent=2))

if __name__=='__main__':
    start=time.time();cells=list(itertools.product(range(3),repeat=4));jobs=[]
    for c in cells:
        _,speed,height,_=c
        n=24 if (height,speed)==(2,2) else 6 if (height,speed) in [(2,1),(1,2)] else 0
        if n:jobs.append((c,n,6000))
    extra=[];attempts=[]
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
        for ok,tried in pool.map(cell_job,jobs):extra.extend(ok);attempts.extend(tried);print('TRAIN_EXTRA',len(extra),round(time.time()-start,1),flush=True)
    oldtrain=json.loads((T/'weight-search-v3/data-train/samples.json').read_text());oldsets=oldtrain+json.loads((T/'weight-search-v3/data-holdout/samples.json').read_text())+json.loads((T.parent/'outputs/super-gate3-v1/samples.json').read_text())
    keys=lambda rows:{tuple(r['launch']+r['initial_spin']) for r in rows}
    assert len(extra)==324 and not keys(extra)&keys(oldsets)
    for r in extra:r['section']='hard_extra'
    freeze('train',oldtrain+extra,attempts)
    held=[];attempts=[]
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
        for ok,tried in pool.map(cell_job,[(c,6,8000) for c in cells]):held.extend(ok);attempts.extend(tried)
    assert len(held)==486 and not keys(held)&keys(oldsets+extra)
    freeze('holdout',held,attempts)
    (W/'data-isolation.json').write_text(json.dumps({'new_training_samples':324,'reused_training_samples':486,'train_total':810,'fresh_holdout_samples':486,'holdout_disjoint_from_all_previous_main_train_holdout_and_new_train':True,'construction_precedes_policy_outcomes':True,'seconds':time.time()-start},indent=2))
    print('FROZEN_DATA_COMPLETE',time.time()-start,flush=True)
