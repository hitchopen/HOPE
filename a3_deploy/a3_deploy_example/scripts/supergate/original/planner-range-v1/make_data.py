from pathlib import Path
import sys,json,itertools,concurrent.futures,hashlib,time,copy
import numpy as np,yaml
W=Path(__file__).resolve().parent;T=W.parent
sys.path.insert(0,str(T));sys.path.append(str(T/'numerical-tools'))
from solve_outer import solve

def cell_job(spec):
    cell,n,offset=spec;ok=[];attempts=[]
    for variant in range(offset,offset+24):
        r=solve((*cell,variant));attempts.append(r)
        if r['feasible']:ok.append(r)
        if len(ok)==n:break
    # Unfilled physical cells are retained as coverage holes.
    if len(ok)!=n:ok=[]
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
    (out/'design.json').write_text(json.dumps({'label':label,'total':len(rows),'cells':81,'samples_per_shard':len(shards[0]),'scope':'Expansion evaluation: three outer lateral bands [-.84,-.70], [.30,.44], [.46,.59], three heights, speeds and arrival angles, six independent samples/cell. Physical feasibility only before outcomes; all failures retained.'},indent=2))
    (out/'SHA256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()},indent=2))

if __name__=='__main__':
    start=time.time();cells=list(itertools.product(range(3),repeat=4));rows=[];attempts=[]
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
        for ok,tried in pool.map(cell_job,[(c,6,10000) for c in cells]):
            rows.extend(ok);attempts.extend(tried);print('OUTER_GENERATED',len(rows),'attempts',len(attempts),round(time.time()-start,1),flush=True)
    assert rows, 'No physically feasible outer cells'
    freeze('outer',rows,attempts)
    filled={r['cell_id'] for r in rows}
    (W/'coverage-holes.json').write_text(json.dumps({'filled_cells':len(filled),'physical_generation_unfilled_cells':[i for i in range(81) if i not in filled],'minimum_samples_each_retained_cell':6,'reason':'Some far-lateral/incoming-angle combinations cannot cross the net inside table width; all attempted trajectories retained, no policy outcomes used.'},indent=2))
    print('FROZEN_OUTER',len(rows),len(attempts),time.time()-start,flush=True)
