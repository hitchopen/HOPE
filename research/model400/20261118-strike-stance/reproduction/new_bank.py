from pathlib import Path
import sys,json,hashlib,itertools,concurrent.futures,time
import numpy as np,yaml
W=Path(__file__).resolve().parent;OLD=Path('/home/user/Documents/Codex/2026-11-08/model-u-fusion-13800-gate3-x20-2/work');sys.path.insert(0,str(OLD));sys.path.insert(0,str(OLD/'numerical-tools'));from build_super_factorial import solve

def job(cell):
 ok=[];attempts=[]
 for variant in range(58000,58300):
  r=solve((*cell,variant));attempts.append(r)
  if r['feasible']:ok.append(r)
  if len(ok)==6:break
 return cell,ok,attempts
if __name__=='__main__':
 D=W/'fresh-holdout';D.mkdir(exist_ok=False);selected=[];attempts=[];start=time.time()
 with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
  for cell,ok,tried in pool.map(job,itertools.product(range(3),repeat=4)):
   assert len(ok)==6,(cell,len(ok));selected.extend(ok);attempts.extend(tried);print('cells',len(selected)//6,'seconds',round(time.time()-start),flush=True)
 rng=np.random.default_rng(2026111829);shards=[[] for _ in range(6)]
 for cell in range(81):
  rows=[r for r in selected if r['cell_id']==cell];rng.shuffle(rows)
  for i,r in enumerate(rows):shards[i].append(dict(r))
 base=yaml.safe_load((OLD/'branch-gate3/fixed-home.yaml').read_text());allrows=[]
 for i,rows in enumerate(shards):
  rng.shuffle(rows)
  for j,r in enumerate(rows):r.update(sample_id=f'400-stance-heldout-{i+1:02d}-{j+1:03d}',shard_id=i+1,shot_id=j+1,aim=[2.055,-.7625]);allrows.append(r)
  scene=dict(base);scene.update(description='Frozen new holdout for model400 native closed-loop weight optimization; no outcome filtering',balls=81,clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]],spin_rad_s_list=[r['initial_spin'] for r in rows]);scene['timing']=dict(base['timing']);scene['timing'].update(flight_s_list=[max(1.5,r['flight_s']+.95) for r in rows],pause_s_list=[.8]*6+[0.]*75);scene['timing']['flight_s_list'][-1]=3.
  (D/f'shard-{i+1:02d}.yaml').write_text(yaml.safe_dump(scene));(D/f'shard-{i+1:02d}-ledger.json').write_text(json.dumps(rows,indent=2))
 keys=lambda rr:{tuple(r['launch']+r['initial_spin']) for r in rr}
 prior=[OLD/'weight-search-v3/data-train/samples.json',OLD/'weight-search-v3/data-holdout/samples.json',OLD.parent/'outputs/super-gate3-v1/samples.json',W.parent/'supergate-opt400-v2/fresh-holdout/samples.json',W.parent/'supergate-opt400-v3/fresh-holdout/samples.json']
 prior += [W.parent/'supergate-opt400-v4'/bank/'samples.json' for bank in ['fresh-holdout','validation-extra1','validation-extra2']]
 prior += [W.parent/'supergate-opt400-v5'/bank/'samples.json' for bank in ['fresh-holdout','validation-extra1','validation-extra2']]
 assert all(not(keys(allrows)&keys(json.loads(f.read_text()))) for f in prior)
 (D/'samples.json').write_text(json.dumps(allrows,indent=2));(D/'attempts.json').write_text(json.dumps(attempts,indent=2));(D/'design.json').write_text(json.dumps({'split':'fresh-heldout','seed':2026111829,'variant_start':58000,'cells':81,'n':486,'prior_launches_disjoint':True,'policy_outcome_filtering':False,'reserved_until_selection_freeze':True},indent=2));(D/'SHA256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in D.iterdir() if f.is_file()},indent=2));print('FROZEN_FRESH_486',flush=True)
