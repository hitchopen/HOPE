from pathlib import Path
import subprocess,json,statistics,hashlib
W=Path(__file__).resolve().parent;T=W.parent;V='/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python'
def run(name,n,threads,extra=[]):
 out=T/name
 with (T/(name+'.log')).open('w') as f:r=subprocess.run(['distrobox','enter','hope','--',V,str(W/'run.py'),str(n),str(threads),str(out),*extra],stdout=f,stderr=subprocess.STDOUT)
 if r.returncode:raise RuntimeError((name,r.returncode))
 receipt=json.loads((out/'run.json').read_text());print(name,receipt['seconds'],flush=True);return out,receipt
p,r=run('detailed-observer-parity',2,8)
checks={f.name:f.read_bytes()==(p/f.name).read_bytes() for f in (T/'internal-batch-parity-8').glob('*.csv')};assert len(checks)==24 and all(checks.values()),checks
(T/'detailed-observer-parity.json').write_text(json.dumps({'all_v1_ball_and_result_csv_bit_identical':True,'files':checks,'library_sha256':r['library_sha256']},indent=2));print('OBSERVER_PRESERVES_PHYSICS',flush=True)
bench=[];reference=None
for index,threads in enumerate([8,24,12,16,16,12,24,8]):
 p,r=run(f'thread-scaling-{index}-{threads}',8,threads,['--environments','24'])
 hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in p.glob('*.csv')}
 if reference is None:reference=hashes
 assert hashes==reference,(threads,'thread-dependent result')
 bench.append({'threads':threads,'seconds':r['seconds'],'environments':24,'physical_trials':192,'all_csv_bit_identical':True})
 (T/'thread-scaling.json').write_text(json.dumps({'runs':bench},indent=2))
medians={t:statistics.median(r['seconds'] for r in bench if r['threads']==t) for t in [8,12,16,24]};best=min(medians,key=medians.get)
(T/'thread-scaling.json').write_text(json.dumps({'runs':bench,'median_seconds':medians,'selected_threads':best,'selection_scope':'Two repeats, 24 environments, 8 shots/environment plus 20 s warmup. Hardware/load-specific; duplicated trajectories for scheduling only, not extra coverage.'},indent=2));print('THREADS_SELECTED',best,medians,flush=True)
extra=['--policy-groups',str(T/'detailed-policy-groups.json')]
p,r=run('detailed-main',200,best,extra)
old=[('super-internal-native',0),('super-internal-native',6),('super-internal-fusions',0),('super-internal-edited',0),('super-internal-fusions',6)]
checks={}
for group,(folder,off) in enumerate(old):
 for shard in range(6):
  for suffix in ['ball.csv','results.csv']:
   name=f'env-{group*6+shard}-{suffix}';checks[name]=(p/name).read_bytes()==(T/folder/f'env-{off+shard}-{suffix}').read_bytes()
assert len(checks)==60 and all(checks.values()),[k for k,v in checks.items() if not v]
(T/'detailed-full-parity.json').write_text(json.dumps({'all_6000_original_trials_unchanged':True,'files':checks},indent=2));print('FULL_PHYSICS_PARITY_PASS',flush=True)
run('detailed-slow',9,best,extra+['--dataset',str(T.parent/'outputs/super-gate3-slow-tail')])
print('DETAILED_RUNS_COMPLETE',flush=True)
