from pathlib import Path
import subprocess,json,sys,hashlib
W=Path(__file__).resolve().parent;V='/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python'
for threads in [1,8]:
 out=W.parent/f'internal-batch-parity-{threads}'
 if out.exists():raise RuntimeError(out)
 with (W.parent/f'internal-batch-parity-{threads}.log').open('w') as f:
  r=subprocess.run(['distrobox','enter','hope','--',V,str(W/'run.py'),'2',str(threads),str(out)],stdout=f,stderr=subprocess.STDOUT)
 if r.returncode:raise RuntimeError('parity run failed '+str(threads))
 print('PARITY_RUN_COMPLETE',threads,flush=True)
a=W.parent/'internal-batch-parity-1';b=W.parent/'internal-batch-parity-8';checks={}
for p in sorted(a.glob('env-*.csv')):checks[p.name]=p.read_bytes()==(b/p.name).read_bytes()
assert len(checks)==24 and all(checks.values()),checks
ra=json.loads((a/'run.json').read_text());rb=json.loads((b/'run.json').read_text());assert ra['library_sha256']==rb['library_sha256'];report={'all_csv_bit_identical':True,'files':checks,'single_thread_seconds':ra['seconds'],'eight_thread_seconds':rb['seconds'],'speedup':ra['seconds']/rb['seconds'],'trials':24,'library_sha256':ra['library_sha256'],'scope':'Full native actor/Planner/physical batch smoke test. 12 independent worlds; 2 distinct shots per world, both original policies. Not hardware or DDS qualification.'};(W.parent/'internal-batch-parity.json').write_text(json.dumps(report,indent=2));print('PARITY_PASS',json.dumps(report),flush=True)
out=W.parent/'super-internal-native'
with (W.parent/'super-internal-native.log').open('w') as f:
 r=subprocess.run(['distrobox','enter','hope','--',V,str(W/'run.py'),'200','8',str(out)],stdout=f,stderr=subprocess.STDOUT)
print('FULL_RUN_EXIT',r.returncode,flush=True)
