from pathlib import Path
import subprocess,time,json
W=Path(__file__).resolve().parent;T=W.parent;V='/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python'
while not (T/'detailed-slow/run.json').exists():time.sleep(2)
assert json.loads((T/'detailed-slow/run.json').read_text())['return_code']==0
for name,n,extra in [('detailed-candidates',200,[]),('detailed-candidates-slow',9,['--dataset',str(T.parent/'outputs/super-gate3-slow-tail')])]:
 cmd=['distrobox','enter','hope','--',V,str(W/'run.py'),str(n),'24',str(T/name),'--policy-groups',str(T/'new-weight-groups.json'),*extra]
 with (T/(name+'.log')).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 print(name,r.returncode,flush=True)
 if r.returncode:raise SystemExit(r.returncode)
print('FOLLOWUP_COMPLETE',flush=True)
