from pathlib import Path
import subprocess
W=Path(__file__).resolve().parent;D=W.parent.parent/'outputs/super-gate3-slow-tail'
V='/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python'
for names,out in [(None,'slow-internal-native'),(['phase1.0_0.0','dual_edit_fh14200_bh13800'],'slow-internal-fusions'),(['14200_bank_1_candidate'],'slow-internal-edited')]:
 cmd=['distrobox','enter','hope','--',V,str(W/'run.py'),'9','8',str(W.parent/out),'--dataset',str(D)]
 if names:
  cmd+=['--policy0',str(W.parent/'models'/names[0]),'--label0',names[0]]
  if len(names)==2:cmd+=['--policy1',str(W.parent/'models'/names[1]),'--label1',names[1]]
  else:cmd+=['--single-model']
 with (W.parent/(out+'.log')).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 print(out,r.returncode,flush=True)
 if r.returncode:raise SystemExit(r.returncode)
