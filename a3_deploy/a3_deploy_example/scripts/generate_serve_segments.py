#!/usr/bin/env python3
"""Export three annotated command segments; no hardware or network access.

The recipe records the native actor's stable physical ready pose and supporting
impedance, not the actor's raw q_des (which includes gravity-support offsets).
Legacy motion values remain unchanged; recipes may label the branch frame event.
The native validator must
accept the result and the MuJoCo loop must pass before using a new recipe.
"""
from pathlib import Path
import numpy as np,csv,hashlib,json
import argparse
parser = argparse.ArgumentParser(description="Regenerate an annotated 100 Hz serve bundle from its simulation-qualified recipe.")
parser.add_argument('--recipe', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
recipe = json.loads(args.recipe.read_text())
base = args.recipe.parent / recipe['source_csv']
assert hashlib.sha256(base.read_bytes()).hexdigest() == recipe['source_sha256'], 'source CSV hash mismatch'
rows=list(csv.reader(base.open()));names=[x.split('::')[1] for x in rows[0][8:39]]
f=np.array([[float(x) for x in r[8:]] for r in rows[6:]])
stand=np.array(recipe['stand_command']);source=np.array(recipe['reference_branch_command']);target=np.array(recipe['policy_ready_command'])
branch=recipe['branch_frame'];duration=recipe['move_s'];readyframe=round(duration*100)
frames=[]
for t in np.arange(readyframe+1)*.01:
 u=t/duration;h=1-10*u**3+15*u**4-6*u**5;dh=(-30*u*u+60*u**3-30*u**4)/duration
 uv=min(t/.15,1);v=.15*(uv-6*uv**3+8*uv**4-3*uv**5);dv=1-18*uv**2+32*uv**3-15*uv**4
 c=target.copy();c[0]=h*source[0]+(1-h)*target[0]+v*source[1];c[1]=dh*(source[0]-target[0])+dv*source[1]
 z=np.clip((t-(duration-.2))/.2,0,1);g=10*z**3-15*z**4+6*z**5
 c[3:]=source[3:]*(1-g)+target[3:]*g;c[2]=target[2]*g;frames.append(c)
frames[0]=source.copy();frames[-1]=target.copy()
p={'commands':np.concatenate([frames,np.repeat(np.array(frames[-1:]),round(recipe['hold_s']*100),axis=0)]),'source':source}
policy=p['commands'].copy();deltaq=p['source'][0]-f[branch,:31];deltav=p['source'][1]-f[branch,31:62]
def smooth(u):
 u=np.clip(u,0,1);return 10*u**3-15*u**4+6*u**5
def ds(u):
 u=np.clip(u,0,1);return 30*u**2-60*u**3+30*u**4
for idx,c in enumerate(policy):
 t=idx*.01;u=np.clip(t/.15,0,1);v=.15*(u-6*u**3+8*u**4-3*u**5);dv=1-18*u**2+32*u**3-15*u**4
 c[0]-=(1-smooth(t/duration))*deltaq+v*deltav
 c[1]-=-ds(t/duration)/duration*deltaq+dv*deltav
policy[0,0]=f[branch,:31];policy[0,1]=f[branch,31:62]
prepare=[]
for idx in range(501):
 u=idx/500;delta=f[0,:31]-stand[0];c=stand.copy();c[0]=stand[0]+smooth(u)*delta;c[1]=ds(u)/5*delta
 for j in [5,6,8,12,13,14,15,16,17,18]:
  elbow=j in (8,15);racket=j>=12;rotation=j==14 or j>=16
  start=(.52 if rotation else .40) if racket else (0 if elbow else .15)
  span=1-start if racket else (.65 if elbow else .85)
  c[0,j]+=(smooth((u-start)/span)-smooth(u))*delta[j]
  c[1,j]+=(ds((u-start)/span)/span/5-ds(u)/5)*delta[j]
 c[0,[23,29]]+=.015*64*u**3*(1-u)**3
 c[1,[23,29]]+=.015*192*u**2*(1-u)**2*(1-2*u)/5
 prepare.append(c)
recover=[]
for row in f[branch:]:
 c=stand.copy();c[0]=row[:31];c[1]=row[31:62];recover.append(c)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
path=args.output
with path.open('w', newline='') as stream:
 legacy=base.read_bytes().decode('utf-8')
 branch_event=recipe.get('branch_event')
 if branch_event:
  assert branch_event == 'SERVE_TO_POLICY', 'unknown branch event'
  lines=legacy.splitlines(keepends=True);parts=lines[6+branch].split(',',3)
  assert parts[2] == 'NONE', 'branch must not replace another serve event'
  parts[2]=branch_event;lines[6+branch]=','.join(parts);legacy=''.join(lines)
 stream.write(legacy)
 w=csv.writer(stream,lineterminator='\n')
 marker=['#HOPE_SERVE_SEGMENTS_V1',branch,recipe['policy_blend_s'],recipe['policy_sha256'],recipe['deploy_sha256']]
 if branch_event: marker.append('frame_event=SERVE_TO_POLICY')
 w.writerow(marker)
 w.writerow(['segment','frame_index','time_s','event']+[f'{prefix}::{name}' for prefix in ['q_rad','qd_rad_s','tau_ff_nm','kp','kd'] for name in names])
 for label,clip in [('STAND_TO_SERVE',prepare),('SERVE_TO_STAND',recover),('SERVE_TO_POLICY',policy)]:
  for idx,c in enumerate(clip):
   event='BEGIN' if idx==0 else ('END' if idx==len(clip)-1 else ('POLICY_READY' if label=='SERVE_TO_POLICY' and idx==readyframe else 'SAMPLE'))
   w.writerow([label,idx,f'{idx*.01:.2f}',event]+[f'{x:.12g}' for x in c.flatten()])
print(path,sha(path))
