from pathlib import Path
import json,yaml,sys
import numpy as np
sys.modules['coverage']=None
from numba import njit
from scipy.optimize import least_squares
W=Path(__file__).resolve().parent
@njit
def step(s,w,dt):
 def fun(s):
  v=s[3:];a=-.12453915600992894*np.sqrt((v*v).sum())*v+.00399279130564425*np.cross(w,v);a[2]-=9.81;return np.concatenate((v,a))
 a=fun(s);b=fun(s+.5*dt*a);c=fun(s+.5*dt*b);d=fun(s+dt*c);return s+dt*(a+2*b+2*c+d)/6
@njit
def flight(q):
 s=np.array([2.4,0.,q[2],q[0],0.,q[1]]);w=np.zeros(3);bounces=0;bounce_x=-10.;net_z=-10.;t=0.
 for i in range(4000):
  prev=s.copy();s=step(s,w,.001);t+=.001
  if prev[0]>=1.37 and s[0]<1.37:net_z=s[2]
  if s[2]<=.0201 and prev[2]>.0201 and s[5]<0 and 0<=s[0]<=2.74:
   bounces+=1;bounce_x=s[0];v=s[3:];slip=v+np.cross(w,np.array([0.,0.,-.02]));slip[2]=0.;n=np.sqrt((slip*slip).sum());dv=-min(.41642144985036184*n,.218520906499864*(1+.9373319241133231)*abs(v[2]))*slip/(n+1e-12);s[3:]+=dv;w-=np.cross(np.array([0.,0.,1.]),dv)/((2/3)*.02);s[5]=-s[5]*.9373319241133231;s[2]=.0201
  if s[0]<=.07:
   a=(prev[0]-.07)/(prev[0]-s[0]);s=prev+a*(s-prev);return s,w,t,bounces,bounce_x,net_z
  if s[2]<-1:break
 return s,w,t,bounces,bounce_x,net_z
rows=[]
for vb,sp in enumerate([2.3,3.25,4.5]):
 for hb,height in enumerate([.9,1.1,1.3]):
  def residual(q):
   s,w,t,b,bx,nz=flight(q);return np.array([(s[2]+.76-height)*4,(np.linalg.norm(s[3:])-sp), max(0,bx-1.30)*2+max(0,.15-bx)*2, max(0,.19-nz)*4,abs(b-1)*2,abs(s[0]-.07)*2])
  opts=[]
  for seed in [[-3,2.2,.49],[-4,3,.5],[-5,2,1],[-2,4,.5]]:
   r=least_squares(residual,seed,bounds=([-9,-3,.1],[-1,8,2.5]),diff_step=.02,max_nfev=250);opts.append((np.linalg.norm(residual(r.x)),r.x))
  error,q=min(opts,key=lambda x:x[0]);s,w,t,b,bx,nz=flight(q);assert b==1 and nz>.18 and abs(s[2]+.76-height)<.035 and abs(np.linalg.norm(s[3:])-sp)<.12,(vb,hb,error,q,s,b,nz)
  for side,y in [('FH',-1.2225),('BHfront',-.7425),('BHother',-.5225)]:
   rows.append({'side':side,'speed_bin':vb,'height_bin':hb,'launch':[2.4,y,float(q[2]),float(q[0]),0.,float(q[1])],'predicted_contact':s.tolist(),'incoming_spin':w.tolist(),'speed':float(np.linalg.norm(s[3:])),'height':float(s[2]+.76),'flight_s':t,'bounce_x':bx,'net_height':nz+.76})
# 27 unique cells plus three repeating boundary cases, all continuous same HOME.
rows+=rows[:3]
scene=yaml.safe_load((W/'branch-gate3/fixed-home.yaml').read_text());scene.update(description='PPQ004 speed/height coverage audit; 27 side x speed x height cells plus 3 repeats. No exact-bank tuple injection.',balls=30,clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]])
scene['timing'].update(flight_s_list=[max(1.15,r['flight_s']+.75) for r in rows],pause_s_list=[1.]*6+[0.]*24);scene['timing']['flight_s_list'][-1]=3.
(W/'expanded-physical.yaml').write_text(yaml.safe_dump(scene));(W/'expanded-physical-ledger.json').write_text(json.dumps(rows,indent=2));print([(r['speed_bin'],r['height_bin'],round(r['speed'],3),round(r['height'],3)) for r in rows[::3]],flush=True)
