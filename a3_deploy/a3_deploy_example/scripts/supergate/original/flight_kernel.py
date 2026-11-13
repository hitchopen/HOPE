import sys
sys.modules['coverage']=None
import numpy as np
from numba import njit
@njit(cache=True)
def step(s,w,dt):
 v=s[3:];a=-.12453915600992894*np.sqrt((v*v).sum())*v+.00399279130564425*np.cross(w,v);a[2]-=9.81
 vb=v+.5*dt*a;b=-.12453915600992894*np.sqrt((vb*vb).sum())*vb+.00399279130564425*np.cross(w,vb);b[2]-=9.81
 vc=v+.5*dt*b;c=-.12453915600992894*np.sqrt((vc*vc).sum())*vc+.00399279130564425*np.cross(w,vc);c[2]-=9.81
 vd=v+dt*c;d=-.12453915600992894*np.sqrt((vd*vd).sum())*vd+.00399279130564425*np.cross(w,vd);d[2]-=9.81
 out=s.copy();out[:3]+=dt*(v+2*vb+2*vc+vd)/6;out[3:]+=dt*(a+2*b+2*c+d)/6;return out
@njit(cache=True)
def flight(initial,spin,plane=.07):
 s=initial.copy();w=spin.copy();bounces=0;bounce=np.array([-10.,-10.]);net=np.array([-10.,-10.]);t=0.
 for i in range(5000):
  prev=s.copy();s=step(s,w,.001);t+=.001
  if prev[0]>=1.37>s[0]:
   f=(prev[0]-1.37)/(prev[0]-s[0]);net[:]=prev[1:3]+f*(s[1:3]-prev[1:3])
  if s[2]<=.0201 and prev[2]>.0201 and s[5]<0 and 0<=s[0]<=2.74 and -1.525<=s[1]<=0:
   bounces+=1;bounce[:]=s[:2];v=s[3:];u=v+np.cross(w,np.array([0.,0.,-.02]));u[2]=0;n=np.sqrt((u*u).sum());dv=-min(.41642144985036184*n,.218520906499864*(1+.9373319241133231)*abs(v[2]))*u/(n+1e-12);s[3:]+=dv;w-=np.cross(np.array([0.,0.,1.]),dv)/((2/3)*.02);s[5]=-s[5]*.9373319241133231;s[2]=.0201
  if s[0]<=plane:
   f=(prev[0]-plane)/(prev[0]-s[0]);s=prev+f*(s-prev);return s,w,t,bounces,bounce,net
  if s[2]<-.8:break
 return s,w,t,bounces,bounce,net
