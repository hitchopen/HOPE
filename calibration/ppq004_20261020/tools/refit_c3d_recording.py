"""Single-recording calibration: rigid-marker spin, RK4 shooting, temporal holdout.
Run in the isolated ballfit environment; no ROS or hardware actions.
Inputs are inspected raw_extracted.npz and raw_manifest.json; raw extraction is
retained alongside the result. No legacy table-centre transform is applied.
"""
import argparse,ctypes,json,sys,subprocess
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation
from ballcore import extract_arcs,arc_parabola,spin_from_quats
ROOT=Path(__file__).resolve().parent
OLD=np.array([.12861635416914574,.00379260504480258])

def dump(p,x):
 p.write_text(json.dumps(x,indent=2,default=lambda o:o.tolist() if isinstance(o,np.ndarray) else o.item() if isinstance(o,np.generic) else str(o),allow_nan=False))

def kernel(out):
 lib=out/'shooting_kernel.so'
 subprocess.run(['g++','-O3','-shared','-fPIC',str(ROOT/'shooting_kernel.cpp'),'-o',str(lib)],check=True)
 f=ctypes.CDLL(str(lib)).shoot
 arr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
 ints=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
 f.argtypes=[ctypes.c_int,ints,arr,arr,ctypes.c_double,ctypes.c_double,ctypes.c_double,arr]
 f.restype=None
 def integrate(states,ws,lens,params,dt):
  lens=np.ascontiguousarray(lens,dtype=np.int32);out=np.empty((sum(lens),3))
  f(len(lens),lens,np.ascontiguousarray(states),np.ascontiguousarray(ws),*params,dt,out)
  return out
 return integrate

def fit(arcs,integrate,params=OLD,free=True,max_nfev=600):
 n=len(arcs);lens=np.array([len(a['t']) for a in arcs]);dt=arcs[0]['t'][1]-arcs[0]['t'][0]
 obs=np.vstack([a['pos'] for a in arcs]);ws=np.array([a['w'] for a in arcs])
 states=np.array([np.r_[a['pos'][0],arc_parabola(a)['v0']] for a in arcs])
 x=np.r_[states.ravel(),params] if free else states.ravel()
 def residual(x):return (integrate(x[:6*n].reshape(n,6),ws,lens,x[-2:] if free else params,dt)-obs).ravel()
 S=lil_matrix((obs.size,len(x)),dtype=int);k=0
 for i,l in enumerate(lens):S[k:k+3*l,6*i:6*i+6]=1;k+=3*l
 if free:S[:,-2:]=1
 lo=np.full(len(x),-np.inf);hi=-lo
 if free:lo[-2:]=[0,0];hi[-2:]=[.6,.03]
 sol=least_squares(residual,x,bounds=(lo,hi),jac_sparsity=S.tocsr(),loss='huber',f_scale=.002,x_scale='jac',max_nfev=max_nfev,ftol=1e-6,gtol=1e-6,xtol=1e-7,
                   tr_options={'atol':1e-10,'btol':1e-10,'maxiter':400})
 rr=residual(sol.x).reshape(-1,3);k=0;errs=[]
 for l in lens:errs.append(np.sqrt(np.mean(np.sum(rr[k:k+l]**2,axis=1))));k+=l
 return {'params':sol.x[-2:] if free else np.array(params),'states':sol.x[:6*n].reshape(n,6),'rms':np.array(errs),'success':bool(sol.success),'nfev':sol.nfev,'cost':sol.cost,'optimality':sol.optimality,'termination':sol.message}

def segments(d,out):
 p=d['ball_pos'].copy();q=d['ball_quat'];t=d['t'];rate=float(d['rate'])
 valid=d['ball_present'].copy() & np.isfinite(p).all(1)
 valid &= (p[:,0]>-2)&(p[:,0]<5)&(p[:,1]>-3)&(p[:,1]<2)&(p[:,2]>-.05)&(p[:,2]<3)
 step=np.linalg.norm(np.diff(p,axis=0),axis=1);bad=(step>30/rate)&valid[1:]&valid[:-1]
 valid[1:] &= ~bad;valid[:-1] &= ~bad
 take={'name':out.name,'t':t,'rate':rate,'ball_pos_t_m':p,'ball_quat_xyzw':q,'ball_present':valid}
 raw=extract_arcs(take,min_frames=45,table_z=0)
 arcs=[];rows=[]
 for i,a in enumerate(raw):
  par=arc_parabola(a);w,ang,rob=spin_from_quats(a['quat'],rate)
  coh=np.linalg.norm(np.nansum(w,axis=0))/max(np.nansum(np.linalg.norm(w,axis=1)),1e-9)
  dur=a['t'][-1]-a['t'][0];speed=np.linalg.norm(par['v0']);rps=np.linalg.norm(rob)/(2*np.pi)
  reasons=[]
  if not .15<=dur<=.85:reasons.append('duration')
  if not 1.5<=speed<=18:reasons.append('speed')
  if not 3<=par['g']<=19:reasons.append('gravity_screen')
  if par['rms']>.012:reasons.append('position_discontinuity')
  if not np.isfinite(rob).all() or np.nanmax(ang)>90:reasons.append('spin_jump')
  if rps>.75 and coh<.65:reasons.append('spin_incoherent')
  row={'id':i,'t0':float(a['t'][0]),'t1':float(a['t'][-1]),'duration':dur,'speed':speed,'spin_rps':rps,'omega':rob if np.isfinite(rob).all() else None,'coherence':coh,'apparent_g':par['g'],'parabola_rms_mm':par['rms']*1000,'rejected':reasons}
  rows.append(row)
  if not reasons:
   a['w']=rob;a['id']=i;a['row']=row;arcs.append(a)
 dump(out/'segments.json',{'frame_count':len(t),'tracking_valid_count':int(d['ball_present'].sum()),'quality_valid_count':int(valid.sum()),'candidate_arcs':len(raw),'accepted_arcs':len(arcs),'arcs':rows})
 return arcs,take

def causal(arcs,integrate,params,window=.15):
 rows=[]
 for a in arcs:
  n=int(round(window*300))+1
  if len(a['t'])<n+15:continue
  b=dict(a,t=a['t'][:n],pos=a['pos'][:n],quat=a['quat'][:n])
  _,_,b['w']=spin_from_quats(b['quat'],300)
  if not np.isfinite(b['w']).all():continue
  fitted=fit([b],integrate,params,False)
  pred=integrate(fitted['states'],np.array([b['w']]),[len(a['t'])],params,1/300)
  err=np.linalg.norm(pred-a['pos'],axis=1)
  row={'id':a['id'],'duration':a['t'][-1]-a['t'][0],'endpoint_mm':err[-1]*1000,'endpoint_y_mm':abs(pred[-1,1]-a['pos'][-1,1])*1000,'endpoint_z_mm':abs(pred[-1,2]-a['pos'][-1,2])*1000,'future_rms_mm':np.sqrt(np.mean(err[n:]**2))*1000}
  # Observed net crossing is a measurement, never a model-generated label.
  ix=np.flatnonzero((a['pos'][:-1,0]-1.37)*(a['pos'][1:,0]-1.37)<0)
  ix=ix[ix>=n]
  ip=np.flatnonzero((pred[:-1,0]-1.37)*(pred[1:,0]-1.37)<0);ip=ip[ip>=n]
  if len(ix):
   row['observed_net_crossing']=True;row['predicted_net_crossing']=bool(len(ip))
   if len(ip):
    def crossing(pos,i):
     f=(1.37-pos[i,0])/(pos[i+1,0]-pos[i,0]);return (i+f)/300,pos[i]*(1-f)+pos[i+1]*f
    to,po=crossing(a['pos'],ix[0]);tp,pp=crossing(pred,ip[0]);row.update(net_time_ms=abs(tp-to)*1000,net_y_mm=abs(pp[1]-po[1])*1000,net_z_mm=abs(pp[2]-po[2])*1000)
  rows.append(row)
 return rows

def summary(rows):
 out={'count':len(rows)}
 for key in ['endpoint_mm','endpoint_y_mm','endpoint_z_mm','future_rms_mm','net_time_ms','net_y_mm','net_z_mm']:
  vals=[r[key] for r in rows if key in r]
  if vals:out[key]={'n':len(vals),'median':float(np.median(vals)),'p90':float(np.percentile(vals,90))}
 return out

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--bootstrap-only',action='store_true');ap.add_argument('--baseline',type=Path);args=ap.parse_args();out=args.output;baseline=json.loads(args.baseline.read_text()) if args.baseline else {};previous=np.array([baseline.get('drag_k',OLD[0]),baseline.get('magnus_k',OLD[1])]);d=dict(np.load(out/'raw_extracted.npz'));integrate=kernel(out)
 arcs,take=segments(d,out);print('arcs',len(arcs),flush=True)
 # Entire flights stay together; temporal blocks are fixed before fitting.
 train=[a for a in arcs if int(a['t'][0]//20)%3!=2 and int(a['t'][0]//20)==int(a['t'][-1]//20)]
 test=[a for a in arcs if int(a['t'][0]//20)%3==2 and int(a['t'][0]//20)==int(a['t'][-1]//20)]
 assert len(train)>=10 and len(test)>=5,(len(train),len(test))
 if args.bootstrap_only:
  allfit=json.loads((out/'all_fit.json').read_text())
 else:
  f=fit(train,integrate);print('TRAIN',f['params'],'rmsmm',np.percentile(f['rms']*1000,[50,90]),'success',f['success'],flush=True)
  # Initial-state nuisance parameters may use all frames only for descriptive fit;
  # causal validation below re-estimates them using the first 150 ms exclusively.
  dump(out/'training_fit.json',{'ids':[a['id'] for a in train],**f,'test_ids':[a['id'] for a in test]})
  results={}
  for name,par in [('previous',previous),('fitted',f['params']),('magnus_off',np.array([f['params'][0],0.]))]:
   rows=causal(test,integrate,par);results[name]={'summary':summary(rows),'rows':rows};print(name,results[name]['summary'],flush=True)
  dump(out/'heldout_validation.json',results)
  allfit=fit(arcs,integrate,params=f['params']);dump(out/'all_fit.json',{'ids':[a['id'] for a in arcs],**allfit});print('ALL',allfit['params'],'rmsmm',np.percentile(allfit['rms']*1000,[50,90]),flush=True)
 # Block bootstrap preserves correlated flights from the same rally period.
 blocks=sorted(set(int(a['t'][0]//20) for a in arcs));rng=np.random.default_rng(1015);boots=[];diagnostics=[]
 for i in range(24):
  selected=rng.choice(blocks,len(blocks),replace=True);sample=[a for b in selected for a in arcs if int(a['t'][0]//20)==b]
  b=fit(sample,integrate,params=allfit['params'],max_nfev=300);boots.append(b['params']);diagnostics.append({k:b[k] for k in ('success','nfev','optimality','termination')});print('bootstrap',i,b['params'],flush=True)
 dump(out/'uncertainty.json',{'method':'20-second block bootstrap, 24 resamples, single recording only','samples':boots,'optimizer_diagnostics':diagnostics,'percentile_95':np.percentile(boots,[2.5,97.5],axis=0)})
if __name__=='__main__':main()
