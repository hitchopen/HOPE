"""Fit capped table slip impulses from observed pre/post contact flight states."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks
from scipy.optimize import least_squares,minimize_scalar
from refit_c3d_recording import kernel,fit,dump,segments
from ballcore import tracked_runs,spin_from_quats
OLD=np.array([.9313775047019153,.38744443855065164,.16706204822993975])

def propagate(state,w,par,dt):
 p=state[:3].copy();v=state[3:].copy();steps=max(1,int(abs(dt)*1200));h=dt/steps
 def acc(v):return np.array([0,0,-9.81])-par[0]*np.linalg.norm(v)*v+par[1]*np.cross(w,v)
 for _ in range(steps):
  a=acc(v);v2=v+h/2*a;b=acc(v2);v3=v+h/2*b;c=acc(v3);v4=v+h*c;e=acc(v4)
  p+=h/6*(v+2*v2+2*v3+v4);v+=h/6*(a+2*b+2*c+e)
 return p,v

def predict(params,vin,win):
 ev,gain,mu=params;slip=vin[:,:2]+.02*np.column_stack((-win[:,1],win[:,0]));s=np.linalg.norm(slip,axis=1)
 j=np.minimum(gain*s,mu*(1+ev)*abs(vin[:,2]));dv=-slip*(j/np.maximum(s,1e-9))[:,None]
 vo=vin.copy();vo[:,:2]+=dv;vo[:,2]=-ev*vin[:,2]
 wo=win.copy();wo[:,0]+=dv[:,1]/(.02*2/3);wo[:,1]-=dv[:,0]/(.02*2/3)
 return vo,wo,j<gain*s-1e-7

def fit_contact(rows):
 vi=np.array([b['vin'] for b in rows]);vo=np.array([b['vout'] for b in rows]);wi=np.array([b['win'] for b in rows]);wo=np.array([b['wout'] for b in rows])
 def residual(x):
  v,w,_=predict(x,vi,wi)
  return np.c_[v-vo,(w[:,:2]-wo[:,:2])*.02*2/3].ravel()
 sol=least_squares(residual,OLD,bounds=([.3,0,0],[1.2,1,1]),loss='huber',f_scale=.15,max_nfev=400)
 return sol.x,{'success':bool(sol.success),'cost':sol.cost,'optimality':sol.optimality,'jac_singular_values':np.linalg.svd(sol.jac,compute_uv=False),'n':len(rows),'capped_count':int(predict(sol.x,vi,wi)[2].sum())}

def evaluate(rows,params):
 vi=np.array([b['vin'] for b in rows]);vo=np.array([b['vout'] for b in rows]);wi=np.array([b['win'] for b in rows]);wo=np.array([b['wout'] for b in rows]);v,w,c=predict(params,vi,wi)
 return {'n':len(rows),'velocity_error_mps_median_p90':np.percentile(np.linalg.norm(v-vo,axis=1),[50,90]),'spin_error_rad_s_median_p90':np.percentile(np.linalg.norm(w-wo,axis=1),[50,90]),'capped_count':int(c.sum())}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--flight-fit',default='all_fit.json');ap.add_argument('--suffix',default='');ap.add_argument('--baseline',type=Path);args=ap.parse_args();o=args.output;baseline=json.loads(args.baseline.read_text()) if args.baseline else {};previous=np.array([baseline.get('restitution_v',OLD[0]),baseline.get('table_tangential_gain',OLD[1]),baseline.get('table_friction_cap_mu',OLD[2])])
 d=dict(np.load(o/'raw_extracted.npz'));integrate=kernel(o);par=np.array(json.loads((o/args.flight_fit).read_text())['params']);_,take=segments(d,o)
 t=d['t'];p=d['ball_pos'];q=d['ball_quat'];rows=[];rejects={};candidates=0
 def reject(x):rejects[x]=rejects.get(x,0)+1
 for start,end in tracked_runs(t,take['ball_present'],300,min_len=70):
  peaks,_=find_peaks(-p[start:end+1,2],prominence=.06,distance=25)
  for i in peaks+start:
   candidates+=1
   if not (.04<p[i,0]<2.70 and -1.49<p[i,1]<-.035 and -.015<p[i,2]<.065):reject('outside_table_contact');continue
   pre=np.arange(i-36,i-4);post=np.arange(i+5,i+37)
   if pre[0]<start or post[-1]>end:reject('incomplete_window');continue
   arcs=[];good=True
   for ix in [pre,post]:
    w,ang,rob=spin_from_quats(q[ix],300);coh=np.linalg.norm(np.sum(w,axis=0))/max(np.sum(np.linalg.norm(w,axis=1)),1e-9)
    if not np.isfinite(rob).all() or np.max(ang)>90 or (np.linalg.norm(rob)>5 and coh<.65):good=False;break
    arcs.append({'t':t[ix],'pos':p[ix],'quat':q[ix],'w':rob})
   if not good:reject('spin_quality');continue
   f=fit(arcs,integrate,par,False)
   if np.max(f['rms'])>.007:reject('flight_residual');continue
   def states(tc):return [propagate(f['states'][k],a['w'],par,tc-a['t'][0]) for k,a in enumerate(arcs)]
   def mismatch(tc):
    x=states(tc);return np.linalg.norm(x[0][0]-x[1][0])**2
   sol=minimize_scalar(mismatch,bounds=(t[i]-.013,t[i]+.013),method='bounded',options={'xatol':1e-8});x=states(sol.x);pc=(x[0][0]+x[1][0])/2
   if np.sqrt(sol.fun)>.018:reject('contact_continuity');continue
   if abs(pc[2]-.02)>.025:reject('contact_height');continue
   vi,vo=x[0][1],x[1][1]
   if not (vi[2]<-.5 and vo[2]>.3 and max(np.linalg.norm(vi),np.linalg.norm(vo))<18):reject('velocity_direction');continue
   rows.append({'time':sol.x,'frame':int(i),'contact_xyz':pc,'continuity_mm':np.sqrt(sol.fun)*1000,'flight_rms_mm':f['rms']*1000,'vin':vi,'vout':vo,'win':arcs[0]['w'],'wout':arcs[1]['w'],'observed_normal_restitution':-vo[2]/vi[2]})
 print('CONTACTS',len(rows),'candidates',candidates,'rejects',rejects,flush=True)
 dump(o/('contacts'+args.suffix+'.json'),{'candidates':candidates,'rejected':rejects,'accepted':rows})
 train=[r for r in rows if int(r['time']//20)%3!=2];test=[r for r in rows if int(r['time']//20)%3==2]
 assert len(train)>=8 and len(test)>=4,(len(train),len(test))
 trainpar,traininfo=fit_contact(train);allpar,allinfo=fit_contact(rows);print('TRAIN',trainpar,traininfo,'ALL',allpar,allinfo,flush=True)
 rng=np.random.default_rng(1015);blocks=sorted(set(int(r['time']//20) for r in rows));boots=[]
 for _ in range(150):
  chosen=rng.choice(blocks,len(blocks),replace=True);sub=[r for b in chosen for r in rows if int(r['time']//20)==b];boots.append(fit_contact(sub)[0])
 result={'flight_state_parameters_from':args.flight_fit,'parameters':allpar,'parameter_order':['restitution_v','table_tangential_gain','table_friction_cap_mu'],'all_fit':allinfo,'train_parameters':trainpar,'train_fit':traininfo,'holdout_old':evaluate(test,previous),'holdout_new':evaluate(test,trainpar),'bootstrap_percentile_95':np.percentile(boots,[2.5,97.5],axis=0),'bootstrap_method':'20-second temporal block resampling, 150 repeats','normal_restitution_distribution':np.percentile([r['observed_normal_restitution'] for r in rows],[10,50,90]),'contact_height_distribution':np.percentile([r['contact_xyz'][2] for r in rows],[10,50,90])}
 dump(o/('contact_fit'+args.suffix+'.json'),result);print(result,flush=True)
if __name__=='__main__':main()
