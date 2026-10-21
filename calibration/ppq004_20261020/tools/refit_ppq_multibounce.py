"""Offline PPQ multi-part audit: conservative net exclusion and independent z fits.
No ROS publication, deployment, or hardware control. Hold out the final file,
including entire flights and bounce sequences; file edges are excluded.
"""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import find_peaks, savgol_filter

from ballcore import tracked_runs, spin_from_quats
from refit_c3d_recording import segments, kernel, fit, causal, summary, dump
from refit_table_contact import propagate, fit_contact, evaluate, predict


def net_guard(p, valid):
    # Conservatively remove ALL low passages near net, not only known impacts.
    near = valid & (abs(p[:, 0]-1.37) < .08) & (p[:, 2] < .24) & (p[:, 2] > -.02) & (p[:, 1] > -1.65) & (p[:, 1] < .12)
    mask = np.convolve(near.astype(int), np.ones(13, int), mode='same') > 0
    return near, mask


def normal_fit(rows):
    vi=np.array([r['normal_vin'] for r in rows]);vo=np.array([r['normal_vout'] for r in rows])
    sol=least_squares(lambda x:vo+x[0]*vi,[.93],bounds=([.3],[1.2]),loss='huber',f_scale=.10)
    return {'restitution_v':float(sol.x[0]),'n':len(rows),'success':bool(sol.success)}


def normal_score(rows,e):
    err=np.array([abs(r['normal_vout']+e*r['normal_vin']) for r in rows])
    return {'n':len(rows),'vertical_velocity_error_mps_median_p90':np.percentile(err,[50,90])}


def position_contacts(d,part,offset,out):
    p=d['ball_pos'];t=d['t'];raw=d['ball_present'] & np.isfinite(p).all(1)
    valid=raw & (p[:,0]>-2)&(p[:,0]<5)&(p[:,1]>-3)&(p[:,1]<2)&(p[:,2]>-.05)&(p[:,2]<3)
    step=np.linalg.norm(np.diff(p,axis=0),axis=1);bad=(step>.10)&valid[1:]&valid[:-1];valid[1:]&=~bad;valid[:-1]&=~bad
    near,net=net_guard(p,valid)
    clean=dict(d);clean['ball_present']=valid & ~net
    # Edge guard prevents an individual trajectory spanning train/test file boundary.
    clean['ball_present'][:450]=False;clean['ball_present'][-450:]=False
    clean_dir=out/f'part{part}'/'screened';clean_dir.mkdir(exist_ok=True)
    arcs,_=segments(clean,clean_dir)
    for a in arcs:
        a['part']=part;a['original_id']=a['id'];a['id']=part*100000+a['id'];a['global_t0']=offset+float(a['t'][0]);a['row']['part']=part
    rows=[];ledger=[];seqs=[]
    for start,end in tracked_runs(t,valid,300,min_len=40):
        z=savgol_filter(p[start:end+1,2],5,2)
        ii,_=find_peaks(-z,prominence=.015,distance=18)
        candidates=ii+start
        # Lower-amplitude bounces may be too small to fit, but MUST still cut
        # neighbouring fit windows so a late hop is never blended into flight.
        guard_idx,_=find_peaks(-z,prominence=.003,distance=8)
        window_cuts=np.unique(np.r_[candidates,guard_idx+start])
        for j,i in enumerate(candidates):
            row={'part':part,'frame':int(i),'time':float(t[i]),'global_time':float(offset+t[i]),'observed_xyz':p[i].tolist()}
            reason=None
            if i<450 or i>len(t)-451:reason='file_edge'
            elif not (.04<p[i,0]<2.70 and -1.49<p[i,1]<-.035 and -.015<p[i,2]<.065):reason='outside_table_contact'
            before=window_cuts[window_cuts<i-3];after=window_cuts[window_cuts>i+3]
            left=max(start,int(before[-1])+5 if len(before) else start)
            right=min(end,int(after[0])-5 if len(after) else end)
            npre=min(36,i-left-4);npost=min(36,right-i-4)
            if reason is None and min(npre,npost)<15:reason='short_window'
            if reason is None and net[max(start,i-npre-4):min(end+1,i+npost+5)].any():reason='net_proximity'
            if reason:
                row['rejected']=reason;ledger.append(row);continue
            pre=np.arange(i-4-npre,i-4);post=np.arange(i+5,i+5+npost)
            # Local quadratic positions yield vz without any adopted kd/km/e.
            pol=[np.polynomial.polynomial.polyfit(t[ix]-t[i],p[ix],2) for ix in [pre,post]]
            def state(k,dt):
                a=pol[k];return a[0]+dt*a[1]+dt*dt*a[2],a[1]+2*dt*a[2]
            sol=minimize_scalar(lambda dt:np.sum((state(0,dt)[0]-state(1,dt)[0])**2),bounds=(-.013,.013),method='bounded',options={'xatol':1e-9})
            dt=sol.x;pp,vi=state(0,dt);po,vo=state(1,dt);pc=(pp+po)/2
            rms=[float(np.sqrt(np.mean(np.sum((np.polynomial.polynomial.polyval(t[ix]-t[i],pol[k]).T-p[ix])**2,axis=1)))) for k,ix in enumerate([pre,post])]
            if max(rms)>.007:reason='position_residual'
            elif np.sqrt(sol.fun)>.018:reason='position_continuity'
            elif abs(pc[2]-.02)>.025:reason='contact_height'
            elif abs(dt)>.0125:reason='contact_time_boundary'
            elif not (vi[2]<-.35 and vo[2]>.20 and max(np.linalg.norm(vi),np.linalg.norm(vo))<18):reason='velocity_direction'
            elif not (pol[0][2,2]<-1 and pol[1][2,2]<-1):reason='nonballistic_acceleration'
            if reason:
                row.update(rejected=reason,local_rms_mm=np.array(rms)*1000);ledger.append(row);continue
            row.update(contact_time=float(t[i]+dt),contact_xyz=pc,normal_vin=float(vi[2]),normal_vout=float(vo[2]),normal_e=float(-vo[2]/vi[2]),local_rms_mm=np.array(rms)*1000,continuity_mm=float(np.sqrt(sol.fun)*1000),pre_start=int(pre[0]),pre_end=int(pre[-1]+1),post_start=int(post[0]),post_end=int(post[-1]+1),vin_poly=vi,vout_poly=vo)
            # Window sensitivity is an independent derivative check, not a new fit gate.
            ev=[]
            for shrink in [0,6]:
                local=[]
                for ix in [pre[shrink:],post[:len(post)-shrink] if shrink else post]:
                    if len(ix)<12:break
                    a=np.polynomial.polynomial.polyfit(t[ix]-t[i],p[ix,2],2);local.append(a[1]+2*dt*a[2])
                if len(local)==2 and local[0]<0:ev.append(float(-local[1]/local[0]))
            row['window_sensitivity_e']=ev
            rows.append(row);ledger.append(dict(row,rejected=None))
    # Link only adjacent observed bounces whose intervening path is a clean flight.
    # Chains need >=3 bounces to supply an interior before/after apex comparison.
    chain=[]
    for row in rows:
        linked=False
        if chain:
            prev=chain[-1];a=prev['frame'];b=row['frame'];dt=t[b]-t[a]
            if .10<dt<1.2 and valid[a:b+1].all() and not net[a:b+1].any() and b-a>20:
                ix=np.arange(a+5,b-4);tau=t[ix]-t[ix[0]];co=np.polynomial.polynomial.polyfit(tau,p[ix],2);res=np.polynomial.polynomial.polyval(tau,co).T-p[ix]
                rms=float(np.sqrt(np.mean(np.sum(res*res,axis=1))));linked=rms<.008 and co[2,2]<-1
        if not linked:
            if len(chain)>=3:seqs.append(chain)
            chain=[]
        chain.append(row)
    if len(chain)>=3:seqs.append(chain)
    heights=[]
    for seqno,seq in enumerate(seqs):
        for r in seq:r['chain_id']=f'{part}:{seqno}'
        for a,b,c in zip(seq,seq[1:],seq[2:]):
            h0=float(np.max(p[a['frame']:b['frame']+1,2])-b['contact_xyz'][2]);h1=float(np.max(p[b['frame']:c['frame']+1,2])-b['contact_xyz'][2])
            if h0>.015 and h1>.005:heights.append({'part':part,'frame':b['frame'],'chain_id':b['chain_id'],'height_before_m':h0,'height_after_m':h1,'apparent_sqrt_height_ratio':float(np.sqrt(h1/h0)),'local_velocity_e':b['normal_e'],'normal_speed_mps':-b['normal_vin']})
    dump(out/f'part{part}'/'contact_position_ledger.json',{'candidate_count':len(ledger),'accepted_count':len(rows),'rejections':dict(Counter(r['rejected'] for r in ledger if r['rejected'])),'candidates':ledger,'net_proximity_frames':int(near.sum()),'net_exclusion_frames':int(net.sum()),'net_proximity_episodes':len(tracked_runs(t,near,300,1)),'multi_bounce_chains':[[r['frame'] for r in seq] for seq in seqs],'interior_height_checks':heights})
    return arcs,rows,heights


def spin_contacts(rows,data,integrate,par):
    accepted=[];rejects=Counter()
    for row in rows:
        d=data[row['part']];t=d['t'];p=d['ball_pos'];q=d['ball_quat'];arcs=[]
        for start,end in [(row['pre_start'],row['pre_end']),(row['post_start'],row['post_end'])]:
            ix=np.arange(start,end);w,ang,rob=spin_from_quats(q[ix],300);coh=np.linalg.norm(np.sum(w,axis=0))/max(np.sum(np.linalg.norm(w,axis=1)),1e-9)
            if not np.isfinite(rob).all() or np.max(ang)>90 or (np.linalg.norm(rob)>5 and coh<.65):break
            arcs.append({'t':t[ix],'pos':p[ix],'quat':q[ix],'w':rob})
        if len(arcs)!=2:rejects['spin_quality']+=1;continue
        f=fit(arcs,integrate,par,False)
        if np.max(f['rms'])>.007:rejects['flight_residual']+=1;continue
        def states(tc):return [propagate(f['states'][k],a['w'],par,tc-a['t'][0]) for k,a in enumerate(arcs)]
        sol=minimize_scalar(lambda tc:np.sum((states(tc)[0][0]-states(tc)[1][0])**2),bounds=(row['time']-.013,row['time']+.013),method='bounded')
        st=states(sol.x);pc=(st[0][0]+st[1][0])/2
        if np.sqrt(sol.fun)>.018 or abs(pc[2]-.02)>.025 or abs(sol.x-row['time'])>.0125:rejects['contact_geometry']+=1;continue
        vi,vo=st[0][1],st[1][1]
        if vi[2]>=-.35 or vo[2]<=.20:rejects['velocity_direction']+=1;continue
        accepted.append(dict(row,vin=vi,vout=vo,win=arcs[0]['w'],wout=arcs[1]['w'],observed_normal_restitution=float(-vo[2]/vi[2]),rk4_contact_xyz=pc))
    return accepted,dict(rejects)


def grouped_bootstrap(rows,fun,n=150):
    rng=np.random.default_rng(1016);blocks=sorted(set((r['part'],int(r['time']//30)) for r in rows));values=[]
    for _ in range(n):
        ids=rng.integers(0,len(blocks),len(blocks));sample=[r for i in ids for r in rows if (r['part'],int(r['time']//30))==blocks[i]];values.append(fun(sample))
    return {'method':f'30s block bootstrap, {n} repeats, within one recording','percentile_95':np.percentile(values,[2.5,97.5],axis=0),'samples':values}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--baseline',type=Path,required=True);ap.add_argument('--bootstrap',type=int,default=24);ap.add_argument('--reuse-flight',action='store_true');args=ap.parse_args();out=args.output;baseline=json.loads(args.baseline.read_text());data={};arcs=[];normal=[];heights=[];offset=0
    fingerprints={str(i):hashlib.sha256((out/f'part{i}'/'raw_extracted.npz').read_bytes()).hexdigest() for i in [1,2,3]}
    if args.reuse_flight:
        assert json.loads((out/'flight_input_fingerprints.json').read_text())==fingerprints, 'Flight inputs changed'
    else:dump(out/'flight_input_fingerprints.json',fingerprints)
    for part in [1,2,3]:
        d=dict(np.load(out/f'part{part}'/'raw_extracted.npz'));data[part]=d
        aa,nn,hh=position_contacts(d,part,offset,out);arcs+=aa;normal+=nn;heights+=hh;offset+=len(d['t'])/300
        print('PART',part,'flights',len(aa),'position contacts',len(nn),'interior multi-bounce',len(hh),flush=True)
    dump(out/'position_contacts.json',normal);dump(out/'multibounce_heights.json',heights)
    train=[a for a in arcs if a['part']!=3];test=[a for a in arcs if a['part']==3];assert len(train)>20 and len(test)>=5,(len(train),len(test));integrate=kernel(out)
    tf=json.loads((out/'training_fit.json').read_text()) if args.reuse_flight else fit(train,integrate,params=[baseline['drag_k'],baseline['magnus_k']]);assert tf['success'],tf['termination'];dump(out/'training_fit.json',dict(tf,ids=[a['id'] for a in train],test_ids=[a['id'] for a in test]));print('TRAIN FLIGHT',tf['params'],flush=True)
    assert tf.get('ids',[a['id'] for a in train])==[a['id'] for a in train], 'Training selection changed'
    val={}
    for name,par in [('ppq002',[baseline['drag_k'],baseline['magnus_k']]),('ppq004',tf['params']),('magnus_off',[tf['params'][0],0])]:
        rr=causal(test,integrate,par);val[name]={'summary':summary(rr),'rows':rr};print('FLIGHT HOLDOUT',name,val[name]['summary'],flush=True)
    dump(out/'flight_holdout.json',val)
    af=json.loads((out/'all_fit.json').read_text()) if args.reuse_flight else fit(arcs,integrate,params=tf['params']);assert af['success'],af['termination'];dump(out/'all_fit.json',dict(af,ids=[a['id'] for a in arcs]));print('ALL FLIGHT',af['params'],flush=True)
    # Held-out contact reconstruction uses TRAIN-ONLY flight coefficients.
    spin,rej=spin_contacts(normal,data,integrate,tf['params']);dump(out/'spin_contacts_strict_holdout.json',{'accepted':spin,'rejected':rej})
    tr=[r for r in spin if r['part']!=3];te=[r for r in spin if r['part']==3];assert len(tr)>10 and len(te)>0,(len(tr),len(te));cp,info=fit_contact(tr)
    print('CONTACT EVIDENCE LIMITED: only',len(te),'held-out contacts; descriptive comparison, not qualified',flush=True)
    contact={'training_params':cp,'training_fit':info,'holdout_ppq002':evaluate(te,[baseline['restitution_v'],baseline['table_tangential_gain'],baseline['table_friction_cap_mu']]),'holdout_ppq004':evaluate(te,cp)}
    print('CONTACT HOLDOUT',contact,flush=True)
    final,rej=spin_contacts(normal,data,integrate,af['params']);dump(out/'spin_contacts_all.json',{'accepted':final,'rejected':rej});cp,ci=fit_contact(final);contact.update(parameters=cp,all_fit=ci,bootstrap=grouped_bootstrap(final,lambda rr:fit_contact(rr)[0]))
    dump(out/'contact_fit.json',contact);print('ALL CONTACT',cp,ci,flush=True)
    nt=[r for r in normal if r['part']!=3];ne=[r for r in normal if r['part']==3];nf=normal_fit(nt);na=normal_fit(normal)
    nresult={'all_fit':na,'training_fit':nf,'holdout_ppq002':normal_score(ne,baseline['restitution_v']),'holdout_ppq004_joint':normal_score(ne,float(contact['training_params'][0])),'holdout_position_only':normal_score(ne,nf['restitution_v']),'bootstrap':grouped_bootstrap(normal,lambda rr:normal_fit(rr)['restitution_v']),'by_part':{str(i):normal_fit([r for r in normal if r['part']==i]) for i in [1,2,3]},'speed_bins':[]}
    for lo,hi in [(0,1),(1,2),(2,3),(3,4),(4,10)]:
        rr=[r for r in normal if lo<=-r['normal_vin']<hi]
        if len(rr)>=3:nresult['speed_bins'].append(dict(normal_fit(rr),range=[lo,hi],individual_e_median=float(np.median([r['normal_e'] for r in rr]))))
    dump(out/'normal_position_fit.json',nresult);print('POSITION NORMAL',na,'holdout',nresult['holdout_position_only'],flush=True)
    br=json.loads((out/'flight_uncertainty.json').read_text()) if args.reuse_flight and (out/'flight_uncertainty.json').exists() else grouped_bootstrap([{'part':a['part'],'time':float(a['t'][0]),'arc':a} for a in arcs],lambda rr:fit([r['arc'] for r in rr],integrate,params=af['params'],max_nfev=400)['params'],args.bootstrap)
    dump(out/'flight_uncertainty.json',br)
    proposal=dict(baseline,drag_k=float(af['params'][0]),magnus_k=float(af['params'][1]),restitution_v=float(cp[0]),table_tangential_gain=float(cp[1]),table_friction_cap_mu=float(cp[2]),restitution_h=float(1-cp[1]),post_net_future_bounce_tangential_gain=float(cp[1]),post_net_future_bounce_restitution_v=float(cp[0]),future_contact_spin_tangential_gain=float(cp[1]))
    dump(out/'candidate_parameters.json',proposal)
    dump(out/'fit_summary.json',{'flight_count':len(arcs),'train_flights':len(train),'test_flights':len(test),'normal_contacts':len(normal),'spin_contacts':len(final),'multi_bounce_interior_checks':len(heights),'candidate':proposal,'deployed':False})
    print('DONE',flush=True)

if __name__=='__main__':main()
