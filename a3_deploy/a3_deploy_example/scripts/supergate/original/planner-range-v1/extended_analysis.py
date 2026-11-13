from pathlib import Path
import json,sys
import numpy as np,pandas as pd
def stats(x):
    x=np.asarray(x,dtype=float);x=x[np.isfinite(x)]
    if not len(x):return {'n':0}
    return {'n':len(x),'mean':float(x.mean()),'std':float(x.std(ddof=1)) if len(x)>1 else 0.,'p05':float(np.quantile(x,.05)),'p95':float(np.quantile(x,.95)),'min':float(x.min()),'max':float(x.max())}
def foot_stats(f):
    loaded=f.right_fz_N>50;bad=loaded&(f.right_medial_fz_N/(f.right_fz_N+1e-9)>.9)&(f.right_outer_above_inner_mm>3)
    return {'samples':len(f),'right_gap_mm':stats(f.right_outer_above_inner_mm),'right_loaded_fraction':float(loaded.mean()),'right_medial_only_fraction_all_samples':float(bad.mean()),'right_medial_only_fraction_loaded_samples':float(bad.sum()/max(1,loaded.sum())),'left_gap_mm':stats(f.left_outer_above_inner_mm)}
def analyze_extended(root,groups,dataset):
    root=Path(root);dataset=Path(dataset);outputs=[]
    for gi,g in enumerate(groups):
        rr=[];kk=[];pp=[];ff=[];late=[];drain=[];engages=[];falls=[];sources=[]
        for e in range(gi*6,gi*6+6):
            r=pd.read_csv(root/f'env-{e}-results.csv');r['env']=e;rr.append(r)
            kin=pd.read_csv(root/f'env-{e}-kinematics.csv');kk.append(kin)
            posture=pd.read_csv(root/f'env-{e}-posture.csv');pp.append(posture)
            f=pd.read_csv(root/f'env-{e}-feet.csv');ff.append(f[(f.shot>0)&(f.t-f.deadline>.6)])
            late.append(f[(f.shot>0)&(f.t-f.deadline>1.4)&(f.ttc>.7)])
            drain.append(f[f.t>f.t.max()-2])
            obs=pd.read_csv(root/f'env-{e}-observations.csv',usecols=['t','shot','obs_109'])
            engaged=obs[(obs.shot>0)&(obs.obs_109>0)&(obs.obs_109<.7)].groupby('shot',as_index=False).first()
            ledger=json.loads((dataset/f'shard-{e%6+1:02d}-ledger.json').read_text())
            for row in r.itertuples():
                sources.append({'source':ledger[row.shot-1].get('validation_source','unspecified'),'valid_plan':int(row.valid_plans>0),'engaged':int(row.shot in set(engaged.shot)),'legal':int(row.legal),'fall_affected':int(row.min_z<.85 or row.max_tilt>.65)})
            for ob in engaged.itertuples():
                shot=r[r.shot==ob.shot].iloc[0];by=float(np.interp(ob.t,posture.t,posture.base_y))
                engages.append({'env':e,'shot':int(ob.shot),'t':ob.t,'side':int(shot.side),'base_y':by,'target_live_dy_at_first_active_sample_m':float(shot.strike_y-by),'target_world_height_m':float(shot.strike_z+.76)})
            fallen=posture[(posture.base_z<.85)|(posture.tilt_deg>np.degrees(.65))]
            if not fallen.empty:
                a=fallen.iloc[0];shot=int(a.shot);history=[]
                for sid in range(max(1,shot-3),shot+1):
                    sr=r[r.shot==sid].iloc[0];sp=posture[posture.shot==sid];ef=next((x for x in engages if x['env']==e and x['shot']==sid),None)
                    history.append({'shot':sid,'side':int(sr.side),'valid_plan':int(sr.valid_plans>0),'first_active_sample':ef,'start_base_y':float(sp.iloc[0].base_y),'end_base_y':float(sp.iloc[-1].base_y),'case':ledger[sid-1]})
                falls.append({'env':e,'first_shot':shot,'first_threshold_time_s':float(a.t),'affected_windows':int(((r.min_z<.85)|(r.max_tilt>.65)).sum()),'preceding_shots':history})
        r=pd.concat(rr);kin=pd.concat(kk);posture=pd.concat(pp);src=pd.DataFrame(sources)
        planned=kin[kin.event=='planned'];contact=kin[kin.event=='contact'];land=r[np.isfinite(r.landing_x)&np.isfinite(r.landing_y)];legal=r[r.legal>0]
        item={'label':g['label'],'launched':len(r),'planner_valid_shots':int((r.valid_plans>0).sum()),'native_engaged_shots':len(engages),'legal':int(r.legal.sum()),'fall_affected_windows':int(((r.min_z<.85)|(r.max_tilt>.65)).sum()),'sequences_with_fall':len(falls),'fall_onsets':falls,'admission_by_source':src.groupby('source').agg(launched=('source','size'),planner_valid=('valid_plan','sum'),native_engaged=('engaged','sum'),legal=('legal','sum'),fall_affected=('fall_affected','sum')).to_dict('index'),'recovery':foot_stats(pd.concat(ff)),'late_wait':foot_stats(pd.concat(late)),'final_2s':foot_stats(pd.concat(drain)),'errors':{},'landing':{},'posture':{},'engaged_target_envelope':{}}
        for event,data in [('planned',planned),('contact',contact)]:
            item['errors'][event]={c:stats(data[c]) for c in ['p_error_m','v_error_mps','n_error_deg','dp_x','dp_y','dp_z','dv_x','dv_y','dv_z','dn_x','dn_y','dn_z']}
        for label,data in [('all_observed',land),('legal',legal)]:item['landing'][label]={c:stats(data[c]) for c in ['landing_x','landing_y']}
        for c in ['base_z','roll_deg','pitch_deg','tilt_deg','pelvis_yaw_deg','torso_yaw_deg','twist_deg']:item['posture'][c]=stats(posture[c])
        for side,name in [(1,'FH'),(-1,'BH')]:item['engaged_target_envelope'][name]=stats([x['target_live_dy_at_first_active_sample_m'] for x in engages if x['side']==side])
        item['limitations']=['First active sample is at 50 Hz, not the exact admission instant.','No hardware, wall-time, transport or lifecycle qualification.','Post-fall windows remain included and are explicitly counted; they are not independent falls.','Zero observed falls applies only to the sampled trajectories and order.']
        outputs.append(item)
    (root/'extended-analysis.json').write_text(json.dumps(outputs,indent=2,allow_nan=False));return outputs
if __name__=='__main__':
    a=analyze_extended(sys.argv[1],json.loads(Path(sys.argv[2]).read_text()),sys.argv[3]);print(json.dumps([{k:r[k] for k in ['label','launched','planner_valid_shots','native_engaged_shots','legal','fall_affected_windows','sequences_with_fall']} for r in a],indent=2))
