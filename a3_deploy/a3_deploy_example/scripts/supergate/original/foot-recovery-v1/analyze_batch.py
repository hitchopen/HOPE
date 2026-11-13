from pathlib import Path
import json,sys
import numpy as np,pandas as pd
def analyze(root,groups):
    root=Path(root);rows=[]
    for k,group in enumerate(groups):
        frames=[];trials=[];kins=[]
        for e in range(k*6,k*6+6):
            f=pd.read_csv(root/f'env-{e}-feet.csv');f['env']=e;frames.append(f)
            r=pd.read_csv(root/f'env-{e}-results.csv');r['env']=e;trials.append(r)
            kin=pd.read_csv(root/f'env-{e}-kinematics.csv');kins.append(kin)
        f=pd.concat(frames);r=pd.concat(trials);kin=pd.concat(kins);f=f[(f.shot>0)&(f.t-f.deadline>.6)]
        bad=(f.right_medial_fz_N/(f.right_fz_N+1e-9)>.9)&(f.right_fz_N>50)&(f.right_outer_above_inner_mm>3)
        row={'model':group['label'],'path':group['path'],'trials':len(r),'fall_windows':int(((r.min_z<.85)|(r.max_tilt>.65)).sum()),'minimum_base_height_m':float(r.min_z.min()),'maximum_base_tilt_deg':float(np.degrees(r.max_tilt.max())),'valid_plans':int((r.valid_plans>0).sum()),'legal':int(r.legal.sum()),'contacts':int((r.contacts>0).sum()),'right_gap_mean_mm':float(f.right_outer_above_inner_mm.mean()),'right_gap_std_mm':float(f.right_outer_above_inner_mm.std()),'right_gap_p95_mm':float(f.right_outer_above_inner_mm.quantile(.95)),'bad_recovery_fraction':float(bad.mean()),'left_gap_p95_mm':float(f.left_outer_above_inner_mm.quantile(.95)),'metric_scope':'continuous per shard; recovery samples from planned strike +0.6 s through next launch; final 5 s drain included','planned_errors':{}}
        for c in ['p_error_m','v_error_mps','n_error_deg','dp_x','dp_y','dp_z','dv_x','dv_y','dv_z','dn_x','dn_y','dn_z']:
            a=kin.loc[kin.event=='planned',c];row['planned_errors'][c]={'mean':float(a.mean()),'std':float(a.std()),'p95':float(a.quantile(.95))}
        rows.append(row)
    return rows
if __name__=='__main__':
    root=Path(sys.argv[1]);groups=json.loads(Path(sys.argv[2]).read_text());rows=analyze(root,groups);(root/'analysis.json').write_text(json.dumps(rows,indent=2));print(json.dumps([{k:r[k] for k in ['model','trials','fall_windows','legal','right_gap_mean_mm','right_gap_p95_mm','bad_recovery_fraction']} for r in rows],indent=2))
