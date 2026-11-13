from pathlib import Path
import json,time,subprocess,sys,os,copy,hashlib
import numpy as np,yaml
W=Path(__file__).resolve().parent;T=W.parent;F=T/'foot-recovery-v1';V='/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python'
sys.path.insert(0,str(F));from analyze_batch import analyze
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def mixed_data():
    out=W/'data-mixed-long'
    if out.exists():return out
    out.mkdir();allrows=[];rng=np.random.default_rng(202611090711)
    for sh in range(1,7):
        rows=[]
        for source,root,n in [('outer',W/'data-outer',67),('fresh_holdout',T/'best-v2-research/data-holdout',81),('main',T.parent/'outputs/super-gate3-v1',52)]:
            batch=json.loads((root/f'shard-{sh:02d}-ledger.json').read_text())[:n]
            for r in batch:r['validation_source']=source
            rows.extend(batch)
        assert len(rows)==200;rng.shuffle(rows)
        for i,r in enumerate(rows):r.update(shard_id=sh,shot_id=i+1,sample_id=f'mixed-{sh:02d}-{i+1:03d}')
        scene=yaml.safe_load((W/'data-outer'/f'shard-{sh:02d}.yaml').read_text());scene.update(balls=200,clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]],spin_rad_s_list=[r['initial_spin'] for r in rows]);scene['timing'].update(flight_s_list=[max(1.5,r['flight_s']+.95) for r in rows],pause_s_list=[.8]*6+[0.]*194);scene['timing']['flight_s_list'][-1]=3.
        (out/f'shard-{sh:02d}.yaml').write_text(yaml.safe_dump(scene));(out/f'shard-{sh:02d}-ledger.json').write_text(json.dumps(rows,indent=2));allrows.extend(rows)
    (out/'samples.json').write_text(json.dumps(allrows,indent=2));(out/'design.json').write_text(json.dumps({'total':1200,'continuous_shots_per_shard':200,'outer':402,'fresh_holdout':486,'main':312,'order_seed':202611090711,'duplicates':len(allrows)-len({tuple(r['launch']+r['initial_spin']) for r in allrows}),'purpose':'Long continuous mixture; no per-shot reset; prior outer trajectories retained, fresh frozen inner holdout and old main added. Bounds are selected using fall outcomes, so this remains empirical testing, not proof outside sampled cases.'},indent=2));(out/'SHA256.json').write_text(json.dumps({p.name:sha(p) for p in out.iterdir() if p.is_file()},indent=2));return out
mixed=mixed_data()
def run(profile,n,dataset,groups,out,log):
    if (out/'run.json').exists():return
    assert not out.exists(),out
    cmd=['distrobox','enter','hope','--','env','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','PP_GATE3_SUPPORT_PROFILE='+str(profile),'taskset','-c','18-23','nice','-n','10',V,str(T/'internal_batch_v6/run.py'),str(n),'6',str(out),'--policy-groups',str(groups),'--dataset',str(dataset)]
    with (W/log).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
    assert r.returncode==0,(log,r.returncode)
def summarize(out,groups):
    a=analyze(out,groups);(out/'analysis.json').write_text(json.dumps(a,indent=2));print('ANALYZED',out.name,json.dumps([{k:r[k] for k in ['model','trials','fall_windows','valid_plans','legal','bad_recovery_fraction']} for r in a]),flush=True);return a
# A previously started native batch is allowed to finish; never overlap simulators.
while not (F/'body-screen/run.json').exists():time.sleep(2)
body=json.loads((F/'candidate-body-groups.json').read_text());body_rows=summarize(F/'body-screen',body)
base=json.loads((W/'best-v1-groups.json').read_text());selected_profile=None;passed=[]
for label,bhmax in [('bh60',.60),('bh45',.45),('bh40',.40)]:
    profile=W/f'support-{label}.yaml'
    if not profile.exists():
        p=yaml.safe_load((W/'support-bh45.yaml').read_text());p['empirical_support']['backhand_y_range']=[-.24,bhmax];profile.write_text(yaml.safe_dump(p,sort_keys=False))
    out=W/(label+'-outer');run(profile,67,W/'data-outer',W/'best-v1-groups.json',out,label+'-outer.log');summary=summarize(out,base)
    if summary[0]['fall_windows']:continue
    out=W/(label+'-mixed-long');run(profile,200,mixed,W/'best-v1-groups.json',out,label+'-mixed-long.log');summary=summarize(out,base)
    if summary[0]['fall_windows']:continue
    selected_profile=profile;passed=[W/(label+'-outer'),out];break
assert selected_profile is not None,'None of the candidate expansions passed; preserve original deployed bounds'
(W/'range-qualified.json').write_text(json.dumps({'status':'SIMULATION_QUALIFIED_FOR_FIELD_INTEGRATION','profile':str(selected_profile),'profile_sha256':sha(selected_profile),'policy_sha256':json.loads((passed[0]/'run.json').read_text())['policy_sha256']['best_v1'],'datasets':[str(x) for x in passed],'fall_detector':'per-shot min pelvis height <0.85 m OR max pelvis tilt >0.65 rad; continuous trial and onset evidence retained','zero_fall_scope':'bounded sampled simulation, not a hardware or universal safety guarantee'},indent=2));print('RANGE_QUALIFIED',selected_profile,flush=True)
# After old best v1 range is available to the deployment chat, continue foot work.
leg=json.loads((F/'live-screen/analysis.json').read_text());eligible=[r for r in body_rows+leg if r['model']!='best_v1' and r['fall_windows']==0];best=min(eligible,key=lambda r:r['bad_recovery_fraction']);candidate=[{'label':best['model'],'path':best['path']}];gp=W/'final-foot-groups.json';gp.write_text(json.dumps(candidate,indent=2));(W/'selected-support-candidate.json').write_text(json.dumps({'selected':candidate[0],'training_summary':best},indent=2))
out=W/'final-foot-mixed-long';run(selected_profile,200,mixed,gp,out,'final-foot-mixed-long.log');summary=summarize(out,candidate);print('FOOT_LONG_COMPLETE',summary[0]['fall_windows'],flush=True)
