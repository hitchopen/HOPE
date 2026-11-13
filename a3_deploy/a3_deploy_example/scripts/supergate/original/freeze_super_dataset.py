from pathlib import Path
import json,hashlib,collections,shutil
import numpy as np,yaml
W=Path(__file__).resolve().parent;O=W.parent/'outputs'/'super-gate3-v1';O.mkdir(parents=True,exist_ok=True)
core=json.loads((W/'super-factorial-selected.json').read_text());origin=json.loads((W/'super-gate3-ledger.json').read_text())
assert len(core)==972
assert set(collections.Counter(r['cell_id'] for r in core).values())=={12}
extra=[]
for row in origin['selected']:
 if row['net'][1]<=.18:continue
 r=dict(row);r.update(requested_id='origin-'+str(row['requested_id']),section='origin_boundary',cell_id=None,lateral_bin=int(np.digitize(r['predicted_contact'][1]+.7625,[-.32,-.15])),angle_bin=int(np.digitize(r['predicted_angle_deg'],[-5.,5.])));extra.append(r)
assert len(extra)==228
rng=np.random.default_rng(2026110801);rng.shuffle(extra)
shards=[[] for _ in range(6)]
for cell in range(81):
 rows=[dict(r) for r in core if r['cell_id']==cell];rng.shuffle(rows)
 for shard in range(6):shards[shard].extend(rows[2*shard:2*shard+2])
base=yaml.safe_load((W/'branch-gate3/fixed-home.yaml').read_text());allrows=[]
for k,rows in enumerate(shards):
 rows.extend(extra[38*k:38*k+38]);rng.shuffle(rows);assert len(rows)==200
 for i,r in enumerate(rows):
  r.update(sample_id=f'SG3v1-{k+1:02d}-{i+1:03d}',shard_id=k+1,shot_id=i+1,aim=[2.055,-.7625]);allrows.append(r)
 scene=dict(base);scene.update(description=f'Super Gate3 v1 shard {k+1}/6: 162 factorial core + 38 origin boundary. Frozen before policy results.',balls=200,clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]],spin_rad_s_list=[r['initial_spin'] for r in rows]);scene['timing']=dict(base['timing']);scene['timing'].update(flight_s_list=[max(1.50,r['flight_s']+.95) for r in rows],pause_s_list=[.8]*6+[0.]*194);scene['timing']['flight_s_list'][-1]=3.
 (O/f'shard-{k+1:02d}.yaml').write_text(yaml.safe_dump(scene));(O/f'shard-{k+1:02d}-ledger.json').write_text(json.dumps(rows,indent=2))
assert len({tuple(r['launch']+r['initial_spin']) for r in allrows})==1200
(O/'samples.json').write_text(json.dumps(allrows,indent=2));(O/'requested-core-including-exclusions.json').write_bytes((W/'super-factorial-requested.json').read_bytes());(O/'requested-origin-including-exclusions.json').write_text(json.dumps(origin['requested'],indent=2))
meta={'version':'super-gate3-v1','seed':2026110801,'policy_outcome_used_to_select':False,'core_count':972,'origin_boundary_count':228,'total_unique':1200,'base_cells':81,'core_samples_per_cell':12,'shards':6,'samples_per_shard':200,'core_samples_per_cell_per_shard':2,'axis_definitions':{'lateral_y_relative_home_m':[[-.68,-.32],[-.32,-.15],[-.15,.28]],'world_height_m':[[.8,1],[1,1.2],[1.2,1.45]],'incoming_speed_norm_mps':[[0,2.5],[2.5,4],[4,None]],'horizontal_incoming_angle_deg':[[-90,-5],[-5,5],[5,90]],'angle_formula':'degrees(atan2(v_y, -v_x))','nominal_contact_plane_world_x_m':.07,'nominal_home_xy_m':[-.5,-.7625],'table_height_m':.76,'outgoing_aim_table_xy_m':[2.055,-.7625]},'candidate_attempts':len(json.loads((W/'super-factorial-requested.json').read_text())),'range':{key:[min(r[key] for r in allrows),max(r[key] for r in allrows)] for key in ['origin_x','origin_y','target_relative_y','predicted_contact_speed','predicted_contact_world_height','predicted_angle_deg']},'ascending_count':sum(r['predicted_contact'][5]>0 for r in allrows),'descending_count':sum(r['predicted_contact'][5]<0 for r in allrows),'geometry':json.loads((W/'geometry-audit.json').read_text())['comparison'],'scope':'Synthetic stratified physical flights. Real measured launch-plane crossings guide range, but launches are not recovered opponent racket contacts. No question tuple injection. Full native Runner/Planner/MuJoCo simulation; no robot hardware.'}
(O/'design.json').write_text(json.dumps(meta,indent=2))
code=O/'reproduction';code.mkdir(exist_ok=True)
for name in ['flight_kernel.py','build_super_factorial.py','fill_factorial.py','build_super_gate3.py','freeze_super_dataset.py','run_isolated.py','isolated_entry.py','run_gate3_suite.py','pp_gate3_ball_launcher.py','rally-suite.sh','hitter-suite.sh']:
 shutil.copy2(W/name,code/name)
shutil.copy2(W/'geometry-audit.json',O/'geometry-audit.json')
manifest={str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file() and p.name!='SHA256.json'}
(O/'SHA256.json').write_text(json.dumps(manifest,indent=2));(W/'super-dataset-frozen.json').write_text(json.dumps({'path':str(O),'dataset_manifest_sha256':hashlib.sha256((O/'SHA256.json').read_bytes()).hexdigest(),'design':meta},indent=2));print(json.dumps(meta,indent=2))
