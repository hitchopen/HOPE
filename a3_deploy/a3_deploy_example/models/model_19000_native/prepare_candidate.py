"""Metadata-only diagnostic candidate, not a claim of native GRU compatibility.

Appends genuine saved metadata without reserializing the original GraphProto.
No training, simulator, hardware or source-actor mutation.
"""
import argparse,hashlib,importlib.util,json,os,sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.dont_write_bytecode=True
import numpy as np
import onnxruntime as ort
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
ORIGINAL=HERE.with_name('model_19000')
PT_SHA='744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0'
ONNX_SHA='f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda'
YAML_SHA='fd7e5361e336791f734e24dcc2f8837319665d5f88748a6bf2c04b6c639f3e51'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def digest(data):return hashlib.sha256(data).hexdigest()
def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def varint(n):
 out=bytearray()
 while n>127:out.append((n&127)|128);n>>=7
 out.append(n);return bytes(out)
def readvar(data,pos):
 n=shift=0
 while True:
  b=data[pos];pos+=1;n|=(b&127)<<shift
  if b<128:return n,pos
  shift+=7;assert shift<=63

def fields(data):
 pos=0
 while pos<len(data):
  start=pos;key,pos=readvar(data,pos);number,wire=key>>3,key&7
  if wire==0:_,pos=readvar(data,pos);payload=None
  elif wire in (1,5):size=8 if wire==1 else 4;payload=data[pos:pos+size];pos+=size
  elif wire==2:size,pos=readvar(data,pos);payload=data[pos:pos+size];pos+=size
  else:raise ValueError('Unsupported protobuf wire type')
  assert pos<=len(data);yield number,wire,payload,data[start:pos]
def string_field(number,value):
 value=value.encode();return varint(number*8+2)+varint(len(value))+value
def append_metadata(raw,metadata):
 # ModelProto metadata_props=14. Every original field/GraphProto byte is retained.
 assert not any(n==14 for n,w,p,b in fields(raw)), 'Original unexpectedly has metadata'
 for key,value in sorted(metadata.items()):
  record=string_field(1,key)+string_field(2,value)
  raw+=varint(14*8+2)+varint(len(record))+record
 return raw

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--checkpoint',type=Path,required=True);ap.add_argument('--params',type=Path,required=True);ap.add_argument('--evidence',type=Path,required=True);a=ap.parse_args();a.evidence.mkdir(parents=True,exist_ok=True)
 old=ORIGINAL/'policy/exported/actor.onnx';old_yaml=ORIGINAL/'policy/params/deploy.yaml'
 assert sha(a.checkpoint)==PT_SHA and sha(old)==ONNX_SHA and sha(old_yaml)==YAML_SHA
 preserve={str(p.relative_to(ORIGINAL)):sha(p) for p in ORIGINAL.rglob('*') if p.is_file()}
 prior=a.evidence/'original-files-before.json'
 if prior.exists():assert json.loads(prior.read_text())==preserve
 else:save(prior,preserve)
 sys.path.insert(0,str(ORIGINAL/'config_export'))
 spec=importlib.util.spec_from_file_location('original_configuration_export',ORIGINAL/'config_export/export_config.py');cfgmod=importlib.util.module_from_spec(spec);spec.loader.exec_module(cfgmod)
 cfg=yaml.load((a.params/'env.yaml').read_text(),Loader=cfgmod.ConfigLoader);recipe=yaml.safe_load((a.params/'task_resolved.yaml').read_text());v=json.loads((ORIGINAL/'deploy-config-verification.json').read_text());report_prov=json.loads((ORIGINAL/'policy/params/training-config-provenance.json').read_text())
 for name,h in report_prov['original_training_config_sha256'].items():assert sha(a.params/name)==h
 d=yaml.safe_load(old_yaml.read_text());rows=v['joint_rows'];obs=v['observation_rows'];names=[r['joint'] for r in rows]
 resolved=v['training_action_delay'];mock=NS(action_manager=NS(get_term=lambda _:NS(passive_joint_names=cfg['actions']['joint_pos']['passive_joint_names'],resolved_contract_metadata=lambda:resolved)),command_manager=NS(get_term=lambda n:NS(cfg=NS(**cfg['commands'][n]))),scene={'robot':NS(data=NS(joint_names=names))},step_dt=d['step_dt'])
 pure=cfgmod.module(cfgmod.UTILS/'utils/sim2real_export.py','native_saved_sim2real');md=pure.sim2real_metadata(mock,recipe['task'])
 def csv(items):return ','.join(str(x) for x in items)
 md.update({'joint_names':csv(names),'body_names':csv(cfg['commands']['motion']['body_names']),'anchor_body_name':cfg['commands']['motion']['anchor_body_name'],'default_joint_pos':csv([r['default_q'] for r in rows]),'action_scale':csv([r['action_scale'] for r in rows]),'joint_stiffness':csv([r['kp'] for r in rows]),'joint_damping':csv([r['kd_wire'] for r in rows]),'observation_names':csv([r['name'] for r in obs]),'observation_history_lengths':csv([r['deployment_history_length'] for r in obs]),'actor_obs_contract':'hitter_pure','actor_obs_total_dim':'110','actor_obs_term_dims':csv([r['dim'] for r in obs]),'actor_obs_term_sources_json':json.dumps({r['name']:r['deployment_source'] for r in obs},separators=(',',':')),'qdes_action_contract':d['contracts']['action']['name'],'qdes_joint_names':csv(names),'qdes_safe_lower_rad':csv([r['safe_lower'] for r in rows]),'qdes_safe_upper_rad':csv([r['safe_upper'] for r in rows]),'qdes_hard_lower_rad':csv([r['hard_lower'] for r in rows]),'qdes_hard_upper_rad':csv([r['hard_upper'] for r in rows]),'qdes_actual_q_hard_tolerance_rad':str(d['contracts']['action']['actual_q_hard_tolerance_rad']),'qdes_actual_q_hard_audit_mode':cfg['actions']['joint_pos']['actual_q_hard_audit_mode'],'qdes_policy_feedback_contract':d['contracts']['action']['policy_feedback'],'a3_training_joint_damping':csv([r['kd_training_total'] for r in rows]),'a3_passive_joint_damping':csv([r['kd_passive'] for r in rows]),'a3_joint_effort_limit':csv([r['effort_limit'] for r in rows]),'a3_control_physics_dt_s':str(v['control_timing']['physics_dt']),'a3_control_decimation':str(v['control_timing']['decimation']),'a3_control_policy_dt_s':str(v['control_timing']['control_dt']),'hitter_pure_checkpoint_sha256':PT_SHA})
 # Follow the existing exporter's clip and supported-envelope metadata using hash-bound assets.
 original_manifest=json.loads((ORIGINAL/'policy/exported/manifest.json').read_text());asset_paths=original_manifest['motion_cfg']['motion_file'];asset_hashes=original_manifest['source_runtime_manifest']['motion_sha256'];reference_assets=[];lengths=[]
 for path,expected in zip(asset_paths,asset_hashes):
  source=ROOT/path;assert sha(source)==expected
  with np.load(source) as data:lengths.append(int(data['joint_pos'].shape[0]));reference_assets.append({'path':path,'sha256':expected,'frames':lengths[-1],'fps':float(np.asarray(data['fps']).reshape(-1)[0]),'joint_pos_shape':list(data['joint_pos'].shape),'body_pos_shape':list(data['body_pos_w'].shape)})
 md['clip_seg_lengths']=csv(lengths);rt=cfg['commands']['racket_target'];phases=rt['strike_phase_per_clip'] if rt['strike_phase_per_clip'] is not None else [rt['strike_phase']]*len(lengths);assert len(phases)==len(lengths);md['clip_strike_phases']=','.join(f'{float(v):.4f}' for v in phases)
 for key,field in [('hitter_pure_pos_range_per_clip','racket_pos_range_per_clip'),('hitter_pure_vel_range_per_clip','racket_vel_range_per_clip'),('hitter_pure_vel_core_range_per_clip','racket_vel_start_range_per_clip'),('hitter_pure_vel_planner_range_per_clip','racket_vel_planner_range_per_clip')]:
  if rt[field] is not None:md[key]=';'.join(','.join(f'{float(lo):.4f},{float(hi):.4f}' for lo,hi in clip) for clip in rt[field])
 md['hitter_pure_vel_planner_mix_prob']=f"{float(rt['racket_vel_planner_mix_prob']):.4f}";md['hitter_pure_vel_range_ramp_steps']=str(int(rt['racket_vel_range_ramp_steps']));md['hitter_pure_base_target_range']=','.join(f'{float(v):.4f}' for v in [*rt['base_target_x_range'],*rt['base_target_y_range']])
 # These are genuine saved/export-source fields; no policy ABI/reference claims are added.
 md={k:str(x) for k,x in md.items()};raw=old.read_bytes();candidate=append_metadata(raw,md)
 assert [p for n,w,p,b in fields(raw) if n==7]==[p for n,w,p,b in fields(candidate) if n==7]
 assert b''.join(b for n,w,p,b in fields(candidate) if n!=14)==raw
 dest=HERE/'policy/exported/policy.onnx';dest.parent.mkdir(parents=True,exist_ok=True)
 if dest.exists():assert dest.read_bytes()==candidate,'Existing candidate differs; preserved, not overwritten'
 else:dest.write_bytes(candidate)
 new_sha=sha(dest);d['provenance']['policy_sha256']=new_sha;d['provenance']['export_source']='model19000_saved_metadata_candidate_graph_unchanged_native_abi_unresolved'
 dy=HERE/'policy/params/deploy.yaml';dy.parent.mkdir(parents=True,exist_ok=True);text=yaml.safe_dump(d,sort_keys=False)
 if dy.exists():assert dy.read_text()==text
 else:dy.write_text(text)
 original=ort.InferenceSession(str(old),providers=['CPUExecutionProvider']);fixed=ort.InferenceSession(str(dest),providers=['CPUExecutionProvider']);assert fixed.get_modelmeta().custom_metadata_map==md
 signature={'inputs':{x.name:x.shape for x in fixed.get_inputs()},'outputs':{x.name:x.shape for x in fixed.get_outputs()}};assert signature=={'inputs':{'obs':[1,110],'h_in':[1,1,110]},'outputs':{'actions':[1,31],'h_out':[1,1,110]}}
 fixtures=np.load(ORIGINAL/'policy/exported/parity_vectors.npz');h=np.random.default_rng(43).normal(0,.1,(1,1,110)).astype(np.float32);h2=h.copy();ae=he=0.
 observations=[*fixtures['obs']]
 # Reuse archived real policy observations spanning FH/BH/rally; this is inference parity, not a simulation.
 recorded=a.params.parent/'attempt-kaub5j2j/scored/policy-trace.jsonl'
 if recorded.exists():
  entries=[json.loads(line) for line in recorded.open()];entries=[r for r in entries if r.get('observation') is not None];observations.extend([np.asarray(entries[i]['observation'],dtype=np.float32) for i in np.linspace(0,len(entries)-1,256,dtype=int)])
 reset_at=[32,192]
 for i,x in enumerate(observations):
  if i in reset_at:h.fill(0);h2.fill(0)
  a1,h=original.run(['actions','h_out'],{'obs':np.asarray(x,np.float32).reshape(1,110),'h_in':h});a2,h2=fixed.run(['actions','h_out'],{'obs':np.asarray(x,np.float32).reshape(1,110),'h_in':h2});ae=max(ae,float(np.max(abs(a1-a2))));he=max(he,float(np.max(abs(h-h2))));np.testing.assert_allclose(a1,a2,atol=2e-5,rtol=2e-4);np.testing.assert_allclose(h,h2,atol=2e-5,rtol=2e-4)
 x=np.asarray(observations[0],np.float32).reshape(1,110);first=fixed.run(['actions','h_out'],{'obs':x,'h_in':np.zeros((1,1,110),np.float32)});again=fixed.run(['actions','h_out'],{'obs':x,'h_in':np.zeros((1,1,110),np.float32)})
 for l,r in zip(first,again):np.testing.assert_array_equal(l,r)
 graph=next(p for n,w,p,b in fields(raw) if n==7)
 manifest={'prepare_source_sha256':sha(Path(__file__)),'parity_input_sources':{'export_vectors_sha256':sha(ORIGINAL/'policy/exported/parity_vectors.npz'),'recorded_policy_trace_sha256':sha(recorded) if recorded.exists() else None,'recorded_indices':'256 evenly spaced nonnull observation rows in chronological order','initial_hidden_rng':'numpy PCG64 seed43 normal(0,.1)','reset_at':[32,192]},'status':'METADATA_ENRICHED_DIAGNOSTIC_CANDIDATE_NOT_NATIVE_READY','run_id':'gru-formal-2b8c09938a','training_source_commit':'33c609022efbb4e85fe6a58a5a8947b0fbd46c76','checkpoint_sha256':PT_SHA,'original_onnx_sha256':ONNX_SHA,'original_deploy_sha256':YAML_SHA,'candidate_onnx_sha256':new_sha,'candidate_deploy_sha256':sha(dy),'graph_proto_sha256':digest(graph),'original_graph_bytes_preserved':True,'all_original_nonmetadata_model_fields_preserved':True,'metadata':md,'signature':signature,'parity':{'status':'PASS','steps':len(observations),'initial_hidden_nonzero':True,'reset_at':reset_at,'action_max_abs_error':ae,'hidden_max_abs_error':he,'atol':2e-5,'rtol':2e-4,'zero_reset_repeat':'PASS','native_runtime_carry_reset_verification':'NOT_IMPLEMENTED_OR_VERIFIED'},'native_loading':'NOT_PROVEN','formal_gate3':'NOT_RUN','unresolved':['Native PpOnnxPolicy supplies obs,time_step; no h_in/h_out state carry/reset path','Native PpPolicy unconditionally calls genuine joint_pos/joint_vel/body_pos_w/body_quat_w reference outputs absent from this frozen actor','Complete native clock/clip/reference export requires authoritative MotionLoader/reference data integration; no synthetic helper outputs/recipe fields supplied','No rebuilt source-bound native recurrent Runner available in sandbox'],'reference_assets':reference_assets,'metadata_sources':{'core':'original deploy YAML and checkpoint/config verification report','runtime':'actual saved commands and existing sim2real_metadata function','no_synthetic_reference_outputs':True},'saved_params_sha256':report_prov['original_training_config_sha256']}
 np.savez_compressed(a.evidence/'actor-parity-inputs.npz',observations=np.asarray(observations,dtype=np.float32),initial_hidden=np.random.default_rng(43).normal(0,.1,(1,1,110)).astype(np.float32),reset_at=np.array(reset_at))
 save(HERE/'candidate-manifest.json',manifest);after={str(p.relative_to(ORIGINAL)):sha(p) for p in ORIGINAL.rglob('*') if p.is_file()};assert after==preserve;assert sha(a.checkpoint)==PT_SHA;save(a.evidence/'original-files-after.json',after);save(a.evidence/'candidate-parity.json',manifest['parity']);print(json.dumps({k:manifest[k] for k in ['status','candidate_onnx_sha256','candidate_deploy_sha256','graph_proto_sha256','parity']},indent=2))
if __name__=='__main__':main()
