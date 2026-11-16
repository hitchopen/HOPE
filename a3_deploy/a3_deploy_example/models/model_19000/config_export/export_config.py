"""Export model19000 deployment YAML only; never rewrite ONNX or start simulation."""
import argparse,ast,hashlib,importlib.util,json,math,re,sys,zipfile
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import onnxruntime as ort
import yaml
from saved_checkpoint_reader import Reader
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
MODEL=HERE.parent
PT_SHA='744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0'
ONNX_SHA='f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda'
SOURCE='33c609022efbb4e85fe6a58a5a8947b0fbd46c76'
REF_COMMIT='51a9f098dd990a2d59f0c1fdd831d5103fae18fa'
UTILS=ROOT/'hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
class ConfigLoader(yaml.SafeLoader):pass
def tagged(loader,suffix,node):
 if isinstance(node,yaml.SequenceNode):return loader.construct_sequence(node)
 if isinstance(node,yaml.MappingNode):return loader.construct_mapping(node)
 return loader.construct_scalar(node)
ConfigLoader.add_multi_constructor('tag:yaml.org,2002:python/',tagged)
def module(path,name):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def configuration_builder():
 """Reuse original builder body; supply recovered metadata instead of editing ONNX."""
 tree=ast.parse((HERE/'reference_export_deploy_cfg.py').read_text());nodes=[]
 for node in tree.body:
  if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SDK_JOINT_NAMES' for t in node.targets):nodes.append(node)
  elif isinstance(node,ast.FunctionDef) and node.name in ['_csv','_finite_vector','build_deploy_cfg']:
   if node.name=='build_deploy_cfg':
    # The first3 statements only read ONNX and metadata. Keep every validation/output statement.
    assert len(node.body)>=3 and [getattr(n.targets[0],'id',None) for n in node.body[:2]]==['model','metadata']
    node.body=node.body[3:];node.args.args.extend([ast.arg(arg='metadata'),ast.arg(arg='obs_dim'),ast.arg(arg='action_dim')])
   nodes.append(node)
 ns={'Any':object,'Path':Path,'math':math,'json':json,'_sha256':sha,'_load_frozen_env':lambda p:yaml.load(p.read_text(),Loader=ConfigLoader)}
 exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),str(HERE/'reference_export_deploy_cfg.py'),'exec'),ns)
 return ns

def patterns_from_source():
 p=UTILS/'robots/agibot_a3.py';tree=ast.parse(p.read_text());values={}
 for n in tree.body:
  if isinstance(n,ast.AnnAssign) and getattr(n.target,'id','') in ['A3_DEPLOY_JOINT_KD_PATTERNS','A3_PASSIVE_JOINT_DAMPING_PATTERNS']:values[n.target.id]=ast.literal_eval(n.value)
 assert len(values)==2
 return p,values

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--checkpoint',type=Path,required=True);ap.add_argument('--params',type=Path,required=True);ap.add_argument('--output',type=Path,default=MODEL/'policy/params/deploy.yaml');ap.add_argument('--report',type=Path,default=MODEL/'deploy-config-verification.json');a=ap.parse_args()
 assert sha(a.checkpoint)==PT_SHA
 onnx=MODEL/'policy/exported/actor.onnx';assert sha(onnx)==ONNX_SHA
 manifest=json.loads((MODEL/'policy/exported/manifest.json').read_text());assert manifest['checkpoint_sha256']==PT_SHA and manifest['onnx_sha256']==ONNX_SHA
 cfg=yaml.load((a.params/'env.yaml').read_text(),Loader=ConfigLoader);recipe=yaml.safe_load((a.params/'task_resolved.yaml').read_text());provenance=json.loads((MODEL/'policy/params/training-config-provenance.json').read_text())
 for n,h in provenance['original_training_config_sha256'].items():assert sha(a.params/n)==h
 source,patterns=patterns_from_source()
 # Bind the wire/passive contract to the training source, not another model.
 import subprocess
 for source_file in [source,UTILS/'utils/sim2real_export.py',UTILS/'tasks/tracking/actor_observation_contract.py',UTILS/'utils/exporter.py']:
  assert subprocess.check_output(['git','-C',str(ROOT),'show',SOURCE+':'+str(source_file.relative_to(ROOT))])==source_file.read_bytes()
 checks={};rows=[]
 with zipfile.ZipFile(a.checkpoint) as z:
  name=next(n for n in z.namelist() if n.endswith('/data.pkl'));saved=Reader(z,name.rsplit('/',1)[0],z.read(name)).load();state=saved['hope_exact_resume_state'];assert saved['iter']==19000 and state['wandb_run_id']=='gru-formal-2b8c09938a'
  agent=yaml.safe_load((a.params/'agent.yaml').read_text());assert not agent['policy']['actor_obs_normalization'] and manifest['normalization']=='identity'
  assert not any(k.startswith('actor_obs_normalizer.') for k in saved['model_state_dict'])
  e=state['environment_resume_state'];f=e['managers']['action_manager']['terms']['joint_pos']['fields'];physical=e['physics_properties']['robot'];data=physical['data'];names=f['_joint_names']
  def row(x):
   v=x.numpy();return (v[0] if v.ndim>1 else v).tolist()
  def same(label,actual,expected):
   np.testing.assert_array_equal(actual,expected);checks[label]={'status':'PASS','actual':actual,'source_values':expected}
  action=manifest['action'];same('joint_order',names,manifest['joint_names'])
  assert f['_actual_q_hard_tolerance_rad']==cfg['actions']['joint_pos']['actual_q_hard_tolerance_rad']
  assert f['_action_contract']==cfg['actions']['joint_pos']['action_contract']
  assert cfg['observations']['policy']['history_length'] is None
  same('joint_ids',row(f['_action_joint_ids']),list(range(31)))
  for key,field in [('offset','_offset'),('scale','_scale'),('lower','_safe_lo'),('upper','_safe_hi')]:same('action.'+key,action[key],row(f[field]))
  for key,field in [('default_q','default_joint_pos'),('default_qd','default_joint_vel')]:same('action.'+key,action[key],row(data[field]))
  for key,field in [('kp','default_joint_stiffness'),('kd_total','default_joint_damping')]:same('actuator.'+key,manifest['actuator'][key],row(data[field]))
  hardlo=row(f['_hard_lo']);hardhi=row(f['_hard_hi']);kp=row(data['default_joint_stiffness']);train_kd=row(data['default_joint_damping']);default=row(f['_offset']);scale=row(f['_scale'])
  def resolve(table,name):
   matches=[v for regex,v in table if re.fullmatch(regex,name)];assert len(matches)==1;return matches[0]
  snapshot_default=default.copy();default=[float(np.float32(resolve(list(cfg['scene']['robot']['init_state']['joint_pos'].items()),n))) for n in names]
  dr=cfg['events']['add_joint_default_pos']['params'];assert dr['operation']=='add';delta=np.array(snapshot_default)-default;assert np.all(delta>=dr['pos_distribution_params'][0]-1e-7) and np.all(delta<=dr['pos_distribution_params'][1]+1e-7)
  wire=[resolve(patterns['A3_DEPLOY_JOINT_KD_PATTERNS'],n) for n in names];passive=[resolve(patterns['A3_PASSIVE_JOINT_DAMPING_PATTERNS'],n) for n in names];np.testing.assert_allclose(train_kd,np.array(wire)+passive,rtol=0,atol=1e-6)
  resolved={'qdes_action_contract':f['_action_contract'],'qdes_policy_feedback_contract':'legacy_applied_raw_v1','qdes_delay_min_steps':f['_qdes_delay_min_steps'],'qdes_delay_max_steps':f['_qdes_delay_max_steps'],'qdes_delay_nominal_fraction':f['_qdes_delay_nominal_fraction'],'qdes_joint_names':names}
  mock=NS(action_manager=NS(get_term=lambda _:NS(passive_joint_names=f['_passive_joint_names'],resolved_contract_metadata=lambda:resolved)),command_manager=NS(get_term=lambda n:NS(cfg=NS(**cfg['commands'][n]))),scene={'robot':NS(data=NS(joint_names=names))},step_dt=e['step_dt'])
  sim2real=module(UTILS/'utils/sim2real_export.py','deploy_sim2real_metadata');md=sim2real.sim2real_metadata(mock,recipe['task'])
  contract_module=module(UTILS/'tasks/tracking/actor_observation_contract.py','deploy_actor_contract');contract=contract_module.get_actor_observation_contract('hitter_pure') if hasattr(contract_module,'get_actor_observation_contract') else contract_module.HITTER_PURE
  terms=contract.terms;assert [(t.name,t.dim) for t in terms]==[(x['name'],x['slice'][1]-x['slice'][0]) for x in manifest['observation_layout']]
  for term,layout in zip(terms,manifest['observation_layout']):
   c=cfg['observations']['policy'][term.name];assert c['func']==layout['source'];assert c['clip'] is None and c['scale'] is None and c['history_length']==0
  csv=lambda x:','.join(str(v) for v in x)
  md.update(joint_names=csv(names),default_joint_pos=csv(default),action_scale=csv(scale),joint_stiffness=csv(kp),joint_damping=csv(wire),qdes_safe_lower_rad=csv(row(f['_safe_lo'])),qdes_safe_upper_rad=csv(row(f['_safe_hi'])),qdes_hard_lower_rad=csv(hardlo),qdes_hard_upper_rad=csv(hardhi),observation_names=csv([t.name for t in terms]),actor_obs_term_dims=csv([t.dim for t in terms]),observation_history_lengths=csv([1]*len(terms)),actor_obs_term_sources_json=json.dumps({t.name:t.deploy_source for t in terms}),actor_obs_contract='hitter_pure',qdes_action_contract=f['_action_contract'],qdes_policy_feedback_contract=resolved['qdes_policy_feedback_contract'],qdes_actual_q_hard_tolerance_rad=str(f['_actual_q_hard_tolerance_rad']),hitter_pure_checkpoint_sha256=PT_SHA)
  assert cfg['sim']['dt']==e['physics_dt']==manifest['physics_dt'] and cfg['decimation']==e['decimation']==manifest['decimation'] and e['step_dt']==manifest['control_dt']==.02
  ns=configuration_builder()
  backend_ast=ast.parse((UTILS/'utils/a3_deploy_manifest.py').read_text());backend=next(ast.literal_eval(n.value) for n in backend_ast.body if isinstance(n,ast.Assign) and any(getattr(t,'id','')=='A3_BACKEND_JOINT_ORDER' for t in n.targets));assert list(backend)==ns['SDK_JOINT_NAMES']
  doc=ns['build_deploy_cfg'](onnx,a.params/'env.yaml',md,110,31)
  doc['provenance']['export_source']='model_19000_saved_checkpoint_and_frozen_config_configuration_only'
  for i,n in enumerate(names):
   sdk=doc['joint_ids_map'][i];assert doc['joint_sdk_names'][sdk]==n;assert doc['stiffness'][sdk]==kp[i] and doc['damping'][sdk]==wire[i]
   rows.append({'policy_index':i,'sdk_index':sdk,'joint':n,'default_q':default[i],'default_qd':row(data['default_joint_vel'])[i],'action_offset':default[i],'checkpoint_env0_offset':snapshot_default[i],'env0_randomization_delta':float(delta[i]),'action_scale':scale[i],'hard_lower':hardlo[i],'hard_upper':hardhi[i],'safe_lower':action['lower'][i],'safe_upper':action['upper'][i],'kp':kp[i],'kd_training_total':train_kd[i],'kd_wire':wire[i],'kd_passive':passive[i],'effort_limit':manifest['actuator']['effort_limit'][i],'armature':manifest['actuator']['armature'][i],'friction':manifest['actuator']['friction'][i]})
  for label,field in [('effort_limit','effort_limit_sim'),('armature','armature'),('friction','friction')]:
   values=[None]*31
   for group in physical['actuators'].values():
    for idx,val in zip(row(group['fields']['_joint_indices']),row(group['fields'][field])):values[int(idx)]=val
   same('actuator.'+label,manifest['actuator'][label],values)
  # Resolve original saved regex defaults/scales and actuator arrays independently.
  for i,n in enumerate(names):
   for label,spec,expected in [('initial_position',cfg['scene']['robot']['init_state']['joint_pos'],default[i]),('action_scale',cfg['actions']['joint_pos']['scale'],scale[i])]:
    value=resolve(list(spec.items()),n);np.testing.assert_allclose(np.float32(value),expected,rtol=0,atol=1e-7)
   groups=[g for g in cfg['scene']['robot']['actuators'].values() if any(re.fullmatch(expr,n) for expr in g['joint_names_expr'])];assert len(groups)==1
   for key,expected in [('stiffness',kp[i]),('damping',train_kd[i])]:
    value=groups[0][key];value=resolve(list(value.items()),n) if isinstance(value,dict) else value;np.testing.assert_allclose(np.float32(value),expected,rtol=0,atol=1e-6)
  a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(yaml.safe_dump(doc,sort_keys=False));assert yaml.safe_load(a.output.read_text())==doc
 session=ort.InferenceSession(str(onnx),providers=['CPUExecutionProvider']);signature={'inputs':{v.name:{'shape':v.shape,'type':v.type} for v in session.get_inputs()},'outputs':{v.name:{'shape':v.shape,'type':v.type} for v in session.get_outputs()}}
 assert all(v['type']=='tensor(float)' for group in signature.values() for v in group.values())
 assert {k:v['shape'] for k,v in signature['inputs'].items()}=={'obs':[1,110],'h_in':[1,1,110]};assert {k:v['shape'] for k,v in signature['outputs'].items()}=={'actions':[1,31],'h_out':[1,1,110]}
 assert sha(onnx)==ONNX_SHA and sha(a.checkpoint)==PT_SHA
 report={'configuration_status':'EXPORTED_AND_CHECKED','receiving_hardware_runner_compatibility':'NOT_YET_VERIFIED','checkpoint_iteration':19000,'run_id':state['wandb_run_id'],'checkpoint_sha256':PT_SHA,'onnx_sha256':ONNX_SHA,'deploy_yaml_sha256':sha(a.output),'onnx_signature':signature,'joint_rows':rows,'observation_rows':[{'name':t.name,'slice':l['slice'],'dim':t.dim,'training_function':cfg['observations']['policy'][t.name]['func'],'training_scale':cfg['observations']['policy'][t.name]['scale'],'training_clip':cfg['observations']['policy'][t.name]['clip'],'training_history_length':cfg['observations']['policy'][t.name]['history_length'],'deployment_history_length':1,'deployment_source':t.deploy_source} for t,l in zip(terms,manifest['observation_layout'])],'checks':checks,'control_timing':{'physics_dt':.005,'decimation':4,'control_dt':.02,'rate_hz':50},'training_action_delay':resolved,'normalization':manifest['normalization'],'default_position_contract':{'exported':'saved nominal pose, not randomized checkpoint env0','startup_distribution':dr['pos_distribution_params'],'max_abs_env0_delta_rad':float(np.max(abs(delta))),'matches_existing_evaluation_manifest_default':False},'source_fields':{'default_joint_pos':'nominal saved env scene.robot.init_state.joint_pos cast float32, as standard exporter default_joint_pos_nominal; checkpoint env0 default/action offset separately audited against startup DR bounds','gains':'checkpoint physics_properties.robot.data.default_joint_stiffness/default_joint_damping; wire/passive source table with training total invariant','hard_safe_limits':'checkpoint action_manager.terms.joint_pos.fields._hard_lo/_hard_hi/_safe_lo/_safe_hi','observations':'saved env.observations.policy; training actor_observation_contract; verified manifest layout','mapping':'checkpoint action _joint_names and _action_joint_ids; reference exporter SDK_JOINT_NAMES','runtime_contract':'saved cfg adapted to existing sim2real_metadata pure function','provenance':'exact PT/ONNX/env SHA256'},'reference_exporter':{'repository':'nohope cached local Git history (read-only source recovery)','commit':REF_COMMIT,'path':'hope_training/whole_body_tracking/scripts/export_deploy_cfg.py','sha256':sha(HERE/'reference_export_deploy_cfg.py'),'adaptation':'AST removes only3 ONNX/metadata read statements; configuration logic unchanged; supplies actual saved metadata in memory, no graph changes'},'sources_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [source,UTILS/'utils/exporter.py',UTILS/'utils/sim2real_export.py',UTILS/'tasks/tracking/actor_observation_contract.py',HERE/'export_config.py',HERE/'saved_checkpoint_reader.py']},'saved_params_sha256':{n:sha(a.params/n) for n in ['env.yaml','agent.yaml','task_resolved.yaml']},'schema_expressiveness_limits':['GRU hidden-state ABI/carry/reset described in companion document, not unsupported loader fields','effort/armature/friction/default joint velocities and training delay are evidence fields in this report; standard deploy schema has no actuator effort/armature/friction arrays or recurrent state configuration']}
 save(a.report,report);print(json.dumps({'configuration_status':report['configuration_status'],'receiving_hardware_runner_compatibility':report['receiving_hardware_runner_compatibility'],'deploy_yaml':str(a.output),'deploy_yaml_sha256':report['deploy_yaml_sha256'],'onnx_sha256':ONNX_SHA},indent=2))
if __name__=='__main__':main()
