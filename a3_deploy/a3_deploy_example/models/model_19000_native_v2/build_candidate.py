"""Reproducible native GRU + genuine frozen reference graph; no training/plant launch."""
import ast,copy,hashlib,importlib.util,json,math,os,re,sys,zipfile
from pathlib import Path
from types import SimpleNamespace as NS
sys.dont_write_bytecode=True
import numpy as np
import onnxruntime as ort
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];OLD=HERE.with_name('model_19000');PRIOR=HERE.with_name('model_19000_native')
ARCHIVE=ROOT.parent/'hope-stationary-gru-evaluations/20261106-eight-checkpoint-comparison/model_19000'
EVIDENCE=ROOT.parent/'hope-stationary-gru-evaluations/20261108-model19000-native-v2'
SOURCE='33c609022efbb4e85fe6a58a5a8947b0fbd46c76';PT_SHA='744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0'
ONNX_SHA='f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda'
UTIL=ROOT/'hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2)+'\n')
def module(path,name):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);sys.modules[name]=m;s.loader.exec_module(m);return m
sys.path.insert(0,str(OLD/'config_export'));from saved_checkpoint_reader import Reader
cfgmod=module(OLD/'config_export/export_config.py','frozen_cfg')
wire=module(PRIOR/'prepare_candidate.py','metadata_wire')
class Attr(dict):
 def __getattr__(self,key):
  if key=='to_dict':return lambda:dict(self)
  try:return self[key]
  except KeyError:raise AttributeError(key)
def attr(x):
 if isinstance(x,dict):return Attr({k:attr(v) for k,v in x.items()})
 if isinstance(x,list):return [attr(v) for v in x]
 return x
class Tensor:
 def __init__(self,x):self.x=np.asarray(x)
 def __getitem__(self,x):return Tensor(self.x[x])
 def cpu(self):return self
 def detach(self):return self
 def tolist(self):return self.x.tolist()
def tensor(x):return Tensor(x.numpy()) if hasattr(x,'numpy') else x

# Protobuf encoding limited to standard ONNX tensor/value/node records. Original nodes are copied byte-for-byte.
def vi(number,n):return wire.varint(number*8)+wire.varint(n)
def blob(number,b):return wire.varint(number*8+2)+wire.varint(len(b))+b
def string(number,s):return blob(number,s.encode())
def value_info(name,shape):
 dimensions=b''.join(blob(1,vi(1,n)) for n in shape)
 tensor_type=vi(1,1)+blob(2,dimensions)
 return string(1,name)+blob(2,blob(1,tensor_type))
def initializer(name,array):
 a=np.ascontiguousarray(array);dtype=1 if a.dtype==np.float32 else 7
 assert a.dtype in (np.dtype('float32'),np.dtype('int64'))
 return b''.join(vi(1,int(n)) for n in a.shape)+vi(2,dtype)+string(8,name)+blob(9,a.tobytes())
def node(op,inputs,outputs,name,attributes=b''):
 return b''.join(string(1,x) for x in inputs)+b''.join(string(2,x) for x in outputs)+string(3,name)+string(4,op)+attributes

class RemoveImports(ast.NodeTransformer):
 def visit_ImportFrom(self,node):return None
 def visit_Import(self,node):return None

def metadata(env,task,checkpoint):
 """Execute the actual training metadata function, omitting only graph I/O, with data-only saved managers."""
 exporter=UTIL/'utils/exporter.py';tree=ast.parse(exporter.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='attach_onnx_metadata');cut=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(getattr(t,'id','')=='model' for t in n.targets) and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='onnx.load')
 fn.body=fn.body[:cut]+[ast.Return(value=ast.Name(id='metadata',ctx=ast.Load()))];fn=RemoveImports().visit(fn)
 robots=ast.parse((UTIL/'robots/agibot_a3.py').read_text());names=['A3_DEPLOY_JOINT_KD_PATTERNS','A3_PASSIVE_JOINT_DAMPING_PATTERNS','_A3_MARKER_JOINTS'];nodes=[]
 for n in robots.body:
  if isinstance(n,(ast.Assign,ast.AnnAssign)):
   target=n.target if isinstance(n,ast.AnnAssign) else n.targets[0]
   if getattr(target,'id','') in names:nodes.append(n)
  elif isinstance(n,ast.FunctionDef) and n.name in ['a3_deploy_joint_kd','a3_passive_joint_damping']:nodes.append(RemoveImports().visit(n))
 contracts=module(UTIL/'tasks/tracking/actor_observation_contract.py','native_contracts');sim=module(UTIL/'utils/sim2real_export.py','native_sim2real');manifest=module(UTIL/'utils/a3_deploy_manifest.py','native_manifest_utils')
 ns={'os':os,'json':json,'math':math,'re':re,'hashlib':hashlib,'Mapping':dict,'Any':object,'_sha256_file':sha,'RECIPES':sim.RECIPES,'sim2real_metadata':sim.sim2real_metadata,'infer_actor_observation_contract':contracts.infer_actor_observation_contract,'removed_terms_vs_full':contracts.removed_terms_vs_full}
 ns.update({k:getattr(manifest,k) for k in dir(manifest) if not k.startswith('__')})
 future=ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)
 exec(compile(ast.fix_missing_locations(ast.Module(body=[future,*nodes,fn],type_ignores=[])),str(exporter),'exec'),ns)
 result=ns['attach_onnx_metadata'](env,'gru-formal-2b8c09938a',str(EVIDENCE),checkpoint_path=str(checkpoint),task_identity=task)
 return {k:manifest.metadata_wire_value(v) for k,v in result.items()}

def build():
 EVIDENCE.mkdir(exist_ok=True,parents=True)
 original=OLD/'policy/exported/actor.onnx';checkpoint=ARCHIVE/'model_19000.pt';assert sha(original)==ONNX_SHA and sha(checkpoint)==PT_SHA
 preserve={str(p.relative_to(ROOT)):sha(p) for base in [OLD,PRIOR] for p in base.rglob('*') if p.is_file()};save(EVIDENCE/'preserved-files-before.json',preserve)
 cfg=yaml.load((ARCHIVE/'params/env.yaml').read_text(),Loader=cfgmod.ConfigLoader);recipe=yaml.safe_load((ARCHIVE/'params/task_resolved.yaml').read_text())['task'];old=json.loads((OLD/'policy/exported/manifest.json').read_text());deploy=yaml.safe_load((OLD/'policy/params/deploy.yaml').read_text())
 with zipfile.ZipFile(checkpoint) as z:
  name=next(n for n in z.namelist() if n.endswith('/data.pkl'));saved=Reader(z,name.rsplit('/',1)[0],z.read(name)).load();assert saved['iter']==19000 and saved['hope_exact_resume_state']['wandb_run_id']=='gru-formal-2b8c09938a';e=saved['hope_exact_resume_state']['environment_resume_state'];terms=e['managers']['command_manager']['terms'];f=e['managers']['action_manager']['terms']['joint_pos']['fields'];indexes=terms['motion']['fields']['body_indexes'].numpy().tolist()
  schemas=module(UTIL/'utils/motion_schema.py','native_motion_schema');arrays={k:[] for k in ['joint_pos','joint_vel','body_pos_w','body_quat_w','body_lin_vel_w','body_ang_vel_w']};assets=[]
  for file,expected in zip(old['motion_cfg']['motion_file'],old['source_runtime_manifest']['motion_sha256']):
   path=ROOT/file;assert sha(path)==expected
   with np.load(path) as data:
    for key in arrays:
     value=data[key]
     if key.startswith('body_'):value=schemas.select_motion_bodies(value,indexes,str(path),key)
     arrays[key].append(np.asarray(value,dtype=np.float32))
    assets.append({'path':file,'sha256':expected,'frames':len(data['joint_pos']),'fps':float(np.asarray(data['fps']).ravel()[0])})
  lengths=[a['frames'] for a in assets];arrays={k:np.concatenate(v,axis=0) for k,v in arrays.items()};assert all(x['fps']==50 for x in assets)
  # Training can apply explicit HOLD q to action defaults; it is already resolved in frozen env nominal init_state.
  data={k:tensor(v) for k,v in e['physics_properties']['robot']['data'].items()};data.update(joint_names=f['_joint_names'],default_joint_pos_nominal=Tensor(deploy['default_joint_pos']),joint_effort_limits=Tensor([old['actuator']['effort_limit']]))
  action=NS(**{k:tensor(v) for k,v in f.items()});action.passive_joint_names=f['_passive_joint_names']
  actions=ast.parse((UTIL/'tasks/tracking/mdp/hope_actions.py').read_text());cls=next(n for n in actions.body if isinstance(n,ast.ClassDef) and n.name=='V11SafeClampedJointPositionAction');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='resolved_contract_metadata');ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'saved_action_metadata','exec'),ns);action.resolved_contract_metadata=lambda:ns['resolved_contract_metadata'](action)
  motion=NS(cfg=attr(cfg['commands']['motion']),motion=NS(**{k:Tensor(v) for k,v in arrays.items()},seg_len=Tensor(lengths),num_segments=len(lengths)))
  commands={'motion':motion,'racket_target':NS(cfg=attr(cfg['commands']['racket_target']))}
  obsrows=old['observation_layout'];obs_names=[x['name'] for x in obsrows];obs_dims=[x['slice'][1]-x['slice'][0] for x in obsrows]
  env=NS(cfg=attr(cfg),scene={'robot':NS(data=NS(**data))},action_manager=NS(get_term=lambda _:action),command_manager=NS(get_term=lambda n:commands[n],active_terms=list(cfg['commands'])),observation_manager=NS(active_terms={'policy':obs_names},cfg=attr(cfg['observations']),group_obs_term_dim={'policy':obs_dims},group_obs_dim={'policy':110}),step_dt=.02)
  md=metadata(env,recipe,checkpoint)
 raw=original.read_bytes();graph=next(p for n,w,p,b in wire.fields(raw) if n==7);assert b'_hope19000_ref_' not in graph
 ext=blob(11,value_info('time_step',[1,1]))
 prefix='_hope19000_ref_';cast=string(1,'to')+vi(3,7)+vi(20,2)
 ext+=blob(1,node('Cast',['time_step'],[prefix+'cast'],prefix+'cast_node',blob(5,cast)))
 ext+=blob(5,initializer(prefix+'shape',np.array([1],np.int64)))+blob(1,node('Reshape',[prefix+'cast',prefix+'shape'],[prefix+'flat'],prefix+'reshape'))
 ext+=blob(5,initializer(prefix+'last',np.array([len(arrays['joint_pos'])-1],np.int64)))+blob(1,node('Min',[prefix+'flat',prefix+'last'],[prefix+'index'],prefix+'clamp_max'))
 for key,value in arrays.items():
  ext+=blob(5,initializer(prefix+key,value))+blob(1,node('Gather',[prefix+key,prefix+'index'],[key],prefix+'gather_'+key))+blob(12,value_info(key,[1,*value.shape[1:]]))
 newgraph=graph+ext;base=b''.join(blob(7,newgraph) if n==7 else field for n,w,p,field in wire.fields(raw) if n!=14);wrapped=wire.append_metadata(base,md)
 output=HERE/'policy/exported/policy.onnx';output.parent.mkdir(parents=True,exist_ok=True)
 if output.exists():assert output.read_bytes()==wrapped,'Different existing v2 graph preserved; use a new candidate version'
 else:output.write_bytes(wrapped)
 deploy['provenance']['policy_sha256']=sha(output);deploy['provenance']['export_source']='exact_model19000_actor_plus_hash_bound_training_motion_references';dy=HERE/'policy/params/deploy.yaml';dy.parent.mkdir(parents=True,exist_ok=True);dy.write_text(yaml.safe_dump(deploy,sort_keys=False))
 session=ort.InferenceSession(str(output),providers=['CPUExecutionProvider']);baseline=ort.InferenceSession(str(original),providers=['CPUExecutionProvider']);assert session.get_modelmeta().custom_metadata_map==md
 obs=np.load(OLD/'policy/exported/parity_vectors.npz')['obs'];trace=ARCHIVE/'attempt-kaub5j2j/scored/policy-trace.jsonl';recorded=[json.loads(x) for x in trace.read_text().splitlines()];recorded=[r['observation'] for r in recorded if r.get('observation') is not None];obs=np.concatenate([obs,np.asarray(recorded,dtype=np.float32)[np.linspace(0,len(recorded)-1,256,dtype=int)]])
 h=np.random.default_rng(43).normal(0,.1,(1,1,110)).astype(np.float32);h2=h.copy();initial=h.copy();act=[];states=[];ae=he=0
 for i,x in enumerate(obs):
  if i in [32,192]:h.fill(0);h2.fill(0)
  a,h=baseline.run(['actions','h_out'],{'obs':x.reshape(1,110),'h_in':h});b,h2=session.run(['actions','h_out'],{'obs':x.reshape(1,110),'h_in':h2,'time_step':np.array([[i]],np.float32)});ae=max(ae,float(abs(a-b).max()));he=max(he,float(abs(h-h2).max()));np.testing.assert_allclose(a,b,atol=2e-5,rtol=2e-4);np.testing.assert_allclose(h,h2,atol=2e-5,rtol=2e-4);act.append(a);states.append(h.copy())
 ref_frames=sorted(set([-1,0,1,*np.cumsum(lengths).tolist(),*[x-1 for x in np.cumsum(lengths)],len(arrays['joint_pos'])+10]));referr=0
 for index in ref_frames:
  refs=session.run(list(arrays),{'obs':np.zeros((1,110),np.float32),'h_in':initial,'time_step':np.array([[index]],np.float32)})
  for key,value in zip(arrays,refs):
   expected=arrays[key][min(index,len(arrays[key])-1)][None];referr=max(referr,float(abs(value-expected).max()));np.testing.assert_array_equal(value,expected)
 np.savez_compressed(EVIDENCE/'reference-arrays.npz',**arrays,body_indexes=indexes,clip_lengths=lengths)
 obs.astype('<f4').tofile(EVIDENCE/'obs.f32');initial.astype('<f4').tofile(EVIDENCE/'initial_h.f32');np.asarray(act,np.float32).tofile(EVIDENCE/'actions.f32');np.asarray(states,np.float32).tofile(EVIDENCE/'hidden.f32')
 # Reference stream for native exact-output verification at the same clock used by the parity driver.
 with (EVIDENCE/'refs.f32').open('wb') as file:
  for i in range(len(obs)):
   for key in ['joint_pos','joint_vel','body_pos_w','body_quat_w']:np.asarray(arrays[key][min(i,len(arrays[key])-1)],dtype='<f4').tofile(file)
 after={str(p.relative_to(ROOT)):sha(p) for base in [OLD,PRIOR] for p in base.rglob('*') if p.is_file()};assert after==preserve
 graphfields=list(wire.fields(newgraph));oldfields=list(wire.fields(graph));assert graphfields[:len(oldfields)]==oldfields
 report={'status':'IMPLEMENTED_CANDIDATE_NATIVE_BUILD_PENDING','run_id':'gru-formal-2b8c09938a','training_source':SOURCE,'checkpoint_sha256':PT_SHA,'original_actor_sha256':ONNX_SHA,'candidate_onnx_sha256':sha(output),'deploy_yaml_sha256':sha(dy),'original_nodes_and_initializers_preserved':True,'actor_parity':{'steps':len(obs),'initial_nonzero_hidden':True,'reset_ticks':[32,192],'max_action_error':ae,'max_hidden_error':he,'atol':2e-5,'rtol':2e-4},'reference_parity':{'sample_frames':[int(x) for x in ref_frames],'max_abs_error':referr,'semantics':'concat clips; time_step float->int64 truncation; clamp maximum; integer Gather; no interpolation','body_indexes':indexes,'body_names':cfg['commands']['motion']['body_names'],'frames':'original saved motion world frame; native root/anchor consumers unchanged'},'reference_assets':assets,'native_verification':'PENDING','formal_gate3':'NOT_RUN','metadata_exporter_source_sha256':sha(UTIL/'utils/exporter.py'),'files_sha256':{str(p.relative_to(EVIDENCE)):sha(p) for p in EVIDENCE.glob('*.f32')}}
 report['configuration_sources']={str(p.relative_to(ROOT.parent)):sha(p) for p in (ARCHIVE/'params').glob('*.yaml')}
 report['software_versions']={'python':sys.version,'numpy':np.__version__,'onnxruntime_python':ort.__version__,'yaml':yaml.__version__}
 save(HERE/'manifest.json',report);save(EVIDENCE/'candidate-verification.json',report);save(EVIDENCE/'preserved-files-after.json',after);print(json.dumps(report,indent=2))
if __name__=='__main__':build()
