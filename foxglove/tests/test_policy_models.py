import hashlib,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'a3'))
from hope_policy_models import handle
from hope_field_operations import FieldOperations,ManagedRecorder
from hope_lifecycle_core import LifecycleConfig
class PolicyModelsTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
  self.roots={h:self.root/h for h in ('hdu','mdu')}
  for root in self.roots.values():
   root.mkdir(); models={}
   for name in ('model14600','model11999','model12600'):
    f=root/(name+'.bin');f.write_bytes(name.encode());d=hashlib.sha256(f.read_bytes()).hexdigest()
    models[name]={'files':{str(f):d},'policy_sha256':d,'deploy_sha256':d}
   (root/'catalog.json').write_text(json.dumps({'version':1,'models':models}))
  self.calls=[];self.fail=False
  self.field=FieldOperations(self.root/'field',self.exchange,recorder=ManagedRecorder(self.root/'records'))
  self.config=LifecycleConfig(revision=1,table_side='P1')
 def exchange(self,host,config,request):
  self.calls.append((host,request.copy()))
  if request['op']=='verify_ball_model':return {'configured_model':'ppq004'}
  if request['op']=='verify_csv':return {'sha256':request['sha256']}
  if host=='mdu' and self.fail and request['op']=='select_policy_model' and request['model']=='model11999':
   handle(self.roots[host],request) # uncertain write committed before timeout
   raise ValueError('timeout after write')
  return handle(self.roots[host],request)
 def select(self,model,stopped=True):return self.field.command({'op':'set_policy_model','model':model},self.config,stopped=stopped)
 def test_pair_selection_persists_and_start_verifies(self):
  self.select('model11999');self.assertEqual(self.field.runner_selection(self.config),('OPTITRACK','DEFAULT'))
  self.assertEqual(json.loads((self.root/'field/selection.json').read_text())['policy_model'],'model11999')
  self.select('model14600');self.assertEqual(self.field.status()['policy_model'],'model14600')
 def test_uncertain_second_write_rolls_back_both(self):
  self.select('model14600');self.fail=True
  with self.assertRaisesRegex(ValueError,'timeout'):self.select('model11999')
  for root in self.roots.values():self.assertEqual(handle(root,{'op':'read_policy_model'})['model'],'model14600')
  self.assertEqual(self.field.status()['policy_model'],'model14600')
 def test_running_rejects_before_writes(self):
  with self.assertRaisesRegex(ValueError,'stop'):self.select('model11999',False)
  self.assertEqual(self.calls,[])
 def test_local_process_guard(self):
  with patch('hope_policy_models.require_stopped',side_effect=ValueError('stop Runner')):
   with self.assertRaisesRegex(ValueError,'stop Runner'):handle(self.roots['mdu'],{'op':'select_policy_model','model':'model11999'})
  self.assertFalse((self.roots['mdu']/'selected.json').exists())
 def test_tampered_bundle_rejected_before_selection(self):
  (self.roots['mdu']/'model11999.bin').write_bytes(b'tampered')
  with self.assertRaisesRegex(ValueError,'changed'):self.select('model11999')
  self.assertEqual(handle(self.roots['hdu'],{'op':'read_policy_model'})['model'],'model14600')
 def test_mismatched_endpoints_reject_start(self):
  self.select('model11999');handle(self.roots['mdu'],{'op':'select_policy_model','model':'model14600'})
  with self.assertRaisesRegex(ValueError,'differs'):self.field.runner_selection(self.config)
 def test_kernel_rejected_for11999_in_both_orders(self):
  self.select('model11999')
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  self.select('model14600');self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.select('model11999')
 def test_12600_selection_persists_and_cross_endpoint_verifies(self):
  self.select('model12600');self.assertEqual(self.field.runner_selection(self.config),('OPTITRACK','DEFAULT'))
  self.assertEqual(json.loads((self.root/'field/selection.json').read_text())['policy_model'],'model12600')
  for root in self.roots.values():self.assertEqual(handle(root,{'op':'read_policy_model'})['model'],'model12600')
 def test_12600_running_rejected_before_writes(self):
  with self.assertRaisesRegex(ValueError,'stop'):self.select('model12600',False)
  self.assertEqual(self.calls,[])
 def test_12600_tamper_prevents_selection(self):
  (self.roots['mdu']/'model12600.bin').write_bytes(b'tampered')
  with self.assertRaisesRegex(ValueError,'changed'):self.select('model12600')
  self.assertEqual(handle(self.roots['hdu'],{'op':'read_policy_model'})['model'],'model14600')
 def test_12600_kernel_rejected_in_both_orders(self):
  self.select('model12600')
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  self.select('model14600');self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.select('model12600')
 def install_serve_defaults(self):
  for root in self.roots.values():
   path=root/'catalog.json';cat=json.loads(path.read_text())
   for model in ('model14600','model12600'):
    cat['models'][model]['serve_csv']={'name':model+'.csv','sha256':model+'-csv'}
   path.write_text(json.dumps(cat))
 def test_serve_csv_follows_policy_and_default_button(self):
  self.install_serve_defaults()
  self.select('model12600');self.assertEqual(self.field.status()['csv']['name'],'model12600.csv')
  self.field._data['csv']={'name':'custom.csv'}
  self.field.command({'op':'use_default_csv'},self.config,stopped=True)
  self.assertEqual(self.field.status()['csv']['name'],'model12600.csv')
  self.select('model14600');self.assertEqual(self.field.status()['csv']['name'],'model14600.csv')
 def test_failed_selection_preserves_matched_csv(self):
  self.install_serve_defaults();self.select('model12600');before=self.field.status()['csv'];self.fail=True
  with self.assertRaisesRegex(ValueError,'timeout'):self.select('model11999')
  self.assertEqual(self.field.status()['policy_model'],'model12600')
  self.assertEqual(self.field.status()['csv'],before)
 def test_default_csv_rejects_mismatched_selected_policy(self):
  self.install_serve_defaults();self.select('model12600')
  handle(self.roots['mdu'],{'op':'select_policy_model','model':'model14600'})
  with self.assertRaisesRegex(ValueError,'differs'):
   self.field.command({'op':'use_default_csv'},self.config,stopped=True)
 def install_17400(self):
  for root in self.roots.values():
   path=root/'catalog.json';cat=json.loads(path.read_text());f=root/'model17400.bin';f.write_bytes(b'17400');d=hashlib.sha256(f.read_bytes()).hexdigest()
   cat['models']['model17400']={'files':{str(f):d},'policy_sha256':d,'deploy_sha256':d,'receive_only':True,'required_ball_model':'ppq004'}
   path.write_text(json.dumps(cat))
 def test_17400_persists_clears_serve_and_preserves_old_choices(self):
  self.install_17400();self.install_serve_defaults();self.select('model12600');self.select('model17400')
  self.assertIsNone(self.field.status()['csv'])
  reloaded=FieldOperations(self.root/'field',self.exchange,recorder=self.field.recorder)
  self.assertEqual(reloaded.status()['policy_model'],'model17400');self.assertEqual(reloaded.runner_selection(self.config),('OPTITRACK','DEFAULT'))
  for name in ('model14600','model11999','model12600','model17400'):self.select(name)
 def test_17400_requires_physics_and_mocap(self):
  self.install_17400();self.select('model17400')
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  original=self.field.exchange
  self.field.exchange=lambda h,c,q: {'configured_model':'ppq002'} if q['op']=='verify_ball_model' else original(h,c,q)
  with self.assertRaisesRegex(ValueError,'PPQ004'):self.field.runner_selection(self.config)
 def test_17400_missing_bundle_rolls_back(self):
  with self.assertRaisesRegex(ValueError,'not installed'):self.select('model17400')
  for root in self.roots.values():self.assertEqual(handle(root,{'op':'read_policy_model'})['model'],'model14600')
 def test_17400_uncertain_write_rolls_back_both_and_csv(self):
  self.install_17400();self.install_serve_defaults();self.select('model12600');old=self.field.status()['csv'];original=self.field.exchange
  def fail(h,c,q):
   result=original(h,c,q)
   if h=='mdu' and q['op']=='select_policy_model' and q['model']=='model17400':raise ValueError('17400 timeout after write')
   return result
  self.field.exchange=fail
  with self.assertRaisesRegex(ValueError,'timeout'):self.select('model17400')
  self.assertEqual(self.field.status()['csv'],old)
  for root in self.roots.values():self.assertEqual(handle(root,{'op':'read_policy_model'})['model'],'model12600')
 def test_17400_running_rejected_and_tampered_marker_rejected(self):
  self.install_17400()
  with self.assertRaisesRegex(ValueError,'stop'):self.select('model17400',False)
  self.assertEqual(self.calls,[])
  p=self.roots['mdu']/'catalog.json';cat=json.loads(p.read_text());cat['models']['model17400']['receive_only']=False;p.write_text(json.dumps(cat))
  with self.assertRaisesRegex(ValueError,'matched'):self.select('model17400')

 def install_serving(self, model):
  from hope_policy_models import SERVE_CSV_NAMES
  for root in self.roots.values():
   path=root/'catalog.json';cat=json.loads(path.read_text())
   f=root/(model+'.bin');f.write_bytes(model.encode());d=hashlib.sha256(f.read_bytes()).hexdigest()
   csv=root/SERVE_CSV_NAMES[model];csv.write_bytes(('matched-'+model).encode());c=hashlib.sha256(csv.read_bytes()).hexdigest()
   cat['models'][model]={'files':{str(f):d,str(csv):c},'policy_sha256':d,'deploy_sha256':d,
    'receive_only':False,'required_ball_model':'ppq004','serve_csv':{'name':csv.name,'sha256':c}}
   path.write_text(json.dumps(cat))

 def test_four_serve_models_pair_their_own_csv(self):
  for model in ('model14600','model12600','model17400','model12000'):
   with self.subTest(model=model):
    self.install_serving(model);self.select(model)
    status=self.field.status();self.assertTrue(status['policy_models'][model]['available'])
    digest=status['csv']['sha256'];self.assertEqual(self.field.runner_selection(self.config),('OPTITRACK',digest))
    self.field._data['csv']={'name':'other.csv','sha256':'a'*64}
    with self.assertRaisesRegex(ValueError,'matched'):self.field.runner_selection(self.config)

 def test_inventory_disables_missing_corrupt_and_mismatched_models(self):
  self.install_serving('model17400')
  first=self.field.refresh_policy_models(self.config)
  self.assertFalse(first['policy_models']['model12000']['available'])
  self.assertTrue(first['policy_models']['model17400']['available'])
  (self.roots['mdu']/'v23-17400.csv').write_bytes(b'corrupt')
  self.assertFalse(self.field.refresh_policy_models(self.config)['policy_models']['model17400']['available'])
  self.calls.clear()
  with self.assertRaisesRegex(ValueError,'changed'):self.select('model17400')
  self.assertFalse(any(q['op']=='select_policy_model' for _,q in self.calls))
  self.install_serving('model17400')
  p=self.roots['mdu']/'catalog.json';c=json.loads(p.read_text());c['models']['model17400']['policy_sha256']='f'*64;p.write_text(json.dumps(c))
  status=self.field.refresh_policy_models(self.config)['policy_models']['model17400']
  self.assertFalse(status['available']);self.assertIn('identities differ',status['reason'])

 def test_inventory_connection_loss_revokes_previous_availability(self):
  self.install_serving('model17400');self.field.refresh_policy_models(self.config)
  previous=self.field.exchange
  def disconnected(host, config, request):
   if host=='mdu':raise ValueError('MDU unavailable')
   return previous(host,config,request)
  self.field.exchange=disconnected
  status=self.field.refresh_policy_models(self.config)
  self.assertTrue(all(not e['available'] for e in status['policy_models'].values()))
  self.assertLess(status['policy_models_age_s'],1)

 def install_12000(self):
  for root in self.roots.values():
   path=root/'catalog.json';cat=json.loads(path.read_text())
   f=root/'model12000.bin';f.write_bytes(b'12000');d=hashlib.sha256(f.read_bytes()).hexdigest()
   csv=root/'v23-12000.csv';csv.write_bytes(b'matched12000');c=hashlib.sha256(csv.read_bytes()).hexdigest()
   cat['models']['model12000']={'files':{str(f):d,str(csv):c},'policy_sha256':d,'deploy_sha256':d,
    'receive_only':False,'required_ball_model':'ppq004','serve_csv':{'name':csv.name,'sha256':c}}
   path.write_text(json.dumps(cat))
 def test_12000_selection_pairs_csv_and_start_verifies(self):
  self.install_12000();self.select('model12000')
  self.assertEqual(self.field.status()['csv']['name'],'v23-12000.csv')
  digest=self.field.status()['csv']['sha256']
  self.assertEqual(self.field.runner_selection(self.config),('OPTITRACK',digest))
  self.field._data['csv']={'name':'custom.csv'}
  self.field.command({'op':'use_default_csv'},self.config,stopped=True)
  self.assertEqual(self.field.status()['csv']['sha256'],digest)
  self.install_17400();self.select('model17400');self.assertIsNone(self.field.status()['csv'])
  self.select('model12000');self.assertEqual(self.field.status()['csv']['sha256'],digest)
 def test_12000_requires_ppq004_and_mocap(self):
  self.install_12000();self.select('model12000')
  with self.assertRaisesRegex(ValueError,'OptiTrack'):self.field.command({'op':'set_mode','mode':'KERNEL'},self.config,stopped=True)
  original=self.field.exchange
  self.field.exchange=lambda h,c,q: {'configured_model':'ppq002'} if q['op']=='verify_ball_model' else original(h,c,q)
  with self.assertRaisesRegex(ValueError,'PPQ004'):self.field.runner_selection(self.config)
 def test_12000_rejects_missing_and_running_before_writes(self):
  with self.assertRaisesRegex(ValueError,'stop'):self.select('model12000',False)
  self.assertEqual(self.calls,[])
  with self.assertRaisesRegex(ValueError,'not installed'):self.select('model12000')
 def test_12000_rejects_unpaired_csv_before_start(self):
  self.install_12000();self.select('model12000')
  for csv in (None, {'name':'legacy.csv','sha256':'a'*64}):
   self.field._data['csv']=csv
   with self.assertRaisesRegex(ValueError,'policy default'):self.field.runner_selection(self.config)
  self.field.command({'op':'use_default_csv'},self.config,stopped=True)
  self.field.runner_selection(self.config)
 def test_12000_csv_tamper_rolls_back_both(self):
  self.install_12000();self.install_serve_defaults();self.select('model12600');before=self.field.status()['csv']
  (self.roots['mdu']/'v23-12000.csv').write_bytes(b'wrong model')
  with self.assertRaisesRegex(ValueError,'changed'):self.select('model12000')
  self.assertEqual(self.field.status()['csv'],before)
  for root in self.roots.values():self.assertEqual(handle(root,{'op':'read_policy_model'})['model'],'model12600')
 def test_12000_rejects_cross_endpoint_csv_mismatch_at_start(self):
  self.install_12000();self.select('model12000')
  p=self.roots['hdu']/'catalog.json';cat=json.loads(p.read_text())
  cat['models']['model12000']['serve_csv']['sha256']='a'*64;p.write_text(json.dumps(cat))
  with self.assertRaisesRegex(ValueError,'CSV differs'):self.field.runner_selection(self.config)
 def test_12000_rejects_missing_capability_and_wrong_default_csv(self):
  self.install_12000();p=self.roots['mdu']/'catalog.json';original=json.loads(p.read_text())
  for field,value in [('receive_only',True),('required_ball_model','ppq002'),('serve_csv',{'name':'v23-12600.csv','sha256':'a'*64})]:
   cat=json.loads(json.dumps(original));cat['models']['model12000'][field]=value;p.write_text(json.dumps(cat))
   with self.assertRaisesRegex(ValueError,'matched'):self.select('model12000')

 def test_fresh_robot_accepts_only_explicitly_installed_model(self):
  self.install_12000()
  for root in self.roots.values():
   p=root/'catalog.json';cat=json.loads(p.read_text())
   cat['models']={'model12000':cat['models']['model12000']};p.write_text(json.dumps(cat))
   (root/'selected.json').write_text(json.dumps({'model':'model12000'}))
  self.select('model12000');self.field.runner_selection(self.config)
  with self.assertRaisesRegex(ValueError,'not installed'):self.select('model14600')
  self.assertEqual(self.field.status()['policy_model'],'model12000')
  self.assertEqual(self.field.status()['csv']['name'],'v23-12000.csv')
 def test_fresh_catalog_rejects_empty_unknown_or_missing_selection(self):
  root=self.roots['mdu'];p=root/'catalog.json'
  for models in ({}, {'unreviewed':{}}, [], None):
   p.write_text(json.dumps({'version':1,'models':models}))
   with self.assertRaisesRegex(ValueError,'invalid policy catalog'):handle(root,{'op':'read_policy_model'})
  p.write_text(json.dumps({'version':1,'models':{}}))
  self.install_12000();cat=json.loads(p.read_text());cat['models']={'model12000':cat['models']['model12000']};p.write_text(json.dumps(cat))
  with self.assertRaisesRegex(ValueError,'not installed'):handle(root,{'op':'read_policy_model'})

class InventorySchedulingTest(unittest.TestCase):
 def test_hash_inventory_only_while_stopped_but_keep_ball_checks(self):
  import ast,threading
  from types import SimpleNamespace
  from unittest.mock import Mock
  source=Path(__file__).resolve().parents[1]/'a3/hope_lifecycle_supervisor.py'
  owner=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='LifecycleModule')
  method=next(n for n in owner.body if isinstance(n,ast.FunctionDef) and n.name=='_refresh_ball_models')
  scope={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
  for state in ('STOPPED','STARTING','RUNNING','KILLING','FAILED'):
   with self.subTest(state=state):
    field=Mock();pending=[]
    node=SimpleNamespace(_lock=threading.Lock(),_stopping=False,_closed=False,_busy=False,
      _field_busy=False,_field_refresh_running=False,_config=LifecycleConfig(revision=1),
      _state=state,_field=field,_field_pool=SimpleNamespace(submit=pending.append))
    scope['_refresh_ball_models'](node);self.assertEqual(len(pending),1);pending[0]()
    self.assertEqual(field.refresh_policy_models.call_count,int(state=='STOPPED'))
    field.refresh_ball_models.assert_called_once_with(node._config)
    self.assertFalse(node._field_refresh_running)

if __name__=='__main__':unittest.main()
