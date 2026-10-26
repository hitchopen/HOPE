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
  with self.assertRaisesRegex(ValueError,'receive-only'):self.select('model17400')

if __name__=='__main__':unittest.main()
