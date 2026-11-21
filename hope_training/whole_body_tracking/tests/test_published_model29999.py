import hashlib,json,os,subprocess,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[3]
BUNDLE=ROOT/'a3_deploy/a3_deploy_example/models/model_29999/policy/exported'

def test_published_actor_is_final_checkpoint_not_prior_bundle():
 m=json.loads((BUNDLE/'manifest.json').read_text())
 assert m['checkpoint_iteration']==29999 and m['wandb_run_id']=='gru-formal-2b8c09938a'
 assert m['checkpoint_sha256']=='6e671c56d1de5ead2a7553f489bf346bb372957d87f0bcc30854e2931e472c88'
 assert hashlib.sha256((BUNDLE/'actor.onnx').read_bytes()).hexdigest()==m['onnx_sha256']=='c361831725a4fd42661a7b6f46c738000120e248bae06e28f9e05195da3e96d3'
 assert m['hidden_shape']==[1,1,110] and len(m['joint_names'])==31
 assert '/home/' not in (BUNDLE/'manifest.json').read_text()

def test_standalone_bundle_loads_outside_repository(tmp_path):
 result=subprocess.run([sys.executable,str(BUNDLE/'verify.py')],cwd=tmp_path,check=True,capture_output=True,text=True)
 assert json.loads(result.stdout)['zero_reset_repeat']=='PASS'

def test_early_fall_results_are_not_described_as_100_completions():
 rows=json.loads((ROOT/'docs/evaluations/gru-formal-2b8c09938a/comparison.json').read_text());by={r['iteration']:r for r in rows}
 for n in [1000,27000]:assert by[n]['launched']==by[n]['resolved_flights']==2 and by[n]['unlaunched']==98 and by[n]['fall_time_s']<25
 for n in [10000,15000,19000]:assert by[n]['launched']==by[n]['resolved_flights']==100 and by[n]['fall_time_s']>152
 assert by[29999]['raw_contacts']==100 and by[29999]['legal_landings']==by[29999]['qualified_contacts']==0
 for r in rows:assert hashlib.sha256((ROOT/r['video_path']).read_bytes()).hexdigest()==r['video_sha256']

def test_motion_path_portability_changes_no_numerical_fields():
 sys.path.insert(0,str(ROOT/'hope_training/whole_body_tracking/scripts'))
 from portable_model_config import portable_motion_config
 old={'motion_file':['/machine/checkout/hope_training/motions/preprocessed/hope_forehand.npz'],'step_dt':.02}
 new=portable_motion_config(old)
 assert new['motion_file']==['hope_training/motions/preprocessed/hope_forehand.npz'] and new['step_dt']==old['step_dt']
 assert old['motion_file'][0].startswith('/machine/')
