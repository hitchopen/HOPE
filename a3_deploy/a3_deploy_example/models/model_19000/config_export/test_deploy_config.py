"""File/decoder checks only, no ONNX inference or simulation."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[5]
MODEL=Path(__file__).resolve().parent.parent

def test_dimensions_mapping_binding_and_gain_orders():
 d=yaml.safe_load((MODEL/'policy/params/deploy.yaml').read_text());r=json.loads((MODEL/'deploy-config-verification.json').read_text());rows=r['joint_rows']
 assert len(rows)==31 and len(d['joint_ids_map'])==31 and sorted(d['joint_ids_map'])==list(range(31))
 assert sum(v['dim']*v['history_length'] for v in d['observations']['policy'].values())==110
 for row in rows:
  i,sdk=row['policy_index'],row['sdk_index']
  assert d['joint_sdk_names'][sdk]==d['actions']['JointPositionAction']['joint_names'][i]==row['joint']
  assert d['stiffness'][sdk]==row['kp'] and d['damping'][sdk]==row['kd_wire']
  np.testing.assert_allclose(row['kd_training_total'],row['kd_wire']+row['kd_passive'],atol=1e-6,rtol=0)
 assert d['step_dt']==.02
 assert r['configuration_status']=='EXPORTED_AND_CHECKED' and r['receiving_hardware_runner_compatibility']=='NOT_YET_VERIFIED'
 assert hashlib.sha256((MODEL/'policy/exported/actor.onnx').read_bytes()).hexdigest()==d['provenance']['policy_sha256']=='f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda'
 assert hashlib.sha256((MODEL/'policy/params/deploy.yaml').read_bytes()).hexdigest()==r['deploy_yaml_sha256']

def test_existing_action_adapter_reads_export_and_matches_saved_decoder():
 sys.path.insert(0,str(ROOT/'a3_deploy/a3_deploy_example/reference'))
 from a3_deploy_onnx_ref_pingpong.action_adapter import ActionAdapter
 from a3_deploy_onnx_ref_pingpong.joint_order import JOINT_NAMES
 d=yaml.safe_load((MODEL/'policy/params/deploy.yaml').read_text());a=d['actions']['JointPositionAction'];adapter=ActionAdapter.from_yaml(MODEL/'policy/params/deploy.yaml');order=[a['joint_names'].index(n) for n in JOINT_NAMES]
 for raw in [np.zeros(31),np.linspace(-10,10,31)]:
  reference=np.clip(np.array(a['offset'])[order]+raw*np.array(a['scale'])[order],np.array(a['clip'])[order,0],np.array(a['clip'])[order,1])
  np.testing.assert_array_equal(adapter.decode(raw),reference)
