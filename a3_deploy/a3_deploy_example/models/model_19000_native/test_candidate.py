"""File/contract checks only; no simulation or native compatibility substitution."""
import copy,importlib.util,json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import yaml
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('native_candidate',HERE/'prepare_candidate.py');candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)

def test_original_graph_and_every_nonmetadata_field_unchanged():
 original=(HERE.with_name('model_19000')/'policy/exported/actor.onnx').read_bytes();new=(HERE/'policy/exported/policy.onnx').read_bytes()
 assert candidate.digest(original)==candidate.ONNX_SHA
 assert b''.join(raw for n,w,p,raw in candidate.fields(new) if n!=14)==original
 assert [p for n,w,p,raw in candidate.fields(original) if n==7]==[p for n,w,p,raw in candidate.fields(new) if n==7]

def test_configuration_values_preserved_and_hash_bound():
 old=yaml.safe_load((HERE.with_name('model_19000')/'policy/params/deploy.yaml').read_text());new=yaml.safe_load((HERE/'policy/params/deploy.yaml').read_text())
 old.pop('provenance');provenance=new.pop('provenance');assert old==new
 assert provenance['policy_sha256']==candidate.sha(HERE/'policy/exported/policy.onnx')
 m=json.loads((HERE/'candidate-manifest.json').read_text());assert m['native_loading']=='NOT_PROVEN' and m['formal_gate3']=='NOT_RUN'
 assert m['parity']['steps']==320 and m['parity']['action_max_abs_error']==m['parity']['hidden_max_abs_error']==0
 assert m['parity']['native_runtime_carry_reset_verification']=='NOT_IMPLEMENTED_OR_VERIFIED'
