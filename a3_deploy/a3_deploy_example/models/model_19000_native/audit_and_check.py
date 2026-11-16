"""Capture real native capability/inspection failures; never start simulation here."""
import argparse,hashlib,importlib.util,json,os,re,shutil,subprocess,sys,tempfile
from pathlib import Path
sys.dont_write_bytecode=True
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
DEFAULT_DIST=ROOT.parent/'nohope/agibot/code_deployment/a3_deploy_example/dist/a3_deploy_x86_64'
HEADER=ROOT/'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_onnx_policy.hpp'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def record(command,folder,name,env=None):
 result=subprocess.run([str(x) for x in command],env=env,capture_output=True,text=True,timeout=90)
 (folder/(name+'.stdout')).write_text(result.stdout);(folder/(name+'.stderr')).write_text(result.stderr)
 item={'command':[str(x) for x in command],'exit_code':result.returncode,'stdout_file':name+'.stdout','stderr_file':name+'.stderr'};save(folder/(name+'.json'),item);return result

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--runner-dist',type=Path,default=DEFAULT_DIST);a=p.parse_args();a.evidence.mkdir(parents=True,exist_ok=True);attempt=Path(tempfile.mkdtemp(prefix='native-preflight-',dir=a.evidence))
 binary=a.runner_dist/'a3_deploy_onnx_ref_pingpong';candidate=json.loads((HERE/'candidate-manifest.json').read_text());assert sha(HERE/'policy/exported/policy.onnx')==candidate['candidate_onnx_sha256'];assert sha(HERE/'policy/params/deploy.yaml')==candidate['candidate_deploy_sha256']
 sourcefiles=[ROOT/'run_gate3.sh',ROOT/'docs/operations/gate3_two_file_runner.md',ROOT/'docs/operations/deploy_and_gate3.md',ROOT/'a3_deploy/a3_deploy_example/scripts/run_gate3.py',HEADER,HEADER.with_name('pp_policy.hpp'),HEADER.with_name('pp_deploy_config.hpp'),ROOT/'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/src/a3_deploy/a3_pingpong_main.cpp']
 sourcefiles+=list((ROOT/'a3_deploy/a3_deploy_example/scripts').glob('pp_gate3_*.py'))+list((ROOT/'a3_deploy/a3_deploy_example/scripts').glob('pp_rally*.py'))
 text=HEADER.read_text();lookups=sorted(set(re.findall(r'LookupMeta\(md, alloc,\s*"([^"]+)"',text)))
 # Preserve exact two files/reference scenario in each attempt before any loading.
 snap=attempt/'policy';shutil.copytree(HERE/'policy',snap)
 scene=ROOT/'a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home-100.yaml';shutil.copyfile(scene,attempt/'fixed-home-100.yaml')
 profile=ROOT/'a3_deploy/a3_deploy_example/config/gate3/runtime.yaml'
 checkout_revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip()
 info={'evaluation_checkout_revision':checkout_revision,'source_sha256':{str(x.relative_to(ROOT)):sha(x) for x in sourcefiles},'scenario_sha256':sha(scene),'runtime_default_sha256':sha(profile),'all_literal_mandatory_metadata_lookups_across_all_branches':lookups,'metadata_lookup_note':'Conditional supported-policy branches differ; this is an inventory, not permission to fabricate all keys. Dynamic key checks also exist.','original_and_candidate_graph_io':candidate['signature'],'native_source_io':'PpOnnxPolicy::Run supplies obs,time_step; PpPolicy requests genuine joint_pos/joint_vel/body_pos_w/body_quat_w; no h_in/h_out carry/reset implementation','active_branch_for_saved_metadata':{'recipe':candidate['metadata']['hitter_pure_training_recipe'],'version':candidate['metadata']['hitter_pure_training_recipe_version'],'runtime_contract':candidate['metadata']['hitter_pure_runtime_contract']},'runner_binary':str(binary),'binary_source_binding':'UNVERIFIED_NO_BUILD_RECEIPT; current checkout revisions do not establish build provenance','process_scope':'sandbox may expose only its PID namespace; no claim of global workstation idle','certification_pass':None,'formal_run_started':False}
 info['machine_process_listing']=record(['ps','-eo','pid,comm'],attempt,'processes').stdout.splitlines()
 if binary.exists():
  info['found_runner_libraries_sha256']={p.name:sha(p) for p in a.runner_dist.iterdir() if p.is_file() and (p.name.startswith('libonnxruntime.so') or p.name.startswith('libaimrt_'))}
  blob=binary.read_bytes();info['runner_binary_sha256']=sha(binary);info['binary_inspection_flag_marker_present']=b'--inspect-policy' in blob;info['binary_inspection_receipt_marker_present']=b'POLICY_INSPECTION_FAILED' in blob
  dist=a.runner_dist;env=dict(os.environ,LD_LIBRARY_PATH=str(dist)+':/opt/ros/jazzy/lib')
  # Dynamic loader --list never enters main or starts a hardware backend.
  loader=record(['/lib64/ld-linux-x86-64.so.2','--list',binary],attempt,'elf-loader-check',env=env);info['elf_loader_exit_code']=loader.returncode
  if not info['binary_inspection_flag_marker_present'] or not info['binary_inspection_receipt_marker_present']:
   info['actual_native_inspection']={'status':'NOT_EXECUTED_UNSUPPORTED_INSTALLED_INSPECTION_BUILD','exit_code':None,'reason':'Do not pass unrecognized inspect flag to a binary that might initialize transport/hardware.'}
  elif loader.returncode:
   info['actual_native_inspection']={'status':'NOT_EXECUTED_MISSING_SHARED_LIBRARIES','exit_code':None,'loader_exit_code':loader.returncode}
  else:
   loaded=record([binary,'--inspect-policy','--policy-dir',snap],attempt,'actual-native-inspection',env=env);info['actual_native_inspection']={'status':'LOADED' if loaded.returncode==0 else 'POLICY_INSPECTION_FAILED','exit_code':loaded.returncode}
   if loaded.returncode==0:
    receipt=yaml.safe_load(loaded.stdout);save(attempt/'policy_inspection.json',receipt)
 else:info['actual_native_inspection']={'status':'NOT_EXECUTED_RUNNER_MISSING','exit_code':None}
 simulator=ROOT.parent/'nohope/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/cmake-build-model21800-gate3/install/bin/aimrt_main'
 if simulator.exists():info['found_simulator_binary']={'path':str(simulator),'sha256':sha(simulator),'source_binding':'NOT_PROVEN_CURRENT_INSTRUMENTED_GATE3_BUILD'}
 oldroot=ROOT.parent/'nohope';cache=oldroot/'agibot/code_deployment/a3_deploy_example/build/a3_pkg_x86_64/CMakeCache.txt'
 if cache.exists():
  info['found_cmake_cache']={'path':str(cache),'sha256':sha(cache),'home_directory':[x for x in cache.read_text().splitlines() if x.startswith('CMAKE_HOME_DIRECTORY:')]}
  info['sibling_source_checkout_head']=subprocess.check_output(['git','-C',str(oldroot),'rev-parse','HEAD'],text=True).strip()
  sibling_header=oldroot/'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_onnx_policy.hpp'
  if sibling_header.exists():info['sibling_native_header_sha256']=sha(sibling_header)
 # Run actual documented check implementation with its default runtime, never a custom evaluator.
 command=[sys.executable,ROOT/'a3_deploy/a3_deploy_example/scripts/run_gate3.py','--policy-dir',HERE/'policy','--scenario',scene,'--check','--headless','--output',attempt/'documented-check']
 checked=record(command,attempt,'documented-gate3-check');info['documented_gate3_check_exit_code']=checked.returncode
 container=record(['podman','inspect','--type','container','hope','--format','{{.Id}}'],attempt,'hope-container-access');info['hope_container_access_exit_code']=container.returncode
 info['native_recurrent_integration_status']='NOT_IMPLEMENTED_OR_VERIFIED';info['formal_gate3_result']='NOT_RUN: native loading and recurrent/reference-contract gates not satisfied'
 save(attempt/'audit.json',info);save(a.evidence/'latest-native-audit.json',{'attempt':str(attempt),'audit_sha256':sha(attempt/'audit.json'),'actual_native_inspection':info['actual_native_inspection'],'formal_gate3_result':info['formal_gate3_result']});print(json.dumps({'attempt':str(attempt),'runner_binary_sha256':info.get('runner_binary_sha256'),'actual_native_inspection':info['actual_native_inspection'],'gate3_check_exit':checked.returncode,'container_access_exit':container.returncode,'formal_gate3_result':info['formal_gate3_result']},indent=2))
if __name__=='__main__':main()
