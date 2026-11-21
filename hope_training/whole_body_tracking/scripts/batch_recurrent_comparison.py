"""Sequential, resumable eight-checkpoint diagnostic; never starts training or hardware."""
import argparse,fcntl,hashlib,html,json,os,shutil,signal,socket,subprocess,tempfile,time
from pathlib import Path


def load(p):return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(p,data):
 p=Path(p);t=p.with_suffix(p.suffix+'.tmp');t.write_text(json.dumps(data,indent=2)+'\n');t.replace(p)
def stage(name,cmd,log,timeout=120,env=None,allowed=(0,)):
 start=time.monotonic();print(f'  START {name}',flush=True)
 with Path(log).open('a') as f:
  f.write('COMMAND '+json.dumps([str(x) for x in cmd])+'\n');f.flush()
  rc=subprocess.run([str(x) for x in cmd],stdout=f,stderr=subprocess.STDOUT,timeout=timeout,env=env).returncode
 elapsed=time.monotonic()-start;print(f'  END {name}: rc={rc} {elapsed:.2f}s | {log}',flush=True)
 if rc not in allowed:raise RuntimeError(f'{name} failed rc={rc}; inspect {log}')
 return rc,elapsed

def classify(summary):
 if summary.get('error') or any(summary.get('mujoco_warning_counts',[0])):return 'INFRASTRUCTURE_OR_NUMERICAL_ERROR'
 if summary.get('rally_failure')=='fall':return 'EARLY_TERMINATION_FALL'
 if summary.get('rally_failure'):return 'INFRASTRUCTURE_OR_SAFETY_ERROR'
 return 'COMPLETE_100' if summary['groups']['all']['launched']==100 else 'INCOMPLETE_INFRASTRUCTURE'

def valid_done(model,signature):
 p=model/'completed.json'
 if not p.exists():return False
 receipt=load(p)
 if receipt['protocol_signature']!=signature:return False
 for name,digest in receipt['sha256'].items():
  f=Path(name)
  if not f.is_file() or sha(f)!=digest:return False
 return True

def index(batch,c):
 data=[]
 for spec in c['models']:
  m=Path(spec['model_dir']);status=load(m/'status.json') if (m/'status.json').exists() else {'status':'PENDING'}
  row={'requested':spec['requested'],'iteration':spec['iteration'],'substitution':spec['substituted'],**status};data.append(row)
 save(batch/'comparison-index.json',{'label':'REFERENCE-SEQUENCE DIAGNOSTIC - NOT FORMAL GATE3','run_id':c['run_id'],'rows':data})
 body=[]
 for r in data:
  movie=r.get('video');link=f'<a href="{html.escape(os.path.relpath(movie,batch))}">MP4</a>' if movie and Path(movie).exists() else 'pending'
  metrics=r.get('metrics',{});cells=[str(r['iteration']),r['status'],link]+[str(metrics.get(k,'—')) for k in ['launched','raw_contacts','qualified_contacts','legal_landings','falls','unlaunched','longest_success_streak']]+[str(r.get('first_serve_video_s','—'))]
  body.append('<tr>'+''.join('<td>'+str(x)+'</td>' if i==2 else '<td>'+html.escape(str(x))+'</td>' for i,x in enumerate(cells))+'</tr>')
 status=load(batch/'batch-status.json') if (batch/'batch-status.json').exists() else {}
 header=['Checkpoint','Status','Video','Launched','Raw contact balls','Qualified','Legal returns','Falls','Unlaunched','Longest success streak','First serve (video s)']
 page='<!doctype html><meta charset="utf-8"><title>Eight checkpoint comparison</title><style>body{font:15px system-ui;margin:2em}table{border-collapse:collapse}td,th{padding:9px;border:1px solid #ddd}th{background:#eee}video{max-width:800px}</style><h1>Reference-sequence diagnostic — NOT formal Gate3</h1><p>Run gru-formal-2b8c09938a. Fixed-home-100, seed0, corrected PPQ004, persistent robot/GRU state. Falls end rallies without rescue. Infrastructure failures are separate.</p><p>All MP4s:800×500,20fps,1× playback, identical camera. Initial idle is archived; first serve normally at20s. Purple dashed is a planner reference, orange/cyan actual flight.</p><p>Batch: '+html.escape(str(status.get('status','PREPARED')))+' '+html.escape(str(status.get('error','')))+'<p><a href="comparison-index.json">JSON results</a> | <a href="batch.json">Frozen plan/hashes</a></p><table><tr>'+''.join('<th>'+h+'</th>' for h in header)+'</tr>'+''.join(body)+'</table>'
 (batch/'index.html').write_text(page)

def integrity(batch,c):
 for base,files in load(batch/'inputs.json').items():
  for name,digest in files.items():
   p=Path(base)/name
   if sha(p)!=digest:raise RuntimeError('Changed frozen comparison input: '+str(p))
 for s in c['models']:
  if sha(s['snapshot'])!=s['sha256']:raise RuntimeError('Checkpoint changed: '+s['snapshot'])
  for n,digest in s['params_sha256'].items():
   if sha(Path(s['model_dir'])/'params'/n)!=digest:raise RuntimeError('Saved configuration changed')


def ensure_bundle(batch,c,s,logs,signature):
 model=Path(s['model_dir']);ready=model/'bundle-ready.json'
 if ready.exists():
  receipt=load(ready)
  if receipt['checkpoint_sha256']==s['sha256'] and receipt['protocol_signature']==signature and all(Path(p).is_file() and sha(p)==h for p,h in receipt['sha256'].items()):
   print('  REUSE verified bundle',model/'bundle',flush=True);return model/'bundle'
 candidate=Path(tempfile.mkdtemp(prefix='bundle-build-',dir=model))
 # External cached bundles are copied to a fresh directory, then independently
 # verified against this exact checkpoint; a manifest alone is insufficient.
 if s.get('cached_bundle'):
  shutil.copytree(s['cached_bundle'],candidate,dirs_exist_ok=True)
 stage('CPU export / exact initializer verification',['distrobox','enter','grasping','--','bash',batch/'export-one.sh',model,candidate],logs/'export.log',c['export_timeout_s'])
 if (model/'bundle').exists():
  # Preserve an invalid/incomplete cache rather than overwrite it.
  (model/'bundle').rename(model/('bundle-unverified-'+str(time.time_ns())))
 candidate.rename(model/'bundle');bundle=model/'bundle'
 files=[bundle/n for n in ['actor.onnx','manifest.json','parity_vectors.npz','actor-load-verification.json']]
 save(ready,{'checkpoint_sha256':s['sha256'],'protocol_signature':signature,'sha256':{str(p):sha(p) for p in files}})
 return bundle


def evaluation_files(attempt,bundle,model):
 out=attempt/'scored'
 return [out/n for n in ['summary.json','per-ball.json','per-ball.csv','physical-evidence.json','configuration.json','ledger.json','events.jsonl','ball-trajectories.jsonl','policy-trace.jsonl','planner-commands.jsonl','render-states.jsonl']]+[attempt/'ros/planner-audit.csv',attempt/'ros/sensor-and-planner.jsonl',attempt/'ros/planner.log',attempt/'parity/verification.json',attempt/'reset-verification.json']+[bundle/n for n in ['actor.onnx','manifest.json','parity_vectors.npz','actor-load-verification.json']]+[model/'snapshot.json']


def resumable_evaluation(attempt,signature):
 ready=attempt/'evaluation-ready.json'
 if not ready.exists():return False
 receipt=load(ready)
 return receipt['protocol_signature']==signature and all(Path(p).is_file() and sha(p)==h for p,h in receipt['sha256'].items())


def finish_recording(batch,c,s,attempt,bundle,timings,signature,state):
 model=Path(s['model_dir']);out=attempt/'scored';logs=attempt/'logs';summary=load(out/'summary.json');metrics=summary['groups']['all']
 status={'status':'RENDER_PENDING_'+state,'attempt':str(attempt),'metrics':{**{k:metrics[k] for k in ['launched','raw_contacts','qualified_contacts','legal_landings','unlaunched','unresolved']},'falls':summary['falls'],'longest_success_streak':summary['longest_success_streak']},'timings_s':timings}
 save(model/'status.json',status);index(batch,c)
 # Preserve failed encodings; resume renders saved states without rerunning physics.
 names=['comparison.mp4','comparison.ass','comparison-video.json','planner-reference-paths.json']
 if any((out/n).exists() for n in names):
  old=out/('render-incomplete-'+str(time.time_ns()));old.mkdir()
  for n in names:
   if (out/n).exists():(out/n).rename(old/n)
 save(out/'render-context.json',{'infrastructure_error':state if state not in ('COMPLETE_100','EARLY_TERMINATION_FALL') else None})
 env=dict(os.environ,MUJOCO_GL='egl',LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1')
 _,timings['offline_render_s']=stage('offline annotated actual-state MP4',[c['python'],Path(c['root'])/'hope_training/whole_body_tracking/scripts/render_checkpoint_comparison.py','--output',out,'--bundle',bundle,'--trace-executable',c['trace_executable']],logs/'render.log',c['render_timeout_s'],env)
 video=load(out/'comparison-video.json');status.update(status=state,video=str(out/'comparison.mp4'),first_serve_video_s=video['first_serve_video_s'],timings_s=timings)
 save(model/'status.json',status);save(attempt/'timings.json',timings);index(batch,c)
 if state not in ('COMPLETE_100','EARLY_TERMINATION_FALL'):raise RuntimeError(state+'; preserved partial evidence/video; not counted as policy failure')
 files=evaluation_files(attempt,bundle,model)+[out/'comparison.mp4',out/'comparison-video.json',model/'status.json']
 save(model/'completed.json',{'protocol_signature':signature,'sha256':{str(p):sha(p) for p in files},'state':state,'checkpoint_sha256':s['sha256']})
 print(f'  SAVED {state}: {out}/comparison.mp4 | first serve {video["first_serve_video_s"]}s',flush=True)


def model_run(batch,c,s,ordinal,signature):
 model=Path(s['model_dir']);print(f'[{ordinal}/8] model_{s["iteration"]} requested={s["requested"]} sha={s["sha256"]}',flush=True)
 if valid_done(model,signature):print('  SKIP: completed result and video hashes verified',flush=True);return
 if (model/'status.json').exists():
  previous=load(model/'status.json');old=Path(previous['attempt']) if previous.get('attempt') else None
  if old is not None and resumable_evaluation(old,signature):
   print('  RESUME OFFLINE RENDER: verified simulation already saved; no ROS/evaluation rerun',flush=True)
   ready=load(old/'evaluation-ready.json')
   try:finish_recording(batch,c,s,old,model/'bundle',load(old/'timings.json'),signature,ready['state'])
   except BaseException as exc:
    status=load(model/'status.json');status.update(status='INFRASTRUCTURE_RENDER_FAILURE',error=repr(exc));save(model/'status.json',status);index(batch,c);raise
   return
 attempt=Path(tempfile.mkdtemp(prefix='attempt-',dir=model));logs=attempt/'logs';logs.mkdir();timings={};status={'status':'RUNNING','attempt':str(attempt)};save(model/'status.json',status);index(batch,c)
 try:
  if not c.get('_containers_checked'):
   for box in ['grasping','hope']:stage('existing container '+box,['podman','container','exists',box],logs/'containers.log',20)
   c['_containers_checked']=True
  if not os.access(c['planner'],os.X_OK):raise FileNotFoundError('Missing existing planner build; no automatic rebuild')
  start=time.monotonic();bundle=ensure_bundle(batch,c,s,logs,signature);timings['export_or_cached_verification_s']=time.monotonic()-start
  _,timings['sequence_reset_and_io_checks_s']=stage('64-step PyTorch/ONNX parity, reset and IO',[c['python'],Path(c['root'])/'hope_training/whole_body_tracking/scripts/mujoco_eval_recurrent.py','--bundle',bundle,'--physical-ball','--verify-only','--output-dir',attempt/'parity'],logs/'parity.log',120)
  _,timings['reset_repeat_s']=stage('repeat after zero hidden reset',[c['python'],Path(c['root'])/'hope_training/whole_body_tracking/scripts/verify_bundle_reset.py','--bundle',bundle,'--output',attempt/'reset-verification.json'],logs/'reset.log',60)
  manifest=load(bundle/'manifest.json');verification=load(bundle/'actor-load-verification.json')
  assert manifest['checkpoint_iteration']==s['iteration'] and manifest['checkpoint_sha256']==s['sha256'] and verification['result']=='PASS'
  # Existing planner binary/install are immutable inputs. No colcon build here.
  import yaml
  params=load(c['parameters']);ns='hope_cmp_'+str(s['iteration'])+'_'+attempt.name.replace('-','_')
  params.update(robot_pose_topic='/'+ns+'/base',debug_session_id=ns,debug_csv_path=str(attempt/'ros/planner-audit.csv'))
  config=attempt/'planner.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
  transport=Path(tempfile.mkdtemp(prefix='hope-cmp-',dir=Path.home()/'.cache'));sock=transport/'planner.sock';bridge=None;bridge_log=(logs/'bridge.log').open('a')
  try:
   print('  START actual planner +100-serve evaluation | PPQ004 | seed0 | initial idle20s',flush=True);start=time.monotonic()
   bridge=subprocess.Popen(['distrobox','enter','hope','--','bash',c['bridge_launcher'],'--socket',str(sock),'--config',str(config),'--planner',c['planner'],'--namespace',ns,'--output',str(attempt/'ros')],stdout=bridge_log,stderr=subprocess.STDOUT,start_new_session=True)
   deadline=time.monotonic()+35
   while not sock.exists():
    if bridge.poll() is not None:raise RuntimeError('ROS bridge failed at startup; '+str(logs/'bridge.log'))
    if time.monotonic()>deadline:raise TimeoutError('ROS startup timeout')
    time.sleep(.1)
   env=dict(os.environ,MUJOCO_GL='glfw',LIBGL_ALWAYS_SOFTWARE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1')
   rc,timings['evaluation_s']=stage('bounded actual simulation',[c['python'],Path(c['root'])/'hope_training/whole_body_tracking/scripts/reference_gru_diagnostic.py','--bundle',bundle,'--scene',c['scene'],'--socket',sock,'--output',attempt/'scored','--table-impact-model','ppq004','--video'],logs/'evaluation.log',c['model_wall_timeout_s'],env,allowed=(0,1))
   try:bridge_rc=bridge.wait(timeout=12)
   except subprocess.TimeoutExpired:raise RuntimeError('Planner failed to stop after evaluation')
   timings['planner_and_evaluation_total_s']=time.monotonic()-start
  finally:
   if bridge is not None and bridge.poll() is None:
    try:subprocess.run(['distrobox','enter','hope','--','/usr/bin/python3',c['stop_bridge'],str(attempt/'ros/ownership.json')],stdout=bridge_log,stderr=subprocess.STDOUT,timeout=10)
    except Exception:pass
    try:os.killpg(bridge.pid,signal.SIGTERM)
    except ProcessLookupError:pass
    try:bridge.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(bridge.pid,signal.SIGKILL);bridge.wait()
   bridge_log.close()
   try:transport.rmdir()
   except OSError:pass
  out=attempt/'scored';summary=load(out/'summary.json');state=classify(summary)
  if bridge_rc!=0:state='INFRASTRUCTURE_BRIDGE_EXIT_'+str(bridge_rc)
  if rc!=0 and state=='COMPLETE_100':state='INFRASTRUCTURE_EVALUATOR_EXIT_'+str(rc)
  save(attempt/'timings.json',timings)
  if state in ('COMPLETE_100','EARLY_TERMINATION_FALL'):
   assert summary['resets_after_initial']==0 and not summary['remaining_live_ids'] and summary['drain_pass']
   assert summary['scheduled_ball_count']==100 and summary['table_impact_model']=='ppq004'
   files=evaluation_files(attempt,bundle,model)
   save(attempt/'evaluation-ready.json',{'protocol_signature':signature,'state':state,'sha256':{str(p):sha(p) for p in files}})
  finish_recording(batch,c,s,attempt,bundle,timings,signature,state)
 except BaseException as exc:
  previous=load(model/'status.json');previous.update(status='INFRASTRUCTURE_FAILURE',error=repr(exc),attempt=str(attempt));save(model/'status.json',previous);save(attempt/'timings.json',timings);index(batch,c);raise


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True);p.add_argument('--check-only',action='store_true');a=p.parse_args();batch=a.batch.resolve();c=load(batch/'batch.json')
 lock=(batch/'batch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 os.chdir(c['root']);print('EIGHT-CHECKPOINT REFERENCE-SEQUENCE DIAGNOSTIC (NOT FORMAL GATE3)',flush=True)
 print('Run',c['run_id'],'checkpoints',[s['iteration'] for s in c['models']],flush=True);print('PPQ004 |100scheduled/model |20s initial idle | no rescue | sequential | existing ROS build',flush=True)
 signature=sha(batch/'inputs.json');integrity(batch,c);index(batch,c)
 if a.check_only:print('Frozen checkpoints/configuration/source hashes: PASS',flush=True);return
 try:
  if all(valid_done(Path(s['model_dir']),signature) for s in c['models']):
   save(batch/'batch-status.json',{'status':'EIGHT_VERIFIED_VIDEOS_COMPLETE'});index(batch,c)
   print('All eight outputs already verified; no container/evaluation launch.',flush=True);return
  for i,s in enumerate(c['models'],1):
   integrity(batch,c);model_run(batch,c,s,i,signature)
  save(batch/'batch-status.json',{'status':'EIGHT_VERIFIED_VIDEOS_COMPLETE'});index(batch,c)
 except BaseException as exc:
  save(batch/'batch-status.json',{'status':'BLOCKED_OR_INFRASTRUCTURE_FAILURE','error':repr(exc)});index(batch,c);print('BATCH STOPPED; evidence preserved. Resume with the same desktop command.',flush=True);raise
if __name__=='__main__':main()
