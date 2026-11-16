"""Portable entry point for the published recurrent model29999 diagnostic.

Uses simulated inputs and the isolated ROS planner only. No hardware bridge,
training, installation, automatic build, or old-bundle fallback is provided.
"""
import argparse,json,os,shlex,signal,socket,subprocess,sys,tempfile,time
from pathlib import Path
from mujoco_eval_recurrent import ROOT

DEFAULT_BUNDLE=ROOT/'a3_deploy/a3_deploy_example/models/model_29999/policy/exported'

def ros_command(args,bridge_args):
 setup=Path(args.ros_setup).resolve()
 if not setup.is_file():raise FileNotFoundError('ROS install setup missing: '+str(setup))
 command='set -eo pipefail\nsource /opt/ros/jazzy/setup.bash\nsource '+shlex.quote(str(setup))+'\n'
 command+='export ROS_DOMAIN_ID='+str(args.ros_domain)+' ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\n'
 command+='exec '+shlex.join(['/usr/bin/python3',str(ROOT/'hope_training/whole_body_tracking/scripts/planner_ros_eval_bridge.py'),*bridge_args])
 return ['bash','-lc',command] if args.ros_host else ['distrobox','enter',args.ros_box,'--','bash','-lc',command]

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--bundle',type=Path,default=DEFAULT_BUNDLE);p.add_argument('--view',action='store_true');p.add_argument('--replay',action='store_true');p.add_argument('--verify-only',action='store_true')
 p.add_argument('--output-root',type=Path,default=Path.home()/'hope-evaluations');p.add_argument('--planner',type=Path,default=Path(os.environ.get('HOPE_PLANNER_BINARY',ROOT/'hope_ws/install/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node')))
 p.add_argument('--ros-setup',type=Path,default=Path(os.environ.get('HOPE_ROS_SETUP',ROOT/'hope_ws/install/setup.bash')));p.add_argument('--ros-box',default=os.environ.get('HOPE_ROS_BOX','hope'));p.add_argument('--ros-host',action='store_true');p.add_argument('--ros-domain',type=int,default=187)
 p.add_argument('--record-video',action='store_true',help='Render an actual-state MP4 after timing-critical simulation; original published videos remain unchanged')
 a=p.parse_args();a.bundle=a.bundle.resolve();m=json.loads((a.bundle/'manifest.json').read_text())
 if m['checkpoint_iteration']!=29999 or m['checkpoint_sha256']!='6e671c56d1de5ead2a7553f489bf346bb372957d87f0bcc30854e2931e472c88':raise ValueError('Expected published model29999, no fallback')
 if a.replay:
  movie=ROOT/'docs/assets/gru-formal-2b8c09938a/model_29999.mp4'
  if not movie.is_file():raise FileNotFoundError(movie)
  import shutil
  player=shutil.which('ffplay')
  command=[player,'-autoexit','-ss','20',str(movie)] if player else ['xdg-open',str(movie)]
  raise SystemExit(subprocess.call(command))
 a.output_root.mkdir(parents=True,exist_ok=True);attempt=Path(tempfile.mkdtemp(prefix='model29999-',dir=a.output_root))
 scripts=ROOT/'hope_training/whole_body_tracking/scripts'
 print(f'Checkpoint29999 | run {m["wandb_run_id"]} | bundle {a.bundle}\nTable PPQ004 | profile {m["task"]["ball_physics"]["sha256"]} |100 scheduled balls | first serve20s\nOutput {attempt}',flush=True)
 subprocess.run([sys.executable,str(scripts/'mujoco_eval_recurrent.py'),'--bundle',str(a.bundle),'--physical-ball','--verify-only','--output-dir',str(attempt/'verification')],check=True)
 subprocess.run([sys.executable,str(scripts/'verify_bundle_reset.py'),'--bundle',str(a.bundle),'--output',str(attempt/'reset.json')],check=True)
 if a.verify_only:return
 if not a.planner.is_file():raise FileNotFoundError('Provide an existing ROS planner binary with --planner (see model README); no automatic build')
 import yaml
 config=json.loads((a.bundle.parent/'params/planner-parameters.json').read_text());ns='hope_gru_'+attempt.name.replace('-','_');config.update(robot_pose_topic='/'+ns+'/base',debug_session_id=ns,debug_csv_path=str(attempt/'ros/planner-audit.csv'))
 config_file=attempt/'planner.yaml';config_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}},sort_keys=False))
 cache=Path(os.environ.get('XDG_CACHE_HOME',Path.home()/'.cache'));cache.mkdir(parents=True,exist_ok=True)
 transport=Path(tempfile.mkdtemp(prefix='hope-gru-',dir=cache));sock=transport/'planner.sock'
 bridge_args=['--socket',str(sock),'--config',str(config_file),'--planner',str(a.planner.resolve()),'--namespace',ns,'--output',str(attempt/'ros')]
 log=(attempt/'bridge.log').open('w');bridge=subprocess.Popen(ros_command(a,bridge_args),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rc=1;bridge_rc=None
 try:
  until=time.monotonic()+35
  while not sock.exists():
   if bridge.poll() is not None or time.monotonic()>until:raise RuntimeError('Isolated ROS planner failed to start; inspect '+str(attempt/'bridge.log'))
   time.sleep(.1)
  args=[sys.executable,str(scripts/'reference_gru_diagnostic.py'),'--bundle',str(a.bundle),'--scene',str(a.bundle.parent/'params/fixed-home-100.yaml'),'--socket',str(sock),'--output',str(attempt/'scored'),'--table-impact-model','ppq004','--video']
  if a.view:args.append('--view')
  rc=subprocess.run(['timeout','--signal=INT','--kill-after=10','915',*args],check=False).returncode
  bridge_rc=bridge.wait(timeout=12)
 finally:
  if bridge.poll() is None:
   stop=['/usr/bin/python3',str(scripts/'stop_owned_planner_bridge.py'),str(attempt/'ros/ownership.json')]
   if not a.ros_host:stop=['distrobox','enter',a.ros_box,'--',*stop]
   try:subprocess.run(stop,stdout=log,stderr=subprocess.STDOUT,timeout=10)
   except Exception:pass
   try:os.killpg(bridge.pid,signal.SIGTERM)
   except ProcessLookupError:pass
   try:bridge.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(bridge.pid,signal.SIGKILL);bridge.wait()
  log.close()
  try:transport.rmdir()
  except OSError:pass
 if a.record_video and (attempt/'scored/render-states.jsonl').exists():
  env=dict(os.environ,MUJOCO_GL='egl',LIBGL_ALWAYS_SOFTWARE='1')
  subprocess.run([sys.executable,str(scripts/'render_reference_diagnostic.py'),'--bundle',str(a.bundle),'--output',str(attempt/'scored')],check=True,env=env,timeout=900)
 print('Results:',attempt,flush=True)
 if bridge_rc!=0:raise SystemExit('Infrastructure failure: planner bridge exit '+str(bridge_rc))
 raise SystemExit(rc)
if __name__=='__main__':main()
