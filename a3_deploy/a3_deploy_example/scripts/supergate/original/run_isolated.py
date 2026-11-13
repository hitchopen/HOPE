"""Run each native Gate3 in private PID/network/IPC/shm/tmp namespaces."""
from pathlib import Path
import subprocess,sys,json,os,concurrent.futures,time
W=Path(__file__).resolve().parent;V=Path('/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin');R=Path('/home/user/Desktop/workspace/nohope/agi/a3_deploy_example/scripts')
S={'14200':W/'download/model_14200_policy','13800':Path('/home/user/Documents/Codex/2026-11-03/new-chat/work/download/extracted/model_13800_policy')}
def run(name,cpus,scenario,root,aim=None):
 out=root/name;out.parent.mkdir(parents=True,exist_ok=True)
 if out.exists():raise ValueError(out)
 tmp=root/(name+'-sandbox-tmp');tmp.mkdir();ros=root/(name+'-ros');ros.mkdir()
 cmd=['podman','exec','--user','1000:1000','hope','/run/host/usr/bin/bwrap','--unshare-user','--uid','1000','--gid','1000','--unshare-pid','--unshare-ipc','--unshare-net','--ro-bind','/','/','--bind','/home/user','/home/user','--proc','/proc','--dev-bind','/dev','/dev','--tmpfs','/dev/shm','--bind',str(tmp),'/tmp','--ro-bind','/tmp/.X11-unix','/tmp/.X11-unix','--setenv','DISPLAY',os.environ.get('DISPLAY',':0'),'--setenv','XAUTHORITY',os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'),'--die-with-parent','--setenv','PATH',str(V)+':/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','--setenv','PYTHONPATH',str(R),'--setenv','ROS_HOME',str(ros),'--setenv','ROS_LOG_DIR',str(ros/'log'),'--setenv','ROS_LOCALHOST_ONLY','1','--setenv','OMP_NUM_THREADS','1','--setenv','OPENBLAS_NUM_THREADS','1','taskset','-c',cpus,'python3',str(W/'isolated_entry.py'),str(out),str(W/'run_gate3_suite.py'),'--policy-dir',str(S[name]),'--runtime',str(W/'runtime-0807.yaml'),'--scenario',str(scenario),'--headless','--output',str(out)]
 if aim:cmd+=['--land-sequence',str(aim)]
 (root/(name+'-command.json')).write_text(json.dumps(cmd,indent=2))
 with (root/(name+'.stdout')).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 print(name,'exit',r.returncode,flush=True)
 return r.returncode
if __name__=='__main__':
 scenario=Path(sys.argv[1]).resolve();root=Path(sys.argv[2]).resolve();aim=Path(sys.argv[3]).resolve() if len(sys.argv)>3 else None
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
  futures=[ex.submit(run,n,c,scenario,root,aim) for n,c in [('13800','0-3,8-15'),('14200','4-7,16-23')]]
  for f in futures:f.result()
