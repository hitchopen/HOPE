import json,os,signal,sys
from pathlib import Path
p=Path(sys.argv[1])
if p.exists():
 x=json.loads(p.read_text());pid=x['bridge_pid'];proc=Path('/proc')/str(pid)/'cmdline'
 if proc.exists():
  args=proc.read_bytes().replace(b'\0',b' ').decode()
  if 'planner_ros_eval_bridge.py' not in args or x['output'] not in args:raise SystemExit('Refuse reused/unrelated PID')
  os.kill(pid,signal.SIGTERM)
