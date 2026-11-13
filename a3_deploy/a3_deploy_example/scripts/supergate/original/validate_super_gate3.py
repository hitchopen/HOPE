from pathlib import Path
import ast,contextlib,io,json,math,os,sys,hashlib,collections
import numpy as np,yaml
from flight_kernel import flight
W=Path(__file__).resolve().parent
# Exercise actual launcher/parser and scene resolver code without starting ROS.
def extract(path,name,env):
 tree=ast.parse(path.read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name);exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),env);return env[name]
import argparse
parse=extract(W/'pp_gate3_ball_launcher.py','parse_args',dict(argparse=argparse,os=os,json=json,math=math,TABLE_HEIGHT_M=.76))
resolve=extract(W/'run_gate3_suite.py','scene_environment',dict(json=json,math=math))
s=yaml.safe_load((W/'super-gate3.yaml').read_text());d=json.loads((W/'super-gate3-ledger.json').read_text());env=resolve(s);os.environ.update(env);sys.argv=['test'];args=parse();rows=d['selected'];assert len(rows)==s['balls']==len(args.spin_sequence)==246
assert args.spin_sequence==s['spin_rad_s_list'];assert len({tuple(r['launch']+r['initial_spin']) for r in rows})==len(rows)
assert len({r['requested_id'] for r in rows})==len(rows)
assert np.allclose(np.array(json.loads(env['PP_SERVES_LIST'])).reshape(-1,6),np.array([r['launch'] for r in rows]))
for bad in ['[]','[[0,0,NaN]]',json.dumps([[0,0,True]]*len(rows))]:
 sys.argv=['test','--spin-sequence',bad]
 with contextlib.redirect_stderr(io.StringIO()):
  try:parse();raise AssertionError('accepted invalid spin')
  except SystemExit as e:assert e.code==2
sys.argv=['test','--spin-sequence','','--spin-rad-s','1,2,3'];assert parse().spin_sequence==[[1,2,3]]*len(rows)
maxerr=0
for r in rows:
 state,w,t,b,bounce,net=flight(np.array(r['launch']),np.array(r['initial_spin']));maxerr=max(maxerr,float(np.max(abs(state-r['predicted_contact']))));assert b==1 and 0<bounce[0]<1.37 and -.025>net[0]>-1.5 and net[1]>.17
 assert int(np.digitize(np.linalg.norm(state[3:]),[2.5,4]))==r['speed_bin'];assert int(np.digitize(state[2]+.76,[1,1.2]))==r['height_bin']
assert maxerr<1e-10
stats={'n':len(rows),'requested_n':len(d['requested']),'unique_n':len(rows),'all_checks_pass':True,'trajectory_replay_max_error':maxerr,'source_x_values':sorted({r['origin_x'] for r in rows}),'source_y_values':sorted({r['origin_y'] for r in rows}),'speed_height_counts':dict(collections.Counter(f"{r['speed_bin']}/{r['height_bin']}" for r in rows)),'source_y_counts':dict(collections.Counter(str(r['origin_y']) for r in rows)),'side_counts':dict(collections.Counter(r['side'] for r in rows)),'ascending':sum(r['predicted_contact'][5]>0 for r in rows),'descending':sum(r['predicted_contact'][5]<0 for r in rows),'predicted_incoming_angle_range_deg':[min(r['predicted_angle_deg'] for r in rows),max(r['predicted_angle_deg'] for r in rows)],'aim_counts':dict(collections.Counter(str(r['aim']) for r in rows))}
(W/'super-gate3-validation.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))
