"""Uniform offline actual-state video with timed labels and recorded planner reference."""
import argparse,csv,hashlib,json,subprocess
from pathlib import Path
from collections import deque
import numpy as np
from mujoco_eval_recurrent import Plant,dump,ROOT
from concurrent_physical_balls import ConcurrentBalls

STYLE={'width':800,'height':500,'fps':20,'playback_speed':1,'camera_lookat':[1.35,0,.85],'camera_distance':6.2,'camera_azimuth':115,'camera_elevation':-24,'reference_color':'purple dashed','actual_colors':'orange before contact; cyan after contact','version':1}
def records(p):
 with Path(p).open() as f:
  for s in f:yield json.loads(s)
def clock(t):
 cs=round(t*100);return f'{cs//360000}:{cs//6000%60:02}:{cs//100%60:02}.{cs%100:02}'
def overlay(t,summary,rows,iteration):
 launched=[r for r in rows if r['launched'] and r['launched_at_s']<=t+1e-8];row=launched[-1] if launched else None
 side='-' if row is None else ('PENDING' if not any(c['applied'] and c['sim_t']<=t for c in row['commands']) else (row['planner_side'] or 'UNASSIGNED').upper())
 def hit_time(r):return min((e['episode_t'] for e in r['events'] if e['kind']=='racket_contact'),default=float('inf'))
 raw=sum(hit_time(r)<=t for r in launched);qualified=sum(r['qualified_contact'] and hit_time(r)<=t for r in launched)
 legal=sum(r['legal_landing'] and r['physical']['landing_event'] is not None and r['physical']['landing_event']['stamp_ns']*1e-9<=t for r in launched)
 status=''
 if summary['error'] or summary['rally_failure']=='numerical_warning':status='INFRASTRUCTURE / SAFETY INTERRUPTION (not a policy fall result)'
 elif summary['rally_failure']=='fall':status=f'EARLY TERMINATION: FALL at {summary["fall_time_s"]:.2f}s | unlaunched {summary["groups"]["all"]["unlaunched"]}'
 if summary['groups']['all']['unresolved']:status+=f' | UNRESOLVED {summary["groups"]["all"]["unresolved"]}'
 return [f'MODEL {iteration} | REFERENCE-SEQUENCE DIAGNOSTIC - NOT GATE3',f'Serve {row["shot_id"] if row else 0}/100 | planner {side} | t={t:.2f}s | PPQ004',f'Raw contact balls {raw} | qualified {qualified} | legal returns {legal}', 'Orange/cyan: ACTUAL | purple dashed: PLANNER REFERENCE',status]
def references(out,summary,rows,exe):
 receipt=json.loads((exe.parent/'cpp-trace-receipt.json').read_text())
 assert hashlib.sha256(exe.read_bytes()).hexdigest()==receipt['artifact_sha256'][exe.name]
 for p,h in receipt['source_sha256'].items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h
 audit=list(csv.DictReader((out.parent/'ros/planner-audit.csv').open()));by={int(r['command_seq']):r for r in audit};paths={}
 for r in rows:
  commands=[c for c in r['commands'] if c['applied']]
  if not commands:continue
  cmd=commands[0];a=by[cmd['decoded']['identity'][0]]
  args=[a['source_time_s']]+[a[k] for k in ['est_x','est_y','est_z','est_vx','est_vy','est_vz','spin_wx_rad_s','spin_wy_rad_s','spin_wz_rad_s']]
  result=subprocess.run([str(exe),*args],capture_output=True,text=True,timeout=10,check=True)
  samples=list(csv.reader(result.stdout.splitlines()));last=samples[-1];assert last[0]=='crossing'
  assert abs(float(last[1])-cmd['decoded']['deadline_unix_s'])<1e-6
  assert np.allclose(np.array(list(map(float,last[2:5])))+[.5,.7625,.76],cmd['decoded']['position_w'],rtol=0,atol=1e-9)
  paths[r['shot_id']]={'visible_at':cmd['sim_t'],'release_at':r.get('released_at_s') or r['launched_at_s']+r['flight_s'],'position':cmd['decoded']['position_w'],
   'path':[(np.array(list(map(float,s[2:5])))+[.5,.7625,.76]).tolist() for s in samples if s[0]=='step'][::8]}
 dump(out/'planner-reference-paths.json',paths);return paths

def render(out,bundle,exe):
 out=Path(out);manifest=json.loads((Path(bundle)/'manifest.json').read_text());summary=json.loads((out/'summary.json').read_text());rows=json.loads((out/'per-ball.json').read_text())
 assert manifest['onnx_sha256']==summary['onnx_sha256'] and manifest['checkpoint_iteration']==summary['iteration']
 movie=out/'comparison.mp4'
 if movie.exists():raise FileExistsError(movie)
 paths=references(out,summary,rows,Path(exe));states=[json.loads(x) for x in (out/'render-states.jsonl').read_text().splitlines()]
 assert states and states[0]['sim_t']<=.021,'Initial idle/state recording missing'
 assert 0<=summary['simulated_seconds']-states[-1]['sim_t']<.061,'Incomplete state recording coverage'
 context=json.loads((out/'render-context.json').read_text()) if (out/'render-context.json').exists() else {}
 if context.get('infrastructure_error'):summary={**summary,'error':context['infrastructure_error']}
 subtitle=out/'comparison.ass'
 header='''[Script Info]\nScriptType: v4.00+\nPlayResX: 800\nPlayResY: 500\n[V4+ Styles]\nFormat: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,Alignment,MarginL,MarginR,MarginV,Encoding\nStyle: Default,DejaVu Sans,17,&H00FFFFFF,&H00FFFFFF,&H00000000,&H90000000,0,0,0,0,100,100,0,0,3,1,0,7,10,10,10,1\n[Events]\nFormat: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text\n'''
 with subtitle.open('x') as f:
  f.write(header)
  for i,state in enumerate(states):
   label=overlay(state['sim_t'],summary,rows,summary['iteration']);text='\\N'.join(label)
   text='{\\fs21\\b1}'+text.replace('\\N','{\\fs16\\b0}\\N',1)
   f.write(f'Dialogue: 0,{clock(i/20)},{clock((i+1)/20)},Default,,0,0,0,,{text}\n')
 p=Plant(manifest,physical=True,ball_slots=9);pool=ConcurrentBalls(p,[r['shot_id'] for r in rows]);mj=p.mj;p.model.vis.global_.offwidth=800;p.model.vis.global_.offheight=500
 renderer=mj.Renderer(p.model,height=500,width=800,max_geom=4000);camera=mj.MjvCamera();camera.lookat[:]=STYLE['camera_lookat'];camera.distance=6.2;camera.azimuth=115;camera.elevation=-24
 ff=subprocess.Popen(['ffmpeg','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','800x500','-r','20','-i','-','-vf',f"subtitles=filename='{subtitle}'",'-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(movie)],stdin=subprocess.PIPE)
 trajectory=records(out/'ball-trajectories.jsonl');sample=next(trajectory,None);history={}
 def line(scene,a,b,color):
  if scene.ngeom>=scene.maxgeom-2:return
  g=scene.geoms[scene.ngeom];mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32));mj.mjv_connector(g,mj.mjtGeom.mjGEOM_LINE,2,a,b);scene.ngeom+=1
 try:
  for state in states:
   t=state['sim_t']
   while sample is not None and sample['sim_t']<=t+1e-8:
    rec=history.setdefault(sample['shot_id'],{'history':deque(maxlen=8000),'terminal_at_s':None});rec['history'].append((np.array(sample['position_w']),sample['racket_contacts']>0))
    if not sample['active']:rec['terminal_at_s']=sample['sim_t']
    sample=next(trajectory,None)
   pool.records=history;p.data.qpos[:]=state['qpos'];mj.mj_forward(p.model,p.data);renderer.update_scene(p.data,camera=camera);pool.decorate(renderer.scene,None,clear=False)
   visible=[v for v in paths.values() if v['visible_at']<=t<v['release_at']]
   if visible:
    path=visible[-1]['path']
    for a,b in zip(path[::2],path[1::2]):line(renderer.scene,a,b,[.8,.2,1,1])
    point=np.array(visible[-1]['position'])
    for axis in np.eye(3)*.035:line(renderer.scene,point-axis,point+axis,[.8,.2,1,1])
   ff.stdin.write(renderer.render().tobytes())
 finally:
  ff.stdin.close();rc=ff.wait(timeout=60);renderer.close()
 if rc:raise RuntimeError('Offline encoding failed')
 subprocess.run(['ffmpeg','-v','error','-i',str(movie),'-f','null','-'],check=True,timeout=120)
 first=min((r['launched_at_s'] for r in rows if r['launched']),default=None);first_video=next((i/20 for i,x in enumerate(states) if first is not None and x['sim_t']>=first-1e-8),None)
 dump(out/'comparison-video.json',{'style':STYLE,'frames':len(states),'duration_s':len(states)/20,'first_serve_video_s':first_video,'first_serve_sim_s':first,'initial_idle_preserved':True,'checkpoint_iteration':summary['iteration'],'onnx_sha256':summary['onnx_sha256'],'video_sha256':hashlib.sha256(movie.read_bytes()).hexdigest(),'semantics':'Recorded actual states + recorded-command planner reference, rendered offline; no new actor/physics scoring.'})
 print('VIDEO COMPLETE',movie,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--bundle',required=True);p.add_argument('--trace-executable',required=True);a=p.parse_args();render(a.output,a.bundle,a.trace_executable)
