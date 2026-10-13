"""Read-only MuJoCo video replay from measured plant and per-ball telemetry."""
import argparse,csv,json,os,time,hashlib
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
import subprocess,imageio_ffmpeg

p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--preview',action='store_true');p.add_argument('--fps',type=int,default=24)
a=p.parse_args();run=a.run.resolve();receipt=json.loads((run/'run.json').read_text())
scene=Path(receipt['runtime']['sim_install'])/'bin/cfg/model/a3_pingpong/a3_pingpong.xml'
m=mujoco.MjModel.from_xml_path(str(scene));m.vis.global_.offwidth=1280;m.vis.global_.offheight=720
mujoco.mj_saveModel(m,str(run/'render_model.mjb'))
d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=720,width=1280,max_geom=10000)
cam=mujoco.MjvCamera();cam.lookat[:]=[.85,-.7625,.8];cam.distance=5.7;cam.azimuth=115;cam.elevation=-25
opt=mujoco.MjvOption();opt.geomgroup[3]=0
qnames=[m.joint(i).name for i in range(m.njnt) if m.jnt_type[i]!=mujoco.mjtJoint.mjJNT_FREE]
qaddr=[int(m.joint(n).qposadr[0]) for n in qnames]
root=int(m.joint('pelvis_free_joint').qposadr[0])
balladdrs=[int(m.joint(i).qposadr[0]) for i in range(m.njnt) if m.joint(i).name.startswith('gate3_ball_free_joint')]
plant_path=run/'pp_mujoco_plant.csv';ball_path=run/'pp_ball_trajectories.csv'
if a.preview and not plant_path.exists():plant_path=Path('/tmp/pp_mujoco_plant.csv');ball_path=Path('/tmp/pp_ball_trajectories.csv')
poses=[];pt=[]
with plant_path.open() as f:
 reader=csv.reader(f);head=next(reader);ti=head.index('wall_time_ns');inds=[head.index('base_'+k) for k in ('x','y','z','qw','qx','qy','qz')]+[head.index('q_'+n) for n in qnames]
 for row in reader:
  if len(row)==len(head):pt.append(int(row[ti]));poses.append([float(row[i]) for i in inds])
pt=np.asarray(pt,dtype=np.int64);poses=np.asarray(poses)
balls={}
with ball_path.open() as f:
 for r in csv.DictReader(f):
  shot=int(r['shot_id']);balls.setdefault(shot,[]).append((int(r['wall_time_ns']),*[float(r[k]) for k in ('x','y','z')],int(r['active'])))
balls={k:np.asarray(v) for k,v in balls.items()}
assert np.all(np.diff(pt)>0), 'robot trace is not ordered'
assert all(np.all(np.diff(b[:,0])>0) for b in balls.values()), 'ball trace is not ordered'
first=min(b[0,0] for b in balls.values());last=max(b[-1,0] for b in balls.values())
start=max(int(first-20e9),int(pt[0]));end=min(int(last+1e9),int(pt[-1]))
# Fixed view fits the measured horizontal flight envelope, including misses.
allxyz=np.concatenate([b[:,1:4] for b in balls.values()]);xmin=min(-1.15,float(allxyz[:,0].min()));xmax=max(3.9,float(allxyz[:,0].max()))
cam.lookat[0]=(xmin+xmax)/2;cam.distance=max(5.7,(xmax-xmin)*1.08)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
bold=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',24)
family={k:('clean' if k<=6 else 'rapid') for k in balls}
colors={'clean':[.35,.8,1,1],'rapid':[1,.9,.3,1],'baseline':[.35,.8,1,1],'fast_deep':[1,.4,.3,1],'high_arc':[.4,1,.45,1],'short':[1,.9,.3,1],'diagonal':[1,.45,.9,1]}
starts=np.asarray([balls[k][0,0] for k in sorted(balls)])
coverage={k:0 for k in balls};terminal_coverage={k:0 for k in balls}

def frame(stamp):
 idx=int(np.clip(np.searchsorted(pt,stamp,side='right')-1,0,len(pt)-1));pose=poses[idx]
 d.qpos[root:root+7]=pose[:7];d.qpos[qaddr]=pose[7:]
 for addr in balladdrs:d.qpos[addr:addr+3]=[100,0,-20];d.qpos[addr+3:addr+7]=[1,0,0,0]
 visible=[]
 for k,b in balls.items():
  if b[0,0]<=stamp<=b[-1,0]+.5e9:
   j=int(np.clip(np.searchsorted(b[:,0],stamp,side='right')-1,0,len(b)-1));visible.append((k,b,j))
   coverage[k]+=1
   if stamp>=b[-1,0]:terminal_coverage[k]+=1
 assert len(visible)<=len(balladdrs), 'video ball slots exhausted'
 for addr,(_,b,j) in zip(balladdrs,visible):d.qpos[addr:addr+3]=b[j,1:4]
 mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt)
 for k,b,j in visible:
  # Lines join recorded points only; no synthetic physics or hidden reset.
  ids=list(range(0,j+1,8))
  if not ids or ids[-1]!=j:ids.append(j)
  color=np.array(colors[family[k]],dtype=np.float32)
  for i1,i2 in zip(ids[:-1],ids[1:]):
   p1=b[i1,1:4];p2=b[i2,1:4]
   if np.linalg.norm(p2-p1)<.003:continue
   if renderer.scene.ngeom>=renderer.scene.maxgeom:break
   g=renderer.scene.geoms[renderer.scene.ngeom]
   mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),color)
   mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,.003,p1,p2);renderer.scene.ngeom+=1
 image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
 draw.rectangle((0,0,1280,82),fill=(17,24,32));draw.text((20,10),'Gate3 | model_14600 | 30 balls | certification: FAIL',font=bold,fill='white')
 t=(stamp-first)*1e-9;launched=int(np.searchsorted(starts,stamp,side='right'))
 label=f'{max(t,0):6.1f} s  |  Shot {launched:03d}/30  |  '+(family.get(launched,'policy entry / idle') if launched else 'policy entry / idle')
 draw.text((20,47),label,font=font,fill=(190,218,240))
 draw.rectangle((0,679,1280,720),fill=(17,24,32));draw.text((20,690),'Measured-state replay | 1x speed | full physical flights | zero initial spin',font=small,fill='white')
 draw.text((900,690),'Active / fading: '+','.join(str(v[0]) for v in visible),font=small,fill='white')
 return np.asarray(image)

if a.preview:
 stamp=first+1.1e9
 Image.fromarray(frame(stamp)).save(run/'video_preview.png');print('PREVIEW_SAVED',flush=True)
else:
 fps=a.fps;out=run/'gate3_30_balls.mp4';n=int(np.ceil((end-start)*1e-9*fps))+1
 writer=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-crf','20','-preset','veryfast','-movflags','+faststart','-threads','2',str(out)],stdin=subprocess.PIPE)
 begin=time.monotonic()
 for i in range(n):
  stamp=min(start+i*1e9/fps,end);im=frame(stamp);writer.stdin.write(im.tobytes())
  if i in (0,n//2,n-1):Image.fromarray(im).save(run/f'video_frame_{i:06d}.jpg')
  if i%240==0:print(f'VIDEO {i}/{n} frames; elapsed {time.monotonic()-begin:.1f}s',flush=True)
 writer.stdin.close();assert writer.wait()==0;renderer.close()
 report=dict(video=str(out),width=1280,height=720,fps=fps,frames=n,duration_s=n/fps,playback_speed=1,source='actual robot and physical-ball telemetry; zero-order sampled poses; no physics rerun',robot_source=str(plant_path),ball_source=str(ball_path),scene_source=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),mujoco_render_version=mujoco.__version__,visibility_audit='Each shot and terminal is inserted in the rendered scene during its measured time range; normal scene occlusion applies.',start_wall_ns=start,end_wall_ns=end,shot_count=len(balls),shot_visible_frames=coverage,shot_terminal_visible_frames=terminal_coverage,all_shots_visible=len(coverage)==30 and all(coverage.values()),all_endpoints_visible=len(terminal_coverage)==30 and all(terminal_coverage.values()))
 (run/'video_receipt.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if not isinstance(v,dict)}),flush=True)
