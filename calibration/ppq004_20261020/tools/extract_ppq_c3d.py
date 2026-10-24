"""Extract a 300 Hz ppq recording in its existing table-corner world frame.

Requires the operator-confirmed ball-centre/world conventions described in
docs/operations/ball_physics_calibration.md. No legacy venue transform is used.
"""
import argparse,json,hashlib,warnings
from pathlib import Path
import numpy as np,c3d
from scipy.spatial.transform import Rotation as R
from extract_canonical import rigid_pose,sphere_center
warnings.filterwarnings('ignore')
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('source',type=Path)
ap.add_argument('--output',type=Path,required=True)
args=ap.parse_args();src=args.source;out=args.output
out.mkdir(parents=True,exist_ok=True)
with src.open('rb') as f:
 r=c3d.Reader(f);labels=[l.replace('\\x00','').split('\x00')[0].strip() for l in r.point_labels];rate=float(r.point_rate)
 assert rate == 300.0, 'This ppq calibration windowing is validated at 300 Hz only'
 units=r.get('POINT:UNITS').string_value.replace('\\x00','').split('\x00')[0].strip().lower()
 assert units == 'mm', f'Expected millimetres, got {units!r}'
 prefixes={'ball':['b_Ball_'],'p1':['b_PPP1_','b_ppp3_'],'p2':['b_PPP2_','b_ppp4_'],'base':['b_UCB_P1_']}
 idx={}
 for key,choices in prefixes.items():
  prefix=next((s for s in choices if any(l.startswith(s) for l in labels)),None)
  if prefix is not None:idx[key]=[i for i,l in enumerate(labels) if l.startswith(prefix)]
 assert 'ball' in idx and len(idx['ball'])>=4, 'No rigid Ball marker group found'
 data={s:[] for s in idx};frames=[]
 for frame,pts,_ in r.read_frames():
  frames.append(frame)
  for s,ids in idx.items():
   p=pts[ids,:3].copy()*.001;p[pts[ids,3]<0]=np.nan;data[s].append(p)
npz={'t':np.asarray(frames)/rate,'rate':rate};report={'source':str(src.resolve()),'first_frame':int(frames[0]),'last_frame':int(frames[-1]),'frames':len(frames),'rate_hz':rate,'duration_s':len(frames)/rate,'labels':labels,'raw_sha256':hashlib.sha256(src.read_bytes()).hexdigest()}
for name,points in data.items():
 points=np.asarray(points);present,cent,q,rms,template=rigid_pose(points,4)
 key=name
 npz.update({key+'_markers':points,key+'_present':present,key+'_centroid':cent,key+'_quat':q,key+'_rms':rms,key+'_template':template})
 report[key]={'markers':points.shape[1],'tracked':int(present.sum()),'centroid_quantiles':np.nanpercentile(cent,[0,5,50,95,100],axis=0).tolist(),'rigid_rms_mm':(np.nanpercentile(rms,[50,90,99])*1000).tolist()}
 if key=='ball':
  c,r,e=sphere_center(template);pos=cent.copy();pos[present]+=R.from_quat(q[present]).apply(c)
  npz['ball_pos']=pos;npz['sphere_offset']=c;report['ball'].update(sphere_offset=c.tolist(),sphere_radius=r,sphere_residual=e)
np.savez_compressed(out/'raw_extracted.npz',**npz);(out/'raw_manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
