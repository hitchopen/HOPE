#!/usr/bin/env python3
"""Generate a separate Serve025 candidate; preserve events and official final pose."""
import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import validate_spin001_serve_mujoco as sim


def smooth(x):
    x=np.clip(x,0,1)
    return x*x*x*(10-15*x+6*x*x)


def generate(source, output, width=.03, ankle=0., shift=(0.,0.,0.),
             rotation=(0.,0.,0.), advance=0., wide_stand=False, squat=0.):
    with open(source,newline='') as f:
        reader=csv.DictReader(f);fields=reader.fieldnames;rows=list(reader)
    frames=[r for r in rows if r['row_kind']=='FRAME']
    original=np.array([[float(r['q_rad::'+j]) for j in sim.SDK_ORDER] for r in frames])
    q=original.copy()
    weight=1-smooth((np.arange(len(q))-167)/300)
    lateral=np.array([1,-1,-1,1])*width
    q[:,[20,24,26,30]]+=(np.ones_like(weight) if wide_stand else weight)[:,None]*lateral
    q[:,[23,29]]+=weight[:,None]*ankle
    q[:,[19,22,23,25,28,29]]+=weight[:,None]*np.array([-1,2,-1,-1,2,-1])*squat
    errors=[]
    if np.linalg.norm(shift)+np.linalg.norm(rotation)>0:
        model,bindings=sim._compile_model(sim.DEFAULT_MODEL);qa=np.array(bindings)[:,0]
        data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('stand').id)
        arm=np.arange(12,19)
        limits=np.array([model.joint(sim.SDK_ORDER[j]).range for j in arm])
        site=model.site('right_racket').id
        for i in range(len(q)):
            data.qpos[qa]=q[i];mujoco.mj_forward(model,data)
            target=data.site_xpos[site].copy()+weight[i]*np.array(shift)
            orient=Rotation.from_rotvec(np.radians(rotation)*weight[i]).as_matrix()@data.site_xmat[site].reshape(3,3)
            reference=q[i,arm].copy()
            def residual(x):
                data.qpos[qa[arm]]=x;mujoco.mj_forward(model,data)
                return np.r_[12*(data.site_xpos[site]-target),
                    2*Rotation.from_matrix(data.site_xmat[site].reshape(3,3)@orient.T).as_rotvec(),
                    .03*(x-reference)]
            x0=reference if i==0 else q[i-1,arm]+original[i,arm]-original[i-1,arm]
            sol=least_squares(residual,np.clip(x0,limits[:,0]+.021,limits[:,1]-.021),
                bounds=(limits[:,0]+.021,limits[:,1]-.021),max_nfev=45,ftol=1e-9,gtol=1e-9,xtol=1e-9)
            q[i,arm]=sol.x
            errors.append(float(np.linalg.norm(residual(sol.x)[:3])/12))
    if advance:
        grid=np.arange(len(q));warp=advance*(1-smooth((grid-110)/57))
        for j in range(12,19):q[:,j]=np.interp(grid+warp,grid,q[:,j])
    # Preserve the native reader's per-frame and joint-specific velocity envelope.
    for i in range(1,len(q)):
        max_step=np.full(31,.029 if i<=52 or i>=167 else .069)
        arm_velocity=np.array([13.613568165555769]*2+[15.707963267948966]*3+[12.775810124598491]*2)
        max_step[12:19]=np.minimum(max_step[12:19],.49*arm_velocity*.01)
        q[i]=np.clip(q[i],q[i-1]-max_step,q[i-1]+max_step)
    q[-1]=original[-1]
    if wide_stand:q[-1,[20,24,26,30]]+=lateral
    dq=np.gradient(q,.01,axis=0);ddq=np.gradient(dq,.01,axis=0)
    dq[[0,-1]]=0;ddq[[0,-1]]=0
    for i,row in enumerate(frames):
        for prefix,values in [('q_rad',q),('qd_rad_s',dq),('qdd_rad_s2',ddq)]:
            for j,name in enumerate(sim.SDK_ORDER):row[prefix+'::'+name]=f'{values[i,j]:.12g}'
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    info=dict(source=str(source),width_roll_rad=width,ankle_pitch_rad=ankle,
        racket_translation_m=list(shift),racket_rotation_deg=list(rotation),advance_frames=advance,
        wide_stand=wide_stand,squat_rad=squat,max_ik_position_error_m=max(errors,default=0),max_step_rad=float(abs(np.diff(q,axis=0)).max()))
    output.with_suffix('.generation.json').write_text(json.dumps(info,indent=2)+'\n')
    return info


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--width',type=float,default=.03);p.add_argument('--ankle',type=float,default=0.)
    p.add_argument('--shift',type=float,nargs=3,default=(0.,0.,0.))
    p.add_argument('--rotation',type=float,nargs=3,default=(0.,0.,0.))
    p.add_argument('--advance',type=float,default=0.)
    p.add_argument('--wide-stand',action='store_true')
    p.add_argument('--squat',type=float,default=0.)
    a=p.parse_args();print(json.dumps(generate(a.source,a.output,a.width,a.ankle,a.shift,a.rotation,a.advance,a.wide_stand,a.squat)))
