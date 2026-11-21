"""Offline measured alignment clips; geometry markers never enter physics."""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import Plant,dump
from concurrent_physical_balls import ConcurrentBalls
from diagnose_strike_alignment import read_rows,bracket


def render(directory,bundle):
    out=Path(directory);report=json.loads((out/'alignment.json').read_text())
    manifest=json.loads((Path(bundle)/'manifest.json').read_text())
    assert manifest['onnx_sha256']==report['onnx_sha256']
    p=Plant(manifest,physical=True,ball_slots=9);pool=ConcurrentBalls(p,[x['shot_id'] for x in report['results']])
    mj=p.mj;p.model.vis.global_.offwidth=960;p.model.vis.global_.offheight=600
    renderer=mj.Renderer(p.model,height=600,width=960,max_geom=2000)
    camera=mj.MjvCamera();camera.lookat[:]=[.95,0,1.];camera.distance=4.5;camera.azimuth=115;camera.elevation=-24
    clips=[]
    def line(scene,a,b,color,width=.004):
        g=scene.geoms[scene.ngeom];mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32));mj.mjv_connector(g,mj.mjtGeom.mjGEOM_CAPSULE,width,a,b);scene.ngeom+=1
    try:
        for side in ['forehand','backhand']:
            result=next(x for x in report['results'] if x['side']==side);sid=result['shot_id'];data=read_rows(out/f'shot-{sid}-surface-samples.jsonl');deadline=result['planned_deadline_sim_s']
            times=list(np.arange(deadline-.5,deadline,.0125))+[deadline]*30+list(np.arange(deadline,deadline+.201,.0125))
            path=out/f'{side}-alignment.mp4'
            if path.exists():raise FileExistsError(path)
            caption=f'{side.upper()} shot {sid} | planned strike t={deadline:.6f}s | 4x slow + strike freeze'
            legend='Magenta intended point | orange ACTUAL ball | blue collision-face normal'
            ff=subprocess.Popen(['ffmpeg','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x600','-r','20','-i','-','-vf',f"drawtext=text='{caption}':x=12:y=12:fontsize=19:fontcolor=white:box=1:boxcolor=black@0.8,drawtext=text='{legend}':x=12:y=40:fontsize=17:fontcolor=white:box=1:boxcolor=black@0.8",'-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE)
            try:
                for index,t in enumerate(times):
                    q,_=bracket(data,t,'qpos');p.data.qpos[:]=q;mj.mj_normalizeQuat(p.model,p.data.qpos);mj.mj_forward(p.model,p.data)
                    renderer.update_scene(p.data,camera=camera);scene=renderer.scene;target=np.array(result['planner_command']['decoded']['position_w'])
                    for axis in np.eye(3):line(scene,target-axis*.045,target+axis*.045,[1,0,1,1])
                    g=mj.mj_name2id(p.model,mj.mjtObj.mjOBJ_GEOM,'right_racket_collision');origin=p.data.geom_xpos[g];normal=p.data.geom_xmat[g].reshape(3,3)[:,0]
                    line(scene,origin-normal*.14,origin+normal*.14,[.05,.35,1,1],.007)
                    history=[np.array(x['ball_w']) for x in data if deadline-.6<=x['sim_t']<=t][::10]
                    for a,b in zip(history[:-1],history[1:]):line(scene,a,b,[1,.45,0,.7],.002)
                    rgb=renderer.render();ff.stdin.write(rgb.tobytes())
                    if abs(t-deadline)<1e-9 and not (out/f'{side}-at-strike.png').exists():
                        subprocess.run(['ffmpeg','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x600','-i','-','-frames:v','1',str(out/f'{side}-at-strike.png')],input=rgb.tobytes(),check=True)
            finally:
                ff.stdin.close();code=ff.wait(timeout=30)
            if code:raise RuntimeError('Alignment encoding failed')
            clips.append(path)
    finally:renderer.close()
    listing=out/'alignment-clips.txt';listing.write_text(''.join(f"file '{x}'\n" for x in clips))
    subprocess.run(['ffmpeg','-v','error','-f','concat','-safe','0','-i',str(listing),'-c','copy',str(out/'alignment.mp4')],check=True)
    dump(out/'alignment-video-receipt.json',{'method':'Verified recorded-action reconstruction and interpolated actual poses; no actor/planner rerun or new scoring. Magenta marker is intended strike only. Blue line is compiled collision mesh thin-axis normal, two-sided.', 'checkpoint_sha256':report['checkpoint_sha256'],'onnx_sha256':report['onnx_sha256'],'sides':['forehand','backhand'],'video':str(out/'alignment.mp4')})

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--alignment',required=True);p.add_argument('--bundle',required=True);a=p.parse_args();render(a.alignment,a.bundle)
