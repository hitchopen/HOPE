"""Offline software video of recorded actual states, NOT a physics resimulation."""
import argparse,json
from collections import deque
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import Plant,ObsTarget,dump
from concurrent_physical_balls import ConcurrentBalls
from recurrent_physical_ball import VideoRecorder


def render(output,bundle):
    out=Path(output);manifest=json.loads((Path(bundle)/'manifest.json').read_text())
    config=json.loads((out/'configuration.json').read_text())
    if config['policy_manifest']['onnx_sha256']!=manifest['onnx_sha256']:raise ValueError('Video bundle does not match evaluation')
    ids=[r['shot_id'] for r in json.loads((out/'ledger.json').read_text())]
    plant=Plant(manifest,physical=True,ball_slots=9);pool=ConcurrentBalls(plant,ids)
    video=out/'diagnostic.mp4'
    if video.exists():raise FileExistsError('Refusing existing video')
    recorder=VideoRecorder(plant,video,fps=20,concurrent=True)
    trajectories=(out/'ball-trajectories.jsonl').open();states=(out/'render-states.jsonl').open()
    next_sample=next(trajectories,None);records={};count=0
    try:
        for line in states:
            frame=json.loads(line)
            while next_sample is not None:
                sample=json.loads(next_sample)
                if sample['sim_t']>frame['sim_t']+1e-8:break
                sid=sample['shot_id']
                r=records.setdefault(sid,{'history':deque(maxlen=8000),'terminal_at_s':None})
                r['history'].append((np.array(sample['position_w']),sample['racket_contacts']>0))
                if not sample['active']:r['terminal_at_s']=sample['sim_t']
                next_sample=next(trajectories,None)
            pool.records=records
            plant.data.qpos[:]=frame['qpos'];plant.mj.mj_forward(plant.model,plant.data)
            target=None if frame['target_position'] is None else ObsTarget(np.array(frame['target_position']),np.array(frame['target_velocity']),0.)
            # The saved states are nominally 20fps, with <20ms sample quantization.
            recorder.capture(target,0,count/20.);count+=1
    finally:
        recorder.close();trajectories.close();states.close()
    dump(out/'video-receipt.json',{'frames':count,'fps':20,'video':str(video),
        'semantics':'Recorded actual qpos and measured trajectory playback; no actor inference, planner solve, new collision or score computed during rendering.',
        'onnx_sha256':manifest['onnx_sha256']})
    print('Recorded-state video:',video,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--bundle',required=True)
    a=p.parse_args();render(a.output,a.bundle)
