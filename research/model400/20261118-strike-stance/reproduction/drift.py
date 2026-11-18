import json,sys
import pandas as pd,numpy as np
from pathlib import Path
r=Path(sys.argv[1]);plant=pd.read_csv(r/'pp_mujoco_plant.csv',usecols=['wall_time_ns','left_foot_x','left_foot_y','right_foot_x','right_foot_y'])
plan=pd.read_csv(r/'pp_planner_debug.csv').query('wire_valid==1').drop_duplicates('command_seq')
anchor=json.loads((r/'pp_mujoco_plant_report.json').read_text())['home_support_frame']['anchors']
yaw=np.radians(anchor['base_yaw_deg']);axis=np.array([-np.sin(yaw),np.cos(yaw)])
def snap(t):
    sec=plant.wall_time_ns.to_numpy()/1e9;i=int(np.searchsorted(sec,t));f=plant.iloc[i]
    left=f[['left_foot_x','left_foot_y']].to_numpy();right=f[['right_foot_x','right_foot_y']].to_numpy()
    return dict(width_cm=float((left-right)@axis*100),right_drift_cm=float(np.linalg.norm(right-anchor['right_foot_xy_m'])*100),left_drift_cm=float(np.linalg.norm(left-anchor['left_foot_xy_m'])*100))
rows=[]
for p in plan.itertuples():rows.append({'source_shot':p.one_shot_flight_seq,'side':'FH' if p.swing_sign>0 else 'BH','pre':snap(p.strike_deadline_wall_s-.35),'post':snap(p.strike_deadline_wall_s+.65)})
Path(sys.argv[2]).write_text(json.dumps(rows,indent=2))
