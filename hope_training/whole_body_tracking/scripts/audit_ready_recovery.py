#!/usr/bin/env python3
"""Supplement Gate3 with actual ready-pose recovery at the commanded station.

Does not change the original Gate3 verdict, initial-anchor audit, or ball ledger.
Each window begins after the protected 0.5 s follow-through and ends before the
next serve can change the command. Final-shot data ends at the last MOTION tick.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def longest_dwell(t, passed, max_gap=.025):
    longest = 0.
    begin = None
    previous = None
    for stamp, ok in zip(t, passed):
        if not ok:
            begin = None
        elif begin is None or previous is None or stamp - previous > max_gap:
            begin = stamp
        else:
            longest = max(longest, stamp - begin)
        previous = stamp
    return float(longest)


def audit(result_dir, reference, model_path=None):
    root = Path(result_dir)
    report = json.loads((root / "pp_rally_report.json").read_text())
    runner = json.loads((root / "pp_mujoco_plant_report.json").read_text())["runner"]
    motion_end = runner["motion_wall_time_last_ns"]
    motion_start = runner.get("motion_wall_time_first_ns", 0)
    pose = json.loads(Path(reference).read_text())
    refs = dict(zip(pose["joint_order"], pose["actual_q_reference_rad"]))
    active_names = [n for n in pose["joint_order"] if not n.startswith("head_")]
    fk = None
    if model_path is not None:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(model_path))
        state = mujoco.MjData(model)
        addresses = {n: int(model.joint(n).qposadr[0]) for n in pose['joint_order']}
        pelvis = model.body('pelvis_link')
        root_joint = int(pelvis.jntadr[0])
        if model.jnt_type[root_joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('FK requires the logged free pelvis pose')
        root_address = int(model.jnt_qposadr[root_joint])
        fk = (mujoco,model,state,addresses,root_address)
    fk_max_position_error = 0.
    columns = ["wall_time_ns", "base_x", "base_y", "base_z", "base_vx", "base_vy",
               "base_qw", "base_qx", "base_qy", "base_qz",
               "torso_qw", "torso_qx", "torso_qy", "torso_qz",
               "q_waist_pitch_joint", "q_left_knee_joint", "q_right_knee_joint",
               "left_foot_x", "left_foot_y", "right_foot_x", "right_foot_y"]
    plant = root / "pp_mujoco_plant.closed.csv"
    if not plant.exists() and not Path(str(plant)+'.gz').exists(): plant = root / "pp_mujoco_plant.csv"
    data, malformed = [], 0
    with (plant.open() if plant.exists() else gzip.open(str(plant)+'.gz', 'rt')) as stream:
        reader = csv.DictReader(stream)
        joint_columns = ['q_'+n for n in active_names]
        foot_columns = [s+'_foot_q'+axis for s in ('left','right') for axis in 'wxyz']
        hand_columns = ['left_wrist_z','right_wrist_z']
        needed = joint_columns+foot_columns+hand_columns
        derived_hands = fk is not None and any(k not in reader.fieldnames for k in hand_columns)
        available = list(reader.fieldnames) + (hand_columns if derived_hands else [])
        missing = [k for k in needed if k not in available]
        for k in needed:
            if k in available and k not in columns: columns.append(k)
        if derived_hands: columns.append('_fk_position_error_m')
        for row in reader:
            try:
                if not motion_start <= int(row['wall_time_ns']) <= motion_end: continue
                if derived_hands:
                    mujoco,model,state,addresses,root_address = fk
                    state.qpos[root_address:root_address+7] = [float(row['base_'+s]) for s in ('x','y','z','qw','qx','qy','qz')]
                    for name,address in addresses.items():
                        state.qpos[address] = float(row['q_'+name])
                    if not np.isfinite(state.qpos).all(): raise ValueError('nonfinite FK input')
                    mujoco.mj_kinematics(model,state)
                    # Refuse a model/frame mismatch rather than invent hand evidence.
                    row_error = 0.
                    for body,prefix in [('torso_Link','torso'),('left_ankle_roll_Link','left_foot'),('right_ankle_roll_Link','right_foot')]:
                        logged=np.array([float(row[prefix+'_'+axis]) for axis in 'xyz'])
                        err=float(np.linalg.norm(state.body(body).xpos-logged))
                        fk_max_position_error=max(fk_max_position_error,err)
                        row_error=max(row_error,err)
                    # xpos and qpos can straddle an integrator step. Such rows
                    # must break the full-pose dwell, not be silently trusted.
                    row['_fk_position_error_m']=row_error
                    for side in ('left','right'):
                        row[side+'_wrist_z']=float(state.body(side+'_wrist_yaw_Link').xpos[2])
                values = [float(row[k]) for k in columns]
                if not all(math.isfinite(v) for v in values): raise ValueError("nonfinite")
                data.append(values)
            except (ValueError, TypeError, KeyError):
                malformed += 1
    a = np.asarray(data)
    if not len(a): raise ValueError("no complete plant records")
    get = lambda k: a[:, columns.index(k)]
    t = get("wall_time_ns") / 1e9
    def rpy(prefix):
        w, x, y, z = [get(prefix + "_q" + k) for k in "wxyz"]
        return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    br, bp, yaw = rpy("base")
    tr, tp, _ = rpy("torso")
    width = (-np.sin(yaw)*(get("left_foot_x")-get("right_foot_x"))
             + np.cos(yaw)*(get("left_foot_y")-get("right_foot_y")))
    fixed = {
        "speed": np.hypot(get("base_vx"), get("base_vy")) <= .12,
        "base_tilt": np.arccos(np.clip(np.cos(br)*np.cos(bp), -1, 1)) <= math.radians(12),
        "heading": np.abs(yaw) <= math.radians(5),
        "waist": np.abs(get("q_waist_pitch_joint")-refs["waist_pitch_joint"]) <= math.radians(3),
        "torso_roll": np.abs(tr-math.radians(pose["metrics"]["torso_rpy_deg"][0])) <= math.radians(3),
        "torso_pitch": np.abs(tp-math.radians(pose["metrics"]["torso_rpy_deg"][1])) <= math.radians(3),
        "knees": np.maximum(np.abs(get("q_left_knee_joint")-refs["left_knee_joint"]),
                            np.abs(get("q_right_knee_joint")-refs["right_knee_joint"])) <= math.radians(5),
        "height": np.abs(get("base_z")-pose["metrics"]["pelvis_height_cm"]/100) <= .02,
        "foot_width": (width >= .25) & (width <= .35),
    }
    legacy_keys = tuple(fixed)
    joint_complete = all(k not in missing for k in joint_columns+foot_columns)
    joint_error = None
    if joint_complete:
        joint_error = np.stack([get('q_'+n)-refs[n] for n in active_names], -1)
        fixed['all_active_joints'] = np.abs(joint_error).max(-1) <= math.radians(5)
        foot_errors = []
        for i,side in enumerate(('left','right')):
            angles = np.stack(rpy(side+'_foot'), -1)
            delta = angles-np.deg2rad(pose['metrics']['foot_rpy_deg'][i])
            foot_errors.append(np.abs(np.arctan2(np.sin(delta),np.cos(delta))).max(-1))
        fixed['foot_orientation'] = np.maximum(*foot_errors) <= math.radians(5)
        fixed['selected_width'] = np.abs(get('left_foot_y')-get('right_foot_y')-pose['metrics']['foot_center_width_cm']/100) <= .02
    full_complete = not missing
    if full_complete:
        hand_error = np.stack([get(side+'_wrist_z')-pose['metrics'][side+'_wrist_height_cm']/100
                               for side in ('left','right')], -1)
        fixed['hand_height'] = np.abs(hand_error).max(-1) <= .03
        if derived_hands:
            fixed['hand_fk_consistency'] = get('_fk_position_error_m') <= .002
            if not fixed['hand_fk_consistency'].any():
                raise ValueError('No FK rows match logged torso/feet; verify model and timing')
    rows = report["rows"]
    output = []
    for i, row in enumerate(rows):
        identity = {k: row.get(k) for k in ("serve", "flight_id", "shot_id")}
        contact = (row.get("runner_physical_contact_pose") or {}).get("physical_contact_wall_time_ns")
        target = row.get("command_station")
        if contact is None or target is None:
            output.append({**identity, "status": "MISSING_CONTACT_OR_COMMAND"})
            continue
        end = motion_end
        if i + 1 < len(rows):
            next_start = (rows[i+1].get("physical_outcome") or {}).get("first_stamp_ns")
            if next_start is None:
                output.append({**identity, "status": "MISSING_NEXT_SERVE_BOUNDARY"})
                continue
            end = min(end, next_start)
        available = max(0., (end-contact)/1e9-.5)
        if i+1 < len(rows) and available < .3:
            output.append({**identity, "status": "INTERRUPTED_BY_NEXT_BALL",
                           "available_recovery_s": available})
            continue
        mask = (t >= contact/1e9 + .5) & (t < end/1e9)
        if not np.any(mask):
            output.append({**identity, "status": "NO_RECOVERY_WINDOW", "available_recovery_s": available})
            continue
        error = np.hypot(get("base_x")-target[0], get("base_y")-target[1])
        checks = {**fixed, "station": error <= .05}
        tt = t[mask]
        full = np.logical_and.reduce([v[mask] for v in checks.values()])
        legacy = np.logical_and.reduce([checks[k][mask] for k in (*legacy_keys,'station')])
        joint_pose = np.logical_and.reduce([v[mask] for k,v in checks.items() if not k.startswith('hand_')])
        station_state = np.logical_and.reduce([checks[k][mask] for k in ("station", "speed", "base_tilt", "heading")])
        output.append({**identity, "status": "OBSERVED", "command_station_m": target,
                       "available_recovery_s": available,
                       "window_duration_s": float(tt[-1]-tt[0]), "samples": int(mask.sum()),
                       "max_sample_gap_s": float(np.diff(tt).max()) if len(tt)>1 else None,
                       "station_error_mean_m": float(error[mask].mean()),
                       "criterion_fraction": {k: float(v[mask].mean()) for k, v in checks.items()},
                       "station_dwell_s": longest_dwell(tt, station_state),
                       "legacy_partial_pose_dwell_s": longest_dwell(tt, legacy),
                       "full_joint_pose_dwell_s": longest_dwell(tt, joint_pose) if joint_complete else None,
                       "full_pose_dwell_s": longest_dwell(tt, full) if full_complete else None,
                       "hand_height_error_rms_m": np.sqrt((hand_error[mask]**2).mean(0)).tolist() if full_complete else None,
                       "joint_task_error_rms_deg": (dict(zip(active_names,np.rad2deg(np.sqrt((joint_error[mask]**2).mean(0))).tolist())) if joint_complete else None)})
    return {"contract": "command_station_actual_full_body_pose_v8_diagnostic",
            "scope": "Supplement only; original Gate3 and initial-anchor verdicts are preserved",
            "reference": str(reference), "malformed_plant_rows": malformed,
            "missing_full_body_columns": missing, "full_body_evidence_complete": full_complete,
            "wrist_height_source": ("actual_q_and_base_forward_kinematics" if derived_hands else "plant_columns" if full_complete else "unavailable"),
            "fk_model_path": str(model_path) if model_path else None,
            "fk_model_xml_sha256": hashlib.sha256(Path(model_path).read_bytes()).hexdigest() if model_path else None,
            "fk_max_logged_body_position_error_m": fk_max_position_error if derived_hands else None,
            "fk_consistent_row_fraction": float(fixed['hand_fk_consistency'].mean()) if derived_hands else None,
            "thresholds": {"station_m": .05, "speed_mps": .12, "base_tilt_deg": 12,
                           "heading_deg": 5, "waist_torso_deg": 3, "knees_deg": 5,
                           "all_active_joint_deg": 5, "foot_orientation_deg": 5,
                           "selected_width_error_m": .02, "hand_height_error_m": .03,
                           "height_m": .02, "width_m": [.25, .35], "dwell_s": .3,
                           "protected_followthrough_s": .5},
            "shots": len(rows), "observed_windows": sum(r["status"] == "OBSERVED" for r in output),
            "eligible_recovery_windows": sum(r.get('available_recovery_s',0)>=.3 for r in output),
            "interrupted_by_next_ball": sum(r['status']=='INTERRUPTED_BY_NEXT_BALL' for r in output),
            "recovery_rate_denominator": "eligible_recovery_windows; short next-ball interruptions are not pose failures",
            "station_recovered": sum(r.get("station_dwell_s", 0) >= .3 for r in output),
            "legacy_partial_pose_recovered": sum(r.get("legacy_partial_pose_dwell_s", 0) >= .3 for r in output),
            "full_joint_pose_recovered": sum((r.get("full_joint_pose_dwell_s") or 0) >= .3 for r in output) if joint_complete else None,
            "full_ready_pose_recovered": sum((r.get("full_pose_dwell_s") or 0) >= .3 for r in output) if full_complete else None,
            "original_session_anchor_recovered": report.get("recovered_engaged_serves"), "rows": output}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", required=True)
    parser.add_argument("--reference", default=str(Path(__file__).resolve().parents[1]/"cfg/references/hold_pose_B_32_5_K20.json"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--model", help="Exact Gate3 MuJoCo XML for reconstructing unlogged wrist heights; FK is checked against logged torso/feet")
    args = parser.parse_args()
    result = audit(args.result_dir, args.reference, args.model)
    Path(args.output).write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k: v for k, v in result.items() if k != "rows"}, indent=2))
