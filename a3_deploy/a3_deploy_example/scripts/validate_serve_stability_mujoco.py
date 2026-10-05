#!/usr/bin/env python3
"""Native Stand -> Serve controller, floating-base MuJoCo and physical ball.

No HAL, ROS command publisher or hardware connection. The ball is released
from the modeled palm; release delay/offset are explicit simulation assumptions.
Gate3's contact law is applied only after MuJoCo detects actual contact.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile


def run(csv, library, ball_library, output=None, delay=.04, payload=0.,
        offset=(0., 0., 0.), profile="hope", hold=2., rate=100., stand_offset=0.,
        offset_frame="world", command_times=None, release_lead_frames=0,
        model_path=None, release_site="left_palm", release_normal_offset=.024,
        initial_snapshot=None, handoff_frame=None, return_seconds=None):
    if offset_frame not in ("world", "palm"):
        raise ValueError("offset_frame must be world or palm")
    if command_times is not None:
        command_times = np.asarray(command_times, dtype=float)
        if len(command_times) < int(rate * (22 + hold)) + 1 or command_times[0] != 0 or np.any(np.diff(command_times) < .001):
            raise ValueError("command schedule must start at zero, span the run, and have >=1 ms spacing")
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib = ctypes.CDLL(str(Path(library).resolve()))
    lib.probe_create.argtypes = [ctypes.c_char_p, ptr]
    lib.probe_create.restype = ctypes.c_void_p
    lib.probe_destroy.argtypes = [ctypes.c_void_p]
    lib.probe_action.argtypes = [ctypes.c_void_p, ctypes.c_int, ptr]
    lib.probe_serve_with_gyro.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr, ptr]
    ball = ctypes.CDLL(str(Path(ball_library).resolve()))
    ball.contact.argtypes = [ptr] * 5
    ball.flight.argtypes = [ptr] * 3
    model, bindings = sim._compile_model(sim.DEFAULT_MODEL if model_path is None else Path(model_path))
    if profile == "hope":
        apply_hope_contact_profile(model)
    elif profile == "vendor":
        for side in ("left", "right"):
            for i in range(model.ngeom):
                name = model.geom(i).name
                if name and side in name and "ankle" in name and "collision" in name:
                    model.geom_contype[i] = model.geom_conaffinity[i] = 0
    qa, va, ac = np.array(bindings).T
    stand_q, kp, kd = sim._official_stand_vectors()
    stand_q[[20, 24, 26, 30]] += np.array([1, -1, -1, 1]) * stand_offset
    command = np.array([stand_q, stand_q * 0, stand_q * 0, kp, kd])
    probe = lib.probe_create(str(Path(csv).resolve()).encode(), command)
    if not probe:
        raise ValueError("native CSV/controller validation failed")
    for name, value, ctype in (("probe_handoff_frame", handoff_frame, ctypes.c_int),
                               ("probe_return_seconds", return_seconds, ctypes.c_double)):
        if value is not None:
            setter = getattr(lib, name)
            setter.argtypes = [ctypes.c_void_p, ctype]
            if setter(probe, value) != 0:
                lib.probe_destroy(probe)
                raise ValueError(f"invalid {name}: {value}")
    if release_lead_frames:
        lib.probe_release_lead_frames.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        if lib.probe_release_lead_frames(probe, release_lead_frames) != 0:
            lib.probe_destroy(probe)
            raise ValueError("invalid release lead")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    data.qpos[:2] = [-.45, -.7625]
    data.qpos[qa] = stand_q
    data.qvel[:] = 0
    warmup_ticks = int(rate * 2)
    if initial_snapshot is not None:
        initial = np.load(initial_snapshot)
        if initial['qpos'].shape != data.qpos.shape or initial['qvel'].shape != data.qvel.shape or initial['command'].shape != command.shape:
            raise ValueError("initial snapshot does not match this plant/command layout")
        if not all(np.isfinite(initial[k]).all() for k in ('qpos', 'qvel', 'command')):
            raise ValueError("nonfinite initial snapshot")
        data.qpos[:] = initial['qpos']
        data.qvel[:] = initial['qvel']
        command = initial['command'].copy()
        warmup_ticks = 0
    bj = model.joint("gate3_ball_free_joint").id
    bq, bv = model.jnt_qposadr[bj], model.jnt_dofadr[bj]
    bb = model.jnt_bodyid[bj]
    pelvis = model.body("pelvis_link").id
    torso = model.body("torso_Link").id
    wrist = model.body("left_wrist_yaw_Link").id
    palm = model.site(release_site).id
    racket = model.site("right_racket").id
    rgeom = model.geom("right_racket_collision").id
    feet = [model.site(x + "_foot").id for x in ("left", "right")]
    data.qpos[bq:bq+3] = [100, 0, -10]
    mujoco.mj_forward(model, data)
    initial_feet = data.site_xpos[feet].copy()
    poses, trace, commands, contacts = [], [], [], []
    phase = 0
    wait_ticks = 0
    play_time = None
    release_time = None
    released = False
    release_geometry = None
    previous = set()
    fell = False
    entry_jump = None
    min_racket_distance = 100.
    try:
        for tick in range(int(rate * (22 + hold))):
            if tick == warmup_ticks:
                assert lib.probe_action(probe, 0, command) == 0
            if phase == 5:
                wait_ticks += 1
                if wait_ticks == int(rate * hold):
                    assert lib.probe_action(probe, 1, command) == 0
                    play_time = float(data.time)
                    release_frame = 48 - release_lead_frames
                    release_time = (float(command_times[tick + release_frame]) if command_times is not None
                                    else play_time + release_frame/rate) + delay
            before = command.copy()
            if tick >= warmup_ticks:
                phase = lib.probe_serve_with_gyro(probe, data.qpos[qa].copy(),
                    data.qvel[va].copy(), data.xquat[pelvis].copy(), data.qvel[3:6].copy(), command)
                if phase < 0 or phase == 16:
                    raise RuntimeError(f"native Serve controller fault at tick {tick}")
            if tick == warmup_ticks:
                entry_jump = np.max(abs(command-before), axis=1).tolist()
            next_time = command_times[tick+1] if command_times is not None else (tick+1)/rate
            for _ in range(round(next_time*1000)-round(data.time*1000)):
                torque = command[3]*(command[0]-data.qpos[qa]) + command[4]*(command[1]-data.qvel[va]) + command[2]
                data.ctrl[ac] = np.clip(torque, model.actuator_ctrlrange[ac, 0], model.actuator_ctrlrange[ac, 1])
                data.xfrc_applied[wrist, 2] = -payload*9.81
                if not released and release_time is not None and data.time >= release_time:
                    palm_rotation = data.site_xmat[palm].reshape(3,3).copy()
                    displacement = palm_rotation @ np.asarray(offset) if offset_frame == "palm" else np.asarray(offset)
                    data.qpos[bq:bq+3] = data.site_xpos[palm] + release_normal_offset*palm_rotation[:,2] + displacement
                    data.qvel[bv:bv+6] = 0
                    mujoco.mj_forward(model, data)
                    overlaps = []
                    for ct in data.contact:
                        names = (model.geom(ct.geom1).name, model.geom(ct.geom2).name)
                        if "gate3_ball_collision" in names and ct.dist < 0:
                            overlaps.append(dict(geoms=names, penetration_m=float(-ct.dist)))
                    release_geometry = dict(world_offset_m=displacement.tolist(),
                        palm_normal=palm_rotation[:,2].tolist(), initial_overlaps=overlaps)
                    released = True
                if not released:
                    data.qpos[bq:bq+3] = [100, 0, -10]
                    data.qvel[bv:bv+6] = 0
                pre = data.qvel[bv:bv+6].copy()
                if released:
                    force = np.zeros(3)
                    ball.flight(pre[:3].copy(), pre[3:].copy(), force)
                    data.xfrc_applied[bb, :3] = force*model.body_mass[bb]
                mujoco.mj_step(model, data)
                if released:
                    min_racket_distance = min(min_racket_distance, float(np.linalg.norm(data.qpos[bq:bq+3]-data.site_xpos[racket])))
                    touching = {}
                    for ct in data.contact:
                        names = (model.geom(ct.geom1).name, model.geom(ct.geom2).name)
                        if "gate3_ball_collision" in names:
                            touching[names[1] if names[0] == "gate3_ball_collision" else names[0]] = ct
                    for name, ct in touching.items():
                        if name in previous:
                            continue
                        contact_geometry = {}
                        if name == "right_racket_collision":
                            spatial = np.zeros(6)
                            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_GEOM, rgeom, spatial, 0)
                            velocity = spatial[3:] + np.cross(spatial[:3], ct.pos-data.geom_xpos[rgeom])
                            normal = data.site_xmat[racket].reshape(3,3)[:,1].copy()
                            relative = ct.pos - data.site_xpos[racket]
                            tangent = relative - normal * np.dot(relative, normal)
                            face_up = np.array([0., 0., 1.]) - normal * normal[2]
                            face_up /= max(np.linalg.norm(face_up), 1e-12)
                            contact_geometry = dict(
                                racket_center_world_m=data.site_xpos[racket].tolist(),
                                contact_point_world_m=ct.pos.tolist(),
                                racket_normal_world=normal.tolist(),
                                face_center_distance_m=float(np.linalg.norm(tangent)),
                                face_vertical_offset_m=float(np.dot(tangent, face_up)))
                            out = np.zeros(9)
                            ball.contact(pre[:3].copy(), velocity, normal, pre[3:].copy(), out)
                            data.qvel[bv:bv+6] = out[:6]
                            data.qpos[bq:bq+3] = ct.pos + .0201*out[6:]
                        elif name == "gate3_table_collision" and pre[2] < 0:
                            data.qvel[bv:bv+3] = pre[:3]*[.64, .64, -.9215]
                            data.qpos[bq+2] = max(data.qpos[bq+2], .7801)
                        contacts.append(dict(time=float(data.time-play_time), geom=name,
                            position=data.qpos[bq:bq+3].tolist(), velocity=data.qvel[bv:bv+3].tolist(),
                            **contact_geometry))
                    previous = set(touching)
            rootmat = data.xmat[pelvis].reshape(3,3)
            torsomat = data.xmat[torso].reshape(3,3)
            footpos = data.site_xpos[feet]
            relative_com = data.subtree_com[pelvis] - footpos.mean(axis=0)
            trace.append([float(data.time), phase, float(data.time-play_time) if play_time is not None else -100,
                sim._tilt_deg(data.qpos[3:7]), np.degrees(np.arctan2(rootmat[0,2],rootmat[2,2])),
                np.degrees(np.arctan2(torsomat[0,2],torsomat[2,2])), data.qpos[2],
                abs(footpos[0,1]-footpos[1,1]), relative_com[0], relative_com[1],
                *data.qpos[bq:bq+3], *data.qvel[bv:bv+3],
                np.max(np.linalg.norm(footpos[:,:2]-initial_feet[:,:2],axis=1)),
                np.max(abs(data.qpos[qa]-command[0]))])
            if output is not None:
                poses.append(data.qpos.copy())
                commands.append(command.copy())
            if sim._tilt_deg(data.qpos[3:7]) > 35 or data.qpos[2] < .65:
                fell = True
                break
            if play_time is not None and data.time > play_time + 7.7:
                break
    finally:
        lib.probe_destroy(probe)
    trace = np.array(trace)
    tables = [c for c in contacts if c['geom']=='gate3_table_collision']
    hits = [c for c in contacts if c['geom']=='right_racket_collision']
    clean_face_contact = bool(hits) and not any(
        c['time'] <= hits[0]['time'] and c['geom'] != 'right_racket_collision'
        for c in contacts)
    legal = bool(clean_face_contact and len(tables)>=2 and 0<tables[0]['position'][0]<1.37 and
        1.37<tables[1]['position'][0]<2.74 and all(-1.525<c['position'][1]<0 for c in tables[:2]) and
        not any(c['geom']=='gate3_net_collision' for c in contacts))
    stats = dict(csv=str(csv), sha256=hashlib.sha256(Path(csv).read_bytes()).hexdigest(),
        initial_snapshot=str(initial_snapshot) if initial_snapshot is not None else None,
        scope="native Serve controller + floating-base MuJoCo + physical contact; modeled palm/release; no hardware qualification",
        contact_profile=profile, delay_s=delay, payload_kg=payload, rate_hz=rate, offset_m=list(offset),
        command_schedule="external monotonic command times" if command_times is not None else "uniform",
        release_lead_frames=release_lead_frames,
        handoff_frame=handoff_frame, return_seconds=return_seconds,
        model_path=str(sim.DEFAULT_MODEL if model_path is None else model_path),
        release_site=release_site, release_normal_offset_m=release_normal_offset,
        clean_face_contact=clean_face_contact,
        central_contact_radius_m=.04,
        central_face_contact=bool(clean_face_contact and hits[0]['face_center_distance_m'] <= .04),
        offset_frame=offset_frame, release_geometry=release_geometry, stand_offset_rad=stand_offset,
        fell=fell, native_entry_jump_q_dq_tau_kp_kd=entry_jump, max_tilt_deg=float(trace[:,3].max()),
        min_root_z_m=float(trace[:,6].min()), prepare_max_pitch_deg=float(abs(trace[trace[:,2]<0,4]).max()),
        max_torso_pitch_deg=float(abs(trace[:,5]).max()), ready_foot_width_m=float(trace[np.argmin(abs(trace[:,2])),7]),
        prepare_pitch_range_deg=[float(trace[trace[:,2]<0,4].min()),float(trace[trace[:,2]<0,4].max())],
        initial_foot_width_m=float(trace[0,7]), max_com_forward_m=float(trace[:,8].max()),
        max_foot_displacement_m=float(trace[:,16].max()), racket_contact=bool(hits), legal_serve=legal,
        min_racket_center_distance_m=min_racket_distance, contacts=contacts,
        opponent_bounce=tables[1]['position'] if len(tables)>1 else None)
    if output:
        output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
        Path(str(output)+'.json').write_text(json.dumps(stats,indent=2)+'\n')
        np.savez_compressed(str(output)+'.npz',trace=trace,qpos=poses,commands=commands)
    return stats


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv',type=Path,required=True)
    p.add_argument('--library',type=Path,required=True)
    p.add_argument('--ball-library',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--delay',type=float,default=.04)
    p.add_argument('--payload',type=float,default=0.)
    p.add_argument('--stand-offset',type=float,default=0.)
    p.add_argument('--rate',type=float,default=100.)
    p.add_argument('--offset',type=float,nargs=3,default=(0.,0.,0.))
    p.add_argument('--offset-frame',choices=['world','palm'],default='world')
    p.add_argument('--profile',choices=['hope','vendor','xml'],default='hope')
    p.add_argument('--handoff-frame',type=int)
    p.add_argument('--return-seconds',type=float)
    args=p.parse_args()
    result=run(args.csv,args.library,args.ball_library,args.output,args.delay,args.payload,
        offset=args.offset,profile=args.profile,rate=args.rate,stand_offset=args.stand_offset,
        offset_frame=args.offset_frame,handoff_frame=args.handoff_frame,
        return_seconds=args.return_seconds)
    print(json.dumps(result,indent=2))
