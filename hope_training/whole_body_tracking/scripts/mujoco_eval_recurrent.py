"""Bounded, CPU GRU behavior inspection in a nominal MuJoCo plant.

Default mode shows target references. --physical-ball evaluates real MuJoCo
contacts with the same frozen actor; neither mode recreates all Isaac task managers.
Run --verify-only for ONNX sequence/reset and plant IO checks before a viewer.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'a3_deploy/a3_deploy_example/reference'))
from a3_deploy_onnx_ref_pingpong.recurrent_policy import RecurrentOnnxPolicy
from a3_deploy_onnx_ref_pingpong.observation import RobotState, ObsTarget, build_observation


def dump(path, data):
    Path(path).write_text(json.dumps(data,indent=2,default=str)+'\n')


class Plant:
    def __init__(self, manifest, physical=False, ball_slots=1, table_impact_model='soft'):
        import mujoco as mj
        self.mj = mj
        self.cfg = manifest
        if table_impact_model not in ('soft','ppq004'):
            raise ValueError('Unsupported evaluation table impact model')
        self.table_impact_model=table_impact_model
        path=ROOT/'a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/models/bin/cfg/model/a3_pingpong/a3_pingpong.xml'
        if physical:
            from recurrent_physical_ball import build_model
            self.model = build_model(mj,path,slots=ball_slots)
        else:
            self.model = mj.MjModel.from_xml_path(str(path))
        m=self.model
        m.opt.timestep=float(manifest['physics_dt'])
        m.opt.integrator=mj.mjtIntegrator.mjINT_IMPLICITFAST
        self.dt=float(manifest['control_dt'])
        self.substeps=int(manifest['decimation'])
        assert abs(m.opt.timestep*self.substeps-self.dt)<1e-9
        names=manifest['joint_names']
        self.jids=np.array([mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in names])
        if len(set(self.jids.tolist()))!=31 or min(self.jids)<0:
            raise ValueError('MJCF joint mapping is not bijective')
        self.qadr=m.jnt_qposadr[self.jids].copy()
        self.vadr=m.jnt_dofadr[self.jids].copy()
        self.aids=[]
        for jid in self.jids:
            matches=np.flatnonzero(m.actuator_trnid[:,0]==jid)
            if len(matches)!=1: raise ValueError('expected exactly one actuator per action joint')
            self.aids.append(int(matches[0]))
        self.aids=np.array(self.aids)
        act=manifest['actuator']; self.action=manifest['action']
        self.offset=np.array(self.action['offset']); self.scale=np.array(self.action['scale'])
        self.lower=np.array(self.action['lower']); self.upper=np.array(self.action['upper'])
        self.default=np.array(self.action['default_q']); self.default_qd=np.array(self.action['default_qd'])
        # Use saved nominal TOTAL damping exactly once, as an implicit drive;
        # do not add the old MJCF passive damping a second time.
        m.dof_damping[self.vadr]=0
        m.dof_armature[self.vadr]=act['armature']
        m.dof_frictionloss[self.vadr]=act['friction']
        for i,aid in enumerate(self.aids):
            m.actuator_gaintype[aid]=mj.mjtGain.mjGAIN_FIXED
            m.actuator_biastype[aid]=mj.mjtBias.mjBIAS_AFFINE
            m.actuator_gainprm[aid]=0; m.actuator_gainprm[aid,0]=act['kp'][i]
            m.actuator_biasprm[aid]=0
            m.actuator_biasprm[aid,1]=-act['kp'][i]
            m.actuator_biasprm[aid,2]=-act['kd_total'][i]
            m.actuator_ctrlrange[aid]=[self.lower[i],self.upper[i]]
            m.actuator_ctrllimited[aid]=1
            m.actuator_forcerange[aid]=[-act['effort_limit'][i],act['effort_limit'][i]]
            m.actuator_forcelimited[aid]=1
        # The vendor scene includes off-court Gate3 balls. Disable all ball
        # contacts/visibility; these are not part of this virtual-target test.
        for gid in range(m.ngeom):
            name=mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,gid) or ''
            if 'ball' in name:
                m.geom_contype[gid]=0; m.geom_conaffinity[gid]=0; m.geom_rgba[gid,3]=0
            elif m.geom_contype[gid]:
                m.geom_friction[gid,0]=1.0
        # StationaryRally uses a virtual table, not the vendor Gate3 physical
        # arena. Do not inherit that arena's x=0/y=-.7625 placement or contacts.
        table=manifest['visual_table']
        for gid in range(m.ngeom):
            name=mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,gid) or ''
            if name.startswith(('gate3_table','gate3_net')):
                m.geom_contype[gid]=0;m.geom_conaffinity[gid]=0
                m.geom_pos[gid,0]=table['near_x']+table['length']/2
                m.geom_pos[gid,1]=0
                if 'table' in name:
                    m.geom_pos[gid,2]=table['surface_z']-m.geom_size[gid,2]
                    m.geom_size[gid,:2]=[table['length']/2,table['width']/2]
                else:
                    m.geom_pos[gid,2]=table['surface_z']+m.geom_size[gid,2]
                    m.geom_size[gid,1]=table['width']/2
        self.data=mj.MjData(m)
        self.root_jid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'pelvis_free_joint')
        self.root_q=int(m.jnt_qposadr[self.root_jid]); self.root_v=int(m.jnt_dofadr[self.root_jid])
        self.gyro=mj.mj_name2id(m,mj.mjtObj.mjOBJ_SENSOR,'pelvis_imu_gyro')
        self.gyro_adr=int(m.sensor_adr[self.gyro])
        self.racket=mj.mj_name2id(m,mj.mjtObj.mjOBJ_SITE,'right_racket')
        self.feet=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,n) for n in ['left_ankle_roll_Link','right_ankle_roll_Link']]
        if min(self.feet+[self.racket,self.gyro])<0: raise ValueError('missing sensor/site/body')
        site=manifest['racket_site']
        if mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,int(m.site_bodyid[self.racket]))!=site['body']:
            raise ValueError('racket site has wrong parent link')
        np.testing.assert_allclose(m.site_pos[self.racket],site['offset'],rtol=0,atol=site['position_tolerance_m'])
        self.viewer=None
        self.ball_physics=None
        self.episode_time=0.
        self.reset(0)
        if physical:
            from recurrent_physical_ball import PhysicalBall
            # Smaller collision steps prevent tunnelling through the thin blade/net.
            # Actor control remains 50 Hz with the original action-delay FIFO.
            m.opt.timestep=.001
            self.substeps=20
            self.ball_physics=PhysicalBall(self,ROOT)
            mj.mj_setConst(m,self.data)
            self.reset(0)

    def reset(self, delay):
        m,d=self.model,self.data
        self.episode_time=0.
        if self.ball_physics is not None:self.ball_physics.park()
        if m.nkey: self.mj.mj_resetDataKeyframe(m,d,0)
        else: self.mj.mj_resetData(m,d)
        root=np.array(self.cfg['root_default'])
        d.qpos[self.root_q:self.root_q+7]=root[:7]
        d.qvel[:]=0
        d.qpos[self.qadr]=self.default
        d.qvel[self.vadr]=self.default_qd
        d.ctrl[self.aids]=self.default
        self.mj.mj_forward(m,d)
        self.anchor=d.qpos[self.root_q:self.root_q+2].copy()
        self.delay=delay
        self.queue=None
        self.previous_action=np.zeros(31,dtype=np.float32)
        self.previous_qdes=self.default.copy()

    def state(self):
        d=self.data
        return RobotState(d.qpos[self.root_q:self.root_q+3].copy(),
            d.qpos[self.root_q+3:self.root_q+7].copy(),d.sensordata[self.gyro_adr:self.gyro_adr+3].copy(),
            d.qpos[self.qadr].copy(),d.qvel[self.vadr].copy()-self.default_qd)

    def observe(self,target):
        return build_observation(self.state(),target,self.previous_action,self.default,self.anchor)

    def decode(self,action):
        raw=np.asarray(action,dtype=np.float64).copy()
        raw[self.action['passive_columns']]=0
        commanded=np.clip(self.offset+self.scale*raw,self.lower,self.upper)
        # Seed the FIFO from the first decoded command, matching hope_actions.
        if self.queue is None: self.queue=[commanded.copy() for _ in range(self.action['delay_max']+1)]
        self.queue=self.queue[1:]+[commanded.copy()]
        executed=self.queue[-1-self.delay].copy()
        self.previous_action=raw.astype(np.float32)
        return commanded,executed

    def step(self,action):
        commanded,executed=self.decode(action)
        self.data.ctrl[self.aids]=executed
        for i in range(self.substeps):
            t=self.episode_time+i*self.model.opt.timestep
            if self.ball_physics is not None:self.ball_physics.before(t)
            self.mj.mj_step(self.model,self.data)
            if self.ball_physics is not None:self.ball_physics.after(t+self.model.opt.timestep)
        self.episode_time+=self.dt
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError('non-finite MuJoCo plant state')
        delta=executed-self.previous_qdes
        self.previous_qdes=executed
        return delta

    def metrics(self):
        d,m=self.data,self.model
        state=self.state(); quat=state.base_quat_w
        tilt=np.degrees(np.arccos(np.clip(1-2*(quat[1]**2+quat[2]**2),-1,1)))
        contacts=set()
        for c in d.contact:
            for gid in (c.geom1,c.geom2):
                if m.geom_bodyid[gid] in self.feet: contacts.add(int(m.geom_bodyid[gid]))
        slip=[]
        for body in self.feet:
            vel=np.zeros(6);self.mj.mj_objectVelocity(m,d,self.mj.mjtObj.mjOBJ_BODY,body,vel,0)
            if body in contacts: slip.append(float(np.linalg.norm(vel[3:5])))
        speed=np.linalg.norm(d.qvel[self.root_v:self.root_v+2])
        return {'base_z':float(state.base_pos_w[2]),'tilt_deg':float(tilt),
            'drift_m':float(np.linalg.norm(state.base_pos_w[:2]-self.anchor)),
            'base_speed':float(speed),'contact_foot_slip':max(slip,default=0.),
            'both_feet_contact':len(contacts)==2,'racket_pos':d.site_xpos[self.racket].copy()}

    def open_viewer(self):
        from mujoco import viewer
        before=set(threading.enumerate())
        self.viewer=viewer.launch_passive(self.model,self.data)
        self.viewer_threads=[t for t in threading.enumerate() if t not in before]
        self.viewer.cam.lookat[:]=[1.35,0,.85] if self.ball_physics is not None else [0,0,1]
        self.viewer.cam.distance=6.2 if self.ball_physics is not None else 3.1
        self.viewer.cam.azimuth=115;self.viewer.cam.elevation=-24

    def draw(self,target,side):
        if self.viewer is None:return
        if self.ball_physics is not None:return self.ball_physics.draw(target,side)
        mj=self.mj
        color=[0,1,.1,1] if side==0 else [.1,.65,1,1]
        with self.viewer.lock():
            scene=self.viewer.user_scn; scene.ngeom=0
            for position,rgba in [(target.pos_w,color),
                                  (self.data.site_xpos[self.racket],[1,.2,.1,1])]:
                radius=.04 if scene.ngeom==0 else .025
                mj.mjv_initGeom(scene.geoms[scene.ngeom],mj.mjtGeom.mjGEOM_SPHERE,
                    [radius]*3,position,np.eye(3).ravel(),np.array(rgba,dtype=np.float32))
                if scene.ngeom==0:
                    scene.geoms[scene.ngeom].label='REFERENCE STRIKE TARGET'
                scene.ngeom+=1
            # A conspicuous crosshair denotes a fixed target, not a moving ball.
            # Neither target velocity nor the strike clock defines a ball flight.
            for axis in np.eye(3)*.09:
                mj.mjv_initGeom(scene.geoms[scene.ngeom],mj.mjtGeom.mjGEOM_LINE,
                    np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([1,1,.1,1],dtype=np.float32))
                mj.mjv_connector(scene.geoms[scene.ngeom],mj.mjtGeom.mjGEOM_LINE,3,
                    target.pos_w-axis,target.pos_w+axis)
                scene.ngeom+=1
            mj.mjv_initGeom(scene.geoms[scene.ngeom],mj.mjtGeom.mjGEOM_ARROW,
                np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
            mj.mjv_connector(scene.geoms[scene.ngeom],mj.mjtGeom.mjGEOM_ARROW,.012,
                target.pos_w,target.pos_w+target.vel_w*.15)
            scene.ngeom+=1
        self.viewer.set_texts((mj.mjtFontScale.mjFONTSCALE_150,mj.mjtGridPos.mjGRID_TOPRIGHT,
            'REFERENCE STRIKE TARGET ONLY\nNo ball flight/contact/bounce is simulated\n'
            +('FOREHAND (green)' if side==0 else 'BACKHAND (blue)')
            +f' | strike clock: {target.time_to_strike:+.2f} s\n'
            +'Yellow crosshair = target; red = racket; arrow = desired racket velocity',''))
        self.viewer.sync()

    def close_viewer(self):
        if self.viewer is not None:
            self.viewer.close()
            # Handle.close requests shutdown asynchronously. Keep model/data
            # alive until this viewer's Python render thread releases them.
            for thread in self.viewer_threads:
                thread.join(timeout=5)
                if thread.is_alive():raise RuntimeError('viewer render thread did not exit')


def verify(bundle,out,physical=False):
    policy=RecurrentOnnxPolicy(bundle)
    fixture=np.load(Path(bundle)/'parity_vectors.npz')
    policy.hidden=fixture['initial_h'].copy()
    action_error=hidden_error=0.
    for i,obs in enumerate(fixture['obs']):
        if i==int(fixture['reset_at']):policy.reset_episode()
        action=policy.infer(obs)
        action_error=max(action_error,float(np.max(np.abs(action-fixture['actions'][i,0]))))
        hidden_error=max(hidden_error,float(np.max(np.abs(policy.hidden-fixture['hidden'][i]))))
        # Float32 recurrent ONNX vs PyTorch; allow accumulated fused-kernel
        # ordering error, not a behavioral difference or a different policy.
        np.testing.assert_allclose(action,fixture['actions'][i,0],rtol=2e-4,atol=2e-5)
        np.testing.assert_allclose(policy.hidden,fixture['hidden'][i],rtol=2e-4,atol=2e-5)
    assert policy.reset_count==1 and policy.steps==64
    plant=Plant(policy.manifest,physical=physical)
    for gid in range(plant.model.ngeom):
        name=plant.mj.mj_id2name(plant.model,plant.mj.mjtObj.mjOBJ_GEOM,gid) or ''
        if name.startswith(('gate3_table','gate3_net')):
            assert not plant.model.geom_contype[gid] and not plant.model.geom_conaffinity[gid]
            assert abs(plant.model.geom_pos[gid,0]-(policy.manifest['visual_table']['near_x']+policy.manifest['visual_table']['length']/2))<1e-12
    target=ObsTarget(np.array([.58,-.44,1.1]),np.array([2.,.2,1.]),.5)
    # Distinct values in every joint column catch SDK-vs-articulation permutations.
    plant.data.qpos[plant.qadr]=plant.default+np.arange(31)*.001
    plant.data.qvel[plant.vadr]=np.arange(31)*.002
    plant.mj.mj_forward(plant.model,plant.data)
    observation=plant.observe(target)
    np.testing.assert_allclose(observation[3:34],np.arange(31)*.001,atol=1e-7)
    np.testing.assert_allclose(observation[34:65],np.arange(31)*.002,atol=1e-7)
    np.testing.assert_allclose(observation[103:106],target.pos_w-plant.state().base_pos_w,atol=1e-7)
    # Actual actuator force must match the named-joint affine implicit drive.
    plant.reset(0)
    raw=np.linspace(-.15,.15,31)
    raw[policy.manifest['action']['passive_columns']]=0
    commanded,executed=plant.decode(raw)
    expected=np.clip(plant.offset+plant.scale*raw,plant.lower,plant.upper)
    np.testing.assert_allclose(commanded,expected,atol=1e-12)
    plant.data.ctrl[plant.aids]=executed
    plant.mj.mj_forward(plant.model,plant.data)
    torque=np.array(policy.manifest['actuator']['kp'])*(expected-plant.data.qpos[plant.qadr])
    torque-=np.array(policy.manifest['actuator']['kd_total'])*plant.data.qvel[plant.vadr]
    limit=np.array(policy.manifest['actuator']['effort_limit'])
    np.testing.assert_allclose(plant.data.actuator_force[plant.aids],np.clip(torque,-limit,limit),atol=1e-8)
    plant.reset(1)
    _,first=plant.decode(raw); _,second=plant.decode(-raw)
    np.testing.assert_array_equal(first,second)
    # Construct full 110-D expectation independently for an identity orientation.
    plant.reset(0);plant.data.qpos[plant.root_q+3:plant.root_q+7]=[1,0,0,0]
    plant.mj.mj_forward(plant.model,plant.data)
    state=plant.state();obs=plant.observe(target)
    expected_obs=np.concatenate([state.base_ang_vel_b,state.q-plant.default,state.qd,
        np.zeros(31),[0,0,-1],[1,0],plant.anchor-state.base_pos_w[:2],
        target.pos_w-state.base_pos_w,target.vel_w,[target.time_to_strike]]).astype(np.float32)
    np.testing.assert_allclose(obs,expected_obs,atol=1e-7)
    result={'result':'PASS','steps':64,'reset_at':int(fixture['reset_at']),
        'initial_hidden_nonzero':bool(np.any(fixture['initial_h'])),
        'action_max_abs_error':action_error,'hidden_max_abs_error':hidden_error,
        'tolerances':{'atol':2e-5,'rtol':2e-4,'reason':'float32 fused GRU/MLP ordering and recurrent accumulation'},
        'finite':True,'observation_columns_and_full_layout':'PASS','joint_actuator_force_mapping':'PASS',
        'action_decode_and_delay':'PASS','control_dt':plant.dt,'physics_dt':plant.model.opt.timestep,
        'saved_virtual_table_geometry_and_no_robot_table_contacts':'PASS',
        'racket_site_matches_saved_wrist_mount_within_micrometers':'PASS',
        'onnx_providers':policy.session.get_providers(),'training_hidden_used':False}
    if physical:
        pb=plant.ball_physics
        result['physical_geometry']={'ball_mass_kg':pb.mass,'ball_radius_m':pb.radius,
            'pairs':pb.pair_settings,'racket_geom':'right_racket_collision',
            'contact_authority':'MuJoCo solver only; no strike-time impulse or ball pose snap',
            'physics_dt':plant.model.opt.timestep,'substeps':plant.substeps}
    dump(out/'verification.json',result)
    print(json.dumps(result,indent=2),flush=True)


def run(args):
    policy=RecurrentOnnxPolicy(args.bundle); manifest=policy.manifest
    task=manifest['task'];cfg=task['racket']; dt=manifest['control_dt']
    bank_path=ROOT/'hope_training/whole_body_tracking'/cfg['venue_tuple_bank_path']
    if hashlib.sha256(bank_path.read_bytes()).hexdigest()!=cfg['venue_tuple_bank_sha256']:
        raise ValueError('PPQ004 bank SHA mismatch')
    bank=np.load(bank_path);rng=np.random.default_rng(args.seed)
    plant=Plant(manifest,physical=args.physical_ball)
    if args.view:plant.open_viewer()
    recorder=None
    if args.video is not None:
        from recurrent_physical_ball import VideoRecorder
        recorder=VideoRecorder(plant,args.video)
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    rows=[];episodes=[];shots=[];episode=-1;episode_t=0.;ball=0
    elapsed=0.;fall_count=0;previous_delta=np.zeros(31);last_racket=None
    side=0;row_id=None;strike_at=0.;next_ball_at=0.
    stage='HOLD'
    def select_target(current_side):
        choices=np.flatnonzero(bank['clip']==current_side)
        return int(rng.choice(choices))
    def reset_episode(reason):
        nonlocal episode,episode_t,ball,row_id,side,strike_at,next_ball_at,last_racket,previous_delta
        if plant.ball_physics is not None:plant.ball_physics.finish(reason)
        if episode>=0:
            episodes.append({'episode':episode,'duration_s':episode_t,'reset_reason':reason,
                'side_at_end':'forehand' if side==0 else 'backhand'})
        episode+=1;episode_t=0.;ball=0
        delay=0 if episode%2==0 else int(manifest['action']['delay_max'])
        plant.reset(delay);policy.reset_episode();side=episode%2
        row_id=select_target(side)
        # Explicit evaluation reset scenario. Training mixes motion, post-swing
        # replay and long stand starts; no single deterministic start reproduces
        # that distribution. Default to a PRE start, with optional HOLD inspection.
        initial_hold=args.initial_hold_seconds
        strike_at=initial_hold+float(np.mean(task['motion']['rally_pre_strike_s_range']))
        next_ball_at=strike_at+float(np.mean(task['motion']['rally_post_strike_s_range']))+1.0
        last_racket=None;previous_delta=np.zeros(31)
        if plant.ball_physics is not None:plant.ball_physics.schedule(bank,row_id,strike_at,episode,side)
        print(f'EPISODE {episode} reset={reason} hidden=ZERO side={side} delay={delay}',flush=True)
    reset_episode('initial')
    begin=time.monotonic()
    error=None
    stopped='simulation_duration'
    try:
        for tick in range(int(round(args.seconds/dt))):
            if args.wall_seconds is not None and time.monotonic()-begin>=args.wall_seconds:
                stopped='wall_duration';break
            if plant.viewer is not None and not plant.viewer.is_running():
                stopped='viewer_closed';break
            if episode_t>=next_ball_at:
                ball+=1;side=1-side;row_id=select_target(side)
                strike_at=episode_t+float(np.mean(task['motion']['rally_pre_strike_s_range']))
                next_ball_at=strike_at+float(np.mean(task['motion']['rally_post_strike_s_range']))+1.0
                if plant.ball_physics is not None:plant.ball_physics.schedule(bank,row_id,strike_at,episode,side)
                print(f'TARGET episode={episode} ball={ball} side={side} hidden=CARRIED',flush=True)
            pre=float(np.mean(task['motion']['rally_pre_strike_s_range']))
            tts=min(pre,strike_at-episode_t)
            stage='HOLD' if episode_t<strike_at-pre else ('PRE' if tts>0 else ('STRIKE' if tts>-.12 else 'RECOVER'))
            target=ObsTarget(np.array(bank['target_position_offset'][row_id],dtype=float),
                np.array(bank['target_velocity'][row_id],dtype=float),tts)
            obs=plant.observe(target);action=policy.infer(obs);delta=plant.step(action)
            metric=plant.metrics();racket=metric.pop('racket_pos')
            velocity=np.zeros(3) if last_racket is None else (racket-last_racket)/dt
            last_racket=racket.copy()
            row={'t':tick*dt,'episode':episode,'episode_t':episode_t,'side':side,'bank_row':row_id,
                'stage':stage,'tts':tts,**metric,'target_error_m':float(np.linalg.norm(racket-target.pos_w)),
                'racket_speed':float(np.linalg.norm(velocity)),
                'velocity_error_mps':float(np.linalg.norm(velocity-target.vel_w)),
                'qdes_step_rms':float(np.sqrt(np.mean(delta**2))),
                'qdes_reversal_fraction':float(np.mean((delta*previous_delta<0)&(np.abs(delta)>.001))),
                'hidden_norm':float(np.linalg.norm(policy.hidden)),
                'ready_position_speed_proxy':bool(metric['drift_m']<=.1 and metric['base_speed']<=.2)}
            if plant.ball_physics is not None:
                bp,bv=plant.ball_physics.state()
                row.update({f'ball_{key}':float(value) for key,value in zip(['x','y','z','vx','vy','vz'],np.r_[bp,bv])})
                row['physical_ball_active']=plant.ball_physics.active
            previous_delta=delta; rows.append(row)
            if 0<=tts<dt:
                shots.append({**row,'physical_contact_verified':False})
                print('REFERENCE_STRIKE',json.dumps(row),flush=True)
            plant.draw(target,side)
            if recorder is not None:recorder.capture(target,side,tick*dt)
            episode_t+=dt;elapsed=(tick+1)*dt
            if args.realtime:
                time.sleep(max(0,begin+elapsed-time.monotonic()))
            if metric['base_z']<.8 or metric['tilt_deg']>60:
                fall_count+=1;reset_episode('fall')
            elif episode_t>=float(task['env']['episode_length_s']):reset_episode('time_out')
    except KeyboardInterrupt:
        stopped='interrupted'
    except Exception as exc:
        error=repr(exc)
        raise
    finally:
        plant.close_viewer()
        if recorder is not None:recorder.close()
        if plant.ball_physics is not None:
            plant.ball_physics.finish('evaluation_end')
            dump(out/'ball_events.json',plant.ball_physics.events)
            dump(out/'physical_shots.json',plant.ball_physics.shots)
        if rows:
            with (out/'trace.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        stages={}
        for name in ['HOLD','PRE','STRIKE','RECOVER']:
            selected=[r for r in rows if r['stage']==name]
            if selected:
                stages[name]={k:float(np.max([r[k] for r in selected])) for k in ['tilt_deg','drift_m','contact_foot_slip','qdes_step_rms','racket_speed']}
                stages[name]['ready_proxy_fraction']=float(np.mean([r['ready_position_speed_proxy'] for r in selected]))
        result={'checkpoint':manifest['checkpoint'],'checkpoint_sha256':manifest['checkpoint_sha256'],
            'checkpoint_iteration':manifest['checkpoint_iteration'],'simulated_seconds':elapsed,
            'wall_seconds':time.monotonic()-begin,'control_steps':len(rows),'view_requested':args.view,'video':str(args.video) if args.video else None,
            'video_frames':recorder.frames if recorder is not None else 0,
            'stop_reason':stopped,
            'scene':'MuJoCo physical ball, actual mesh contacts' if args.physical_ball else 'nominal MuJoCo robot + target/reference markers; no physical ball',
            'physical_ball_contact_verified':False,'falls':fall_count,'episode_resets':policy.reset_count,
            'episodes':episodes,'strike_reference_samples':shots,'stage_summaries':stages,
            'sides_seen':sorted({r['side'] for r in rows}),'numerical_error':error,
            'mujoco_warnings':plant.data.warning.number.tolist(),
            'limitations':([x for x in manifest['limitations'] if 'Only target/reference markers' not in x]+[
                'Physical mode uses 1 ms MuJoCo collision steps; actor control remains 20 ms.',
                'MuJoCo soft Coulomb contact approximates fitted restitution; fitted tangential spin impulse is not reproduced.',
                'Incoming serve is only the final post-bounce flight segment, truncated at the table plane.',
                'Ball collides only with racket face, table, net and floor; other robot geoms are excluded.'
            ]) if args.physical_ball else manifest['limitations'],'command':sys.argv,
            'seed':args.seed,'hidden_reset_rule':'ZERO only on episode reset; carried across targets',
            'initial_hold_seconds':args.initial_hold_seconds,
            'action_delay_rule':'alternating nominal 0-tick and maximum 1-tick episode cohorts',
            'onnx_providers':policy.session.get_providers()}
        if plant.ball_physics is not None:
            pb=plant.ball_physics
            result['physical_ball_contact_verified']=any(s['racket_contacts'] for s in pb.shots)
            result['physical_summary']={k:sum(s[k] for s in pb.shots) for k in ['racket_contacts','table_contacts','net_contacts','floor_contacts','legal_return']}
            result['physical_summary']['misses']=sum(s['outcome']=='miss' and s['complete'] for s in pb.shots)
            result['physical_summary']['shots']=len(pb.shots)
            result['physical_summary']['incomplete_shots']=sum(not s['complete'] for s in pb.shots)
            result['physical_summary']['max_ball_speed_mps']=pb.max_speed
            dump(out/'physical_config.json',{'profile':pb.profile,'physics':pb.cfg,'pair_settings':pb.pair_settings,
                'manifest':manifest,'seed':args.seed,'command':sys.argv,'physics_dt':plant.model.opt.timestep,
                'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('recurrent_physical_ball.py'),ROOT/'a3_deploy/a3_deploy_example/reference/a3_deploy_onnx_ref_pingpong/recurrent_policy.py',ROOT/'a3_deploy/a3_deploy_example/reference/a3_deploy_onnx_ref_pingpong/observation.py']},
                'control_dt':plant.dt,'serve_horizon':.6,'serve_plane_margin':.005,
                'serve_integration_dt':.001,'reverse_speed_cap':40,'ball_speed_fail_bound':50,
                'ball_abs_position_fail_bound':30,'reference_is_collision_free':True})
        dump(out/'results.json',result)
        print('RESULTS',json.dumps({k:v for k,v in result.items() if k not in ['strike_reference_samples','episodes','limitations']}),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--verify-only',action='store_true')
    p.add_argument('--video',type=Path,help='Offscreen MP4 using installed ffmpeg; requires --physical-ball and MUJOCO_GL=egl.')
    p.add_argument('--physical-ball',action='store_true',help='Real ball contacts; preserve the frozen actor and target observation contract.')
    p.add_argument('--view',action='store_true')
    p.add_argument('--realtime',action='store_true')
    p.add_argument('--seconds',type=float,default=90)
    p.add_argument('--wall-seconds',type=float,help='Optional wall-clock bound, including viewer sync, up to 120 seconds.')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--initial-hold-seconds',type=float,default=0,
                   help='Explicit evaluation start scenario; 4 reproduces the long-HOLD start duration, 0 starts PRE.')
    args=p.parse_args()
    if args.video is not None and not args.physical_ball:p.error('--video requires --physical-ball')
    if not 0<args.seconds<=120: p.error('--seconds must be in (0,120]')
    if not 0<=args.initial_hold_seconds<=10:p.error('initial HOLD must be in [0,10]')
    if args.wall_seconds is not None and not 0<args.wall_seconds<=120:p.error('wall duration must be in (0,120]')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.verify_only:verify(args.bundle,args.output_dir,physical=args.physical_ball)
    else:run(args)


if __name__=='__main__':main()
