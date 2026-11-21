"""Physical-ball extension; no actor inputs, action overrides or strike-clock impulses.

Serve integration is a NumPy port of physical_ball.back_integrate_incoming and
virtual_ball.rk4_step. Contacts use the existing MuJoCo racket mesh and pair
restitution approximation from mujoco_pingpong_scene, never the Isaac snap path.
The explicit ppq004 evaluation mode replaces ONLY a real table-contact response
with the saved capped-slip law. It never changes qpos or racket contacts.
"""
from collections import deque
import hashlib
import numpy as np
import yaml
from mujoco_pingpong_scene import _dampratio_from_restitution


def build_model(mj, path, slots=1):
    spec = mj.MjSpec.from_file(str(path))
    for slot in range(slots):
        suffix='' if slot==0 else f'_return{slot}'
        spec.add_pair(geomname1='gate3_ball_collision'+suffix, geomname2='floor')
    return spec.compile()


def flight_step(p, v, w, h, cfg):
    f = cfg['flight']
    def acc(x):
        return np.array([0., 0., -f['g']])-f['k_d']*np.linalg.norm(x)*x+f['k_m']*np.cross(w,x)
    a1=acc(v); v2=v+h*a1/2; a2=acc(v2)
    v3=v+h*a2/2; a3=acc(v3); v4=v+h*a3; a4=acc(v4)
    return p+h*(v+2*v2+2*v3+v4)/6, v+h*(a1+2*a2+2*a3+a4)/6


def table_impact(v,w,cfg):
    """Documented PPQ004 table_contact law, world SI units, shell inertia.

    Same equations as hope_planner_cpp/ball_dynamics.hpp. This is not a fitted
    correction to a strike target: the archived profile supplies every value.
    """
    v=np.asarray(v,dtype=float).copy();w=np.asarray(w,dtype=float).copy()
    radius=cfg['ball']['radius'];inertia=cfg['ball']['inertia_coeff']
    c=cfg['contact']['table']
    if v[2]>=0:raise ValueError('Table impact requires an approaching ball')
    slip=np.array([v[0]-radius*w[1],v[1]+radius*w[0],0.])
    speed=np.linalg.norm(slip)
    impulse=min(max(0.,c['a_t'])*speed,
                max(0.,c['mu_safety'])*(1+c['e_eff'])*abs(v[2]))
    dv=-impulse/speed*slip if speed>1e-12 else np.zeros(3)
    v+=dv;v[2]=-c['e_eff']*v[2]
    w+=np.array([dv[1],-dv[0],0.])/(inertia*radius)
    return v,w


def incoming_serve(p, v, w, horizon, cfg, surface_z):
    p=np.array(p,dtype=float);v=np.array(v,dtype=float);w=np.array(w,dtype=float)
    effective=0.; path=[p.copy()]
    for _ in range(int(round(horizon/.001))):
        pp,vv=flight_step(p,v,w,-.001,cfg)
        if pp[2]<=surface_z+cfg['ball']['radius']+.005 or np.linalg.norm(vv)>40: break
        p,v=pp,vv;effective+=.001;path.append(p.copy())
    if effective<.01:raise ValueError('bank row has no safe incoming flight segment')
    return p,v,effective,np.array(path[::-1])


class PhysicalBall:
    def __init__(self, plant, root, slot=0):
        self.slot=slot
        self.plant=plant;self.mj=plant.mj;self.m=plant.model;self.d=plant.data
        profile=plant.cfg['task']['ball_physics'];path=root/profile['path']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=profile['sha256']:raise ValueError('ball physics SHA mismatch')
        self.cfg=yaml.safe_load(path.read_text());self.profile=profile
        self.fallback=yaml.safe_load((root/'configs/ball_physics.yaml').read_text())
        m=self.m;m.opt.gravity[:]=[0,0,-self.cfg['flight']['g']]
        self.table=plant.cfg['visual_table'];self.radius=self.cfg['ball']['radius'];self.mass=self.cfg['ball']['mass']
        self.table_impact_model=getattr(plant,'table_impact_model','soft')
        self.table_suppressed=False;self.table_pair=None
        def gid(name):return self.mj.mj_name2id(m,self.mj.mjtObj.mjOBJ_GEOM,name)
        suffix='' if slot==0 else f'_return{slot}'
        self.gid=gid('gate3_ball_collision'+suffix);self.racket=gid('right_racket_collision')
        self.table_gid=gid('gate3_table_collision');self.net_gid=gid('gate3_net_collision');self.floor=gid('floor')
        self.bid=int(m.geom_bodyid[self.gid]);jid=self.mj.mj_name2id(m,self.mj.mjtObj.mjOBJ_JOINT,'gate3_ball_free_joint'+suffix)
        self.q=int(m.jnt_qposadr[jid]);self.v=int(m.jnt_dofadr[jid])
        m.body_mass[self.bid]=self.mass
        m.body_inertia[self.bid]=self.cfg['ball']['inertia_coeff']*self.mass*self.radius**2
        m.geom_size[self.gid,0]=self.radius;m.geom_rgba[self.gid]=[1,.45,.02,1]
        # Only explicit pairs can touch this ball. Table/net cannot push robot.
        m.geom_contype[self.gid]=0;m.geom_conaffinity[self.gid]=0
        m.geom_rgba[self.net_gid]=[.95,.95,.95,.7]
        self.pair_settings={}
        self.racket_pair=None
        for i,(a,b) in enumerate(zip(m.pair_geom1,m.pair_geom2)):
            if self.gid not in (a,b):continue
            other=int(b if a==self.gid else a)
            name={self.table_gid:'table',self.net_gid:'net',self.racket:'paddle',self.floor:'floor'}.get(other)
            if name is None:
                # Disable vendor palm support: no unrequested serve-hand contact.
                m.pair_margin[i]=-100
                continue
            c=self.cfg['contact'].get(name,self.fallback['contact'][name])
            e=c.get('e_eff',c.get('restitution'));mu=c.get('mu_safety',c.get('dynamic_friction'))
            m.pair_solref[i]=[-100000.,-2*_dampratio_from_restitution(e)*np.sqrt(100000.)]
            m.pair_friction[i]=[mu,mu,.005,.0001,.0001];m.pair_dim[i]=3
            self.pair_settings[name]={'restitution':e,'friction':mu,'solref':m.pair_solref[i].tolist()}
            if name=='paddle':self.racket_pair=i
            if name=='table':self.table_pair=i
        assert set(self.pair_settings)=={'table','net','paddle','floor'}
        self.table_pair_margin=float(m.pair_margin[self.table_pair])
        if self.table_impact_model=='ppq004':
            if abs(self.cfg['ball']['inertia_coeff']-2/3)>1e-12:
                raise ValueError('Planner table law requires shell inertia 2/3')
            self.pair_settings['table']['impact_model']='ppq004_capped_slip_actual_contact_v1'
            self.pair_settings['table']['a_t']=self.cfg['contact']['table']['a_t']
        self.history=deque(maxlen=2000);self.events=[];self.shots=[];self.current=None
        self.active=False;self.pending=None;self.reference=np.empty((0,3));self.contacts=set()
        self.max_speed=0.;self.max_abs_position=0.
        self.park()

    def state(self):return self.d.qpos[self.q:self.q+3].copy(),self.d.qvel[self.v:self.v+3].copy()

    def park(self):
        if self.table_suppressed:
            self.m.pair_margin[self.table_pair]=self.table_pair_margin
            self.table_suppressed=False
        self.active=False;self.pending=None
        self.d.qpos[self.q:self.q+7]=[100+self.slot,0,10,1,0,0,0];self.d.qvel[self.v:self.v+6]=0
        self.d.xfrc_applied[self.bid]=0
        self.mj.mj_forward(self.m,self.d)

    def finish(self,reason):
        if self.current is not None:
            s=self.current;s['end_reason']=reason
            s['outcome']='legal_return' if s['legal_return'] else ('miss' if not s['racket_contacts'] else 'hit_without_legal_return')
            s['complete']=bool(s['floor_contacts'] or s['first_landing'] is not None or (reason=='next_serve' and not s['racket_contacts']))
            self.shots.append(s);self.current=None
        self.park()

    def schedule(self,bank,row,strike_at,episode,side):
        self.finish('next_serve')
        p=np.array(bank['target_position_offset'][row],dtype=float)
        v=np.array(bank['incoming_velocity'][row],dtype=float);w=np.array(bank['incoming_spin'][row],dtype=float)
        launch,vel,effective,reference=incoming_serve(p,v,w,.6,self.cfg,self.table['surface_z'])
        self.reference=reference;self.history.clear();self.contacts=set()
        self.pending=(strike_at-effective,launch,vel,w)
        self.current={'episode':episode,'side':side,'bank_row':row,'strike_at':strike_at,
            'launch_at':strike_at-effective,'effective_incoming_seconds':effective,
            'requested_incoming_seconds':.6,'target_position':p.tolist(),'incoming_velocity':v.tolist(),
            'incoming_spin':w.tolist(),'launch_position':launch.tolist(),'launch_velocity':vel.tolist(),
            'intended_landing_xy':bank['intended_landing_xy'][row].tolist(),
            'racket_contacts':0,'net_contacts':0,'floor_contacts':0,'table_contacts':0,
            'net_clear':False,'first_landing':None,'legal_return':False,'arrival_error_m':None}
        self.strike_at=strike_at;self.target_position=p;self.arrival_done=False

    def before(self,t):
        if self.pending is not None and t+1e-9>=self.pending[0]:
            launch_at,p,v,w=self.pending
            # Launch only at serve start; integrate fractional launch tick forward.
            if t>launch_at:p,v=flight_step(p,v,w,t-launch_at,self.cfg)
            self.d.qpos[self.q:self.q+7]=[*p,1,0,0,0];self.d.qvel[self.v:self.v+3]=v
            self.d.qvel[self.v+3:self.v+6]=w
            self.pending=None;self.active=True
            self.mj.mj_forward(self.m,self.d)
            self.event('launch',t,p)
        self.before_p,self.before_v=self.state()
        if not self.active:
            self.d.qvel[self.v:self.v+6]=0
            self.d.qpos[self.q:self.q+3]=[100+self.slot,0,10]
            return
        p,v=self.before_p,self.before_v
        if self.table_suppressed and p[2]>self.table['surface_z']+self.radius+1e-5:
            self.m.pair_margin[self.table_pair]=self.table_pair_margin
            self.table_suppressed=False
        if self.table_suppressed and t-self.table_suppressed_at>.05:
            raise FloatingPointError('PPQ004 impact did not separate from table within 50ms')
        # Freejoint angular velocity is local; Magnus requires world spin.
        w=self.d.xmat[self.bid].reshape(3,3)@self.d.qvel[self.v+3:self.v+6]
        self.before_w=w.copy()
        self.d.xfrc_applied[self.bid,:3]=self.mass*(-self.cfg['flight']['k_d']*np.linalg.norm(v)*v+self.cfg['flight']['k_m']*np.cross(w,v))
        # Venue normal restitution evaluated at actual relative contact speed.
        normal=self.d.site_xmat[self.plant.racket].reshape(3,3)[:,1]
        rv=np.zeros(6);self.mj.mj_objectVelocity(self.m,self.d,self.mj.mjtObj.mjOBJ_SITE,self.plant.racket,rv,0)
        un=abs(np.dot(v-rv[3:],normal));c=self.cfg['contact']['paddle']
        e=np.clip(c['e_exp_g1']*np.exp(c['e_exp_g2']*un),.05,.95)
        self.m.pair_solref[self.racket_pair,1]=-2*_dampratio_from_restitution(e)*np.sqrt(100000.)

    def event(self,kind,t,p,**extra):
        self.events.append({'kind':kind,'episode':self.current['episode'] if self.current else None,
            'bank_row':self.current['bank_row'] if self.current else None,'episode_t':float(t),'position':np.asarray(p).tolist(),**extra})

    def after(self,t):
        self.d.xfrc_applied[self.bid]=0
        if not self.active:return
        p,v=self.state();s=self.current
        speed=np.linalg.norm(v);self.max_speed=max(self.max_speed,float(speed));self.max_abs_position=max(self.max_abs_position,float(np.max(np.abs(p))))
        if not np.isfinite(p).all() or not np.isfinite(v).all() or speed>50 or np.max(np.abs(p))>30:
            raise FloatingPointError('physical ball exceeded finite position/speed bounds')
        if not self.arrival_done and t>=self.strike_at:
            self.arrival_done=True
            if s['racket_contacts']==0:s['arrival_error_m']=float(np.linalg.norm(p-self.target_position))
        new=set()
        for ci,c in enumerate(self.d.contact):
            if self.gid not in (c.geom1,c.geom2):continue
            other=int(c.geom2 if c.geom1==self.gid else c.geom1)
            force=np.zeros(6);self.mj.mj_contactForce(self.m,self.d,ci,force)
            if force[0]<=1e-8:continue
            new.add(other)
            if other in self.contacts:continue
            kind={self.racket:'racket',self.table_gid:'table',self.net_gid:'net',self.floor:'floor'}.get(other,'other')
            key=kind+'_contacts'
            if key in s:s[key]+=1
            extra={}
            if (other==self.table_gid and self.table_impact_model=='ppq004'
                    and not self.table_suppressed and self.before_v[2]<0
                    and abs(c.frame[2])>.999):
                # Replace, rather than add to, the engine's first-step impulse.
                # This branch requires an actual finite-geometry contact with
                # positive normal force. It never runs on a strike-clock event.
                qpos_before=self.d.qpos[self.q:self.q+7].copy()
                engine_velocity=v.copy()
                v,w=table_impact(self.before_v,self.before_w,self.cfg)
                self.d.qvel[self.v:self.v+3]=v
                rotation=np.zeros(9)
                self.mj.mju_quat2Mat(rotation,self.d.qpos[self.q+3:self.q+7])
                self.d.qvel[self.v+3:self.v+6]=rotation.reshape(3,3).T@w
                # No second soft-contact impulse while this same bounce exits
                # its small (1ms quantized) penetration. Never reposition it.
                self.m.pair_margin[self.table_pair]=-100.
                self.table_suppressed=True
                self.table_suppressed_at=t
                np.testing.assert_array_equal(self.d.qpos[self.q:self.q+7],qpos_before)
                extra={'impact_model':'ppq004_capped_slip_actual_contact_v1',
                    'angular_velocity_before_w':self.before_w.tolist(),
                    'angular_velocity_after_w':w.tolist(),
                    'engine_velocity_replaced':engine_velocity.tolist(),
                    'engine_normal_force':float(force[0]),'position_modified':False,
                    'normal_impulse_n_s':float(self.mass*(v[2]-self.before_v[2]))}
            self.event(kind+'_contact',t,p,velocity_before=self.before_v.tolist(),velocity_after=v.tolist(),penetration=float(-c.dist),normal_force=float(force[0]),**extra)
            if other==self.table_gid and s['racket_contacts']:
                if s['first_landing'] is None:self.landing(p,t,'table_contact')
                self.update_legal()
        self.contacts=new
        nx=self.table['near_x']+self.table['length']/2;top=self.table['surface_z']
        if self.before_p[0]<nx<=p[0]:
            f=(nx-self.before_p[0])/(p[0]-self.before_p[0]);cross=self.before_p+f*(p-self.before_p)
            clear=bool(cross[2]>top+.1525+self.radius and abs(cross[1])<=self.table['width']/2)
            self.event('net_crossing_outgoing',t,cross,clear=clear,after_racket=bool(s['racket_contacts']))
            if s['racket_contacts'] and s['first_landing'] is None:s['net_clear']=clear
        z=top+self.radius
        if self.before_p[2]>z>=p[2] and s['racket_contacts'] and s['first_landing'] is None:
            f=(z-self.before_p[2])/(p[2]-self.before_p[2]);cross=self.before_p+f*(p-self.before_p)
            self.landing(cross,t,'surface_plane')
        self.history.append((p.copy(),bool(s['racket_contacts'])))
        if s['floor_contacts'] or p[2]<-.1:self.active=False

    def landing(self,p,t,source):
        s=self.current;tab=self.table;nx=tab['near_x']+tab['length']/2
        s['first_landing']={'position':np.asarray(p).tolist(),'source':source,'episode_t':float(t)}
        on_opp=nx<p[0]<=tab['near_x']+tab['length'] and abs(p[1])<=tab['width']/2
        # Require a real table collision as well as clearance and opponent location.
        s['legal_return']=bool(s['racket_contacts']==1 and s['net_clear'] and not s['net_contacts'] and on_opp and s['table_contacts'])
        self.event('first_return_landing',t,p,legal=s['legal_return'],source=source)

    def update_legal(self):
        s=self.current
        if s['first_landing'] is None:return
        p=s['first_landing']['position'];tab=self.table;nx=tab['near_x']+tab['length']/2
        s['legal_return']=bool(s['racket_contacts']==1 and s['net_clear'] and not s['net_contacts'] and s['table_contacts'] and nx<p[0]<=tab['near_x']+tab['length'] and abs(p[1])<=tab['width']/2)

    def decorate(self,scene,target,side,clear=True):
        mj=self.mj
        if clear:scene.ngeom=0
        def line(a,b,color,width=2):
            if scene.ngeom>=scene.maxgeom:return
            g=scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
            mj.mjv_connector(g,mj.mjtGeom.mjGEOM_LINE,width,a,b);scene.ngeom+=1
        for a,b in zip(self.reference[::12],self.reference[6::12]):line(a,b,[.7,.7,.7,.65])
        hist=list(self.history)[::2]
        for (a,_),(b,hit) in zip(hist,hist[1:]):line(a,b,[0,1,1,1] if hit else [1,.55,0,1],3)
        for axis in np.eye(3)*.04:line(target.pos_w-axis,target.pos_w+axis,[1,1,0,1])
        if scene.ngeom<scene.maxgeom:
            g=scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_SPHERE,[.005]*3,target.pos_w,np.eye(3).ravel(),np.array([1,1,0,1],dtype=np.float32))
            s=self.current or {}
            g.label=('FH' if side==0 else 'BH')+(' HIT' if s.get('racket_contacts') else (' MISS' if target.time_to_strike<-.3 else ' INCOMING'))
            scene.ngeom+=1

    def draw(self,target,side):
        viewer=self.plant.viewer
        if viewer is None:return
        mj=self.mj
        with viewer.lock():self.decorate(viewer.user_scn,target,side)
        s=self.current or {};outcome='ACTUAL CONTACT' if s.get('racket_contacts') else ('MISS' if target.time_to_strike<-.3 else 'INCOMING / WAITING')
        viewer.set_texts((mj.mjtFontScale.mjFONTSCALE_150,mj.mjtGridPos.mjGRID_TOPRIGHT,
            f"PHYSICAL BALL | {'FOREHAND' if side==0 else 'BACKHAND'} | {outcome}\nOrange incoming; cyan after actual racket contact\nDashed grey: reference flight; yellow cross: strike target\nNet contacts: {s.get('net_contacts',0)} | legal return: {s.get('legal_return',False)}",''))
        viewer.sync()


class VideoRecorder:
    """CPU offscreen recording, using installed ffmpeg; no desktop required."""
    def __init__(self,plant,path,fps=20,concurrent=False):
        import subprocess
        self.plant=plant;self.fps=fps;self.frames=0
        plant.model.vis.global_.offwidth=800;plant.model.vis.global_.offheight=500
        self.renderer=plant.mj.Renderer(plant.model,height=500,width=800,max_geom=4000)
        self.camera=plant.mj.MjvCamera();self.camera.lookat[:]=[1.35,0,.85]
        self.camera.distance=6.2;self.camera.azimuth=115;self.camera.elevation=-24
        self.option=plant.mj.MjvOption()
        first='REFERENCE-SEQUENCE GRU DIAGNOSTIC - NOT CERTIFICATION' if concurrent else 'PHYSICAL BALL | orange incoming - cyan after contact'
        second='All trails actual - orange before contact - cyan after contact' if concurrent else 'Dashed grey reference | yellow strike target | actual misses retained'
        self.process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo',
            '-pixel_format','rgb24','-video_size','800x500','-framerate',str(fps),'-i','-',
            '-vf',f"drawtext=text='{first}':x=12:y=12:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.7,drawtext=text='{second}':x=12:y=38:fontsize=16:fontcolor=white:box=1:boxcolor=black@0.7",
            '-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE)

    def capture(self,target,side,elapsed):
        if elapsed+1e-8<self.frames/self.fps:return
        r=self.renderer;p=self.plant
        r.update_scene(p.data,camera=self.camera,scene_option=self.option)
        p.ball_physics.decorate(r.scene,target,side,clear=False)
        self.process.stdin.write(r.render().tobytes());self.frames+=1

    def close(self):
        self.process.stdin.close()
        code=self.process.wait(timeout=30)
        self.renderer.close()
        if code:raise RuntimeError(f'ffmpeg failed: {code}')
