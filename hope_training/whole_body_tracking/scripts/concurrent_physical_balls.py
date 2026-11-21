"""Independent physical balls; releasing the planner pointer never parks a flight."""
from collections import deque
from pathlib import Path
import sys
import numpy as np
from recurrent_physical_ball import PhysicalBall
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'a3_deploy/a3_deploy_example/scripts'))
from pp_gate3_core import PhysicalEvidenceAccumulator


class PoolFull(RuntimeError):
    pass


class ConcurrentBalls:
    def __init__(self,plant,shot_ids,slots=9,trajectory=None):
        self.plant=plant;self.mj=plant.mj
        self.slots=[PhysicalBall(plant,ROOT,slot=i) for i in range(slots)]
        self.score=PhysicalEvidenceAccumulator(shot_ids,min_samples=20,max_sample_gap_s=.05,
                                              min_post_contact_landing_observation_s=.75)
        self.records={};self.incoming=None;self.before_hook=None;self.after_hook=None
        self.trajectory=trajectory;self.peak_concurrent=0
        self.cfg=self.slots[0].cfg;self.pair_settings=self.slots[0].pair_settings
        for b in self.slots:b.retired=True;b.shot_id=None;b.rest_since=None
        plant.ball_physics=self
        self.mj.mj_setConst(plant.model,plant.data)
        plant.reset(0);self.park()

    def park(self):
        # Only initialization/reset API. The evaluation never calls this in-rally.
        if any(not b.retired for b in self.slots):raise RuntimeError('Cannot park live or pending flights')
        for b in self.slots:b.park()
        self.incoming=None

    def launch(self,serve,t):
        if self.incoming is not None:raise RuntimeError('Release current planner pointer before launch')
        shot_id=int(serve['shot_id'])
        if shot_id in self.records:raise ValueError('Duplicate shot ID')
        b=next((b for b in self.slots if b.retired),None)
        if b is None:raise PoolFull('All physical slots occupied; wait without truncation')
        # Allocation only touches this free ball; no robot/actor or other ball state.
        b.park();b.events=[];b.history=deque(maxlen=8000);b.contacts=set()
        b.current={'episode':0,'bank_row':None,'shot_id':shot_id,'racket_contacts':0,
            'table_contacts':0,'net_contacts':0,'floor_contacts':0,'first_landing':None,
            'net_clear':False,'legal_return':False,'arrival_error_m':None}
        b.arrival_done=True;b.strike_at=1e99;b.target_position=np.zeros(3)
        b.pending=(t,np.array(serve['position_w']),np.array(serve['velocity_w']),np.array(serve['spin_w']))
        b.reference=np.empty((0,3));b.retired=False;b.shot_id=shot_id;b.rest_since=None
        record={'shot_id':shot_id,'slot':b.slot,'launched_at_s':t,'released_at_s':None,
                'terminal_at_s':None,'terminal_kind':None,'events':b.events,'history':b.history}
        self.records[shot_id]=record;self.incoming=b
        b.before(t)  # Physically initialize this free body atomically at the launch timestamp.
        self.ingest(b,t,True)
        self.peak_concurrent=max(self.peak_concurrent,len(self.live))
        return record

    def release(self,t):
        if self.incoming is not None:
            self.records[self.incoming.shot_id]['released_at_s']=t
        self.incoming=None

    def before(self,t):
        if self.before_hook is not None:self.before_hook(t)
        for b in self.slots:b.before(t)

    def ingest(self,b,t,active):
        p,v=b.state();c=b.current
        force=max((e.get('normal_force',0.) for e in b.events[-3:] if e['kind']=='racket_contact'),default=0.)
        stamp=int(round(t*1e9))
        self.score.ingest(stamp_ns=stamp,shot_id=b.shot_id,active=active,
            position=(p-np.array([.5,.7625,0.])).tolist(),velocity=v.tolist(),
            racket_contact_count=c['racket_contacts'],table_contact_count=c['table_contacts'],
            net_contact_count=c['net_contacts'],racket_normal_force_n=force)
        if self.trajectory is not None and (not active or stamp%5_000_000==0):
            self.trajectory({'sim_t':t,'shot_id':b.shot_id,'slot':b.slot,'active':active,
                'position_w':p.tolist(),'velocity_w':v.tolist(),
                'racket_contacts':c['racket_contacts'],'table_contacts':c['table_contacts'],
                'net_contacts':c['net_contacts'],'is_planner_input':b is self.incoming})

    def after(self,t):
        for b in self.slots:
            if b.retired:continue
            b.after(t)
            if not b.active and b.pending is not None:continue
            p,v=b.state()
            before=b.before_p
            for plane,kind in [(1.87,'net_crossing_incoming'),(.58,'hit_plane_incoming')]:
                if before[0]>plane>=p[0]:
                    f=(plane-before[0])/(p[0]-before[0]);cross=before+f*(p-before)
                    b.event(kind,t,cross,clear=bool(cross[2]>.9325 and abs(cross[1])<=.7625))
            self.ingest(b,t,True)
            # A physical measured endpoint, never an external park or slot deadline.
            ground=bool(p[2]<=b.radius and p[2]>=-.1 and v[2]<0 and b.current['floor_contacts']>0)
            on_table=(.5<=p[0]<=3.24 and abs(p[1])<=.7625 and abs(p[2]-.78)<=.001
                      and np.linalg.norm(v[:2])<=.002 and abs(v[2])<=.12 and b.current['table_contacts']>0)
            if not on_table:b.rest_since=None
            elif b.rest_since is None:b.rest_since=t
            rest=bool(b.rest_since is not None and t-b.rest_since>=2.)
            if ground or rest:
                # Same measured state, explicitly inactive after the terminal edge.
                self.ingest(b,t+1e-9,False)
                r=self.records[b.shot_id];r['terminal_at_s']=t;r['terminal_kind']='ground_crossing' if ground else 'settled_on_table'
                r['terminal_position']=p.tolist();r['terminal_velocity']=v.tolist()
                b.active=False;b.retired=True
            else:
                # Parent's single-ball floor latch must not truncate an unresolved flight.
                b.active=True
        self.peak_concurrent=max(self.peak_concurrent,sum(not b.retired for b in self.slots))
        if self.after_hook is not None:self.after_hook(t)

    @property
    def live(self):return [b.shot_id for b in self.slots if not b.retired]

    def planner_ball(self):
        b=self.incoming
        return b if b is not None and not b.retired and b.active else None

    def decorate(self,scene,target,side=0,clear=True):
        mj=self.mj
        if clear:scene.ngeom=0
        # Current flights plus two recently resolved trails; all raw trajectories persist.
        records=list(self.records.values())
        shown=[r for r in records if r['terminal_at_s'] is None]+[r for r in records if r['terminal_at_s'] is not None][-2:]
        for r in shown:
            hist=list(r['history'])[::10]
            for (a,_),(b,hit) in zip(hist,hist[1:]):
                if scene.ngeom>=scene.maxgeom-5:break
                g=scene.geoms[scene.ngeom]
                mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([0,1,1,1] if hit else [1,.55,0,1],dtype=np.float32))
                mj.mjv_connector(g,mj.mjtGeom.mjGEOM_LINE,2,a,b);scene.ngeom+=1
        if target is not None and scene.ngeom<scene.maxgeom:
            g=scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_SPHERE,[.012]*3,target.pos_w,np.eye(3).ravel(),np.array([1,1,0,1],dtype=np.float32))
            g.label='LIVE PLANNER TARGET';scene.ngeom+=1

    def draw(self,target,side=0):
        viewer=self.plant.viewer
        if viewer is None:return
        with viewer.lock():self.decorate(viewer.user_scn,target)
        current=self.planner_ball()
        viewer.set_texts((self.mj.mjtFontScale.mjFONTSCALE_150,self.mj.mjtGridPos.mjGRID_TOPRIGHT,
            f'REFERENCE-SEQUENCE GRU DIAGNOSTIC | NOT GATE3 CERTIFICATION\nIncoming ID: {current.shot_id if current else "none"} | active: {self.live}\nOrange incoming; cyan after actual contact; yellow live plan',''))
        viewer.sync()
