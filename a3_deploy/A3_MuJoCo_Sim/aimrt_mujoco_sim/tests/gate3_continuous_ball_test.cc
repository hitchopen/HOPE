// Regression: launcher handoff preserves a real MuJoCo trajectory and telemetry.
#include <cassert>
#include <cmath>
#include <iostream>
#include "mujoco_sim_module/common/gate3_ball_layout.h"
namespace g = aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball;
int main(int argc, char** argv) {
  g::TableRestTracker rest;
  mjtNum rp[3]={2.2,-.76,.7802},rv[3]={.001,0,.03};
  assert(!rest.Update(1,0,rp,rv,1));
  assert(!rest.Update(1,1.99,rp,rv,1));
  assert(rest.Update(1,2.01,rp,rv,1));
  assert(!rest.Update(2,2.02,rp,rv,1));  // reused slot resets dwell
  rv[0]=.02;
  assert(!rest.Update(2,10,rp,rv,1));    // rolling ball never timeouts
  rv[0]=.001;rp[2]=1.0;
  assert(!rest.Update(2,20,rp,rv,1));    // airborne ball never timeouts
  rp[2]=.7802;
  assert(!rest.Update(2,30,rp,rv,1));
  assert(!rest.Update(2,0,rp,rv,1));     // simulation reset resets dwell
  rp[0]=2.7393;                       // table-edge rest is still physical rest
  assert(rest.Update(2,2.01,rp,rv,1));
  rp[0]=2.75;
  assert(!rest.Update(2,10,rp,rv,1));
  char error[1024];
  auto* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) { std::cerr << error; return 1; }
  auto* d = mj_makeData(m);
  auto* reference = mj_makeData(m);
  const int j = g::SlotJoint(m, 0), q = m->jnt_qposadr[j], v = m->jnt_dofadr[j];
  d->qpos[q] = 1.8; d->qpos[q+1] = -0.76; d->qpos[q+2] = 2.0;
  d->qvel[v] = 2.0; d->qvel[v+2] = -1.0;
  d->userdata[g::kShotId] = 7; d->userdata[g::kActive] = 1;
  d->userdata[g::kRacketContactCount] = 1; d->userdata[g::kTableContactCount] = 1;
  mj_copyData(reference,m,d);
  g::PreserveFlight(m,d,q,v);
  const int j1=g::SlotJoint(m,1), q1=m->jnt_qposadr[j1], v1=m->jnt_dofadr[j1];
  for(int i=0;i<7;++i) assert(d->qpos[q+i]==d->qpos[q1+i]);
  for(int i=0;i<6;++i) assert(d->qvel[v+i]==d->qvel[v1+i]);
  for(int i=0;i<7;++i) assert(d->userdata[i]==d->userdata[7+i]);
  // Next serve overwrites only slot 0. The prior ball continues in slot 1.
  d->qpos[q]=100;d->qpos[q+2]=-10;
  std::fill_n(d->qvel+v,6,0.0);
  bool touched_table=false;
  const int table=mj_name2id(m,mjOBJ_GEOM,g::kTableGeomName);
  const int geom=mj_name2id(m,mjOBJ_GEOM,g::SlotName(g::kGeomName,1).c_str());
  assert(g::IsFlightGeom(m,geom));
  assert(!g::IsFlightGeom(m,table));
  // Ball physical flight must match a single-ball reference across the handoff.
  for(int t=0;t<std::ceil(0.8/m->opt.timestep);++t) {
    mj_step(m,d);mj_step(m,reference);
    for(int i=0;i<3;++i) assert(std::abs(d->qpos[q1+i]-reference->qpos[q+i])<1e-9);
    for(int c=0;c<d->ncon;++c) {
      auto& x=d->contact[c];
      if((x.geom1==geom&&x.geom2==table)||(x.geom2==geom&&x.geom1==table)) touched_table=true;
    }
  }
  assert(touched_table);
  // A terminal sample awaiting publication cannot be silently overwritten.
  for(int slot=1;slot<g::kFlightSlots;++slot) {
    d->userdata[slot*7+g::kShotId]=slot;
    d->userdata[slot*7+g::kActive]=0;
  }
  bool pending_refused=false;
  try { g::PreserveFlight(m,d,q,v); } catch(const std::runtime_error&) {pending_refused=true;}
  assert(pending_refused);
  d->userdata[7+g::kActive]=-1;
  g::PreserveFlight(m,d,q,v);
  assert(d->userdata[7+g::kActive]==1);
  // A full pool must report an error; never silently replace an airborne ball.
  for(int slot=1;slot<g::kFlightSlots;++slot) d->userdata[slot*7+g::kActive]=1;
  bool refused=false;
  try { g::PreserveFlight(m,d,q,v); } catch(const std::runtime_error&) {refused=true;}
  assert(refused);
  std::cout << "PASS: exact handoff, preserved counters, continuous trajectory, actual table contact, overflow refusal\n";
  mj_deleteData(d);mj_deleteData(reference);mj_deleteModel(m);
}
