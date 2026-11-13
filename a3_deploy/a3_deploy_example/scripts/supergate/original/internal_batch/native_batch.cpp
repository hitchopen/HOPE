// Offline, single-process MuJoCo batch. Production physics, actor and Planner
// cores; direct in-process messages on simulation clocks, no DDS/HAL.
#include "physics.hpp"
#include "mujoco/mjbatch.h"
#include "a3_pingpong/pp_policy.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
#include "hope_planner_cpp/future_contact_predictor.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/return_physics_yaml.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/swing_side_support.hpp"
#include <fstream>
#include <iomanip>
#include <memory>
#include <atomic>
#include <sstream>
#include <chrono>
namespace hp=hope_planner_cpp;
using robot_io::RobotCommand;
struct Row {
 int started=0,complete=0,contacts=0,tables=0,nets=0,legal=0,plans=0,valid_plans=0;
 double landx=NAN,landy=NAN,contact_time=NAN,lead=NAN,minz=10,max_tilt=0,max_tau=0;
 double strike[6]={NAN,NAN,NAN,NAN,NAN,NAN};
 double side=0;
 std::string reason="no_snapshot";
};
struct Shared {
 const double *initial_q,*initial_v,*initial_command,*cases;
 const int* bindings;
 int ncase,nenv,nthread;
 std::vector<std::string> policies;
 std::string out,return_yaml,ball_yaml,packet_yaml,planner_yaml;
 std::atomic<int> completed{0};
 std::atomic<int> exceptions{0};
};
static double val(YAML::Node n,const char*k,double fallback) {return n[k]?n[k].as<double>():fallback;}
static bool flag(YAML::Node n,const char*k,bool fallback) {return n[k]?n[k].as<bool>():fallback;}
static hp::EstimatorConfig estimator_config(YAML::Node n,const char* window) {
 hp::EstimatorConfig c;c.window_s=val(n,window,.18);c.max_sample_gap_s=val(n,"batch_max_sample_gap_s",0.);
 c.single_flight_only=flag(n,"batch_single_flight_only",false);c.min_span_s=val(n,"batch_estimator_min_span_s",.08);
 c.min_samples=val(n,"batch_estimator_min_samples",12);c.post_bounce_min_span_s=val(n,"batch_post_bounce_min_span_s",0.);
 c.post_bounce_exclusion_s=val(n,"batch_post_bounce_exclusion_s",.008);c.spin_enabled=flag(n,"future_contact_magnus_enabled",true);c.spin_min_coherence=val(n,"future_contact_spin_min_coherence",.4);return c;
}
static hp::FutureContactConfig future_config(YAML::Node n) {
 hp::FutureContactConfig c;c.spin_enabled=flag(n,"future_contact_spin_enabled",true);c.magnus_enabled=flag(n,"future_contact_magnus_enabled",true);
 c.tangential_gain=val(n,"future_contact_spin_tangential_gain",.41642144985036184);c.omega_scale=val(n,"future_contact_spin_omega_scale",1.);c.min_coherence=val(n,"future_contact_spin_min_coherence",.4);c.require_spin_for_future_contact=flag(n,"future_contact_require_spin",true);return c;
}
struct Environment {
 const mjModel*m; mjData*d; Shared& shared; int id;
 BallPhysicsRuntime physics;
 std::unique_ptr<a3_pingpong::PpPolicy> policy;
 std::shared_ptr<a3_pingpong::PpBasePoseInput> base=std::make_shared<a3_pingpong::PpBasePoseInput>();
 std::shared_ptr<a3_pingpong::PpRacketTargetInput> target=std::make_shared<a3_pingpong::PpRacketTargetInput>(4);
 std::unique_ptr<hp::IncomingTrajectory> incoming;
 std::unique_ptr<hp::FutureContactPredictor> predictor;
 std::unique_ptr<hp::RacketTargetPlanner> planner;
 hp::BallPhysics ball;hp::PlannerConfig pc;hp::EstimatorConfig ec;hp::SpinEstimatorConfig sc;
 RobotCommand command;std::vector<Row> rows;std::vector<double> pending_wire;
 int qa[31],va[31],ac[31],pelvis,torso;
 unsigned tick=0;std::uint64_t base_sequence=0,command_sequence=0;
 double pending_time=0,last_side=0;int current=-1;
 std::ofstream trajectory;
 Environment(const mjModel*mm,mjData*dd,int idx,Shared&ss):m(mm),d(dd),shared(ss),id(idx),physics(mm,dd),rows(ss.ncase) {
  for(int j=0;j<31;j++){qa[j]=ss.bindings[3*j];va[j]=ss.bindings[3*j+1];ac[j]=ss.bindings[3*j+2];}
  pelvis=mj_name2id(m,mjOBJ_BODY,"pelvis_link");torso=mj_name2id(m,mjOBJ_BODY,"torso_Link");
  std::copy_n(ss.initial_q,m->nq,d->qpos);std::copy_n(ss.initial_v,m->nv,d->qvel);d->time=0;
  for(int s=0;s<gate3::kFlightSlots;s++){int j=gate3::SlotJoint(m,s),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];d->qpos[q]=100+s;d->qpos[q+2]=-10;std::fill_n(d->qvel+v,6,0.);std::fill_n(d->userdata+s*gate3::kRequiredUserData,gate3::kRequiredUserData,0.);}
  mj_forward(m,d);a3_pingpong::simulation_time_s=0;
  a3_pingpong::PpPolicyConfig cfg;cfg.deploy_cfg_path=ss.policies[id]+"/params/deploy.yaml";cfg.planner_mode=true;cfg.policy_native=true;cfg.single_swing=true;cfg.level=0;cfg.swing_rest_s=.5;cfg.command_timeout_s=.5;cfg.loc_mode=a3_pingpong::LocMode::kExternalBase;cfg.shared_session_home=true;cfg.gate3_qdes_audit_only=true;cfg.gate3_force_every_flight=true;
  policy=std::make_unique<a3_pingpong::PpPolicy>(ss.policies[id]+"/exported/policy.onnx",cfg);policy->SetBasePoseInput(base);policy->SetRacketInput(target);policy->SetSessionHomePublisher([](const std::vector<double>&){});policy->rearm_yaw_align();
  command.q_des=Eigen::Map<const Eigen::VectorXd>(ss.initial_command,31);command.dq_des=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+31,31);command.tau_ff=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+62,31);command.kp=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+93,31);command.kd=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+124,31);
  hp::load_return_physics_yaml(ss.return_yaml,ball,pc);hp::load_ball_profile_yaml(ss.ball_yaml,ball);pc.max_predict_time_s=2.6;
  auto pn=YAML::LoadFile(ss.planner_yaml)["hope_planner"]["ros__parameters"];auto pk=YAML::LoadFile(ss.packet_yaml)["hope_ball_flight_packetizer"]["ros__parameters"];
  ec=estimator_config(pn,"batch_estimator_window_s");sc.window_s=val(pn,"spin_window_s",.18);
  auto fc=future_config(pn);predictor=std::make_unique<hp::FutureContactPredictor>(ball,pc,hp::TableParams{},fc);planner=std::make_unique<hp::RacketTargetPlanner>(ball,pc,hp::TableParams{});
  hp::IncomingTrajectoryConfig ic;ic.estimator_window_s=val(pk,"flight_window_s",.18);ic.estimator_max_sample_gap_s=val(pk,"batch_max_sample_gap_s",.02);ic.direction_fit_samples=val(pk,"incoming_direction_fit_samples",8);ic.pre_roll_samples=val(pk,"incoming_pre_roll_samples",48);ic.adaptive.enabled=flag(pk,"adaptive_commit_enabled",true);ic.adaptive.policy_lead_s=val(pk,"adaptive_policy_lead_s",.35);ic.adaptive.transport_budget_s=val(pk,"adaptive_transport_budget_s",.03);ic.adaptive.solve_budget_s=val(pk,"adaptive_solve_budget_s",.005);ic.adaptive.policy_tick_s=.02;
  hp::CommitPredictionConfig cp;cp.observed_physics=ball;cp.future_physics=ball;cp.estimator=estimator_config(pk,"flight_window_s");cp.spin_estimator=sc;cp.future_contact=future_config(pk);cp.planner=pc;cp.planner.x_hit=.07;incoming=std::make_unique<hp::IncomingTrajectory>(ic,cp);
  trajectory.open(ss.out+"/env-"+std::to_string(id)+"-ball.csv");trajectory<<"t,shot,slot,x,y,z,vx,vy,vz,racket_contacts,table_contacts,net_contacts\n"<<std::setprecision(12);
 }
 robot_io::RobotState state() {
  robot_io::RobotState s;s.q.resize(31);s.dq.resize(31);for(int j=0;j<31;j++){s.q[j]=d->qpos[qa[j]];s.dq[j]=d->qvel[va[j]];}
  s.imu_quat_wxyz=Eigen::Map<const Eigen::Vector4d>(d->xquat+4*pelvis);s.imu_gyro=Eigen::Map<const Eigen::Vector3d>(d->qvel+3);s.has_secondary_imu=true;s.sec_imu_quat_wxyz=Eigen::Map<const Eigen::Vector4d>(d->xquat+4*torso);return s;
 }
 void control() {
  double now=a3_pingpong::PpNowWallSec(),sec=std::floor(now);
  base->SetFromFlat({2,1,double(++base_sequence),sec,std::floor((now-sec)*1e9),d->qpos[0],d->qpos[1],d->qpos[2],d->qpos[3],d->qpos[4],d->qpos[5],d->qpos[6],1,double(a3_pingpong::kV17RequiredBaseFlags),1,1});
  auto st=state();policy->RecordCommandDelivery(command,true,false,&st);if(!policy->ComputeCommand(tick++,st,command))throw std::runtime_error("native ComputeCommand failed");
  auto home=policy->session_home_unsafe();if(home.epoch)incoming->set_prediction_x_hit(home.x+home.plane_x);
 }
 void observe() {
  if(current<0||d->userdata[gate3::kActive]<=.5)return;
  int j=gate3::SlotJoint(m,0),q=m->jnt_qposadr[j];hp::BallSample sample;sample.source_time_s=a3_pingpong::PpNowWallSec();sample.source_time_ns=std::llround(sample.source_time_s*1e9);sample.position=hp::Vec3(d->qpos[q],d->qpos[q+1],d->qpos[q+2]-.76);sample.orientation=Eigen::Quaterniond(d->qpos[q+3],d->qpos[q+4],d->qpos[q+5],d->qpos[q+6]);sample.orientation_valid=true;sample.sequence=base_sequence;auto update=incoming->observe(sample);if(!update.snapshot_ready)return;
  Row&r=rows[current];r.plans++;hp::BatchPhysicsEstimator est(ball,ec,sc);hp::SpinEstimator spin(sc);for(size_t i=0;i<update.snapshot.sample_count;i++){est.push(update.snapshot.samples[i]);spin.push(update.snapshot.samples[i]);}
  auto bs=est.estimate();r.reason=bs.reason;if(!bs.valid)return;auto home=policy->session_home_unsafe();if(!home.epoch){r.reason="runner_home_missing";return;}
  auto strike=predictor->predict(bs,home.x+home.plane_x,spin.estimate());r.reason=strike.reason;if(!strike.valid)return;
  double side=hp::select_swing_sign_with_support(strike.ball_position.y(),home.y,-.24,.04,last_side,{-.68,-.24},{-.24,.28});if(!std::isfinite(side)){r.reason="swing_side_target_unsupported";return;}last_side=side;
  auto cmd=planner->plan(strike,hp::Vec3(2.055,-.7625,0),.5);r.reason=cmd.reason;if(!cmd.valid)return;
  ++command_sequence;auto wire=hp::Schema2Packer::pack(&cmd,side,strike.strike_source_time_s,.76,std::llround(a3_pingpong::PpNowWallSec()*1e9),{command_sequence,std::uint64_t(current+1),1},bs.sample_count,bs.sample_span_s);
  pending_wire.assign(wire.values.begin(),wire.values.end());pending_wire[0]=4;pending_wire.push_back(double(home.epoch));pending_time=d->time+.008;r.valid_plans++;r.side=side;r.lead=strike.strike_source_time_s-a3_pingpong::PpNowWallSec();for(int k=0;k<3;k++){r.strike[k]=strike.ball_position[k];r.strike[k+3]=strike.ball_velocity[k];}
 }
 void monitor(int step) {
  double tilt=std::acos(std::clamp(d->xmat[9*pelvis+8],-1.,1.));
  if(current>=0){rows[current].minz=std::min(rows[current].minz,d->qpos[2]);rows[current].max_tilt=std::max(rows[current].max_tilt,tilt);}
  for(int s=0;s<gate3::kFlightSlots;s++){
   auto*t=d->userdata+s*gate3::kRequiredUserData;int shot=int(t[gate3::kShotId])-1;if(shot<0||shot>=shared.ncase)continue;
   Row&r=rows[shot];int j=gate3::SlotJoint(m,s),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];
   if(t[gate3::kRacketContactCount]>r.contacts){r.contact_time=d->time;}
   if(t[gate3::kTableContactCount]>r.tables&&r.contacts>0&&!std::isfinite(r.landx)){r.landx=d->qpos[q];r.landy=d->qpos[q+1];r.legal=(r.landx>1.37&&r.landx<2.74&&r.landy>-1.525&&r.landy<0&&t[gate3::kNetContactCount]==0);}
   r.contacts=int(t[gate3::kRacketContactCount]);r.tables=int(t[gate3::kTableContactCount]);r.nets=int(t[gate3::kNetContactCount]);
   if(step%4==0&&t[gate3::kActive]>=0){trajectory<<d->time<<','<<shot+1<<','<<s;for(int k=0;k<3;k++)trajectory<<','<<d->qpos[q+k];for(int k=0;k<3;k++)trajectory<<','<<d->qvel[v+k];trajectory<<','<<r.contacts<<','<<r.tables<<','<<r.nets<<'\n';}
   if(t[gate3::kActive]==0){r.complete=1;t[gate3::kActive]=-1;}
  }
 }
 void advance(int steps) {
  for(int i=0;i<steps;i++){
   a3_pingpong::simulation_time_s=d->time;
   if(!pending_wire.empty()&&d->time>=pending_time){target->SetFromFlat(pending_wire);pending_wire.clear();}
   int step=int(std::llround(d->time/.001));if(step%20==0)control();if(step%4==0)observe();
   for(int j=0;j<31;j++){double tau=command.kp[j]*(command.q_des[j]-d->qpos[qa[j]])+command.kd[j]*(command.dq_des[j]-d->qvel[va[j]])+command.tau_ff[j];if(current>=0)rows[current].max_tau=std::max(rows[current].max_tau,std::abs(tau)/m->actuator_ctrlrange[2*ac[j]+1]);d->ctrl[ac[j]]=std::clamp(tau,m->actuator_ctrlrange[2*ac[j]],m->actuator_ctrlrange[2*ac[j]+1]);}
   for(int s=0;s<gate3::kFlightSlots;s++)physics.ApplyGate3BallDrag(s);mj_step(m,d);for(int s=0;s<gate3::kFlightSlots;s++)physics.UpdateGate3BallContacts(s);monitor(step);
  }
 }
 void park() {
  int j=gate3::SlotJoint(m,0),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];gate3::PreserveFlight(m,d,q,v);d->qpos[q]=100;d->qpos[q+1]=0;d->qpos[q+2]=-10;std::fill_n(d->qvel+v,6,0);std::fill_n(d->userdata,gate3::kRequiredUserData,0);mj_forward(m,d);
 }
 void launch(int k) {
  current=k;rows[k].started=1;const double*r=shared.cases+(id%6*shared.ncase+k)*11;
  int j=gate3::SlotJoint(m,0),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];std::copy_n(r,3,d->qpos+q);d->qpos[q+2]+=.76;d->qpos[q+3]=1;std::fill_n(d->qpos+q+4,3,0);std::copy_n(r+3,3,d->qvel+v);std::copy_n(r+6,3,d->qvel+v+3);std::fill_n(d->userdata,gate3::kRequiredUserData,0);d->userdata[0]=k+1;d->userdata[1]=1;mj_forward(m,d);
 }
 void run() {
  advance(20000);
  for(int k=0;k<shared.ncase;k++){launch(k);const double*r=shared.cases+(id%6*shared.ncase+k)*11;advance(int(std::llround(r[9]*1000)));park();advance(int(std::llround(r[10]*1000)));if(k%20==19)std::fprintf(stderr,"BATCH env=%d completed_launches=%d/%d sim=%.3f\n",id,k+1,shared.ncase,d->time);}
  advance(5000);save();
 }
 void save() {
  std::ofstream f(shared.out+"/env-"+std::to_string(id)+"-results.csv");f<<"shot,started,complete,contacts,tables,nets,legal,landing_x,landing_y,contact_time,plans,valid_plans,lead,min_z,max_tilt,max_torque_ratio,side,strike_x,strike_y,strike_z,strike_vx,strike_vy,strike_vz,reason\n"<<std::setprecision(12);
  for(size_t i=0;i<rows.size();i++){const auto&r=rows[i];f<<i+1<<','<<r.started<<','<<r.complete<<','<<r.contacts<<','<<r.tables<<','<<r.nets<<','<<r.legal<<','<<r.landx<<','<<r.landy<<','<<r.contact_time<<','<<r.plans<<','<<r.valid_plans<<','<<r.lead<<','<<r.minz<<','<<r.max_tilt<<','<<r.max_tau<<','<<r.side;for(double x:r.strike)f<<','<<x;f<<','<<r.reason<<'\n';}
 }
};
static void execute(const mjModel*m,mjData*d,int id,void*arg) {
 auto&s=*static_cast<Shared*>(arg);try{Environment e(m,d,id,s);e.run();s.completed++;}catch(const std::exception&e){std::ofstream(s.out+"/env-"+std::to_string(id)+"-error.txt")<<e.what();s.exceptions++;}
}
extern "C" int batch_run(const mjModel*m,const double*q,const double*v,const double*command,const int*bindings,const double*cases,int ncase,int nenv,int nthread,const char*p0,const char*p1,const char*out,const char*return_yaml,const char*ball_yaml,const char*packet_yaml,const char*planner_yaml) {
 try{Shared s;s.initial_q=q;s.initial_v=v;s.initial_command=command;s.bindings=bindings;s.cases=cases;s.ncase=ncase;s.nenv=nenv;s.nthread=nthread;s.out=out;s.return_yaml=return_yaml;s.ball_yaml=ball_yaml;s.packet_yaml=packet_yaml;s.planner_yaml=planner_yaml;for(int i=0;i<nenv;i++)s.policies.push_back(i<6?p0:p1);
 mjBatchConfig cfg;mjb_defaultConfig(&cfg);cfg.nthread=nthread;cfg.persistent=1;char error[1024];mjBatch*b=mjb_makeBatch(m,nenv,&cfg,error,sizeof(error));if(!b)throw std::runtime_error(error);int failed=mjb_apply(b,nullptr,nenv,execute,&s,1);mjb_deleteBatch(b);return failed+s.exceptions.load();
 }catch(const std::exception&e){std::fprintf(stderr,"BATCH_ERROR %s\n",e.what());return -1;}
}
