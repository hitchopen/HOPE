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
 double deadline=NAN,target_p[3]={NAN,NAN,NAN},target_v[3]={NAN,NAN,NAN},target_n[3]={NAN,NAN,NAN};
 bool planned_sample=false,contact_sample=false;
 double peak_pelvis_yaw=0,peak_torso_yaw=0,peak_twist=0,end_pelvis_yaw=0,end_torso_yaw=0,peak_roll=0,peak_pitch=0,base_xy_drift=0;
 int posture_samples=0,torso_over30=0,torso_over45=0,clipped_steps=0,control_steps=0,trunk_floor_contacts=0;
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
 std::ofstream kinematics,posture,observations,planned_state,control_state,feet,warmup_posture;
 int foot_body[2];
 bool early_reject=false,aborted=false;
 bool live_support=false;std::array<double,2> fh_support{-.68,-.24},bh_support{-.24,.28};
 mjData* diagnostic=nullptr;
 int racket_site,racket_geom;
 double diagnostic_time=-1,initial_pelvis_yaw=0,initial_torso_yaw=0;
 std::vector<mjtNum> jacobian;
 Environment(const mjModel*mm,mjData*dd,int idx,Shared&ss):m(mm),d(dd),shared(ss),id(idx),physics(mm,dd),rows(ss.ncase) {
  for(int j=0;j<31;j++){qa[j]=ss.bindings[3*j];va[j]=ss.bindings[3*j+1];ac[j]=ss.bindings[3*j+2];}
  pelvis=mj_name2id(m,mjOBJ_BODY,"pelvis_link");torso=mj_name2id(m,mjOBJ_BODY,"torso_Link");
  std::copy_n(ss.initial_q,m->nq,d->qpos);std::copy_n(ss.initial_v,m->nv,d->qvel);d->time=0;
  for(int s=0;s<gate3::kFlightSlots;s++){int j=gate3::SlotJoint(m,s),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];d->qpos[q]=100+s;d->qpos[q+2]=-10;std::fill_n(d->qvel+v,6,0.);std::fill_n(d->userdata+s*gate3::kRequiredUserData,gate3::kRequiredUserData,0.);}
  mj_forward(m,d);a3_pingpong::simulation_time_s=0;
  a3_pingpong::PpPolicyConfig cfg;cfg.deploy_cfg_path=ss.policies[id]+"/params/deploy.yaml";cfg.planner_mode=true;cfg.policy_native=true;cfg.single_swing=true;cfg.level=0;cfg.swing_rest_s=.5;cfg.command_timeout_s=.5;cfg.loc_mode=a3_pingpong::LocMode::kExternalBase;cfg.shared_session_home=true;cfg.gate3_qdes_audit_only=true;cfg.gate3_force_every_flight=true;
  if(const char* support_path=std::getenv("PP_GATE3_SUPPORT_PROFILE")){
   auto sp=YAML::LoadFile(support_path);live_support=sp["live_base"].as<bool>();cfg.target_support_live_base=live_support;cfg.empirical_lateral_support=a3_pingpong::ParseEmpiricalLateralSupport(sp["empirical_support"]);
   fh_support=a3_pingpong::EffectiveLateralSupport(cfg.empirical_lateral_support,0,fh_support);bh_support=a3_pingpong::EffectiveLateralSupport(cfg.empirical_lateral_support,1,bh_support);
  }
  if(const char* early=std::getenv("PP_GATE3_EARLY_REJECT"))early_reject=std::string(early)=="1";
  policy=std::make_unique<a3_pingpong::PpPolicy>(ss.policies[id]+"/exported/policy.onnx",cfg);policy->SetBasePoseInput(base);policy->SetRacketInput(target);policy->SetSessionHomePublisher([](const std::vector<double>&){});policy->rearm_yaw_align();
  command.q_des=Eigen::Map<const Eigen::VectorXd>(ss.initial_command,31);command.dq_des=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+31,31);command.tau_ff=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+62,31);command.kp=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+93,31);command.kd=Eigen::Map<const Eigen::VectorXd>(ss.initial_command+124,31);
  hp::load_return_physics_yaml(ss.return_yaml,ball,pc);hp::load_ball_profile_yaml(ss.ball_yaml,ball);pc.max_predict_time_s=2.6;
  auto pn=YAML::LoadFile(ss.planner_yaml)["hope_planner"]["ros__parameters"];auto pk=YAML::LoadFile(ss.packet_yaml)["hope_ball_flight_packetizer"]["ros__parameters"];
  ec=estimator_config(pn,"batch_estimator_window_s");sc.window_s=val(pn,"spin_window_s",.18);
  auto fc=future_config(pn);predictor=std::make_unique<hp::FutureContactPredictor>(ball,pc,hp::TableParams{},fc);planner=std::make_unique<hp::RacketTargetPlanner>(ball,pc,hp::TableParams{});
  hp::IncomingTrajectoryConfig ic;ic.estimator_window_s=val(pk,"flight_window_s",.18);ic.estimator_max_sample_gap_s=val(pk,"batch_max_sample_gap_s",.02);ic.direction_fit_samples=val(pk,"incoming_direction_fit_samples",8);ic.pre_roll_samples=val(pk,"incoming_pre_roll_samples",48);ic.adaptive.enabled=flag(pk,"adaptive_commit_enabled",true);ic.adaptive.policy_lead_s=val(pk,"adaptive_policy_lead_s",.35);ic.adaptive.transport_budget_s=val(pk,"adaptive_transport_budget_s",.03);ic.adaptive.solve_budget_s=val(pk,"adaptive_solve_budget_s",.005);ic.adaptive.policy_tick_s=.02;
  hp::CommitPredictionConfig cp;cp.observed_physics=ball;cp.future_physics=ball;cp.estimator=estimator_config(pk,"flight_window_s");cp.spin_estimator=sc;cp.future_contact=future_config(pk);cp.planner=pc;cp.planner.x_hit=.07;incoming=std::make_unique<hp::IncomingTrajectory>(ic,cp);
  trajectory.open(ss.out+"/env-"+std::to_string(id)+"-ball.csv");trajectory<<"t,shot,slot,x,y,z,vx,vy,vz,racket_contacts,table_contacts,net_contacts\n"<<std::setprecision(12);
  diagnostic=mj_makeData(m);if(!diagnostic)throw std::runtime_error("diagnostic data allocation failed");jacobian.resize(3*m->nv);
  racket_site=mj_name2id(m,mjOBJ_SITE,"right_racket");racket_geom=mj_name2id(m,mjOBJ_GEOM,gate3::kRacketGeomName);
  kinematics.open(ss.out+"/env-"+std::to_string(id)+"-kinematics.csv");kinematics<<"shot,event,t,deadline,dt_ms";for(auto prefix:{"target_p","target_v","target_n","racket_p","racket_v","racket_n","dp","dv","dn","legacy_site_p","surface_p","surface_v"})for(auto axis:{"x","y","z"})kinematics<<','<<prefix<<'_'<<axis;kinematics<<",p_error_m,v_error_mps,n_error_deg,pelvis_yaw_deg,torso_yaw_deg,twist_deg\n"<<std::setprecision(12);
  warmup_posture.open(ss.out+"/env-"+std::to_string(id)+"-warmup-posture.csv");warmup_posture<<"t,base_x,base_y,base_z,roll_deg,pitch_deg,tilt_deg,pelvis_yaw_deg,torso_yaw_deg\n"<<std::setprecision(12);
  posture.open(ss.out+"/env-"+std::to_string(id)+"-posture.csv");posture<<"t,shot,base_x,base_y,base_z,pelvis_yaw_deg,torso_yaw_deg,twist_deg,roll_deg,pitch_deg,tilt_deg\n"<<std::setprecision(12);
  observations.open(ss.out+"/env-"+std::to_string(id)+"-observations.csv");observations<<"t,shot,tick";for(int k=0;k<110;k++)observations<<",obs_"<<k;observations<<"\n"<<std::setprecision(12);
  planned_state.open(ss.out+"/env-"+std::to_string(id)+"-planned-state.csv");planned_state<<"shot,t";for(int k=0;k<m->nq;k++)planned_state<<",q_"<<k;for(int k=0;k<m->nv;k++)planned_state<<",v_"<<k;planned_state<<"\n"<<std::setprecision(12);
  feet.open(ss.out+"/env-"+std::to_string(id)+"-feet.csv");feet<<"t,force_t,shot,deadline,ttc";
  for(int s=0;s<2;s++){std::string side=s?"right":"left";foot_body[s]=mj_name2id(m,mjOBJ_BODY,(side+"_ankle_roll_Link").c_str());for(auto name:{"roll_deg","pitch_deg","outer_above_inner_mm","fz_N","medial_fz_N","lateral_fz_N","center_fz_N","cop_medial_mm","min_medial_z_mm","min_lateral_z_mm"})feet<<','<<side<<'_'<<name;}
  for(auto prefix:{"q","qdes","tau"})for(int j=0;j<31;j++)feet<<','<<prefix<<'_'<<j;
  feet<<"\n"<<std::setprecision(12);
  control_state.open(ss.out+"/env-"+std::to_string(id)+"-control-state.csv");control_state<<"shot,t,tick";for(int k=0;k<m->nq;k++)control_state<<",q_"<<k;for(int k=0;k<m->nv;k++)control_state<<",v_"<<k;control_state<<"\n"<<std::setprecision(12);
 }
 ~Environment(){if(diagnostic)mj_deleteData(diagnostic);}
 static double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
 static double yaw(const mjtNum* a){return std::atan2(a[3],a[0]);}
 void diagnostic_pose(){
  if(diagnostic_time==d->time)return;
  std::copy_n(d->qpos,m->nq,diagnostic->qpos);std::copy_n(d->qvel,m->nv,diagnostic->qvel);
  mj_kinematics(m,diagnostic);mj_comPos(m,diagnostic);diagnostic_time=d->time;
 }
 void sample_kinematics(int shot,const char* event,const mjData* data,double t,const mjtNum* surface=nullptr){
  Row&r=rows[shot];if(!std::isfinite(r.deadline))return;
  Eigen::Map<const Eigen::Vector3d> site(data->site_xpos+3*racket_site);
  const auto* rot=data->site_xmat+9*racket_site;Eigen::Vector3d n(rot[1],rot[4],rot[7]);
  Eigen::Vector3d p=site+n*(r.side<0?-.013208:0.);Eigen::Vector3d targetn=Eigen::Map<const Eigen::Vector3d>(r.target_n);if(n.dot(targetn)<0)n=-n;
  mj_jac(m,data,jacobian.data(),nullptr,p.data(),m->site_bodyid[racket_site]);Eigen::Vector3d v=Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor>>(jacobian.data(),3,m->nv)*Eigen::Map<const Eigen::VectorXd>(data->qvel,m->nv);
  Eigen::Vector3d dp=p-Eigen::Map<const Eigen::Vector3d>(r.target_p),dv=v-Eigen::Map<const Eigen::Vector3d>(r.target_v),dn=n-targetn;
  Eigen::Vector3d surfacep=Eigen::Vector3d::Constant(NAN),surfacev=surfacep;
  if(surface){surfacep=Eigen::Map<const Eigen::Vector3d>(surface);mjtNum spatial[6];mj_objectVelocity(m,data,mjOBJ_GEOM,racket_geom,spatial,0);surfacev=Eigen::Map<const Eigen::Vector3d>(spatial+3)+Eigen::Map<const Eigen::Vector3d>(spatial).cross(surfacep-Eigen::Map<const Eigen::Vector3d>(data->geom_xpos+3*racket_geom));}
  kinematics<<shot+1<<','<<event<<','<<t<<','<<r.deadline<<','<<1000*(t-r.deadline);
  for(auto a:{Eigen::Vector3d(Eigen::Map<const Eigen::Vector3d>(r.target_p)),Eigen::Vector3d(Eigen::Map<const Eigen::Vector3d>(r.target_v)),targetn,p,v,n,dp,dv,dn,Eigen::Vector3d(site),surfacep,surfacev})for(int k=0;k<3;k++)kinematics<<','<<a[k];
  double py=angle(yaw(data->xmat+9*pelvis)-initial_pelvis_yaw),ty=angle(yaw(data->xmat+9*torso)-initial_torso_yaw);
  kinematics<<','<<dp.norm()<<','<<dv.norm()<<','<<std::acos(std::clamp(n.dot(targetn),-1.,1.))*180/M_PI<<','<<py*180/M_PI<<','<<ty*180/M_PI<<','<<angle(ty-py)*180/M_PI<<'\n';
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
  const auto& ob=policy->last_obs_unsafe();if(ob.size()==110){observations<<d->time<<','<<current+1<<','<<tick;for(int k=0;k<110;k++)observations<<','<<ob[k];observations<<'\n';}
  if(ob.size()==110&&current>=0&&ob[109]>.0-.20&&ob[109]<.30){control_state<<current+1<<','<<d->time<<','<<tick;for(int k=0;k<m->nq;k++)control_state<<','<<d->qpos[k];for(int k=0;k<m->nv;k++)control_state<<','<<d->qvel[k];control_state<<'\n';}
 }
 void observe() {
  if(current<0||d->userdata[gate3::kActive]<=.5)return;
  int j=gate3::SlotJoint(m,0),q=m->jnt_qposadr[j];hp::BallSample sample;sample.source_time_s=a3_pingpong::PpNowWallSec();sample.source_time_ns=std::llround(sample.source_time_s*1e9);sample.position=hp::Vec3(d->qpos[q],d->qpos[q+1],d->qpos[q+2]-.76);sample.orientation=Eigen::Quaterniond(d->qpos[q+3],d->qpos[q+4],d->qpos[q+5],d->qpos[q+6]);sample.orientation_valid=true;sample.sequence=base_sequence;auto update=incoming->observe(sample);if(!update.snapshot_ready)return;
  Row&r=rows[current];r.plans++;hp::BatchPhysicsEstimator est(ball,ec,sc);hp::SpinEstimator spin(sc);for(size_t i=0;i<update.snapshot.sample_count;i++){est.push(update.snapshot.samples[i]);spin.push(update.snapshot.samples[i]);}
  auto bs=est.estimate();r.reason=bs.reason;if(!bs.valid)return;auto home=policy->session_home_unsafe();if(!home.epoch){r.reason="runner_home_missing";return;}
  auto strike=predictor->predict(bs,home.x+home.plane_x,spin.estimate());r.reason=strike.reason;if(!strike.valid)return;
  double side=hp::select_swing_sign_with_support(strike.ball_position.y(),live_support?d->qpos[1]:home.y,-.24,.04,last_side,{fh_support[0],fh_support[1]},{bh_support[0],bh_support[1]});if(!std::isfinite(side)){r.reason="swing_side_target_unsupported";return;}last_side=side;
  auto cmd=planner->plan(strike,hp::Vec3(2.055,-.7625,0),.5);r.reason=cmd.reason;if(!cmd.valid)return;
  ++command_sequence;auto wire=hp::Schema2Packer::pack(&cmd,side,strike.strike_source_time_s,.76,std::llround(a3_pingpong::PpNowWallSec()*1e9),{command_sequence,std::uint64_t(current+1),1},bs.sample_count,bs.sample_span_s);
  pending_wire.assign(wire.values.begin(),wire.values.end());pending_wire[0]=4;pending_wire.push_back(double(home.epoch));pending_time=d->time+.008;r.valid_plans++;r.side=side;r.lead=strike.strike_source_time_s-a3_pingpong::PpNowWallSec();for(int k=0;k<3;k++){r.strike[k]=strike.ball_position[k];r.strike[k+3]=strike.ball_velocity[k];}
  r.deadline=strike.strike_source_time_s-1800000000.;for(int k=0;k<3;k++){r.target_p[k]=cmd.position[k]+(k==2?.76:0);r.target_v[k]=cmd.velocity[k];r.target_n[k]=cmd.normal[k];}
 }
 // Read-only: use native solved contact forces from the preceding 1 ms step,
 // and independent FK at the current integrated state. No mj_forward on live data.
 void sample_feet(){
  diagnostic_pose();const auto&ob=policy->last_obs_unsafe();
  feet<<d->time<<','<<d->time-.001<<','<<current+1<<','<<(current>=0?rows[current].deadline:NAN)<<','<<(ob.size()==110?ob[109]:NAN);
  for(int s=0;s<2;s++){
   int body=foot_body[s];const auto*R=diagnostic->xmat+9*body;double total=0,med=0,lat=0,center=0,cop=0,zmed=1e9,zlat=1e9;double sign=s?1:-1;
   for(int ci=0;ci<d->ncon;ci++){
    const auto&ct=d->contact[ci];int b1=m->geom_bodyid[ct.geom1],b2=m->geom_bodyid[ct.geom2];
    if(!((b1==body&&b2==0)||(b2==body&&b1==0)))continue;
    mjtNum f[6];mj_contactForce(m,d,ci,f);double fz=std::abs(ct.frame[2]*f[0]+ct.frame[5]*f[1]+ct.frame[8]*f[2]);
    const auto*oldR=d->xmat+9*body;double y=0;for(int k=0;k<3;k++)y+=oldR[3*k+1]*(ct.pos[k]-d->xpos[3*body+k]);y*=sign;
    total+=fz;cop+=fz*y;if(y>.005)med+=fz;else if(y<-.005)lat+=fz;else center+=fz;
   }
   for(int g=0;g<m->ngeom;g++)if(m->geom_bodyid[g]==body&&m->geom_contype[g]&&m->geom_type[g]==mjGEOM_MESH){
    int mesh=m->geom_dataid[g];for(int v=0;v<m->mesh_vertnum[mesh];v++){const auto*p=m->mesh_vert+3*(m->mesh_vertadr[mesh]+v);double world[3];for(int k=0;k<3;k++){world[k]=diagnostic->geom_xpos[3*g+k];for(int j=0;j<3;j++)world[k]+=diagnostic->geom_xmat[9*g+3*k+j]*p[j];}double y=0;for(int k=0;k<3;k++)y+=R[3*k+1]*(world[k]-diagnostic->xpos[3*body+k]);y*=sign;if(y>.005)zmed=std::min(zmed,world[2]);if(y<-.005)zlat=std::min(zlat,world[2]);}
   }
   feet<<','<<std::atan2(R[7],R[8])*180/M_PI<<','<<std::asin(std::clamp(-R[6],-1.,1.))*180/M_PI<<','<<-sign*.102*R[7]*1000<<','<<total<<','<<med<<','<<lat<<','<<center<<','<<(total>1e-8?cop/total*1000:NAN)<<','<<zmed*1000<<','<<zlat*1000;
  }
  for(int j=0;j<31;j++)feet<<','<<d->qpos[qa[j]];for(int j=0;j<31;j++)feet<<','<<command.q_des[j];for(int j=0;j<31;j++)feet<<','<<d->ctrl[ac[j]];feet<<'\n';
 }
 void monitor(int step) {
  if(step%20==0)sample_feet();
  double tilt=std::acos(std::clamp(d->xmat[9*pelvis+8],-1.,1.));
  if(current<0&&step%20==0){
   diagnostic_pose();const auto*pr=diagnostic->xmat+9*pelvis;const auto*tr=diagnostic->xmat+9*torso;
   warmup_posture<<d->time<<','<<d->qpos[0]<<','<<d->qpos[1]<<','<<d->qpos[2]<<','<<std::atan2(pr[7],pr[8])*180/M_PI<<','<<std::asin(std::clamp(-pr[6],-1.,1.))*180/M_PI<<','<<tilt*180/M_PI<<','<<yaw(pr)*180/M_PI<<','<<yaw(tr)*180/M_PI<<'\n';
  }
  if(current>=0){rows[current].minz=std::min(rows[current].minz,d->qpos[2]);rows[current].max_tilt=std::max(rows[current].max_tilt,tilt);}
  for(int s=0;s<gate3::kFlightSlots;s++){
   auto*t=d->userdata+s*gate3::kRequiredUserData;int shot=int(t[gate3::kShotId])-1;if(shot<0||shot>=shared.ncase)continue;
   Row&r=rows[shot];int j=gate3::SlotJoint(m,s),q=m->jnt_qposadr[j],v=m->jnt_dofadr[j];
   if(t[gate3::kRacketContactCount]>r.contacts){r.contact_time=d->time;}
   if(t[gate3::kRacketContactCount]>r.contacts&&!r.contact_sample){
    const int ball_geom=mj_name2id(m,mjOBJ_GEOM,gate3::SlotName(gate3::kGeomName,s).c_str());
    for(int c=0;c<d->ncon;c++){const auto&ct=d->contact[c];if((ct.geom1==ball_geom&&ct.geom2==racket_geom)||(ct.geom2==ball_geom&&ct.geom1==racket_geom)){sample_kinematics(shot,"contact",d,d->time-.001,ct.pos);r.contact_sample=true;break;}}
   }
   if(t[gate3::kTableContactCount]>r.tables&&r.contacts>0&&!std::isfinite(r.landx)){r.landx=d->qpos[q];r.landy=d->qpos[q+1];r.legal=(r.landx>1.37&&r.landx<2.74&&r.landy>-1.525&&r.landy<0&&t[gate3::kNetContactCount]==0);}
   r.contacts=int(t[gate3::kRacketContactCount]);r.tables=int(t[gate3::kTableContactCount]);r.nets=int(t[gate3::kNetContactCount]);
   if(step%4==0&&t[gate3::kActive]>=0){trajectory<<d->time<<','<<shot+1<<','<<s;for(int k=0;k<3;k++)trajectory<<','<<d->qpos[q+k];for(int k=0;k<3;k++)trajectory<<','<<d->qvel[v+k];trajectory<<','<<r.contacts<<','<<r.tables<<','<<r.nets<<'\n';}
   if(t[gate3::kActive]==0){r.complete=1;t[gate3::kActive]=-1;}
  }
  if(current>=0){
   auto&r=rows[current];
   if(!r.planned_sample&&std::isfinite(r.deadline)&&d->time>=r.deadline){diagnostic_pose();sample_kinematics(current,"planned",diagnostic,d->time);r.planned_sample=true;planned_state<<current+1<<','<<d->time;for(int k=0;k<m->nq;k++)planned_state<<','<<d->qpos[k];for(int k=0;k<m->nv;k++)planned_state<<','<<d->qvel[k];planned_state<<'\n';}
   if(step%4==0){diagnostic_pose();const auto*pr=diagnostic->xmat+9*pelvis;const auto*tr=diagnostic->xmat+9*torso;double py=angle(yaw(pr)-initial_pelvis_yaw),ty=angle(yaw(tr)-initial_torso_yaw),tw=angle(ty-py),roll=std::atan2(pr[7],pr[8]),pitch=std::asin(std::clamp(-pr[6],-1.,1.));
    r.peak_pelvis_yaw=std::max(r.peak_pelvis_yaw,std::abs(py));r.peak_torso_yaw=std::max(r.peak_torso_yaw,std::abs(ty));r.peak_twist=std::max(r.peak_twist,std::abs(tw));r.end_pelvis_yaw=py;r.end_torso_yaw=ty;r.peak_roll=std::max(r.peak_roll,std::abs(roll));r.peak_pitch=std::max(r.peak_pitch,std::abs(pitch));r.base_xy_drift=std::max(r.base_xy_drift,std::hypot(d->qpos[0]+.5,d->qpos[1]+.7625));r.posture_samples++;r.torso_over30+=std::abs(ty)>M_PI/6;r.torso_over45+=std::abs(ty)>M_PI/4;
    if(step%20==0)posture<<d->time<<','<<current+1<<','<<d->qpos[0]<<','<<d->qpos[1]<<','<<d->qpos[2]<<','<<py*180/M_PI<<','<<ty*180/M_PI<<','<<tw*180/M_PI<<','<<roll*180/M_PI<<','<<pitch*180/M_PI<<','<<tilt*180/M_PI<<'\n';
   }
  }
 }
 void advance(int steps) {
  for(int i=0;i<steps;i++){
   a3_pingpong::simulation_time_s=d->time;
   if(!pending_wire.empty()&&d->time>=pending_time){target->SetFromFlat(pending_wire);pending_wire.clear();}
   int step=int(std::llround(d->time/.001));if(step%20==0)control();if(step%4==0)observe();
   bool clip=false;for(int j=0;j<31;j++){double tau=command.kp[j]*(command.q_des[j]-d->qpos[qa[j]])+command.kd[j]*(command.dq_des[j]-d->qvel[va[j]])+command.tau_ff[j];if(current>=0)rows[current].max_tau=std::max(rows[current].max_tau,std::abs(tau)/m->actuator_ctrlrange[2*ac[j]+1]);clip|=tau<m->actuator_ctrlrange[2*ac[j]]||tau>m->actuator_ctrlrange[2*ac[j]+1];d->ctrl[ac[j]]=std::clamp(tau,m->actuator_ctrlrange[2*ac[j]],m->actuator_ctrlrange[2*ac[j]+1]);}if(current>=0){rows[current].control_steps++;rows[current].clipped_steps+=clip;}
   for(int s=0;s<gate3::kFlightSlots;s++)physics.ApplyGate3BallDrag(s);mj_step(m,d);for(int s=0;s<gate3::kFlightSlots;s++)physics.UpdateGate3BallContacts(s);monitor(step);
   if(early_reject&&current>=0&&(rows[current].minz<.85||rows[current].max_tilt>.65)){
    aborted=true;std::ofstream f(shared.out+"/env-"+std::to_string(id)+"-early-rejection.csv");
    f<<"shot,time_s,min_z,max_tilt_rad,planned_shots\n"<<std::setprecision(12)<<current+1<<','<<d->time<<','<<rows[current].minz<<','<<rows[current].max_tilt<<','<<shared.ncase<<'\n';
    std::fprintf(stderr,"EARLY_REJECT env=%d shot=%d t=%.3f\n",id,current+1,d->time);break;
   }
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
  diagnostic_pose();initial_pelvis_yaw=yaw(diagnostic->xmat+9*pelvis);initial_torso_yaw=yaw(diagnostic->xmat+9*torso);
  for(int k=0;k<shared.ncase;k++){launch(k);const double*r=shared.cases+(id%6*shared.ncase+k)*11;advance(int(std::llround(r[9]*1000)));if(aborted)break;park();advance(int(std::llround(r[10]*1000)));if(aborted)break;if(k%20==19)std::fprintf(stderr,"BATCH env=%d completed_launches=%d/%d sim=%.3f\n",id,k+1,shared.ncase,d->time);}
  if(!aborted)advance(5000);if(aborted)rows.resize(current+1);save();
 }
 void save() {
  std::ofstream f(shared.out+"/env-"+std::to_string(id)+"-results.csv");f<<"shot,started,complete,contacts,tables,nets,legal,landing_x,landing_y,contact_time,plans,valid_plans,lead,min_z,max_tilt,max_torque_ratio,side,strike_x,strike_y,strike_z,strike_vx,strike_vy,strike_vz,reason\n"<<std::setprecision(12);
  for(size_t i=0;i<rows.size();i++){const auto&r=rows[i];f<<i+1<<','<<r.started<<','<<r.complete<<','<<r.contacts<<','<<r.tables<<','<<r.nets<<','<<r.legal<<','<<r.landx<<','<<r.landy<<','<<r.contact_time<<','<<r.plans<<','<<r.valid_plans<<','<<r.lead<<','<<r.minz<<','<<r.max_tilt<<','<<r.max_tau<<','<<r.side;for(double x:r.strike)f<<','<<x;f<<','<<r.reason<<'\n';}
  std::ofstream a(shared.out+"/env-"+std::to_string(id)+"-audit.csv");a<<"shot,planned_sample,contact_sample,peak_pelvis_yaw_deg,peak_torso_yaw_deg,peak_twist_deg,end_pelvis_yaw_deg,end_torso_yaw_deg,peak_roll_deg,peak_pitch_deg,max_base_xy_drift_m,posture_samples,torso_over30_samples,torso_over45_samples,clipped_steps,control_steps\n"<<std::setprecision(12);
  for(size_t i=0;i<rows.size();i++){const auto&r=rows[i];a<<i+1<<','<<r.planned_sample<<','<<r.contact_sample;for(double x:{r.peak_pelvis_yaw,r.peak_torso_yaw,r.peak_twist,r.end_pelvis_yaw,r.end_torso_yaw,r.peak_roll,r.peak_pitch})a<<','<<x*180/M_PI;a<<','<<r.base_xy_drift<<','<<r.posture_samples<<','<<r.torso_over30<<','<<r.torso_over45<<','<<r.clipped_steps<<','<<r.control_steps<<'\n';}
  std::ofstream warnings(shared.out+"/env-"+std::to_string(id)+"-warnings.csv");warnings<<"warning_id,number,lastinfo\n";for(int i=0;i<mjNWARNING;i++)warnings<<i<<','<<d->warning[i].number<<','<<d->warning[i].lastinfo<<'\n';
 }
};
static void execute(const mjModel*m,mjData*d,int id,void*arg) {
 auto&s=*static_cast<Shared*>(arg);try{Environment e(m,d,id,s);e.run();s.completed++;}catch(const std::exception&e){std::ofstream(s.out+"/env-"+std::to_string(id)+"-error.txt")<<e.what();s.exceptions++;}
}
extern "C" int batch_run(const mjModel*m,const double*q,const double*v,const double*command,const int*bindings,const double*cases,int ncase,int nenv,int nthread,const char*p0,const char*p1,const char*out,const char*return_yaml,const char*ball_yaml,const char*packet_yaml,const char*planner_yaml) {
 try{Shared s;s.initial_q=q;s.initial_v=v;s.initial_command=command;s.bindings=bindings;s.cases=cases;s.ncase=ncase;s.nenv=nenv;s.nthread=nthread;s.out=out;s.return_yaml=return_yaml;s.ball_yaml=ball_yaml;s.packet_yaml=packet_yaml;s.planner_yaml=planner_yaml;std::vector<std::string>groups;std::istringstream in(p0);std::string line;while(std::getline(in,line))groups.push_back(line);for(int i=0;i<nenv;i++)s.policies.push_back(groups.size()>1?groups.at(i/6):(i%12<6?p0:p1));
 mjBatchConfig cfg;mjb_defaultConfig(&cfg);cfg.nthread=nthread;cfg.persistent=1;char error[1024];mjBatch*b=mjb_makeBatch(m,nenv,&cfg,error,sizeof(error));if(!b)throw std::runtime_error(error);int failed=mjb_apply(b,nullptr,nenv,execute,&s,1);mjb_deleteBatch(b);return failed+s.exceptions.load();
 }catch(const std::exception&e){std::fprintf(stderr,"BATCH_ERROR %s\n",e.what());return -1;}
}
