#include "a3_pingpong/pp_policy.hpp"
#include "a3_pingpong/pp_command_transition.hpp"
#include "mujoco/mujoco.h"
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace a3_pingpong;
int main(int argc,char** argv) {
  if(argc!=6) return 2;
  const std::string dir=argv[1], mode=argv[3]; const double duration=std::stod(argv[4]);
  PpPolicyConfig cfg;cfg.deploy_cfg_path=dir+"/params/deploy.yaml";cfg.loc_mode=LocMode::kExternalBase;
  cfg.policy_native=true;cfg.planner_mode=true;cfg.level=0;
  PpPolicy p(dir+"/exported/policy.onnx",cfg);
  auto base=std::make_shared<PpBasePoseInput>();p.SetBasePoseInput(base);
  char error[2048];auto* m=mj_loadXML(argv[2],nullptr,error,sizeof(error));
  if(!m) {std::cerr<<error;return 3;}auto* d=mj_makeData(m);
  mj_resetDataKeyframe(m,d,mj_name2id(m,mjOBJ_KEY,"stand"));
  d->qpos[0]=-.5;d->qpos[1]=-.7625;
  int q[31],v[31],act[31];
  for(int i=0;i<31;++i) {
    int sdk=p.isaac_to_sdk()[i];const auto name=p.onnx().joint_names()[i];int j=mj_name2id(m,mjOBJ_JOINT,name.c_str());
    q[sdk]=m->jnt_qposadr[j];v[sdk]=m->jnt_dofadr[j];act[sdk]=mj_name2id(m,mjOBJ_ACTUATOR,(name+"_motor").c_str());
  }
  robot_io::RobotCommand stand;stand.q_des=p.official_stand_q();stand.kp=p.official_stand_kp();stand.kd=p.official_stand_kd();stand.dq_des=stand.tau_ff=Eigen::VectorXd::Zero(31);
  auto step=[&](const robot_io::RobotCommand& c) {
    for(int i=0;i<31;++i) {double tau=c.kp[i]*(c.q_des[i]-d->qpos[q[i]])+c.kd[i]*(c.dq_des[i]-d->qvel[v[i]])+c.tau_ff[i];
      d->ctrl[act[i]]=std::clamp(tau,m->actuator_ctrlrange[2*act[i]],m->actuator_ctrlrange[2*act[i]+1]);}
    mj_step(m,d);
  };
  // Direct standing reset, then an identical 1 s PD settle for every variant.
  for(int i=0;i<1000;++i) step(stand);
  {
    std::ofstream initial(std::string(argv[5])+".initial",std::ios::binary);
    initial.write(reinterpret_cast<const char*>(d->qpos),m->nq*sizeof(mjtNum));
    initial.write(reinterpret_cast<const char*>(d->qvel),m->nv*sizeof(mjtNum));
  }
  const int pelvis=mj_name2id(m,mjOBJ_BODY,"pelvis_link"),torso=mj_name2id(m,mjOBJ_BODY,"torso_Link");
  PpCommandTransition transition;p.rearm_yaw_align();
  std::ofstream out(argv[5]);out<<std::setprecision(17)<<"t,torso_pitch_deg,waist_q,waist_qdes,base_z,kp,action8,feedback8\n";
  double peak=-1e9,peak_abs=0,minz=10;
  for(int tick=0;tick<400;++tick) {
    robot_io::RobotState s;s.q.resize(31);s.dq.resize(31);
    for(int i=0;i<31;++i){s.q[i]=d->qpos[q[i]];s.dq[i]=d->qvel[v[i]];}
    for(int i=0;i<4;++i){s.imu_quat_wxyz[i]=d->xquat[4*pelvis+i];s.sec_imu_quat_wxyz[i]=d->xquat[4*torso+i];}
    s.has_secondary_imu=true;mjtNum vel[6];mj_objectVelocity(m,d,mjOBJ_BODY,pelvis,vel,1);
    for(int i=0;i<3;++i)s.imu_gyro[i]=vel[i];mj_objectVelocity(m,d,mjOBJ_BODY,torso,vel,1);
    for(int i=0;i<3;++i)s.sec_imu_gyro[i]=vel[i];
    auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    base->SetFromFlat({2,1,double(tick+1),double(ns/1000000000),double(ns%1000000000),d->xpos[3*pelvis],d->xpos[3*pelvis+1],d->xpos[3*pelvis+2],s.imu_quat_wxyz[0],s.imu_quat_wxyz[1],s.imu_quat_wxyz[2],s.imu_quat_wxyz[3],1,63,1,1});
    p.set_mode_handoff_hold(tick*.02<duration);
    robot_io::RobotCommand cmd;p.ComputeCommand(tick,s,cmd);double raw=p.last_action()[8];
    if(tick==0){if(mode=="legacy")transition.Begin(stand,cmd,duration);else transition.BeginImpedance(stand,cmd,duration);}
    transition.Apply(tick*.02,cmd);
    p.RecordCommandDelivery(cmd,true,mode=="torque" && transition.active(), &s);
    for(int j=0;j<20;++j) step(cmd);
    const auto* quat=d->xquat+4*torso;double pitch=std::asin(std::clamp(2*(quat[0]*quat[2]-quat[3]*quat[1]),-1.,1.))*180/M_PI;
    peak=std::max(peak,pitch);peak_abs=std::max(peak_abs,std::abs(pitch));minz=std::min(minz,d->xpos[3*pelvis+2]);
    out<<tick*.02<<','<<pitch<<','<<d->qpos[q[2]]<<','<<cmd.q_des[2]<<','<<d->xpos[3*pelvis+2]<<','<<cmd.kp[2]<<','<<raw<<','<<p.last_action()[8]<<'\n';
  }
  std::cout<<"RESULT "<<mode<<" duration="<<duration<<" peak="<<peak<<" minz="<<minz<<'\n';
  mj_deleteData(d);mj_deleteModel(m);
  return mode=="torque" && (peak_abs>5 || minz<.95) ? 1 : 0;
}
