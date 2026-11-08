#include <cassert>
#include <cmath>
#include <iostream>
#include "a3_pingpong/pp_policy.hpp"
#include "hope_planner_cpp/swing_side_support.hpp"
using namespace a3_pingpong;
int main(int argc, char** argv) {
  assert(argc==3);
  for(bool live : {false,true}) for(int case_id : {0,1}) {
    PpPolicyConfig cfg;
    cfg.deploy_cfg_path=argv[2];
    cfg.planner_mode=cfg.policy_native=cfg.single_swing=cfg.shared_session_home=true;
    cfg.target_support_live_base=live;cfg.loc_mode=LocMode::kExternalBase;cfg.level=0;
    PpPolicy policy(argv[1],cfg);
    auto base=std::make_shared<PpBasePoseInput>();auto racket=std::make_shared<PpRacketTargetInput>(policy.expected_planner_wire_schema());
    policy.SetBasePoseInput(base);policy.SetRacketInput(racket);
    std::vector<double> receipt;policy.SetSessionHomePublisher([&](const auto& w){receipt=w;});
    robot_io::RobotState state;state.q=to_sdk_order(policy.onnx().default_q(),policy.isaac_to_sdk());state.dq=Eigen::VectorXd::Zero(31);state.imu_quat_wxyz=Vec4(1,0,0,0);state.imu_gyro=Vec3::Zero();
    robot_io::RobotCommand command;int tick=0;std::uint64_t sequence=0;
    const double hx=-.5,hy=-.76;
    auto step=[&](double by){const double now=PpNowWallSec(),sec=std::floor(now);base->SetFromFlat({2,1,double(++sequence),sec,std::floor((now-sec)*1e9),hx,by,.95,1,0,0,0,1,double(kV17RequiredBaseFlags),7,8});assert(policy.ComputeCommand(tick++,state,command));};
    step(hy);a3_deploy::SessionHomeReceipt home;assert(a3_deploy::SessionHomeReceipt::parse(receipt,home));
    const double drift=case_id==0?.15:-.15;
    const double target_y=hy+(case_id==0?.35:.20);
    const auto& box=policy.onnx().hp_pos_boxes()[1];const auto& vel=policy.onnx().hp_vel_boxes()[1];
    const double reference=live?hy+drift:hy;
    const double sign=hope_planner_cpp::select_swing_sign_with_support(target_y,reference,-.24,.04,0,{- .68,-.24},{-.24,.28});
    const bool expected=case_id==0?live:!live;
    assert(expected ? sign==-1. : !std::isfinite(sign));
    step(hy+drift);
    const double now=PpNowWallSec(),sec=std::floor(now),ns=std::floor((now-sec)*1e9);
    // Also submit the refused candidate to Runner, proving that a permissive
    // upstream node cannot bypass the corresponding Runner support decision.
    racket->SetFromFlat({4,1,-1,hx+home.plane_x,target_y,(box[4]+box[5])*.5,(vel[0]+vel[1])*.5,(vel[2]+vel[3])*.5,(vel[4]+vel[5])*.5,.35,sec+ns*1e-9+.35,0,sec,ns,1,1,1,12,.1,double(home.epoch)});
    for(int i=0;i<30&&policy.level()!=1;++i)step(hy+drift);
    assert((policy.level()==1)==expected);
    if(!expected)assert(policy.planner_status()==(live?"live_base_target_unsupported":"fixed_station_target_unsupported"));
    assert(policy.session_home_unsafe().epoch==home.epoch);
    assert(policy.session_home_unsafe().y==hy);
    const auto& obs=policy.last_obs_unsafe();assert(std::abs(obs[102]+drift)<1e-6);
    if(expected)assert(std::abs(obs[104]-(target_y-hy-drift))<1e-6);
    std::cout<<"PASS live="<<live<<" case="<<case_id<<" accepted="<<expected<<" station unchanged\n";
  }
}
