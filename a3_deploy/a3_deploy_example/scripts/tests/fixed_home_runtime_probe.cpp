// Exercises production observation and HOME receipt paths without transport or HAL.
#include "a3_pingpong/pp_policy.hpp"
#include <iostream>
#include <stdexcept>
using namespace a3_pingpong;
void require(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
int main(int argc, char** argv) {
  try {
    require(argc == 2, "policy directory required");
    std::string root=argv[1];
    PpPolicyConfig cfg; cfg.planner_mode=true; cfg.policy_native=true;
    cfg.single_swing=true; cfg.level=0; cfg.shared_session_home=true;
    cfg.loc_mode=LocMode::kExternalBase; cfg.deploy_cfg_path=root+"/params/deploy.yaml";
    PpPolicy p(root+"/exported/policy.onnx", cfg);
    auto base=std::make_shared<PpBasePoseInput>();
    p.SetBasePoseInput(base);p.SetRacketInput(std::make_shared<PpRacketTargetInput>());
    p.SetSessionHomePublisher([](const std::vector<double>&){});
    robot_io::RobotState st;st.q=p.official_stand_q();st.dq=Eigen::VectorXd::Zero(31);
    st.imu_quat_wxyz=Eigen::Vector4d(1,0,0,0);st.imu_gyro=Eigen::Vector3d::Zero();
    auto step=[&](int tick,double x,double y) {
      double now=PpNowWallSec(),sec=std::floor(now);
      base->SetFromFlat({2,1,double(tick+1),sec,std::floor((now-sec)*1e9),x,y,1.,1,0,0,0,1,double(kV17RequiredBaseFlags),1,1});
      robot_io::RobotCommand command;
      require(p.ComputeCommand(tick,st,command),"command failed");
      require(command.q_des.allFinite(),"nonfinite command");
    };
    p.rearm_yaw_align();step(0,-.5,-.7625);
    auto home=p.session_home_unsafe();
    require(home.epoch!=0,"HOME absent");require(!home.runtime_planes,"fixed HOME advertised moving planes");
    require(std::abs(home.plane_x-.57)<1e-9,"wrong selected plane");
    step(1,-.47,-.7225);
    require(p.session_home_unsafe().epoch==home.epoch,"drift redefined HOME epoch");
    const auto& obs=p.last_obs_unsafe();
    require(obs.size()==110,"wrong observation size");
    require(std::abs(obs[101]+.03)<1e-6 && std::abs(obs[102]+.04)<1e-6,"actor lost immutable HOME during drift");
    require(obs[65+11]==0. && obs[65+16]==0.,"passive head action fed back");
    p.rearm_yaw_align();step(2,-.4,-.6);
    require(p.session_home_unsafe().epoch!=home.epoch,"explicit reset did not renew HOME");
    std::cout<<"FIXED_HOME_RUNTIME_PASS\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
