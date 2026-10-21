// Offline native-ONNX test: no backend, ROS, actuator publisher or robot access.
#include <cassert>
#include <iostream>
#include "a3_pingpong/pp_policy.hpp"

using namespace a3_pingpong;
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  PpPolicyConfig cfg;
  cfg.deploy_cfg_path = argv[2];
  cfg.planner_mode = cfg.policy_native = cfg.single_swing = cfg.shared_session_home = true;
  cfg.loc_mode = LocMode::kExternalBase;
  cfg.level = 0;
  PpPolicy policy(argv[1], cfg);
  auto base = std::make_shared<PpBasePoseInput>();
  auto racket = std::make_shared<PpRacketTargetInput>(policy.expected_planner_wire_schema());
  policy.SetBasePoseInput(base);
  policy.SetRacketInput(racket);
  std::vector<double> published;
  policy.SetSessionHomePublisher([&](const auto& wire) { published = wire; });
  robot_io::RobotState state;
  state.q = to_sdk_order(policy.onnx().default_q(), policy.isaac_to_sdk());
  state.dq = Eigen::VectorXd::Zero(31);
  state.imu_quat_wxyz = Vec4(1, 0, 0, 0);
  state.imu_gyro = Vec3::Zero();
  robot_io::RobotCommand command;
  std::uint64_t sequence = 0;
  auto feed_base = [&](double x) {
    const double now = PpNowWallSec(), sec = std::floor(now);
    base->SetFromFlat({2, 1, double(++sequence), sec, std::floor((now-sec)*1e9),
        x, -.7625, .95, 1, 0, 0, 0, 1, double(kV17RequiredBaseFlags), 7, 8});
  };
  feed_base(-.50);
  assert(policy.ComputeCommand(0, state, command));
  a3_deploy::SessionHomeReceipt first;
  assert(a3_deploy::SessionHomeReceipt::parse(published, first));
  assert(first.x == -.50 && std::abs(first.plane_x - .58) < 1e-6);
  feed_base(-.45);
  assert(policy.ComputeCommand(1, state, command));
  assert(policy.session_home_unsafe().x == -.50);
  assert(policy.session_home_unsafe().epoch == first.epoch);
  std::cout << "PASS native policy HOME is captured from measured base and stays fixed under drift\n";
  policy.rearm_yaw_align();
  feed_base(-.80);
  assert(policy.ComputeCommand(2, state, command));
  const auto second = policy.session_home_unsafe();
  assert(second.epoch > first.epoch && second.x == -.80);
  auto feed_target = [&](std::uint64_t epoch, double x, int revision) {
    const double now = PpNowWallSec(), sec = std::floor(now), nsec = std::floor((now-sec)*1e9);
    const double wire_now = sec + nsec*1e-9;
    racket->SetFromFlat({4,1,1,x,-1.2025,1.10,2,.4,.8,.25,wire_now+.25,0,
        sec,nsec,double(revision),1,double(revision),12,.1,double(epoch)});
  };
  feed_target(first.epoch, -.22, 1); // even correct new geometry cannot rescue an old HOME
  feed_base(-.80);
  assert(policy.ComputeCommand(3, state, command));
  assert(policy.level() == 0 && policy.planner_status() == "session_home_mismatch");
  feed_target(second.epoch, .15, 2); // old hard-coded plane, correct epoch
  feed_base(-.80);
  assert(policy.ComputeCommand(4, state, command));
  assert(policy.level() == 0 && policy.planner_status() == "session_home_mismatch");
  bool engaged = false;
  for (int tick=5; tick<30; ++tick) {
    feed_target(second.epoch, -.22, tick);
    feed_base(-.80);
    assert(policy.ComputeCommand(tick, state, command));
    if (policy.level() == 1) { engaged = true; break; }
  }
  assert(engaged);
  const auto& obs = policy.last_obs_unsafe();
  assert(std::abs(obs[101]) < 1e-6); // no fictitious forward station command
  assert(std::abs(obs[103] - .58) < 1e-6);
  std::cout << "PASS native policy rejects old HOME/wrong plane; matching new HOME executes with dx_station=0, dx_target=.58\n";
  feed_base(-.75); // 5 cm forward drift must remain visible to the actor
  assert(policy.ComputeCommand(30, state, command));
  assert(std::abs(policy.last_obs_unsafe()[101] + .05) < 1e-6);
  assert(std::abs(policy.last_obs_unsafe()[103] - .53) < 1e-6);
  assert(policy.session_home_unsafe().x == -.80);
  std::cout << "PASS forward drift changes actor dx_station=-.05, dx_target=.53 without moving HOME\n";
  policy.rearm_yaw_align();
  feed_base(-.50);
  assert(policy.ComputeCommand(31, state, command));
  const auto dynamic_home = policy.session_home_unsafe();
  assert(dynamic_home.runtime_planes);
  feed_base(-.40);
  assert(policy.ComputeCommand(32, state, command));
  const double now = PpNowWallSec(), sec = std::floor(now), nsec = std::floor((now-sec)*1e9);
  const double wire_now = sec+nsec*1e-9;
  std::vector<double> dynamic{5,1,1,.18,-1.2025,1.10,2,.4,.8,.25,wire_now+.25,0,
      sec,nsec,100,2,1,12,.1,double(dynamic_home.epoch),-.40,1};
  racket->SetFromFlat(dynamic);
  engaged = false;
  for (int tick=33; tick<60; ++tick) {
    feed_base(-.40); assert(policy.ComputeCommand(tick, state, command));
    if (policy.level()==1) {engaged=true;break;}
  }
  assert(engaged);
  assert(std::abs(policy.last_obs_unsafe()[101]) < 1e-6);
  assert(std::abs(policy.last_obs_unsafe()[103]-.58) < 1e-6);
  assert(policy.session_home_unsafe().x == -.50);
  dynamic[3]=.28;dynamic[20]=-.30;dynamic[14]=101;dynamic[16]=2;dynamic[21]=2;
  racket->SetFromFlat(dynamic);
  feed_base(-.40);assert(policy.ComputeCommand(60,state,command));
  assert(std::abs(policy.last_obs_unsafe()[103]-.58) < 1e-6);
  std::cout << "PASS dynamic per-flight plane executes dx_station=0 with unchanged HOME; same-flight mode/plane mutation cannot retarget the actor\n";
}
