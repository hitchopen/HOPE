// In-memory production Runner policy probe. No HAL, DDS, or robot connection.
#include "a3_pingpong/pp_policy.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
  if (argc != 5 && argc != 6) return 2;  // ONNX deploy.yaml lifecycle-case schema [enabled]
  try {
    const int scenario = std::stoi(argv[3]), schema = std::stoi(argv[4]);
    const bool pending_case = scenario == 2 || scenario == 4;
    const bool revocation_enabled = argc == 6 && std::string(argv[5]) == "enabled";
    a3_pingpong::PpPolicyConfig config;
    config.deploy_cfg_path = argv[2]; config.shared_session_home = true;
#ifndef HOPE_RUNNER_BASELINE
    config.planner_preengagement_revocation = revocation_enabled;
#endif
    config.planner_mode = true; config.policy_native = true;
    config.single_swing = true; config.level = 0; config.engage_settle_s = 0;
    config.loc_mode = a3_pingpong::LocMode::kExternalBase;
    a3_pingpong::PpPolicy policy(argv[1], config);
    auto base = std::make_shared<a3_pingpong::PpBasePoseInput>();
#ifdef HOPE_RUNNER_BASELINE
    auto input = std::make_shared<a3_pingpong::PpRacketTargetInput>(4);
#else
    auto input = std::make_shared<a3_pingpong::PpRacketTargetInput>(4, revocation_enabled);
#endif
    policy.SetBasePoseInput(base); policy.SetRacketInput(input);
    robot_io::RobotState state;
    state.q = policy.official_stand_q(); state.dq = Eigen::VectorXd::Zero(31);
    state.imu_quat_wxyz = Eigen::Vector4d(1,0,0,0); state.imu_gyro.setZero();
    std::uint64_t tick = 0;
    auto step = [&] {
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      base->SetFromFlat({2,1,double(tick+1),double(stamp/1000000000LL),double(stamp%1000000000LL),
          -.5,-.7625,.95,1,0,0,0,1,47,7,8});
      robot_io::RobotCommand command;
      if (!policy.ComputeCommand(tick++, state, command)) throw std::runtime_error("policy step failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    };
    for (int i=0;i<10;++i) step();
    const auto home = policy.session_home_unsafe();
    if (!home.epoch) throw std::runtime_error("no production HOME receipt");
    auto wire = [&](bool valid, unsigned sequence, unsigned revision) {
      const double now = a3_pingpong::PpNowWallSec();
      const double lead = pending_case ? 1.2 : .5;
      std::vector<double> packet={double(schema),valid ? 1. : 0.,1,
          home.x+home.plane_x,home.y-.44,1.05,1.92,.19,1.03,
          valid ? lead : 0.,valid ? now+lead : 0.,0,
          std::floor(now),std::floor((now-std::floor(now))*1e9),
          double(sequence),1,double(revision),30,.1,double(home.epoch)};
      if (schema==5) { packet.push_back(home.x); packet.push_back(1); }
      return packet;
    };
    if (scenario != 1) input->SetFromFlat(wire(true,1,1));
    const bool retained_before = input->Latest().has_valid;
    bool policy_observed_before = false;
    if (pending_case) {
      for (int i=0;i<5;++i) step();
      policy_observed_before = policy.planner_trace_snapshot(tick).planner_flight_id == 1;
      if (policy.level() != 0 || !policy_observed_before)
        throw std::runtime_error("fixture did not exercise an observed unengaged target");
    }
    if (scenario == 4) {
      auto unrelated = wire(false,3,2); unrelated[15] = 2;
      input->SetFromFlat(unrelated);
    }
    if (scenario != 3) input->SetFromFlat(wire(false,2,2));
    bool engaged = false, active_after = false;
    std::string status;
    for (int i=0;i<65;++i) {
      step(); status=policy.planner_trace_snapshot(tick).status;
      if (policy.level()==1) {
        engaged=true;
        if (scenario==3) { input->SetFromFlat(wire(false,2,2)); step(); active_after=policy.level()==1; }
        break;
      }
    }
    std::cout << "{\"case\":" << scenario << ",\"schema\":" << schema
        << ",\"revocation_enabled\":" << (revocation_enabled ? "true" : "false")
        << ",\"retained_before\":" << (retained_before ? "true" : "false")
        << ",\"policy_observed_before\":" << (policy_observed_before ? "true" : "false")
        << ",\"invalid_after\":" << (input->Latest().invalid_after ? "true" : "false")
        << ",\"engaged\":" << (engaged ? "true" : "false")
        << ",\"active_after_revocation\":" << (active_after ? "true" : "false")
        << ",\"status\":\"" << status << "\"}\n";
    return (scenario==1 || pending_case ? !engaged : engaged && active_after) ? 0 : 1;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
