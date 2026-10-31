// Offline native ONNX regression: no ROS, hardware backend or actuator publisher.
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "a3_pingpong/pp_policy.hpp"

using namespace a3_pingpong;

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  for (int mode : {0, 1, 2}) {
    PpPolicyConfig cfg;
    cfg.deploy_cfg_path = argv[2];
    cfg.planner_mode = cfg.policy_native = cfg.single_swing = cfg.shared_session_home = true;
    cfg.loc_mode = LocMode::kExternalBase;
    cfg.level = 0;
    PpPolicy policy(argv[1], cfg);
    assert(policy.onnx().is_fixed_home_contract());
    auto base = std::make_shared<PpBasePoseInput>();
    auto racket = std::make_shared<PpRacketTargetInput>(policy.expected_planner_wire_schema());
    policy.SetBasePoseInput(base);
    policy.SetRacketInput(racket);
    std::vector<double> receipt;
    policy.SetSessionHomePublisher([&](const auto& wire) { receipt = wire; });
    robot_io::RobotState state;
    state.q = to_sdk_order(policy.onnx().default_q(), policy.isaac_to_sdk());
    state.dq = Eigen::VectorXd::Zero(31);
    state.imu_quat_wxyz = Vec4(1, 0, 0, 0);
    state.imu_gyro = Vec3::Zero();
    robot_io::RobotCommand command;
    std::uint64_t sequence = 0;
    int tick = 0;
    constexpr double home_x = -.5, home_y = -.7625;
    auto step = [&](double x) {
      const double now = PpNowWallSec(), sec = std::floor(now);
      base->SetFromFlat({2, 1, double(++sequence), sec, std::floor((now-sec)*1e9),
          x, home_y, .95, 1, 0, 0, 0, 1, double(kV17RequiredBaseFlags), 7, 8});
      assert(policy.ComputeCommand(tick++, state, command));
    };
    step(home_x);
    a3_deploy::SessionHomeReceipt home;
    assert(a3_deploy::SessionHomeReceipt::parse(receipt, home));
    assert(home.runtime_planes && receipt.size() == 10 && receipt[0] == 2);
    assert(home.x == home_x && home.y == home_y);
    const auto& box = policy.onnx().hp_pos_boxes()[0];
    const auto& velocity = policy.onnx().hp_vel_boxes()[0];
    const double offset = home.plane_x;
    // Choose a real in-support displacement, distinct from the fixed HOME.
    const double drift = box[1] > offset + .02 ? .02 : -.02;
    assert(offset + drift >= box[0] && offset + drift <= box[1]);
    const double base_x = home_x + drift;
    const double reference = mode == 0 ? home_x : base_x;
    const double target_x = reference + offset;
    std::uint64_t command_sequence = 0;
    auto target = [&](std::uint64_t epoch, double ref, int flight, int revision = 1) {
      const double now = PpNowWallSec(), sec = std::floor(now), ns = std::floor((now-sec)*1e9);
      std::vector<double> wire{mode == 0 ? 4. : 5., 1, 1, ref + offset,
          home_y + (box[2]+box[3])*.5, (box[4]+box[5])*.5,
          (velocity[0]+velocity[1])*.5, (velocity[2]+velocity[3])*.5,
          (velocity[4]+velocity[5])*.5, .25, sec+ns*1e-9+.25, 0,
          sec, ns, double(++command_sequence), double(flight), double(revision), 12, .1, double(epoch)};
      if (mode != 0) wire.insert(wire.end(), {ref, double(mode)});
      racket->SetFromFlat(wire);
    };
    // Capability must not bypass HOME identity or the model's reach envelope.
    target(home.epoch-1, reference, 1); step(base_x);
    assert(policy.level() == 0 && policy.planner_status() == "session_home_mismatch");
    if (mode != 0) {
      target(home.epoch, home_x + box[1] - offset + .10, 2); step(base_x);
      assert(policy.level() == 0 && policy.planner_status() == "fixed_station_target_unsupported");
    }
    target(home.epoch, reference, 3);
    for (int i=0; i<30 && policy.level()!=1; ++i) step(base_x);
    assert(policy.level() == 1);
    const auto& obs = policy.last_obs_unsafe();
    assert(std::abs(obs[101] + drift) < 1e-6); // HOME - measured base, never zeroed
    assert(std::abs(obs[102]) < 1e-6);
    assert(std::abs(obs[103] - (target_x-base_x)) < 1e-6);
    assert(policy.session_home_unsafe().epoch == home.epoch);
    assert(policy.session_home_unsafe().x == home_x);
    // A revision to the admitted flight cannot move its target or station.
    target(home.epoch, reference+.01, 3, 2); step(base_x);
    assert(std::abs(policy.last_obs_unsafe()[103] - (target_x-base_x)) < 1e-6);
    // Measured drift remains visible after admission, including station recovery.
    step(base_x+.005);
    assert(std::abs(policy.last_obs_unsafe()[101] + drift + .005) < 1e-6);
    assert(std::abs(policy.last_obs_unsafe()[103] - (target_x-base_x-.005)) < 1e-6);
    for (int i=0; i<100 && policy.level()!=0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      step(base_x+.005);
    }
    assert(policy.level() == 0);
    step(base_x+.005);
    assert(std::abs(policy.last_obs_unsafe()[101] + drift + .005) < 1e-6);
    assert(policy.session_home_unsafe().epoch == home.epoch);
    assert(policy.session_home_unsafe().x == home_x);
    std::cout << "PASS fixed-HOME mode=" << mode
              << ": dynamic receipt, target accepted, immutable HOME/station, drift observable, invalid targets rejected\n";
  }
}
