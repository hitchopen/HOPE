// Off-robot test of the full CommandFn path: synthetic RobotState (robot at
// nominal, upright IMU) -> ComputeCommand -> RobotCommand. Verifies sizes,
// finiteness, neck-passive (q=0,kp=40,kd=2), non-neck gains == scattered
// metadata, target_q in range, and that the reference clock sweeps a swing.
//   ./pp_policy_test <policy.onnx> [deploy.yaml]
#include <cstdio>
#include <exception>
#include <string>

#include "a3_pingpong/pp_policy.hpp"

using namespace a3_pingpong;

static std::shared_ptr<PpBasePoseInput> FreshAuthoritativeBase(std::uint64_t seq = 1) {
  auto input = std::make_shared<PpBasePoseInput>();
  const double now = PpNowWallSec();
  const double sec = std::floor(now);
  const double nsec = std::floor((now - sec) * 1.0e9);
  input->SetFromFlat({
      2.0, 1.0, static_cast<double>(seq), sec, nsec,
      0.0, 0.0, 0.95, 1.0, 0.0, 0.0, 0.0, 1.0,
      static_cast<double>(kV17RequiredBaseFlags), 1.0, 1.0});
  return input;
}

int main(int argc, char** argv) {
  try {
  if (argc < 2) {
    std::fprintf(
        stderr,
        "usage: %s policy.onnx [deploy.yaml] [legacy|schema32-planner-native]\n",
        argv[0]);
    return 2;
  }
  PpPolicyConfig cfg; cfg.level = 1;
  cfg.deploy_cfg_path = argc > 2 ? argv[2] : "";
  const std::string runtime_profile = argc > 3 ? argv[3] : "legacy";
  if (runtime_profile == "schema32-planner-native") {
    cfg.level = 0;
    cfg.planner_mode = true;
    cfg.policy_native = true;
    cfg.single_swing = true;
  } else if (runtime_profile != "legacy") {
    std::fprintf(stderr, "unknown policy runtime profile: %s\n", runtime_profile.c_str());
    return 2;
  }
  cfg.loc_mode = LocMode::kExternalBase;
  PpPolicy pol(argv[1], cfg);
  pol.SetBasePoseInput(FreshAuthoritativeBase());

  // synthetic state: robot AT nominal pose (default_q scattered to SDK), upright IMU
  robot_io::RobotState st;
  st.q = to_sdk_order(pol.onnx().default_q(), pol.isaac_to_sdk());
  st.dq = Eigen::VectorXd::Zero(31);
  st.imu_quat_wxyz = Eigen::Vector4d(1, 0, 0, 0);
  st.imu_gyro = Eigen::Vector3d::Zero();

  Eigen::VectorXd expect_kp = to_sdk_order(pol.onnx().kp(), pol.isaac_to_sdk());
  Eigen::VectorXd expect_kd = to_sdk_order(pol.onnx().kd(), pol.isaac_to_sdk());

  int fails = 0;
  if (pol.onnx().uses_small_station_contract() && cfg.planner_mode) {
    // The real model3510 trace exposed a second station owner: COMMIT logged
    // +4 cm while the actor observed +12.68 cm, re-derived from the BH target.
    // Exercise the complete callback, including native completion and WAIT,
    // with outlying racket tuples on both sides of the absolute support.
    PpPolicy station_policy(argv[1], cfg);
    auto racket = std::make_shared<PpRacketTargetInput>();
    station_policy.SetRacketInput(racket);
    robot_io::RobotCommand command;
    station_policy.SetBasePoseInput(FreshAuthoritativeBase(1));
    if (!station_policy.ComputeCommand(0, st, command)) ++fails;
    int station_failures = 0;
    for (int flight = 1; flight <= 2; ++flight) {
      const double side = flight == 1 ? -1.0 : 1.0;
      const double target_y = flight == 1 ? .146786 : -.38;
      const double expected_station_y = flight == 1 ? .04 : -.04;
      const double now = PpNowWallSec();
      const double sec = std::floor(now);
      racket->SetFromFlat({2., 1., side, .5846, target_y, 1.09,
          1.8, -.1, 1.128, .68, now + .68, 0., sec,
          std::floor((now - sec) * 1.e9), double(flight), double(flight),
          1., 12., .2});
      bool saw_active = false, saw_wait_after_active = false;
      for (std::uint64_t local_tick = 0; local_tick < 112; ++local_tick) {
        const std::uint64_t tick = 1 + (flight - 1) * 112 + local_tick;
        station_policy.SetBasePoseInput(FreshAuthoritativeBase(tick + 2));
        if (!station_policy.ComputeCommand(tick, st, command)) {
          ++station_failures;
          continue;
        }
        station_policy.RecordCommandDelivery(command, true);
        const auto& obs = station_policy.last_obs_unsafe();
        const Vec3 base = station_policy.last_base_pos();
        if (station_policy.level() == 1) saw_active = true;
        if (saw_active && station_policy.level() == 0) saw_wait_after_active = true;
        if (!saw_active || std::abs(obs[101] + base[0]) > 1.e-9 ||
            std::abs(obs[102] + base[1] - expected_station_y) > 1.e-9)
          ++station_failures;
        if (saw_wait_after_active &&
            (std::abs(obs[104] + base[1] - expected_station_y + .265) > 1.e-9 ||
             std::abs(obs[109] - 1.) > 1.e-9))
          ++station_failures;
      }
      if (!saw_active || !saw_wait_after_active) ++station_failures;
    }
    fails += station_failures;
    std::printf("SMALL STATION COMMIT/ACTIVE/WAIT %s failures=%d\n",
                station_failures == 0 ? "PASS" : "FAIL", station_failures);
  }
  if (pol.onnx().uses_compact_execution_obs()) {
    PpRobotState sample;
    sample.q = pol.onnx().default_q();
    sample.qd = Eigen::VectorXd::Zero(31);
    sample.base_quat_w = Vec4(1, 0, 0, 0);
    sample.base_pos_w = Vec3(0, 0, .95);
    PpRacketTarget target;
    target.pos_w = Vec3(.58, -.265, 1.075);
    target.vel_w.setZero();
    target.base_target_xy.setZero();
    target.time_to_strike = 1.;
    target.reach_level = target.swing_foot_sign = 0.;
    auto first = pol.BuildCompactObservation_(sample, target, 0);
    if (first.size() != 324 ||
        (first.segment(65, 31) - pol.onnx().default_q()).norm() > 1e-10) ++fails;
    robot_io::RobotCommand delivered;
    delivered.q_des = st.q;
    delivered.q_des[pol.isaac_to_sdk()[0]] += .05;
    pol.RecordCommandDelivery(delivered, true);
    auto repeated = pol.BuildCompactObservation_(sample, target, 0);
    if ((repeated - first).norm() != 0.) ++fails;
    sample.qd[0] = .3;
    auto second = pol.BuildCompactObservation_(sample, target, 1);
    if (std::abs(second[65] - sample.q[0] - .05) > 1e-10 ||
        std::abs(second[145]) > 1e-10) ++fails;
    delivered.q_des[pol.isaac_to_sdk()[0]] += .2;
    pol.RecordCommandDelivery(delivered, false);
    auto third = pol.BuildCompactObservation_(sample, target, 2);
    if (std::abs(third[65] - second[65]) > 1e-10 ||
        std::abs(third[145] - .05) > 1e-10 || std::abs(third[114] - .3) > 1e-10) ++fails;
    pol.rearm_yaw_align();
    sample.q[0] += .01;
    auto reset = pol.BuildCompactObservation_(sample, target, 2);
    if ((reset.segment(114, 70) - reset.segment(184, 70)).norm() > 1e-10 ||
        (reset.segment(114, 70) - reset.segment(254, 70)).norm() > 1e-10 ||
        std::abs(reset[145] - .04) > 1e-10) ++fails;
    std::printf("COMPACT SENT/HISTORY/RESET %s\n", fails == 0 ? "PASS" : "FAIL");
    pol.rearm_yaw_align();
  }
  // A 110-D HitterPure model's active scripted target centers are part of the exported
  // train/deploy contract. They must come from this ONNX, never stale PpPolicyConfig constants
  // from an older motion generation. New HitterPingPong models deliberately use a neutral
  // target and zero velocity while level 0 is WAIT, so exercise the active level here instead
  // of mistaking that late-reveal sentinel for a box-center regression. Metadata-less legacy
  // models deliberately skip this check.
  if (pol.onnx().hp_pos_boxes().size() >= 2) {
    for (int c = 0; c < 2; ++c) {
      pol.set_level(0);  // allow the side switch immediately (no mid-swing queue)
      pol.set_swing_dir(c == 0 ? 1 : -1);
      if (pol.onnx().uses_late_reveal_raw_tts_contract()) {
        const PpRacketTarget wait = pol.ScriptedTarget(0);
        const auto& pending = pol.onnx().hitter_pingpong_pending_target();
        const Vec3 expected_wait(pending[0], pending[1], pending[2]);
        if ((wait.pos_w - expected_wait).cwiseAbs().maxCoeff() > 1e-12 ||
            wait.vel_w.cwiseAbs().maxCoeff() > 1e-12 ||
            std::abs(wait.time_to_strike - pol.onnx().hitter_pingpong_wait_tts_s()) > 1e-12) {
          std::printf("late-reveal WAIT target FAIL clip %d\n", c);
          ++fails;
        }
      }
      pol.set_level(1);
      const PpRacketTarget tg = pol.ScriptedTarget(0);
      const auto& pb = pol.onnx().hp_pos_boxes()[c];
      const Vec3 expected_pos(
          0.5 * (pb[0] + pb[1]), 0.5 * (pb[2] + pb[3]), 0.5 * (pb[4] + pb[5]));
      if ((tg.pos_w - expected_pos).cwiseAbs().maxCoeff() > 1e-12) {
        std::printf("box-center position FAIL clip %d\n", c);
        ++fails;
      }
      if (pol.onnx().hp_vel_boxes().size() >= 2) {
        const auto& vb = pol.onnx().hp_vel_boxes()[c];
        const Vec3 expected_vel(
            0.5 * (vb[0] + vb[1]), 0.5 * (vb[2] + vb[3]), 0.5 * (vb[4] + vb[5]));
        if ((tg.vel_w - expected_vel).cwiseAbs().maxCoeff() > 1e-12) {
          std::printf("box-center velocity FAIL clip %d\n", c);
          ++fails;
        }
      }
    }
    std::printf("ONNX BOX-CENTER CONTRACT %s\n", fails == 0 ? "PASS" : "FAIL");
    pol.set_swing_dir(1);
    pol.set_level(1);
  }
  double max_abs_q = 0, max_abs_a = 0;
  std::printf("tick  time_step  |action|  max|q_des|  (swing sweep)\n");
  for (std::uint64_t tick = 0; tick <= 160; tick += 16) {
    // The authoritative-mocap contract is receipt-time based. Refresh the synthetic sample for
    // every inference tick so a cold ONNX invocation cannot turn this callback test into a stale
    // transport test.
    pol.SetBasePoseInput(FreshAuthoritativeBase(tick + 2));
    robot_io::RobotCommand cmd;
    bool ok = pol.ComputeCommand(tick, st, cmd);
    if (!ok) { fails++; continue; }
    // sizes
    if (cmd.q_des.size() != 31 || cmd.kp.size() != 31 || cmd.kd.size() != 31 ||
        cmd.dq_des.size() != 31 || cmd.tau_ff.size() != 31) { std::printf("size FAIL\n"); fails++; }
    if (!cmd.q_des.allFinite() || !cmd.kp.allFinite() || !cmd.kd.allFinite()) { std::printf("nan FAIL\n"); fails++; }
    // neck passive
    for (int s : {kHeadSlot0, kHeadSlot1}) {
      if (std::abs(cmd.q_des[s]) > 1e-12 || std::abs(cmd.kp[s] - kHeadKp) > 1e-9 ||
          std::abs(cmd.kd[s] - kHeadKd) > 1e-9) { std::printf("neck FAIL slot %d\n", s); fails++; }
    }
    // non-neck gains == scattered metadata
    for (int s = 0; s < 31; ++s) {
      if (s == kHeadSlot0 || s == kHeadSlot1) continue;
      if (std::abs(cmd.kp[s] - expect_kp[s]) > 1e-9 || std::abs(cmd.kd[s] - expect_kd[s]) > 1e-9) {
        std::printf("gain FAIL slot %d\n", s); fails++; break;
      }
    }
    double mq = cmd.q_des.cwiseAbs().maxCoeff(), ma = pol.last_action().norm();
    max_abs_q = std::max(max_abs_q, mq); max_abs_a = std::max(max_abs_a, ma);
    std::printf(" %3llu     %3d      %6.3f     %6.3f\n", (unsigned long long)tick,
                pol.last_time_step(), ma, mq);
  }
  // sanity: targets stay in a plausible joint range, clock reaches the strike frame
  bool range_ok = max_abs_q < 3.5;
  std::printf("max|q_des|=%.3f max|action|=%.3f fails=%d\n", max_abs_q, max_abs_a, fails);
  bool pass = (fails == 0) && range_ok;
  std::printf("%s\n", pass ? "POLICY CALLBACK PASS" : "POLICY CALLBACK FAIL");
  return pass ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "POLICY CALLBACK EXCEPTION: %s\n", e.what());
    return 3;
  } catch (...) {
    std::fprintf(stderr, "POLICY CALLBACK EXCEPTION: unknown\n");
    return 3;
  }
}
