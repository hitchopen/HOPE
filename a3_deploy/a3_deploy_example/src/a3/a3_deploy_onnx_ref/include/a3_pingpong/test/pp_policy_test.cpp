// Off-robot test of the full CommandFn path: synthetic RobotState (robot at
// nominal, upright IMU) -> ComputeCommand -> RobotCommand. Verifies sizes,
// finiteness, neck-passive (q=0,kp=40,kd=2), non-neck gains == scattered
// metadata, target_q in range, and that the reference clock sweeps a swing.
//   ./pp_policy_test <policy.onnx> [deploy.yaml]
#include <cstdio>
#include <exception>

#include "a3_pingpong/pp_policy.hpp"

using namespace a3_pingpong;

static std::shared_ptr<PpBasePoseInput> FreshAuthoritativeBase() {
  auto input = std::make_shared<PpBasePoseInput>();
  const double now = PpNowWallSec();
  const double sec = std::floor(now);
  const double nsec = std::floor((now - sec) * 1.0e9);
  input->SetFromFlat({
      2.0, 1.0, 1.0, sec, nsec,
      0.0, 0.0, 0.95, 1.0, 0.0, 0.0, 0.0, 1.0,
      static_cast<double>(kV17RequiredBaseFlags), 1.0, 1.0});
  return input;
}

int main(int argc, char** argv) {
  try {
  if (argc < 2) { std::fprintf(stderr, "usage: %s policy.onnx [deploy.yaml]\n", argv[0]); return 2; }
  PpPolicyConfig cfg; cfg.level = 1;
  cfg.deploy_cfg_path = argc > 2 ? argv[2] : "";
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
  // A 110-D HitterPure model's scripted/idle target centers are part of the exported
  // train/deploy contract. They must come from this ONNX, never stale PpPolicyConfig constants
  // from an older motion generation. Metadata-less legacy models deliberately skip this check.
  if (pol.onnx().hp_pos_boxes().size() >= 2) {
    pol.set_level(0);  // allow the side switch immediately (no mid-swing queue)
    for (int c = 0; c < 2; ++c) {
      pol.set_swing_dir(c == 0 ? 1 : -1);
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
