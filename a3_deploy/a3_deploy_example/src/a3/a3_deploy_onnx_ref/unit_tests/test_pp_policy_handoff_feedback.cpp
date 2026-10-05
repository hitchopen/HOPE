#include "a3_pingpong/pp_policy.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>

// Opt-in native test with the same exported two-file fixture as Gate3. No ROS,
// simulator, transport or training process is started.
TEST(PpPolicyHandoffFeedback, DeliveredCommandOnlyDuringAffineModeHandoff) {
  const char* directory = std::getenv("GATE3_TEST_POLICY110");
  if (!directory) GTEST_SKIP() << "Set GATE3_TEST_POLICY110 to a V11 affine export";
  const std::filesystem::path path(directory);
  a3_pingpong::PpPolicyConfig cfg;
  cfg.loc_mode = a3_pingpong::LocMode::kExternalBase;
  cfg.deploy_cfg_path = (path / "params/deploy.yaml").string();
  a3_pingpong::PpPolicy policy((path / "exported/policy.onnx").string(), cfg);
  ASSERT_TRUE(policy.onnx().has_v11_affine_safe_qdes_contract());
  robot_io::RobotCommand delivered;
  delivered.q_des = policy.official_stand_q();
  delivered.q_des[2] = .1;  // SDK waist pitch, Isaac action slot 8
  delivered.q_des[3] = .03; // passive head must remain zero in action feedback
  delivered.kp = policy.official_stand_kp();
  delivered.kd = policy.official_stand_kd();
  delivered.dq_des = delivered.tau_ff = Eigen::VectorXd::Zero(31);
  robot_io::RobotState state;
  state.q = policy.official_stand_q();
  state.dq = Eigen::VectorXd::Constant(31, .2);
  state.q[2] = .01;
  policy.RecordCommandDelivery(delivered, true, true, &state);
  const double equivalent_q = policy.last_action()[8] * policy.onnx().action_scale()[8];
  const double nominal_torque = policy.onnx().kp()[8] * (equivalent_q - state.q[2]) -
      policy.onnx().kd()[8] * state.dq[2];
  const double sent_torque = delivered.kp[2] * (delivered.q_des[2] - state.q[2]) -
      delivered.kd[2] * state.dq[2];
  EXPECT_NEAR(nominal_torque, sent_torque, 1e-10);
  EXPECT_DOUBLE_EQ(policy.last_action()[11], 0.);

  const auto feedback = policy.last_action().eval();
  auto rejected = delivered;
  rejected.q_des[2] = -.2;
  policy.RecordCommandDelivery(rejected, false, true, &state);
  EXPECT_TRUE(policy.last_action().isApprox(feedback, 1e-12));

  // Normal policy execution retains the actor's raw feedback, even when the
  // downstream command differs; the correction must end with the handoff.
  policy.RecordCommandDelivery(rejected, true, false);
  EXPECT_TRUE(policy.last_action().isApprox(feedback, 1e-12));
  policy.rearm_yaw_align();
  EXPECT_DOUBLE_EQ(policy.last_action().norm(), 0.);
}
