#include <gtest/gtest.h>

#include "a3_pingpong/pp_joint_limits.hpp"
#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_runtime_contract.hpp"

namespace a3_pingpong {
namespace {

TEST(PpRuntimeContract, AcceptsLegacyAndRuntimeV1Models) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("", ""),
            HitterPureRuntimeContract::kLegacy);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v9"),
            HitterPureRuntimeContract::kRallyFinalV1);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_final_v3"),
            HitterPureRuntimeContract::kRallyFinalV1);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedRuntimeV2AndComponentRecipes) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v10"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v11"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v12"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v13"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v14"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v17"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v9"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v10"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v11"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v12"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v13"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v14"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v17"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("", "rally_v10"),
               std::runtime_error);
}

TEST(PpRuntimeContract, RejectsUnknownRuntimeVersion) {
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v3", "rally_v11"),
               std::runtime_error);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedV17FixedStationRuntime) {
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v17"),
      HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1);
  EXPECT_THROW(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v14"),
      std::runtime_error);
  EXPECT_THROW(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v15"),
      std::runtime_error);
}

TEST(PpRuntimeContract, PhysicalHardLimitUsesStrictExportedTolerance) {
  constexpr double lo = -0.5;
  constexpr double hi = 0.4;
  constexpr double tolerance = 0.002;
  EXPECT_FALSE(exceeds_joint_hard_limit(lo - tolerance, lo, hi, tolerance));
  EXPECT_FALSE(exceeds_joint_hard_limit(hi + tolerance, lo, hi, tolerance));
  EXPECT_TRUE(
      exceeds_joint_hard_limit(lo - tolerance - 1e-9, lo, hi, tolerance));
  EXPECT_TRUE(
      exceeds_joint_hard_limit(hi + tolerance + 1e-9, lo, hi, tolerance));
}

TEST(PpRuntimeContract, ExportedTelemetryModeDoesNotTurnMeasuredQIntoFault) {
  constexpr double lo = -0.5;
  constexpr double hi = 0.4;
  constexpr double tolerance = 0.002;
  EXPECT_EQ(classify_actual_q_hard_limit(0.0, lo, hi, tolerance, true),
            ActualQHardLimitDisposition::kInside);
  EXPECT_EQ(
      classify_actual_q_hard_limit(hi + tolerance + 1e-6, lo, hi,
                                   tolerance, true),
      ActualQHardLimitDisposition::kTelemetry);
  EXPECT_EQ(
      classify_actual_q_hard_limit(hi + tolerance + 1e-6, lo, hi,
                                   tolerance, false),
      ActualQHardLimitDisposition::kFault);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedV15RuntimeAndRecipe) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_v15", "rally_v15"),
            HitterPureRuntimeContract::kRallyV15);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_v15", "rally_v14"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v15"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("", "rally_v15"),
               std::runtime_error);
}

TEST(PpObsBuilder, V15KeepsExact110PrefixAndAppendsDeployableLocalization) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);
  state.localization_age = 0.6;

  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;

  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);

  const Eigen::VectorXd obs110 =
      build_obs_110(state, target, last_action, default_q);
  const Eigen::VectorXd obs113 =
      build_obs_113(state, target, last_action, default_q);

  ASSERT_EQ(obs110.size(), kObsDim110);
  ASSERT_EQ(obs113.size(), kObsDim113);
  EXPECT_TRUE(obs113.head(kObsDim110).isApprox(obs110, 0.0));
  EXPECT_DOUBLE_EQ(obs113[kObsDim110], state.base_velocity_xy_w[0]);
  EXPECT_DOUBLE_EQ(obs113[kObsDim110 + 1], state.base_velocity_xy_w[1]);
  EXPECT_DOUBLE_EQ(obs113[kObsDim110 + 2], state.localization_age);
}

TEST(PpObsBuilder, V15ClampsLocalizationAgeOnly) {
  PpRobotState state;
  state.base_pos_w.setZero();
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b.setZero();
  state.q = Eigen::VectorXd::Zero(kNumJoints);
  state.qd = Eigen::VectorXd::Zero(kNumJoints);
  state.base_velocity_xy_w = Vec2(-1.2, 2.4);
  state.localization_age = 4.0;

  PpRacketTarget target;
  target.pos_w.setZero();
  target.vel_w.setZero();
  target.base_target_xy.setZero();
  target.time_to_strike = 0.0;

  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd obs = build_obs_113(state, target, zeros, zeros);
  EXPECT_DOUBLE_EQ(obs[kObsDim110], -1.2);
  EXPECT_DOUBLE_EQ(obs[kObsDim110 + 1], 2.4);
  EXPECT_DOUBLE_EQ(obs[kObsDim110 + 2], 1.0);
}

TEST(PpObsBuilder, RewrittenV15KeepsExact113PrefixAndAppendsFiniteGaitCommand) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.0, 0.1, 1.0);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b.setZero();
  state.q = Eigen::VectorXd::Zero(kNumJoints);
  state.qd = Eigen::VectorXd::Zero(kNumJoints);
  state.base_velocity_xy_w = Vec2(0.0, -0.25);
  state.localization_age = 0.2;

  PpRacketTarget target;
  target.pos_w = Vec3(0.58, -0.3, 1.0);
  target.vel_w = Vec3(2.0, 0.2, 0.8);
  target.base_target_xy = Vec2(0.0, -0.2);
  target.time_to_strike = 0.8;
  target.desired_lateral_velocity = -0.3;
  target.gait_clock = Vec2(0.25, -0.75);
  target.locomotion_mode = 1.0;
  target.upper_intervention = 0.0;

  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd obs113 = build_obs_113(state, target, zeros, zeros);
  const Eigen::VectorXd obs118 = build_obs_118(state, target, zeros, zeros);
  ASSERT_EQ(obs118.size(), kObsDim118);
  EXPECT_TRUE(obs118.head(kObsDim113).isApprox(obs113, 0.0));
  EXPECT_DOUBLE_EQ(obs118[113], -0.3);
  EXPECT_DOUBLE_EQ(obs118[114], 0.25);
  EXPECT_DOUBLE_EQ(obs118[115], -0.75);
  EXPECT_DOUBLE_EQ(obs118[116], 1.0);
  EXPECT_DOUBLE_EQ(obs118[117], 0.0);
}

}  // namespace
}  // namespace a3_pingpong
