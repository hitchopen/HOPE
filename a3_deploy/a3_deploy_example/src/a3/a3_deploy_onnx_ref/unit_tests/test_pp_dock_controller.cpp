#include "a3_pingpong/pp_dock_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

namespace {

a3_pingpong::DockPose Pose(double x, double y, double yaw, double age = 0.01, bool frame_ok = true) {
  a3_pingpong::DockPose pose;
  pose.x = x;
  pose.y = y;
  pose.yaw = yaw;
  pose.age_s = age;
  pose.valid = true;
  pose.frame_ok = frame_ok;
  return pose;
}

a3_pingpong::DockConfig TestConfig() {
  a3_pingpong::DockConfig cfg;
  cfg.yaw_priority_rad = 10.0;
  return cfg;
}

}  // namespace

TEST(PpDockController, BodyFrameSignsAndYawWrap) {
  using namespace a3_pingpong;
  DockConfig cfg = TestConfig();
  cfg.target_x = 1.0;
  cfg.target_y = 0.0;
  cfg.target_yaw = 0.0;
  PpDockController dock(cfg);
  dock.Begin(0.0);
  const auto ahead = dock.Update(0.0, Pose(0.0, 0.0, 0.0), false, false, false);
  EXPECT_GT(ahead.velocity.x(), 0.0);
  EXPECT_NEAR(ahead.velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(ahead.velocity.z(), 0.0, 1e-9);

  DockConfig side = TestConfig();
  side.target_x = 1.0;
  side.target_y = 0.0;
  side.target_yaw = M_PI / 2.0;
  PpDockController turned(side);
  turned.Begin(0.0);
  const auto left = turned.Update(0.0, Pose(0.0, 0.0, M_PI / 2.0), false, false, false);
  EXPECT_NEAR(left.velocity.x(), 0.0, 1e-9);
  EXPECT_LT(left.velocity.y(), 0.0);

  DockConfig spin = TestConfig();
  spin.target_x = -0.50;
  spin.target_y = -0.7625;
  PpDockController near_pi(spin);
  near_pi.Begin(0.0);
  const auto yaw = near_pi.Update(0.0, Pose(-0.50, -0.7625, 179.0 * M_PI / 180.0), false, false, false);
  EXPECT_LT(yaw.velocity.z(), 0.0);
  EXPECT_GT(yaw.error_yaw, -M_PI);
  EXPECT_LT(yaw.error_yaw, -M_PI + 0.05);
  EXPECT_EQ(yaw.result, DockResult::kRunning);
}

TEST(PpDockController, SaturatesAndAppliesMinimumSpeed) {
  using namespace a3_pingpong;
  DockConfig cfg = TestConfig();
  cfg.target_x = 0.0;
  cfg.target_y = 0.0;
  PpDockController far(cfg);
  far.Begin(0.0);
  const auto saturated = far.Update(0.0, Pose(-2.0, 0.0, 0.0), false, false, false);
  EXPECT_DOUBLE_EQ(saturated.velocity.x(), cfg.forward_m_s);

  PpDockController creep(cfg);
  creep.Begin(0.0);
  const auto boosted = creep.Update(0.0, Pose(-0.04, 0.0, 0.0), false, false, false);
  EXPECT_NEAR(boosted.velocity.x(), cfg.min_linear_m_s, 1e-9);
}

TEST(PpDockController, HysteresisVerifyRetryTimeoutAndMocap) {
  using namespace a3_pingpong;
  DockConfig cfg = TestConfig();
  cfg.settle_hold_s = 0.25;
  cfg.max_attempts = 3;
  cfg.timeout_s = 45.0;
  PpDockController dock(cfg);
  const auto target = Pose(cfg.target_x, cfg.target_y, cfg.target_yaw);
  dock.Begin(0.0);
  EXPECT_EQ(dock.Update(0.0, target, false, false, false).phase, DockPhase::kSettle);
  EXPECT_EQ(dock.Update(0.1, Pose(cfg.target_x + 0.04, cfg.target_y, 0.0), false, true, true).phase,
            DockPhase::kSettle);
  EXPECT_EQ(dock.Update(0.2, Pose(cfg.target_x + 0.06, cfg.target_y, 0.0), false, true, true).phase,
            DockPhase::kAlign);

  PpDockController verify(cfg);
  verify.Begin(0.0);
  verify.Update(0.0, target, false, true, true);
  verify.Update(0.0, target, false, true, true);
  const auto checked = verify.Update(0.25, target, false, true, true);
  EXPECT_EQ(checked.phase, DockPhase::kDone);
  EXPECT_EQ(checked.result, DockResult::kDone);

  PpDockController retries(cfg);
  retries.Begin(0.0);
  EXPECT_EQ(retries.Update(0.0, target, true, true, true).phase, DockPhase::kEntry);
  const auto drifted = Pose(cfg.target_x + 0.04, cfg.target_y, 0.0);
  auto FailOnce = [&](double settle_time) {
    retries.Update(settle_time, target, false, true, true);
    retries.Update(settle_time, drifted, false, true, true);
    return retries.Update(settle_time + 0.25, drifted, false, true, true);
  };
  auto again = FailOnce(0.25);
  EXPECT_EQ(again.phase, DockPhase::kAlign);
  EXPECT_EQ(again.attempt, 1);
  again = FailOnce(1.0);
  EXPECT_EQ(again.attempt, 2);
  again = FailOnce(2.0);
  EXPECT_EQ(again.phase, DockPhase::kFailed);
  EXPECT_EQ(again.reason, DockReason::kAttemptsExhausted);

  PpDockController expired(cfg);
  expired.Begin(0.0);
  const auto timeout = expired.Update(45.01, target, false, false, false);
  EXPECT_EQ(timeout.reason, DockReason::kTimeout);

  PpDockController stale(cfg);
  stale.Begin(0.0);
  EXPECT_EQ(stale.Update(0.0, Pose(cfg.target_x, cfg.target_y, 0.0, 0.2), false, false, false).reason,
            DockReason::kMocapStale);
  PpDockController blind(cfg);
  blind.Begin(0.0);
  DockPose invalid = target;
  invalid.valid = false;
  EXPECT_EQ(blind.Update(0.0, invalid, false, false, false).reason, DockReason::kMocapInvalid);
  PpDockController frame(cfg);
  frame.Begin(0.0);
  EXPECT_EQ(frame.Update(0.0, Pose(cfg.target_x, cfg.target_y, 0.0, 0.01, false), false, false, false).reason,
            DockReason::kWrongFrame);
}

TEST(PpDockController, AcceptsTwoMetreFullTurnAndRejectsPastThreeMetres) {
  using namespace a3_pingpong;
  DockConfig cfg = TestConfig();
  PpDockController accepted(cfg);
  accepted.Begin(0.0);
  const auto far = accepted.Update(0.0, Pose(cfg.target_x - 2.0, cfg.target_y, M_PI), false, false, false);
  EXPECT_EQ(far.result, DockResult::kRunning);
  EXPECT_NEAR(far.distance, 2.0, 1e-9);
  EXPECT_NE(far.velocity.z(), 0.0);

  PpDockController rejected(cfg);
  rejected.Begin(0.0);
  const auto too_far = rejected.Update(0.0, Pose(cfg.target_x - 3.1, cfg.target_y, 0.0), false, false, false);
  EXPECT_EQ(too_far.phase, DockPhase::kFailed);
  EXPECT_EQ(too_far.reason, DockReason::kStartTooFar);
  EXPECT_TRUE(too_far.velocity.isZero());
}

TEST(PpDockController, StopAndYamlDefaults) {
  using namespace a3_pingpong;
  PpDockController dock(TestConfig());
  dock.Begin(0.0);
  dock.RequestStop();
  const auto stopped = dock.Update(0.1, Pose(0.0, 0.0, 0.0), false, false, false);
  EXPECT_EQ(stopped.reason, DockReason::kStopped);
  EXPECT_TRUE(stopped.velocity.isZero());

  const auto path = std::filesystem::path(__FILE__).parent_path() / "../../../../assets/a3_runtime/dock/dock.yaml";
  const DockConfig loaded = LoadDockConfig(path.lexically_normal().string());
  EXPECT_DOUBLE_EQ(loaded.target_x, -0.50);
  EXPECT_DOUBLE_EQ(loaded.target_y, -0.7625);
  EXPECT_DOUBLE_EQ(loaded.target_yaw, 0.0);
  EXPECT_DOUBLE_EQ(loaded.timeout_s, 45.0);
  EXPECT_DOUBLE_EQ(loaded.max_start_distance_m, 3.0);
  EXPECT_EQ(loaded.max_attempts, 5);
  EXPECT_GT(loaded.approach_distance_m, 0.0);
  EXPECT_LT(loaded.position_tolerance_m, loaded.reopen_position_m);
}

TEST(PpDockController, SteersTowardTheTargetBeforeTheFinalYaw) {
  using namespace a3_pingpong;
  DockConfig cfg = TestConfig();
  cfg.yaw_priority_rad = 0.60;
  cfg.approach_distance_m = 0.45;
  cfg.target_x = -0.50;
  cfg.target_y = -0.7625;
  cfg.target_yaw = 0.0;
  PpDockController far(cfg);
  far.Begin(0.0);
  const auto backed = far.Update(0.0, Pose(0.10, cfg.target_y, 1.2), false, false, false);
  EXPECT_GT(backed.velocity.z(), 0.0);
  EXPECT_NEAR(backed.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(backed.velocity.y(), 0.0, 1e-9);

  PpDockController near(cfg);
  near.Begin(0.0);
  const auto spin = near.Update(0.0, Pose(cfg.target_x - 0.20, cfg.target_y, M_PI), false, false, false);
  EXPECT_NE(spin.velocity.z(), 0.0);
  EXPECT_NEAR(spin.velocity.x(), 0.0, 1e-9);
}
