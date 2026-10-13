#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/adaptive_flight_commit.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace hope_planner_cpp {
namespace {
CommitPredictionConfig physics() {
  CommitPredictionConfig p;
  p.observed_physics.drag_k = 0.0;
  p.observed_physics.table_tangential_gain = 0.0;
  p.observed_physics.restitution_v = 0.9;
  p.future_physics = p.observed_physics;
  p.planner.adaptive_predict_horizon = true;
  return p;
}
IncomingTrajectoryConfig config() {
  IncomingTrajectoryConfig c;
  c.adaptive.enabled = true;
  return c;
}
BallSample incoming(double t, double speed = 1.5, double receipt_age = 0.0) {
  BallSample s;
  s.source_time_s = 100.0 + t;
  s.source_time_ns = std::llround(s.source_time_s * 1e9);
  s.receipt_wall_ns = std::llround((s.source_time_s + receipt_age) * 1e9);
  const double tb = (-3.0 + std::sqrt(9.0 + 2 * 9.81 * 0.78)) / 9.81;
  double z = 0.8 - 3.0 * t - 4.905 * t * t;
  if (t > tb) {
    const double u = t - tb;
    z = 0.02 + 0.9 * (3.0 + 9.81 * tb) * u - 4.905 * u * u;
  }
  s.position = Vec3(1.37 - speed * t, -0.6, z);
  return s;
}
TEST(AdaptiveFlightCommit, FastFlightCommitsBeforeFixed50msExactlyOnce) {
  IncomingTrajectory tr(config(), physics());
  int count = 0;
  TrajectorySnapshot snapshot;
  for (int i = -90; i < 100; ++i) {
    auto u = tr.observe(incoming(i / 360.0, 4.0));
    if (u.snapshot_ready) { ++count; snapshot = u.snapshot; }
  }
  ASSERT_EQ(count, 1);
  EXPECT_EQ(snapshot.commit_reason, "short_lead_early");
  EXPECT_GE(snapshot.one_shot.commit_source_time_s, 100.0);
  EXPECT_LT(snapshot.one_shot.commit_source_time_s, 100.05);
  EXPECT_EQ(tr.phase(), IncomingPhase::kWaitOutgoing);
  EXPECT_FALSE(tr.poll(102.0).snapshot_ready);
}
TEST(AdaptiveFlightCommit, SlowFlightUsesOnlyCleanPostBounceSamples) {
  IncomingTrajectory tr(config(), physics());
  TrajectorySnapshot snapshot;
  for (int i = -90; i < 180; ++i) {
    auto u = tr.observe(incoming(i / 360.0));
    if (u.snapshot_ready) snapshot = u.snapshot;
  }
  ASSERT_GT(snapshot.sample_count, 11U);
  EXPECT_EQ(snapshot.commit_reason, "observed_post_bounce");
  EXPECT_GT(snapshot.one_shot.commit_source_time_s, 100.20);
  EXPECT_LT(snapshot.one_shot.commit_source_time_s, 100.40);
  EXPECT_GT(snapshot.samples[0].source_time_s, 100.20);
  BatchPhysicsEstimator estimator(physics().observed_physics, EstimatorConfig{});
  for (std::size_t i = 0; i < snapshot.sample_count; ++i) estimator.push(snapshot.samples[i]);
  const auto state = estimator.estimate();
  ASSERT_TRUE(state.valid);
  EXPECT_FALSE(state.bounce_transition_used);
  EXPECT_NEAR(state.velocity.x(), -1.5, 1e-3);
  EXPECT_NEAR(state.position.z(), snapshot.latest_sample()->position.z(), 1e-3);
}
TEST(AdaptiveFlightCommit, SourceStallSendsSavedExposureWithoutRetiming) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 36; ++i) ASSERT_FALSE(tr.observe(incoming(i / 360.0)).snapshot_ready);
  ASSERT_EQ(tr.phase(), IncomingPhase::kWaitCommit);
  const auto u = tr.poll(100.50);
  ASSERT_TRUE(u.snapshot_ready);
  EXPECT_EQ(u.snapshot.commit_reason, "deadline_saved_prediction");
  EXPECT_LE(u.snapshot.latest_sample()->source_time_s, 100.10 + 1e-8);
  for (std::size_t i = 0; i < u.snapshot.sample_count; ++i)
    EXPECT_EQ(u.snapshot.samples[i].source_time_ns,
              std::llround(u.snapshot.samples[i].source_time_s * 1e9));
  EXPECT_FALSE(tr.poll(100.60).snapshot_ready);
}
TEST(AdaptiveFlightCommit, TimerBetweenFramesDoesNotBypassBounceDecision) {
  IncomingTrajectory tr(config(), physics());
  TrajectorySnapshot snapshot;
  int count = 0;
  double timer = 99.75;
  for (int i = -90; i < 180; ++i) {
    const auto sample = incoming(i / 360.0);
    while (timer < sample.source_time_s) {
      const auto u = tr.poll(timer);
      if (u.snapshot_ready) { ++count; snapshot = u.snapshot; }
      timer += 0.001;
    }
    const auto u = tr.observe(sample);
    if (u.snapshot_ready) { ++count; snapshot = u.snapshot; }
  }
  ASSERT_EQ(count, 1);
  EXPECT_EQ(snapshot.commit_reason, "observed_post_bounce");
  EXPECT_GT(snapshot.one_shot.commit_source_time_s, 100.20);
}
TEST(AdaptiveFlightCommit, ReceiptDelayConsumesPreparationBudget) {
  IncomingTrajectory tr(config(), physics());
  TrajectorySnapshot snapshot;
  for (int i = -90; i < 20; ++i) {
    auto u = tr.observe(incoming(i / 360.0, 2.5, 0.20));
    if (u.snapshot_ready) snapshot = u.snapshot;
  }
  ASSERT_GT(snapshot.sample_count, 0U);
  EXPECT_EQ(snapshot.commit_reason, "short_lead_early");
  EXPECT_LT(snapshot.latest_sample()->source_time_s, 100.05);
  EXPECT_GT(snapshot.one_shot.commit_source_time_s, 100.19);
}
TEST(AdaptiveFlightCommit, ResetCannotReleaseAnOldCandidate) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 36; ++i) tr.observe(incoming(i / 360.0));
  tr.reset_phase();
  EXPECT_FALSE(tr.poll(101.0).snapshot_ready);
}
TEST(AdaptiveFlightCommit, ReversalCancelsPendingPostBounceWait) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 36; ++i) ASSERT_FALSE(tr.observe(incoming(i / 360.0)).snapshot_ready);
  ASSERT_EQ(tr.phase(), IncomingPhase::kWaitCommit);
  bool cancelled = false;
  for (int i = 37; i < 60; ++i) {
    auto sample = incoming(i / 360.0);
    sample.position.x() = 1.22 + 2.0 * (i / 360.0 - 0.1);
    const auto update = tr.observe(sample);
    cancelled = cancelled || update.incoming_cancelled;
    EXPECT_FALSE(update.snapshot_ready);
  }
  EXPECT_TRUE(cancelled);
  EXPECT_FALSE(tr.poll(101.0).snapshot_ready);
}
TEST(AdaptiveFlightCommit, RejectsNonFiniteTimingConfiguration) {
  auto c = config(); c.adaptive.transport_budget_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(IncomingTrajectory(c, physics()), std::invalid_argument);
  auto p = physics(); p.future_physics.restitution_v = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(IncomingTrajectory(config(), p), std::invalid_argument);
}

TEST(AdaptiveFlightCommit, MeasuredContactPacketRetainsCausalRawHistoryForReceiver) {
  auto c = config();
  auto p = physics();
  c.adaptive.post_bounce_span_s = 0.04;
  p.estimator.post_bounce_min_span_s = 0.04;
  IncomingTrajectory tr(c, p);
  TrajectorySnapshot packet;
  int emitted = 0;
  for (int i = -90; i < 180; ++i) {
    auto update = tr.observe(incoming(i / 360.0));
    if (update.snapshot_ready) { packet = update.snapshot; ++emitted; }
  }
  ASSERT_EQ(emitted, 1);
  ASSERT_EQ(packet.commit_reason, "observed_post_bounce");
  EXPECT_LT(packet.one_shot.commit_source_time_s, 100.30);
  BatchPhysicsEstimator receiver(p.observed_physics, p.estimator);
  for (std::size_t i = 0; i < packet.sample_count; ++i) {
    receiver.push(packet.samples[i]);
    EXPECT_EQ(packet.samples[i].source_time_ns,
              std::llround(packet.samples[i].source_time_s * 1e9));
  }
  const auto state = receiver.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_EQ(state.reason, "post_bounce_free_flight");
  EXPECT_TRUE(state.bounce_epoch_active);
  EXPECT_FALSE(state.bounce_transition_used);
  EXPECT_LT(packet.samples[0].source_time_s, state.bounce_source_time_s);
  EXPECT_DOUBLE_EQ(state.source_time_s, packet.latest_sample()->source_time_s);
  EXPECT_NEAR(state.velocity.x(), -1.5, 1e-3);
  EXPECT_FALSE(tr.poll(102.0).snapshot_ready);
}

TEST(AdaptiveFlightCommit, RejectsInconsistentContactWindowsAtConstruction) {
  auto c = config();
  c.adaptive.post_bounce_span_s = 0.04;
  EXPECT_THROW(IncomingTrajectory(c, physics()), std::invalid_argument);
  auto p = physics();
  p.estimator.post_bounce_min_span_s = 0.04;
  EXPECT_NO_THROW(IncomingTrajectory(c, p));
  p.estimator.post_bounce_exclusion_s = 0.02;
  EXPECT_THROW(IncomingTrajectory(c, p), std::invalid_argument);
  p.estimator.post_bounce_min_span_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(IncomingTrajectory(c, p), std::invalid_argument);
}
}  // namespace
}  // namespace hope_planner_cpp
