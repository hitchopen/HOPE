#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/adaptive_flight_commit.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace hope_planner_cpp {
namespace {
constexpr double kHz = 300.0;
const double kBounce = (-3.0 + std::sqrt(9.0 + 2 * 9.81 * 0.78)) / 9.81;
CommitPredictionConfig physics() {
  CommitPredictionConfig p;
  p.observed_physics.drag_k = 0.0;
  p.observed_physics.table_tangential_gain = 0.0;
  p.observed_physics.restitution_v = 0.9;
  p.future_physics = p.observed_physics;
  p.estimator.single_flight_only = true;
  p.estimator.min_samples = 24;
  p.estimator.post_bounce_min_span_s = 0.08;
  p.estimator.max_sample_gap_s = 0.02;
  p.planner.adaptive_predict_horizon = true;
  return p;
}
IncomingTrajectoryConfig config() {
  IncomingTrajectoryConfig c;
  c.adaptive.enabled = true;
  c.estimator_max_sample_gap_s = 0.02;
  c.direction_fit_samples = 8;
  c.pre_roll_samples = 48;
  return c;
}
BallSample incoming(double t, double speed = 1.5, double receipt_age = 0.0) {
  BallSample s;
  s.source_time_s = 100.0 + t;
  s.source_time_ns = std::llround(s.source_time_s * 1e9);
  s.receipt_wall_ns = std::llround((s.source_time_s + receipt_age) * 1e9);
  double z = 0.8 - 3.0 * t - 4.905 * t * t;
  if (t > kBounce) {
    const double u = t - kBounce;
    z = 0.02 + 0.9 * (3.0 + 9.81 * kBounce) * u - 4.905 * u * u;
  }
  s.position = Vec3(1.37 - speed * t, -0.6, z);
  return s;
}
TrajectorySnapshot replay(IncomingTrajectoryConfig c, double speed,
                          double receipt_age = 0.0, bool timers = false) {
  IncomingTrajectory tr(c, physics());
  TrajectorySnapshot snapshot;
  int count = 0;
  double timer = 99.7;
  for (int i = -90; i < 300; ++i) {
    const auto sample = incoming(i / kHz, speed, receipt_age);
    while (timers && timer < sample.source_time_s + receipt_age) {
      const auto u = tr.poll(timer);
      if (u.snapshot_ready) { ++count; snapshot = u.snapshot; }
      timer += 0.001;
    }
    const auto u = tr.observe(sample);
    if (u.snapshot_ready) { ++count; snapshot = u.snapshot; }
  }
  EXPECT_EQ(count, 1);
  EXPECT_FALSE(tr.poll(102.0).snapshot_ready);
  return snapshot;
}
TEST(AdaptiveFlightCommit, BeforeBounceUsesLatestFreeFlightAtHitDeadline) {
  const auto packet = replay(config(), 2.0);
  ASSERT_GT(packet.sample_count, 24U);
  const double expected = 100.0 + 1.22 / 2.0 - 0.475;
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, expected, 0.004);
  EXPECT_GT(packet.one_shot.commit_source_time_s, 100.05);
  EXPECT_LT(packet.latest_sample()->source_time_s, 100.0 + kBounce);
  EXPECT_EQ(packet.commit_reason, "hit_deadline");
  // The final sample is the exposure that triggered commit, even if the
  // previous preview tick was less than 10 ms ago.
  EXPECT_NEAR(packet.latest_sample()->source_time_s,
              packet.one_shot.commit_source_time_s, 1e-9);
  BatchPhysicsEstimator receiver(physics().observed_physics, physics().estimator);
  for (std::size_t i = 0; i < packet.sample_count; ++i) receiver.push(packet.samples[i]);
  const auto state = receiver.estimate();
  ASSERT_TRUE(state.valid);
  const FutureContactPredictor predictor(physics().future_physics, physics().planner, {});
  const auto strike = predictor.predict(state, 0.15, {});
  ASSERT_TRUE(strike.valid);
  EXPECT_EQ(strike.predicted_bounces, 1);
  EXPECT_NEAR(strike.strike_source_time_s, 100.61, 0.002);
}
TEST(AdaptiveFlightCommit, PostBouncePacketNeverContainsTheBounce) {
  const auto packet = replay(config(), 1.5);
  ASSERT_GT(packet.sample_count, 24U);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.0 + 1.22 / 1.5 - 0.475, 0.004);
  EXPECT_GT(packet.samples[0].source_time_s, 100.0 + kBounce + 0.008);
  BatchPhysicsEstimator receiver(physics().observed_physics, physics().estimator);
  for (std::size_t i = 0; i < packet.sample_count; ++i) receiver.push(packet.samples[i]);
  const auto state = receiver.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_FALSE(state.bounce_transition_used);
  EXPECT_NEAR(state.velocity.x(), -1.5, 1e-3);
  const FutureContactPredictor predictor(physics().future_physics, physics().planner, {});
  const auto strike = predictor.predict(state, 0.15, {});
  EXPECT_TRUE(strike.valid);
  EXPECT_EQ(strike.predicted_bounces, 0);
}
TEST(AdaptiveFlightCommit, NoNetBaselineOrHalfSecondCap) {
  auto c = config();
  c.commit_delay_s = 9.0;  // irrelevant in deadline mode
  const auto packet = replay(c, 1.0);
  EXPECT_GT(packet.one_shot.commit_source_time_s, 100.5);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.0 + 1.22 - 0.475, 0.004);
}
TEST(AdaptiveFlightCommit, Full300HzWindowPreservesEveryExposure) {
  const auto packet = replay(config(), 1.0);
  ASSERT_GE(packet.sample_count, 54U);
  ASSERT_LE(packet.sample_count, 55U);
  const double end = packet.latest_sample()->source_time_s;
  EXPECT_GE(packet.samples[0].source_time_s, end - 0.18 - 1e-9);
  for (std::size_t i = 0; i < packet.sample_count; ++i) {
    EXPECT_EQ(packet.samples[i].source_time_ns,
              std::llround(packet.samples[i].source_time_s * 1e9));
    if (i) EXPECT_NEAR(packet.samples[i].source_time_s - packet.samples[i-1].source_time_s,
                      1.0 / kHz, 1e-10);
  }
}
TEST(AdaptiveFlightCommit, Transport30msMovesCommit70msLater) {
  auto c = config();
  const auto original = replay(c, 1.5);
  c.adaptive.transport_budget_s = 0.03;
  const auto reduced = replay(c, 1.5);
  EXPECT_NEAR(reduced.one_shot.commit_source_time_s - original.one_shot.commit_source_time_s,
              0.07, 0.004);
  EXPECT_GT(reduced.latest_sample()->source_time_s, original.latest_sample()->source_time_s);
}
TEST(AdaptiveFlightCommit, FastFlightIsSubmittedImmediatelyAfterNet) {
  const auto packet = replay(config(), 4.0);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.0, 0.004);
  EXPECT_LT(packet.latest_sample()->source_time_s, 100.0 + kBounce);
}
TEST(AdaptiveFlightCommit, TimerIncludesLatestFrameWithoutEarlyBounceSubmission) {
  const auto packet = replay(config(), 1.5, 0.0, true);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.0 + 1.22 / 1.5 - 0.475, 0.002);
  EXPECT_LT(packet.one_shot.commit_source_time_s - packet.latest_sample()->source_time_s, 1.0/kHz+0.001);
}
TEST(AdaptiveFlightCommit, ReceiptAgeConsumesBudgetWithoutChangingExposureClock) {
  const auto packet = replay(config(), 2.0, 0.02);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.135, 0.004);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s - packet.latest_sample()->source_time_s, 0.02, 1e-9);
}
TEST(AdaptiveFlightCommit, SourceStallCannotPublishAnOldPrediction) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 30; ++i) ASSERT_FALSE(tr.observe(incoming(i / kHz)).snapshot_ready);
  EXPECT_FALSE(tr.poll(100.5).snapshot_ready);
  EXPECT_FALSE(tr.poll(101.0).snapshot_ready);
}
TEST(AdaptiveFlightCommit, ResetAndOutgoingReversalCancelPendingFlight) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 30; ++i) ASSERT_FALSE(tr.observe(incoming(i / kHz)).snapshot_ready);
  bool cancelled = false;
  for (int i = 31; i < 60; ++i) {
    auto sample = incoming(i / kHz);
    sample.position.x() = 1.22 + 2.0 * (i / kHz - 0.1);
    const auto update = tr.observe(sample);
    cancelled = cancelled || update.incoming_cancelled;
    EXPECT_FALSE(update.snapshot_ready);
  }
  EXPECT_TRUE(cancelled);
  EXPECT_FALSE(tr.poll(101.0).snapshot_ready);
  tr.reset_phase();
  EXPECT_FALSE(tr.poll(102.0).snapshot_ready);
}
TEST(AdaptiveFlightCommit, GapDropsOldHistoryButDoesNotRestartFlightIdentity) {
  IncomingTrajectory tr(config(), physics());
  TrajectorySnapshot packet;
  for (int i = -90; i < 300; ++i) {
    if (i > 120 && i < 135) continue;  // a 50 ms outage after bounce
    const auto u = tr.observe(incoming(i/kHz, 1.0));
    if (u.snapshot_ready) packet = u.snapshot;
  }
  ASSERT_GT(packet.sample_count, 24U);
  EXPECT_EQ(packet.trajectory_epoch, 1U);
  EXPECT_GE(packet.samples[0].source_time_s, 100.45);
}
TEST(AdaptiveFlightCommit, StrictFitNeverUsesMixedHistoryDuringContactWarmup) {
  auto p = physics();
  BatchPhysicsEstimator estimator(p.observed_physics, p.estimator);
  bool saw_pending = false, saw_post = false;
  for (int i = -40; i < 140; ++i) {
    estimator.push(incoming(i/kHz));
    const auto state = estimator.estimate();
    EXPECT_FALSE(state.valid && state.bounce_transition_used);
    saw_pending |= state.reason == "contact_confirmation_pending" ||
                   state.reason == "post_bounce_segment_not_ready";
    if (state.valid && state.bounce_epoch_active) {
      saw_post = true;
      EXPECT_EQ(state.pre_bounce_samples, 0U);
      EXPECT_GT(state.post_bounce_samples, 0U);
    }
  }
  EXPECT_TRUE(saw_pending);
  EXPECT_TRUE(saw_post);
}
TEST(AdaptiveFlightCommit, NewSegmentCannotSendSavedPreGapHistoryAtDeadline) {
  IncomingTrajectory tr(config(), physics());
  for (int i = -90; i <= 30; ++i)
    ASSERT_FALSE(tr.observe(incoming(i/kHz, 2.0)).snapshot_ready);
  // The old candidate is only 40 ms old, but is no longer the latest segment.
  EXPECT_FALSE(tr.observe(incoming(0.14, 2.0)).snapshot_ready);
  EXPECT_FALSE(tr.poll(100.141).snapshot_ready);
}
TEST(AdaptiveFlightCommit, UpdatedHitPlaneRetimesPendingStateInsteadOfFreezingFirstDeadline) {
  auto p = physics();
  p.planner.x_hit = 0.30;
  IncomingTrajectory tr(config(), p);
  TrajectorySnapshot packet;
  for (int i = -90; i < 180; ++i) {
    if (i == 30) tr.set_prediction_x_hit(0.15);
    const auto u = tr.observe(incoming(i/kHz));
    if (u.snapshot_ready) packet = u.snapshot;
  }
  ASSERT_GT(packet.sample_count, 24U);
  EXPECT_NEAR(packet.one_shot.commit_source_time_s, 100.0 + 1.22/1.5 - 0.475, 0.004);
  EXPECT_GT(packet.samples[0].source_time_s, 100.0 + kBounce + 0.008);
}
TEST(AdaptiveFlightCommit, RejectsNonFiniteBudgets) {
  for (int field = 0; field < 5; ++field) {
    auto c = config();
    double* fields[] = {&c.adaptive.transport_budget_s, &c.adaptive.solve_budget_s,
        &c.adaptive.policy_tick_s, &c.adaptive.max_sample_age_s, &c.adaptive.evaluation_period_s};
    *fields[field] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(IncomingTrajectory(c, physics()), std::invalid_argument);
  }
  auto p = physics();
  p.future_physics.restitution_v = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(IncomingTrajectory(config(), p), std::invalid_argument);
  p = physics();
  p.estimator.post_bounce_min_span_s = 0.04;
  EXPECT_THROW(IncomingTrajectory(config(), p), std::invalid_argument);
  p.estimator.min_span_s = 0.04;
  EXPECT_NO_THROW(IncomingTrajectory(config(), p));
}
}  // namespace
}  // namespace hope_planner_cpp
