#include "hope_planner_cpp/incoming_trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace hope_planner_cpp {
namespace {

BallSample sample(double time_s, double x) {
  BallSample value;
  value.source_time_s = time_s;
  value.position = Vec3(x, -0.5, 0.4);
  value.sequence = static_cast<std::uint64_t>(std::llround(time_s * 1000.0));
  return value;
}

IncomingTrajectoryConfig test_config() {
  IncomingTrajectoryConfig config;
  config.net_x = 1.37;
  config.estimator_window_s = 0.18;
  config.commit_delay_s = 0.05;
  config.opponent_side_margin_m = 0.03;
  config.incoming_speed_threshold_mps = 0.2;
  config.outgoing_speed_threshold_mps = 0.2;
  config.direction_fit_samples = 3;
  config.direction_confirmations = 2;
  config.pre_roll_samples = 16;
  config.source_gap_reset_s = 0.25;
  return config;
}

TEST(IncomingTrajectory, QuickIncomingSnapshotContainsOnlyCurrentEpoch) {
  IncomingTrajectory trajectory(test_config());
  IncomingTrajectoryUpdate result;
  constexpr double dt = 1.0 / 360.0;
  // Opponent contact is represented by the X maximum at t=10.0. The ball
  // reaches the net about 80 ms later and commits another 50 ms later.
  for (int i = 0; i < 55; ++i) {
    const double time = 10.0 + i * dt;
    const double x = 1.62 - 3.0 * (time - 10.0);
    const auto update = trajectory.observe(sample(time, x));
    if (update.snapshot_ready) result = update;
  }
  ASSERT_TRUE(result.snapshot_ready);
  EXPECT_EQ(result.snapshot.trajectory_epoch, 1U);
  ASSERT_GT(result.snapshot.sample_count, 30U);
  EXPECT_GE(result.snapshot.samples[0].source_time_s, 10.0 - 1.0e-12);
  EXPECT_NEAR(
      result.snapshot.one_shot.commit_source_time_s - 10.0,
      (1.62 - 1.37) / 3.0 + 0.05,
      2.0 * dt);
  EXPECT_GT(
      result.snapshot.samples[result.snapshot.sample_count - 1].source_time_s -
          result.snapshot.samples[0].source_time_s,
      0.12);
  EXPECT_EQ(trajectory.phase(), IncomingPhase::kWaitOutgoing);
}

TEST(IncomingTrajectory, IgnoresOutgoingAndBacktracksOpponentTurnaround) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  IncomingTrajectoryUpdate first;
  for (int i = 0; i < 60; ++i) {
    const double time = 20.0 + i * dt;
    const auto update = trajectory.observe(sample(time, 1.60 - 3.0 * (time - 20.0)));
    if (update.snapshot_ready) first = update;
  }
  ASSERT_TRUE(first.snapshot_ready);

  // Robot return: ignored while +X and while crossing to the opponent side.
  double time = 20.17;
  for (int i = 0; i < 80; ++i, time += dt) {
    trajectory.observe(sample(time, 0.95 + 3.0 * (time - 20.17)));
  }
  EXPECT_EQ(trajectory.phase(), IncomingPhase::kWaitOpponentReturn);
  EXPECT_EQ(trajectory.retained_samples(), 0U);

  // A short noisy plateau followed by stable -X confirms the opponent return.
  const double turn_time = time;
  const double turn_x = 0.95 + 3.0 * (turn_time - 20.17);
  for (int i = 0; i < 3; ++i, time += dt) {
    trajectory.observe(sample(time, turn_x + (i == 1 ? 0.0002 : 0.0)));
  }
  IncomingTrajectoryUpdate second;
  for (int i = 0; i < 90; ++i, time += dt) {
    const auto update = trajectory.observe(
        sample(time, turn_x - 3.0 * (time - turn_time)));
    if (update.snapshot_ready) {
      second = update;
      break;
    }
  }
  ASSERT_TRUE(second.snapshot_ready);
  EXPECT_EQ(second.snapshot.trajectory_epoch, 2U);
  EXPECT_EQ(second.snapshot.segment_boundary_reason, "opponent_turnaround");
  EXPECT_GE(second.snapshot.samples[0].source_time_s, turn_time - 2.0 * dt);
  for (std::size_t i = 1; i < second.snapshot.sample_count; ++i) {
    EXPECT_LE(
        second.snapshot.samples[i].position.x(),
        second.snapshot.samples[0].position.x() + 0.001);
  }
}

TEST(IncomingTrajectory, MissDoesNotRearmOnSameIncomingBall) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  double time = 30.0;
  bool committed = false;
  for (int i = 0; i < 70; ++i, time += dt) {
    const auto update = trajectory.observe(sample(time, 1.60 - 3.0 * (time - 30.0)));
    committed = committed || update.snapshot_ready;
  }
  ASSERT_TRUE(committed);
  ASSERT_EQ(trajectory.phase(), IncomingPhase::kWaitOutgoing);
  const auto epoch = trajectory.trajectory_epoch();

  // A missed ball keeps moving -X on our side. It cannot become a new return.
  for (int i = 0; i < 30; ++i, time += dt) {
    trajectory.observe(sample(time, 1.0 - 2.0 * i * dt));
  }
  EXPECT_EQ(trajectory.phase(), IncomingPhase::kWaitOutgoing);
  EXPECT_EQ(trajectory.trajectory_epoch(), epoch);
}

TEST(IncomingTrajectory, SourceGapAllowsASeparateNewRally) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  for (int i = 0; i < 70; ++i) {
    trajectory.observe(sample(40.0 + i * dt, 1.60 - 3.0 * i * dt));
  }
  ASSERT_EQ(trajectory.phase(), IncomingPhase::kWaitOutgoing);
  const auto old_epoch = trajectory.trajectory_epoch();

  bool reset_seen = false;
  bool incoming_seen = false;
  for (int i = 0; i < 12; ++i) {
    const auto update = trajectory.observe(
        sample(41.0 + i * dt, 1.70 - 2.0 * i * dt));
    reset_seen = reset_seen || update.source_epoch_reset;
    incoming_seen = incoming_seen || update.incoming_started;
  }
  EXPECT_TRUE(reset_seen);
  EXPECT_TRUE(incoming_seen);
  EXPECT_EQ(trajectory.trajectory_epoch(), old_epoch + 1);
}

TEST(IncomingTrajectory, OpponentSidePositionNoiseDoesNotStartAnEpoch) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  for (int i = 0; i < 100; ++i) {
    const double noise = (i % 2 == 0) ? 0.0001 : -0.0001;
    const auto update = trajectory.observe(sample(50.0 + i * dt, 1.60 + noise));
    EXPECT_FALSE(update.incoming_started);
    EXPECT_FALSE(update.snapshot_ready);
  }
  EXPECT_EQ(trajectory.phase(), IncomingPhase::kSeekIncoming);
  EXPECT_EQ(trajectory.trajectory_epoch(), 0U);
}

TEST(IncomingTrajectory, StableOutgoingEvidenceAbandonsUncommittedCandidate) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  double time = 55.0;
  double x = 1.55;
  for (int i = 0; i < 8; ++i, time += dt, x -= 1.0 * dt) {
    trajectory.observe(sample(time, x));
  }
  ASSERT_EQ(trajectory.phase(), IncomingPhase::kCollectIncoming);
  const auto false_epoch = trajectory.trajectory_epoch();

  for (int i = 0; i < 8; ++i, time += dt, x += 2.0 * dt) {
    trajectory.observe(sample(time, x));
  }
  ASSERT_EQ(trajectory.phase(), IncomingPhase::kWaitOpponentReturn);
  EXPECT_EQ(trajectory.retained_samples(), 0U);

  IncomingTrajectoryUpdate result;
  const double turnaround_time = time;
  const double turnaround_x = x;
  for (int i = 0; i < 100; ++i, time += dt) {
    const auto update = trajectory.observe(
        sample(time, turnaround_x - 3.0 * (time - turnaround_time)));
    if (update.snapshot_ready) {
      result = update;
      break;
    }
  }
  ASSERT_TRUE(result.snapshot_ready);
  EXPECT_EQ(result.snapshot.trajectory_epoch, false_epoch + 1);
  EXPECT_EQ(result.snapshot.segment_boundary_reason, "opponent_turnaround");
  EXPECT_GE(
      result.snapshot.samples[0].source_time_s,
      turnaround_time - dt - 1.0e-12);
}

TEST(IncomingTrajectory, SeparatesEpochsEvenWhenPreviousTailIsUnderWindow) {
  IncomingTrajectory trajectory(test_config());
  constexpr double dt = 1.0 / 360.0;
  double time = 60.0;
  IncomingTrajectoryUpdate first;
  for (int i = 0; i < 32; ++i, time += dt) {
    const auto update = trajectory.observe(
        sample(time, 1.60 - 10.0 * (time - 60.0)));
    if (update.snapshot_ready) first = update;
  }
  ASSERT_TRUE(first.snapshot_ready);

  const double outgoing_start = time;
  for (int i = 0; i < 24; ++i, time += dt) {
    trajectory.observe(sample(time, 1.0 + 10.0 * (time - outgoing_start)));
  }
  ASSERT_EQ(trajectory.phase(), IncomingPhase::kWaitOpponentReturn);

  const double turnaround_time = time;
  const double turnaround_x = 1.0 + 10.0 * (time - outgoing_start);
  IncomingTrajectoryUpdate second;
  for (int i = 0; i < 40; ++i, time += dt) {
    const auto update = trajectory.observe(
        sample(time, turnaround_x - 10.0 * (time - turnaround_time)));
    if (update.snapshot_ready) {
      second = update;
      break;
    }
  }
  ASSERT_TRUE(second.snapshot_ready);
  ASSERT_TRUE(std::isfinite(second.snapshot.previous_segment_last_source_time_s));
  EXPECT_LT(
      second.snapshot.one_shot.commit_source_time_s -
          second.snapshot.previous_segment_last_source_time_s,
      test_config().estimator_window_s);
  EXPECT_GE(
      second.snapshot.samples[0].source_time_s,
      turnaround_time - dt - 1.0e-12);
  for (std::size_t i = 1; i < second.snapshot.sample_count; ++i) {
    EXPECT_LE(
        second.snapshot.samples[i].position.x(),
        second.snapshot.samples[i - 1].position.x() + 1.0e-12);
  }
}

TEST(LatestSnapshotMailbox, NewestPendingSnapshotSupersedesOldest) {
  LatestSnapshotMailbox mailbox;
  TrajectorySnapshot first;
  first.trajectory_epoch = 1;
  first.snapshot_sequence = 1;
  TrajectorySnapshot second;
  second.trajectory_epoch = 2;
  second.snapshot_sequence = 2;
  EXPECT_FALSE(mailbox.publish(first));
  EXPECT_TRUE(mailbox.publish(second));
  TrajectorySnapshot taken;
  ASSERT_TRUE(mailbox.try_take(taken));
  EXPECT_EQ(taken.trajectory_epoch, 2U);
  EXPECT_EQ(mailbox.published(), 2U);
  EXPECT_EQ(mailbox.consumed(), 1U);
  EXPECT_EQ(mailbox.superseded(), 1U);
  EXPECT_FALSE(mailbox.try_take(taken));
}

}  // namespace
}  // namespace hope_planner_cpp
