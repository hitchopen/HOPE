#include "hope_planner_cpp/batch_physics_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace hope_planner_cpp {
namespace {

void integrate(Vec3& position, Vec3& velocity, const BallPhysics& physics, double dt) {
  const Vec3 acceleration = -physics.drag_k * velocity.norm() * velocity + physics.gravity;
  position += velocity * dt + 0.5 * acceleration * dt * dt;
  velocity += acceleration * dt;
}

TEST(BatchPhysicsEstimator, RecoversNonRecursivePhysicalStateAtWindowEnd) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.12;
  config.min_span_s = 0.08;
  config.min_samples = 12;
  config.huber_delta_m = 0.01;
  config.robust_iterations = 3;
  BatchPhysicsEstimator estimator(physics, config);

  Vec3 position(1.8, -0.65, 0.75);
  Vec3 velocity(-5.0, 0.40, 1.0);
  double time_s = 10.0;
  constexpr double sensor_dt = 1.0 / 360.0;
  Vec3 last_true_position = position;
  Vec3 last_true_velocity = velocity;
  for (int sample = 0; sample < 44; ++sample) {
    last_true_position = position;
    last_true_velocity = velocity;
    BallSample measurement;
    measurement.source_time_s = time_s;
    measurement.position = position;
    // One 5 cm mocap outlier must be downweighted rather than turning into a
    // persistent recursive state error.
    if (sample == 21) measurement.position.y() += 0.05;
    measurement.sequence = static_cast<std::uint64_t>(sample + 1);
    estimator.push(measurement);
    integrate(position, velocity, physics, sensor_dt);
    time_s += sensor_dt;
  }

  const BallState state = estimator.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_LT((state.position - last_true_position).norm(), 0.015);
  EXPECT_LT((state.velocity - last_true_velocity).norm(), 0.35);
  EXPECT_EQ(state.reason, "estimate_valid");
  EXPECT_TRUE(std::isfinite(state.residual_rms_m));
}

TEST(BatchPhysicsEstimator, DoesNotRestartForAClockGap) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.15;
  config.min_span_s = 0.08;
  config.min_samples = 8;
  BatchPhysicsEstimator estimator(physics, config);
  Vec3 position(1.5, -0.5, 0.7);
  Vec3 velocity(-3.0, 0.0, 0.5);
  double time_s = 20.0;
  for (int sample = 0; sample < 30; ++sample) {
    BallSample measurement;
    measurement.source_time_s = time_s;
    measurement.position = position;
    estimator.push(measurement);
    const double dt = sample == 12 ? 0.025 : 1.0 / 360.0;
    integrate(position, velocity, physics, dt);
    time_s += dt;
  }
  const BallState state = estimator.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_GE(state.sample_count, 20U);
}

TEST(BatchPhysicsEstimator, SplitsTheWindowAtBounceLocalMinimum) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.12;
  BatchPhysicsEstimator estimator(physics, config);
  for (int i = 0; i < 12; ++i) {
    BallSample sample;
    sample.source_time_s = 30.0 + i * 0.003;
    sample.position = Vec3(1.0 - 0.01 * i, -0.5, 0.10 - 0.006 * i);
    estimator.push(sample);
  }
  const std::size_t before = estimator.sample_count();
  for (int index = 0; index < 5; ++index) {
    BallSample rising;
    rising.source_time_s = 30.0 + (12 + index) * 0.003;
    rising.position = Vec3(
        0.88 - 0.01 * index, -0.5, 0.040 + 0.010 * index);
    estimator.push(rising);
  }
  EXPECT_TRUE(estimator.bounce_transition_active());
  EXPECT_GT(estimator.sample_count(), before);
  EXPECT_EQ(estimator.sample_count(), before + 5U);
}

TEST(BatchPhysicsEstimator, UsesPreBounceHistoryAsSoonAsBounceIsConfirmed) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.14;
  config.min_span_s = 0.08;
  config.min_samples = 12;
  config.huber_delta_m = 0.01;
  config.recency_half_life_s = 0.0;
  config.robust_iterations = 4;
  BatchPhysicsEstimator estimator(physics, config);

  Vec3 position(1.55, -0.60, 0.145);
  Vec3 velocity(-4.1, 0.35, -1.30);
  constexpr double sensor_dt = 1.0 / 360.0;
  double time_s = 40.0;
  bool bounced = false;
  Vec3 first_post_velocity = Vec3::Zero();
  std::size_t samples_before_first_post = 0;
  for (int sample_index = 0; sample_index < 80; ++sample_index) {
    BallSample measurement;
    measurement.source_time_s = time_s;
    measurement.position = position;
    measurement.sequence = static_cast<std::uint64_t>(sample_index + 1);
    estimator.push(measurement);

    integrate(position, velocity, physics, sensor_dt);
    if (!bounced && position.z() <= 0.02 && velocity.z() < 0.0) {
      position.z() = 0.02;
      const double tangential_speed = velocity.head<2>().norm();
      const double impulse = std::min(
          physics.table_tangential_gain * tangential_speed,
          physics.table_friction_cap_mu * (1.0 + physics.restitution_v) *
              std::abs(velocity.z()));
      if (tangential_speed > 1.0e-12) {
        velocity.head<2>() -=
            (impulse / tangential_speed) * velocity.head<2>();
      }
      velocity.z() = -physics.restitution_v * velocity.z();
      bounced = true;
      first_post_velocity = velocity;
      samples_before_first_post = estimator.sample_count();
    }
    time_s += sensor_dt;
    if (bounced && estimator.bounce_transition_active()) {
      break;
    }
  }

  ASSERT_TRUE(bounced);
  ASSERT_TRUE(estimator.bounce_transition_active());
  const BallState state = estimator.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_TRUE(state.bounce_transition_used);
  EXPECT_GE(state.pre_bounce_samples, config.min_samples);
  EXPECT_EQ(state.post_bounce_samples, config.bounce_confirmation_samples);
  EXPECT_GT(state.sample_count, samples_before_first_post);
  EXPECT_GT(state.velocity.z(), 0.0);
  EXPECT_LT((state.velocity - first_post_velocity).norm(), 0.75);
}

TEST(BatchPhysicsEstimator, DoesNotResplitOnPostBounceNoiseMinimum) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.30;
  config.bounce_min_reversal_m = 0.0001;
  config.bounce_refractory_s = 0.12;
  BatchPhysicsEstimator estimator(physics, config);
  for (int index = 0; index < 20; ++index) {
    BallSample sample;
    sample.source_time_s = 50.0 + index * 0.003;
    sample.position = Vec3(1.0 - index * 0.01, -0.5, 0.15 - index * 0.006);
    estimator.push(sample);
  }
  for (int index = 0; index < 5; ++index) {
    BallSample rise;
    rise.source_time_s = 50.060 + index * 0.003;
    rise.position = Vec3(
        0.80 - index * 0.01, -0.5, 0.040 + index * 0.006);
    estimator.push(rise);
  }
  ASSERT_TRUE(estimator.bounce_transition_active());
  const std::size_t after_physical_bounce = estimator.sample_count();

  const std::array<double, 7> noisy_z{
      0.055, 0.050, 0.052, 0.056, 0.060, 0.049, 0.047};
  for (std::size_t index = 0; index < noisy_z.size(); ++index) {
    BallSample noisy;
    noisy.source_time_s = 50.075 + static_cast<double>(index) * 0.003;
    noisy.position = Vec3(
        0.76 - static_cast<double>(index) * 0.01, -0.5, noisy_z[index]);
    estimator.push(noisy);
  }

  EXPECT_FALSE(estimator.bounce_detected());
  EXPECT_TRUE(estimator.bounce_transition_active());
  EXPECT_EQ(estimator.sample_count(), after_physical_bounce + noisy_z.size());
}

TEST(BatchPhysicsEstimator, ConfirmsLargeSparseRiseAfterTrackingGap) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.20;
  BatchPhysicsEstimator estimator(physics, config);
  for (int index = 0; index < 22; ++index) {
    BallSample sample;
    sample.source_time_s = 60.0 + index / 360.0;
    sample.position = Vec3(
        1.2 - index * 0.01, -0.4, 0.14 - index * 0.006);
    estimator.push(sample);
  }
  BallSample after_gap;
  after_gap.source_time_s = 60.0 + 21.0 / 360.0 + 0.025;
  after_gap.position = Vec3(0.97, -0.4, 0.090);
  estimator.push(after_gap);

  EXPECT_TRUE(estimator.bounce_detected());
  EXPECT_TRUE(estimator.bounce_transition_active());
  const BallState state = estimator.estimate();
  ASSERT_TRUE(state.valid) << state.reason;
  EXPECT_EQ(state.post_bounce_samples, 1U);
  EXPECT_GT(state.velocity.z(), 0.0);
}

TEST(BatchPhysicsEstimator, KeepsOneBounceEpochAfterContactLeavesWindow) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.12;
  config.min_span_s = 0.06;
  config.min_samples = 8;
  BatchPhysicsEstimator estimator(physics, config);
  estimator.begin_flight();

  for (int index = 0; index < 24; ++index) {
    BallSample descending;
    descending.source_time_s = 70.0 + index / 360.0;
    descending.position = Vec3(
        1.1 - 0.01 * index, -0.5, 0.16 - 0.005 * index);
    estimator.push(descending);
  }
  for (int index = 0; index < 60; ++index) {
    BallSample rising;
    rising.source_time_s = 70.0 + (24 + index) / 360.0;
    rising.position = Vec3(
        0.86 - 0.007 * index, -0.5, 0.055 + 0.004 * index);
    estimator.push(rising);
  }

  EXPECT_TRUE(estimator.bounce_epoch_active());
  EXPECT_FALSE(estimator.bounce_transition_active());
  const BallState state = estimator.estimate();
  EXPECT_TRUE(state.bounce_epoch_active);
  EXPECT_FALSE(state.bounce_transition_used);
  EXPECT_TRUE(std::isfinite(state.bounce_source_time_s));
  EXPECT_EQ(state.post_bounce_samples, state.sample_count);
}

TEST(BatchPhysicsEstimator, NewFlightClearsBounceEpochButKeepsHistory) {
  BallPhysics physics;
  EstimatorConfig config;
  config.window_s = 0.20;
  BatchPhysicsEstimator estimator(physics, config);
  for (int index = 0; index < 18; ++index) {
    BallSample descending;
    descending.source_time_s = 80.0 + index * 0.003;
    descending.position = Vec3(
        1.0 - index * 0.01, -0.5, 0.14 - index * 0.006);
    estimator.push(descending);
  }
  for (int index = 0; index < 5; ++index) {
    BallSample rising;
    rising.source_time_s = 80.054 + index * 0.003;
    rising.position = Vec3(
        0.82 - index * 0.01, -0.5, 0.046 + index * 0.008);
    estimator.push(rising);
  }
  ASSERT_TRUE(estimator.bounce_epoch_active());
  const auto count = estimator.sample_count();
  estimator.begin_flight();
  EXPECT_FALSE(estimator.bounce_epoch_active());
  EXPECT_FALSE(estimator.bounce_transition_active());
  EXPECT_EQ(estimator.sample_count(), count);
}

}  // namespace
}  // namespace hope_planner_cpp
