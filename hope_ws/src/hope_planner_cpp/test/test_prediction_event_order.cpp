#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

BallPhysics contact_only_physics() {
  BallPhysics physics;
  physics.drag_k = 0.0;
  physics.magnus_k = 0.0;
  physics.gravity = Vec3::Zero();
  physics.table_tangential_gain = 0.2;
  physics.table_friction_cap_mu = 2.0;  // analytic uncapped event-order fixture
  physics.restitution_h = 0.8;
  physics.restitution_v = 0.9;
  return physics;
}

BallState incoming(SpinPhysicsMode mode) {
  BallState state;
  state.valid = true;
  state.source_time_s = 12.0;
  // Contact occurs 0.75 ms into a 1 ms integration step.
  const double contact_z = mode == SpinPhysicsMode::kLegacyNoSpin ? 0.0 : 0.02;
  state.position = Vec3(1.0, -0.7, contact_z + 0.00075);
  state.velocity = Vec3(-2.0, 0.0, -1.0);
  return state;
}

TEST(PredictionEventOrder, DescendingCrossingPrecedesSameStepContact) {
  for (auto mode : {SpinPhysicsMode::kLegacyNoSpin,
                    SpinPhysicsMode::kVenueGripBounce,
                    SpinPhysicsMode::kVenueGripBounceAndMagnus,
                    SpinPhysicsMode::kNakashimaBounce,
                    SpinPhysicsMode::kNakashimaBounceAndMagnus}) {
    SCOPED_TRACE(static_cast<int>(mode));
    const auto state = incoming(mode);
    auto config = PlannerConfig{};
    const auto coarse = TrajectoryPredictor(contact_only_physics(), config, {}).predict_with_spin(
        state, 0.9995, Vec3::Zero(), mode);
    config.integrate_dt_s = 0.0001;
    const auto fine = TrajectoryPredictor(contact_only_physics(), config, {}).predict_with_spin(
        state, 0.9995, Vec3::Zero(), mode);
    EXPECT_TRUE(coarse.valid);
    EXPECT_EQ(coarse.reason, "prediction_valid");
    EXPECT_EQ(coarse.predicted_bounces, 0);
    EXPECT_NEAR(coarse.strike_source_time_s, 12.00025, 1e-10);
    EXPECT_NEAR(coarse.ball_position.z(), state.position.z() - 0.00025, 1e-10);
    EXPECT_NEAR(coarse.ball_velocity.z(), -1.0, 1e-10);
    EXPECT_EQ(coarse.valid, fine.valid);
    EXPECT_EQ(coarse.predicted_bounces, fine.predicted_bounces);
    EXPECT_LT((coarse.ball_position - fine.ball_position).norm(), 1e-10);
    EXPECT_LT((coarse.ball_velocity - fine.ball_velocity).norm(), 1e-10);
    EXPECT_EQ(coarse.ball_spin_rad_s, Vec3::Zero());
  }
}

TEST(PredictionEventOrder, CrossingAfterContactUsesReboundArc) {
  for (auto mode : {SpinPhysicsMode::kLegacyNoSpin, SpinPhysicsMode::kVenueGripBounce}) {
    SCOPED_TRACE(static_cast<int>(mode));
    const auto strike = TrajectoryPredictor(contact_only_physics(), {}, {}).predict_with_spin(
        incoming(mode), 0.9984, Vec3::Zero(), mode);
    ASSERT_TRUE(strike.valid) << strike.reason;
    EXPECT_EQ(strike.predicted_bounces, 1);
    EXPECT_NEAR(strike.strike_source_time_s, 12.0008125, 1e-10);
    EXPECT_NEAR(strike.ball_velocity.x(), -1.6, 1e-10);
    EXPECT_NEAR(strike.ball_velocity.z(), 0.9, 1e-10);
    if (mode == SpinPhysicsMode::kVenueGripBounce)
      EXPECT_NEAR(strike.ball_spin_rad_s.y(), -30.0, 1e-10);
  }
}

TEST(PredictionEventOrder, LaterImpactReversalDoesNotHideEarlierCrossing) {
  const auto mode = SpinPhysicsMode::kVenueGripBounce;
  const auto strike = TrajectoryPredictor(contact_only_physics(), {}, {}).predict_with_spin(
      incoming(mode), 0.9995, Vec3(0.0, 500.0, 0.0), mode);
  EXPECT_TRUE(strike.valid);
  EXPECT_EQ(strike.reason, "prediction_valid");
  EXPECT_EQ(strike.predicted_bounces, 0);
  EXPECT_NEAR(strike.strike_source_time_s, 12.00025, 1e-10);
  EXPECT_NEAR(strike.ball_velocity.x(), -2.0, 1e-10);
  EXPECT_EQ(strike.ball_spin_rad_s, Vec3(0.0, 500.0, 0.0));
}

TEST(PredictionEventOrder, LowDescendingCrossingsHaveNoPolicyHeightCutoff) {
  for (auto mode : {SpinPhysicsMode::kLegacyNoSpin,
                    SpinPhysicsMode::kVenueGripBounce,
                    SpinPhysicsMode::kVenueGripBounceAndMagnus,
                    SpinPhysicsMode::kNakashimaBounce,
                    SpinPhysicsMode::kNakashimaBounceAndMagnus}) {
    for (double height : {0.03, 0.049, 0.05, 0.051}) {
      SCOPED_TRACE(static_cast<int>(mode));
      SCOPED_TRACE(height);
      auto state = incoming(mode);
      state.position.z() = height + 0.00025;
      const auto strike = TrajectoryPredictor(contact_only_physics(), {}, {}).predict_with_spin(
          state, 0.9995, Vec3::Zero(), mode);
      ASSERT_TRUE(strike.valid) << strike.reason;
      EXPECT_EQ(strike.predicted_bounces, 0);
      EXPECT_NEAR(strike.ball_position.z(), height, 1e-10);
      EXPECT_NEAR(strike.ball_velocity.z(), -1.0, 1e-10);
    }
  }
}

}  // namespace
}  // namespace hope_planner_cpp
