#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

TEST(TrajectoryAndStage3, MatchesThePythonScalarReference) {
  BallPhysics physics;
  PlannerConfig config;
  config.x_hit = 0.15;
  config.target_land = Vec3(2.055, -0.7625, 0.0);
  config.delta_t_flight = 0.5;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  RacketTargetPlanner stage3(physics, config, table);

  BallState state;
  state.position = Vec3(1.4, -0.55, 0.35);
  state.velocity = Vec3(-4.2, 0.35, -1.2);
  state.source_time_s = 123.0;
  state.valid = true;
  const StrikeTarget strike = predictor.predict(state, config.x_hit);
  ASSERT_TRUE(strike.valid) << strike.reason;
  EXPECT_NEAR(strike.ball_position.x(), 0.15, 1e-12);
  EXPECT_NEAR(strike.ball_position.y(), -0.44583333, 1e-8);
  EXPECT_NEAR(strike.ball_position.z(), 0.30292193, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.x(), -2.24334337, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.y(), 0.18694528, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.z(), 0.11741171, 1e-8);
  EXPECT_NEAR(strike.strike_source_time_s, 123.41005860349028, 1e-10);
  EXPECT_EQ(strike.predicted_bounces, 1);

  const RacketCommand command = stage3.plan(
      strike, config.target_land, config.delta_t_flight);
  ASSERT_TRUE(command.valid) << command.reason;
  EXPECT_TRUE(command.clears_net);
  EXPECT_FALSE(command.bypasses_net_posts);
  EXPECT_NEAR(command.velocity.x(), 1.98456672, 1e-8);
  EXPECT_NEAR(command.velocity.y(), -0.27380446, 1e-8);
  EXPECT_NEAR(command.velocity.z(), 0.53137587, 1e-8);
  EXPECT_NEAR(command.normal.x(), 0.95750697, 1e-8);
  EXPECT_NEAR(command.normal.y(), -0.13210424, 1e-8);
  EXPECT_NEAR(command.normal.z(), 0.25637641, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.x(), 4.33657509, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.y(), -0.72086549, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.z(), 1.87921184, 1e-8);
}

TEST(TrajectoryAndStage3, KeepsMathematicalNoCrossingInvalid) {
  BallPhysics physics;
  PlannerConfig config;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  BallState state;
  state.position = Vec3(0.1, -0.5, 0.5);
  state.velocity = Vec3(-2.0, 0.0, 0.0);
  state.source_time_s = 1.0;
  state.valid = true;
  const auto strike = predictor.predict(state, 0.15);
  EXPECT_FALSE(strike.valid);
  EXPECT_EQ(strike.reason, "no_hit_plane_crossing");
}

TEST(TrajectoryAndStage3, LegacySpinModeIsExactlyTheLegacyPath) {
  BallPhysics physics;
  PlannerConfig config;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  BallState state;
  state.position = Vec3(1.4, -0.55, 0.35);
  state.velocity = Vec3(-4.2, 0.35, -1.2);
  state.source_time_s = 123.0;
  state.valid = true;
  const auto legacy = predictor.predict(state, 0.15);
  const auto dispatched = predictor.predict_with_spin(
      state, 0.15, Vec3(20.0, -30.0, 10.0),
      SpinPhysicsMode::kLegacyNoSpin);
  EXPECT_EQ(dispatched.valid, legacy.valid);
  EXPECT_EQ(dispatched.reason, legacy.reason);
  EXPECT_EQ(dispatched.ball_position, legacy.ball_position);
  EXPECT_EQ(dispatched.ball_velocity, legacy.ball_velocity);
  EXPECT_EQ(dispatched.strike_source_time_s, legacy.strike_source_time_s);
}

TEST(TrajectoryAndStage3, NakashimaTopspinChangesPostBounceCrossing) {
  BallPhysics physics;
  PlannerConfig config;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  BallState state;
  state.position = Vec3(1.4, -0.55, 0.35);
  state.velocity = Vec3(-4.2, 0.35, -1.2);
  state.source_time_s = 123.0;
  state.valid = true;
  const auto no_spin = predictor.predict(state, 0.15);
  const auto topspin = predictor.predict_with_spin(
      state, 0.15, Vec3(0.0, -40.0, 0.0),
      SpinPhysicsMode::kNakashimaBounceAndMagnus);
  ASSERT_TRUE(no_spin.valid);
  ASSERT_TRUE(topspin.valid) << topspin.reason;
  EXPECT_EQ(topspin.predicted_bounces, 1);
  EXPECT_GT((topspin.ball_position - no_spin.ball_position).norm(), 0.01);
  EXPECT_GT((topspin.ball_velocity - no_spin.ball_velocity).norm(), 0.10);
}

TEST(TrajectoryAndStage3, VenueGripSpinPathProducesFiniteCrossing) {
  BallPhysics physics;
  PlannerConfig config;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  BallState state;
  state.position = Vec3(1.4, -0.55, 0.35);
  state.velocity = Vec3(-4.2, 0.35, -1.2);
  state.source_time_s = 123.0;
  state.valid = true;
  const auto result = predictor.predict_with_spin(
      state, 0.15, Vec3(5.0, -35.0, 8.0),
      SpinPhysicsMode::kVenueGripBounceAndMagnus);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.ball_position.allFinite());
  EXPECT_TRUE(result.ball_velocity.allFinite());
  EXPECT_EQ(result.predicted_bounces, 1);
  // Independent scalar Python port of contact_model.predict_contact plus the
  // 1 ms Stage-2 integrator. This anchors the full flight/contact/crossing
  // path, not merely finiteness of the C++ result.
  EXPECT_NEAR(result.ball_position.x(), 0.15, 1.0e-12);
  EXPECT_NEAR(result.ball_position.y(), -0.46401110440006949, 1.0e-9);
  EXPECT_NEAR(result.ball_position.z(), 0.28809310753079093, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.x(), -2.5655457961134989, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.y(), 0.12888004786570353, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.z(), -0.089614952199157896, 1.0e-9);
  EXPECT_NEAR(result.strike_source_time_s, 123.39008443740325, 1.0e-10);
}

TEST(TrajectoryAndStage3, ZeroEffectiveFutureBounceGainArrivesEarlier) {
  BallPhysics venue_physics;
  venue_physics.table_tangential_gain = 0.369;
  BallPhysics one_shot_physics = venue_physics;
  one_shot_physics.table_tangential_gain = 0.0;
  PlannerConfig config;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  TableParams table;
  TrajectoryPredictor venue_predictor(venue_physics, config, table);
  TrajectoryPredictor one_shot_predictor(one_shot_physics, config, table);

  BallState state;
  state.position = Vec3(1.30, -0.55, 0.35);
  state.velocity = Vec3(-3.0, 0.15, -1.0);
  state.source_time_s = 10.0;
  state.valid = true;
  const auto venue = venue_predictor.predict_with_spin(
      state, 0.05, Vec3::Zero(), SpinPhysicsMode::kVenueGripBounce);
  const auto one_shot = one_shot_predictor.predict_with_spin(
      state, 0.05, Vec3::Zero(), SpinPhysicsMode::kVenueGripBounce);
  ASSERT_TRUE(venue.valid) << venue.reason;
  ASSERT_TRUE(one_shot.valid) << one_shot.reason;
  ASSERT_EQ(venue.predicted_bounces, 1);
  ASSERT_EQ(one_shot.predicted_bounces, 1);
  EXPECT_LT(one_shot.strike_source_time_s, venue.strike_source_time_s);
  EXPECT_LT(one_shot.ball_velocity.x(), venue.ball_velocity.x());
}

}  // namespace
}  // namespace hope_planner_cpp
