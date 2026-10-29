#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include "hope_planner_cpp/return_physics_yaml.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

TEST(TrajectoryAndStage3, PreservesIncomingReferenceAndSolvesFullReturn) {
  // Golden values use the 2026-08-23/24 engineering physics candidate.
  BallPhysics physics;
  physics.drag_k = 0.1317;
  physics.restitution_h = 0.64;
  physics.restitution_v = 0.9607;
  PlannerConfig config;
  config.x_hit = 0.15;
  config.target_land = Vec3(2.055, -0.7625, 0.0);
  config.delta_t_flight = 0.5;
  config.adaptive_predict_horizon = true;
  config.max_predict_time_cap_s = 3.0;
  load_return_physics_yaml(std::string(HOPE_PLANNER_CONFIG_DIR) + "/return_physics.yaml",
                           physics, config);
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
  EXPECT_NEAR(strike.ball_position.z(), 0.32482849618220022, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.x(), -2.2222423506951761, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.y(), 0.18518686255793151, 1e-8);
  EXPECT_NEAR(strike.ball_velocity.z(), 0.18474390147945227, 1e-8);
  EXPECT_NEAR(strike.strike_source_time_s, 123.41225305436849, 1e-10);
  EXPECT_EQ(strike.predicted_bounces, 1);

  const RacketCommand command = stage3.plan(
      strike, config.target_land, config.delta_t_flight);
  ASSERT_TRUE(command.valid) << command.reason;
  EXPECT_TRUE(command.clears_net);
  EXPECT_FALSE(command.bypasses_net_posts);
  const auto forward = stage3.evaluate_return(strike, command.velocity);
  ASSERT_TRUE(forward.valid);
  EXPECT_LE((forward.bounce_position - Vec3(config.target_land.x(), config.target_land.y(),
                                           physics.radius)).norm(), config.return_position_tolerance_m);
  EXPECT_NEAR(forward.flight_time_s, config.delta_t_flight, config.return_time_tolerance_s);
  EXPECT_EQ(command.outgoing_ball_velocity, forward.outgoing_velocity);
  EXPECT_EQ(command.outgoing_ball_spin_rad_s, forward.outgoing_spin_rad_s);
  EXPECT_GT(command.outgoing_ball_spin_rad_s.norm(), 0.0);
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
  // This research-path threshold is defined for the historical venue tuple.
  physics.drag_k = 0.1317;
  physics.restitution_h = 0.64;
  physics.restitution_v = 0.9607;
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
  // Preserve the independently computed historical scalar reference.
  physics.magnus_k = 0.00327844;
  physics.table_tangential_gain = 0.369;
  physics.table_friction_cap_mu = 2.0;
  physics.drag_k = 0.1317;
  physics.restitution_h = 0.64;
  physics.restitution_v = 0.9607;
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
  EXPECT_TRUE(result.ball_spin_rad_s.allFinite());
  EXPECT_GT((result.ball_spin_rad_s - Vec3(5.0, -35.0, 8.0)).norm(), 0.1);
  // Independent scalar Python port of contact_model.predict_contact plus the
  // 1 ms Stage-2 integrator. This anchors the full flight/contact/crossing
  // path, not merely finiteness of the C++ result.
  EXPECT_NEAR(result.ball_position.x(), 0.15, 1.0e-12);
  EXPECT_NEAR(result.ball_position.y(), -0.46261832412462123, 1.0e-9);
  EXPECT_NEAR(result.ball_position.z(), 0.31448053104194496, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.x(), -2.5251616994552961, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.y(), 0.13378372389668103, 1.0e-9);
  EXPECT_NEAR(result.ball_velocity.z(), 0.049669376072277609, 1.0e-9);
  EXPECT_NEAR(result.strike_source_time_s, 123.39237824750332, 1.0e-10);
}

TEST(TrajectoryAndStage3, FullReturnRejectsMissingCoefficientsAndNonconvergence) {
  BallPhysics physics;
  PlannerConfig config;
  StrikeTarget strike;
  strike.valid = true;
  strike.ball_position = Vec3(.1, -.7, .3);
  strike.ball_velocity = Vec3(-2.0, .2, -1.0);
  strike.ball_spin_rad_s = Vec3(20., -30., 10.);
  const Vec3 target(2.2, -.7, 0.0);
  const auto missing = RacketTargetPlanner(physics, config, {}).plan(strike, target, .5);
  EXPECT_FALSE(missing.valid);
  EXPECT_EQ(missing.reason, "full_return_parameters_invalid");
  load_return_physics_yaml(std::string(HOPE_PLANNER_CONFIG_DIR) + "/return_physics.yaml",
                           physics, config);
  config.return_newton_iterations = 1;
  config.return_alternative_flight_times.clear();
  const auto unfinished = RacketTargetPlanner(physics, config, {}).plan(strike, target, .5);
  EXPECT_FALSE(unfinished.valid);
  EXPECT_EQ(unfinished.reason, "full_return_solve_failed");
}

TEST(TrajectoryAndStage3, DisabledIncomingMagnusRetainsSpinForFullReturn) {
  BallPhysics physics;
  PlannerConfig config;
  load_return_physics_yaml(std::string(HOPE_PLANNER_CONFIG_DIR) + "/return_physics.yaml",
                           physics, config);
  BallState state;
  state.valid = true;
  state.position = Vec3(.8, -.7, .5);
  state.velocity = Vec3(-3., .1, -.2);
  const Vec3 spin(12., -24., 8.);
  const auto strike = TrajectoryPredictor(physics, config, {}).predict_with_spin(
      state, .7, spin, SpinPhysicsMode::kVenueGripBounce);
  ASSERT_TRUE(strike.valid);
  ASSERT_EQ(strike.predicted_bounces, 0);
  EXPECT_EQ(strike.ball_spin_rad_s, spin);
  const RacketTargetPlanner planner(physics, config, {});
  const auto with_spin = planner.evaluate_return(strike, Vec3(2., .2, .8));
  auto no_spin = strike;
  no_spin.ball_spin_rad_s.setZero();
  const auto zero = planner.evaluate_return(no_spin, Vec3(2., .2, .8));
  ASSERT_TRUE(with_spin.valid);
  ASSERT_TRUE(zero.valid);
  EXPECT_GT((with_spin.bounce_position - zero.bounce_position).norm(), 1e-3);
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

TEST(TrajectoryAndStage3, HistoricalUncappedZeroSpinRetentionMatchesDiagonalContact) {
  BallPhysics physics;
  // Disable flight forces to isolate the actual production contact dispatch.
  physics.drag_k = 0.0;
  physics.gravity = Vec3::Zero();
  physics.table_tangential_gain = 0.348260;
  physics.table_friction_cap_mu = 2.0;
  physics.restitution_v = 0.916598;
  PlannerConfig config;
  TableParams table;
  TrajectoryPredictor predictor(physics, config, table);
  BallState state;
  state.position = Vec3(1.0, -0.7, 0.12);
  state.velocity = Vec3(-2.0, 0.2, -1.0);
  state.source_time_s = 10.0;
  state.valid = true;
  const auto strike = predictor.predict_with_spin(
      state, 0.6, Vec3::Zero(), SpinPhysicsMode::kVenueGripBounce);
  ASSERT_TRUE(strike.valid) << strike.reason;
  ASSERT_EQ(strike.predicted_bounces, 1);
  EXPECT_NEAR(strike.ball_velocity.x(), -2.0 * 0.651740, 1e-9);
  EXPECT_NEAR(strike.ball_velocity.y(), 0.2 * 0.651740, 1e-9);
  EXPECT_NEAR(strike.ball_velocity.z(), 0.916598, 1e-9);
}

}  // namespace
}  // namespace hope_planner_cpp
