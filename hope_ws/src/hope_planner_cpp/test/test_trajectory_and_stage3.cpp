#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

TEST(TrajectoryAndStage3,
     SharedReceivingPhysicsAndUnmodifiedLegacyStage3RemainFinite) {
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
  auto fine_config = config;
  fine_config.integrate_dt_s = .00005;
  const auto reference = TrajectoryPredictor(physics, fine_config, table)
                             .predict(state, config.x_hit);
  ASSERT_TRUE(reference.valid);
  EXPECT_LT((strike.ball_position - reference.ball_position).norm(), .001);
  EXPECT_LT((strike.ball_velocity - reference.ball_velocity).norm(), .005);
  EXPECT_NEAR(strike.strike_source_time_s, reference.strike_source_time_s,
              .0005);
  EXPECT_EQ(strike.predicted_bounces, 1);

  const RacketCommand command =
      stage3.plan(strike, config.target_land, config.delta_t_flight);
  ASSERT_TRUE(command.valid) << command.reason;
  EXPECT_TRUE(command.clears_net);
  EXPECT_FALSE(command.bypasses_net_posts);
  EXPECT_NEAR(command.normal.norm(), 1., 1e-12);
  EXPECT_TRUE(command.velocity.allFinite());
  EXPECT_TRUE(command.outgoing_ball_velocity.allFinite());
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
      state, 0.15, Vec3(20.0, -30.0, 10.0), SpinPhysicsMode::kLegacyNoSpin);
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
  const auto topspin =
      predictor.predict_with_spin(state, 0.15, Vec3(0.0, -40.0, 0.0),
                                  SpinPhysicsMode::kNakashimaBounceAndMagnus);
  ASSERT_TRUE(no_spin.valid);
  ASSERT_TRUE(topspin.valid) << topspin.reason;
  EXPECT_EQ(topspin.predicted_bounces, 1);
  EXPECT_GT((topspin.ball_position - no_spin.ball_position).norm(), 0.005);
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
  const auto result =
      predictor.predict_with_spin(state, 0.15, Vec3(5.0, -35.0, 8.0),
                                  SpinPhysicsMode::kVenueGripBounceAndMagnus);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.ball_position.allFinite());
  EXPECT_TRUE(result.ball_velocity.allFinite());
  EXPECT_EQ(result.predicted_bounces, 1);
  // RK4/shared thin-shell profile supersedes historical Euler scalar goldens.
  // An independent DOP853 gate checks the profile, not just this refinement.
  config.integrate_dt_s = .00005;
  const auto reference =
      TrajectoryPredictor(physics, config, table)
          .predict_with_spin(state, .15, Vec3(5., -35., 8.),
                             SpinPhysicsMode::kVenueGripBounceAndMagnus);
  ASSERT_TRUE(reference.valid);
  EXPECT_LT((result.ball_position - reference.ball_position).norm(), .001);
  EXPECT_LT((result.ball_velocity - reference.ball_velocity).norm(), .005);
  EXPECT_NEAR(result.strike_source_time_s, reference.strike_source_time_s,
              .0005);
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

TEST(TrajectoryAndStage3,
     HistoricalUncappedZeroSpinRetentionMatchesDiagonalContact) {
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

} // namespace
} // namespace hope_planner_cpp

TEST(ScopedProduction, FixedBallInputPreservesOriginalStage3ScalarGoldens) {
  using namespace hope_planner_cpp;
  BallPhysics physics;
  physics.drag_k = .1317;
  physics.restitution_h = .64;
  physics.restitution_v = .9607;
  StrikeTarget strike;
  strike.valid = true;
  strike.ball_position = Vec3(.15, -.4458333333333333, .32482849618220022);
  strike.ball_velocity =
      Vec3(-2.2222423506951761, .18518686255793151, .18474390147945227);
  strike.strike_source_time_s = 123.41225305436849;
  strike.predicted_bounces = 1;
  const auto command = RacketTargetPlanner(physics, {}, {})
                           .plan(strike, Vec3(2.055, -.7625, 0), .5);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.velocity.x(), 2.0019087745012918, 1e-8);
  EXPECT_NEAR(command.velocity.y(), -.27676397921076445, 1e-8);
  EXPECT_NEAR(command.velocity.z(), .50034781141521933, 1e-8);
  EXPECT_NEAR(command.normal.x(), .96154701177380075, 1e-8);
  EXPECT_NEAR(command.normal.y(), -.13293391815170605, 1e-8);
  EXPECT_NEAR(command.normal.z(), .24032460871435926, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.x(), 4.3617531685177013, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.y(), -.72505081770656621, 1e-8);
  EXPECT_NEAR(command.outgoing_ball_velocity.z(), 1.8303172614475833, 1e-8);
}
