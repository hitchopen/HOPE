#include "hope_planner_cpp/future_contact_predictor.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace hope_planner_cpp {
namespace {
BallState incoming_state() {
  BallState s;
  s.valid = true;
  s.source_time_s = 100.;
  s.position = Vec3(1.2, -0.7, 0.2);
  s.velocity = Vec3(-2.5, 0., -1.);
  return s;
}
BallPhysics physics() {
  BallPhysics p;
  p.drag_k = 0.;
  p.table_tangential_gain = .2;
  p.restitution_v = .96;
  // This fixture exercises uncapped spin sensitivity, independently of the
  // currently fitted material cap (whose active branch has a separate test).
  p.table_friction_cap_mu = 2.0;
  return p;
}
TEST(FutureContactPredictor, MissingAndIncoherentSpinPreserveLegacyPrediction) {
  FutureContactConfig c; c.spin_enabled = true;
  PlannerConfig planner; planner.adaptive_predict_horizon = true;
  FutureContactPredictor predictor(physics(), planner, {}, c);
  const auto expected = TrajectoryPredictor(physics(), planner, {}).predict_with_spin(
      incoming_state(), .4, Vec3::Zero(), SpinPhysicsMode::kVenueGripBounce);
  ASSERT_TRUE(expected.valid);
  for (int kind = 0; kind < 3; ++kind) {
    SpinEstimate spin;
    spin.valid = kind != 0;
    spin.coherence = kind == 2 ? 1. : .39;
    spin.omega_rad_s = Vec3(0., 50., 0.);
    if (kind == 2) spin.omega_rad_s.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(predictor.uses_spin(spin));
    auto actual = predictor.predict(incoming_state(), .4, spin);
    ASSERT_TRUE(actual.valid);
    EXPECT_DOUBLE_EQ(actual.strike_source_time_s, expected.strike_source_time_s);
    EXPECT_EQ(actual.ball_position, expected.ball_position);
  }
}
TEST(FutureContactPredictor, SpinChangesContactTimeInPhysicalDirection) {
  FutureContactConfig c; c.spin_enabled = true;
  PlannerConfig planner; planner.adaptive_predict_horizon = true;
  FutureContactPredictor predictor(physics(), planner, {}, c);
  SpinEstimate spin; spin.valid = true; spin.coherence = .9;
  spin.omega_rad_s = Vec3(0., 40., 0.);
  auto slower = predictor.predict(incoming_state(), .4, spin);
  spin.omega_rad_s.y() = -40.;
  auto faster = predictor.predict(incoming_state(), .4, spin);
  ASSERT_TRUE(slower.valid); ASSERT_TRUE(faster.valid);
  EXPECT_GT(slower.strike_source_time_s, faster.strike_source_time_s + .03);
  // No future table contact means spin must not introduce a Magnus term.
  auto s = incoming_state(); s.position.z() = .8; s.velocity.z() = 1.;
  auto a = predictor.predict(s, .9, spin);
  spin.omega_rad_s = Vec3(100., 100., 100.);
  auto b = predictor.predict(s, .9, spin);
  ASSERT_TRUE(a.valid); ASSERT_TRUE(b.valid);
  EXPECT_DOUBLE_EQ(a.strike_source_time_s, b.strike_source_time_s);
  EXPECT_EQ(a.ball_position, b.ball_position);
}
TEST(FutureContactPredictor, DisabledModeAndInvalidConfiguration) {
  PlannerConfig planner; planner.adaptive_predict_horizon = true;
  SpinEstimate spin; spin.valid = true; spin.coherence = 1.;
  spin.omega_rad_s = Vec3(0., 40., 0.);
  FutureContactPredictor predictor(physics(), planner, {});
  EXPECT_FALSE(predictor.uses_spin(spin));
  FutureContactConfig c; c.omega_scale = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FutureContactPredictor(physics(), planner, {}, c), std::invalid_argument);
  c.omega_scale = 2.;
  EXPECT_THROW(FutureContactPredictor(physics(), planner, {}, c), std::invalid_argument);
}
}  // namespace
}  // namespace hope_planner_cpp
