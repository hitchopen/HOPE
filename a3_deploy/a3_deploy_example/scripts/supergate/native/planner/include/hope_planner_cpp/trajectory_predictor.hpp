#pragma once

#include "hope_planner_cpp/types.hpp"

namespace hope_planner_cpp {

class TrajectoryPredictor {
 public:
  TrajectoryPredictor(BallPhysics physics, PlannerConfig config, TableParams table);

  StrikeTarget predict(const BallState& state, double x_hit) const noexcept;

  // Spin-aware research path used by offline replay and the runtime shadow.
  // kLegacyNoSpin dispatches to predict() exactly.  Callers decide whether a
  // shadow is publishable; this method never changes command admission.
  StrikeTarget predict_with_spin(
      const BallState& state,
      double x_hit,
      const Vec3& omega_rad_s,
      SpinPhysicsMode mode) const noexcept;

 private:
  double prediction_horizon_s(const BallState& state, double x_hit) const noexcept;
  Vec3 flight_acceleration(
      const Vec3& velocity,
      const Vec3& omega_rad_s,
      bool use_magnus) const noexcept;
  void apply_nakashima_bounce(
      const Vec3& velocity_in,
      const Vec3& omega_in,
      Vec3* velocity_out,
      Vec3* omega_out) const noexcept;
  void apply_venue_grip_bounce(
      const Vec3& velocity_in,
      const Vec3& omega_in,
      Vec3* velocity_out,
      Vec3* omega_out) const noexcept;

  BallPhysics physics_;
  PlannerConfig config_;
  TableParams table_;
};

}  // namespace hope_planner_cpp
