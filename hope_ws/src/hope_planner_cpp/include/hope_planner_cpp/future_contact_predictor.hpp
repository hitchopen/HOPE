#pragma once

#include "hope_planner_cpp/trajectory_predictor.hpp"
#include <cmath>
#include <stdexcept>

namespace hope_planner_cpp {

struct FutureContactConfig {
  bool spin_enabled = false;
  bool magnus_enabled = false;
  double tangential_gain = BallPhysics{}.table_tangential_gain;
  double omega_scale = 1.0;
  double min_coherence = 0.4;
};

// Shared by the causal packet preview and the receiving Planner. Unusable
// orientation selects the existing effective zero-spin model, never a reject.
// Magnus is opt-in and shares the estimator's segment-consistent spin.
// No change to an already published strike deadline.
class FutureContactPredictor {
 public:
  FutureContactPredictor(BallPhysics fallback, PlannerConfig planner,
                         TableParams table, FutureContactConfig config = {})
      : config_(validate(config)), fallback_(fallback, planner, table),
        spin_(spin_physics(fallback, config_), planner, table) {}

  bool uses_spin(const SpinEstimate& spin) const noexcept {
    return config_.spin_enabled && spin.valid && spin.omega_rad_s.allFinite() &&
        std::isfinite(spin.coherence) && spin.coherence >= config_.min_coherence;
  }

  StrikeTarget predict(const BallState& state, double x_hit,
                       const SpinEstimate& spin) const noexcept {
    const SpinEstimate& effective = state.spin_model_enabled ? state.physics_spin : spin;
    if (uses_spin(effective)) {
      return spin_.predict_with_spin(state, x_hit,
          config_.omega_scale * effective.omega_rad_s,
          config_.magnus_enabled ? SpinPhysicsMode::kVenueGripBounceAndMagnus
                                 : SpinPhysicsMode::kVenueGripBounce);
    }
    // A measured contact can create spin even when the incoming orientation
    // was unavailable. Preserve that zero-prior impulse from the state fit.
    const Vec3 prior = state.spin_model_enabled ? effective.omega_rad_s : Vec3::Zero();
    return fallback_.predict_with_spin(state, x_hit, prior,
        config_.magnus_enabled ? SpinPhysicsMode::kVenueGripBounceAndMagnus
                               : SpinPhysicsMode::kVenueGripBounce);
  }

 private:
  static FutureContactConfig validate(FutureContactConfig c) {
    const auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
    if (!unit(c.tangential_gain) || !unit(c.omega_scale) || !unit(c.min_coherence))
      throw std::invalid_argument("invalid future contact spin configuration");
    return c;
  }
  static BallPhysics spin_physics(BallPhysics p, const FutureContactConfig& c) {
    p.table_tangential_gain = c.tangential_gain;
    return p;
  }
  FutureContactConfig config_;
  TrajectoryPredictor fallback_;
  TrajectoryPredictor spin_;
};

}  // namespace hope_planner_cpp
