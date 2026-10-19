#pragma once

#include "hope_planner_cpp/common_ball_physics.hpp"
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
        spin_(spin_physics(fallback, config_), planner, table),
        physics_(fallback) {}

  bool uses_spin(const SpinEstimate &spin) const noexcept {
    return config_.spin_enabled &&
           (spin.effective.usable() ||
            (spin.valid && spin.omega_rad_s.allFinite() &&
             spin.omega_rad_s.squaredNorm() > 0 &&
             std::isfinite(spin.coherence) &&
             spin.coherence >= config_.min_coherence));
  }

  std::pair<BallPhysics, EffectiveSpin>
  model_snapshot(const BallState &state, const SpinEstimate &spin) const {
    auto physics = physics_;
    EffectiveSpin resolved;
    const auto &effective =
        state.spin_model_enabled ? state.physics_spin : spin;
    if (uses_spin(effective)) {
      resolved = effective.effective;
      resolved.omega = config_.omega_scale * (effective.effective.usable()
                                                  ? *effective.effective.omega
                                                  : effective.omega_rad_s);
      if (resolved.mode == EffectiveSpinMode::POSITION_ONLY)
        resolved.mode = EffectiveSpinMode::MEASURED;
      physics.table_tangential_gain = config_.tangential_gain;
      if (!config_.magnus_enabled)
        physics.magnus_k = 0;
    }
    return {physics, resolved};
  }

  StrikeTarget predict(const BallState &state, double x_hit,
                       const SpinEstimate &spin) const noexcept {
    const SpinEstimate &effective =
        state.spin_model_enabled ? state.physics_spin : spin;
    if (uses_spin(effective)) {
      return spin_.predict_with_spin(
          state, x_hit,
          config_.omega_scale * (effective.effective.usable()
                                     ? *effective.effective.omega
                                     : effective.omega_rad_s),
          config_.magnus_enabled ? SpinPhysicsMode::kVenueGripBounceAndMagnus
                                 : SpinPhysicsMode::kVenueGripBounce);
    }
    // Rejected raw vectors cannot enter the model. POSITION_ONLY has no angular
    // state.
    return fallback_.predict(state, x_hit);
  }

private:
  static FutureContactConfig validate(FutureContactConfig c) {
    const auto unit = [](double v) {
      return std::isfinite(v) && v >= 0.0 && v <= 1.0;
    };
    if (!(unit(c.tangential_gain) && c.tangential_gain <= .4) ||
        !unit(c.omega_scale) || !unit(c.min_coherence))
      throw std::invalid_argument("invalid future contact spin configuration");
    return c;
  }
  static BallPhysics spin_physics(BallPhysics p, const FutureContactConfig &c) {
    p.table_tangential_gain = c.tangential_gain;
    return p;
  }
  FutureContactConfig config_;
  TrajectoryPredictor fallback_;
  TrajectoryPredictor spin_;
  BallPhysics physics_;
};

} // namespace hope_planner_cpp
