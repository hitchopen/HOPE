#pragma once
#include "hope_planner_cpp/probabilistic_types.hpp"
namespace hope_planner_cpp {
Matrix6 positive_covariance(const Matrix6 &covariance);
State6 free_flight(const State6 &state, double dt, const BallPhysics &physics, const EffectiveSpin& spin = {});
struct ContactStep {
  EffectiveSpin post_spin;
  bool bounced = false;
  double fraction = 0.;
  State6 contact = State6::Zero();
};
ContactStep physical_step(State6 &state, double dt,
                          const ProbabilisticConfig &config);
Matrix6 process_covariance(double dt, double spectral_density);
} // namespace hope_planner_cpp
