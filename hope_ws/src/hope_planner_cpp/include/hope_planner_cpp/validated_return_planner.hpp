#pragma once
#include "hope_planner_cpp/return_eligibility.hpp"

namespace hope_planner_cpp {
struct PaddleImpact {
  bool valid = false;
  Vec3 outgoing = Vec3::Zero();
  std::string reason = "IMPACT_UNSUPPORTED";
};
// Normal faces +X, from face toward approaching ball. Normal-only, constant e.
PaddleImpact reconstruct_paddle_impact(const Vec3 &incoming, const Vec3 &racket_velocity,
                                      const Vec3 &normal, double restitution);
struct ValidatedReturnSettings {
  Vec3 landing_surface{2.055, -.7625, 0};
  std::vector<double> durations_s{.50, .40, .60, .70};
  int max_iterations = 12;
  double endpoint_tolerance_m = 1e-5, landing_tolerance_m = .001;
  double time_tolerance_s = .0005, impact_tolerance_mps = 1e-8;
  double restitution = .654, max_racket_speed_mps = 8, max_ball_speed_mps = 20;
  void validate(const ProbabilisticConfig &) const;
};
struct ValidatedReturn {
  RacketCommand command;
  ModelState reconstructed;
  Vec3 first_landing = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  bool solve_ok = false, impact_ok = false, return_ok = false;
  double selected_duration_s = 0, landing_time_s = 0;
  double solver_residual_m = std::numeric_limits<double>::infinity();
  double impact_residual_mps = std::numeric_limits<double>::infinity();
  double net_clearance_m = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0, attempts = 0;
  std::string reason = "NOT_SOLVED", model_config_id;
};
class ValidatedReturnPlanner {
public:
  ValidatedReturnPlanner(ProbabilisticConfig, ValidatedReturnSettings = {});
  ValidatedReturn solve(const ContactPlanningInput &) const;
  // Public so callers can validate the representable action after conversion.
  bool validate_action(const ContactPlanningInput &, ValidatedReturn &) const;
private:
  Vec3 endpoint(const Vec3 &, const Vec3 &, double) const;
  ProbabilisticConfig config_;
  ValidatedReturnSettings settings_;
  std::string return_config_id_;
};
} // namespace hope_planner_cpp
