#pragma once
#include "hope_planner_cpp/probabilistic_physics.hpp"
#include "hope_planner_cpp/forecast_noise_config.hpp"
#include <limits>
#include <optional>
#include <vector>
namespace hope_planner_cpp {
using Matrix7 = Eigen::Matrix<double,7,7>;
enum class LinearizationQuality { Valid, Unavailable, BranchSensitive, IllConditioned };
const char *quality_name(LinearizationQuality);
struct ForecastWaypoint {
  ForecastCovarianceBudget<6> covariance_budget;
  double dt_s=0.;
  bool physical_continuation=true;
  State6 state=State6::Zero();
  Matrix6 covariance=Matrix6::Constant(std::numeric_limits<double>::quiet_NaN());
  LinearizationQuality quality=LinearizationQuality::Unavailable;
};
struct ForecastPhysicsEvent {
  ForecastCovarianceBudget<7> covariance_budget;
  std::string kind;
  double dt_s=0.;
  State6 pre=State6::Zero(), post=State6::Zero();
  Matrix7 covariance=Matrix7::Constant(std::numeric_limits<double>::quiet_NaN());
  LinearizationQuality quality=LinearizationQuality::Unavailable;
  double time_std_s=std::numeric_limits<double>::quiet_NaN();
  bool physically_usable=false;
};
struct NominalBallForecast {
  bool available=false;
  StateBelief source;
  std::vector<ForecastWaypoint> waypoints;
  std::vector<ForecastPhysicsEvent> events;
  std::optional<ForecastPhysicsEvent> hit_plane;
  std::string termination_reason="uninitialized";
  std::string uncertainty_provenance="local_linearization_v1;frozen_ckf_noise;uncalibrated;clock_and_plane_unavailable";
};
Matrix6 free_flight_jacobian(const State6 &, double, const BallPhysics &);
Matrix6 physical_step_jacobian(const State6 &, double, const ProbabilisticConfig &,
                              bool &branch_changed);
Matrix6 bounce_saltation(const State6 &pre, const ProbabilisticConfig &);
Matrix7 spatial_event_covariance(const State6 &, const Matrix6 &, int axis,
                                 const BallPhysics &);
NominalBallForecast forecast_nominal(const StateBelief &, const ProbabilisticConfig &);
NominalBallForecast forecast_nominal(const StateBelief &, const ProbabilisticConfig &, const ForecastNoiseConfig &);
} // namespace hope_planner_cpp
