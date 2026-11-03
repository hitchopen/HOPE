#pragma once
#include "hope_planner_cpp/flight_state_v2.hpp"

namespace hope_planner_cpp {
// Immutable application handoff. Game history remains evidence, never a gate.
struct ContactPlanningInput {
  ShotToken token;
  HitPlaneSnapshot plane;
  BallSample source;
  PhysicalEvent contact;
  Matrix7 covariance = Matrix7::Constant(std::numeric_limits<double>::quiet_NaN());
  std::int64_t arrival_ns = 0, expires_ns = 0;
  std::uint64_t event_id = 0;
  std::size_t observation_count = 0;
  double observation_span_s = 0;
  HistoryContinuity history = HistoryContinuity::kUnknown;
  MatchContext game_context = MatchContext::UNKNOWN;
  bool bounce_observed = false;
  std::string model_config_id;
};
struct ReturnEligibility {
  bool eligible = false;
  std::string reason = "RETURN_UNSUPPORTED";
  std::optional<ContactPlanningInput> input;
};
ReturnEligibility select_physical_return(const FlightPrediction &, const ShotSnapshot &,
                                        const ProbabilisticConfig &, std::int64_t now_ns);
} // namespace hope_planner_cpp
