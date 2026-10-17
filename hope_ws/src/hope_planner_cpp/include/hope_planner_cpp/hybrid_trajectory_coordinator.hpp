#pragma once
#include "hope_planner_cpp/ball_flight_state_machine.hpp"
#include "hope_planner_cpp/nominal_event_forecast.hpp"
#include "hope_planner_cpp/probabilistic_ball_tracker.hpp"
#include <functional>
#include <optional>
namespace hope_planner_cpp {
struct ForecastUpdate {
  enum Kind { StateUpdate, Trajectory, Invalidation } kind = StateUpdate;
  StateBelief state;
  FlightState flight;
  TrackerUpdate update;
  BounceContext history;
  std::optional<NominalBallForecast> forecast;
  std::string reason, config_id;
  std::int64_t rollout_complete_steady_ns=0;
};
// Synchronous sequenced coordinator: one CKF; no forecast queue or worker.
// Clock mapping and registered-frame checks belong to the transport adapter.
class HybridTrajectoryCoordinator {
public:
  using Sink = std::function<bool(const ForecastUpdate &)>;
  HybridTrajectoryCoordinator(ProbabilisticConfig, ReturnRegionPolicy,
                              std::string frame, Sink, Sink projection_sink = {},
                              std::uint64_t plane_generation = 0,
                              std::optional<ForecastNoiseConfig> forecast_noise = std::nullopt);
  void begin(std::int64_t ns, std::uint64_t rally, bool serving, bool complete);
  void reset(std::int64_t ns, const std::string &reason);
  bool verified_contact(std::int64_t ns, std::uint64_t rally, ContactSide,
                        const Vec3 &, const std::string &source);
  void observe(BallSample, const std::string &frame, double source_age_s);
  void expire(std::int64_t now_source_ns);
  void invalidate(const std::string &, std::int64_t ns);
  const StateBelief &belief() const { return tracker_.belief(); }
  const FlightState &flight() const { return machine_.state(); }
  const std::string &config_id() const { return config_id_; }
private:
  double local_time(std::int64_t ns);
  StateBelief envelope(std::int64_t ns) const;
  FlightState output_flight() const;
  bool emit(ForecastUpdate);
  ProbabilisticConfig config_;
  ForecastNoiseConfig forecast_noise_;
  ReturnRegionPolicy policy_;
  std::string frame_, config_id_;
  Sink sink_, projection_sink_;
  std::uint64_t plane_generation_=0;
  ProbabilisticBallTracker tracker_;
  BallFlightStateMachine machine_;
  std::optional<std::int64_t> origin_, latest_ns_, last_observed_ns_;
  std::uint64_t forecast_epoch_=0,forecast_revision_=0;
  std::optional<std::int64_t> forecast_expiry_;
  double last_output_ = -1e300;
};
} // namespace hope_planner_cpp
