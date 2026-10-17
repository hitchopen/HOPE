#pragma once
#include "hope_planner_cpp/strike_flight_detector.hpp"
#include "hope_msgs/msg/ball_forecast.hpp"
#include "hope_msgs/msg/hybrid_trajectory_context.hpp"
#include <deque>
#include <optional>
#include <set>

namespace hope_planner_cpp {
// Inclusive selected-contact bounds in the registered table frame, never a
// landing target or an interval recentered on the moving robot base.
bool strike_target_selector_hit_y_allowed(double contact_y, double min_y, double max_y);
struct SelectorFrozenPlane {
  double x = 0;
  std::uint64_t generation = 0, world = 0, calibration = 0;
  bool synthetic = false;
};
struct StrikeTargetSelectorConfig {
  StrikeFlightDetectorConfig incoming;
  BallPhysics physics;
  PlannerConfig planner = [] { PlannerConfig p; p.adaptive_predict_horizon=true; return p; }();
  EstimatorConfig estimator;
  std::optional<SelectorFrozenPlane> frozen_plane;
  std::string frame = "table", registration, clock = "ros_system", context_authority;
  bool synthetic = false;
  std::int64_t max_age_ns = 30000000, grace_ns = 10000000, min_lead_ns = 50000000;
};
struct StrikeTargetSelectorCandidate {
  StrikeTarget strike;
  std::int64_t absolute_hit_ns = 0;
  std::uint64_t revision = 0;
  std::string linearization_quality;
};
struct BestPlaneEstimate {
  StrikeTarget strike;
  std::int64_t absolute_hit_ns = 0, source_ns = 0, receipt_ns = 0;
  std::uint64_t flight_id = 0;
  hope_msgs::msg::BallForecast metadata;
};
struct StrikeTargetSelection {
  StrikeTarget strike;
  hope_msgs::msg::BallForecast forecast;  // compact provenance, empty for observation fits
  std::uint64_t physical_flight_id = 0, session_id = 1, plane_generation = 0;
  std::uint64_t world_frame_id = 0, calibration_id = 0;
  bool known_test_plane = false, fallback_used = false, late_acquisition = false;
  std::int64_t absolute_hit_ns = 0, source_ns = 0, observation_receipt_ns = 0;
  std::int64_t commit_ns = 0, net_ns = 0, selected_ns = 0, forecast_receipt_ns = 0;
  std::size_t sample_count = 0;
  double sample_span_s = 0, plane_x = 0, estimator_ms = 0, rollout_ms = 0;
  int verified_contacts = 0;
  std::string origin = "invalid", eligibility_status = "unknown", quality, reason;
};
// Callback-owned, bounded input history. A decision is driven by the net timer,
// never by arrival of a post-net snapshot. Legacy fits run once in poll().
class StrikeTargetSelector {
 public:
  explicit StrikeTargetSelector(StrikeTargetSelectorConfig config);
  void observe(BallSample, const std::string& frame, std::int64_t now_ns, std::int64_t steady_ns);
  bool forecast(const hope_msgs::msg::BallForecast&, std::int64_t receipt_ns);
  bool full_forecast(const hope_msgs::msg::BallForecast&, std::int64_t receipt_ns);
  void context(const hope_msgs::msg::HybridTrajectoryContext&, std::int64_t now_ns);
  std::optional<StrikeTargetSelection> poll(std::int64_t now_ns, std::int64_t steady_ns);
  static StrikeTargetSelectorCandidate extract(const hope_msgs::msg::BallForecast&, const StrikeTargetSelectorConfig&, std::int64_t);
  const std::string& reason() const { return reason_; }
  std::size_t cached() const { return cache_.size(); }
  bool halted() const { return halted_; }
  std::uint64_t session_id() const { return session_; }
  std::uint64_t flight_id() const { return flight_; }
  const char* phase() const { return incoming_.phase(); }
 private:
  void fence(const std::string&);
  bool establish_owner(const hope_msgs::msg::BallForecast&);
  bool authorized_reset_ack(const hope_msgs::msg::BallForecast&) const;
  void schedule(std::int64_t net_ns, std::int64_t now_ns, std::int64_t steady_ns);
  void classify_history(StrikeTargetSelection&);
  void fallback(StrikeTargetSelection&, std::int64_t now_ns);
  StrikeTargetSelection decision_metadata(std::int64_t now_ns) const;
  StrikeTargetSelectorConfig config_;
  StrikeFlightDetector incoming_;
  std::deque<BestPlaneEstimate> cache_;
  std::optional<BestPlaneEstimate> best_crossing_;
  struct SavedState { BallState state; std::int64_t source_ns, receipt_ns; hope_msgs::msg::BallForecast metadata; std::array<double,21> covariance; };
  std::deque<SavedState> states_;
  std::deque<BallSample> samples_;
  std::deque<std::int64_t> authorized_reset_sources_;
  std::deque<hope_msgs::msg::HybridTrajectoryContext> contacts_;
  std::set<std::string> retired_;
  std::optional<hope_msgs::msg::BallForecast> owner_;
  std::optional<SelectorFrozenPlane> plane_;
  std::optional<StrikeTargetSelection> terminal_;
  std::uint64_t event_ = 0, revision_ = 0, ckf_epoch_ = 0, full_event_ = 0, full_revision_ = 0, full_epoch_ = 0;
  std::uint64_t flight_ = 0, session_ = 1, rally_ = 0;
  std::int64_t origin_ = 0, last_pose_ = 0, last_forecast_ = 0, last_full_ = 0, start_ = 0,
      commit_ = 0, net_ = 0, deadline_ = 0, context_begin_ = 0, context_last_ = 0;
  Vec3 last_position_ = Vec3::Zero();
  bool pending_ = false, decided_ = false, halted_ = false, complete_ = false, contact_illegal_ = false,
      retirement_exhausted_ = false, late_acquisition_ = false;
  std::string reason_ = "waiting_for_flight";
};
double strike_target_selector_swing(double y, double base_y, double previous,
                                    double split = -.25, double hysteresis = .04);
}  // namespace hope_planner_cpp
