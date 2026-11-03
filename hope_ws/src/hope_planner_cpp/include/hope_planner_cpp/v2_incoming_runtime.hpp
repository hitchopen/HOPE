#pragma once
#include "hope_planner_cpp/v2_incoming_coordinator.hpp"
#include "hope_planner_cpp/flight_transport_v2.hpp"
#include "hope_msgs/msg/strike_candidate_v1.hpp"
#include "hope_planner_cpp/bounded_stage_trace.hpp"
#include <atomic>
#include <rclcpp/rclcpp.hpp>

namespace hope_planner_cpp {
// ROS I/O for the shared owner; all forecasts execute on PlannerNode's existing
// solver thread. The packet queue has priority over the replaceable preview.
class V2IncomingRuntime {
 public:
  V2IncomingRuntime(rclcpp::Node&, const BallPhysics&, const TableParams&,
                    std::function<void()> wake,
                    std::function<std::pair<std::uint64_t, bool>()> reference,
                    std::function<bool(const V2StrikeCandidate&)> revoke);
  ~V2IncomingRuntime();
  bool pending() const { return pending_.load(); }
  bool packet_ready(std::int64_t cutoff_ns) const {
    return latest_source_ns_.load() >= cutoff_ns ||
        static_cast<long double>(node_.now().nanoseconds()) - cutoff_ns >= 25000000.L;
  }
  void continuous(const IncomingPlane&);
  StrikeTarget packet(const TrajectorySnapshot&, const IncomingPlane&, BallState&);
  bool command(const std::function<bool(V2StrikeCandidate&)>& publication);
  void update_reference(const std::function<std::pair<std::uint64_t, bool>()>& update);
  const IncomingSourceContract& contract() const { return contract_; }
  IncomingCounters counters() const { return owner_->counters(); }
  std::uint64_t superseded() const { return superseded_.load(); }
  bool reference_matches(std::uint64_t calibration, std::uint64_t world) const {
    return calibration == base_calibration_ && world == base_world_;
  }
  static constexpr const char* prefix = "/hope/shadow/v2_incoming/";

 private:
  void flush_statuses();
  void candidate(const V2StrikeCandidate&);
  bool forecast(IncomingQuery&, IncomingEvaluation&);
  void trace(const char*, std::int64_t source, std::uint64_t revision,
             std::uint64_t generation, const std::string& reason = {},
             std::int64_t started_ns = 0);
  rclcpp::Node& node_;
  IncomingSourceContract contract_;
  std::unique_ptr<V2IncomingCoordinator> owner_;
  std::optional<IncomingQuery> command_query_;
  std::optional<IncomingEvaluation> command_evaluation_;
  std::atomic<bool> pending_{true};
  std::atomic<std::uint64_t> superseded_{0};
  std::atomic<std::int64_t> latest_source_ns_{0};
  std::atomic<std::int64_t> pending_source_ns_{0};
  std::optional<IncomingPlane> selected_plane_;
  std::uint64_t selected_generation_ = 0, base_calibration_ = 0, base_world_ = 0;
  std::mutex output_mutex_;
  BoundedStageTrace accounting_;
  std::function<bool(const V2StrikeCandidate&)> revoke_;
  std::uint64_t last_candidate_revision_ = 0, last_compact_revision_ = 0;
  bool continuous_output_ = true;
  double full_rate_hz_ = 50;
  std::int64_t next_full_ns_ = 0;
  std::string producer_;
  rclcpp::Subscription<hope_msgs::msg::BallObservationV2>::SharedPtr observations_;
  rclcpp::Publisher<hope_msgs::msg::FlightPredictionV2>::SharedPtr compact_, full_;
  rclcpp::Publisher<hope_msgs::msg::StrikeCandidateV1>::SharedPtr candidates_, revocations_;
  rclcpp::TimerBase::SharedPtr expiry_;
};
}  // namespace hope_planner_cpp
