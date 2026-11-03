#pragma once

#include "hope_planner_cpp/v2_strike_adapter.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include <functional>
#include <set>

namespace hope_planner_cpp {
struct IncomingSourceContract {
  std::string session, frame, registration, clock_domain, clock_mapping, ball;
  std::size_t history_capacity = 512, checkpoint_capacity = 8;
  std::int64_t checkpoint_period_ns = 10000000;
};
struct IncomingQuery {
  FlightPipeline pipeline;
  IncomingPlane plane;
  std::uint64_t semantic_epoch = 0, revision = 0;
  bool command = false;
  std::string packet_identity;
};
struct IncomingEvaluation {
  FlightPrediction forecast;
  V2StrikeCandidate candidate;
};
struct IncomingCounters {
  std::uint64_t offered = 0, qualified = 0, tracker_accepted = 0, tracker_rejected = 0;
  std::uint64_t queries = 0, completed = 0, obsolete = 0, commands = 0, revoked = 0;
  std::size_t history = 0, checkpoints = 0;
  std::int64_t latest_source_ns = 0;
};

// One owner of physical state. Query copies are immutable source-cutoff views,
// not independent live trackers. Callers schedule at most one running query and
// a replaceable latest pending request; command queries have priority.
class V2IncomingCoordinator {
 public:
  V2IncomingCoordinator(ProbabilisticConfig, ShotSettings, IncomingSourceContract);
  bool observe(const BallSample&, std::int64_t now_ns);
  IncomingQuery continuous(const IncomingPlane&);
  std::optional<IncomingQuery> packet(const TrajectorySnapshot&, const IncomingPlane&,
                                     std::int64_t now_ns, std::string& reason);
  static IncomingEvaluation evaluate(IncomingQuery&, std::int64_t now_ns);

  // Publication is inside the authority lock so an observed revocation and a
  // valid publication have a defined ordering. Callback must not reenter owner.
  bool publish(const IncomingQuery&, IncomingEvaluation&, std::int64_t now_ns,
               const std::function<bool(V2StrikeCandidate&)>& publication);
  void revoke(const std::string& reason);
  bool update_reference(const std::function<std::pair<std::uint64_t, bool>()>& update);
  void expire(std::int64_t now_ns);
  std::vector<V2StrikeCandidate> take_statuses();
  IncomingCounters counters() const;
  ShotSnapshot state() const;

 private:
  struct Checkpoint { std::int64_t source_ns; FlightPipeline pipeline; };
  IncomingQuery query(FlightPipeline, const IncomingPlane&, bool command);
  void revoke_locked(const std::string&);
  void unavailable_locked(const std::string&, IncomingStatus);
  bool current(const IncomingQuery&, std::int64_t now_ns) const;
  static bool same_sample(const BallSample&, const BallSample&);
  mutable std::mutex mutex_;
  IncomingSourceContract contract_;
  FlightPipeline pipeline_;
  std::deque<BallSample> history_;
  std::deque<Checkpoint> checkpoints_;
  std::deque<V2StrikeCandidate> statuses_;
  std::optional<V2StrikeCandidate> latest_candidate_, command_candidate_;
  std::optional<IncomingPlane> latest_plane_;
  std::deque<std::pair<std::uint64_t, std::uint64_t>> claimed_flights_;
  std::string packet_producer_;
  std::uint64_t last_packet_sequence_ = 0, epoch_ = 1, revision_ = 0, published_revision_ = 0;
  std::uint64_t last_command_revision_ = 0;
  bool source_fault_ = false, source_expired_ = false, status_overflow_ = false;
  bool reference_received_ = false, reference_valid_ = false;
  std::uint64_t home_epoch_ = 0;
  IncomingCounters counters_;
};
}  // namespace hope_planner_cpp
