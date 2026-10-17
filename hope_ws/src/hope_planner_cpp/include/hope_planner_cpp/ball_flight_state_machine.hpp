#pragma once
#include "hope_planner_cpp/return_region.hpp"
#include <deque>
namespace hope_planner_cpp {
class BallFlightStateMachine {
public:
  explicit BallFlightStateMachine(ProbabilisticConfig config);
  void begin_rally(std::uint64_t rally_id, bool we_are_serving);
  void set_bounce_history_hint(int previous_bounces);
  void update(const BallSample *accepted_sample, const StateBelief &,
              const TrackerUpdate &);
  const FlightState &state() const { return state_; }
  const BounceContext &bounce_context() const { return history_; }
  void set_history_continuity(HistoryContinuity);
  void set_return_history_count_hint(int);
  void add_verified_contact(const ContactHistoryEvent &);

private:
  void new_track();
  ProbabilisticConfig config_;
  FlightState state_;
  BounceContext history_;
  bool complete_history_pending_ = false, verified_hint_pending_ = false;
  std::deque<BallSample> direction_, bounce_samples_;
  std::size_t incoming_confirmations_ = 0, outgoing_confirmations_ = 0;
  bool hint_pending_ = false, track_active_ = false;
  bool first_incoming_seen_ = false, return_departed_ = false;
};
} // namespace hope_planner_cpp
