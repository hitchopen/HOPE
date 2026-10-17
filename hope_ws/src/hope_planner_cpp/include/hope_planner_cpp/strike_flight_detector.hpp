#pragma once
#include "hope_planner_cpp/types.hpp"
#include <deque>
namespace hope_planner_cpp {
struct StrikeFlightDetectorConfig {
  double net_x=1.37, commit_delay_s=.05;
};
struct StrikeFlightUpdate {
  bool incoming_started=false, net_crossed=false, source_epoch_reset=false, late_acquisition=false;
  std::int64_t segment_start_ns=0, net_cross_ns=0;
};
// Source-stamped X direction and net timing only; no trajectory or swing model.
class StrikeFlightDetector {
 public:
  explicit StrikeFlightDetector(StrikeFlightDetectorConfig c);
  StrikeFlightUpdate observe(const BallSample&);
  void reset_phase();
  const char* phase() const { return active_ ? "incoming" : "seek_incoming"; }
 private:
  struct Point {double x; std::int64_t ns;};
  StrikeFlightDetectorConfig config_;
  std::deque<Point> points_;
  std::int64_t last_=0, start_=0, net_=0;
  unsigned negative_=0, positive_=0;
  bool active_=false, crossed_=false;
};
} // namespace hope_planner_cpp
