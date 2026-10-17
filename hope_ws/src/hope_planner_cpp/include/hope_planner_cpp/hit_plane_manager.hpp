#pragma once
#include "hope_planner_cpp/types.hpp"
#include <deque>
#include <optional>
#include <vector>
namespace hope_planner_cpp {
struct HitPlaneSettings {
  double offset_m=.58, window_s=.5, max_age_s=.2, max_span_m=.01;
  double min_x_m=-.30, max_x_m=.30;
  std::size_t min_samples=10;
  void validate() const;
};
struct HitPlaneSnapshot {
  double x_hit_m=0., base_x_m=0., offset_m=.58;
  std::string frame_id, registration_id;
  std::int64_t source_ns=0, receipt_ns=0, activated_source_ns=0;
  std::uint64_t calibration_id=0, world_frame_id=0, generation=0, rally_id=0;
  bool calibrated=false, test_plane=false;
  std::string reason="not_calibrated";
};
class HitPlaneManager {
public:
  HitPlaneManager(HitPlaneSettings, std::string frame, std::string registration,
                  std::uint64_t expected_world_id);
  bool observe(const std::vector<double>& wire, std::int64_t receipt_steady_ns,
               std::int64_t now_source_ns, bool registered_clock);
  bool freeze(std::int64_t now_steady_ns, std::int64_t now_source_ns,
              std::uint64_t rally_id, bool safe_boundary);
  void use_test_plane(double x, std::int64_t now_source_ns);
  void revoke(const std::string& reason);
  const HitPlaneSnapshot& snapshot() const { return frozen_; }
  const std::string& reason() const { return reason_; }
private:
  struct Sample { double x; std::int64_t receipt, source; std::uint64_t calibration; };
  HitPlaneSettings settings_;
  HitPlaneSnapshot frozen_;
  std::uint64_t world_id_, last_sequence_=0;
  std::int64_t last_source_=0;
  std::deque<Sample> samples_;
  std::string reason_="not_calibrated";
};
} // namespace hope_planner_cpp
