#pragma once

#include "hope_planner_cpp/adaptive_commit_config.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"

namespace hope_planner_cpp {

// Causal selection of a single immutable packet, before Runner sees the shot.
// Clean post-contact samples replace an unobserved bounce only when the initial
// prediction leaves room. No timer or fallback regenerates a strike deadline.
class AdaptiveFlightCommit {
 public:
  AdaptiveFlightCommit(AdaptiveCommitConfig config, CommitPredictionConfig prediction);
  void reset() noexcept;
  bool consider(const TrajectorySnapshot& candidate, double fixed_commit_s,
                double now_s, TrajectorySnapshot& output) noexcept;
  bool poll(double now_s, TrajectorySnapshot& output) noexcept;
  void set_x_hit(double x_hit) noexcept;

 private:
  bool evaluate(const TrajectorySnapshot& candidate, double fixed_commit_s,
                double now_s, bool boundary_due, TrajectorySnapshot& output) noexcept;
  void finish(double now_s, const char* reason, TrajectorySnapshot& output) noexcept;
  AdaptiveCommitConfig config_;
  CommitPredictionConfig prediction_;
  TrajectorySnapshot saved_;
  bool have_saved_ = false;
  bool baseline_seen_ = false;
  double deadline_s_ = 0.0;
  double fixed_commit_s_ = 0.0;
  double last_evaluation_s_ = -std::numeric_limits<double>::infinity();
  double bounce_source_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace hope_planner_cpp
