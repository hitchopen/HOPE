#pragma once

#include "hope_planner_cpp/adaptive_commit_config.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"

namespace hope_planner_cpp {

// Causal selection of a single immutable packet, before Runner sees the shot.
// Every exposure updates the rolling state. Only the predicted hit deadline
// schedules submission; neither net+delay nor measured contact triggers a send.
class AdaptiveFlightCommit {
 public:
  AdaptiveFlightCommit(AdaptiveCommitConfig config, CommitPredictionConfig prediction);
  void reset() noexcept;
  bool consider(const TrajectorySnapshot& candidate,
                double now_s, TrajectorySnapshot& output) noexcept;
  bool poll(double now_s, TrajectorySnapshot& output) noexcept;
  void set_x_hit(double x_hit) noexcept;

 private:
  bool evaluate(double now_s, bool force, TrajectorySnapshot& output) noexcept;
  void finish(double now_s, const char* reason, TrajectorySnapshot& output) noexcept;
  AdaptiveCommitConfig config_;
  CommitPredictionConfig prediction_;
  BatchPhysicsEstimator estimator_;
  SpinEstimator spin_;
  TrajectorySnapshot latest_;
  TrajectorySnapshot saved_;
  bool have_latest_ = false;
  bool have_saved_ = false;
  double deadline_s_ = std::numeric_limits<double>::infinity();
  double last_evaluation_s_ = -std::numeric_limits<double>::infinity();
  double last_pushed_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace hope_planner_cpp
