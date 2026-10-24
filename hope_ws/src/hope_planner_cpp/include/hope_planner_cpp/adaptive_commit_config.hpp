#pragma once

#include "hope_planner_cpp/types.hpp"
#include "hope_planner_cpp/future_contact_predictor.hpp"

namespace hope_planner_cpp {

struct AdaptiveCommitConfig {
  bool enabled = false;
  // These budgets select when to send ONE target. They never change its
  // physical deadline, reject a short-lead ball, or modify an active swing.
  double policy_lead_s = 0.35;
  double transport_budget_s = 0.10;
  double solve_budget_s = 0.005;
  double policy_tick_s = 0.02;
  // A stalled source cannot turn an arbitrarily old fit into a final target.
  double max_sample_age_s = 0.05;
  double evaluation_period_s = 0.01;
};

struct CommitPredictionConfig {
  BallPhysics observed_physics;
  BallPhysics future_physics;
  EstimatorConfig estimator;
  SpinEstimatorConfig spin_estimator;
  FutureContactConfig future_contact;
  PlannerConfig planner;
  TableParams table;
};

}  // namespace hope_planner_cpp
