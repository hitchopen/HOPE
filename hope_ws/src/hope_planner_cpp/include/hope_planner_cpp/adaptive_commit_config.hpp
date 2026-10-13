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
  double max_post_net_wait_s = 0.50;
  double post_bounce_span_s = 0.08;
  double contact_exclusion_s = 0.008;
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
