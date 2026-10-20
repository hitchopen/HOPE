#include "hope_planner_cpp/adaptive_flight_commit.hpp"

#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hope_planner_cpp {

AdaptiveFlightCommit::AdaptiveFlightCommit(
    AdaptiveCommitConfig config, CommitPredictionConfig prediction)
    : config_(std::move(config)), prediction_(std::move(prediction)) {
  const auto valid_physics = [](const BallPhysics& p) {
    const auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
    return p.gravity.allFinite() && std::isfinite(p.drag_k) && p.drag_k >= 0.0 &&
        std::isfinite(p.radius) && p.radius > 0.0 && unit(p.restitution_h) &&
        unit(p.restitution_v) && unit(p.table_tangential_gain);
  };
  if (!std::isfinite(config_.policy_lead_s) || config_.policy_lead_s <= 0.0 ||
      !std::isfinite(config_.transport_budget_s) || config_.transport_budget_s < 0.0 ||
      !std::isfinite(config_.max_post_net_wait_s) || config_.max_post_net_wait_s <= 0.0 ||
      !std::isfinite(config_.post_bounce_span_s) || config_.post_bounce_span_s < 0.02 ||
      !std::isfinite(config_.contact_exclusion_s) || config_.contact_exclusion_s < 0.0 ||
      !std::isfinite(config_.evaluation_period_s) || config_.evaluation_period_s <= 0.0 ||
      !std::isfinite(prediction_.planner.x_hit) ||
      !valid_physics(prediction_.observed_physics) || !valid_physics(prediction_.future_physics)) {
    throw std::invalid_argument("invalid adaptive flight commit configuration");
  }
  // Validate before entering the noexcept callback path. A shorter contact
  // window is meaningful only when the receiver can reproduce the same fit.
  const BatchPhysicsEstimator validated_estimator(
      prediction_.observed_physics, prediction_.estimator, prediction_.spin_estimator);
  const FutureContactPredictor validated_predictor(prediction_.future_physics,
      prediction_.planner, prediction_.table, prediction_.future_contact);
  const SpinEstimator validated_spin(prediction_.spin_estimator);
  if ((prediction_.estimator.post_bounce_min_span_s <= 0.0 &&
       config_.post_bounce_span_s < prediction_.estimator.min_span_s) ||
      (prediction_.estimator.post_bounce_min_span_s > 0.0 &&
       (std::abs(config_.post_bounce_span_s -
                 prediction_.estimator.post_bounce_min_span_s) > 1.0e-9 ||
        std::abs(config_.contact_exclusion_s -
                 prediction_.estimator.post_bounce_exclusion_s) > 1.0e-9))) {
    throw std::invalid_argument("adaptive and estimator contact windows must agree");
  }
}

void AdaptiveFlightCommit::reset() noexcept {
  have_saved_ = false;
  baseline_seen_ = false;
  deadline_s_ = 0.0;
  fixed_commit_s_ = 0.0;
  last_evaluation_s_ = -std::numeric_limits<double>::infinity();
  bounce_source_s_ = -std::numeric_limits<double>::infinity();
}

void AdaptiveFlightCommit::set_x_hit(double x_hit) noexcept {
  if (std::isfinite(x_hit)) prediction_.planner.x_hit = x_hit;
}

void AdaptiveFlightCommit::finish(
    double now_s, const char* reason, TrajectorySnapshot& output) noexcept {
  output = saved_;
  output.one_shot.commit_source_time_s =
      std::max(output.one_shot.net_cross_source_time_s, now_s);
  output.commit_reason = reason;
  reset();
}

bool AdaptiveFlightCommit::poll(double now_s, TrajectorySnapshot& output) noexcept {
  if (!have_saved_ || !std::isfinite(now_s) || now_s < deadline_s_) return false;
  // The wall timer may run between exposure callbacks at the 50 ms boundary.
  // Evaluate the most recent causal sample before choosing whether to wait;
  // simply sending here would silently disable adaptation on the live node.
  if (!baseline_seen_) return evaluate(saved_, fixed_commit_s_, now_s, true, output);
  finish(now_s, "deadline_saved_prediction", output);
  return true;
}

bool AdaptiveFlightCommit::consider(
    const TrajectorySnapshot& candidate, double fixed_commit_s,
    double now_s, TrajectorySnapshot& output) noexcept {
  return evaluate(candidate, fixed_commit_s, now_s,
                  candidate.latest_sample() &&
                      candidate.latest_sample()->source_time_s >= fixed_commit_s,
                  output);
}

bool AdaptiveFlightCommit::evaluate(
    const TrajectorySnapshot& candidate, double fixed_commit_s,
    double now_s, bool boundary_due, TrajectorySnapshot& output) noexcept {
  if (!candidate.latest_sample() || !std::isfinite(now_s)) return false;
  const double source = candidate.latest_sample()->source_time_s;
  if (!have_saved_) {
    saved_ = candidate;
    have_saved_ = true;
    // Allow one preview period for the first exposure at/after the fixed
    // boundary. This preserves the normal 50 ms sample selection; if input
    // stalls, the wall timer still provides a bounded causal fallback.
    deadline_s_ = fixed_commit_s + config_.evaluation_period_s;
    fixed_commit_s_ = fixed_commit_s;
  }
  if (!baseline_seen_) saved_ = candidate;
  // Evaluate the exact first fixed-boundary sample even if it falls between
  // preview ticks. Fixed-mode numerical behavior remains reproducible.
  if (source - last_evaluation_s_ < config_.evaluation_period_s &&
      (baseline_seen_ || !boundary_due) && now_s < deadline_s_) return false;
  last_evaluation_s_ = source;
  BatchPhysicsEstimator estimator(prediction_.observed_physics, prediction_.estimator, prediction_.spin_estimator);
  for (std::size_t i = 0; i < candidate.sample_count; ++i) estimator.push(candidate.samples[i]);
  const BallState state = estimator.estimate();
  FutureContactPredictor predictor(prediction_.future_physics, prediction_.planner,
                                   prediction_.table, prediction_.future_contact);
  const auto spin_for = [this](const TrajectorySnapshot& snapshot) {
    SpinEstimator spin(prediction_.spin_estimator);
    for (std::size_t i = 0; i < snapshot.sample_count; ++i) spin.push(snapshot.samples[i]);
    return spin.estimate();
  };
  const StrikeTarget strike = predictor.predict(state, prediction_.planner.x_hit,
                                                spin_for(candidate));
  const double reserve = config_.policy_lead_s + config_.transport_budget_s;

  // The same estimator is used again on the HDU. Retain the contact and its
  // raw history in the packet so the receiver independently selects exactly
  // this measured post-contact fit, even when it spans less than the normal
  // pre-contact readiness window. Never substitute a synthetic state/sample.
  if (strike.valid && state.reason == "post_bounce_free_flight") {
    saved_ = candidate;
    finish(now_s, "observed_post_bounce", output);
    return true;
  }

  if (!baseline_seen_) {
    saved_ = candidate;
    if (strike.valid && strike.strike_source_time_s - now_s <= reserve) {
      finish(now_s, "short_lead_early", output);
      return true;
    }
    if (!boundary_due) return false;
    baseline_seen_ = true;
    if (!strike.valid) {
      finish(now_s, "fixed_boundary_no_preview", output);
      return true;
    }
    deadline_s_ = std::min(
        candidate.one_shot.net_cross_source_time_s + config_.max_post_net_wait_s,
        strike.strike_source_time_s - reserve);
    const double acceleration = -prediction_.observed_physics.gravity.z() +
        prediction_.observed_physics.drag_k * state.velocity.norm() * state.velocity.z();
    const double discriminant = state.velocity.z() * state.velocity.z() +
        2.0 * acceleration * (state.position.z() - prediction_.observed_physics.radius);
    const double until_bounce = acceleration > 0.0 && discriminant >= 0.0
        ? (state.velocity.z() + std::sqrt(discriminant)) / acceleration : -1.0;
    const double contact = state.bounce_epoch_active
        ? state.bounce_source_time_s : source + until_bounce;
    const double ready = contact + config_.post_bounce_span_s +
        config_.contact_exclusion_s + config_.evaluation_period_s + std::max(0.0, now_s - source);
    if ((!state.bounce_epoch_active && until_bounce <= 0.0) ||
        !std::isfinite(ready) || ready >= deadline_s_) {
      finish(now_s, "fixed_boundary_no_bounce_budget", output);
      return true;
    }
  }

  // The partial bounce fit can transiently predict a dead ball. It neither
  // replaces the saved usable packet nor extends the irrevocable send deadline.
  if (state.bounce_epoch_active && std::isfinite(state.bounce_source_time_s))
    bounce_source_s_ = state.bounce_source_time_s;
  if (strike.valid && !state.bounce_transition_used) {
    saved_ = candidate;
    deadline_s_ = std::min(deadline_s_, strike.strike_source_time_s - reserve);
  }
  if (prediction_.estimator.post_bounce_min_span_s <= 0.0 &&
      std::isfinite(bounce_source_s_) &&
      source >= bounce_source_s_ + config_.post_bounce_span_s + config_.contact_exclusion_s) {
    TrajectorySnapshot clean = candidate;
    clean.sample_count = 0;
    for (std::size_t i = 0; i < candidate.sample_count; ++i) {
      if (candidate.samples[i].source_time_s > bounce_source_s_ + config_.contact_exclusion_s)
        clean.samples[clean.sample_count++] = candidate.samples[i];
    }
    if (clean.sample_count >= prediction_.estimator.min_samples &&
        clean.latest_sample()->source_time_s - clean.samples[0].source_time_s >= config_.post_bounce_span_s) {
      BatchPhysicsEstimator post(prediction_.observed_physics, prediction_.estimator, prediction_.spin_estimator);
      for (std::size_t i = 0; i < clean.sample_count; ++i) post.push(clean.samples[i]);
      const StrikeTarget updated = predictor.predict(
          post.estimate(), prediction_.planner.x_hit, spin_for(clean));
      if (updated.valid) {
        saved_ = clean;
        finish(now_s, "observed_post_bounce", output);
        return true;
      }
    }
  }
  return poll(now_s, output);
}

}  // namespace hope_planner_cpp
