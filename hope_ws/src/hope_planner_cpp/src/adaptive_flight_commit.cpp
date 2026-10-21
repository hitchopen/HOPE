#include "hope_planner_cpp/adaptive_flight_commit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hope_planner_cpp {

AdaptiveFlightCommit::AdaptiveFlightCommit(
    AdaptiveCommitConfig config, CommitPredictionConfig prediction)
    : config_(std::move(config)), prediction_(std::move(prediction)),
      estimator_(prediction_.observed_physics, prediction_.estimator,
                 prediction_.spin_estimator), spin_(prediction_.spin_estimator) {
  const auto nonnegative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  const auto valid_physics = [](const BallPhysics& p) {
    const auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
    return p.gravity.allFinite() && std::isfinite(p.drag_k) && p.drag_k >= 0.0 &&
        std::isfinite(p.radius) && p.radius > 0.0 && unit(p.restitution_h) &&
        unit(p.restitution_v) && unit(p.table_tangential_gain);
  };
  if (!nonnegative(config_.policy_lead_s) || config_.policy_lead_s == 0.0 ||
      !nonnegative(config_.transport_budget_s) ||
      !nonnegative(config_.solve_budget_s) || !nonnegative(config_.policy_tick_s) ||
      !nonnegative(config_.max_sample_age_s) || config_.max_sample_age_s == 0.0 ||
      !nonnegative(config_.evaluation_period_s) || config_.evaluation_period_s == 0.0 ||
      !std::isfinite(prediction_.planner.x_hit) ||
      !valid_physics(prediction_.observed_physics) || !valid_physics(prediction_.future_physics))
    throw std::invalid_argument("invalid deadline flight commit configuration");
  // Validate both physics models before entering the noexcept callback path.
  const BatchPhysicsEstimator validate_future(prediction_.future_physics,
      prediction_.estimator, prediction_.spin_estimator);
  const FutureContactPredictor validate_predictor(prediction_.future_physics,
      prediction_.planner, prediction_.table, prediction_.future_contact);
  // The receiver gets a clean segment, not the contact history. Its ordinary
  // free-flight readiness must therefore admit the same exported sample span.
  if (prediction_.estimator.single_flight_only &&
      prediction_.estimator.post_bounce_min_span_s > 0.0 &&
      prediction_.estimator.post_bounce_min_span_s < prediction_.estimator.min_span_s)
    throw std::invalid_argument("clean contact packet needs matching receiver fit span");
}

void AdaptiveFlightCommit::reset() noexcept {
  estimator_.reset();
  spin_.reset();
  have_latest_ = false;
  have_saved_ = false;
  deadline_s_ = std::numeric_limits<double>::infinity();
  last_evaluation_s_ = last_pushed_s_ = -std::numeric_limits<double>::infinity();
}

void AdaptiveFlightCommit::set_x_hit(double x_hit) noexcept {
  if (std::isfinite(x_hit) && x_hit != prediction_.planner.x_hit) {
    prediction_.planner.x_hit = x_hit;
    last_evaluation_s_ = -std::numeric_limits<double>::infinity();
  }
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
  if (!have_latest_ || !std::isfinite(now_s) || now_s < deadline_s_) return false;
  // The final solve includes exposures received since the last preview tick.
  return evaluate(now_s, true, output);
}

bool AdaptiveFlightCommit::consider(
    const TrajectorySnapshot& candidate, double now_s,
    TrajectorySnapshot& output) noexcept {
  if (!candidate.latest_sample() || !std::isfinite(now_s)) return false;
  latest_ = candidate;
  have_latest_ = true;
  // Keep contact/segment bookkeeping across preview ticks; feed EVERY original
  // exposure, not only the last frame of each 10 ms evaluation interval.
  for (std::size_t i = 0; i < candidate.sample_count; ++i) {
    const auto& sample = candidate.samples[i];
    if (sample.source_time_s <= last_pushed_s_) continue;
    if (prediction_.estimator.max_sample_gap_s > 0.0 &&
        sample.source_time_s - last_pushed_s_ > prediction_.estimator.max_sample_gap_s) {
      have_saved_ = false;
      spin_.reset();
      last_evaluation_s_ = -std::numeric_limits<double>::infinity();
    }
    estimator_.push(sample);
    spin_.push(sample);
    last_pushed_s_ = sample.source_time_s;
  }
  return evaluate(now_s, now_s >= deadline_s_, output);
}

bool AdaptiveFlightCommit::evaluate(
    double now_s, bool force, TrajectorySnapshot& output) noexcept {
  const double source = latest_.latest_sample()->source_time_s;
  if (source > last_evaluation_s_ &&
      (force || source - last_evaluation_s_ + 1e-9 >= config_.evaluation_period_s)) {
    last_evaluation_s_ = source;
    const BallState state = estimator_.estimate();
    const FutureContactPredictor predictor(prediction_.future_physics,
        prediction_.planner, prediction_.table, prediction_.future_contact);
    const StrikeTarget strike = predictor.predict(state, prediction_.planner.x_hit,
                                                  spin_.estimate());
    if (strike.valid && state.sample_count > 0 &&
        state.sample_count <= latest_.sample_count) {
      saved_ = latest_;
      // Export only the raw samples actually fitted. In single-flight mode a
      // confirmed bounce starts a NEW window; the packet never straddles it.
      const std::size_t first = latest_.sample_count - state.sample_count;
      std::copy_n(latest_.samples.begin() + first, state.sample_count, saved_.samples.begin());
      saved_.sample_count = state.sample_count;
      have_saved_ = true;
      const double reserve = config_.policy_lead_s + config_.transport_budget_s +
          config_.solve_budget_s + config_.policy_tick_s;
      // Until submission, the latest valid physical prediction owns timing.
      // Invalid partial contact fits and source stalls cannot move this clock.
      deadline_s_ = strike.strike_source_time_s - reserve;
    }
  }
  if (!have_saved_ || now_s < deadline_s_) return false;
  const double age = now_s - saved_.latest_sample()->source_time_s;
  if (!std::isfinite(age) || age < -0.05 || age > config_.max_sample_age_s) {
    // No fabricated current timestamp and no stale target on a stalled stream.
    reset();
    return false;
  }
  finish(now_s, "hit_deadline", output);
  return true;
}

}  // namespace hope_planner_cpp
