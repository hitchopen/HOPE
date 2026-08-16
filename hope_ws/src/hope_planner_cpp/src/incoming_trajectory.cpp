#include "hope_planner_cpp/incoming_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hope_planner_cpp {
namespace {

template <typename T, std::size_t Capacity>
void append_bounded(
    std::array<T, Capacity>& values,
    std::size_t& count,
    std::size_t limit,
    const T& value) noexcept {
  limit = std::clamp<std::size_t>(limit, 1, Capacity);
  if (count >= limit) {
    std::move(values.begin() + 1, values.begin() + static_cast<std::ptrdiff_t>(count),
              values.begin());
    --count;
  }
  values[count++] = value;
}

}  // namespace

const char* incoming_phase_name(IncomingPhase phase) noexcept {
  switch (phase) {
    case IncomingPhase::kSeekIncoming: return "seek_incoming";
    case IncomingPhase::kCollectIncoming: return "collect_incoming";
    case IncomingPhase::kWaitCommit: return "wait_commit";
    case IncomingPhase::kWaitOutgoing: return "wait_outgoing";
    case IncomingPhase::kWaitOpponentReturn: return "wait_opponent_return";
  }
  return "unknown";
}

IncomingTrajectory::IncomingTrajectory(IncomingTrajectoryConfig config)
    : config_(std::move(config)) {
  config_.estimator_window_s = std::max(0.02, config_.estimator_window_s);
  config_.commit_delay_s = std::max(0.0, config_.commit_delay_s);
  config_.rolling_snapshot_period_s =
      std::max(0.001, config_.rolling_snapshot_period_s);
  config_.rolling_snapshot_min_span_s = std::clamp(
      config_.rolling_snapshot_min_span_s,
      0.0,
      config_.estimator_window_s);
  config_.rolling_snapshot_min_samples = std::clamp<std::size_t>(
      config_.rolling_snapshot_min_samples, 2, kMaxEstimatorSamples);
  config_.opponent_side_margin_m = std::max(0.0, config_.opponent_side_margin_m);
  config_.incoming_speed_threshold_mps =
      std::max(0.01, config_.incoming_speed_threshold_mps);
  config_.outgoing_speed_threshold_mps =
      std::max(0.01, config_.outgoing_speed_threshold_mps);
  config_.source_gap_reset_s = std::max(0.02, config_.source_gap_reset_s);
  config_.direction_fit_samples = std::clamp<std::size_t>(
      config_.direction_fit_samples, 3, kMaxEstimatorSamples);
  config_.direction_confirmations = std::clamp<std::size_t>(
      config_.direction_confirmations, 1, 8);
  config_.pre_roll_samples = std::clamp<std::size_t>(
      config_.pre_roll_samples,
      config_.direction_fit_samples + config_.direction_confirmations,
      kMaxEstimatorSamples);
}

void IncomingTrajectory::reset_phase(bool keep_epoch) noexcept {
  phase_ = IncomingPhase::kSeekIncoming;
  pre_roll_count_ = 0;
  history_count_ = 0;
  incoming_confirmations_ = 0;
  outgoing_confirmations_ = 0;
  last_source_time_s_ = -std::numeric_limits<double>::infinity();
  net_cross_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
  commit_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
  last_snapshot_source_time_s_ =
      -std::numeric_limits<double>::infinity();
  current_segment_start_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  current_boundary_reason_ = "none";
  seek_after_source_reset_ = true;
  if (!keep_epoch) {
    trajectory_epoch_ = 0;
    snapshot_sequence_ = 0;
    previous_segment_last_source_time_s_ =
        std::numeric_limits<double>::quiet_NaN();
  }
}

void IncomingTrajectory::append_pre_roll(const BallSample& sample) noexcept {
  append_bounded(pre_roll_, pre_roll_count_, config_.pre_roll_samples, sample);
}

void IncomingTrajectory::append_history(const BallSample& sample) noexcept {
  append_bounded(history_, history_count_, kMaxEstimatorSamples, sample);
  trim_history();
}

void IncomingTrajectory::trim_history() noexcept {
  if (history_count_ < 2) return;
  const double cutoff = history_[history_count_ - 1].source_time_s -
      config_.estimator_window_s;
  std::size_t first = 0;
  while (first + 1 < history_count_ && history_[first].source_time_s < cutoff) {
    ++first;
  }
  if (first == 0) return;
  std::move(
      history_.begin() + static_cast<std::ptrdiff_t>(first),
      history_.begin() + static_cast<std::ptrdiff_t>(history_count_),
      history_.begin());
  history_count_ -= first;
}

double IncomingTrajectory::direction_slope_mps() const noexcept {
  if (pre_roll_count_ < config_.direction_fit_samples) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t first = pre_roll_count_ - config_.direction_fit_samples;
  const double origin = pre_roll_[first].source_time_s;
  double mean_t = 0.0;
  double mean_x = 0.0;
  for (std::size_t i = first; i < pre_roll_count_; ++i) {
    mean_t += pre_roll_[i].source_time_s - origin;
    mean_x += pre_roll_[i].position.x();
  }
  const double count = static_cast<double>(config_.direction_fit_samples);
  mean_t /= count;
  mean_x /= count;
  double covariance = 0.0;
  double variance = 0.0;
  for (std::size_t i = first; i < pre_roll_count_; ++i) {
    const double dt = pre_roll_[i].source_time_s - origin - mean_t;
    covariance += dt * (pre_roll_[i].position.x() - mean_x);
    variance += dt * dt;
  }
  return variance > 1.0e-12
      ? covariance / variance
      : std::numeric_limits<double>::quiet_NaN();
}

bool IncomingTrajectory::opponent_side_visible() const noexcept {
  const double minimum_x = config_.net_x + config_.opponent_side_margin_m;
  for (std::size_t i = 0; i < pre_roll_count_; ++i) {
    if (pre_roll_[i].position.x() >= minimum_x) return true;
  }
  return false;
}

void IncomingTrajectory::update_direction_confirmations(double slope_mps) noexcept {
  if (std::isfinite(slope_mps) &&
      slope_mps <= -config_.incoming_speed_threshold_mps) {
    ++incoming_confirmations_;
  } else {
    incoming_confirmations_ = 0;
  }
  if (std::isfinite(slope_mps) &&
      slope_mps >= config_.outgoing_speed_threshold_mps) {
    ++outgoing_confirmations_;
  } else {
    outgoing_confirmations_ = 0;
  }
}

std::size_t IncomingTrajectory::incoming_boundary_index() const noexcept {
  if (pre_roll_count_ == 0) return 0;
  std::size_t maximum_index = 0;
  double maximum_x = pre_roll_[0].position.x();
  for (std::size_t i = 1; i < pre_roll_count_; ++i) {
    if (pre_roll_[i].position.x() >= maximum_x) {
      maximum_x = pre_roll_[i].position.x();
      maximum_index = i;
    }
  }
  return maximum_index;
}

void IncomingTrajectory::start_incoming(
    const char* reason,
    IncomingTrajectoryUpdate& update) noexcept {
  const std::size_t boundary = incoming_boundary_index();
  history_count_ = 0;
  net_cross_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
  commit_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
  last_snapshot_source_time_s_ =
      -std::numeric_limits<double>::infinity();
  current_boundary_reason_ = reason;
  current_segment_start_source_time_s_ = pre_roll_count_ > 0
      ? pre_roll_[boundary].source_time_s
      : std::numeric_limits<double>::quiet_NaN();
  ++trajectory_epoch_;
  phase_ = IncomingPhase::kCollectIncoming;
  incoming_confirmations_ = 0;
  outgoing_confirmations_ = 0;
  seek_after_source_reset_ = false;
  update.incoming_started = true;
  for (std::size_t i = boundary; i < pre_roll_count_; ++i) {
    collect_sample(pre_roll_[i], update);
    if (update.snapshot_ready) break;
  }
  if (!update.snapshot_ready) maybe_make_rolling_snapshot(update);
}

void IncomingTrajectory::collect_sample(
    const BallSample& sample,
    IncomingTrajectoryUpdate& update) noexcept {
  if (history_count_ > 0 &&
      sample.source_time_s <= history_[history_count_ - 1].source_time_s) {
    return;
  }
  const bool crossing = history_count_ > 0 &&
      history_[history_count_ - 1].position.x() > config_.net_x &&
      sample.position.x() <= config_.net_x;
  if (crossing && phase_ == IncomingPhase::kCollectIncoming) {
    const BallSample& previous = history_[history_count_ - 1];
    const double denominator = previous.position.x() - sample.position.x();
    const double fraction = denominator > 1.0e-12
        ? std::clamp(
              (previous.position.x() - config_.net_x) / denominator,
              0.0, 1.0)
        : 1.0;
    net_cross_source_time_s_ = previous.source_time_s +
        fraction * (sample.source_time_s - previous.source_time_s);
    commit_source_time_s_ = net_cross_source_time_s_ + config_.commit_delay_s;
    phase_ = IncomingPhase::kWaitCommit;
    update.net_crossed = true;
  }
  append_history(sample);
  if (phase_ == IncomingPhase::kWaitCommit &&
      std::isfinite(commit_source_time_s_) &&
      sample.source_time_s + 1.0e-12 >= commit_source_time_s_) {
    make_snapshot(update, true);
  }
}

void IncomingTrajectory::maybe_make_rolling_snapshot(
    IncomingTrajectoryUpdate& update) noexcept {
  if (!config_.rolling_snapshots_enabled || update.snapshot_ready ||
      history_count_ < config_.rolling_snapshot_min_samples) {
    return;
  }
  const double latest = history_[history_count_ - 1].source_time_s;
  const double span = latest - history_[0].source_time_s;
  if (span + 1.0e-12 < config_.rolling_snapshot_min_span_s ||
      latest + 1.0e-12 <
          last_snapshot_source_time_s_ + config_.rolling_snapshot_period_s) {
    return;
  }
  make_snapshot(update, false);
}

void IncomingTrajectory::make_snapshot(
    IncomingTrajectoryUpdate& update,
    bool final_commit) noexcept {
  TrajectorySnapshot snapshot;
  snapshot.sample_count = history_count_;
  std::copy_n(history_.begin(), history_count_, snapshot.samples.begin());
  snapshot.trajectory_epoch = trajectory_epoch_;
  snapshot.snapshot_sequence = ++snapshot_sequence_;
  snapshot.segment_start_source_time_s = current_segment_start_source_time_s_;
  snapshot.previous_segment_last_source_time_s =
      previous_segment_last_source_time_s_;
  snapshot.segment_boundary_reason = current_boundary_reason_;
  snapshot.one_shot.commit_due = final_commit;
  snapshot.one_shot.flight_sequence = trajectory_epoch_;
  snapshot.one_shot.net_cross_source_time_s = net_cross_source_time_s_;
  snapshot.one_shot.commit_source_time_s = commit_source_time_s_;
  update.snapshot = std::move(snapshot);
  update.snapshot_ready = true;
  if (history_count_ > 0) {
    last_snapshot_source_time_s_ =
        history_[history_count_ - 1].source_time_s;
  }

  if (!final_commit) return;

  if (history_count_ > 0) {
    previous_segment_last_source_time_s_ =
        history_[history_count_ - 1].source_time_s;
  }
  history_count_ = 0;
  phase_ = IncomingPhase::kWaitOutgoing;
  incoming_confirmations_ = 0;
  outgoing_confirmations_ = 0;
}

IncomingTrajectoryUpdate IncomingTrajectory::observe(
    const BallSample& sample) noexcept {
  IncomingTrajectoryUpdate update;
  if (!std::isfinite(sample.source_time_s) || !sample.position.allFinite()) {
    return update;
  }

  if (std::isfinite(last_source_time_s_)) {
    const double gap = sample.source_time_s - last_source_time_s_;
    if (gap <= 0.0 || gap > config_.source_gap_reset_s) {
      reset_phase(true);
      update.source_epoch_reset = true;
    }
  }
  last_source_time_s_ = sample.source_time_s;
  append_pre_roll(sample);
  const double slope = direction_slope_mps();
  update_direction_confirmations(slope);

  if (phase_ == IncomingPhase::kSeekIncoming) {
    if (incoming_confirmations_ >= config_.direction_confirmations &&
        opponent_side_visible()) {
      start_incoming(
          seek_after_source_reset_ ? "source_epoch_incoming" : "initial_incoming",
          update);
    }
    return update;
  }

  if (phase_ == IncomingPhase::kCollectIncoming) {
    // A stable +X flight that has reached the opponent side is positive
    // evidence that an uncommitted incoming candidate was not the current
    // return. This is intentionally not a timeout and cannot rearm a missed
    // ball that keeps moving toward the robot.
    if (outgoing_confirmations_ >= config_.direction_confirmations &&
        sample.position.x() >= config_.net_x + config_.opponent_side_margin_m) {
      history_count_ = 0;
      net_cross_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
      commit_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
      phase_ = IncomingPhase::kWaitOpponentReturn;
      incoming_confirmations_ = 0;
      return update;
    }
    collect_sample(sample, update);
    if (!update.snapshot_ready) maybe_make_rolling_snapshot(update);
    return update;
  }

  if (phase_ == IncomingPhase::kWaitCommit) {
    collect_sample(sample, update);
    if (!update.snapshot_ready) maybe_make_rolling_snapshot(update);
    return update;
  }

  if (phase_ == IncomingPhase::kWaitOutgoing) {
    if (outgoing_confirmations_ >= config_.direction_confirmations &&
        sample.position.x() >= config_.net_x + config_.opponent_side_margin_m) {
      phase_ = IncomingPhase::kWaitOpponentReturn;
      incoming_confirmations_ = 0;
    }
    return update;
  }

  if (phase_ == IncomingPhase::kWaitOpponentReturn &&
      incoming_confirmations_ >= config_.direction_confirmations &&
      opponent_side_visible()) {
    start_incoming("opponent_turnaround", update);
  }
  return update;
}

bool LatestSnapshotMailbox::publish(
    const TrajectorySnapshot& snapshot) noexcept {
  bool replaced = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replaced = has_pending_;
    pending_ = snapshot;
    has_pending_ = true;
  }
  published_.fetch_add(1, std::memory_order_relaxed);
  if (replaced) superseded_.fetch_add(1, std::memory_order_relaxed);
  return replaced;
}

bool LatestSnapshotMailbox::try_take(TrajectorySnapshot& snapshot) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pending_) return false;
    snapshot = pending_;
    has_pending_ = false;
  }
  consumed_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool LatestSnapshotMailbox::has_pending() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_pending_;
}

}  // namespace hope_planner_cpp
