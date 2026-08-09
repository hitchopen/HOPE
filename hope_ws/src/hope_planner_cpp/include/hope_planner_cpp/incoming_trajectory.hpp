#pragma once

#include "hope_planner_cpp/post_net_one_shot.hpp"
#include "hope_planner_cpp/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>

namespace hope_planner_cpp {

enum class IncomingPhase : std::uint8_t {
  kSeekIncoming = 0,
  kCollectIncoming = 1,
  kWaitCommit = 2,
  kWaitOutgoing = 3,
  kWaitOpponentReturn = 4,
};

const char* incoming_phase_name(IncomingPhase phase) noexcept;

struct IncomingTrajectoryConfig {
  double net_x = 1.37;
  double estimator_window_s = 0.18;
  double commit_delay_s = 0.05;
  double opponent_side_margin_m = 0.05;
  double incoming_speed_threshold_mps = 0.25;
  double outgoing_speed_threshold_mps = 0.25;
  double source_gap_reset_s = 0.25;
  std::size_t direction_fit_samples = 4;
  std::size_t direction_confirmations = 2;
  std::size_t pre_roll_samples = 24;
};

// Immutable, source-time-causal input to the one-shot solver. The callback
// creates it at the first sample on/after net crossing + commit_delay. The
// solver never observes samples that arrived after this boundary.
struct TrajectorySnapshot {
  std::array<BallSample, kMaxEstimatorSamples> samples{};
  std::size_t sample_count = 0;
  std::uint64_t trajectory_epoch = 0;
  std::uint64_t snapshot_sequence = 0;
  double segment_start_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double previous_segment_last_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
  const char* segment_boundary_reason = "none";
  PostNetOneShotEvent one_shot;

  const BallSample* latest_sample() const noexcept {
    return sample_count == 0 ? nullptr : &samples[sample_count - 1];
  }
};

struct IncomingTrajectoryUpdate {
  bool source_epoch_reset = false;
  bool incoming_started = false;
  bool net_crossed = false;
  bool snapshot_ready = false;
  TrajectorySnapshot snapshot;
};

// Callback-owned flight segmenter. Ignored outgoing samples are retained only
// in a small pre-roll used to identify the opponent-side X turnaround. They
// never enter the estimator history. An incoming segment stays latched through
// direction noise; only stable +X motion on the opponent side can abandon an
// uncommitted candidate before its net crossing.
class IncomingTrajectory {
 public:
  explicit IncomingTrajectory(IncomingTrajectoryConfig config);

  IncomingTrajectoryUpdate observe(const BallSample& sample) noexcept;
  void reset_phase(bool keep_epoch = true) noexcept;

  IncomingPhase phase() const noexcept { return phase_; }
  std::uint64_t trajectory_epoch() const noexcept { return trajectory_epoch_; }
  std::uint64_t snapshot_count() const noexcept { return snapshot_sequence_; }
  std::size_t retained_samples() const noexcept { return history_count_; }

 private:
  void append_pre_roll(const BallSample& sample) noexcept;
  void append_history(const BallSample& sample) noexcept;
  void trim_history() noexcept;
  double direction_slope_mps() const noexcept;
  bool opponent_side_visible() const noexcept;
  void update_direction_confirmations(double slope_mps) noexcept;
  std::size_t incoming_boundary_index() const noexcept;
  void start_incoming(
      const char* reason,
      IncomingTrajectoryUpdate& update) noexcept;
  void collect_sample(
      const BallSample& sample,
      IncomingTrajectoryUpdate& update) noexcept;
  void make_snapshot(IncomingTrajectoryUpdate& update) noexcept;

  IncomingTrajectoryConfig config_;
  IncomingPhase phase_ = IncomingPhase::kSeekIncoming;
  std::array<BallSample, kMaxEstimatorSamples> pre_roll_{};
  std::size_t pre_roll_count_ = 0;
  std::array<BallSample, kMaxEstimatorSamples> history_{};
  std::size_t history_count_ = 0;
  std::size_t incoming_confirmations_ = 0;
  std::size_t outgoing_confirmations_ = 0;
  std::uint64_t trajectory_epoch_ = 0;
  std::uint64_t snapshot_sequence_ = 0;
  double last_source_time_s_ =
      -std::numeric_limits<double>::infinity();
  double net_cross_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double commit_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double current_segment_start_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double previous_segment_last_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  const char* current_boundary_reason_ = "none";
  bool seek_after_source_reset_ = false;
};

// One pending immutable solve, latest flight wins. Publishing a newer flight
// replaces an older pending snapshot instead of rejecting the newest data.
class LatestSnapshotMailbox {
 public:
  bool publish(const TrajectorySnapshot& snapshot) noexcept;
  bool try_take(TrajectorySnapshot& snapshot) noexcept;
  bool has_pending() const noexcept;

  std::uint64_t published() const noexcept { return published_.load(); }
  std::uint64_t consumed() const noexcept { return consumed_.load(); }
  std::uint64_t superseded() const noexcept { return superseded_.load(); }

 private:
  mutable std::mutex mutex_;
  TrajectorySnapshot pending_;
  bool has_pending_ = false;
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> consumed_{0};
  std::atomic<std::uint64_t> superseded_{0};
};

}  // namespace hope_planner_cpp
