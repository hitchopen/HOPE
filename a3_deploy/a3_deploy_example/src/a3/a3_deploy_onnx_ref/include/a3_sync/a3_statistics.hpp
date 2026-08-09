// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Simplified slice of aimrl_sdk::Statistics (sync path only). Publish-side
// stats (publish_*, commit_total, wait_frame_*) are intentionally omitted —
// they will return with PR 6. See notes/a3_backend_plan.md §8 / PR 3.
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace a3_sync {

class A3SyncStatistics {
 public:
  struct ChannelHealth {
    bool ok{false};
    bool stale{false};
    bool interpolated{false};
    bool held{false};
    bool source_stamp_valid{false};
    std::int64_t age_ns{-1};
    std::int64_t source_stamp_ns{-1};
  };

  struct LatencyBucket {
    std::uint64_t count{0};
    std::int64_t total_ns{0};
    std::int64_t min_ns{0};
    std::int64_t max_ns{0};
  };

  struct LatencySnapshot {
    LatencyBucket state_transport_apparent{};
    LatencyBucket state_ready{};
    LatencyBucket state_header_skew{};
    LatencyBucket group_pair_skew{};
    std::uint64_t resample_interpolated{0};
    std::uint64_t resample_held{0};
  };

  struct Snapshot {
    std::uint64_t tick_total{0};
    std::uint64_t frame_complete_total{0};
    std::uint64_t frame_aligned_total{0};
    // Per-channel "no usable sample at tick" counters.
    std::uint64_t missing_waist{0};
    std::uint64_t missing_leg{0};
    std::uint64_t missing_arm{0};
    std::uint64_t missing_neck{0};
    std::uint64_t missing_pelvis_imu{0};
    std::uint64_t missing_torso_imu{0};
    // Per-channel "sample existed but exceeded max_sample_age_ns" counters.
    std::uint64_t stale_waist{0};
    std::uint64_t stale_leg{0};
    std::uint64_t stale_arm{0};
    std::uint64_t stale_neck{0};
    std::uint64_t stale_pelvis_imu{0};
    std::uint64_t stale_torso_imu{0};
    // Most recent measured values for operator dashboards.
    std::int64_t last_skew_ns{0};
    std::int64_t last_tick_interval_ns{0};
    std::int64_t last_age_waist_ns{-1};
    std::int64_t last_age_leg_ns{-1};
    std::int64_t last_age_arm_ns{-1};
    std::int64_t last_age_neck_ns{-1};
    std::int64_t last_age_pelvis_imu_ns{-1};
    std::int64_t last_age_torso_imu_ns{-1};
    // Most recent raw samples as received by subscribers, before sync-loop
    // bracket selection or interpolation.
    bool latest_waist_stamp_valid{false};
    bool latest_leg_stamp_valid{false};
    bool latest_arm_stamp_valid{false};
    bool latest_neck_stamp_valid{false};
    bool latest_pelvis_imu_stamp_valid{false};
    bool latest_torso_imu_stamp_valid{false};
    std::int64_t latest_waist_stamp_ns{0};
    std::int64_t latest_leg_stamp_ns{0};
    std::int64_t latest_arm_stamp_ns{0};
    std::int64_t latest_neck_stamp_ns{0};
    std::int64_t latest_pelvis_imu_stamp_ns{0};
    std::int64_t latest_torso_imu_stamp_ns{0};
    bool latest_waist_sequence_valid{false};
    bool latest_leg_sequence_valid{false};
    bool latest_arm_sequence_valid{false};
    bool latest_neck_sequence_valid{false};
    std::uint32_t latest_waist_sequence{0};
    std::uint32_t latest_leg_sequence{0};
    std::uint32_t latest_arm_sequence{0};
    std::uint32_t latest_neck_sequence{0};
  };

  A3SyncStatistics() = default;

  void OnTick(std::int64_t tick_ns, bool complete, bool aligned,
              std::int64_t skew_ns,
              ChannelHealth waist, ChannelHealth leg, ChannelHealth arm,
              ChannelHealth neck, ChannelHealth pelvis,
              ChannelHealth torso) noexcept {
    const auto prev_tick = last_tick_ns_.exchange(tick_ns,
                                                  std::memory_order_relaxed);
    if (prev_tick > 0) {
      last_tick_interval_ns_.store(tick_ns - prev_tick,
                                   std::memory_order_relaxed);
    }
    tick_total_.fetch_add(1, std::memory_order_relaxed);
    if (complete) frame_complete_total_.fetch_add(1, std::memory_order_relaxed);
    if (aligned)  frame_aligned_total_.fetch_add(1, std::memory_order_relaxed);
    if (!waist.ok && !waist.stale)   missing_waist_.fetch_add(1,  std::memory_order_relaxed);
    if (!leg.ok && !leg.stale)       missing_leg_.fetch_add(1,    std::memory_order_relaxed);
    if (!arm.ok && !arm.stale)       missing_arm_.fetch_add(1,    std::memory_order_relaxed);
    if (!neck.ok && !neck.stale)     missing_neck_.fetch_add(1,   std::memory_order_relaxed);
    if (!pelvis.ok && !pelvis.stale) missing_pelvis_.fetch_add(1, std::memory_order_relaxed);
    if (!torso.ok && !torso.stale)   missing_torso_.fetch_add(1,  std::memory_order_relaxed);
    if (waist.stale)  stale_waist_.fetch_add(1,  std::memory_order_relaxed);
    if (leg.stale)    stale_leg_.fetch_add(1,    std::memory_order_relaxed);
    if (arm.stale)    stale_arm_.fetch_add(1,    std::memory_order_relaxed);
    if (neck.stale)   stale_neck_.fetch_add(1,   std::memory_order_relaxed);
    if (pelvis.stale) stale_pelvis_.fetch_add(1, std::memory_order_relaxed);
    if (torso.stale)  stale_torso_.fetch_add(1,  std::memory_order_relaxed);
    last_skew_ns_.store(skew_ns, std::memory_order_relaxed);
    last_age_waist_ns_.store(waist.age_ns, std::memory_order_relaxed);
    last_age_leg_ns_.store(leg.age_ns, std::memory_order_relaxed);
    last_age_arm_ns_.store(arm.age_ns, std::memory_order_relaxed);
    last_age_neck_ns_.store(neck.age_ns, std::memory_order_relaxed);
    last_age_pelvis_ns_.store(pelvis.age_ns, std::memory_order_relaxed);
    last_age_torso_ns_.store(torso.age_ns, std::memory_order_relaxed);
  }

  void RecordStateTransportApparentNs(std::int64_t ns) noexcept {
    RecordLatency_(state_transport_count_, state_transport_total_ns_,
                   state_transport_min_ns_, state_transport_max_ns_, ns);
  }

  void RecordStateReadyNs(std::int64_t ns) noexcept {
    if (ns < 0) return;
    RecordLatency_(state_ready_count_, state_ready_total_ns_,
                   state_ready_min_ns_, state_ready_max_ns_, ns);
  }

  void RecordStateHeaderSkewNs(std::int64_t ns) noexcept {
    if (ns < 0) return;
    RecordLatency_(state_header_skew_count_, state_header_skew_total_ns_,
                   state_header_skew_min_ns_, state_header_skew_max_ns_, ns);
  }

  void RecordGroupPairSkewNs(std::int64_t ns) noexcept {
    if (ns < 0) return;
    RecordLatency_(group_pair_skew_count_, group_pair_skew_total_ns_,
                   group_pair_skew_min_ns_, group_pair_skew_max_ns_, ns);
  }

  void RecordResampleSamples(std::uint64_t interpolated,
                             std::uint64_t held) noexcept {
    resample_interpolated_.fetch_add(interpolated, std::memory_order_relaxed);
    resample_held_.fetch_add(held, std::memory_order_relaxed);
  }

  void RecordLatestWaistRaw(bool stamp_valid, std::int64_t stamp_ns,
                            bool sequence_valid,
                            std::uint32_t sequence) noexcept {
    RecordLatestRaw_(latest_waist_stamp_valid_, latest_waist_stamp_ns_,
                     latest_waist_sequence_valid_, latest_waist_sequence_,
                     stamp_valid, stamp_ns, sequence_valid, sequence);
  }

  void RecordLatestLegRaw(bool stamp_valid, std::int64_t stamp_ns,
                          bool sequence_valid,
                          std::uint32_t sequence) noexcept {
    RecordLatestRaw_(latest_leg_stamp_valid_, latest_leg_stamp_ns_,
                     latest_leg_sequence_valid_, latest_leg_sequence_,
                     stamp_valid, stamp_ns, sequence_valid, sequence);
  }

  void RecordLatestArmRaw(bool stamp_valid, std::int64_t stamp_ns,
                          bool sequence_valid,
                          std::uint32_t sequence) noexcept {
    RecordLatestRaw_(latest_arm_stamp_valid_, latest_arm_stamp_ns_,
                     latest_arm_sequence_valid_, latest_arm_sequence_,
                     stamp_valid, stamp_ns, sequence_valid, sequence);
  }

  void RecordLatestNeckRaw(bool stamp_valid, std::int64_t stamp_ns,
                           bool sequence_valid,
                           std::uint32_t sequence) noexcept {
    RecordLatestRaw_(latest_neck_stamp_valid_, latest_neck_stamp_ns_,
                     latest_neck_sequence_valid_, latest_neck_sequence_,
                     stamp_valid, stamp_ns, sequence_valid, sequence);
  }

  void RecordLatestPelvisImuRaw(bool stamp_valid,
                                std::int64_t stamp_ns) noexcept {
    latest_pelvis_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
    latest_pelvis_stamp_valid_.store(stamp_valid, std::memory_order_release);
  }

  void RecordLatestTorsoImuRaw(bool stamp_valid,
                               std::int64_t stamp_ns) noexcept {
    latest_torso_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
    latest_torso_stamp_valid_.store(stamp_valid, std::memory_order_release);
  }

  Snapshot snapshot() const noexcept {
    Snapshot s;
    s.tick_total             = tick_total_.load(std::memory_order_relaxed);
    s.frame_complete_total   = frame_complete_total_.load(std::memory_order_relaxed);
    s.frame_aligned_total    = frame_aligned_total_.load(std::memory_order_relaxed);
    s.missing_waist          = missing_waist_.load(std::memory_order_relaxed);
    s.missing_leg            = missing_leg_.load(std::memory_order_relaxed);
    s.missing_arm            = missing_arm_.load(std::memory_order_relaxed);
    s.missing_neck           = missing_neck_.load(std::memory_order_relaxed);
    s.missing_pelvis_imu     = missing_pelvis_.load(std::memory_order_relaxed);
    s.missing_torso_imu      = missing_torso_.load(std::memory_order_relaxed);
    s.stale_waist            = stale_waist_.load(std::memory_order_relaxed);
    s.stale_leg              = stale_leg_.load(std::memory_order_relaxed);
    s.stale_arm              = stale_arm_.load(std::memory_order_relaxed);
    s.stale_neck             = stale_neck_.load(std::memory_order_relaxed);
    s.stale_pelvis_imu       = stale_pelvis_.load(std::memory_order_relaxed);
    s.stale_torso_imu        = stale_torso_.load(std::memory_order_relaxed);
    s.last_skew_ns           = last_skew_ns_.load(std::memory_order_relaxed);
    s.last_tick_interval_ns  = last_tick_interval_ns_.load(std::memory_order_relaxed);
    s.last_age_waist_ns      = last_age_waist_ns_.load(std::memory_order_relaxed);
    s.last_age_leg_ns        = last_age_leg_ns_.load(std::memory_order_relaxed);
    s.last_age_arm_ns        = last_age_arm_ns_.load(std::memory_order_relaxed);
    s.last_age_neck_ns       = last_age_neck_ns_.load(std::memory_order_relaxed);
    s.last_age_pelvis_imu_ns = last_age_pelvis_ns_.load(std::memory_order_relaxed);
    s.last_age_torso_imu_ns  = last_age_torso_ns_.load(std::memory_order_relaxed);
    s.latest_waist_stamp_valid = latest_waist_stamp_valid_.load(std::memory_order_acquire);
    s.latest_leg_stamp_valid = latest_leg_stamp_valid_.load(std::memory_order_acquire);
    s.latest_arm_stamp_valid = latest_arm_stamp_valid_.load(std::memory_order_acquire);
    s.latest_neck_stamp_valid = latest_neck_stamp_valid_.load(std::memory_order_acquire);
    s.latest_pelvis_imu_stamp_valid = latest_pelvis_stamp_valid_.load(std::memory_order_acquire);
    s.latest_torso_imu_stamp_valid = latest_torso_stamp_valid_.load(std::memory_order_acquire);
    s.latest_waist_stamp_ns = latest_waist_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_leg_stamp_ns = latest_leg_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_arm_stamp_ns = latest_arm_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_neck_stamp_ns = latest_neck_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_pelvis_imu_stamp_ns = latest_pelvis_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_torso_imu_stamp_ns = latest_torso_stamp_ns_.load(std::memory_order_relaxed);
    s.latest_waist_sequence_valid = latest_waist_sequence_valid_.load(std::memory_order_acquire);
    s.latest_leg_sequence_valid = latest_leg_sequence_valid_.load(std::memory_order_acquire);
    s.latest_arm_sequence_valid = latest_arm_sequence_valid_.load(std::memory_order_acquire);
    s.latest_neck_sequence_valid = latest_neck_sequence_valid_.load(std::memory_order_acquire);
    s.latest_waist_sequence = latest_waist_sequence_.load(std::memory_order_relaxed);
    s.latest_leg_sequence = latest_leg_sequence_.load(std::memory_order_relaxed);
    s.latest_arm_sequence = latest_arm_sequence_.load(std::memory_order_relaxed);
    s.latest_neck_sequence = latest_neck_sequence_.load(std::memory_order_relaxed);
    return s;
  }

  LatencySnapshot ConsumeLatencySnapshot() noexcept {
    LatencySnapshot out;
    out.state_transport_apparent = ConsumeLatency_(
        state_transport_count_, state_transport_total_ns_,
        state_transport_min_ns_, state_transport_max_ns_);
    out.state_ready = ConsumeLatency_(
        state_ready_count_, state_ready_total_ns_,
        state_ready_min_ns_, state_ready_max_ns_);
    out.state_header_skew = ConsumeLatency_(
        state_header_skew_count_, state_header_skew_total_ns_,
        state_header_skew_min_ns_, state_header_skew_max_ns_);
    out.group_pair_skew = ConsumeLatency_(
        group_pair_skew_count_, group_pair_skew_total_ns_,
        group_pair_skew_min_ns_, group_pair_skew_max_ns_);
    out.resample_interpolated =
        resample_interpolated_.exchange(0, std::memory_order_acq_rel);
    out.resample_held =
        resample_held_.exchange(0, std::memory_order_acq_rel);
    return out;
  }

  void Reset() noexcept {
    tick_total_.store(0, std::memory_order_relaxed);
    frame_complete_total_.store(0, std::memory_order_relaxed);
    frame_aligned_total_.store(0, std::memory_order_relaxed);
    missing_waist_.store(0, std::memory_order_relaxed);
    missing_leg_.store(0, std::memory_order_relaxed);
    missing_arm_.store(0, std::memory_order_relaxed);
    missing_neck_.store(0, std::memory_order_relaxed);
    missing_pelvis_.store(0, std::memory_order_relaxed);
    missing_torso_.store(0, std::memory_order_relaxed);
    stale_waist_.store(0, std::memory_order_relaxed);
    stale_leg_.store(0, std::memory_order_relaxed);
    stale_arm_.store(0, std::memory_order_relaxed);
    stale_neck_.store(0, std::memory_order_relaxed);
    stale_pelvis_.store(0, std::memory_order_relaxed);
    stale_torso_.store(0, std::memory_order_relaxed);
    last_skew_ns_.store(0, std::memory_order_relaxed);
    last_tick_ns_.store(0, std::memory_order_relaxed);
    last_tick_interval_ns_.store(0, std::memory_order_relaxed);
    last_age_waist_ns_.store(-1, std::memory_order_relaxed);
    last_age_leg_ns_.store(-1, std::memory_order_relaxed);
    last_age_arm_ns_.store(-1, std::memory_order_relaxed);
    last_age_neck_ns_.store(-1, std::memory_order_relaxed);
    last_age_pelvis_ns_.store(-1, std::memory_order_relaxed);
    last_age_torso_ns_.store(-1, std::memory_order_relaxed);
    ResetLatestRaw_();
    ConsumeLatencySnapshot();
  }

 private:
  static constexpr std::int64_t kLatencyMinSentinel =
      std::numeric_limits<std::int64_t>::max();
  static constexpr std::int64_t kLatencyMaxSentinel =
      std::numeric_limits<std::int64_t>::min();

  static void RecordLatency_(std::atomic<std::uint64_t>& count,
                             std::atomic<std::int64_t>& total,
                             std::atomic<std::int64_t>& min_value,
                             std::atomic<std::int64_t>& max_value,
                             std::int64_t ns) noexcept {
    total.fetch_add(ns, std::memory_order_relaxed);

    auto old_min = min_value.load(std::memory_order_relaxed);
    while (ns < old_min &&
           !min_value.compare_exchange_weak(old_min, ns,
                                            std::memory_order_release,
                                            std::memory_order_relaxed)) {
    }
    auto old_max = max_value.load(std::memory_order_relaxed);
    while (ns > old_max &&
           !max_value.compare_exchange_weak(old_max, ns,
                                            std::memory_order_release,
                                            std::memory_order_relaxed)) {
    }
    count.fetch_add(1, std::memory_order_release);
  }

  static LatencyBucket ConsumeLatency_(
      std::atomic<std::uint64_t>& count,
      std::atomic<std::int64_t>& total,
      std::atomic<std::int64_t>& min_value,
      std::atomic<std::int64_t>& max_value) noexcept {
    LatencyBucket out;
    out.count = count.exchange(0, std::memory_order_acq_rel);
    out.total_ns = total.exchange(0, std::memory_order_acq_rel);
    out.min_ns = min_value.exchange(kLatencyMinSentinel,
                                    std::memory_order_acq_rel);
    out.max_ns = max_value.exchange(kLatencyMaxSentinel,
                                    std::memory_order_acq_rel);
    if (out.count == 0 || out.min_ns == kLatencyMinSentinel ||
        out.max_ns == kLatencyMaxSentinel) {
      out = {};
    }
    return out;
  }

  static void RecordLatestRaw_(std::atomic<bool>& stamp_valid_dst,
                               std::atomic<std::int64_t>& stamp_dst,
                               std::atomic<bool>& sequence_valid_dst,
                               std::atomic<std::uint32_t>& sequence_dst,
                               bool stamp_valid, std::int64_t stamp_ns,
                               bool sequence_valid,
                               std::uint32_t sequence) noexcept {
    stamp_dst.store(stamp_ns, std::memory_order_relaxed);
    sequence_dst.store(sequence, std::memory_order_relaxed);
    sequence_valid_dst.store(sequence_valid, std::memory_order_release);
    stamp_valid_dst.store(stamp_valid, std::memory_order_release);
  }

  void ResetLatestRaw_() noexcept {
    latest_waist_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_leg_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_arm_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_neck_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_pelvis_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_torso_stamp_valid_.store(false, std::memory_order_relaxed);
    latest_waist_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_leg_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_arm_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_neck_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_pelvis_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_torso_stamp_ns_.store(0, std::memory_order_relaxed);
    latest_waist_sequence_valid_.store(false, std::memory_order_relaxed);
    latest_leg_sequence_valid_.store(false, std::memory_order_relaxed);
    latest_arm_sequence_valid_.store(false, std::memory_order_relaxed);
    latest_neck_sequence_valid_.store(false, std::memory_order_relaxed);
    latest_waist_sequence_.store(0, std::memory_order_relaxed);
    latest_leg_sequence_.store(0, std::memory_order_relaxed);
    latest_arm_sequence_.store(0, std::memory_order_relaxed);
    latest_neck_sequence_.store(0, std::memory_order_relaxed);
  }

  std::atomic<std::uint64_t> tick_total_{0};
  std::atomic<std::uint64_t> frame_complete_total_{0};
  std::atomic<std::uint64_t> frame_aligned_total_{0};
  std::atomic<std::uint64_t> missing_waist_{0};
  std::atomic<std::uint64_t> missing_leg_{0};
  std::atomic<std::uint64_t> missing_arm_{0};
  std::atomic<std::uint64_t> missing_neck_{0};
  std::atomic<std::uint64_t> missing_pelvis_{0};
  std::atomic<std::uint64_t> missing_torso_{0};
  std::atomic<std::uint64_t> stale_waist_{0};
  std::atomic<std::uint64_t> stale_leg_{0};
  std::atomic<std::uint64_t> stale_arm_{0};
  std::atomic<std::uint64_t> stale_neck_{0};
  std::atomic<std::uint64_t> stale_pelvis_{0};
  std::atomic<std::uint64_t> stale_torso_{0};
  std::atomic<std::int64_t>  last_skew_ns_{0};
  std::atomic<std::int64_t>  last_tick_ns_{0};
  std::atomic<std::int64_t>  last_tick_interval_ns_{0};
  std::atomic<std::int64_t>  last_age_waist_ns_{-1};
  std::atomic<std::int64_t>  last_age_leg_ns_{-1};
  std::atomic<std::int64_t>  last_age_arm_ns_{-1};
  std::atomic<std::int64_t>  last_age_neck_ns_{-1};
  std::atomic<std::int64_t>  last_age_pelvis_ns_{-1};
  std::atomic<std::int64_t>  last_age_torso_ns_{-1};
  std::atomic<std::uint64_t> state_transport_count_{0};
  std::atomic<std::int64_t>  state_transport_total_ns_{0};
  std::atomic<std::int64_t>  state_transport_min_ns_{kLatencyMinSentinel};
  std::atomic<std::int64_t>  state_transport_max_ns_{kLatencyMaxSentinel};
  std::atomic<std::uint64_t> state_ready_count_{0};
  std::atomic<std::int64_t>  state_ready_total_ns_{0};
  std::atomic<std::int64_t>  state_ready_min_ns_{kLatencyMinSentinel};
  std::atomic<std::int64_t>  state_ready_max_ns_{kLatencyMaxSentinel};
  std::atomic<std::uint64_t> state_header_skew_count_{0};
  std::atomic<std::int64_t>  state_header_skew_total_ns_{0};
  std::atomic<std::int64_t>  state_header_skew_min_ns_{kLatencyMinSentinel};
  std::atomic<std::int64_t>  state_header_skew_max_ns_{kLatencyMaxSentinel};
  std::atomic<std::uint64_t> group_pair_skew_count_{0};
  std::atomic<std::int64_t>  group_pair_skew_total_ns_{0};
  std::atomic<std::int64_t>  group_pair_skew_min_ns_{kLatencyMinSentinel};
  std::atomic<std::int64_t>  group_pair_skew_max_ns_{kLatencyMaxSentinel};
  std::atomic<std::uint64_t> resample_interpolated_{0};
  std::atomic<std::uint64_t> resample_held_{0};
  std::atomic<bool> latest_waist_stamp_valid_{false};
  std::atomic<bool> latest_leg_stamp_valid_{false};
  std::atomic<bool> latest_arm_stamp_valid_{false};
  std::atomic<bool> latest_neck_stamp_valid_{false};
  std::atomic<bool> latest_pelvis_stamp_valid_{false};
  std::atomic<bool> latest_torso_stamp_valid_{false};
  std::atomic<std::int64_t> latest_waist_stamp_ns_{0};
  std::atomic<std::int64_t> latest_leg_stamp_ns_{0};
  std::atomic<std::int64_t> latest_arm_stamp_ns_{0};
  std::atomic<std::int64_t> latest_neck_stamp_ns_{0};
  std::atomic<std::int64_t> latest_pelvis_stamp_ns_{0};
  std::atomic<std::int64_t> latest_torso_stamp_ns_{0};
  std::atomic<bool> latest_waist_sequence_valid_{false};
  std::atomic<bool> latest_leg_sequence_valid_{false};
  std::atomic<bool> latest_arm_sequence_valid_{false};
  std::atomic<bool> latest_neck_sequence_valid_{false};
  std::atomic<std::uint32_t> latest_waist_sequence_{0};
  std::atomic<std::uint32_t> latest_leg_sequence_{0};
  std::atomic<std::uint32_t> latest_arm_sequence_{0};
  std::atomic<std::uint32_t> latest_neck_sequence_{0};
};

}  // namespace a3_sync
