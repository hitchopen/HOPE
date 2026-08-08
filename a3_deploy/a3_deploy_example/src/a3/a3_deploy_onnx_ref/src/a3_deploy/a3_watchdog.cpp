// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8 Task 8.3
#include "a3_deploy/a3_watchdog.hpp"

namespace a3_deploy {

A3Watchdog::A3Watchdog(WatchdogConfig cfg) : cfg_(cfg) {}

WatchdogVerdict A3Watchdog::Check(std::int64_t now_ns,
                                  std::int64_t state_ts_ns,
                                  bool aligned) noexcept {
  // Update streak first so StaleFrame does NOT reset it, but aligned=true
  // always clears it (recovery).
  if (aligned) {
    unaligned_streak_.store(0, std::memory_order_relaxed);
  } else {
    unaligned_streak_.fetch_add(1, std::memory_order_relaxed);
  }

  const std::int64_t age = now_ns - state_ts_ns;
  if (age > cfg_.max_frame_age_ns) {
    stale_count_.fetch_add(1, std::memory_order_relaxed);
    return WatchdogVerdict::StaleFrame;
  }

  const int streak = unaligned_streak_.load(std::memory_order_relaxed);
  if (!aligned && streak >= cfg_.max_consecutive_unaligned) {
    unaligned_count_.fetch_add(1, std::memory_order_relaxed);
    return WatchdogVerdict::ChronicUnaligned;
  }

  return WatchdogVerdict::Ok;
}

void A3Watchdog::Reset() noexcept {
  unaligned_streak_.store(0, std::memory_order_relaxed);
}

}  // namespace a3_deploy
