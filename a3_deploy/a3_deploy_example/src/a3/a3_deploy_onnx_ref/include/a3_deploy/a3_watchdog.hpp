// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8 Task 8.3
//
// A3Watchdog: pure stateful frame-health checker used by A3PolicyDriver. Every
// policy tick it is handed (now_ns, state.timestamp_ns, aligned) and returns
// one of {Ok, StaleFrame, ChronicUnaligned}. It does NOT trigger any side
// effects — it is the caller's job to act on the verdict (e.g. issue a safe
// halt command).
#pragma once

#include <atomic>
#include <cstdint>

namespace a3_deploy {

struct WatchdogConfig {
  // Max allowed age (wall-clock now - state.timestamp_ns) before a frame is
  // declared stale. Default 50ms (2.5x sync period @ 200Hz).
  std::int64_t max_frame_age_ns = 50'000'000;

  // Max consecutive aligned=false frames before triggering ChronicUnaligned.
  // (Each policy tick checks; 5 @ 50Hz = 100ms total.)
  int max_consecutive_unaligned = 5;
};

enum class WatchdogVerdict {
  Ok,
  StaleFrame,
  ChronicUnaligned,
};

class A3Watchdog {
 public:
  explicit A3Watchdog(WatchdogConfig cfg);

  // Called every policy tick with the latest state observation.
  // Returns Ok if the policy may consume this state, else a verdict
  // indicating why to safe-halt.
  //
  // Verdict priority: StaleFrame > ChronicUnaligned > Ok. A stale frame
  // does NOT reset the unaligned streak (stale and chronic can coexist);
  // an aligned=true frame DOES reset the streak even if the frame is stale.
  WatchdogVerdict Check(std::int64_t now_ns,
                        std::int64_t state_ts_ns,
                        bool aligned) noexcept;

  // Resets internal state (unaligned streak). Call after recovery.
  void Reset() noexcept;

  // Diagnostics.
  std::uint64_t StaleCount() const noexcept { return stale_count_.load(std::memory_order_relaxed); }
  std::uint64_t UnalignedCount() const noexcept { return unaligned_count_.load(std::memory_order_relaxed); }
  int CurrentUnalignedStreak() const noexcept { return unaligned_streak_.load(std::memory_order_relaxed); }

  const WatchdogConfig& cfg() const noexcept { return cfg_; }

 private:
  WatchdogConfig cfg_;
  std::atomic<int> unaligned_streak_{0};
  std::atomic<std::uint64_t> stale_count_{0};
  std::atomic<std::uint64_t> unaligned_count_{0};
};

}  // namespace a3_deploy
