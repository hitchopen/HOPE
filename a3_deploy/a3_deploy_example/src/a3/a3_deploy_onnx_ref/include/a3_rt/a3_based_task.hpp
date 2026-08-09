// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// PR 4/10 of the A3 backend adaptation (see notes/a3_backend_plan.md §PR 4).
//
// A3BasedTask: abstract base for periodic RT threads.
// Simplified from motion_control_a3/src/base/task_base/based_task.{h,cpp}:
//   - Pure pthread / std::thread (no AimRT executor).
//   - clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) drives the period.
//   - Optional SCHED_FIFO + CPU affinity via RtSched.
//   - Overrun detection + counter, max run/wake-lateness stats.
//   - Graceful Stop() + join.
//
// Subclasses override RunOnce() (and optionally OnStart / OnStop). RunOnce()
// is invoked once per period inside the RT thread and must not throw.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "a3_rt/a3_rt.hpp"

namespace a3_rt {

class A3BasedTask {
 public:
  struct Options {
    std::string name = "a3_task";     // For logging and set_thread_name.
    std::int64_t period_ns = 20'000'000;  // 50 Hz default.
    std::int64_t first_wake_monotonic_ns = 0;  // 0 = first tick after one period.
    RtSched sched{};                      // default: no RT, no affinity.
  };

  struct Stats {
    std::uint64_t tick_count = 0;
    std::uint64_t overrun_count = 0;
    std::int64_t max_run_ns = 0;
    std::int64_t last_run_ns = 0;
    std::int64_t max_wake_lateness_ns = 0;
  };

  explicit A3BasedTask(Options opt);
  virtual ~A3BasedTask();

  A3BasedTask(const A3BasedTask&) = delete;
  A3BasedTask& operator=(const A3BasedTask&) = delete;

  // Start the RT thread. Non-blocking. Returns false if already running.
  bool Start();

  // Signal the thread to stop and join it. Safe to call multiple times.
  void Stop();

  bool Running() const noexcept { return running_.load(std::memory_order_acquire); }

  Stats GetStats() const noexcept;

 protected:
  // Override this. Called once per period inside the RT thread. Must not throw.
  virtual void RunOnce() noexcept = 0;

  // Optional hooks running on the RT thread before/after the loop.
  virtual void OnStart() noexcept {}
  virtual void OnStop() noexcept {}

  const Options& options() const noexcept { return opt_; }

 private:
  void ThreadMain();

  Options opt_;
  std::atomic<bool> running_{false};
  std::atomic<bool> should_stop_{false};
  std::thread thread_;

  // Stats: written only by the RT thread; read atomically from outside.
  std::atomic<std::uint64_t> tick_count_{0};
  std::atomic<std::uint64_t> overrun_count_{0};
  std::atomic<std::int64_t> max_run_ns_{0};
  std::atomic<std::int64_t> last_run_ns_{0};
  std::atomic<std::int64_t> max_wake_lateness_ns_{0};
};

}  // namespace a3_rt
