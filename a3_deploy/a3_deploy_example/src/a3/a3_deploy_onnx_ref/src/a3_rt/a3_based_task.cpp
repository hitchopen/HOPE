// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// PR 4/10 of the A3 backend adaptation (see notes/a3_backend_plan.md §PR 4).
#include "a3_rt/a3_based_task.hpp"

#include <pthread.h>
#include <sys/prctl.h>
#include <time.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace a3_rt {
namespace {

constexpr std::int64_t kNsPerSec = 1'000'000'000LL;

inline std::int64_t ToNs(const timespec& ts) noexcept {
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec +
         static_cast<std::int64_t>(ts.tv_nsec);
}

inline timespec FromNs(std::int64_t ns) noexcept {
  if (ns < 0) ns = 0;
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(ns / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
  return ts;
}

// store max via relaxed RMW. Only the RT thread writes, so loaded value is ours.
inline void StoreMax(std::atomic<std::int64_t>& slot, std::int64_t v) noexcept {
  if (v > slot.load(std::memory_order_relaxed)) {
    slot.store(v, std::memory_order_relaxed);
  }
}

}  // namespace

A3BasedTask::A3BasedTask(Options opt) : opt_(std::move(opt)) {
  if (opt_.period_ns <= 0) {
    opt_.period_ns = 20'000'000;  // guard: default 50 Hz
  }
}

A3BasedTask::~A3BasedTask() { Stop(); }

bool A3BasedTask::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return false;  // already running
  }
  should_stop_.store(false, std::memory_order_relaxed);
  try {
    thread_ = std::thread(&A3BasedTask::ThreadMain, this);
  } catch (...) {
    running_.store(false, std::memory_order_release);
    return false;
  }
  return true;
}

void A3BasedTask::Stop() {
  should_stop_.store(true, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
  running_.store(false, std::memory_order_release);
}

A3BasedTask::Stats A3BasedTask::GetStats() const noexcept {
  Stats s;
  s.tick_count = tick_count_.load(std::memory_order_relaxed);
  s.overrun_count = overrun_count_.load(std::memory_order_relaxed);
  s.max_run_ns = max_run_ns_.load(std::memory_order_relaxed);
  s.last_run_ns = last_run_ns_.load(std::memory_order_relaxed);
  s.max_wake_lateness_ns = max_wake_lateness_ns_.load(std::memory_order_relaxed);
  return s;
}

void A3BasedTask::ThreadMain() {
  // Best-effort thread name (<=15 chars).
  if (!opt_.name.empty()) {
    char buf[16]{};
    std::strncpy(buf, opt_.name.c_str(), sizeof(buf) - 1);
    prctl(PR_SET_NAME, buf, 0, 0, 0);
  }

  // Best-effort scheduling + affinity. Failures are logged but not fatal.
  SetRtSchedFifo(opt_.sched.priority);
  PinCurrentThreadToCpu(opt_.sched.cpu);

  OnStart();

  timespec next{};
  if (clock_gettime(CLOCK_MONOTONIC, &next) != 0) {
    std::fprintf(stderr, "[a3_rt][%s] clock_gettime failed: %s\n",
                 opt_.name.c_str(), std::strerror(errno));
    OnStop();
    return;
  }

  const std::int64_t period_ns = opt_.period_ns;
  std::int64_t next_wake_ns =
      opt_.first_wake_monotonic_ns > 0
          ? opt_.first_wake_monotonic_ns
          : ToNs(next) + period_ns;

  while (!should_stop_.load(std::memory_order_relaxed)) {
    next = FromNs(next_wake_ns);

    // Sleep until absolute `next`. Retry on EINTR.
    int rc = 0;
    while (true) {
      rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
      if (rc == 0) break;
      if (rc == EINTR) continue;
      std::fprintf(stderr, "[a3_rt][%s] clock_nanosleep failed: %s\n",
                   opt_.name.c_str(), std::strerror(rc));
      break;
    }
    if (rc != 0 && rc != EINTR) break;

    // Re-check stop flag after sleep so Stop() returns promptly.
    if (should_stop_.load(std::memory_order_relaxed)) break;

    timespec t0{};
    clock_gettime(CLOCK_MONOTONIC, &t0);
    const std::int64_t lateness = ToNs(t0) - ToNs(next);
    StoreMax(max_wake_lateness_ns_, lateness);

    RunOnce();

    timespec t1{};
    clock_gettime(CLOCK_MONOTONIC, &t1);
    const std::int64_t run_ns = ToNs(t1) - ToNs(t0);

    tick_count_.fetch_add(1, std::memory_order_relaxed);
    last_run_ns_.store(run_ns, std::memory_order_relaxed);
    StoreMax(max_run_ns_, run_ns);

    if (run_ns > period_ns) {
      overrun_count_.fetch_add(1, std::memory_order_relaxed);
      // Reset schedule to "now" so one slow tick does not cause a burst of
      // catch-up ticks (matches motion_control_a3 ResetClock() behaviour).
      if (clock_gettime(CLOCK_MONOTONIC, &next) != 0) break;
      next_wake_ns = ToNs(next) + period_ns;
    } else {
      next_wake_ns += period_ns;
    }
  }

  OnStop();
}

}  // namespace a3_rt
