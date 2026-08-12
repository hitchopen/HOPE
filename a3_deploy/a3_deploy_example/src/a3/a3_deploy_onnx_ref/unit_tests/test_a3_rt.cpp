// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// PR 4/10 of the A3 backend adaptation (see notes/a3_backend_plan.md §PR 4).
//
// GoogleTest coverage for a3_rt::A3BasedTask + RT helpers.
// Notes: SCHED_FIFO (priority>0) requires CAP_SYS_NICE / root, which CI
// generally lacks. All tests here use priority=0 so they pass unprivileged;
// SetRtSchedFifo() is exercised directly with graceful-failure semantics.
#include <gtest/gtest.h>

#include <time.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

#include "a3_rt/a3_based_task.hpp"
#include "a3_rt/a3_rt.hpp"

namespace {

std::int64_t MonotonicNowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(ts.tv_nsec);
}

class CountingTask : public a3_rt::A3BasedTask {
 public:
  CountingTask(Options opt, std::chrono::microseconds work)
      : A3BasedTask(std::move(opt)), work_(work) {}

  std::atomic<std::uint64_t> run_calls{0};

 protected:
  void RunOnce() noexcept override {
    run_calls.fetch_add(1, std::memory_order_relaxed);
    if (work_.count() > 0) {
      std::this_thread::sleep_for(work_);
    }
  }

 private:
  std::chrono::microseconds work_;
};

class FirstWakeTask : public a3_rt::A3BasedTask {
 public:
  explicit FirstWakeTask(Options opt) : A3BasedTask(std::move(opt)) {}

  std::atomic<std::uint64_t> run_calls{0};
  std::atomic<std::int64_t> first_run_ns{0};

 protected:
  void RunOnce() noexcept override {
    const auto now = MonotonicNowNs();
    std::int64_t expected = 0;
    first_run_ns.compare_exchange_strong(expected, now);
    run_calls.fetch_add(1, std::memory_order_relaxed);
  }
};

}  // namespace

// -----------------------------------------------------------------------------
// Frequency accuracy: 100 Hz for ~1 second, expect ~100 ticks, no overrun.
// -----------------------------------------------------------------------------
TEST(A3BasedTaskTest, FrequencyAccuracy100Hz) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_100hz";
  opt.period_ns = 10'000'000;  // 100 Hz
  CountingTask task(opt, std::chrono::microseconds(0));

  ASSERT_TRUE(task.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  task.Stop();

  auto s = task.GetStats();
  // Wide tolerance for CI/loaded machines.
  EXPECT_GE(s.tick_count, 93u) << "too few ticks";
  EXPECT_LE(s.tick_count, 110u) << "too many ticks";
  EXPECT_EQ(s.overrun_count, 0u) << "unexpected overruns on idle task";
}

// -----------------------------------------------------------------------------
// Start/Stop lifecycle: Running() flips, double-Stop is safe.
// -----------------------------------------------------------------------------
TEST(A3BasedTaskTest, StartStopLifecycle) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_lifecycle";
  opt.period_ns = 5'000'000;  // 200 Hz
  CountingTask task(opt, std::chrono::microseconds(0));

  EXPECT_FALSE(task.Running());
  ASSERT_TRUE(task.Start());
  EXPECT_TRUE(task.Running());
  // Double-start should fail.
  EXPECT_FALSE(task.Start());

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  task.Stop();
  EXPECT_FALSE(task.Running());
  // Double-stop is safe.
  task.Stop();
  EXPECT_FALSE(task.Running());

  EXPECT_GT(task.GetStats().tick_count, 0u);
}

TEST(A3BasedTaskTest, FirstWakeMonotonicNsDelaysFirstTick) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_first_wake";
  opt.period_ns = 10'000'000;  // 100 Hz after the first wake.
  const auto target_ns = MonotonicNowNs() + 80'000'000;
  opt.first_wake_monotonic_ns = target_ns;
  FirstWakeTask task(opt);

  ASSERT_TRUE(task.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  task.Stop();

  const auto first_run_ns = task.first_run_ns.load(std::memory_order_relaxed);
  ASSERT_GT(first_run_ns, 0);
  EXPECT_GE(first_run_ns, target_ns);
  EXPECT_GE(task.GetStats().tick_count, 4u);
}

// -----------------------------------------------------------------------------
// max_run_ns reflects a 5 ms sleep inside RunOnce.
// -----------------------------------------------------------------------------
TEST(A3BasedTaskTest, MaxRunNsTracksWork) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_max_run";
  opt.period_ns = 20'000'000;  // 50 Hz (plenty of slack for 5ms work)
  CountingTask task(opt, std::chrono::milliseconds(5));

  ASSERT_TRUE(task.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  task.Stop();

  auto s = task.GetStats();
  EXPECT_GE(s.max_run_ns, 4'500'000) << "max_run_ns should reflect ~5ms work";
  EXPECT_GT(s.tick_count, 0u);
  // 5ms work at 20ms period -> no overruns expected.
  EXPECT_EQ(s.overrun_count, 0u);
}

// -----------------------------------------------------------------------------
// Deliberate overrun: 30 ms work in a 10 ms period -> every tick overruns.
// Run for ~300 ms; after a single overrun the clock resets, so we expect
// ticks every ~30 ms ==> ~10 overruns. Use a loose lower bound of 5.
// -----------------------------------------------------------------------------
TEST(A3BasedTaskTest, OverrunDetection) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_overrun";
  opt.period_ns = 10'000'000;  // 10 ms
  CountingTask task(opt, std::chrono::milliseconds(30));

  ASSERT_TRUE(task.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  task.Stop();

  auto s = task.GetStats();
  EXPECT_GE(s.overrun_count, 5u) << "expected deliberate overruns";
  EXPECT_EQ(s.overrun_count, s.tick_count)
      << "every tick should overrun when work > period";
  EXPECT_GE(s.max_run_ns, 25'000'000);
}

// -----------------------------------------------------------------------------
// SCHED_FIFO request with priority=0 is a no-op and always succeeds.
// With priority>0 the call may legitimately fail (EPERM) on CI; the wrapper
// must not crash and Start() must still succeed.
// -----------------------------------------------------------------------------
TEST(A3RtTest, SetRtSchedFifoPriorityZeroIsNoop) {
  EXPECT_TRUE(a3_rt::SetRtSchedFifo(0));
}

TEST(A3RtTest, PinCurrentThreadToCpuNegativeIsNoop) {
  EXPECT_TRUE(a3_rt::PinCurrentThreadToCpu(-1));
}

TEST(A3BasedTaskTest, StartSucceedsWhenRtUnavailable) {
  // Try to request SCHED_FIFO priority=50. On CI this normally fails with
  // EPERM, but Start() must still succeed and ticks must still run.
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_rt_fallback";
  opt.period_ns = 10'000'000;
  opt.sched.priority = 50;  // may fail silently
  CountingTask task(opt, std::chrono::microseconds(0));

  ASSERT_TRUE(task.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  task.Stop();
  EXPECT_GT(task.GetStats().tick_count, 0u);
}

// -----------------------------------------------------------------------------
// GetStats() must be safe to call concurrently with the RT thread.
// Hammer it from multiple reader threads; require no crashes and monotonic
// tick_count from any single reader's view.
// -----------------------------------------------------------------------------
TEST(A3BasedTaskTest, GetStatsThreadSafe) {
  a3_rt::A3BasedTask::Options opt;
  opt.name = "test_stats_race";
  opt.period_ns = 2'000'000;  // 500 Hz
  CountingTask task(opt, std::chrono::microseconds(0));
  ASSERT_TRUE(task.Start());

  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> observed_max{0};
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([&] {
      std::uint64_t prev = 0;
      while (!stop.load()) {
        auto s = task.GetStats();
        EXPECT_GE(s.tick_count, prev);
        prev = s.tick_count;
        std::uint64_t cur_max = observed_max.load();
        while (s.tick_count > cur_max &&
               !observed_max.compare_exchange_weak(cur_max, s.tick_count)) {
        }
      }
    });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop.store(true);
  for (auto& t : readers) t.join();
  task.Stop();

  EXPECT_GT(observed_max.load(), 0u);
  EXPECT_GE(task.GetStats().tick_count, observed_max.load())
      << "final tick_count should be >= any earlier observation";
}
