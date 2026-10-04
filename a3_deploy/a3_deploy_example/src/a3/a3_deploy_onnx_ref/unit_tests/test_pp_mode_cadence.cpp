#include "a3_pingpong/pp_mode_cadence.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include "a3_rt/a3_based_task.hpp"
#include "a3_io/serve_hal_clock.hpp"

TEST(PpServeCadence, HalReceiptCannotShortenOrRephaseCsvPlayback) {
  constexpr std::int64_t epoch = 1'000'000'000;
  for (int phase = 0; phase < 10; ++phase) {
    for (int delay_us : {117, 1320, 3400, 8432, 8994}) {
      a3_pingpong::PpServeCadence cadence;
      const auto publication = epoch + phase * 10'000'000 + 20'000;
      const auto selected = publication + delay_us * 1000;
      ASSERT_TRUE(cadence.Poll(publication - 20'000));
      cadence.TrackHalSelection(publication);
      int frame = 48;
      std::int64_t contact = 0, previous = publication - 20'000;
      for (auto now = publication - 20'000 + 2'000'000; frame <= 78; now += 2'000'000) {
        if (cadence.tracking_hal()) {
          cadence.ObserveHalSelection(now, now >= selected ? selected : 0);
        }
        if (!cadence.Poll(now)) continue;
        ASSERT_GE(now, selected);
        EXPECT_EQ(now - previous, 10'000'000);
        previous = now;
        if (frame++ == 78) contact = now;
      }
      // A HAL selection is not physical release feedback. Its asynchronous
      // phase must never rewrite the time axis of an open-loop CSV.
      EXPECT_EQ(contact - (publication - 20'000), 310'000'000);
    }
  }
}

TEST(PpServeCadence, CancellingTelemetryDoesNotResetCsvPhase) {
  a3_pingpong::PpServeCadence clock;
  ASSERT_TRUE(clock.Poll(100'000'000));
  clock.TrackHalSelection(100'020'000);
  clock.CancelHalTracking(104'000'000);
  EXPECT_FALSE(clock.Poll(104'000'000));
  EXPECT_FALSE(clock.Poll(108'000'000));
  EXPECT_TRUE(clock.Poll(110'000'000));
}

TEST(PpServeCadence, MissingStaleAndLateReceiptsDoNotStopOrDelayAnyCsvRow) {
  using Clock = a3_pingpong::PpServeCadence;
  for (std::int64_t edge : {0LL, 99'000'000LL, 139'000'000LL, 149'000'000LL}) {
    Clock clock;
    ASSERT_TRUE(clock.Poll(100'000'000)); // Frame 47.
    clock.TrackHalSelection(100'020'000);
    unsigned row = 48;
    for (auto now = 102'000'000LL; row <= 110; now += 2'000'000) {
      if (clock.tracking_hal()) clock.ObserveHalSelection(now, edge <= now ? edge : 0);
      if (!clock.Poll(now)) continue;
      EXPECT_EQ(now, 100'000'000 + (row - 47) * 10'000'000LL);
      ++row;
    }
    EXPECT_EQ(row, 111U);
    EXPECT_TRUE(clock.ConsumeMissedSelection());
    EXPECT_FALSE(clock.ConsumeMissedSelection());
    EXPECT_FALSE(clock.tracking_hal());
  }
}

TEST(ServeHalClock, RejectsPartialAndInvalidSnapshotsWithoutBlocking) {
  a3_io::ServeHalClock clock{};
  a3_io::ServeHalSnapshot snapshot;
  EXPECT_FALSE(a3_io::ReadServeHalClock(&clock, snapshot));
  clock.magic = a3_io::ServeHalClock::kMagic;
  clock.sequence = 1;
  EXPECT_FALSE(a3_io::ReadServeHalClock(&clock, snapshot));
  clock.sequence = 2;
  clock.pid = 42; clock.selected_ns = 101; clock.release_edge_ns = 100; clock.left = 2000;
  ASSERT_TRUE(a3_io::ReadServeHalClock(&clock, snapshot));
  EXPECT_EQ(snapshot.release_edge_ns, 100U);
  EXPECT_EQ(snapshot.pid, 42U);
}

TEST(ServeHalClock, ColdStartupNeedsHeartbeatButNeverAFakeRelease) {
  a3_io::ServeHalSnapshot snapshot;
  EXPECT_FALSE(snapshot.Ready(1'000'000'000));
  snapshot.pid = 42;
  snapshot.heartbeat_ns = 999'000'000;
  EXPECT_TRUE(snapshot.Ready(1'000'000'000));
  EXPECT_EQ(snapshot.selected_ns, 0U);
  EXPECT_EQ(snapshot.release_edge_ns, 0U);
  EXPECT_FALSE(snapshot.Ready(1'100'000'001));
  EXPECT_FALSE(snapshot.Ready(998'000'000));
  a3_pingpong::PpServeCadence clock;
  clock.Poll(1'000'000'000);
  clock.TrackHalSelection(1'000'000'000);
  EXPECT_EQ(clock.ObserveHalSelection(1'002'000'000, snapshot.release_edge_ns),
            a3_pingpong::PpServeCadence::Receipt::kWaiting);
  EXPECT_FALSE(clock.Poll(1'002'000'000));
}

TEST(PpServeCadence, DifferentClicksAndRepeatedReleasesKeepOnePhase) {
  constexpr std::int64_t epoch = 1'000'250'000;
  // Exercise all five possible 500 Hz click phases, with and without long
  // intervening Stand/Teleop time. The clock is polled in every mode.
  for (int click_tick : {1, 2, 3, 4, 5, 1001, 1003, 5017}) {
    a3_pingpong::PpServeCadence clock;
    std::vector<std::int64_t> frames;
    for (int tick = 0; frames.size() < 111; ++tick) {
      auto scheduled = epoch + tick * 2'000'000LL;
      const bool due = clock.Poll(scheduled);
      if (!due || tick < click_tick) continue;
      frames.push_back(scheduled);
      if (frames.size() == 48) clock.NotePublication(scheduled + 28'294);
    }
    EXPECT_GE(frames.front(), epoch + click_tick * 2'000'000LL);
    EXPECT_LT(frames.front(), epoch + click_tick * 2'000'000LL + 10'000'000);
    for (std::size_t i = 0; i < frames.size(); ++i) {
      EXPECT_EQ((frames[i] - epoch) % 10'000'000, 0);
      EXPECT_EQ(frames[i] - frames.front(), i * 10'000'000);
    }
    // A second Play on this same clock also keeps the phase after RELEASE.
    const auto next = frames.back() + 10'000'000;
    EXPECT_TRUE(clock.Poll(next));
    clock.NotePublication(next + 16'042);
    EXPECT_FALSE(clock.Poll(next + 2'000'000));
    EXPECT_TRUE(clock.Poll(next + 10'000'000));
  }
}

TEST(PpServeCadence, SlowPublicationDefersWholeFramesWithoutChangingPhase) {
  a3_pingpong::PpServeCadence clock;
  EXPECT_TRUE(clock.Poll(470'000'000));
  clock.NotePublication(523'450'000);
  for (auto time = 524'000'000LL; time < 540'000'000LL; time += 2'000'000)
    EXPECT_FALSE(clock.Poll(time));
  EXPECT_TRUE(clock.Poll(540'000'000));
  EXPECT_FALSE(clock.Poll(542'000'000));
  EXPECT_TRUE(clock.Poll(550'000'000));
}

TEST(PpServeCadence, DriverStallNeverBurstsOrChoosesANewPhase) {
  a3_pingpong::PpServeCadence clock;
  EXPECT_TRUE(clock.Poll(0));
  EXPECT_TRUE(clock.Poll(66'000'000));
  EXPECT_FALSE(clock.Poll(68'000'000));
  EXPECT_FALSE(clock.Poll(70'000'000));
  EXPECT_FALSE(clock.Poll(78'000'000));
  EXPECT_TRUE(clock.Poll(80'000'000));
  EXPECT_TRUE(clock.Poll(90'000'000));
}

TEST(PpModeCadence, SharedDriverDeadlineDoesNotAmplifyWakeJitter) {
  std::vector<std::int64_t> old_times, shared_times;
  a3_pingpong::PpModeCadence old_clock, shared_clock;
  for (int tick = 0; tick < 500; ++tick) {
    const std::int64_t scheduled = 1'000'000'000LL + tick * 2'000'000LL;
    const auto actual = scheduled + (tick % 2 ? 20'000 : 80'000);
    if (old_clock.DueTime(actual, 2, 10'000'000)) old_times.push_back(actual);
    if (shared_clock.DueTime(scheduled, 2, 10'000'000)) shared_times.push_back(actual);
  }
  ASSERT_EQ(shared_times.size(), 100U);
  std::int64_t old_max_error = 0;
  for (std::size_t i = 1; i < old_times.size(); ++i)
    old_max_error = std::max(old_max_error, std::abs(old_times[i] - old_times[i-1] - 10'000'000));
  EXPECT_GE(old_max_error, 1'900'000);  // reproduce the former 8/12 ms cadence
  for (std::size_t i = 1; i < shared_times.size(); ++i)
    EXPECT_LE(std::abs(shared_times[i] - shared_times[i-1] - 10'000'000), 60'000);
}

TEST(PpModeCadence, SharedDeadlineStillUsesElapsedTimeAfterDriverRebase) {
  a3_pingpong::PpModeCadence cadence;
  std::int64_t scheduled = 1'000'000'000;
  EXPECT_TRUE(cadence.DueTime(scheduled, 2, 10'000'000));
  // Driver stall: no synthetic callback for each missed 2 ms period.
  scheduled = a3_rt::NextWakeNs(scheduled, scheduled + 65'000'000, 2'000'000);
  EXPECT_TRUE(cadence.DueTime(scheduled, 2, 10'000'000));
  for (int i = 1; i < 5; ++i)
    EXPECT_FALSE(cadence.DueTime(scheduled + i * 2'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(scheduled + 10'000'000, 2, 10'000'000));
}

TEST(PpModeCadence, OffCycleModeEntryDoesNotShortenFirstCsvInterval) {
  a3_pingpong::PpModeCadence cadence;
  EXPECT_TRUE(cadence.Due(1, 1, 10));
  EXPECT_TRUE(cadence.Due(4, 2, 5));
  for (unsigned tick = 5; tick < 9; ++tick) EXPECT_FALSE(cadence.Due(tick, 2, 5));
  EXPECT_TRUE(cadence.Due(9, 2, 5));
}

TEST(PpModeCadence, MaintainsServeAndReceiveRatesAndResetsAfterTeleop) {
  a3_pingpong::PpModeCadence cadence;
  int serve = 0, receive = 0;
  for (unsigned tick = 1; tick <= 500; ++tick) serve += cadence.Due(tick, 2, 5);
  for (unsigned tick = 501; tick <= 1000; ++tick) receive += cadence.Due(tick, 3, 10);
  EXPECT_EQ(serve, 100);
  EXPECT_EQ(receive, 50);
  cadence.Reset();
  EXPECT_TRUE(cadence.Due(1003, 3, 10));
  EXPECT_FALSE(cadence.Due(1011, 3, 10));
  EXPECT_TRUE(cadence.Due(1013, 3, 10));
}

TEST(PpModeCadence, MonotonicClockPreservesCsvRateWithDriverOverruns) {
  for (int hz : {470, 475, 480, 500}) {
    a3_pingpong::PpModeCadence cadence;
    int frames = 0;
    for (int poll = 0; poll < hz * 10; ++poll) {
      auto ns = static_cast<std::int64_t>(poll) * 1'000'000'000 / hz;
      frames += cadence.DueTime(ns, 2, 10'000'000);
    }
    EXPECT_EQ(frames, 1000) << hz;
  }
}

TEST(PpModeCadence, TimeEntryAndPauseNeverSkipOrBurstMotionFrames) {
  a3_pingpong::PpModeCadence cadence;
  EXPECT_TRUE(cadence.DueTime(0, 1, 20'000'000));
  EXPECT_TRUE(cadence.DueTime(3'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(12'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(14'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(100'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(102'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(108'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(110'000'000, 2, 10'000'000));
  cadence.Reset();
  EXPECT_TRUE(cadence.DueTime(112'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(114'000'000, 2, 10'000'000));
}

TEST(PpModeCadence, SlowReleasePublicationDoesNotCompressNextSwingFrame) {
  a3_pingpong::PpModeCadence cadence;
  EXPECT_TRUE(cadence.DueTime(470'000'000, 2, 10'000'000));
  cadence.RebaseAfter(523'450'000, 2, 10'000'000);
  EXPECT_FALSE(cadence.DueTime(524'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(532'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(534'000'000, 2, 10'000'000));
  EXPECT_FALSE(cadence.DueTime(542'000'000, 2, 10'000'000));
  EXPECT_TRUE(cadence.DueTime(544'000'000, 2, 10'000'000));
}
