#include "a3_pingpong/pp_mode_cadence.hpp"
#include <gtest/gtest.h>

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
