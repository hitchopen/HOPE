#include "a3_pingpong/pp_planner_input.hpp"
#include "a3_pingpong/pp_runner_control.hpp"
#include <chrono>
#include <thread>
#include <gtest/gtest.h>

namespace {
using namespace a3_pingpong;
TEST(PpBasePoseHold, ShortGapNeverInventsSensorProgressOrExtendsDeadline) {
  PpBasePoseInput input;
  PpBaseSample real, held;
  const double now = PpNowWallSec(), sec = std::floor(now);
  std::vector<double> wire{2,1,7,sec,std::floor((now-sec)*1e9),
      -.5,.2,1,1,0,0,0,1,double(kV17RequiredBaseFlags),1,2};
  input.SetFromFlat(wire);
  ASSERT_TRUE(input.LatestForPolicy(real, .100, true));
  PpBasePoseHold hold;
  const Vec4 corrected(std::cos(.01), 0, 0, std::sin(.01));
  const auto initial = hold.Apply(real, corrected);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  ASSERT_TRUE(input.LatestForPolicy(held, .100, true));
  ASSERT_TRUE(held.held_for_policy);
  for (int i = 0; i < 50; ++i) {
    const auto used = hold.Apply(held, Vec4(0,1,0,0));
    EXPECT_EQ(used.pos, initial.pos);
    EXPECT_EQ(used.quat, initial.quat);
    EXPECT_EQ(used.seq, real.seq);
    EXPECT_EQ(used.source_wall_s, real.source_wall_s);
    EXPECT_EQ(used.receipt_steady_s, real.receipt_steady_s);
    EXPECT_TRUE(used.held_for_policy);
  }
  input.SetFromFlat(wire);
  std::this_thread::sleep_for(std::chrono::milliseconds(85));
  EXPECT_FALSE(input.LatestForPolicy(held, 10., true));
  wire[2] = 8; wire[5] = -.4;
  input.SetFromFlat(wire);
  ASSERT_TRUE(input.LatestForPolicy(real, .100, true));
  EXPECT_FALSE(real.held_for_policy);
  EXPECT_EQ(hold.Apply(real, real.quat).pos, real.pos);
  wire[1] = 0;
  input.SetFromFlat(wire);
  EXPECT_FALSE(input.LatestForPolicy(held, .100, true));
}
TEST(PpBasePoseHold, HeldInputDoesNotShortenPlayAdmissionOrServeCompletionDeadline) {
  PpRunnerControl control(RunnerMode::kMotion, 1, "test");
  control.RequireMocap(true);
  EXPECT_FALSE(control.ObserveMocap(true, true, .025));
  EXPECT_EQ(control.mode(), RunnerMode::kMotion);
  EXPECT_EQ(control.EncodeMocapSafety()[5], 0.);
  EXPECT_EQ(control.EncodeMocapSafety()[8], 1.);
  EXPECT_TRUE(control.SetRuntimeMode(RunnerMode::kServe, true));
  EXPECT_TRUE(control.ConsumeServePrepare());
  EXPECT_TRUE(control.CompleteServe());
  EXPECT_EQ(control.mode(), RunnerMode::kMotion);
  EXPECT_EQ(control.EncodeMocapSafety()[6], 0.);
  EXPECT_EQ(control.EncodeMocapSafety()[7], 0.);
  EXPECT_TRUE(control.ObserveMocap(false));
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
}
TEST(PpBasePoseHold, CacheDoesNotCrossRealReceiptOrCoordinateFrameEpoch) {
  PpBasePoseHold hold;
  PpBaseSample sample;
  sample.receipt_steady_s = 5.; sample.seq = 7; sample.pos = Vec3(1,2,3);
  const auto previous = hold.Apply(sample, Vec4(0,0,0,1));
  sample.held_for_policy = true;
  EXPECT_EQ(hold.Apply(sample, Vec4(1,0,0,0)).quat, previous.quat);
  sample.seq = 1; sample.receipt_steady_s = 6.; sample.pos = Vec3(4,5,6);
  EXPECT_EQ(hold.Apply(sample, Vec4(0,1,0,0)).pos, sample.pos);
  EXPECT_EQ(hold.Apply(sample, Vec4(0,1,0,0)).quat, sample.quat);
  sample.world_frame_id = 9; sample.quat = Vec4(0,0,1,0);
  EXPECT_EQ(hold.Apply(sample, Vec4(0,1,0,0)).quat, sample.quat);
}
TEST(PpBasePoseHold, MissingInitialOrInvalidSamplesCannotSeedPolicyHold) {
  PpBasePoseInput input;
  PpBaseSample out;
  EXPECT_FALSE(input.LatestForPolicy(out, .100, true));
  input.SetFromFlat({2,0});
  EXPECT_FALSE(input.LatestForPolicy(out, .100, true));
  EXPECT_FALSE(input.LatestForPolicy(out, NAN, true));
}
}  // namespace
