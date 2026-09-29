#include <gtest/gtest.h>
#include "a3_pingpong/pp_execution_history.hpp"
#include "a3_pingpong/pp_qdes_contract.hpp"

namespace a3_pingpong {
TEST(PpExecutionHistory, ResetAndTickHistoryKeepOnlyProprioFrames) {
  ExecutionHistory502 history;
  ExecutionHistory502::Prefix prefix{};
  ExecutionHistory502::Joints raw{}, sent{}, actual{};
  raw.fill(7.); sent.fill(3.); actual.fill(1.);
  prefix[0] = 11.; prefix[34] = 12.; prefix[96] = 13.; prefix[103] = 999.;
  auto obs = history.Build(prefix, raw, sent, actual, 0);
  EXPECT_DOUBLE_EQ(obs[112], 7.);
  EXPECT_DOUBLE_EQ(obs[143], 3.);
  EXPECT_DOUBLE_EQ(obs[174], 2.);
  EXPECT_DOUBLE_EQ(obs[205], 1.);
  EXPECT_DOUBLE_EQ(obs[205 + 31], 12.);
  EXPECT_DOUBLE_EQ(obs[205 + 62], 13.);
  EXPECT_DOUBLE_EQ(obs[205 + 65], 11.);
  EXPECT_DOUBLE_EQ(obs[205 + 68], 3.);
  for (int tick = 1; tick <= 4; ++tick) {
    actual.fill(tick + 1.);
    obs = history.Build(prefix, raw, sent, actual, tick);
  }
  EXPECT_DOUBLE_EQ(obs[205], 4.);
  EXPECT_DOUBLE_EQ(obs[304], 3.);
  EXPECT_DOUBLE_EQ(obs[403], 2.);
  EXPECT_EQ(obs, history.Build(prefix, raw, sent, actual, 4));
  history.Reset(); actual.fill(9.);
  obs = history.Build(prefix, raw, sent, actual, 4);
  EXPECT_DOUBLE_EQ(obs[205], 9.);
  EXPECT_DOUBLE_EQ(obs[403], 9.);
}

TEST(PpExecutionHistory, FinalRunnerOverrideOwnsFeedbackAndNextSlewState) {
  double final_target[] = {.05, -.1, .2};
  const double defaults[] = {0., 0., .2};
  const double scales[] = {.5, .2, .1};
  const bool passive[] = {false, false, true};
  double state[] = {.3, .3, .3}, feedback[3]{};
  CommitV12FinalTarget(3, final_target, defaults, scales, passive, state, feedback);
  EXPECT_DOUBLE_EQ(state[0], .05);
  EXPECT_DOUBLE_EQ(feedback[0], .1);
  EXPECT_DOUBLE_EQ(feedback[1], -.5);
  EXPECT_DOUBLE_EQ(feedback[2], 0.);
  const auto next = ApplyV12SlewSafeQdes(10., 0., .5, -1., 1., state[0], .1, false, 1.);
  EXPECT_NEAR(next.q_hat, .15, 1.e-12);
}

TEST(PpExecutionHistory, CompactSentAndVelocityDebtFramesKeepFourTicks) {
  CompactExecutionHistory324 history;
  CompactExecutionHistory324::Prefix prefix{};
  CompactExecutionHistory324::Joints sent{}, actual{};
  sent.fill(3.); actual.fill(1.);
  prefix[34] = 12.; prefix[76] = .2; prefix[81] = -.1;
  prefix[96] = 13.; prefix[0] = 11.; prefix[103] = 999.;
  auto obs = history.Build(prefix, sent, actual, 0);
  EXPECT_DOUBLE_EQ(obs[65], 3.);
  EXPECT_DOUBLE_EQ(obs[76], 3.);  // full sent target, no overloaded head slot
  EXPECT_DOUBLE_EQ(obs[112], .2);
  EXPECT_DOUBLE_EQ(obs[113], -.1);
  EXPECT_DOUBLE_EQ(obs[114], 12.);
  EXPECT_DOUBLE_EQ(obs[145], 2.);
  EXPECT_DOUBLE_EQ(obs[176], 13.);
  EXPECT_DOUBLE_EQ(obs[179], 11.);
  EXPECT_DOUBLE_EQ(obs[182], .2);
  EXPECT_DOUBLE_EQ(obs[103], 999.);  // task data only in current frame
  for (int tick = 1; tick <= 4; ++tick) {
    actual.fill(tick + 1.);
    obs = history.Build(prefix, sent, actual, tick);
  }
  EXPECT_DOUBLE_EQ(obs[145], -1.);
  EXPECT_DOUBLE_EQ(obs[215], 0.);
  EXPECT_DOUBLE_EQ(obs[285], 1.);
  EXPECT_EQ(obs, history.Build(prefix, sent, actual, 4));
  history.Reset(); actual.fill(9.);
  obs = history.Build(prefix, sent, actual, 4);
  EXPECT_DOUBLE_EQ(obs[145], -6.);
  EXPECT_DOUBLE_EQ(obs[285], -6.);
}
}  // namespace a3_pingpong
