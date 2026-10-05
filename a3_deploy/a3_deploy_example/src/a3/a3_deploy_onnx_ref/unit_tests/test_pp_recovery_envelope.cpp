// Recovery-envelope monitor (Layer C): thresholds, hysteresis, mode gating.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "a3_pingpong/pp_recovery_envelope.hpp"

namespace a3_pingpong {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDeg = M_PI / 180.0;
constexpr double kDt = 0.02;  // 50 Hz

TEST(PpRecoveryEnvelope, ModeParsingDefaultsToCapabilityAndFailsClosed) {
  EXPECT_EQ(ParseRecoveryEnvelopeMode(nullptr), RecoveryEnvelopeMode::kCapability);
  EXPECT_EQ(ParseRecoveryEnvelopeMode(""), RecoveryEnvelopeMode::kCapability);
  EXPECT_EQ(ParseRecoveryEnvelopeMode("capability"), RecoveryEnvelopeMode::kCapability);
  EXPECT_EQ(ParseRecoveryEnvelopeMode("production"), RecoveryEnvelopeMode::kProduction);
  EXPECT_THROW(ParseRecoveryEnvelopeMode("Production"), std::runtime_error);
  EXPECT_THROW(ParseRecoveryEnvelopeMode("telemetry"), std::runtime_error);
  EXPECT_THROW(ParseRecoveryEnvelopeMode("1"), std::runtime_error);
  EXPECT_STREQ(RecoveryEnvelopeModeName(RecoveryEnvelopeMode::kCapability), "capability");
  EXPECT_STREQ(RecoveryEnvelopeModeName(RecoveryEnvelopeMode::kProduction), "production");
}

TEST(PpRecoveryEnvelope, TiltFromProjectedGravity) {
  EXPECT_DOUBLE_EQ(TiltFromProjectedGravity(0.0, 0.0), 0.0);
  EXPECT_NEAR(TiltFromProjectedGravity(std::sin(10.0 * kDeg), 0.0), 10.0 * kDeg, 1e-12);
  EXPECT_NEAR(TiltFromProjectedGravity(0.0, -std::sin(4.0 * kDeg)), 4.0 * kDeg, 1e-12);
  // slightly non-unit gravity must clip instead of producing NaN
  EXPECT_DOUBLE_EQ(TiltFromProjectedGravity(0.8, 0.8), std::asin(1.0));
  EXPECT_TRUE(std::isnan(TiltFromProjectedGravity(kNaN, 0.0)));
}

TEST(PpRecoveryEnvelope, InstantLevelThresholdsAreNestedAndExclusive) {
  // L0
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0), 0);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.049, 0.199, 3.99 * kDeg), 0);
  // each metric alone pushes to L1 at its L0 bound (bounds are exclusive)
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.05, 0.0, 0.0), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.20, 0.0), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 4.0 * kDeg), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.099, 0.349, 6.99 * kDeg), 1);
  // L2
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.10, 0.0, 0.0), 2);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.35, 0.0), 2);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 7.0 * kDeg), 2);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.149, 0.499, 9.99 * kDeg), 2);
  // The MuJoCo failure onset: 12 cm off HOME at 0.14 m/s -> L2.
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.12, 0.14, 1.0 * kDeg), 2);
  // L3
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.15, 0.0, 0.0), 3);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.50, 0.0), 3);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 10.0 * kDeg), 3);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(1.0, 2.0, 1.0), 3);
  // fail closed on unknown inputs (HOME unset / NaN speed / NaN tilt)
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(kNaN, 0.0, 0.0), 3);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, kNaN, 0.0), 3);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, kNaN), 3);
}

TEST(PpRecoveryEnvelope, Schema33SupportAnchorChannelIsOptionalAndNested) {
  // NaN (not computed) never changes the pelvis-only classification.
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, kNaN), 0);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.12, 0.14, 1.0 * kDeg, {}, kNaN), 2);
  // The Gate3 torture precursor: pelvis 8.2 cm / 0.08 m/s / 1.3 deg is L1 on pelvis
  // channels alone, but the right sole was 16.5 cm off its anchor -> L3.
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.082, 0.08, 1.3 * kDeg), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.082, 0.08, 1.3 * kDeg, {}, 0.165), 3);
  // Nested thresholds, exclusive bounds.
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.039), 0);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.04), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.079), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.08), 2);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.119), 2);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.0, 0.0, 0.0, {}, 0.12), 3);
  // A wide stance at the ball-40 width (~41 cm -> ~7 cm anchor error) is L1 with the pelvis
  // at HOME, so production would still commit; by ball 50 (~44.5 cm, ~9 cm) it is L2 and
  // new commits are refused.
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.02, 0.05, 1.0 * kDeg, {}, 0.07), 1);
  EXPECT_EQ(RecoveryEnvelopeInstantLevel(0.02, 0.05, 1.0 * kDeg, {}, 0.09), 2);
  EXPECT_TRUE(RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, 2).block_new_commits);
  // The monitor threads the channel through Step with hysteresis intact.
  RecoveryEnvelopeMonitor m;
  EXPECT_EQ(m.Step(0.0, 0.0, 0.0, kDt).level, 0);
  EXPECT_EQ(m.Step(0.0, 0.0, 0.0, kDt, 0.13).level, 3);
  for (int i = 0; i < 14; ++i) EXPECT_EQ(m.Step(0.0, 0.0, 0.0, kDt, 0.02).level, 3);
  EXPECT_EQ(m.Step(0.0, 0.0, 0.0, kDt, 0.02).level, 0);
}

TEST(PpRecoveryEnvelope, FirstSampleAdoptsInstantLevelAndRisesImmediately) {
  RecoveryEnvelopeMonitor m;
  EXPECT_FALSE(m.initialized());
  EXPECT_EQ(m.level(), 3);  // unknown before the first sample -> fail closed
  auto u = m.Step(0.0, 0.0, 0.0, kDt);
  EXPECT_EQ(u.level, 0);
  EXPECT_EQ(u.previous_level, -1);
  EXPECT_TRUE(u.changed);
  u = m.Step(0.12, 0.14, 0.0, kDt);  // L0 -> L2 in one tick
  EXPECT_EQ(u.level, 2);
  EXPECT_EQ(u.previous_level, 0);
  EXPECT_TRUE(u.changed);
  u = m.Step(0.30, 0.0, 0.0, kDt);   // L2 -> L3 in one tick
  EXPECT_EQ(u.level, 3);
  EXPECT_TRUE(u.changed);
}

TEST(PpRecoveryEnvelope, DecreaseRequiresContinuousDwell) {
  RecoveryEnvelopeMonitor m;
  m.Step(0.30, 0.0, 0.0, kDt);  // L3
  ASSERT_EQ(m.level(), 3);
  // 14 ticks (0.28 s) of L0 conditions: still L3
  for (int i = 0; i < 14; ++i) {
    const auto u = m.Step(0.0, 0.0, 0.0, kDt);
    EXPECT_EQ(u.level, 3) << "tick " << i;
    EXPECT_FALSE(u.changed) << "tick " << i;
  }
  // 15th tick (0.30 s): drops straight to L0 (every lower level's conditions held)
  const auto u = m.Step(0.0, 0.0, 0.0, kDt);
  EXPECT_EQ(u.level, 0);
  EXPECT_EQ(u.previous_level, 3);
  EXPECT_TRUE(u.changed);
}

TEST(PpRecoveryEnvelope, DwellIsBrokenByReturningToTheCurrentLevel) {
  RecoveryEnvelopeMonitor m;
  m.Step(0.12, 0.0, 0.0, kDt);  // L2
  for (int i = 0; i < 10; ++i) m.Step(0.0, 0.0, 0.0, kDt);  // 0.20 s of L0
  EXPECT_EQ(m.level(), 2);
  m.Step(0.12, 0.0, 0.0, kDt);  // back to L2 -> dwell resets
  for (int i = 0; i < 14; ++i) m.Step(0.0, 0.0, 0.0, kDt);  // 0.28 s
  EXPECT_EQ(m.level(), 2);
  m.Step(0.0, 0.0, 0.0, kDt);  // 0.30 s
  EXPECT_EQ(m.level(), 0);
}

TEST(PpRecoveryEnvelope, FlappingBetweenLowerLevelsDropsOnlyToTheHighestHeld) {
  RecoveryEnvelopeMonitor m;
  m.Step(0.30, 0.0, 0.0, kDt);  // L3
  // Alternate L0 / L1 for 0.30 s: L1's conditions held throughout, L0's did not.
  for (int i = 0; i < 15; ++i) {
    const double d = (i % 2 == 0) ? 0.0 : 0.07;  // 0.07 -> L1
    m.Step(d, 0.0, 0.0, kDt);
  }
  EXPECT_EQ(m.level(), 1);
  // Another 0.30 s of pure L0 is needed to reach L0.
  for (int i = 0; i < 14; ++i) m.Step(0.0, 0.0, 0.0, kDt);
  EXPECT_EQ(m.level(), 1);
  m.Step(0.0, 0.0, 0.0, kDt);
  EXPECT_EQ(m.level(), 0);
}

TEST(PpRecoveryEnvelope, ResetForgetsHistory) {
  RecoveryEnvelopeMonitor m;
  m.Step(0.30, 0.0, 0.0, kDt);
  ASSERT_EQ(m.level(), 3);
  m.Reset();
  EXPECT_FALSE(m.initialized());
  EXPECT_EQ(m.level(), 3);
  const auto u = m.Step(0.0, 0.0, 0.0, kDt);
  EXPECT_EQ(u.level, 0);
  EXPECT_EQ(u.previous_level, -1);
}

TEST(PpRecoveryEnvelope, StepRejectsInvalidDt) {
  RecoveryEnvelopeMonitor m;
  EXPECT_THROW(m.Step(0.0, 0.0, 0.0, -kDt), std::invalid_argument);
  EXPECT_THROW(m.Step(0.0, 0.0, 0.0, kNaN), std::invalid_argument);
}

TEST(PpRecoveryEnvelope, CapabilityModeNeverBlocks) {
  for (int level = 0; level < kRecoveryEnvelopeLevelCount; ++level) {
    const auto g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kCapability, level);
    EXPECT_FALSE(g.block_new_commits) << level;
    EXPECT_FALSE(g.expire_pending) << level;
    EXPECT_STREQ(g.reason, kRecoveryEnvelopeReasonNone) << level;
  }
}

TEST(PpRecoveryEnvelope, ProductionModeBlocksAtL2AndExpiresAtL3) {
  auto g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, 0);
  EXPECT_FALSE(g.block_new_commits);
  EXPECT_FALSE(g.expire_pending);
  EXPECT_STREQ(g.reason, "none");
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, 1);
  EXPECT_FALSE(g.block_new_commits);
  EXPECT_FALSE(g.expire_pending);
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, 2);
  EXPECT_TRUE(g.block_new_commits);
  EXPECT_FALSE(g.expire_pending);
  EXPECT_STREQ(g.reason, "envelope_level2_recovering");
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, 3);
  EXPECT_TRUE(g.block_new_commits);
  EXPECT_TRUE(g.expire_pending);
  EXPECT_STREQ(g.reason, "envelope_level3_safe_hold");
}

TEST(PpRecoveryEnvelope, EndToEndMujocoFailureReplayInProduction) {
  // HOME hold (L0) -> recovery 12 cm off at 0.14 m/s (L2) -> falling (L3) -> recovered.
  RecoveryEnvelopeMonitor m;
  m.Step(0.01, 0.05, 1.0 * kDeg, kDt);
  auto g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, m.level());
  EXPECT_FALSE(g.block_new_commits);
  m.Step(0.12, 0.14, 2.0 * kDeg, kDt);
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, m.level());
  EXPECT_TRUE(g.block_new_commits);
  EXPECT_FALSE(g.expire_pending);
  m.Step(0.14, 0.30, 12.0 * kDeg, kDt);
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, m.level());
  EXPECT_TRUE(g.expire_pending);
  // Recovery back to HOME: blocks persist through the 0.30 s dwell, then release.
  for (int i = 0; i < 14; ++i) {
    m.Step(0.02, 0.05, 1.0 * kDeg, kDt);
    EXPECT_TRUE(RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, m.level())
                    .expire_pending) << i;
  }
  m.Step(0.02, 0.05, 1.0 * kDeg, kDt);
  g = RecoveryEnvelopeGateFor(RecoveryEnvelopeMode::kProduction, m.level());
  EXPECT_FALSE(g.block_new_commits);
  EXPECT_EQ(m.level(), 0);
}

}  // namespace
}  // namespace a3_pingpong
