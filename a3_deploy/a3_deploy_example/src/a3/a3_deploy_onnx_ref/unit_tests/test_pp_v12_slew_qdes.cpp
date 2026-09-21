// Host parity tests for the v12_affine_safe_slew_qdes_v1 action contract.
// Pure arithmetic only (pp_qdes_contract.hpp) -- no ONNX runtime required.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "a3_pingpong/pp_deploy_contract.hpp"
#include "a3_pingpong/pp_qdes_contract.hpp"

namespace a3_pingpong {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

V12SlewScaleParams SpecParams() {
  // Exact values from the contract specification.
  return {0.0698, 0.1745, 0.25, 0.50, 0.35};
}

// Isaac/action order used by the deployed models: the head lives at 11/16 and
// the 12 hip/knee/ankle joints are interleaved.  Built from the backend order
// so the leg/head classification is exercised on realistic names.
std::vector<std::string> ActionOrderJointNames() {
  // A representative interleaved order (matches hitter_pure_v4 head slots 11/16).
  return {
      "left_hip_pitch_joint",      // 0
      "right_hip_pitch_joint",     // 1
      "waist_yaw_joint",           // 2
      "left_hip_roll_joint",       // 3
      "right_hip_roll_joint",      // 4
      "waist_roll_joint",          // 5
      "left_hip_yaw_joint",        // 6
      "right_hip_yaw_joint",       // 7
      "waist_pitch_joint",         // 8
      "left_knee_joint",           // 9
      "right_knee_joint",          // 10
      "head_yaw_joint",            // 11
      "left_shoulder_pitch_joint", // 12
      "right_shoulder_pitch_joint",// 13
      "left_ankle_pitch_joint",    // 14
      "right_ankle_pitch_joint",   // 15
      "head_pitch_joint",          // 16
      "left_shoulder_roll_joint",  // 17
      "right_shoulder_roll_joint", // 18
      "left_ankle_roll_joint",     // 19
      "right_ankle_roll_joint",    // 20
      "left_shoulder_yaw_joint",   // 21
      "right_shoulder_yaw_joint",  // 22
      "left_elbow_joint",          // 23
      "right_elbow_joint",         // 24
      "left_wrist_roll_joint",     // 25
      "right_wrist_roll_joint",    // 26
      "left_wrist_pitch_joint",    // 27
      "right_wrist_pitch_joint",   // 28
      "left_wrist_yaw_joint",      // 29
      "right_wrist_yaw_joint",     // 30
  };
}

struct Fixture {
  std::vector<std::string> names = ActionOrderJointNames();
  std::vector<double> action_scale, slew, mask, default_q, safe_lo, safe_hi;
  std::vector<bool> passive;
  Fixture() {
    for (const std::string& n : names) {
      const bool leg = V12SlewJointIsLeg(n);
      const bool head = V12SlewJointIsPassiveHead(n);
      action_scale.push_back(head ? 0.25 : (leg ? 0.25 : 0.5));
      slew.push_back(head ? 0.0 : (leg ? 0.30 : 0.45));
      mask.push_back(leg ? 1.0 : 0.0);
      default_q.push_back(leg ? -0.10 : 0.05);
      safe_lo.push_back(-2.0);
      safe_hi.push_back(2.0);
      passive.push_back(head);
    }
  }
  std::vector<char> passive_bytes() const {
    std::vector<char> out(passive.size());
    for (std::size_t i = 0; i < passive.size(); ++i) out[i] = passive[i] ? 1 : 0;
    return out;
  }
};

TEST(PpV12SlewQdes, ReachingDeployedKneeLimitDoesNotEscapeByCancellation) {
  const double lo = 0.00872659683;
  const auto result = ApplyV12SlewSafeQdes(
      -100., .25, .25, lo, 2.36492062, .15, .300000012, true, 1.);
  EXPECT_FALSE(result.saturated);
  EXPECT_EQ(result.q_hat, lo);
  EXPECT_GE(result.q_hat, lo);
}

TEST(PpV12SlewQdes, ReachingUpperLimitUsesExactEndpoint) {
  const double hi = -0.0479966402;
  for (double previous : {-.1, -.2, -.3, -.4, -.6}) {
    const auto result = ApplyV12SlewSafeQdes(
        100., -.13, .25, -2.48273039, hi, previous, .7, false, 1.);
    EXPECT_FALSE(result.saturated);
    EXPECT_EQ(result.q_hat, hi);
    EXPECT_LE(result.q_hat, hi);
  }
}

TEST(PpV12SlewQdes, ScaleFactorIsOneInsideFullScaleRegion) {
  const auto p = SpecParams();
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.0, 0.0, p).scale, 1.0);
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.0698, 0.25, p).scale, 1.0);
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.01, 0.10, p).scale, 1.0);
}

TEST(PpV12SlewQdes, ScaleFactorRampsLinearlyAndFloorsAtMinScale) {
  const auto p = SpecParams();
  // tilt half way between theta0 and theta1, speed quiet -> f_tilt = 0.5
  const double tilt_mid = 0.5 * (p.tilt_full_scale_rad + p.tilt_min_scale_rad);
  const auto mid = ComputeV12SlewScale(tilt_mid, 0.0, p);
  EXPECT_NEAR(mid.f_tilt, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(mid.f_speed, 1.0);
  EXPECT_NEAR(mid.scale, 0.35 + 0.65 * 0.5, 1e-12);
  // speed half way, tilt quiet -> f_speed = 0.5
  const auto smid = ComputeV12SlewScale(0.0, 0.375, p);
  EXPECT_NEAR(smid.f_speed, 0.5, 1e-12);
  EXPECT_NEAR(smid.scale, 0.35 + 0.65 * 0.5, 1e-12);
  // min(f_tilt, f_speed): tilt 0.75 of the way (f=0.25), speed 0.25 of the way (f=0.75)
  const double tilt_q3 = p.tilt_full_scale_rad + 0.75 * (p.tilt_min_scale_rad - p.tilt_full_scale_rad);
  const auto both = ComputeV12SlewScale(tilt_q3, 0.3125, p);
  EXPECT_NEAR(both.f_tilt, 0.25, 1e-12);
  EXPECT_NEAR(both.f_speed, 0.75, 1e-12);
  EXPECT_NEAR(both.scale, 0.35 + 0.65 * 0.25, 1e-12);
  // at/above the min-scale thresholds -> exactly s_min, never below
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.1745, 0.0, p).scale, 0.35);
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.0, 0.50, p).scale, 0.35);
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(1.0, 3.0, p).scale, 0.35);
  // Example from the failure evidence: 0.14 m/s base speed, small tilt -> full scale.
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.02, 0.14, p).scale, 1.0);
}

TEST(PpV12SlewQdes, ScaleFactorConservativelyHandlesNonFiniteInputsAndRejectsBadParams) {
  const auto p = SpecParams();
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(kNaN, 0.0, p).scale, p.min_scale);
  EXPECT_DOUBLE_EQ(ComputeV12SlewScale(0.0, kInf, p).scale, p.min_scale);
  V12SlewScaleParams bad = p;
  bad.tilt_min_scale_rad = bad.tilt_full_scale_rad;  // zero-width ramp
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  bad = p; bad.speed_min_scale_mps = 0.10;            // v1 < v0
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  bad = p; bad.min_scale = 0.0;
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  bad = p; bad.min_scale = 1.0001;
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  bad = p; bad.tilt_full_scale_rad = -0.01;
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  bad = p; bad.speed_full_scale_mps = kNaN;
  EXPECT_THROW(ValidateV12SlewScaleParams(bad), std::runtime_error);
  EXPECT_NO_THROW(ValidateV12SlewScaleParams(p));
}

TEST(PpV12SlewQdes, TransparentBelowTheBound) {
  // default -0.10, scale 0.25, prev = -0.10, a = 0.8 -> q_nom = 0.10, delta = 0.20 < D = 0.30
  const auto r = ApplyV12SlewSafeQdes(0.8, -0.10, 0.25, -2.0, 2.0, -0.10, 0.30, true, 1.0);
  EXPECT_DOUBLE_EQ(r.q_nominal, ComputeV11AffineSafeQdes(0.8, -0.10, 0.25, -2.0, 2.0));
  EXPECT_DOUBLE_EQ(r.q_hat, r.q_nominal);
  EXPECT_FALSE(r.saturated);
  EXPECT_DOUBLE_EQ(r.clip_rad, 0.0);
  EXPECT_DOUBLE_EQ(r.bound_rad, 0.30);
  // exactly at the bound is still transparent (clip is inclusive)
  const auto edge = ApplyV12SlewSafeQdes(1.2, -0.10, 0.25, -2.0, 2.0, -0.10, 0.30, true, 1.0);
  EXPECT_NEAR(edge.delta_requested, 0.30, 1e-12);
  EXPECT_NEAR(edge.q_hat, 0.20, 1e-12);
  EXPECT_FALSE(edge.saturated);
}

TEST(PpV12SlewQdes, ClippedAboveTheBoundBothDirections) {
  // The failure evidence: a 1.5 rad hip-pitch request in one tick.
  const auto up = ApplyV12SlewSafeQdes(6.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 1.0);
  EXPECT_DOUBLE_EQ(up.q_nominal, 1.5);
  EXPECT_DOUBLE_EQ(up.q_hat, 0.30);
  EXPECT_TRUE(up.saturated);
  EXPECT_NEAR(up.clip_rad, 1.2, 1e-12);
  const auto down = ApplyV12SlewSafeQdes(-6.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 1.0);
  EXPECT_DOUBLE_EQ(down.q_hat, -0.30);
  EXPECT_TRUE(down.saturated);
  // Bang-bang between +/-2 rad rails from a previous +2: one tick moves only -D.
  const auto bang = ApplyV12SlewSafeQdes(-100.0, 0.0, 0.25, -2.0, 2.0, 2.0, 0.30, true, 1.0);
  EXPECT_DOUBLE_EQ(bang.q_nominal, -2.0);
  EXPECT_DOUBLE_EQ(bang.q_hat, 1.7);
  EXPECT_NEAR(bang.clip_rad, 3.7, 1e-12);
}

TEST(PpV12SlewQdes, LegBudgetScalesWithScaleFactorArmsDoNot) {
  // scale 0.35: leg D = 0.30 * 0.35 = 0.105, arm D stays 0.45
  const auto leg = ApplyV12SlewSafeQdes(6.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 0.35);
  EXPECT_NEAR(leg.bound_rad, 0.105, 1e-12);
  EXPECT_NEAR(leg.q_hat, 0.105, 1e-12);
  const auto arm = ApplyV12SlewSafeQdes(6.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.45, false, 0.35);
  EXPECT_DOUBLE_EQ(arm.bound_rad, 0.45);
  EXPECT_DOUBLE_EQ(arm.q_hat, 0.45);
}

TEST(PpV12SlewQdes, StepRejectsInvalidInputs) {
  EXPECT_THROW(ApplyV12SlewSafeQdes(kNaN, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 1.0),
               std::invalid_argument);
  EXPECT_THROW(ApplyV12SlewSafeQdes(0.0, 0.0, 0.25, -2.0, 2.0, kInf, 0.30, true, 1.0),
               std::invalid_argument);
  EXPECT_THROW(ApplyV12SlewSafeQdes(0.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.0, true, 1.0),
               std::invalid_argument);   // zero slew for an executable joint
  EXPECT_THROW(ApplyV12SlewSafeQdes(0.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 0.0),
               std::invalid_argument);   // scale 0
  EXPECT_THROW(ApplyV12SlewSafeQdes(0.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 1.5),
               std::invalid_argument);   // scale > 1
}

TEST(PpV12SlewQdes, FeedbackIsExecutedTargetInRawActionCoordinates) {
  EXPECT_DOUBLE_EQ(ComputeV12ExecutedQdesFeedback(0.30, 0.0, 0.25), 1.2);
  EXPECT_DOUBLE_EQ(ComputeV12ExecutedQdesFeedback(-0.10, -0.10, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(ComputeV12ExecutedQdesFeedback(0.15, 0.05, 0.5), 0.2);
  EXPECT_THROW(ComputeV12ExecutedQdesFeedback(0.1, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(ComputeV12ExecutedQdesFeedback(kNaN, 0.0, 0.25), std::invalid_argument);
  // Round trip: a request inside the bound feeds back exactly the raw action.
  const auto r = ApplyV12SlewSafeQdes(0.8, -0.10, 0.25, -2.0, 2.0, -0.10, 0.30, true, 1.0);
  EXPECT_NEAR(ComputeV12ExecutedQdesFeedback(r.q_hat, -0.10, 0.25), 0.8, 1e-12);
  // A saturated request feeds back the executed (clipped) action, not the raw one.
  const auto s = ApplyV12SlewSafeQdes(6.0, 0.0, 0.25, -2.0, 2.0, 0.0, 0.30, true, 1.0);
  EXPECT_NEAR(ComputeV12ExecutedQdesFeedback(s.q_hat, 0.0, 0.25), 1.2, 1e-12);
}

TEST(PpV12SlewQdes, VectorStepPinsPassiveHeadAndReportsTelemetry) {
  Fixture f;
  const auto passive = f.passive_bytes();
  std::vector<double> raw(31, 0.0), prev(f.default_q), q_hat(31, 0.0), fb(31, 99.0);
  raw[0] = 6.0;     // left_hip_pitch: 1.5 rad request -> clipped to 0.30 (leg, scale 1)
  raw[12] = 1.0;    // left_shoulder_pitch: 0.5 rad request -> clipped to 0.45 (arm)
  raw[11] = 3.0;    // head_yaw: must be ignored
  raw[16] = -3.0;   // head_pitch: must be ignored
  raw[9] = 0.4;     // left_knee: 0.10 rad request -> transparent
  const auto t = ApplyV12SlewSafeQdesVector(
      31, raw.data(), f.default_q.data(), f.action_scale.data(), f.safe_lo.data(),
      f.safe_hi.data(), prev.data(), f.slew.data(), f.mask.data(),
      reinterpret_cast<const bool*>(passive.data()), 0.0, 0.0, SpecParams(),
      q_hat.data(), fb.data());
  EXPECT_DOUBLE_EQ(t.scale, 1.0);
  EXPECT_EQ(t.saturated_count, 2);
  EXPECT_EQ(t.saturated_leg_count, 1);
  EXPECT_NEAR(t.max_clip_rad, 1.2, 1e-12);
  // head slots: q_hat = default, feedback 0, regardless of the raw action
  EXPECT_DOUBLE_EQ(q_hat[11], f.default_q[11]);
  EXPECT_DOUBLE_EQ(q_hat[16], f.default_q[16]);
  EXPECT_DOUBLE_EQ(fb[11], 0.0);
  EXPECT_DOUBLE_EQ(fb[16], 0.0);
  // saturated leg
  EXPECT_NEAR(q_hat[0], f.default_q[0] + 0.30, 1e-12);
  EXPECT_NEAR(fb[0], 1.2, 1e-12);
  // saturated arm
  EXPECT_NEAR(q_hat[12], f.default_q[12] + 0.45, 1e-12);
  EXPECT_NEAR(fb[12], 0.9, 1e-12);
  // transparent knee
  EXPECT_NEAR(q_hat[9], f.default_q[9] + 0.10, 1e-12);
  EXPECT_NEAR(fb[9], 0.4, 1e-12);
  // untouched joints stay at default with zero feedback
  EXPECT_DOUBLE_EQ(q_hat[30], f.default_q[30]);
  EXPECT_DOUBLE_EQ(fb[30], 0.0);

  // Second tick with the same request: the leg keeps walking toward q_nom by D.
  prev = q_hat;
  const auto t2 = ApplyV12SlewSafeQdesVector(
      31, raw.data(), f.default_q.data(), f.action_scale.data(), f.safe_lo.data(),
      f.safe_hi.data(), prev.data(), f.slew.data(), f.mask.data(),
      reinterpret_cast<const bool*>(passive.data()), 0.0, 0.0, SpecParams(),
      q_hat.data(), fb.data());
  EXPECT_NEAR(q_hat[0], f.default_q[0] + 0.60, 1e-12);
  EXPECT_EQ(t2.saturated_leg_count, 1);
  // Arm reached its 0.5 rad request (0.45 + 0.05) -> no longer saturated.
  EXPECT_NEAR(q_hat[12], f.default_q[12] + 0.50, 1e-12);
  EXPECT_EQ(t2.saturated_count, 1);
}

TEST(PpV12SlewQdes, VectorStepUnderTiltShrinksOnlyLegBudget) {
  Fixture f;
  const auto passive = f.passive_bytes();
  std::vector<double> raw(31, 6.0), q_hat(31, 0.0);
  raw[11] = raw[16] = 0.0;
  // tilt >= 10 deg -> scale 0.35: legs D = 0.30*0.35 = 0.105, arms D = 0.45
  const auto t = ApplyV12SlewSafeQdesVector(
      31, raw.data(), f.default_q.data(), f.action_scale.data(), f.safe_lo.data(),
      f.safe_hi.data(), f.default_q.data(), f.slew.data(), f.mask.data(),
      reinterpret_cast<const bool*>(passive.data()), 0.20, 0.0, SpecParams(),
      q_hat.data(), nullptr);
  EXPECT_DOUBLE_EQ(t.scale, 0.35);
  EXPECT_EQ(t.saturated_leg_count, 12);
  EXPECT_EQ(t.saturated_count, 29);
  for (int j = 0; j < 31; ++j) {
    if (f.passive[j]) continue;
    const double step = q_hat[j] - f.default_q[j];
    EXPECT_NEAR(step, f.mask[j] != 0.0 ? 0.105 : 0.45, 1e-12) << "joint " << j;
  }
}

TEST(PpV12SlewQdes, V11DecodeIsUnchangedAndV12WithLooseBoundReproducesIt) {
  // The V11 reference cases (byte-identical to the Python exporter reference).
  EXPECT_DOUBLE_EQ(ComputeV11AffineSafeQdes(-10.0, 0.1, 0.2, -0.5, 0.5), -0.5);
  EXPECT_DOUBLE_EQ(ComputeV11AffineSafeQdes(0.25, -0.2, 0.4, -0.5, 0.5), -0.1);
  EXPECT_DOUBLE_EQ(ComputeV11AffineSafeQdes(10.0, 0.3, 0.5, -0.5, 0.5), 0.5);
  // Regression sweep: with a bound larger than any possible step the v12 executed target is
  // bit-identical to the V11 decode, so v12's only effect is the slew.
  for (int k = -40; k <= 40; ++k) {
    const double a = 0.25 * k;
    const double v11 = ComputeV11AffineSafeQdes(a, 0.05, 0.5, -2.0, 2.0);
    const auto v12 = ApplyV12SlewSafeQdes(a, 0.05, 0.5, -2.0, 2.0, 0.05, 100.0, true, 1.0);
    EXPECT_DOUBLE_EQ(v12.q_hat, v11);
    EXPECT_DOUBLE_EQ(v12.q_nominal, v11);
    EXPECT_FALSE(v12.saturated);
  }
}

TEST(PpV12SlewQdes, MetadataValidatorAcceptsSpecShapedMetadata) {
  Fixture f;
  EXPECT_NO_THROW(ValidateV12SlewContractMetadata(
      f.names, f.action_scale, f.slew, f.mask, SpecParams(),
      kV12ExecutedQdesRawFeedbackContract));
  // The backend SDK order is also a legal action order (12 legs, 2 heads).
  std::vector<std::string> sdk_names;
  for (const std::string_view n : kA3BackendJointOrder) sdk_names.emplace_back(n);
  std::vector<double> scale(31, 0.5), slew(31, 0.3), mask(31, 0.0);
  for (std::size_t j = 0; j < 31; ++j) {
    if (V12SlewJointIsLeg(sdk_names[j])) mask[j] = 1.0;
    if (V12SlewJointIsPassiveHead(sdk_names[j])) slew[j] = 0.0;
  }
  EXPECT_NO_THROW(ValidateV12SlewContractMetadata(
      sdk_names, scale, slew, mask, SpecParams(), kV12ExecutedQdesRawFeedbackContract));
}

TEST(PpV12SlewQdes, MetadataValidatorFailsClosed) {
  const Fixture f;
  const auto p = SpecParams();
  const std::string fb = kV12ExecutedQdesRawFeedbackContract;
  auto expect_throw = [&](const std::vector<double>& scale, const std::vector<double>& slew,
                          const std::vector<double>& mask, const V12SlewScaleParams& params,
                          const std::string& feedback, const char* what) {
    EXPECT_THROW(ValidateV12SlewContractMetadata(f.names, scale, slew, mask, params, feedback),
                 std::runtime_error) << what;
  };
  {  // wrong lengths
    std::vector<double> short_slew(f.slew.begin(), f.slew.end() - 1);
    expect_throw(f.action_scale, short_slew, f.mask, p, fb, "short slew");
    std::vector<double> long_mask(f.mask); long_mask.push_back(0.0);
    expect_throw(f.action_scale, f.slew, long_mask, p, fb, "long mask");
    std::vector<double> short_scale(f.action_scale.begin(), f.action_scale.end() - 2);
    expect_throw(short_scale, f.slew, f.mask, p, fb, "short scale");
  }
  {  // non-finite / negative / zero slew on an executable joint
    auto slew = f.slew; slew[0] = kNaN;
    expect_throw(f.action_scale, slew, f.mask, p, fb, "nan slew");
    slew = f.slew; slew[12] = -0.1;
    expect_throw(f.action_scale, slew, f.mask, p, fb, "negative slew");
    slew = f.slew; slew[9] = 0.0;
    expect_throw(f.action_scale, slew, f.mask, p, fb, "zero slew executable");
    slew = f.slew; slew[11] = -0.0001;  // head: negative is still rejected
    expect_throw(f.action_scale, slew, f.mask, p, fb, "negative head slew");
    slew = f.slew; slew[11] = 0.0; slew[16] = 0.0;  // head zero is fine
    EXPECT_NO_THROW(ValidateV12SlewContractMetadata(f.names, f.action_scale, slew, f.mask, p, fb));
  }
  {  // leg mask problems
    auto mask = f.mask; mask[0] = 0.5;
    expect_throw(f.action_scale, f.slew, mask, p, fb, "non-binary mask");
    mask = f.mask; mask[0] = 0.0;  // a hip unmasked
    expect_throw(f.action_scale, f.slew, mask, p, fb, "leg unmasked");
    mask = f.mask; mask[12] = 1.0;  // a shoulder masked
    expect_throw(f.action_scale, f.slew, mask, p, fb, "arm masked");
    mask = f.mask; mask[3] = kInf;
    expect_throw(f.action_scale, f.slew, mask, p, fb, "inf mask");
  }
  {  // action scale must be positive for executable joints (feedback divides by it)
    auto scale = f.action_scale; scale[23] = 0.0;
    expect_throw(scale, f.slew, f.mask, p, fb, "zero action scale");
    scale = f.action_scale; scale[23] = -0.5;
    expect_throw(scale, f.slew, f.mask, p, fb, "negative action scale");
    scale = f.action_scale; scale[11] = 0.0;  // head scale may be zero
    EXPECT_NO_THROW(ValidateV12SlewContractMetadata(f.names, scale, f.slew, f.mask, p, fb));
  }
  {  // scale params
    auto bad = p; bad.min_scale = 0.0;
    expect_throw(f.action_scale, f.slew, f.mask, bad, fb, "min_scale 0");
    bad = p; bad.tilt_min_scale_rad = 0.05;  // below full-scale
    expect_throw(f.action_scale, f.slew, f.mask, bad, fb, "tilt ramp inverted");
  }
  {  // feedback contract
    expect_throw(f.action_scale, f.slew, f.mask, p, "", "missing feedback");
    expect_throw(f.action_scale, f.slew, f.mask, p, "executed_qdes_normalized", "v15 feedback");
    expect_throw(f.action_scale, f.slew, f.mask, p, "raw", "raw feedback");
  }
  {  // joint set problems
    auto names = f.names; names[11] = "neck_joint";  // one head missing
    EXPECT_THROW(ValidateV12SlewContractMetadata(names, f.action_scale, f.slew, f.mask, p, fb),
                 std::runtime_error);
    names = f.names; names[0] = "left_toe_joint";  // 11 legs; mask[0]=1 now mismatches
    EXPECT_THROW(ValidateV12SlewContractMetadata(names, f.action_scale, f.slew, f.mask, p, fb),
                 std::runtime_error);
    names = f.names; names.pop_back();
    std::vector<double> s30(f.action_scale.begin(), f.action_scale.end() - 1);
    std::vector<double> l30(f.slew.begin(), f.slew.end() - 1);
    std::vector<double> m30(f.mask.begin(), f.mask.end() - 1);
    EXPECT_THROW(ValidateV12SlewContractMetadata(names, s30, l30, m30, p, fb),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace a3_pingpong
