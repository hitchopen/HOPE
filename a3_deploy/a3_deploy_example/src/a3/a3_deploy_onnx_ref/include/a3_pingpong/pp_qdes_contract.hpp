// Scalar, allocation-free feasible_qdes_v3 contract shared by the C++ runtime
// and deterministic host parity tests.
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace a3_pingpong {

enum class QdesTightestConstraint : int {
  kSafe = 0,
  kRate = 1,
  kTracking = 2,
  kTorque = 3,
};

struct FeasibleQdesV3Input {
  double raw_action;
  double previous_qdes;
  double q;
  double qd;
  double safe_lo;
  double safe_hi;
  double rate_limit;
  double tracking_limit;
  double kp;
  double kd;
  double effort_limit;
  double torque_headroom;
  double dt;
  double actual_q_guard_horizon_s;
  bool actual_q_guard_enabled;
};

struct FeasibleQdesV3Result {
  double qdes;
  double selected_lo;
  double selected_hi;
  double rate_lo;
  double rate_hi;
  double tracking_lo;
  double tracking_hi;
  double torque_lo;
  double torque_hi;
  double safe_torque_lo;
  double safe_torque_hi;
  double base_lo;
  double base_hi;
  double full_lo;
  double full_hi;
  double normalized_action;
  int fallback_level;
  QdesTightestConstraint tightest;
  bool full_feasible;
  bool base_feasible;
  bool safe_torque_feasible;
  bool actual_q_guard_active;
  bool actual_q_guard_upper;
  bool actual_q_guard_lower;
};

inline FeasibleQdesV3Result ComputeFeasibleQdesV3(
    const FeasibleQdesV3Input& in) {
  const double values[] = {
      in.raw_action, in.previous_qdes, in.q, in.qd, in.safe_lo,
      in.safe_hi, in.rate_limit, in.tracking_limit, in.kp, in.kd,
      in.effort_limit, in.torque_headroom, in.dt,
      in.actual_q_guard_horizon_s};
  for (double value : values) {
    if (!std::isfinite(value))
      throw std::invalid_argument("feasible_qdes_v3 input contains NaN/Inf");
  }
  if (!(in.safe_lo < in.safe_hi) || in.rate_limit < 0.0 ||
      in.tracking_limit < 0.0 || !(in.kp > 0.0) || in.kd < 0.0 ||
      !(in.effort_limit > 0.0) || !(in.torque_headroom > 0.0) ||
      in.torque_headroom > 1.0 || !(in.dt > 0.0) ||
      (in.actual_q_guard_enabled &&
       in.actual_q_guard_horizon_s < in.dt)) {
    throw std::invalid_argument("feasible_qdes_v3 input contract is invalid");
  }

  FeasibleQdesV3Result out{};
  const double rate_delta = in.rate_limit * in.dt;
  out.rate_lo = in.previous_qdes - rate_delta;
  out.rate_hi = in.previous_qdes + rate_delta;
  out.tracking_lo = in.q - in.tracking_limit;
  out.tracking_hi = in.q + in.tracking_limit;
  const double torque_limit = in.effort_limit * in.torque_headroom;
  out.torque_lo = in.q + (in.kd * in.qd - torque_limit) / in.kp;
  out.torque_hi = in.q + (in.kd * in.qd + torque_limit) / in.kp;
  out.safe_torque_lo = std::max(in.safe_lo, out.torque_lo);
  out.safe_torque_hi = std::min(in.safe_hi, out.torque_hi);
  out.safe_torque_feasible = out.safe_torque_lo <= out.safe_torque_hi;
  out.base_lo = std::max(out.safe_torque_lo, out.tracking_lo);
  out.base_hi = std::min(out.safe_torque_hi, out.tracking_hi);
  out.base_feasible = out.base_lo <= out.base_hi;
  out.full_lo = std::max(out.base_lo, out.rate_lo);
  out.full_hi = std::min(out.base_hi, out.rate_hi);
  out.full_feasible = out.full_lo <= out.full_hi;
  if (out.full_feasible) {
    out.selected_lo = out.full_lo;
    out.selected_hi = out.full_hi;
    out.fallback_level = 0;
  } else if (out.base_feasible) {
    out.selected_lo = out.base_lo;
    out.selected_hi = out.base_hi;
    out.fallback_level = 1;
  } else if (out.safe_torque_feasible) {
    out.selected_lo = out.safe_torque_lo;
    out.selected_hi = out.safe_torque_hi;
    out.fallback_level = 2;
  } else {
    out.selected_lo = in.safe_lo;
    out.selected_hi = in.safe_hi;
    out.fallback_level = 3;
  }
  const double predicted_q =
      in.q + in.qd * in.actual_q_guard_horizon_s;
  out.actual_q_guard_upper =
      in.actual_q_guard_enabled &&
      (in.q >= in.safe_hi ||
       (in.qd > 0.0 && predicted_q >= in.safe_hi));
  out.actual_q_guard_lower =
      in.actual_q_guard_enabled &&
      (in.q <= in.safe_lo ||
       (in.qd < 0.0 && predicted_q <= in.safe_lo));
  out.actual_q_guard_active =
      out.actual_q_guard_upper || out.actual_q_guard_lower;
  if (out.actual_q_guard_active) {
    const double emergency_lower = out.safe_torque_feasible
        ? out.safe_torque_lo : in.safe_lo;
    const double emergency_upper = out.safe_torque_feasible
        ? out.safe_torque_hi : in.safe_hi;
    const double emergency_target =
        out.actual_q_guard_upper ? emergency_lower : emergency_upper;
    out.selected_lo = emergency_target;
    out.selected_hi = emergency_target;
    out.fallback_level = out.safe_torque_feasible ? 2 : 3;
  }
  const double anchor =
      std::min(std::max(in.previous_qdes, out.selected_lo), out.selected_hi);
  out.normalized_action = std::tanh(in.raw_action);
  out.qdes = anchor +
      (out.normalized_action >= 0.0
           ? out.normalized_action * (out.selected_hi - anchor)
           : out.normalized_action * (anchor - out.selected_lo));

  constexpr double eps = 1.0e-7;
  out.tightest = QdesTightestConstraint::kSafe;
  if (out.torque_lo > in.safe_lo + eps ||
      out.torque_hi < in.safe_hi - eps)
    out.tightest = QdesTightestConstraint::kTorque;
  if (out.tracking_lo > out.safe_torque_lo + eps ||
      out.tracking_hi < out.safe_torque_hi - eps)
    out.tightest = QdesTightestConstraint::kTracking;
  if (out.full_feasible &&
      (out.rate_lo > out.base_lo + eps || out.rate_hi < out.base_hi - eps))
    out.tightest = QdesTightestConstraint::kRate;
  if (out.actual_q_guard_active)
    out.tightest = out.safe_torque_feasible
        ? QdesTightestConstraint::kTorque
        : QdesTightestConstraint::kSafe;
  if (!std::isfinite(out.qdes) || out.qdes < in.safe_lo ||
      out.qdes > in.safe_hi)
    throw std::runtime_error(
        "feasible_qdes_v3 produced a non-finite or position-unsafe q_des");
  return out;
}

inline double ClampPassiveDefaultToQdesInterval(
    double default_q, const FeasibleQdesV3Result& contract) {
  if (!std::isfinite(default_q))
    throw std::invalid_argument("passive q_des default contains NaN/Inf");
  return std::min(
      std::max(default_q, contract.selected_lo), contract.selected_hi);
}

// V17/V11 stateless action decoder.  Keeping this scalar helper free of
// ONNX/robot dependencies lets host tests compare it directly with the Python
// exporter reference without changing the deployed arithmetic.
inline double ComputeV11AffineSafeQdes(
    double raw_action, double default_q, double action_scale,
    double safe_lo, double safe_hi) {
  const double target = default_q + action_scale * raw_action;
  return std::min(std::max(target, safe_lo), safe_hi);
}

// ---------------------------------------------------------------------------
// v12_affine_safe_slew_qdes_v1
//
// Stateful successor of the V11 affine decoder.  Per 50 Hz tick and per joint:
//   q_nom   = ComputeV11AffineSafeQdes(a, default, scale, safe_lo, safe_hi)
//   f_tilt  = clip((theta1 - theta) / (theta1 - theta0), 0, 1)
//   f_speed = clip((v1 - v) / (v1 - v0), 0, 1)
//   s       = s_min + (1 - s_min) * min(f_tilt, f_speed)
//   D_j     = slew_j * (leg_mask_j ? s : 1)
//   q_hat_j = q_hat_prev_j + clip(q_nom_j - q_hat_prev_j, -D_j, +D_j)
// Passive head slots are NOT slewed: q_hat = default.  The action fed back to
// the policy is the executed target in raw-action coordinates,
//   a_fb_j = (q_hat_j - default_j) / scale_j       (0 for passive head slots).
// The training side implements exactly this arithmetic; keep every helper
// below scalar/pointer based (no Eigen, no ONNX) so host parity tests can run
// without the runtime.
// ---------------------------------------------------------------------------
inline constexpr char kV11AffineSafeQdesContract[] = "v11_affine_safe_qdes_v1";
inline constexpr char kV12AffineSafeSlewQdesContract[] =
    "v12_affine_safe_slew_qdes_v1";
inline constexpr char kV12ExecutedQdesRawFeedbackContract[] =
    "executed_qdes_raw_v12";
inline constexpr int kV12SlewJointCount = 31;
inline constexpr int kV12SlewLegJointCount = 12;

struct V12SlewScaleParams {
  double tilt_full_scale_rad;   // theta0: tilt at/below which legs get the full slew budget
  double tilt_min_scale_rad;    // theta1: tilt at/above which legs get s_min
  double speed_full_scale_mps;  // v0
  double speed_min_scale_mps;   // v1
  double min_scale;             // s_min in (0, 1]
};

struct V12SlewScaleResult {
  double f_tilt;
  double f_speed;
  double scale;
};

inline void ValidateV12SlewScaleParams(const V12SlewScaleParams& p) {
  const double values[] = {
      p.tilt_full_scale_rad, p.tilt_min_scale_rad, p.speed_full_scale_mps,
      p.speed_min_scale_mps, p.min_scale};
  for (double value : values) {
    if (!std::isfinite(value))
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 slew scale metadata contains NaN/Inf");
  }
  if (p.tilt_full_scale_rad < 0.0 ||
      !(p.tilt_min_scale_rad > p.tilt_full_scale_rad) ||
      p.speed_full_scale_mps < 0.0 ||
      !(p.speed_min_scale_mps > p.speed_full_scale_mps) ||
      !(p.min_scale > 0.0) || p.min_scale > 1.0) {
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 slew scale metadata is invalid "
        "(need 0 <= tilt_full < tilt_min, 0 <= speed_full < speed_min, "
        "0 < min_scale <= 1)");
  }
}

inline V12SlewScaleResult ComputeV12SlewScale(
    double tilt_rad, double speed_mps, const V12SlewScaleParams& p) {
  // Match the training tensor contract: an unknown actor-visible state gets the most
  // conservative leg budget.  The recovery-envelope monitor independently classifies the same
  // non-finite sample as L3; raw policy actions and q_des values remain strict finite
  // preconditions below.
  if (!std::isfinite(tilt_rad)) tilt_rad = p.tilt_min_scale_rad;
  if (!std::isfinite(speed_mps)) speed_mps = p.speed_min_scale_mps;
  V12SlewScaleResult out{};
  out.f_tilt = std::min(std::max(
      (p.tilt_min_scale_rad - tilt_rad) /
          (p.tilt_min_scale_rad - p.tilt_full_scale_rad), 0.0), 1.0);
  out.f_speed = std::min(std::max(
      (p.speed_min_scale_mps - speed_mps) /
          (p.speed_min_scale_mps - p.speed_full_scale_mps), 0.0), 1.0);
  out.scale = p.min_scale +
      (1.0 - p.min_scale) * std::min(out.f_tilt, out.f_speed);
  return out;
}

struct V12SlewJointResult {
  double q_nominal;        // V11 affine-safe request before slew
  double q_hat;            // executed target after slew
  double delta_requested;  // q_nominal - q_hat_prev (signed)
  double delta_applied;    // clipped step actually taken (signed)
  double clip_rad;         // |delta_requested| - |delta_applied| >= 0
  double bound_rad;        // D_j used this tick
  bool saturated;          // the clip bound was active
};

inline V12SlewJointResult ApplyV12SlewSafeQdes(
    double raw_action, double default_q, double action_scale,
    double safe_lo, double safe_hi, double q_hat_prev,
    double slew_limit_rad_per_tick, bool leg_joint, double scale) {
  const double values[] = {
      raw_action, default_q, action_scale, safe_lo, safe_hi, q_hat_prev,
      slew_limit_rad_per_tick, scale};
  for (double value : values) {
    if (!std::isfinite(value))
      throw std::invalid_argument("v12 slew input contains NaN/Inf");
  }
  if (!(slew_limit_rad_per_tick > 0.0) || !(scale > 0.0) || scale > 1.0)
    throw std::invalid_argument(
        "v12 slew requires slew_limit > 0 and 0 < scale <= 1");
  V12SlewJointResult out{};
  out.q_nominal = ComputeV11AffineSafeQdes(
      raw_action, default_q, action_scale, safe_lo, safe_hi);
  out.bound_rad = slew_limit_rad_per_tick * (leg_joint ? scale : 1.0);
  out.delta_requested = out.q_nominal - q_hat_prev;
  out.delta_applied =
      std::min(std::max(out.delta_requested, -out.bound_rad), out.bound_rad);
  out.saturated = std::fabs(out.delta_requested) > out.bound_rad;
  // When the full step fits, use the already bounded endpoint exactly.
  // prev + (nominal - prev) can round just OUTSIDE a safe bound (e.g. the
  // deployed knee lower limit by 1.2e-17 rad), falsely latching zero-gain halt.
  // This preserves the interval and slew budget; no safety tolerance is widened.
  out.q_hat = out.saturated ? q_hat_prev + out.delta_applied : out.q_nominal;
  out.clip_rad = std::fabs(out.delta_requested) - std::fabs(out.delta_applied);
  if (out.clip_rad < 0.0) out.clip_rad = 0.0;
  return out;
}

inline double ComputeV12ExecutedQdesFeedback(
    double q_hat, double default_q, double action_scale) {
  if (!std::isfinite(q_hat) || !std::isfinite(default_q) ||
      !std::isfinite(action_scale) || !(action_scale > 0.0))
    throw std::invalid_argument(
        "v12 executed-q_des feedback requires finite inputs and action_scale > 0");
  return (q_hat - default_q) / action_scale;
}

struct V12SlewVectorTelemetry {
  double scale = 1.0;
  int saturated_count = 0;      // executable joints where the clip bound was active
  int saturated_leg_count = 0;  // leg-mask joints where the clip bound was active
  double max_clip_rad = 0.0;    // largest |requested| - |applied| over all joints
};

// Commit an action-order target AFTER all downstream clamps/overrides. The
// state and feedback buffers may alias their corresponding input buffers.
inline void CommitV12FinalTarget(int n, const double* final_target,
    const double* default_q, const double* action_scale, const bool* passive_mask,
    double* slew_state, double* feedback) {
  for (int j = 0; j < n; ++j) {
    if (!std::isfinite(final_target[j]))
      throw std::runtime_error("non-finite final V12 target");
    slew_state[j] = final_target[j];
    feedback[j] = passive_mask[j] ? 0.0 :
        ComputeV12ExecutedQdesFeedback(final_target[j], default_q[j], action_scale[j]);
  }
}

// Whole-vector step.  `passive_mask[j]` marks the two passive head slots whose
// target is pinned to default (no slew, feedback 0).  `q_hat_prev` is read
// before `q_hat_out` is written so the two may alias.  Returns telemetry.
inline V12SlewVectorTelemetry ApplyV12SlewSafeQdesVector(
    int n, const double* raw_action, const double* default_q,
    const double* action_scale, const double* safe_lo, const double* safe_hi,
    const double* q_hat_prev, const double* slew_limit_rad_per_tick,
    const double* leg_mask, const bool* passive_mask, double tilt_rad,
    double speed_mps, const V12SlewScaleParams& params, double* q_hat_out,
    double* feedback_out) {
  V12SlewVectorTelemetry t;
  t.scale = ComputeV12SlewScale(tilt_rad, speed_mps, params).scale;
  for (int j = 0; j < n; ++j) {
    if (passive_mask[j]) {
      q_hat_out[j] = default_q[j];
      if (feedback_out) feedback_out[j] = 0.0;
      continue;
    }
    const V12SlewJointResult r = ApplyV12SlewSafeQdes(
        raw_action[j], default_q[j], action_scale[j], safe_lo[j], safe_hi[j],
        q_hat_prev[j], slew_limit_rad_per_tick[j], leg_mask[j] != 0.0, t.scale);
    q_hat_out[j] = r.q_hat;
    if (feedback_out)
      feedback_out[j] =
          ComputeV12ExecutedQdesFeedback(r.q_hat, default_q[j], action_scale[j]);
    if (r.saturated) {
      ++t.saturated_count;
      if (leg_mask[j] != 0.0) ++t.saturated_leg_count;
    }
    t.max_clip_rad = std::max(t.max_clip_rad, r.clip_rad);
  }
  return t;
}

inline bool V12SlewJointIsLeg(const std::string& joint_name) {
  return joint_name.find("hip") != std::string::npos ||
         joint_name.find("knee") != std::string::npos ||
         joint_name.find("ankle") != std::string::npos;
}

inline bool V12SlewJointIsPassiveHead(const std::string& joint_name) {
  return joint_name == "head_yaw_joint" || joint_name == "head_pitch_joint";
}

// Fail-closed metadata validation for the v12 contract.  Every array is in the
// ONNX action (Isaac) order carried by joint_names.  Throws std::runtime_error
// with a load-time message on the first violation.
inline void ValidateV12SlewContractMetadata(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& action_scale,
    const std::vector<double>& slew_limit_rad_per_tick,
    const std::vector<double>& leg_mask,
    const V12SlewScaleParams& params,
    const std::string& last_action_feedback) {
  if (joint_names.size() != static_cast<std::size_t>(kV12SlewJointCount))
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 requires 31 joint names");
  if (action_scale.size() != joint_names.size() ||
      slew_limit_rad_per_tick.size() != joint_names.size() ||
      leg_mask.size() != joint_names.size())
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 metadata array length != 31 "
        "(action_scale/qdes_slew_limit_rad_per_tick/qdes_slew_leg_mask)");
  ValidateV12SlewScaleParams(params);
  if (last_action_feedback != kV12ExecutedQdesRawFeedbackContract)
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 requires hitter_pure_last_action_feedback="
        "executed_qdes_raw_v12; got '" + last_action_feedback + "'");
  int leg_count = 0;
  int head_count = 0;
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    const std::string& name = joint_names[j];
    const double slew = slew_limit_rad_per_tick[j];
    const double mask = leg_mask[j];
    const double scale = action_scale[j];
    if (!std::isfinite(slew) || !std::isfinite(mask) || !std::isfinite(scale))
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 metadata contains NaN/Inf for '" +
          name + "'");
    if (slew < 0.0)
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 negative slew limit for '" + name + "'");
    if (mask != 0.0 && mask != 1.0)
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 leg mask must be exactly 0 or 1 for '" +
          name + "'");
    const bool is_leg = V12SlewJointIsLeg(name);
    if ((mask == 1.0) != is_leg)
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 leg mask does not match the hip/knee/"
          "ankle joint set at '" + name + "'");
    if (is_leg) ++leg_count;
    if (V12SlewJointIsPassiveHead(name)) {
      ++head_count;
      continue;  // passive: slew 0.0 (n/a) is legal, scale is unused
    }
    if (!(slew > 0.0))
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 slew limit must be > 0 for executable "
          "joint '" + name + "'");
    if (!(scale > 0.0))
      throw std::runtime_error(
          "v12_affine_safe_slew_qdes_v1 action_scale must be > 0 for executable "
          "joint '" + name + "' (executed-q_des feedback divides by it)");
  }
  if (leg_count != kV12SlewLegJointCount)
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 expects exactly 12 hip/knee/ankle joints");
  if (head_count != 2)
    throw std::runtime_error(
        "v12_affine_safe_slew_qdes_v1 expects exactly head_yaw_joint and "
        "head_pitch_joint as passive slots");
}

}  // namespace a3_pingpong
