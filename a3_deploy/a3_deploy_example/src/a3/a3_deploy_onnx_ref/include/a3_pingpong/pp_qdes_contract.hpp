// Scalar, allocation-free feasible_qdes_v3 contract shared by the C++ runtime
// and deterministic host parity tests.
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

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

}  // namespace a3_pingpong
