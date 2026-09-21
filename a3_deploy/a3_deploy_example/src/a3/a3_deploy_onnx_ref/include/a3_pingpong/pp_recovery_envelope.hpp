// Recovery-envelope monitor (Layer C).
//
// Every control tick the runner classifies how far the robot is from its
// trained HOME hold using only actor-visible quantities:
//   d     = ||mocap base xy - session HOME xy||            [m]
//   v     = ||filtered mocap base velocity xy||             [m/s]  (0 when stale)
//   theta = asin(clip(||g_xy||, 0, 1)), g = projected gravity in the base frame
// into four nested levels
//   L0: d < 0.05 && v < 0.20 && theta < 4 deg
//   L1: d < 0.10 && v < 0.35 && theta < 7 deg
//   L2: d < 0.15 && v < 0.50 && theta < 10 deg
//   L3: otherwise (also any non-finite input -> fail closed)
// Schema33 adds an optional fourth, support-geometry input
//   a     = max_i ||sole_i xy - HOME anchor_i xy||        [m]   (leg FK + mocap pose)
//   L0: a < 0.04, L1: a < 0.08, L2: a < 0.12, L3 otherwise.
// It is OPTIONAL by contract: NaN means "not computed" and does not change the
// level (the three pelvis inputs keep their fail-closed semantics).
// Sole displacement can detect stance drift even while pelvis measurements
// remain inside their thresholds.
// with hysteresis: the level may rise immediately but may only fall after the
// lower level's conditions have held for 0.30 s continuously.
//
// Two runtime modes, selected by the PP_ENVELOPE_MODE environment variable:
//   capability (default): telemetry only.  Never blocks anything, so Gate3
//                         keeps exposing policy failures.
//   production:           L2 refuses NEW planner flight commits while the
//                         current flight continues; L3 additionally expires
//                         the pending flight into the neutral WAIT tuple so the
//                         trained HOME-return behaviour takes over.
// The monitor never produces a q_des; existing fall guards are untouched.
//
// Everything here is allocation-free and free of ONNX/robot dependencies so
// host unit tests can exercise thresholds, hysteresis and mode gating.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace a3_pingpong {

inline constexpr char kRecoveryEnvelopeModeEnv[] = "PP_ENVELOPE_MODE";
inline constexpr int kRecoveryEnvelopeLevelCount = 4;
inline constexpr int kRecoveryEnvelopeLevelSafeHold = 3;
inline constexpr int kRecoveryEnvelopeLevelRecovering = 2;
inline constexpr char kRecoveryEnvelopeReasonNone[] = "none";
inline constexpr char kRecoveryEnvelopeReasonLevel2[] =
    "envelope_level2_recovering";
inline constexpr char kRecoveryEnvelopeReasonLevel3[] =
    "envelope_level3_safe_hold";

enum class RecoveryEnvelopeMode { kCapability = 0, kProduction = 1 };

// nullptr / "" / "capability" -> capability; "production" -> production;
// anything else is a configuration error (fail closed at start-up).
inline RecoveryEnvelopeMode ParseRecoveryEnvelopeMode(const char* value) {
  if (value == nullptr || *value == '\0' ||
      std::strcmp(value, "capability") == 0)
    return RecoveryEnvelopeMode::kCapability;
  if (std::strcmp(value, "production") == 0)
    return RecoveryEnvelopeMode::kProduction;
  throw std::runtime_error(
      std::string(kRecoveryEnvelopeModeEnv) +
      " must be 'capability' or 'production'; got '" + value + "'");
}

inline const char* RecoveryEnvelopeModeName(RecoveryEnvelopeMode mode) {
  return mode == RecoveryEnvelopeMode::kProduction ? "production"
                                                   : "capability";
}

struct RecoveryEnvelopeThresholds {
  // Upper (exclusive) bounds for L0, L1, L2.  L3 is everything else.
  std::array<double, 3> home_dist_m = {0.05, 0.10, 0.15};
  std::array<double, 3> speed_mps = {0.20, 0.35, 0.50};
  std::array<double, 3> tilt_rad = {
      4.0 * M_PI / 180.0, 7.0 * M_PI / 180.0, 10.0 * M_PI / 180.0};
  // Schema33 support geometry: largest sole-vs-HOME-anchor error (m).
  std::array<double, 3> support_anchor_error_m = {0.04, 0.08, 0.12};
  // A level may decrease only after the lower level's conditions have held for
  // this long without interruption.
  double decrease_hold_s = 0.30;
};

// Tilt angle of the base from the projected-gravity unit vector.  ||g_xy|| is
// clipped to [0, 1] so a slightly non-unit vector cannot produce NaN.
inline double TiltFromProjectedGravity(double gx, double gy) {
  if (!std::isfinite(gx) || !std::isfinite(gy)) return NAN;
  const double gxy = std::min(std::max(std::hypot(gx, gy), 0.0), 1.0);
  return std::asin(gxy);
}

// Memoryless classification.  Non-finite pelvis input -> L3 (fail closed).  The optional
// Schema33 support-anchor error is ignored when NaN (not computed) and otherwise raises
// the level like the other three channels.
inline int RecoveryEnvelopeInstantLevel(
    double home_dist_m, double speed_mps, double tilt_rad,
    const RecoveryEnvelopeThresholds& thr = {},
    double support_anchor_error_m = NAN) {
  if (!std::isfinite(home_dist_m) || !std::isfinite(speed_mps) ||
      !std::isfinite(tilt_rad))
    return kRecoveryEnvelopeLevelSafeHold;
  const bool use_support = std::isfinite(support_anchor_error_m);
  for (int level = 0; level < 3; ++level) {
    const auto k = static_cast<std::size_t>(level);
    if (home_dist_m < thr.home_dist_m[k] && speed_mps < thr.speed_mps[k] &&
        tilt_rad < thr.tilt_rad[k] &&
        (!use_support || support_anchor_error_m < thr.support_anchor_error_m[k]))
      return level;
  }
  return kRecoveryEnvelopeLevelSafeHold;
}

class RecoveryEnvelopeMonitor {
 public:
  struct Update {
    int level = kRecoveryEnvelopeLevelSafeHold;   // hysteresis-filtered level
    int instant_level = kRecoveryEnvelopeLevelSafeHold;
    int previous_level = -1;                       // -1 on the first sample
    bool changed = false;
  };

  explicit RecoveryEnvelopeMonitor(RecoveryEnvelopeThresholds thr = {})
      : thr_(thr) {}

  const RecoveryEnvelopeThresholds& thresholds() const { return thr_; }
  bool initialized() const { return initialized_; }
  int level() const {
    return initialized_ ? level_ : kRecoveryEnvelopeLevelSafeHold;
  }
  double decrease_dwell_s() const { return dwell_s_; }

  // Forget all history (MOTION entry).  The next Step() adopts the
  // instantaneous level directly.
  void Reset() {
    initialized_ = false;
    level_ = kRecoveryEnvelopeLevelSafeHold;
    dwell_s_ = 0.0;
    dwell_max_level_ = -1;
  }

  Update Step(double home_dist_m, double speed_mps, double tilt_rad,
              double dt_s, double support_anchor_error_m = NAN) {
    if (!std::isfinite(dt_s) || dt_s < 0.0)
      throw std::invalid_argument("recovery envelope dt must be finite and >= 0");
    Update out;
    out.instant_level = RecoveryEnvelopeInstantLevel(
        home_dist_m, speed_mps, tilt_rad, thr_, support_anchor_error_m);
    if (!initialized_) {
      initialized_ = true;
      level_ = out.instant_level;
      dwell_s_ = 0.0;
      dwell_max_level_ = -1;
      out.previous_level = -1;
      out.changed = true;
      out.level = level_;
      return out;
    }
    out.previous_level = level_;
    if (out.instant_level >= level_) {
      // Rising (or re-touching the current level) is immediate and breaks any
      // continuous lower-level dwell.
      level_ = out.instant_level;
      dwell_s_ = 0.0;
      dwell_max_level_ = -1;
    } else {
      dwell_s_ += dt_s;
      dwell_max_level_ = std::max(dwell_max_level_, out.instant_level);
      if (dwell_s_ + 1.0e-9 >= thr_.decrease_hold_s) {
        // Every level >= dwell_max_level_ has had its conditions satisfied for
        // the whole dwell; drop to the highest such level and restart.
        level_ = dwell_max_level_;
        dwell_s_ = 0.0;
        dwell_max_level_ = -1;
      }
    }
    out.level = level_;
    out.changed = level_ != out.previous_level;
    return out;
  }

 private:
  RecoveryEnvelopeThresholds thr_;
  bool initialized_ = false;
  int level_ = kRecoveryEnvelopeLevelSafeHold;
  double dwell_s_ = 0.0;
  int dwell_max_level_ = -1;
};

struct RecoveryEnvelopeGate {
  bool block_new_commits = false;  // refuse NEW planner flight commits this tick
  bool expire_pending = false;     // expire the pending flight into the WAIT tuple
  const char* reason = kRecoveryEnvelopeReasonNone;
};

inline RecoveryEnvelopeGate RecoveryEnvelopeGateFor(
    RecoveryEnvelopeMode mode, int level) {
  RecoveryEnvelopeGate gate;
  if (mode != RecoveryEnvelopeMode::kProduction) return gate;
  if (level >= kRecoveryEnvelopeLevelSafeHold) {
    gate.block_new_commits = true;
    gate.expire_pending = true;
    gate.reason = kRecoveryEnvelopeReasonLevel3;
  } else if (level == kRecoveryEnvelopeLevelRecovering) {
    gate.block_new_commits = true;
    gate.reason = kRecoveryEnvelopeReasonLevel2;
  }
  return gate;
}

}  // namespace a3_pingpong
