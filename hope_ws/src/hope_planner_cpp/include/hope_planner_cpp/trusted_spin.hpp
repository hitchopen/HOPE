#pragma once
#include "hope_planner_cpp/common_ball_physics.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
namespace hope_planner_cpp {
// Owned by one physical shot. A missing measurement breaks only differencing.
class TrustedSpin {
public:
  explicit TrustedSpin(SpinEstimatorConfig c = {}, double coherence = .4)
      : estimator_(c), coherence_(coherence) {}
  void reset() {
    estimator_.reset();
    value_ = {};
    last_time_s_.reset();
  }
  const EffectiveSpin &value() const { return value_; }
  void restore(const EffectiveSpin &s) {
    value_ = s;
    estimator_.reset();
    last_time_s_.reset();
  }
  EffectiveSpin observe(const BallSample &sample) {
    if (last_time_s_ && sample.source_time_s <= *last_time_s_)
      return value_;
    last_time_s_ = sample.source_time_s;
    bool measured = false;
    if (sample.producer_omega_valid &&
        sample.producer_omega_rad_s.allFinite() &&
        sample.producer_omega_rad_s.squaredNorm() > 0) {
      value_.omega = sample.producer_omega_rad_s;
      measured = true;
      value_.provenance =
          "authoritative_producer_rad_s;conditional_model_uncertainty";
      estimator_.reset();
    } else {
      auto qualified = sample;
      qualified.orientation_valid =
          sample.orientation_valid && sample.orientation_authoritative;
      estimator_.push(qualified);
      auto s = estimator_.estimate();
      if (estimator_.current_increment_valid() && s.valid &&
          s.coherence >= coherence_ && s.omega_rad_s.allFinite() &&
          s.omega_rad_s.squaredNorm() > 0) {
        value_.omega = s.omega_rad_s;
        measured = true;
        value_.provenance = "authoritative_quaternion_increments;constant_spin_"
                            "prior;conditional_uncertainty";
      }
    }
    if (measured) {
      value_.measurement_time_ns = sample.source_time_ns;
      value_.mode = EffectiveSpinMode::MEASURED;
    } else if (value_.usable() &&
               value_.mode != EffectiveSpinMode::CONTACT_PROPAGATED)
      value_.mode = EffectiveSpinMode::HELD;
    return value_;
  }
  // Returns false for repeated/out-of-order contact IDs; no second impulse.
  bool contact(std::uint64_t id, std::int64_t time_ns, const Vec3 &pre_velocity,
               const BallPhysics &p) {
    if (!id || id <= value_.last_contact_id)
      return false;
    if (value_.usable() && value_.measurement_time_ns <= time_ns) {
      ModelState s;
      s.velocity = pre_velocity;
      s.spin = value_;
      apply_model_contact(s, p);
      value_ = s.spin;
    }
    value_.last_contact_id = id;
    estimator_.reset();
    return true;
  }

private:
  SpinEstimator estimator_;
  double coherence_;
  EffectiveSpin value_;
  std::optional<double> last_time_s_;
};
} // namespace hope_planner_cpp
