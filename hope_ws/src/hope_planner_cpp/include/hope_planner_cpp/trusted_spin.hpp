#pragma once
#include "hope_planner_cpp/common_ball_physics.hpp"
#include "hope_planner_cpp/flight_spin_estimator.hpp"
#include <algorithm>
#include <vector>
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
    contact_ids_.clear();
    last_contact_time_ns_.reset();
  }
  const EffectiveSpin &value() const { return value_; }
  void restore(const EffectiveSpin &s) {
    reset();
    value_ = s;
    if (s.last_contact_id)
      contact_ids_.push_back(s.last_contact_id);
  }
  // Restored owners supply the bounded confirmed ledger, without reapplying it.
  void remember_contact(std::uint64_t id, std::int64_t ns) {
    if (std::find(contact_ids_.begin(), contact_ids_.end(), id) ==
        contact_ids_.end())
      contact_ids_.push_back(id);
    if (id == value_.last_contact_id)
      last_contact_time_ns_ = ns;
  }
  bool can_apply_contact(std::int64_t ns) const {
    return !value_.usable() || ns < value_.measurement_time_ns ||
           !last_contact_time_ns_ || ns >= *last_contact_time_ns_;
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
      value_.provenance = "authoritative_producer_rad_s;" +
                          sample.producer_omega_provenance +
                          ";conditional_model_uncertainty";
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
  // IDs identify events, not their chronological order. Historical evidence may
  // arrive after a newer prediction has already received a smaller ID.
  bool contact(std::uint64_t id, std::int64_t time_ns, const Vec3 &pre_velocity,
               const BallPhysics &p) {
    if (!id ||
        std::find(contact_ids_.begin(), contact_ids_.end(), id) !=
            contact_ids_.end() ||
        !can_apply_contact(time_ns))
      return false;
    remember_contact(id, time_ns);
    // A fresh measurement already includes older physical impulses.
    if (value_.usable() && time_ns < value_.measurement_time_ns)
      return true;
    if (value_.usable() && value_.measurement_time_ns <= time_ns) {
      ModelState s;
      s.velocity = pre_velocity;
      s.spin = value_;
      apply_model_contact(s, p);
      value_ = s.spin;
    }
    value_.last_contact_id = id;
    last_contact_time_ns_ = time_ns;
    estimator_.reset();
    return true;
  }

private:
  FlightSpinEstimator estimator_;
  double coherence_;
  EffectiveSpin value_;
  std::optional<double> last_time_s_;
  // Bounded by the owning shot/batch event ledger.
  std::vector<std::uint64_t> contact_ids_;
  std::optional<std::int64_t> last_contact_time_ns_;
};
} // namespace hope_planner_cpp
