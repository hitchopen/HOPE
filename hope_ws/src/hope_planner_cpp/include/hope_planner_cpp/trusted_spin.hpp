#pragma once
#include "hope_planner_cpp/common_ball_physics.hpp"
#include "hope_planner_cpp/flight_spin_estimator.hpp"
#include <algorithm>
#include <deque>
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
    history_.clear();
    baseline_ = {};
    baseline_time_ns_.reset();
    contact_checkpoints_.clear();
  }
  const EffectiveSpin &value() const { return value_; }
  void restore(const EffectiveSpin &s) {
    reset();
    value_ = s;
    baseline_ = s;
    baseline_time_ns_ = s.measurement_time_ns;
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
    return predates_retained_source(ns) || !last_contact_time_ns_ || ns >= *last_contact_time_ns_;
  }
  EffectiveSpin at(std::int64_t ns) const {
    EffectiveSpin result;
    auto latest = std::numeric_limits<std::int64_t>::min();
    if (baseline_time_ns_ && *baseline_time_ns_ <= ns) {
      result = baseline_;
      latest = *baseline_time_ns_;
    }
    for (const auto &entry : history_) {
      if (entry.sample.source_time_ns > ns) break;
      result = entry.value;
      latest = entry.sample.source_time_ns;
    }
    for (const auto &entry : contact_checkpoints_)
      if (entry.first <= ns && entry.first >= latest) {
        result = entry.second;
        latest = entry.first;
      }
    return result;
  }
  EffectiveSpin observe(const BallSample &sample) {
    if (last_time_s_ && sample.source_time_s <= *last_time_s_)
      return value_;
    last_time_s_ = sample.source_time_s;
    observe_value(sample);
    history_.push_back({sample, value_});
    // Bounded causal suffix, copied with the owning shot checkpoint. A
    // contact older than this suffix cannot recover discarded measurements.
    while (history_.size() > 192) {
      baseline_ = history_.front().value;
      baseline_time_ns_ = history_.front().sample.source_time_ns;
      history_.pop_front();
    }
    return value_;
  }
private:
  void observe_value(const BallSample &sample) {
    bool measured = false;
    if (sample.producer_omega_valid &&
        sample.producer_omega_rad_s.allFinite()) {
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
          s.coherence >= coherence_ && s.omega_rad_s.allFinite()) {
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
  }
public:
  // IDs identify events, not their chronological order. Historical evidence may
  // arrive after a newer prediction has already received a smaller ID.
  bool contact(std::uint64_t id, std::int64_t time_ns, const Vec3 &pre_velocity,
               const BallPhysics &p, std::int64_t bracket_end_ns = 0) {
    if (!id ||
        std::find(contact_ids_.begin(), contact_ids_.end(), id) !=
            contact_ids_.end() ||
        !can_apply_contact(time_ns))
      return false;
    // Verified prehistory before all retained measurements does not change the
    // already-post-contact signal. There is no pre-impact spin to reconstruct.
    if (predates_retained_source(time_ns)) {
      contact_ids_.push_back(id);
      return true;
    }
    value_ = at(time_ns);
    remember_contact(id, time_ns);
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
    // Refit only the bounded post-contact suffix. No quaternion increment
    // touching the uncertain contact bracket can be declared measured spin.
    const auto post_contact = value_;
    contact_checkpoints_.push_back({time_ns, post_contact});
    if (contact_checkpoints_.size() > 128) contact_checkpoints_.pop_front();
    for (auto &entry : history_) {
      if (entry.sample.source_time_ns <= time_ns) continue;
      if (entry.sample.source_time_ns >= std::max(time_ns, bracket_end_ns))
        observe_value(entry.sample);
      entry.value = value_;
    }
    return true;
  }

private:
  bool predates_retained_source(std::int64_t ns) const {
    return !history_.empty() && ns < history_.front().sample.source_time_ns &&
           (!baseline_time_ns_ || ns < *baseline_time_ns_);
  }
  FlightSpinEstimator estimator_;
  double coherence_;
  EffectiveSpin value_;
  std::optional<double> last_time_s_;
  // Bounded by the owning shot/batch event ledger.
  std::vector<std::uint64_t> contact_ids_;
  std::optional<std::int64_t> last_contact_time_ns_;
  struct Entry { BallSample sample; EffectiveSpin value; };
  std::deque<Entry> history_;
  EffectiveSpin baseline_;
  std::optional<std::int64_t> baseline_time_ns_;
  std::deque<std::pair<std::int64_t, EffectiveSpin>> contact_checkpoints_;
};
} // namespace hope_planner_cpp
