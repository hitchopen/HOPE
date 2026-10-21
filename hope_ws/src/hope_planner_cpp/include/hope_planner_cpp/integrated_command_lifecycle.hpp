#pragma once
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/validated_return_planner.hpp"
#include <stdexcept>

namespace hope_planner_cpp {
// Schema 2 has no boot nonce. A fresh instance requires a fresh receiver.
// Source epochs must already be mapped to Unix ns before invoking this class.
class IntegratedCommandLifecycle {
public:
  static constexpr std::uint64_t max_wire_integer = (1ULL << 52);
  explicit IntegratedCommandLifecycle(std::uint64_t sequence = 0) : sequence_(sequence) {}
  std::optional<WireIdentity> admit(const ContactPlanningInput &input,
      const ValidatedReturn &result, std::uint64_t current_generation,
      std::int64_t now_ns, double min_lead_s = .08) {
    if (!result.command.valid || !result.solve_ok || !result.impact_ok || !result.return_ok ||
        input.token.generation != current_generation || input.token.generation < generation_ ||
        now_ns > input.expires_ns || static_cast<long double>(now_ns) <
            static_cast<long double>(input.source.source_time_ns)-1000000.L ||
        (static_cast<long double>(input.arrival_ns)-now_ns)*1e-9L < min_lead_s ||
        !std::isfinite(min_lead_s) || min_lead_s < 0)
      return {};
    if (input.token.generation == generation_ &&
        (committed_ || input.source.source_time_ns <= last_source_)) return {};
    if (input.token.generation != generation_) {
      if (flight_ == max_wire_integer) throw std::overflow_error("wire flight exhausted: coordinated restart required");
      generation_=input.token.generation; ++flight_; revision_=0; committed_=false;
      swing_sign_=0;
    }
    if (sequence_ >= max_wire_integer || revision_ >= max_wire_integer)
      throw std::overflow_error("wire sequence exhausted: coordinated restart required");
    last_source_=input.source.source_time_ns;
    last_arrival_=input.arrival_ns;
    ++revision_;
    // Conservative producer freeze. Receiver owns actual swing commitment;
    // producer never claims invalidation can cancel an executing swing.
    committed_=(static_cast<long double>(input.arrival_ns)-now_ns)*1e-9L <= .12;
    return WireIdentity{++sequence_,flight_,revision_};
  }
  WireIdentity invalidation() {
    if (sequence_ >= max_wire_integer) throw std::overflow_error("wire sequence exhausted");
    return {++sequence_,flight_,revision_};
  }
  bool committed() const { return committed_; }
  bool refresh_idle(std::int64_t now_ns) const {
    return !committed_ || static_cast<long double>(now_ns)-last_arrival_ > 1000000000.L;
  }
  double latch_swing(double sign) { if (swing_sign_==0) swing_sign_=sign; return swing_sign_; }
private:
  std::uint64_t sequence_=0,flight_=0,revision_=0,generation_=0;
  std::int64_t last_source_=std::numeric_limits<std::int64_t>::min();
  std::int64_t last_arrival_=0;
  bool committed_=false;
  double swing_sign_=0;
};
} // namespace hope_planner_cpp
