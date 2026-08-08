#pragma once

namespace a3_pingpong {

struct RuntimeHandoffReset {
  // Bounded contracts train an executed-q_des feedback observation and seed it
  // from the measured posture on the first policy tick.  V11 affine instead
  // trains ordinary previous-action history and its static handoff explicitly
  // resets that history to affine action zero.
  bool seed_measured_qdes_feedback = false;
  // A deterministic serve ends at the exact policy default.  Its first policy
  // tick must therefore enter the static hold at that same command, rather
  // than recapturing an imperfect measured posture as a new q_des.
  bool force_exact_default_static = true;
};

inline RuntimeHandoffReset runtime_handoff_reset(
    bool has_bounded_qdes_contract) {
  RuntimeHandoffReset out;
  out.seed_measured_qdes_feedback = has_bounded_qdes_contract;
  return out;
}

// Pre-first-swing static handoff has two intentionally different safety regimes. A true cold
// boot has never released active station tracking, so there is no walk momentum to prove through
// localization. After any active pending station, an abort must prove base settle before the
// stiff official stand can take over.
inline bool prefirst_static_allowed(bool planner_have_hold, bool near_station,
                                    bool active_station_tracking_started,
                                    bool base_settled, bool upright_still,
                                    bool recovery_timeout_elapsed) {
  if (planner_have_hold || !near_station) return false;
  if (!active_station_tracking_started) return upright_still;
  return base_settled && (upright_still || recovery_timeout_elapsed);
}

}  // namespace a3_pingpong
