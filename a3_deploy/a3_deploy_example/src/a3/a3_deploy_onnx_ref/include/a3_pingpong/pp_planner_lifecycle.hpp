#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "a3_deploy/numeric_safety.hpp"

namespace a3_pingpong {

inline constexpr char kPlannerEngageLogPrefix[] = "[pp engage]";
inline constexpr char kPlannerCompletionLogPrefix[] = "[pp completion]";
inline constexpr std::array<double, 4> kPlannerRapidHomeStationRange = {
    0.0, 0.0, -0.06, 0.06};
inline constexpr double kPlannerRapidCommandStepMaxM = 0.06;
inline constexpr double kPlannerRapidSupportNumericTolerance = 1.0e-9;
inline constexpr double kPlannerLifecycleTimeToleranceS = 1.0e-6;

enum class PendingStationGapDecision {
  kNoPending,
  kHoldBlocked,
  kExpire,
};

enum class PlannerActorEntryTtsDecision {
  kWaitAboveSupport,
  kCommitInSupport,
  kRejectBelowSupport,
};

// Schema 22 keeps the exported actor-entry interval as distribution telemetry,
// not as a second ball-admission clock. A positive finite cold command can
// therefore enter below the training interval; an expired/non-finite command
// remains fail-closed.
enum class PlannerSchema22ColdTtsDecision {
  kWaitAboveSupport,
  kCommitInSupport,
  kCommitBelowSupportTelemetry,
  kRejectExpiredOrInvalid,
};

enum class PlannerSchema22ColdReceiptDecision {
  kAcceptFreshReceipt,
  kAcceptRetainedFlight,
  kRejectInvalidAge,
  kRejectStaleInitialReceipt,
  kRejectInvalidInitialReceipt,
};

enum class PlannerRapidPreemptDecision {
  kDisabled,
  kInvalidCandidate,
  kSameFlight,
  kWaitProtectedFollowthrough,
  kWaitBeforeCommitDelaySupport,
  kExpiredAfterCommitDelaySupport,
  kWaitAboveSupport,
  kExpiredBelowSupport,
  kCommit,
};

enum class PlannerRapidCandidateIdentityDecision {
  kInvalid,
  kSameFrozenFlight,
  kRejectConsumedFlight,
  kRejectExpiredFlight,
  kRejectNonmonotonicFlight,
  kLatchFreshFlight,
  kKeepPendingRevision,
  kUpdatePendingRevision,
  kBusyDropDifferentFlight,
};

struct PlannerRapidStaticCommandSupport {
  bool ok = false;
  bool absolute_station_ok = false;
  bool command_step_ok = false;
  bool z_ok = false;
  bool speed_ok = false;
  bool velocity_ok = false;
  double command_step_m = 0.0;
};

enum class PlannerSchema22CoreBoxCoverageDisposition {
  kCovered,
  kTelemetryOnly,
};

// The exported core/planner boxes remain useful for measuring train/real
// distribution coverage. They are not the full training distribution because
// empirical joint-tail samples can legitimately live outside these boxes.
// Wire/schema/identity/finiteness checks are owned independently by the Runner;
// a miss in this core-box coverage is telemetry and must not swallow a valid
// physical flight.
inline PlannerSchema22CoreBoxCoverageDisposition
planner_schema22_core_box_coverage_disposition(
    const PlannerRapidStaticCommandSupport& support) noexcept {
  return support.ok
      ? PlannerSchema22CoreBoxCoverageDisposition::kCovered
      : PlannerSchema22CoreBoxCoverageDisposition::kTelemetryOnly;
}

inline std::array<double, 2> planner_schema22_project_station_to_home(
    double requested_x_w, double requested_y_w, bool session_home_set,
    double session_home_x_w, double session_home_y_w) noexcept {
  if (!session_home_set ||
      !a3_deploy::numeric_safety::IsFinite(requested_x_w) ||
      !a3_deploy::numeric_safety::IsFinite(requested_y_w) ||
      !a3_deploy::numeric_safety::IsFinite(session_home_x_w) ||
      !a3_deploy::numeric_safety::IsFinite(session_home_y_w)) {
    return {requested_x_w, requested_y_w};
  }
  return {
      session_home_x_w,
      std::clamp(
          requested_y_w,
          session_home_y_w + kPlannerRapidHomeStationRange[2],
          session_home_y_w + kPlannerRapidHomeStationRange[3])};
}

struct PlannerFlightIdentity {
  std::uint64_t producer_epoch = 0;
  std::uint64_t flight_id = 0;
};

inline bool planner_flight_identity_valid(
    const PlannerFlightIdentity& identity) noexcept {
  return identity.producer_epoch > 0 && identity.flight_id > 0;
}

inline bool planner_same_flight_identity(
    const PlannerFlightIdentity& lhs,
    const PlannerFlightIdentity& rhs) noexcept {
  return planner_flight_identity_valid(lhs) &&
      planner_flight_identity_valid(rhs) &&
      lhs.producer_epoch == rhs.producer_epoch &&
      lhs.flight_id == rhs.flight_id;
}

// x86 MuJoCo Gate3 capability probe: admit every new, positive-time physical
// flight while a swing is active.  This deliberately ignores the trained TTS,
// protected-followthrough and old-contact-delay windows so the plant sees the
// policy response instead of a Runner rejection.  Wire/identity/finiteness and
// a future strike deadline remain mandatory; this is not a production mode.
inline PlannerRapidPreemptDecision planner_gate3_force_every_flight_decision(
    bool enabled, bool planner_engaged, int candidate_schema,
    const PlannerFlightIdentity& candidate_identity,
    const PlannerFlightIdentity& frozen_identity,
    double candidate_tts_s) noexcept {
  if (!enabled) return PlannerRapidPreemptDecision::kDisabled;
  if (!planner_engaged || candidate_schema != 2 ||
      !planner_flight_identity_valid(candidate_identity) ||
      !planner_flight_identity_valid(frozen_identity) ||
      !a3_deploy::numeric_safety::IsFinite(candidate_tts_s) ||
      candidate_tts_s <= 0.0) {
    return PlannerRapidPreemptDecision::kInvalidCandidate;
  }
  if (planner_same_flight_identity(candidate_identity, frozen_identity)) {
    return PlannerRapidPreemptDecision::kSameFlight;
  }
  return PlannerRapidPreemptDecision::kCommit;
}

enum class PlannerFlightCompletionKind {
  kNone,
  kRapidPreempt,
  kNativeEnd,
};

inline const char* planner_flight_completion_kind_name(
    PlannerFlightCompletionKind kind) noexcept {
  switch (kind) {
    case PlannerFlightCompletionKind::kRapidPreempt:
      return "preempt";
    case PlannerFlightCompletionKind::kNativeEnd:
      return "native";
    case PlannerFlightCompletionKind::kNone:
    default:
      return "none";
  }
}

// Latched authoritative completion event for schema 22. The event remains in
// the per-tick trace until the next completion; consumers detect the one-shot
// edge by a strictly increasing sequence. Producer epochs permit a restarted
// schema-2 source to begin again at flight 1 without aliasing the prior source.
struct PlannerFlightCompletionState {
  std::uint64_t sequence = 0;
  PlannerFlightIdentity completed_identity{};
  PlannerFlightCompletionKind kind = PlannerFlightCompletionKind::kNone;
};

inline bool planner_record_flight_completion_once(
    bool contract_enabled, const PlannerFlightIdentity& identity,
    PlannerFlightCompletionKind kind,
    PlannerFlightCompletionState& state) noexcept {
  if (!contract_enabled || !planner_flight_identity_valid(identity) ||
      kind == PlannerFlightCompletionKind::kNone) {
    return false;
  }
  if (planner_flight_identity_valid(state.completed_identity) &&
      (identity.producer_epoch < state.completed_identity.producer_epoch ||
       (identity.producer_epoch == state.completed_identity.producer_epoch &&
        identity.flight_id <= state.completed_identity.flight_id))) {
    return false;
  }
  ++state.sequence;
  state.completed_identity = identity;
  state.kind = kind;
  return true;
}

enum class PlannerProducerEpochDecision {
  kInvalid,
  kInitialize,
  kSame,
  kRejectOlder,
  kResetIdle,
  kResetActiveBaseline,
};

inline PlannerProducerEpochDecision planner_producer_epoch_decision(
    std::uint64_t observed_epoch, std::uint64_t retained_epoch,
    bool active_swing) noexcept {
  if (observed_epoch == 0) return PlannerProducerEpochDecision::kInvalid;
  if (retained_epoch == 0) return PlannerProducerEpochDecision::kInitialize;
  if (observed_epoch == retained_epoch)
    return PlannerProducerEpochDecision::kSame;
  if (observed_epoch < retained_epoch)
    return PlannerProducerEpochDecision::kRejectOlder;
  return active_swing
      ? PlannerProducerEpochDecision::kResetActiveBaseline
      : PlannerProducerEpochDecision::kResetIdle;
}

// A Planner shot identity is a stronger clock edge than level/direction. A completed shot can
// leave the cached level at one until late in ComputeCommand; the next same-side flight then
// engages as a 1->1 transition. Resetting on the monotonic shot sequence prevents that new flight
// from inheriting the previous shot's already-expired clock origin.
inline bool planner_swing_clock_needs_reset(
    int current_level, int previous_level, int current_direction,
    int previous_direction, std::uint64_t shot_sequence_before_engage,
    std::uint64_t shot_sequence_after_engage) noexcept {
  return shot_sequence_after_engage != shot_sequence_before_engage ||
         (current_level == 1 && previous_level != 1) ||
         current_direction != previous_direction;
}

// Production applies this exact transition before ScriptedTarget() reads the origin. Returning
// the retained origin for a same-shot 1->1 continuation makes that ordering explicit and lets the
// hardware complete->engage trace be tested without duplicating the assignment in the test.
inline std::uint64_t planner_swing_clock_origin_after_engage(
    std::uint64_t tick, std::uint64_t retained_origin, int current_level,
    int previous_level, int current_direction, int previous_direction,
    std::uint64_t shot_sequence_before_engage,
    std::uint64_t shot_sequence_after_engage) noexcept {
  return planner_swing_clock_needs_reset(
             current_level, previous_level, current_direction,
             previous_direction, shot_sequence_before_engage,
             shot_sequence_after_engage)
      ? tick
      : retained_origin;
}

inline double planner_swing_elapsed_s(
    std::uint64_t tick, std::uint64_t origin, double policy_dt_s,
    double swing_speed) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(policy_dt_s) || policy_dt_s <= 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(swing_speed) || swing_speed < 0.0) {
    return 0.0;
  }
  return static_cast<double>(tick >= origin ? tick - origin : 0) *
         policy_dt_s * swing_speed;
}

inline double planner_running_tts(
    double initial_tts_s, std::uint64_t tick, std::uint64_t origin,
    double policy_dt_s, double swing_speed) noexcept {
  return initial_tts_s -
         planner_swing_elapsed_s(tick, origin, policy_dt_s, swing_speed);
}

// v1 exposed a constant WAIT sentinel. v2 reuses that existing scalar as a
// visible, unsaturated elapsed clock, exactly matching training's
//   wait_tts_s - wait_age_s
// without adding an observation dimension.
inline double planner_wait_tts(double initial_tts_s, bool visible_elapsed,
                               std::uint64_t tick, std::uint64_t origin,
                               double policy_dt_s) noexcept {
  if (!visible_elapsed) return initial_tts_s;
  return planner_running_tts(initial_tts_s, tick, origin, policy_dt_s, 1.0);
}

inline bool planner_followthrough_complete(
    double running_tts_s, double post_contact_followthrough_s) noexcept {
  return a3_deploy::numeric_safety::IsFinite(running_tts_s) &&
         a3_deploy::numeric_safety::IsFinite(post_contact_followthrough_s) &&
         post_contact_followthrough_s >= 0.0 &&
         running_tts_s <=
             -post_contact_followthrough_s + kPlannerLifecycleTimeToleranceS;
}

// Build2-minimal rootfix has two intentionally separate contact-tail edges:
//
//   * contact+0.12 s changes only the actor-visible base target to the immutable
//     session HOME; and
//   * the native clip end remains the sole level-1 completion/resample edge.
//
// Keeping this decision separate from planner_followthrough_complete() prevents
// the HOME publication from being accidentally routed through the legacy HOME
// preemption path, which would cut the native q/qdot reference tail short.
struct PlannerNativeTailHomeDecision {
  bool latch_recovery = false;
  bool publish_home_base_target = false;
};

inline PlannerNativeTailHomeDecision planner_native_tail_home_step(
    bool contract_enabled, bool planner_mode, int swing_level,
    bool planner_engaged, bool session_home_set, double running_tts_s,
    double post_contact_home_s, bool recovery_already_latched) noexcept {
  const bool active_native_tail =
      contract_enabled && planner_mode && swing_level == 1 && session_home_set;
  const bool crosses_home_edge =
      active_native_tail && planner_engaged && !recovery_already_latched &&
      planner_followthrough_complete(running_tts_s, post_contact_home_s);
  return PlannerNativeTailHomeDecision{
      crosses_home_edge,
      active_native_tail && (recovery_already_latched || crosses_home_edge)};
}

inline bool planner_native_clip_end_complete(double reference_tts_s,
                                             double native_min_tts_s) noexcept {
  return a3_deploy::numeric_safety::IsFinite(reference_tts_s) &&
         a3_deploy::numeric_safety::IsFinite(native_min_tts_s) &&
         reference_tts_s < native_min_tts_s;
}

inline void planner_sync_completion_cache(int& previous_level) noexcept {
  previous_level = 0;
}

// During an actor-visible localization outage, preserve the last verified world-station anchor.
// Falling back to the held base coordinate is reserved for cold start (or a corrupt anchor); doing
// it on every stale tick silently changes the 110-D station error to zero.
inline double planner_stale_station_coordinate(
    bool hold_station_set, double hold_station_coordinate,
    double held_base_coordinate) noexcept {
  return hold_station_set &&
             a3_deploy::numeric_safety::IsFinite(hold_station_coordinate)
      ? hold_station_coordinate
      : held_base_coordinate;
}

// Deterministic target-snapshot clock for a clip with a near-static prefix.
// This is scheduling, not an admission test: a valid latest-value command is
// sampled once the requested prefix duration has elapsed. The clamp keeps the
// clock before the measured dynamic onset.
inline double planner_prefix_commit_tts(double windup_s,
                                        double requested_skip_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(windup_s) || windup_s <= 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(requested_skip_s)) {
    return 0.0;
  }
  const double skip = std::clamp(requested_skip_s, 0.0, 0.45 * windup_s);
  return windup_s - skip;
}

// Last contract-supported start, corresponding to the deepest 45% prefix
// skip. It is deliberately independent of the selected commit delay so a
// 20/50 Hz scheduling tick or one planner revision cannot make the nominal
// commit instant unreachable.
inline double planner_prefix_hard_late_tts(double windup_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(windup_s) || windup_s <= 0.0)
    return 0.0;
  return 0.55 * windup_s;
}

// Training's entry_prefix_phase_cap is the maximum fraction of the pre-strike reference prefix
// that may be skipped at COMMIT.  Convert that exact metadata semantic back to remaining TTS.
// cap=1 is a deadline-aligned genuine late reveal; cap=0 keeps the reference at windup.
inline double planner_late_reveal_reference_floor_tts(
    double windup_s, double prefix_phase_cap) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(windup_s) || windup_s <= 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(prefix_phase_cap)) {
    return 0.0;
  }
  const double cap = std::clamp(prefix_phase_cap, 0.0, 1.0);
  return (1.0 - cap) * windup_s;
}

// model_21800 samples the latest pending target exactly at the deepest
// near-static/dynamic boundary. Other recipes retain their requested prefix
// schedule. This chooses a sample time only; it does not reject a revision.
inline double planner_target_sample_tts(bool fixed_dynamic_boundary,
                                        double windup_s,
                                        double requested_skip_s) noexcept {
  return fixed_dynamic_boundary
      ? planner_prefix_hard_late_tts(windup_s)
      : planner_prefix_commit_tts(windup_s, requested_skip_s);
}

// A late-reveal policy must see the final command no later than the upper edge of the actor-entry
// support it was trained on. This is a sampling time, not a READY/admission gate: the mailbox keeps
// the latest valid same-shot prediction until this deadline, then commits it atomically.
inline double planner_actor_entry_commit_tts(double support_lo_s,
                                             double support_hi_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(support_lo_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_hi_s) ||
      support_lo_s <= 0.0 || support_hi_s < support_lo_s) {
    return 0.0;
  }
  return support_hi_s;
}

// The v2 external event owns COMMIT, but it may only instantiate actor states
// from that side's trained TTS support. Above the upper edge the latest Planner
// revision remains hidden; inside the closed interval it commits atomically;
// below the lower edge the shot is already too late and is rejected.
inline PlannerActorEntryTtsDecision planner_actor_entry_tts_decision(
    double time_to_strike_s, double support_lo_s,
    double support_hi_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(time_to_strike_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_lo_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_hi_s) ||
      support_lo_s <= 0.0 || support_hi_s < support_lo_s ||
      time_to_strike_s < support_lo_s) {
    return PlannerActorEntryTtsDecision::kRejectBelowSupport;
  }
  return time_to_strike_s > support_hi_s
      ? PlannerActorEntryTtsDecision::kWaitAboveSupport
      : PlannerActorEntryTtsDecision::kCommitInSupport;
}

inline PlannerSchema22ColdTtsDecision planner_schema22_cold_tts_decision(
    double time_to_strike_s, double support_lo_s,
    double support_hi_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(time_to_strike_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_lo_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_hi_s) ||
      support_lo_s <= 0.0 || support_hi_s < support_lo_s ||
      time_to_strike_s <= 0.0) {
    return PlannerSchema22ColdTtsDecision::kRejectExpiredOrInvalid;
  }
  if (time_to_strike_s > support_hi_s) {
    return PlannerSchema22ColdTtsDecision::kWaitAboveSupport;
  }
  return time_to_strike_s < support_lo_s
      ? PlannerSchema22ColdTtsDecision::kCommitBelowSupportTelemetry
      : PlannerSchema22ColdTtsDecision::kCommitInSupport;
}

// A fresh physical flight must enter through a finite, timely mailbox receipt.
// Once that identity and its future positive deadline have been verified by the
// caller, waiting for the actor-entry TTS is scheduling: mailbox age and a later
// invalid revision cannot become a second admission clock for the retained
// last-valid command.
inline PlannerSchema22ColdReceiptDecision
planner_schema22_cold_receipt_decision(
    bool same_verified_retained_flight, double valid_age_s,
    double command_timeout_s, bool invalid_after) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(valid_age_s) ||
      valid_age_s < 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(command_timeout_s) ||
      command_timeout_s < 0.0) {
    return PlannerSchema22ColdReceiptDecision::kRejectInvalidAge;
  }
  if (same_verified_retained_flight) {
    return PlannerSchema22ColdReceiptDecision::kAcceptRetainedFlight;
  }
  if (invalid_after) {
    return PlannerSchema22ColdReceiptDecision::kRejectInvalidInitialReceipt;
  }
  return valid_age_s > command_timeout_s
      ? PlannerSchema22ColdReceiptDecision::kRejectStaleInitialReceipt
      : PlannerSchema22ColdReceiptDecision::kAcceptFreshReceipt;
}

// Schema 22's external COMMIT is policy-independent. Classify only immutable command/session
// core-box coverage here: neither measured base pose nor READY/settle/action/outcome is input.
// `command_anchor` is the immutable session HOME published by the task lifecycle.
inline PlannerRapidStaticCommandSupport planner_rapid_static_command_support(
    int clip, int frame_code, double target_z_w, double target_speed_w,
    bool velocity_in_support, double station_x_w, double station_y_w,
    bool session_home_set, double session_home_x_w, double session_home_y_w,
    const std::array<double, 4>& absolute_station_range,
    double absolute_station_margin, double command_anchor_x_w,
    double command_anchor_y_w, double command_step_max_m, double z_lo_w,
    double z_hi_w, double z_margin_m, double speed_max_mps) noexcept {
  PlannerRapidStaticCommandSupport out;
  if (clip < 0 || clip >= 2 || frame_code != 0 || !session_home_set ||
      !a3_deploy::numeric_safety::IsFinite(target_z_w) ||
      !a3_deploy::numeric_safety::IsFinite(target_speed_w) ||
      !a3_deploy::numeric_safety::IsFinite(station_x_w) ||
      !a3_deploy::numeric_safety::IsFinite(station_y_w) ||
      !a3_deploy::numeric_safety::IsFinite(session_home_x_w) ||
      !a3_deploy::numeric_safety::IsFinite(session_home_y_w) ||
      !a3_deploy::numeric_safety::IsFinite(command_anchor_x_w) ||
      !a3_deploy::numeric_safety::IsFinite(command_anchor_y_w) ||
      !a3_deploy::numeric_safety::IsFinite(absolute_station_margin) ||
      absolute_station_margin < 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(command_step_max_m) ||
      command_step_max_m < 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(z_lo_w) ||
      !a3_deploy::numeric_safety::IsFinite(z_hi_w) || z_hi_w < z_lo_w ||
      !a3_deploy::numeric_safety::IsFinite(z_margin_m) || z_margin_m < 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(speed_max_mps) ||
      speed_max_mps < 0.0) {
    return out;
  }
  for (double bound : absolute_station_range) {
    if (!a3_deploy::numeric_safety::IsFinite(bound)) return out;
  }
  if (absolute_station_range[1] < absolute_station_range[0] ||
      absolute_station_range[3] < absolute_station_range[2]) {
    return out;
  }

  const double home_dx = station_x_w - session_home_x_w;
  const double home_dy = station_y_w - session_home_y_w;
  const double command_dx = station_x_w - command_anchor_x_w;
  const double command_dy = station_y_w - command_anchor_y_w;
  out.command_step_m = std::hypot(command_dx, command_dy);
  out.absolute_station_ok =
      home_dx >= absolute_station_range[0] - absolute_station_margin &&
      home_dx <= absolute_station_range[1] + absolute_station_margin &&
      home_dy >= absolute_station_range[2] - absolute_station_margin &&
      home_dy <= absolute_station_range[3] + absolute_station_margin;
  out.command_step_ok =
      a3_deploy::numeric_safety::IsFinite(out.command_step_m) &&
      out.command_step_m <= command_step_max_m;
  out.z_ok = target_z_w >= z_lo_w - z_margin_m &&
      target_z_w <= z_hi_w + z_margin_m;
  out.speed_ok = target_speed_w <= speed_max_mps;
  out.velocity_ok = velocity_in_support;
  out.ok = out.absolute_station_ok && out.command_step_ok && out.z_ok &&
      out.speed_ok && out.velocity_ok;
  return out;
}

// Exact schema-22 exported core-box coverage classifier. Runtime CLI margins are
// intentionally absent from this signature: HOME x is fixed, HOME-relative y and command
// step are both limited to 6 cm, frame must be world, and only the artifact's per-clip
// core/planner z/velocity boxes are supplied by the caller. Empirical joint-tail training
// may extend beyond them. The Runner reports this result as distribution telemetry; wire
// validity, identity, finiteness and positive deadlines remain separate fail-closed checks.
// The named epsilon is numerical comparison tolerance, not a tunable margin.
inline PlannerRapidStaticCommandSupport
planner_schema22_exact_command_support(
    int clip, int frame_code, double target_z_w, double target_speed_w,
    bool velocity_in_support, double station_x_w, double station_y_w,
    bool session_home_set, double session_home_x_w, double session_home_y_w,
    double z_lo_w, double z_hi_w, double speed_max_mps) noexcept {
  return planner_rapid_static_command_support(
      clip, frame_code, target_z_w, target_speed_w, velocity_in_support,
      station_x_w, station_y_w, session_home_set, session_home_x_w,
      session_home_y_w, kPlannerRapidHomeStationRange,
      kPlannerRapidSupportNumericTolerance, session_home_x_w,
      session_home_y_w,
      kPlannerRapidCommandStepMaxM + kPlannerRapidSupportNumericTolerance,
      z_lo_w, z_hi_w, kPlannerRapidSupportNumericTolerance,
      speed_max_mps + kPlannerRapidSupportNumericTolerance);
}

// Schema 22 admits a new physical flight while level 1 remains active. Its temporal admission
// is deliberately stricter than idle engage: an explicit schema-2 flight identity is mandatory, a
// revision of the currently frozen flight is never a next-shot edge, and both the next-shot
// TTS and old-contact commit-delay supports are closed temporal decisions rather than soft
// telemetry.
inline PlannerRapidPreemptDecision planner_rapid_preempt_decision(
    bool contract_enabled, bool planner_engaged, int candidate_schema,
    const PlannerFlightIdentity& candidate_identity,
    const PlannerFlightIdentity& frozen_identity,
    double old_signed_tts_s, double no_preempt_s, double candidate_tts_s,
    double support_lo_s, double support_hi_s, double commit_delay_lo_s,
    double commit_delay_hi_s) noexcept {
  if (!contract_enabled) return PlannerRapidPreemptDecision::kDisabled;
  if (!planner_engaged || candidate_schema != 2 ||
      !planner_flight_identity_valid(candidate_identity) ||
      !planner_flight_identity_valid(frozen_identity) ||
      !a3_deploy::numeric_safety::IsFinite(old_signed_tts_s) ||
      !a3_deploy::numeric_safety::IsFinite(no_preempt_s) ||
      no_preempt_s < 0.0 ||
      !a3_deploy::numeric_safety::IsFinite(candidate_tts_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_lo_s) ||
      !a3_deploy::numeric_safety::IsFinite(support_hi_s) ||
      support_lo_s <= 0.0 || support_hi_s < support_lo_s ||
      !a3_deploy::numeric_safety::IsFinite(commit_delay_lo_s) ||
      !a3_deploy::numeric_safety::IsFinite(commit_delay_hi_s) ||
      commit_delay_lo_s < no_preempt_s ||
      commit_delay_hi_s < commit_delay_lo_s) {
    return PlannerRapidPreemptDecision::kInvalidCandidate;
  }
  if (planner_same_flight_identity(candidate_identity, frozen_identity))
    return PlannerRapidPreemptDecision::kSameFlight;
  if (candidate_tts_s < support_lo_s - kPlannerLifecycleTimeToleranceS)
    return PlannerRapidPreemptDecision::kExpiredBelowSupport;
  if (!planner_followthrough_complete(old_signed_tts_s, no_preempt_s))
    return PlannerRapidPreemptDecision::kWaitProtectedFollowthrough;
  const double elapsed_since_old_contact_s = -old_signed_tts_s;
  if (elapsed_since_old_contact_s <
      commit_delay_lo_s - kPlannerLifecycleTimeToleranceS)
    return PlannerRapidPreemptDecision::kWaitBeforeCommitDelaySupport;
  if (elapsed_since_old_contact_s >
      commit_delay_hi_s + kPlannerLifecycleTimeToleranceS)
    return PlannerRapidPreemptDecision::kExpiredAfterCommitDelaySupport;
  if (candidate_tts_s > support_hi_s + kPlannerLifecycleTimeToleranceS)
    return PlannerRapidPreemptDecision::kWaitAboveSupport;
  return PlannerRapidPreemptDecision::kCommit;
}

// The schema-2 Planner assigns monotonically increasing flight identities within one runtime
// epoch. Once a next flight is latched, a different later mailbox flight cannot silently replace
// it: doing so would couple target/side from one ball to the clock sampled for another. Same-flight
// revisions update only when both revision and command sequences move forward.
inline PlannerRapidCandidateIdentityDecision
planner_rapid_candidate_identity_decision(
    const PlannerFlightIdentity& candidate_identity,
    std::uint64_t candidate_revision_id,
    std::uint64_t candidate_command_seq,
    const PlannerFlightIdentity& frozen_identity,
    const PlannerFlightIdentity& consumed_identity,
    const PlannerFlightIdentity& expired_high_watermark,
    bool pending_active, const PlannerFlightIdentity& pending_identity,
    std::uint64_t pending_revision_id,
    std::uint64_t pending_command_seq) noexcept {
  if (!planner_flight_identity_valid(candidate_identity) ||
      candidate_revision_id == 0 || candidate_command_seq == 0 ||
      !planner_flight_identity_valid(frozen_identity) ||
      !planner_flight_identity_valid(consumed_identity)) {
    return PlannerRapidCandidateIdentityDecision::kInvalid;
  }
  if (planner_same_flight_identity(candidate_identity, frozen_identity)) {
    return PlannerRapidCandidateIdentityDecision::kSameFrozenFlight;
  }
  if (planner_same_flight_identity(candidate_identity, consumed_identity))
    return PlannerRapidCandidateIdentityDecision::kRejectConsumedFlight;
  // `expired_flight_id` is a monotonic high-watermark, not a one-element cache.
  // Remembering only the last exact ID would permit an older already-expired
  // flight after a later flight also expired.
  if (planner_flight_identity_valid(expired_high_watermark) &&
      candidate_identity.producer_epoch ==
          expired_high_watermark.producer_epoch &&
      candidate_identity.flight_id <= expired_high_watermark.flight_id)
    return PlannerRapidCandidateIdentityDecision::kRejectExpiredFlight;
  const std::uint64_t newest_epoch = std::max(
      frozen_identity.producer_epoch, consumed_identity.producer_epoch);
  if (candidate_identity.producer_epoch < newest_epoch ||
      (candidate_identity.producer_epoch == frozen_identity.producer_epoch &&
       candidate_identity.flight_id <= frozen_identity.flight_id) ||
      (candidate_identity.producer_epoch == consumed_identity.producer_epoch &&
       candidate_identity.flight_id <= consumed_identity.flight_id)) {
    return PlannerRapidCandidateIdentityDecision::kRejectNonmonotonicFlight;
  }
  if (pending_active) {
    if (!planner_same_flight_identity(candidate_identity, pending_identity))
      return PlannerRapidCandidateIdentityDecision::kBusyDropDifferentFlight;
    return candidate_revision_id > pending_revision_id &&
                   candidate_command_seq > pending_command_seq
        ? PlannerRapidCandidateIdentityDecision::kUpdatePendingRevision
        : PlannerRapidCandidateIdentityDecision::kKeepPendingRevision;
  }
  return PlannerRapidCandidateIdentityDecision::kLatchFreshFlight;
}

// A pending snapshot may outlive the latest-value mailbox entry when a later physical flight is
// busy-dropped. Preserve the first tuple, but never turn that latch into an infinite freshness
// lease: commit still requires a recent schema-2 revision on the local monotonic clock.
inline bool planner_rapid_pending_is_fresh(
    double now_steady_s, double last_fresh_steady_s,
    double command_timeout_s) noexcept {
  return a3_deploy::numeric_safety::IsFinite(now_steady_s) &&
      a3_deploy::numeric_safety::IsFinite(last_fresh_steady_s) &&
      a3_deploy::numeric_safety::IsFinite(command_timeout_s) &&
      last_fresh_steady_s > 0.0 && command_timeout_s >= 0.0 &&
      now_steady_s >= last_fresh_steady_s &&
      now_steady_s - last_fresh_steady_s <= command_timeout_s;
}

struct PlannerRapidPreemptEdge {
  int swing_direction = 1;
  int pending_swing_direction = 0;
  std::uint64_t clock_origin_tick = 0;
  std::uint64_t shot_sequence = 0;
};

// The schema-22 policy was trained on an actor-owned Markov transition, so the runtime edge is
// a direct direction/clock/identity replacement.  It must never route an opposite-side flight
// through set_swing_dir()'s historical mid-swing queue.
inline PlannerRapidPreemptEdge planner_rapid_preempt_edge(
    std::uint64_t tick, double swing_sign,
    std::uint64_t shot_sequence_before) noexcept {
  return PlannerRapidPreemptEdge{
      swing_sign >= 0.0 ? 1 : -1, 0, tick, shot_sequence_before + 1};
}

// A fresh-flight mailbox may already contain the next ball while the current ball is swinging.
// Schema 22 therefore permits streaming refinements only when they belong to the exact frozen
// flight.  Passing false preserves every older schema's historical per-tick behavior.
inline bool planner_stream_target_matches_frozen_flight(
    bool require_frozen_flight,
    const PlannerFlightIdentity& candidate_identity,
    const PlannerFlightIdentity& frozen_identity) noexcept {
  return !require_frozen_flight ||
         planner_same_flight_identity(candidate_identity, frozen_identity);
}

struct PlannerPhaseStart {
  double clock_tts_s = 0.0;
  double expected_strike_lateness_s = 0.0;
  bool late_phase_clamped = false;
};

// A future command is always accepted by the policy-native timing path, but a
// command that arrives inside the dynamic part of a frozen-target clip must not
// teleport the actor directly to that frame. Start its reference clock at the
// deepest contract-supported near-static prefix instead. This changes only the
// actor clock seed; it is not a command-admission or stability gate.
inline PlannerPhaseStart planner_phase_continuous_start(
    bool enabled, double raw_tts_s, double deepest_prefix_tts_s) noexcept {
  PlannerPhaseStart result{raw_tts_s, 0.0, false};
  if (!enabled || !a3_deploy::numeric_safety::IsFinite(raw_tts_s) ||
      !a3_deploy::numeric_safety::IsFinite(deepest_prefix_tts_s) ||
      raw_tts_s <= 0.0 || deepest_prefix_tts_s <= 0.0) {
    return result;
  }
  result.clock_tts_s = std::max(raw_tts_s, deepest_prefix_tts_s);
  result.expected_strike_lateness_s = result.clock_tts_s - raw_tts_s;
  result.late_phase_clamped = result.clock_tts_s > raw_tts_s;
  return result;
}

// V4's motion frame is a pure function of the actor-visible signed TTS: the same 0.40-s
// observation must select the same clip frame regardless of the COMMIT time. Historical frozen
// clips retain their near-static-prefix protection and possibly shifted reference clock.
inline PlannerPhaseStart planner_contact_frame_start(
    bool signed_tts_contact_frame,
    bool legacy_phase_clamp_enabled,
    double actor_tts_s,
    double deepest_prefix_tts_s) noexcept {
  if (signed_tts_contact_frame) {
    return PlannerPhaseStart{actor_tts_s, 0.0, false};
  }
  return planner_phase_continuous_start(
      legacy_phase_clamp_enabled, actor_tts_s, deepest_prefix_tts_s);
}

inline double planner_motion_reference_tts(
    bool signed_tts_contact_frame,
    double actor_tts_s,
    double legacy_reference_tts_s) noexcept {
  return signed_tts_contact_frame ? actor_tts_s : legacy_reference_tts_s;
}

// Schema is a wire-contract requirement. Stability count is deliberately
// accepted at every value: it remains audit telemetry and never controls
// release for the revisioned model_21800 path.
inline bool planner_revision_release_blocked(bool requires_schema2,
                                             int schema,
                                             int stable_revision_count) noexcept {
  (void)stable_revision_count;
  return requires_schema2 && schema != 2;
}

// A planner command's strike_time is an absolute timestamp. Predictions for the
// same physical ball may move slightly as more mocap samples arrive, while the
// next ball is separated by a much larger interval. This comparison is a
// lifecycle guard: after one engage, the same physical shot cannot be consumed
// again even if the latest-value mailbox still contains a fresh command.
inline bool same_planner_shot(double candidate_strike_time,
                              double consumed_strike_time,
                              double tolerance_s) noexcept {
  return a3_deploy::numeric_safety::IsFinite(candidate_strike_time) &&
         a3_deploy::numeric_safety::IsFinite(consumed_strike_time) &&
         a3_deploy::numeric_safety::IsFinite(tolerance_s) &&
         candidate_strike_time > 0.0 && consumed_strike_time > 0.0 &&
         tolerance_s >= 0.0 &&
         std::fabs(candidate_strike_time - consumed_strike_time) <= tolerance_s;
}

// v2 uses the Planner's explicit fresh-flight event as shot identity. This is
// what permits two valid rapid balls whose strike times are less than the v1
// tolerance apart, while revisions of one flight remain one-shot even if their
// predicted strike time moves. v1 retains the strike-time compatibility path.
inline bool same_planner_shot_identity(
    bool require_external_flight_event, std::uint64_t candidate_flight_id,
    std::uint64_t consumed_flight_id, double candidate_strike_time,
    double consumed_strike_time, double tolerance_s) noexcept {
  if (require_external_flight_event) {
    return candidate_flight_id > 0 && consumed_flight_id > 0 &&
           candidate_flight_id == consumed_flight_id;
  }
  return same_planner_shot(candidate_strike_time, consumed_strike_time,
                           tolerance_s);
}

// A planner/localization gap must block release without immediately destroying a station walk.
// The station remains useful while the ball is still approaching (and briefly after its predicted
// strike time); only an explicitly expired shot is allowed to tear down the pending lifecycle.
// This helper is pure so the safety boundary is covered without constructing the full runner.
inline PendingStationGapDecision pending_station_gap_decision(
    bool pending_active, double time_to_strike, double expire_after_strike_s) noexcept {
  if (!pending_active) return PendingStationGapDecision::kNoPending;
  if (!a3_deploy::numeric_safety::IsFinite(time_to_strike) ||
      !a3_deploy::numeric_safety::IsFinite(expire_after_strike_s) ||
      expire_after_strike_s < 0.0) {
    return PendingStationGapDecision::kHoldBlocked;
  }
  return time_to_strike < -expire_after_strike_s
      ? PendingStationGapDecision::kExpire
      : PendingStationGapDecision::kHoldBlocked;
}

// strike_time is the physical-shot identity. If either side lacks that identity, callers retain
// the legacy station/clip comparison instead of inventing a new-shot edge.
inline bool planner_shot_changed(double candidate_strike_time,
                                 double pending_strike_time,
                                 double tolerance_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(candidate_strike_time) ||
      !a3_deploy::numeric_safety::IsFinite(pending_strike_time) ||
      candidate_strike_time <= 0.0 || pending_strike_time <= 0.0) {
    return false;
  }
  return !same_planner_shot(candidate_strike_time, pending_strike_time, tolerance_s);
}

// Policy-native field execution lets the learned hold/recovery behavior run on
// the ball clock. READY and yaw remain visible as telemetry, but do not become
// extra release conditions that were absent from training.
inline bool planner_heading_blocks_release(bool policy_native, bool yaw_outside_limit) noexcept {
  return yaw_outside_limit && !policy_native;
}

inline bool planner_station_blocks_release(bool policy_native,
                                           bool station_only,
                                           bool station_ready) noexcept {
  if (station_only) return true;
  return !policy_native && !station_ready;
}

inline bool planner_target_blocks_release(bool policy_native,
                                          bool target_in_support) noexcept {
  return !policy_native && !target_in_support;
}

// Legacy field policy-native execution treats transport freshness and a later invalid revision as
// audit signals. Continuous-rally-v3 instead marks the external command authoritative, so stale or
// superseded-invalid input must fail closed even in policy-native mode. Isolated
// replay/qualification profiles retain their stricter admission behavior.
inline bool planner_command_health_blocks_release(
    bool policy_native, bool unhealthy,
    bool authoritative_health_fail_closed = false) noexcept {
  return unhealthy && (authoritative_health_fail_closed || !policy_native);
}

inline bool planner_command_age_unhealthy(
    double valid_age_s, double timeout_s,
    bool authoritative_health_fail_closed = false) noexcept {
  if (authoritative_health_fail_closed &&
      (!std::isfinite(valid_age_s) || valid_age_s < 0.0 ||
       !std::isfinite(timeout_s) || timeout_s < 0.0)) {
    return true;
  }
  return valid_age_s > timeout_s;
}

inline bool planner_invalid_revision_unhealthy(
    bool invalid_after, double valid_age_s, double grace_s,
    bool authoritative_health_fail_closed = false) noexcept {
  if (!invalid_after) return false;
  return authoritative_health_fail_closed || valid_age_s > grace_s;
}

// The clip already owns its recovery frames.  Do not add a second inter-shot
// admission delay to normal policy-native field execution; retain the legacy
// rest only for replay/qualification profiles.
inline bool planner_rest_blocks_release(bool policy_native,
                                        bool rest_active) noexcept {
  return rest_active && !policy_native;
}

// A positive TTS is an executable future event, even when it arrives after the
// nominal near-static prefix boundary.  In policy-native field execution the
// cutoff is telemetry only; the reference clock starts at the corresponding
// in-clip phase.  A non-positive TTS is not a gate rejection: the event has
// already expired and there is no future strike time to execute.
inline bool planner_timing_blocks_release(bool policy_native,
                                          double time_to_strike_s,
                                          double nominal_cutoff_s) noexcept {
  if (!a3_deploy::numeric_safety::IsFinite(time_to_strike_s) ||
      !a3_deploy::numeric_safety::IsFinite(nominal_cutoff_s)) {
    return true;
  }
  if (time_to_strike_s <= 0.0) return true;
  return !policy_native && time_to_strike_s < nominal_cutoff_s;
}

// A same-shot target can temporarily leave the strike support (for example while a one-bounce
// prediction passes through an unstable intermediate fit) without invalidating the already-derived
// station. Keep accumulating move/settle readiness for that unchanged pending station, but keep the
// strike release blocked until a supported target returns. A new/different station still fails
// closed and must create a fresh lifecycle.
inline bool pending_station_can_progress_during_target_gap(
    bool pending_active, bool same_clip, double station_delta_m,
    double same_station_tolerance_m = 0.05) noexcept {
  return pending_active && same_clip &&
         a3_deploy::numeric_safety::IsFinite(station_delta_m) &&
         a3_deploy::numeric_safety::IsFinite(same_station_tolerance_m) &&
         same_station_tolerance_m >= 0.0 &&
         station_delta_m <= same_station_tolerance_m;
}

// Release may bridge a short target-support flap only from a previously supported command for
// the exact pending lifecycle.  This is deliberately stricter than station progression: the
// station must already be READY, the clip must still match, and the supported target must be no
// older than the existing planner-invalid grace.  Timing-window checks remain with the caller.
inline bool pending_target_latch_can_release(
    bool pending_active, bool station_ready, bool same_clip,
    double supported_target_age_s, double grace_s) noexcept {
  return pending_active && station_ready && same_clip &&
         a3_deploy::numeric_safety::IsFinite(supported_target_age_s) &&
         a3_deploy::numeric_safety::IsFinite(grace_s) &&
         supported_target_age_s >= 0.0 && grace_s >= 0.0 &&
         supported_target_age_s <= grace_s;
}

}  // namespace a3_pingpong
