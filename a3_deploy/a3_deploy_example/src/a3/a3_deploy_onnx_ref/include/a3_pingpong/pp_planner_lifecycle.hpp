#pragma once

#include <algorithm>
#include <cmath>

#include "a3_deploy/numeric_safety.hpp"

namespace a3_pingpong {

enum class PendingStationGapDecision {
  kNoPending,
  kHoldBlocked,
  kExpire,
};

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

// Field policy-native execution treats transport freshness and a later invalid
// revision as audit signals.  The latest finite valid prediction remains the
// executable command while its strike time is still in the future.  Isolated
// replay/qualification profiles retain their stricter admission behavior.
inline bool planner_command_health_blocks_release(bool policy_native,
                                                  bool unhealthy) noexcept {
  return unhealthy && !policy_native;
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
