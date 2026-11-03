#include "hope_planner_cpp/v2_strike_adapter.hpp"

namespace hope_planner_cpp {
const char* name(IncomingStatus status) {
  switch (status) {
    case IncomingStatus::Acquiring: return "ACQUIRING";
    case IncomingStatus::Unavailable: return "UNAVAILABLE";
    case IncomingStatus::Supported: return "SUPPORTED";
    case IncomingStatus::Ineligible: return "INELIGIBLE";
    case IncomingStatus::Revoked: return "REVOKED";
    case IncomingStatus::Expired: return "EXPIRED";
  }
  return "UNAVAILABLE";
}

IncomingPlane incoming_plane(const PlaneSelection& selection, const std::string& frame,
                             const std::string& registration, bool tentative,
                             std::int64_t pose_source_ns) {
  IncomingPlane plane;
  plane.selection = selection;
  plane.tentative = tentative;
  plane.snapshot.x_hit_m = selection.plane_x;
  plane.snapshot.base_x_m = selection.reference_x;
  plane.snapshot.offset_m = selection.plane_x - selection.reference_x;
  plane.snapshot.frame_id = frame;
  plane.snapshot.registration_id = registration;
  plane.snapshot.generation = selection.epoch;
  plane.snapshot.rally_id = selection.flight;
  plane.snapshot.source_ns = pose_source_ns;
  plane.snapshot.calibrated = selection.valid && selection.epoch != 0 &&
      std::isfinite(selection.plane_x) && std::isfinite(selection.reference_x) &&
      !frame.empty() && !registration.empty();
  plane.snapshot.reason = plane.snapshot.calibrated
      ? (tentative ? "tentative_managed_plane" : "committed_managed_plane") : selection.reason;
  return plane;
}

V2StrikeCandidate evaluate_v2_strike(const FlightPrediction& forecast, const ShotSnapshot& shot,
                                    const ProbabilisticConfig& config, const IncomingPlane& plane,
                                    std::int64_t now_ns) {
  V2StrikeCandidate candidate;
  candidate.plane = plane;
  candidate.token = forecast.token;
  candidate.source_ns = forecast.reference_ns;
  candidate.expires_ns = forecast.expires_ns;
  candidate.model_config_id = forecast.model_config_id;
  candidate.uncertainty_provenance = forecast.nominal.uncertainty_provenance;
  candidate.observed_events = shot.events;
  candidate.predicted_events = forecast.events;
  candidate.physical_state_available = forecast.physical_available && forecast.belief.initialized;
  if (shot.observations.empty() || !forecast.belief.initialized) return candidate;
  candidate.status = IncomingStatus::Unavailable;
  candidate.reason = "SOURCE_PREFIX_MISMATCH";
  if (!shot.generation || forecast.token.generation != shot.generation ||
      forecast.token.context_revision != shot.revision ||
      forecast.token.source_ns != forecast.reference_ns ||
      shot.observations.back().source_time_ns != forecast.reference_ns)
    return candidate;
  for (const auto& event : shot.events) {
    if (event.status == EventStatus::CONFIRMED && event.generation == shot.generation &&
        event.event_time_ns >= shot.boundary_ns && event.event_time_ns <= forecast.reference_ns &&
        event.confirmed_at_ns <= forecast.reference_ns &&
        (event.kind == PhysicalEventKind::NET_COLLISION || event.kind == PhysicalEventKind::GROUND)) {
      candidate.status = IncomingStatus::Ineligible;
      candidate.reason = "CONFIRMED_TERMINAL_CONTACT";
      return candidate;
    }
  }
  candidate.reason = "PLANE_IDENTITY_MISMATCH";
  if (!plane.snapshot.calibrated || !forecast.plane.calibrated ||
      forecast.plane.x_hit_m != plane.snapshot.x_hit_m ||
      forecast.plane.generation != plane.snapshot.generation ||
      forecast.plane.rally_id != plane.snapshot.rally_id ||
      forecast.plane.frame_id != plane.snapshot.frame_id ||
      forecast.plane.registration_id != plane.snapshot.registration_id)
    return candidate;
  const auto eligibility = select_physical_return(forecast, shot, config, now_ns);
  candidate.reason = eligibility.reason;
  if (!eligibility.eligible) {
    if (eligibility.reason == "SOURCE_EXPIRED" || eligibility.reason == "CONTACT_EXPIRED")
      candidate.status = IncomingStatus::Expired;
    else if (eligibility.reason == "SECOND_RECEIVER_BOUNCE" ||
             eligibility.reason == "NOT_INCOMING" ||
             eligibility.reason == "PLANE_BEFORE_FIRST_BOUNCE" ||
             eligibility.reason == "PHYSICAL_TERMINATION_BEFORE_CONTACT")
      candidate.status = IncomingStatus::Ineligible;
    // An unresolved edge is not evidence of a confirmed physical termination.
    for (const auto& event : forecast.nominal.events) {
      if (event.kind == "receiving_plane") break;
      if (event.kind == "edge_ambiguous") {
        candidate.status = IncomingStatus::Unavailable;
        candidate.reason = "EDGE_ORDER_UNCERTAIN";
        break;
      }
    }
    return candidate;
  }
  candidate.contact = eligibility.input;
  candidate.strike_contact_supported = true;
  candidate.status = IncomingStatus::Supported;
  candidate.command_eligible = !plane.tentative && plane.selection.valid &&
      plane.selection.epoch != 0 && plane.selection.flight != 0;
  if (!candidate.command_eligible)
    candidate.reason = plane.tentative ? "TENTATIVE_PLANE" : "WAITING_FLIGHT_COMMIT";
  for (const auto& event : forecast.nominal.events) {
    if (event.kind == "receiving_plane") {
      candidate.contact_time_std_s = event.time_std_s;
      break;
    }
    if (event.kind == "table_bounce" && event.dt_s > 1e-9) ++candidate.future_bounces;
  }
  return candidate;
}

StrikeTarget v2_strike_target(const V2StrikeCandidate& candidate, std::int64_t now_ns) {
  StrikeTarget strike;
  strike.reason = candidate.reason;
  if (candidate.status != IncomingStatus::Supported || !candidate.command_eligible ||
      !candidate.strike_contact_supported || !candidate.physical_state_available ||
      candidate.plane.tentative || !candidate.contact)
    return strike;
  const auto& contact = *candidate.contact;
  if (now_ns > candidate.expires_ns || now_ns > contact.expires_ns ||
      now_ns >= contact.arrival_ns ||
      static_cast<long double>(now_ns) < static_cast<long double>(candidate.source_ns) - 1000000.L) {
    strike.reason = "CONTACT_OR_SOURCE_EXPIRED";
    return strike;
  }
  if (!contact.contact.pre.position.allFinite() || !contact.contact.pre.velocity.allFinite()) {
    strike.reason = "NONFINITE_CONTACT";
    return strike;
  }
  strike.ball_position = contact.contact.pre.position;
  strike.ball_velocity = contact.contact.pre.velocity;
  strike.strike_source_time_s = static_cast<double>(contact.arrival_ns) * 1e-9;
  strike.predicted_bounces = candidate.future_bounces;
  strike.valid = true;
  strike.reason = "V2_SUPPORTED_CONTACT";
  return strike;
}
}  // namespace hope_planner_cpp
