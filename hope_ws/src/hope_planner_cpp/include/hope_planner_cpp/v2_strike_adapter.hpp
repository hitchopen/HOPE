#pragma once

#include "hope_planner_cpp/return_eligibility.hpp"
#include "hope_planner_cpp/runtime_plane_reference.hpp"

namespace hope_planner_cpp {
enum class IncomingStatus { Acquiring, Unavailable, Supported, Ineligible, Revoked, Expired };
const char* name(IncomingStatus status);

// This context is supplied by the managed plane selector, never by a second
// HitPlaneManager. A preview is useful physical evidence but has no authority.
struct IncomingPlane {
  PlaneSelection selection;
  HitPlaneSnapshot snapshot;
  bool tentative = true;
};
IncomingPlane incoming_plane(const PlaneSelection&, const std::string& frame,
                             const std::string& registration, bool tentative,
                             std::int64_t pose_source_ns = 0);

struct V2StrikeCandidate {
  bool physical_state_available = false;
  bool strike_contact_supported = false;
  bool command_eligible = false;
  IncomingStatus status = IncomingStatus::Acquiring;
  std::string reason = "ACQUIRING";
  std::string session_id, model_config_id, uncertainty_provenance;
  ShotToken token;
  std::uint64_t semantic_epoch = 0, revision = 0;
  std::uint64_t command_sequence = 0, command_flight_id = 0;
  std::uint64_t command_revision_id = 0;
  std::uint64_t revokes_revision = 0;
  IncomingPlane plane;
  std::int64_t source_ns = 0, expires_ns = 0;
  std::optional<ContactPlanningInput> contact;
  std::vector<ShotEvent> observed_events;
  std::vector<PhysicalEvent> predicted_events;
  double contact_time_std_s = std::numeric_limits<double>::quiet_NaN();
  int future_bounces = 0;
};

V2StrikeCandidate evaluate_v2_strike(const FlightPrediction&, const ShotSnapshot&,
                                    const ProbabilisticConfig&, const IncomingPlane&,
                                    std::int64_t now_ns);
StrikeTarget v2_strike_target(const V2StrikeCandidate&, std::int64_t now_ns);
}  // namespace hope_planner_cpp
