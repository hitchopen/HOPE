#include "hope_planner_cpp/return_eligibility.hpp"
#include <algorithm>
#include <cmath>

namespace hope_planner_cpp {
ReturnEligibility select_physical_return(const FlightPrediction &f, const ShotSnapshot &s,
                                        const ProbabilisticConfig &c, std::int64_t now) {
  ReturnEligibility out;
  auto fail = [&](const char *why) { out.reason = why; return out; };
  if (!f.physical_available || !f.belief.initialized || s.observations.empty() ||
      f.token.generation != s.generation)
    return fail("RETURN_UNSUPPORTED");
  if (s.direction >= 0 || !f.belief.mean.allFinite() || f.belief.mean[3] >= -.5)
    return fail("NOT_INCOMING");
  if (!f.plane.calibrated || f.plane.frame_id != s.observations.back().frame_id ||
      f.plane.registration_id.empty())
    return fail("PLANE_UNQUALIFIED");
  if (now > f.expires_ns || static_cast<long double>(now) <
      static_cast<long double>(f.reference_ns) - 1000000.L)
    return fail("SOURCE_EXPIRED");
  const auto &source = s.observations.back();
  if (!source.position_valid || !source.source_time_valid || !source.clock_mapping_valid ||
      source.frame_id.empty() || source.calibration_id.empty() ||
      source.calibration_id != f.plane.registration_id ||
      source.clock_mapping_id.empty() || source.ball_id.empty() ||
      !std::isfinite(source.clock_uncertainty_s) || source.clock_uncertainty_s < 0 ||
      source.clock_uncertainty_s > .001)
    return fail("SOURCE_UNQUALIFIED");
  int bounces = 0;
  bool observed = false;
  double latest_bounce_upper = -std::numeric_limits<double>::infinity();
  // Only confirmed local observations. Predicted ledger entries are counted
  // below from this rollout, so refinement cannot double-count a bounce.
  for (const auto &e : s.events) {
    if (e.generation != s.generation || e.status != EventStatus::CONFIRMED ||
        e.event_time_ns < s.boundary_ns || e.event_time_ns > f.reference_ns ||
        e.confirmed_at_ns > f.reference_ns || e.kind != PhysicalEventKind::TABLE ||
        e.pre.position.x() >= c.table.net_x || e.pre.velocity.x() >= -.5)
      continue;
    ++bounces;
    observed = true;
    latest_bounce_upper = std::max(latest_bounce_upper,
        static_cast<double>((static_cast<long double>(e.event_time_ns) - f.reference_ns +
                             e.time_uncertainty_ns) * 1e-9L));
  }
  if (bounces > 1) return fail("SECOND_RECEIVER_BOUNCE");
  double previous = -1;
  for (const auto &e : f.nominal.events) {
    if (!std::isfinite(e.dt_s) || e.dt_s < previous || !e.pre.allFinite())
      return fail("EVENT_ORDER_UNSUPPORTED");
    previous = e.dt_s;
    if (e.kind == "ground" || e.kind == "edge_ambiguous" || e.kind == "net_collision")
      return fail("PHYSICAL_TERMINATION_BEFORE_CONTACT");
    if (e.kind == "table_bounce" && e.pre[0] < c.table.net_x && e.dt_s > 1e-9) {
      if (++bounces > 1) return fail("SECOND_RECEIVER_BOUNCE");
      if (e.quality != LinearizationQuality::Valid || !std::isfinite(e.time_std_s) || e.time_std_s < 0)
        return fail("BOUNCE_ORDER_UNCERTAIN");
      latest_bounce_upper = e.dt_s + 3 * e.time_std_s;
    }
    if (e.kind != "receiving_plane") continue;
    if (bounces != 1) return fail("PLANE_BEFORE_FIRST_BOUNCE");
    if (e.dt_s <= 0 || e.pre[3] >= -.5 ||
        e.quality != LinearizationQuality::Valid || !e.covariance.allFinite() ||
        !std::isfinite(e.time_std_s) || e.time_std_s < 0 || e.time_std_s > .05 ||
        e.dt_s - 3 * e.time_std_s <= latest_bounce_upper + .001 ||
        e.covariance.diagonal().minCoeff() < -1e-12 ||
        e.covariance.diagonal().head<3>().maxCoeff() > .15 * .15)
      return fail("CONTACT_ORDER_UNCERTAIN");
    ContactPlanningInput input;
    input.token = f.token; input.plane = f.plane; input.source = source;
    input.contact.kind = PhysicalEventKind::PLANE; input.contact.dt_s = e.dt_s;
    input.contact.pre.position = e.pre.head<3>();
    input.contact.pre.velocity = e.pre.tail<3>(); input.contact.pre.spin = e.pre_spin;
    input.contact.post = input.contact.pre; input.covariance = e.covariance;
    auto dt = duration_ns(e.dt_s);
    if (!dt || !checked_arrival(f.reference_ns, *dt, input.arrival_ns) || input.arrival_ns <= now)
      return fail("CONTACT_EXPIRED");
    input.expires_ns = f.expires_ns; input.history = s.continuity;
    input.game_context = s.match; input.bounce_observed = observed;
    input.model_config_id = f.model_config_id;
    input.observation_count = f.belief.observation_count;
    input.observation_span_s = f.belief.last_measurement_time_s - f.belief.first_measurement_time_s;
    for (const auto &event : s.events)
      if (event.kind == PhysicalEventKind::PLANE && event.status == EventStatus::PREDICTED)
        input.event_id = event.id;
    out.input = std::move(input); out.eligible = true; out.reason = "RETURN_ELIGIBLE";
    return out;
  }
  return fail("NO_FUTURE_PLANE");
}
} // namespace hope_planner_cpp
