#include "hope_planner_cpp/flight_transport_v2.hpp"
#include <cmath>
#include <iomanip>
#include <locale>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
namespace hope_planner_cpp {
namespace {
template <class T> void vector(T &out, const Vec3 &v) {
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
}
template <class T> Vec3 vector(const T &v) { return Vec3(v.x, v.y, v.z); }
const char *context_name(MatchContext c) {
  return c == MatchContext::SERVE          ? "SERVE"
         : c == MatchContext::NORMAL_RALLY ? "NORMAL_RALLY"
                                           : "UNKNOWN";
}
} // namespace
const char *name(EffectiveSpinMode m) {
  switch (m) {
  case EffectiveSpinMode::MEASURED:
    return "MEASURED";
  case EffectiveSpinMode::HELD:
    return "HELD";
  case EffectiveSpinMode::CONTACT_PROPAGATED:
    return "CONTACT_PROPAGATED";
  default:
    return "POSITION_ONLY";
  }
}
const char *name(EventStatus s) {
  switch (s) {
  case EventStatus::CONFIRMED:
    return "CONFIRMED";
  case EventStatus::SUPERSEDED:
    return "SUPERSEDED";
  case EventStatus::CANCELLED:
    return "CANCELLED";
  case EventStatus::AMBIGUOUS:
    return "AMBIGUOUS";
  default:
    return "PREDICTED";
  }
}
const char *name(ForecastStatus s) {
  switch (s) {
  case ForecastStatus::READY:
    return "READY";
  case ForecastStatus::WARMING_UP:
    return "WARMING_UP";
  case ForecastStatus::HORIZON_EXHAUSTED:
    return "HORIZON_EXHAUSTED";
  case ForecastStatus::UNCERTAIN:
    return "UNCERTAIN";
  default:
    return "REVOKED";
  }
}
BallSample decode_observation(const hope_msgs::msg::BallObservationV2 &w) {
  BallSample s;
  s.source_time_ns = w.source_time_ns;
  s.source_time_valid = w.source_time_valid && w.schema_version == 2;
  s.clock_domain = w.clock_domain;
  s.clock_mapping_id = w.clock_mapping_id;
  s.clock_mapping_valid = w.clock_mapping_valid;
  s.clock_uncertainty_s = w.clock_uncertainty_s;
  s.sequence = w.sequence;
  s.receipt_wall_ns = w.receipt_wall_ns;
  s.frame_id = w.frame_id;
  s.calibration_id = w.calibration_id;
  s.ball_id = w.ball_id;
  s.position = vector(w.position);
  s.position_valid = w.position_valid && s.position.allFinite();
  s.orientation = Eigen::Quaterniond(w.orientation.w, w.orientation.x,
                                     w.orientation.y, w.orientation.z);
  s.orientation_authoritative = w.orientation_authoritative;
  s.orientation_valid = w.orientation_authoritative && w.orientation_valid &&
                        s.orientation.coeffs().allFinite() &&
                        s.orientation.norm() >= .5 &&
                        s.orientation.norm() <= 1.5;
  if (s.orientation_valid)
    s.orientation.normalize();
  s.producer_omega_rad_s = vector(w.omega_rad_s);
  s.producer_omega_valid = w.omega_valid &&
                           s.producer_omega_rad_s.allFinite() &&
                           s.producer_omega_rad_s.squaredNorm() > 0;
  if (s.clock_domain.empty() || !std::isfinite(s.clock_uncertainty_s) ||
      s.clock_uncertainty_s < 0)
    s.clock_mapping_valid = false;
  return s;
}
hope_msgs::msg::BallObservationV2 encode_observation(const BallSample &s) {
  hope_msgs::msg::BallObservationV2 w;
  w.source_time_ns = s.source_time_ns;
  w.source_time_valid = s.source_time_valid;
  w.clock_domain = s.clock_domain;
  w.clock_mapping_id = s.clock_mapping_id;
  w.clock_mapping_valid = s.clock_mapping_valid;
  w.clock_uncertainty_s = s.clock_uncertainty_s;
  w.sequence = s.sequence;
  w.receipt_wall_ns = s.receipt_wall_ns;
  w.frame_id = s.frame_id;
  w.calibration_id = s.calibration_id;
  w.ball_id = s.ball_id;
  vector(w.position, s.position);
  w.position_valid = s.position_valid;
  w.orientation.x = s.orientation.x();
  w.orientation.y = s.orientation.y();
  w.orientation.z = s.orientation.z();
  w.orientation.w = s.orientation.w();
  w.orientation_authoritative = s.orientation_authoritative;
  w.orientation_valid = s.orientation_valid && s.orientation_authoritative;
  vector(w.omega_rad_s, s.producer_omega_rad_s);
  w.omega_valid = s.producer_omega_valid;
  return w;
}
hope_msgs::msg::FlightPredictionV2 encode_prediction(const FlightPrediction &p,
                                                     const ShotContext &shot) {
  hope_msgs::msg::FlightPredictionV2 w;
  w.shot_generation = p.token.generation;
  w.context_revision = p.token.context_revision;
  w.forecast_revision = p.token.solve_sequence;
  w.play_state = name(p.state);
  w.forecast_route = name(p.route);
  w.forecast_status = name(p.status);
  w.classification_status =
      shot.classification() == ClassificationStatus::CONFIRMED ? "CONFIRMED"
                                                               : "PROVISIONAL";
  w.history_continuity = name(shot.continuity());
  w.match_context = context_name(shot.match_context());
  w.physical_available = p.physical_available;
  w.target_valid = p.target.valid;
  w.target_incoming = p.target.incoming;
  w.target_historical = p.target.historical;
  w.reason = p.reason;
  w.prediction_reference_time_ns = p.reference_ns;
  w.arrival_dt_ns = p.arrival_dt_ns;
  w.predicted_arrival_time_ns = p.arrival_ns;
  w.expires_at_ns = p.expires_ns;
  w.uncertainty_provenance = p.nominal.uncertainty_provenance;
  w.plane_x = p.plane.x_hit_m;
  w.plane_available = p.plane.calibrated;
  w.plane_revision = p.plane.generation;
  w.plane_pose_source_time_ns = p.plane.source_ns;
  if (!shot.samples().empty()) {
    const auto &s = shot.samples().back();
    w.frame_id = s.frame_id;
    w.calibration_id = s.calibration_id;
    w.ball_id = s.ball_id;
    w.clock_domain = s.clock_domain;
    w.clock_mapping_id = s.clock_mapping_id;
  }
  if (p.target.event) {
    const auto &e = *p.target.event;
    w.target_kind = name(e.kind);
    for (const auto &entry : shot.events())
      if (entry.kind == e.kind && entry.status != EventStatus::SUPERSEDED &&
          entry.status != EventStatus::CANCELLED &&
          ((!p.target.historical && entry.event_time_ns == p.arrival_ns) ||
           (p.target.historical &&
            (entry.pre.position - e.pre.position).norm() < 1e-6))) {
        w.target_event_id = entry.id;
        break;
      }
    vector(w.target_position, e.pre.position);
    vector(w.target_velocity, e.pre.velocity);
    w.target_spin_available = e.pre.spin.usable();
    w.target_spin_mode = name(e.pre.spin.mode);
    w.target_spin_measurement_time_ns = e.pre.spin.measurement_time_ns;
    w.target_spin_last_contact_id = e.pre.spin.last_contact_id;
    if (e.pre.spin.usable())
      vector(w.target_omega_rad_s, *e.pre.spin.omega);
  }
  for (const auto &e : shot.events()) {
    hope_msgs::msg::FlightEventV2 event;
    event.event_id = e.id;
    event.shot_generation = e.generation;
    event.kind = name(e.kind);
    event.status = name(e.status);
    event.side = name(e.side);
    event.event_time_ns = e.event_time_ns;
    event.confirmed_at_ns = e.confirmed_at_ns;
    event.time_uncertainty_ns = e.time_uncertainty_ns;
    event.prediction_reference_time_ns = p.reference_ns;
    if (e.event_time_ns >= p.reference_ns)
      event.arrival_dt_ns = e.event_time_ns - p.reference_ns;
    vector(event.position, e.pre.position);
    vector(event.pre_velocity, e.pre.velocity);
    vector(event.post_velocity, e.post.velocity);
    event.spin_available = e.post.spin.usable();
    event.spin_mode = name(e.post.spin.mode);
    event.spin_measurement_time_ns = e.post.spin.measurement_time_ns;
    event.spin_last_contact_id = e.post.spin.last_contact_id;
    if (e.post.spin.usable())
      vector(event.omega_rad_s, *e.post.spin.omega);
    event.uncertainty_provenance = w.uncertainty_provenance;
    event.linearization_quality = "unavailable";
    event.covariance.fill(NAN);
    for (const auto &physical : p.nominal.events)
      if (physical.kind == event.kind &&
          std::abs(physical.dt_s - event.arrival_dt_ns * 1e-9) < .03) {
        for (int row = 0; row < 7; ++row)
          for (int col = 0; col < 7; ++col)
            event.covariance[row * 7 + col] = physical.covariance(row, col);
        event.linearization_quality = quality_name(physical.quality);
        break;
      }
    w.events.push_back(event);
  }
  for (const auto &point : p.nominal.waypoints) {
    auto dt = duration_ns(point.dt_s);
    hope_msgs::msg::FlightWaypointV2 item;
    if (!dt ||
        !checked_arrival(p.reference_ns, *dt, item.predicted_arrival_time_ns))
      continue;
    item.prediction_reference_time_ns = p.reference_ns;
    item.arrival_dt_ns = *dt;
    vector(item.position, point.state.head<3>());
    vector(item.velocity, point.state.tail<3>());
    item.spin_available = point.spin.usable();
    item.spin_mode = name(point.spin.mode);
    if (point.spin.usable())
      vector(item.omega_rad_s, *point.spin.omega);
    item.linearization_quality = quality_name(point.quality);
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col)
        item.covariance[row * 6 + col] = point.covariance(row, col);
    w.waypoints.push_back(item);
  }
  return w;
}
std::string packet_v2_hash(hope_msgs::msg::FlightPacketV2 packet) {
  packet.payload_sha256.clear();
  // Canonical value serialization excludes CDR alignment padding. Seventeen
  // digits distinguish every finite double; field order is pinned by schema
  // version 2.
  std::ostringstream canonical;
  canonical.imbue(std::locale::classic());
  canonical << std::setprecision(17);
  hope_msgs::msg::to_flow_style_yaml(packet, canonical);
  return sha256("hope-flight-packet-v2:canonical-flow-yaml-f64-17:" +
                canonical.str());
}
bool validate_packet_v2(const hope_msgs::msg::FlightPacketV2 &p,
                        std::string &reason) {
  auto fail = [&](const char *why) {
    reason = why;
    return false;
  };
  if (p.schema_version != 2 || p.session_id.empty() || p.producer_id.empty() ||
      !p.shot_generation || p.config_id.empty())
    return fail("packet_identity");
  if (p.observations.empty() || p.observations.size() > 4096 ||
      p.event_ledger.size() > 4096)
    return fail("packet_capacity");
  std::optional<BallSample> previous;
  for (const auto &wire : p.observations) {
    auto s = decode_observation(wire);
    if (!s.source_time_valid || !s.position_valid ||
        s.source_time_ns < p.boundary_time_ns)
      return fail("packet_sample_invalid");
    if (previous && (s.source_time_ns <= previous->source_time_ns ||
                     s.clock_domain != previous->clock_domain ||
                     s.frame_id != previous->frame_id ||
                     s.calibration_id != previous->calibration_id ||
                     s.ball_id != previous->ball_id ||
                     s.clock_mapping_id != previous->clock_mapping_id))
      return fail("packet_sample_order_or_provenance");
    previous = s;
  }
  if (p.checkpoint_spin_available &&
      (!vector(p.checkpoint_omega_rad_s).allFinite() ||
       p.checkpoint_measurement_time_ns >
           p.observations.back().source_time_ns ||
       p.checkpoint_spin_mode == "POSITION_ONLY"))
    return fail("packet_spin_checkpoint");
  for (const auto &e : p.event_ledger)
    if (e.shot_generation != p.shot_generation ||
        (e.status == "CONFIRMED" && e.confirmed_at_ns < e.event_time_ns))
      return fail("packet_event_identity");
  if (packet_v2_hash(p) != p.payload_sha256)
    return fail("packet_hash");
  reason = "valid";
  return true;
}
} // namespace hope_planner_cpp
namespace hope_planner_cpp {
hope_msgs::msg::FlightPacketV2
encode_packet_v2(const FlightPipeline &pipeline,
                 const FlightPrediction &prediction, const std::string &session,
                 const std::string &producer, const std::string &config) {
  hope_msgs::msg::FlightPacketV2 p;
  const auto snapshot = pipeline.shot().export_snapshot();
  const auto checkpoint = pipeline.checkpoint();
  p.session_id = session;
  p.producer_id = producer;
  p.config_id = config;
  p.shot_generation = snapshot.generation;
  p.context_revision = snapshot.revision;
  p.boundary_time_ns = snapshot.boundary_ns;
  p.history_continuity = name(snapshot.continuity);
  p.play_state = name(snapshot.state);
  p.forecast_route = name(pipeline.shot().route());
  p.checkpoint_spin_available = snapshot.spin.usable();
  if (snapshot.spin.usable())
    vector(p.checkpoint_omega_rad_s, *snapshot.spin.omega);
  p.checkpoint_spin_mode = name(snapshot.spin.mode);
  p.checkpoint_measurement_time_ns = snapshot.spin.measurement_time_ns;
  p.checkpoint_last_contact_id = snapshot.spin.last_contact_id;
  for (const auto &s : snapshot.observations)
    p.observations.push_back(encode_observation(s));
  p.event_ledger = encode_prediction(prediction, pipeline.shot()).events;
  p.source_time_origin_ns = pipeline.source_origin_ns();
  p.state_source_time_ns = checkpoint.belief.source_time_ns;
  p.last_measurement_time_ns = checkpoint.belief.last_measurement_time_ns;
  p.first_measurement_time_s = checkpoint.belief.first_measurement_time_s;
  p.last_measurement_gap_s = checkpoint.belief.last_measurement_gap_s;
  p.observation_count = checkpoint.belief.observation_count;
  p.rejected_measurements = checkpoint.belief.rejected_measurements;
  p.innovation_squared = checkpoint.belief.innovation_squared;
  p.innovation_rms_m = checkpoint.belief.innovation_rms_m;
  p.innovation_max_m = checkpoint.belief.innovation_max_m;
  p.innovation_sum_squared = checkpoint.innovation_sum_squared;
  p.consecutive_rejections = checkpoint.consecutive_rejections;
  p.noise_fingerprint = checkpoint.noise_fingerprint;
  for (int i = 0; i < 6; ++i)
    p.state_mean[i] = checkpoint.belief.mean[i];
  for (int i = 0; i < 3; ++i)
    p.bias_mean[i] = checkpoint.bias_mean[i];
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      p.state_covariance[i * 6 + j] = checkpoint.belief.covariance(i, j);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      p.bias_covariance[i * 3 + j] = checkpoint.bias_covariance(i, j);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 3; ++j)
      p.state_bias_covariance[i * 3 + j] =
          checkpoint.state_bias_covariance(i, j);
  p.state_spin_available = checkpoint.belief.effective_spin.usable();
  if (p.state_spin_available)
    vector(p.state_omega_rad_s, *checkpoint.belief.effective_spin.omega);
  p.state_spin_mode = name(checkpoint.belief.effective_spin.mode);
  p.state_spin_measurement_time_ns =
      checkpoint.belief.effective_spin.measurement_time_ns;
  p.state_spin_last_contact_id =
      checkpoint.belief.effective_spin.last_contact_id;
  p.match_context = context_name(snapshot.match);
  p.classification_status =
      snapshot.classification == ClassificationStatus::CONFIRMED
          ? "CONFIRMED"
          : "PROVISIONAL";
  p.payload_sha256 = packet_v2_hash(p);
  return p;
}
void restore_packet_v2(FlightPipeline &pipeline,
                       const hope_msgs::msg::FlightPacketV2 &p,
                       const std::string &expected_config) {
  std::string reason;
  if (!validate_packet_v2(p, reason) || p.config_id != expected_config)
    throw std::invalid_argument("packet v2 rejected:" + reason);
  ShotSnapshot snapshot;
  snapshot.generation = p.shot_generation;
  snapshot.revision = p.context_revision;
  snapshot.boundary_ns = p.boundary_time_ns;
  if (p.play_state == "RALLY")
    snapshot.state = PlayState::RALLY;
  else if (p.play_state == "SERVE")
    snapshot.state = PlayState::SERVE;
  else if (p.play_state == "OPPONENT_RALLY")
    snapshot.state = PlayState::OPPONENT_RALLY;
  else if (p.play_state == "OPPONENT_SERVE")
    snapshot.state = PlayState::OPPONENT_SERVE;
  else
    throw std::invalid_argument("invalid packet play state");
  snapshot.direction =
      (snapshot.state == PlayState::RALLY || snapshot.state == PlayState::SERVE)
          ? 1
          : -1;
  snapshot.continuity =
      p.history_continuity == "complete"     ? HistoryContinuity::kComplete
      : p.history_continuity == "incomplete" ? HistoryContinuity::kIncomplete
                                             : HistoryContinuity::kUnknown;
  snapshot.match = p.match_context == "SERVE" ? MatchContext::SERVE
                   : p.match_context == "NORMAL_RALLY"
                       ? MatchContext::NORMAL_RALLY
                       : MatchContext::UNKNOWN;
  snapshot.classification = p.classification_status == "CONFIRMED"
                                ? ClassificationStatus::CONFIRMED
                                : ClassificationStatus::PROVISIONAL;
  if (p.checkpoint_spin_available) {
    snapshot.spin.omega = vector(p.checkpoint_omega_rad_s);
    snapshot.spin.mode = p.checkpoint_spin_mode == "MEASURED"
                             ? EffectiveSpinMode::MEASURED
                         : p.checkpoint_spin_mode == "HELD"
                             ? EffectiveSpinMode::HELD
                             : EffectiveSpinMode::CONTACT_PROPAGATED;
    snapshot.spin.measurement_time_ns = p.checkpoint_measurement_time_ns;
    snapshot.spin.last_contact_id = p.checkpoint_last_contact_id;
  }
  for (const auto &wire : p.observations) {
    auto sample = decode_observation(wire);
    sample.shot_generation = p.shot_generation;
    sample.source_time_s =
        double((static_cast<long double>(sample.source_time_ns) -
                p.source_time_origin_ns) *
               1e-9L);
    snapshot.observations.push_back(sample);
  }
  for (const auto &wire : p.event_ledger) {
    ShotEvent e;
    e.id = wire.event_id;
    e.generation = wire.shot_generation;
    e.event_time_ns = wire.event_time_ns;
    e.confirmed_at_ns = wire.confirmed_at_ns;
    e.time_uncertainty_ns = wire.time_uncertainty_ns;
    e.kind = wire.kind == "table_bounce"      ? PhysicalEventKind::TABLE
             : wire.kind == "net_crossing"    ? PhysicalEventKind::NET_CROSSING
             : wire.kind == "net_collision"   ? PhysicalEventKind::NET_COLLISION
             : wire.kind == "apex"            ? PhysicalEventKind::APEX
             : wire.kind == "opponent_end"    ? PhysicalEventKind::END
             : wire.kind == "receiving_plane" ? PhysicalEventKind::PLANE
             : wire.kind == "edge_ambiguous" ? PhysicalEventKind::EDGE_AMBIGUOUS
                                             : PhysicalEventKind::GROUND;
    e.status = wire.status == "CONFIRMED"   ? EventStatus::CONFIRMED
               : wire.status == "PREDICTED" ? EventStatus::PREDICTED
               : wire.status == "CANCELLED" ? EventStatus::CANCELLED
               : wire.status == "AMBIGUOUS" ? EventStatus::AMBIGUOUS
                                            : EventStatus::SUPERSEDED;
    e.side = wire.side == "opponent"   ? ContactSide::kOpponent
             : wire.side == "receiver" ? ContactSide::kReceiver
                                       : ContactSide::kAmbiguous;
    e.pre.position = e.post.position = vector(wire.position);
    e.pre.velocity = vector(wire.pre_velocity);
    e.post.velocity = vector(wire.post_velocity);
    e.source = "packet_checkpoint";
    snapshot.events.push_back(e);
  }
  auto checkpoint = pipeline.checkpoint();
  checkpoint.belief.initialized = true;
  checkpoint.belief.source_time_origin_ns = p.source_time_origin_ns;
  checkpoint.belief.source_time_ns = p.state_source_time_ns;
  checkpoint.belief.last_measurement_time_ns = p.last_measurement_time_ns;
  checkpoint.belief.source_time_s =
      double((static_cast<long double>(p.state_source_time_ns) -
              p.source_time_origin_ns) *
             1e-9L);
  checkpoint.belief.first_measurement_time_s = p.first_measurement_time_s;
  checkpoint.belief.last_measurement_time_s =
      double((static_cast<long double>(p.last_measurement_time_ns) -
              p.source_time_origin_ns) *
             1e-9L);
  checkpoint.belief.last_measurement_gap_s = p.last_measurement_gap_s;
  checkpoint.belief.observation_count = p.observation_count;
  checkpoint.belief.rejected_measurements = p.rejected_measurements;
  checkpoint.belief.innovation_squared = p.innovation_squared;
  checkpoint.belief.innovation_rms_m = p.innovation_rms_m;
  checkpoint.belief.innovation_max_m = p.innovation_max_m;
  checkpoint.belief.effective_spin = {};
  if (p.state_spin_available) {
    checkpoint.belief.effective_spin.omega = vector(p.state_omega_rad_s);
    checkpoint.belief.effective_spin.mode =
        p.state_spin_mode == "MEASURED" ? EffectiveSpinMode::MEASURED
        : p.state_spin_mode == "HELD"   ? EffectiveSpinMode::HELD
                                        : EffectiveSpinMode::CONTACT_PROPAGATED;
    checkpoint.belief.effective_spin.measurement_time_ns =
        p.state_spin_measurement_time_ns;
    checkpoint.belief.effective_spin.last_contact_id =
        p.state_spin_last_contact_id;
  }
  checkpoint.innovation_sum_squared = p.innovation_sum_squared;
  checkpoint.consecutive_rejections = p.consecutive_rejections;
  checkpoint.noise_fingerprint = p.noise_fingerprint;
  for (int i = 0; i < 6; ++i)
    checkpoint.belief.mean[i] = p.state_mean[i];
  for (int i = 0; i < 3; ++i)
    checkpoint.bias_mean[i] = p.bias_mean[i];
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      checkpoint.belief.covariance(i, j) = p.state_covariance[i * 6 + j];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      checkpoint.bias_covariance(i, j) = p.bias_covariance[i * 3 + j];
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 3; ++j)
      checkpoint.state_bias_covariance(i, j) =
          p.state_bias_covariance[i * 3 + j];
  pipeline.restore(snapshot, checkpoint, p.source_time_origin_ns);
}
} // namespace hope_planner_cpp
