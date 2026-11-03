#include "hope_planner_cpp/v2_incoming_runtime.hpp"

namespace hope_planner_cpp {
namespace {
template<class Wire> void vector(Wire& wire, const Vec3& value) {
  wire.x = value.x(); wire.y = value.y(); wire.z = value.z();
}
std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
V2IncomingRuntime::V2IncomingRuntime(rclcpp::Node& node, const BallPhysics& physics,
                                   const TableParams& table, std::function<void()> wake,
                                   std::function<std::pair<std::uint64_t, bool>()> reference,
                                   std::function<bool(const V2StrikeCandidate&)> revoke)
    : node_(node), revoke_(std::move(revoke)) {
  auto text = [&](const char* key, const char* fallback = "") {
    return node.declare_parameter<std::string>(std::string("v2_incoming.") + key, fallback);
  };
  if (text("output_mode", "shadow") != "shadow")
    throw std::invalid_argument("v2_incoming requires separately authorized deployment; use shadow");
  contract_.session = text("session_id");
  contract_.frame = text("frame_id");
  contract_.registration = text("calibration_id");
  contract_.clock_domain = text("clock_domain");
  contract_.clock_mapping = text("clock_mapping_id");
  contract_.ball = text("ball_id");
  const auto calibration = node.declare_parameter<std::int64_t>("v2_incoming.base_calibration_id", 0);
  const auto world = node.declare_parameter<std::int64_t>("v2_incoming.base_world_id", 0);
  if (calibration <= 0 || world <= 0)
    throw std::invalid_argument("v2_incoming requires explicit registered base calibration/world identities");
  base_calibration_ = calibration; base_world_ = world;
  producer_ = text("producer_id", "v2-incoming-planner");
  accounting_.configure(text("stage_trace_path"));
  continuous_output_ = node.declare_parameter("v2_incoming.continuous_output", true);
  full_rate_hz_ = node.declare_parameter("v2_incoming.full_output_rate_hz", 50.);
  if (!std::isfinite(full_rate_hz_) || full_rate_hz_ < 0 || full_rate_hz_ > 300)
    throw std::invalid_argument("full_output_rate_hz must be in [0,300]; zero disables full output");
  ProbabilisticConfig config; config.physics = physics; config.table = table;
  owner_ = std::make_unique<V2IncomingCoordinator>(config, ShotSettings{}, contract_);
  compact_ = node.create_publisher<hope_msgs::msg::FlightPredictionV2>(
      std::string(prefix)+"prediction", rclcpp::QoS(1).reliable());
  full_ = node.create_publisher<hope_msgs::msg::FlightPredictionV2>(
      std::string(prefix)+"trajectory", rclcpp::QoS(1).best_effort());
  candidates_ = node.create_publisher<hope_msgs::msg::StrikeCandidateV1>(
      std::string(prefix)+"candidate", rclcpp::QoS(1).reliable());
  // Separate retained status stream prevents latest-value candidate traffic
  // from hiding a revocation of an earlier one-shot command.
  revocations_ = node.create_publisher<hope_msgs::msg::StrikeCandidateV1>(
      std::string(prefix)+"revocation", rclcpp::QoS(64).reliable().transient_local());
  if (std::string(compact_->get_topic_name()) != std::string(prefix)+"prediction" ||
      std::string(full_->get_topic_name()) != std::string(prefix)+"trajectory" ||
      std::string(candidates_->get_topic_name()) != std::string(prefix)+"candidate" ||
      std::string(revocations_->get_topic_name()) != std::string(prefix)+"revocation")
    throw std::invalid_argument("V2 incoming shadow outputs cannot be remapped");
  observations_ = node.create_subscription<hope_msgs::msg::BallObservationV2>(
      text("observation_topic", "/hope/ball/observation_v2"), rclcpp::QoS(8).best_effort(),
      [this, wake](hope_msgs::msg::BallObservationV2::ConstSharedPtr message) {
        auto sample = decode_observation(*message);
        sample.receipt_steady_ns = steady_ns();
        sample.receipt_wall_ns = node_.now().nanoseconds();
        trace("planner_received", sample.source_time_ns, sample.sequence, 0);
        const bool accepted = owner_->observe(sample, sample.receipt_wall_ns);
        trace(accepted ? "tracker_accepted" : "tracker_rejected", sample.source_time_ns,
              sample.sequence, 0, {}, sample.receipt_steady_ns);
        latest_source_ns_.store(owner_->counters().latest_source_ns);
        const auto replaced = pending_source_ns_.exchange(sample.source_time_ns);
        if (pending_.exchange(true)) {
          ++superseded_;
          if (replaced) trace("forecast_superseded", replaced, 0, 0);
        }
        trace("forecast_requested", sample.source_time_ns, sample.sequence, 0);
        wake();
      });
  expiry_ = node.create_wall_timer(std::chrono::milliseconds(5), [this, reference] {
    owner_->update_reference(reference);
    owner_->expire(node_.now().nanoseconds());
    flush_statuses();
  });
}

V2IncomingRuntime::~V2IncomingRuntime() {
  expiry_.reset(); observations_.reset();
  try { accounting_.flush(); }
  catch (const std::exception& e) { RCLCPP_ERROR(node_.get_logger(), "%s", e.what()); }
}
void V2IncomingRuntime::trace(const char* stage, std::int64_t source,
    std::uint64_t revision, std::uint64_t generation, const std::string& reason,
    std::int64_t started) {
  if (!accounting_.enabled()) return;
  const auto now = steady_ns();
  accounting_.record(stage, source, revision, generation, node_.now().nanoseconds(),
                     now, started ? now-started : 0, reason);
}

void V2IncomingRuntime::update_reference(
    const std::function<std::pair<std::uint64_t, bool>()>& update) {
  if (owner_->update_reference(update)) pending_.store(true);
}

void V2IncomingRuntime::candidate(const V2StrikeCandidate& value) {
  hope_msgs::msg::StrikeCandidateV1 wire;
  wire.session_id = contract_.session; wire.producer_id = producer_;
  wire.frame_id = contract_.frame; wire.calibration_id = contract_.registration;
  wire.clock_domain = contract_.clock_domain; wire.clock_mapping_id = contract_.clock_mapping;
  wire.ball_id = contract_.ball; wire.model_config_id = value.model_config_id;
  wire.home_epoch = value.plane.selection.epoch;
  wire.physical_shot_generation = value.token.generation;
  wire.context_revision = value.token.context_revision; wire.prediction_revision = value.revision;
  wire.semantic_epoch = value.semantic_epoch; wire.revokes_revision = value.revokes_revision;
  wire.command_sequence = value.command_sequence; wire.command_flight_id = value.command_flight_id;
  wire.source_time_ns = value.source_ns; wire.expires_at_ns = value.expires_ns;
  wire.physical_state_available = value.physical_state_available;
  wire.strike_contact_supported = value.strike_contact_supported;
  wire.command_eligible = value.command_eligible;
  wire.runner_commitment_state = "UNKNOWN";  // Shadow decoding is not engagement feedback.
  wire.status = name(value.status); wire.reason = value.reason;
  wire.plane_available = value.plane.snapshot.calibrated;
  wire.plane_tentative = value.plane.tentative;
  wire.plane_mode = static_cast<std::uint8_t>(value.plane.selection.mode);
  wire.plane_flight_id = value.plane.selection.flight;
  wire.plane_reference_x = value.plane.selection.reference_x;
  wire.plane_x = value.plane.snapshot.x_hit_m;
  wire.contact_time_std_s = value.contact_time_std_s;
  wire.uncertainty_provenance = value.uncertainty_provenance;
  wire.contact_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  if (value.contact) {
    wire.contact_time_ns = value.contact->arrival_ns;
    vector(wire.contact_position, value.contact->contact.pre.position);
    vector(wire.contact_velocity, value.contact->contact.pre.velocity);
    for (int row=0; row<7; ++row) for (int col=0; col<7; ++col)
      wire.contact_covariance[row*7+col] = value.contact->covariance(row,col);
  }
  // Event evidence is bounded and independent of the full trajectory cadence.
  // In particular, a revocation contains the newer contradictory event ledger.
  for (const auto& event : value.observed_events) {
    hope_msgs::msg::FlightEventV2 evidence;
    evidence.event_id = event.id; evidence.shot_id = evidence.shot_generation = event.generation;
    evidence.kind = name(event.kind); evidence.status = name(event.status); evidence.side = name(event.side);
    evidence.event_time_ns = event.event_time_ns; evidence.confirmed_at_ns = event.confirmed_at_ns;
    evidence.time_uncertainty_ns = event.time_uncertainty_ns;
    evidence.prediction_reference_time_ns = value.source_ns;
    const long double dt = static_cast<long double>(event.event_time_ns) - value.source_ns;
    if (dt >= std::numeric_limits<std::int64_t>::min() && dt <= std::numeric_limits<std::int64_t>::max())
      evidence.arrival_dt_ns = static_cast<std::int64_t>(dt);
    evidence.has_prediction_reference = event.last_prediction_reference_ns.has_value();
    evidence.last_prediction_reference_time_ns = event.last_prediction_reference_ns.value_or(0);
    evidence.evidence_source = event.source;
    vector(evidence.position, event.pre.position);
    vector(evidence.pre_velocity, event.pre.velocity); vector(evidence.post_velocity, event.post.velocity);
    evidence.spin_available = event.post.spin.usable(); evidence.spin_mode = name(event.post.spin.mode);
    evidence.spin_measurement_time_ns = event.post.spin.measurement_time_ns;
    evidence.spin_last_contact_id = event.post.spin.last_contact_id;
    evidence.spin_provenance = event.post.spin.provenance;
    evidence.spin_measured_valid = event.post.spin.usable() &&
        event.post.spin.mode == EffectiveSpinMode::MEASURED &&
        event.post.spin.measurement_time_ns == event.event_time_ns;
    evidence.pre_spin_available = event.pre.spin.usable(); evidence.pre_spin_mode = name(event.pre.spin.mode);
    evidence.pre_spin_measurement_time_ns = event.pre.spin.measurement_time_ns;
    evidence.pre_spin_last_contact_id = event.pre.spin.last_contact_id;
    evidence.pre_spin_provenance = event.pre.spin.provenance;
    if (event.pre.spin.usable()) vector(evidence.pre_omega_rad_s, *event.pre.spin.omega);
    if (event.post.spin.usable()) vector(evidence.omega_rad_s, *event.post.spin.omega);
    evidence.covariance.fill(std::numeric_limits<double>::quiet_NaN());
    evidence.linearization_quality = "unavailable";
    evidence.uncertainty_provenance = value.uncertainty_provenance;
    if (event.kind == PhysicalEventKind::TABLE &&
        (event.status == EventStatus::PREDICTED || event.status == EventStatus::CONFIRMED))
      for (const auto& other : value.observed_events)
        if (other.generation == event.generation && other.kind == PhysicalEventKind::TABLE &&
            (other.status == EventStatus::PREDICTED || other.status == EventStatus::CONFIRMED) &&
            other.event_time_ns <= event.event_time_ns) ++evidence.bounce_index;
    wire.event_evidence.push_back(std::move(evidence));
  }
  wire.published_at_ns = node_.now().nanoseconds();
  std::lock_guard<std::mutex> output_lock(output_mutex_);
  bool published = false;
  if (value.status == IncomingStatus::Revoked || value.status == IncomingStatus::Expired) {
    revocations_->publish(wire);
    published = true;
  }
  if (continuous_output_ && value.revision > last_candidate_revision_) {
    candidates_->publish(wire);
    last_candidate_revision_ = value.revision;
    published = true;
  }
  if (published) trace("candidate_published", value.source_ns, value.revision, value.token.generation, wire.status);
}

void V2IncomingRuntime::flush_statuses() {
  const auto statuses = owner_->take_statuses();
  for (const auto& status : statuses) {
    if (status.command_flight_id && revoke_)
      trace(revoke_(status) ? "command_revoked" : "revocation_failed", status.source_ns,
            status.revision, status.token.generation, status.reason);
    candidate(status);
    if (!continuous_output_) continue;
    hope_msgs::msg::FlightPredictionV2 wire;
    wire.session_id = contract_.session; wire.producer_id = producer_;
    wire.frame_id = contract_.frame; wire.calibration_id = contract_.registration;
    wire.ball_id = contract_.ball; wire.clock_domain = contract_.clock_domain;
    wire.clock_mapping_id = contract_.clock_mapping;
    wire.model_config_id = wire.config_id = status.model_config_id;
    wire.shot_id = wire.shot_generation = status.token.generation;
    wire.context_revision = status.token.context_revision;
    wire.forecast_revision = status.revision;
    wire.forecast_status = name(status.status); wire.reason = status.reason;
    wire.prediction_reference_time_ns = status.source_ns;
    wire.expires_at_ns = status.expires_ns;
    wire.plane_x = status.plane.snapshot.x_hit_m;
    wire.plane_available = status.plane.snapshot.calibrated;
    wire.plane_revision = status.plane.snapshot.generation;
    wire.generated_at_ns = wire.published_at_ns = node_.now().nanoseconds();
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    if (wire.forecast_revision > last_compact_revision_) {
      compact_->publish(wire);
      last_compact_revision_ = wire.forecast_revision;
      trace("status_published", status.source_ns, status.revision, status.token.generation, status.reason);
    } else {
      trace("status_superseded", status.source_ns, status.revision, status.token.generation, status.reason);
    }
  }
}

bool V2IncomingRuntime::forecast(IncomingQuery& query, IncomingEvaluation& result) {
  const auto metrics = owner_->counters();
  return owner_->publish(query, result, node_.now().nanoseconds(), [&](V2StrikeCandidate& value) {
    if (!continuous_output_) return true;
    if (value.source_ns && static_cast<long double>(value.expires_ns) - node_.now().nanoseconds() < 5000000.L)
      return false;
    auto wire = encode_prediction(result.forecast, query.pipeline.shot());
    wire.session_id = contract_.session; wire.producer_id = producer_;
    wire.config_id = value.model_config_id;
    wire.forecast_revision = value.revision;
    wire.input_count = metrics.offered;
    wire.superseded_count = superseded_.load();
    wire.generated_at_ns = node_.now().nanoseconds();
    wire.published_at_ns = wire.generated_at_ns;
    if (value.status == IncomingStatus::Revoked || value.status == IncomingStatus::Expired) {
      wire.target_valid = false; wire.forecast_status = name(value.status); wire.reason = value.reason;
    }
    auto points = std::move(wire.waypoints);
    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      compact_->publish(wire);
      last_compact_revision_ = wire.forecast_revision;
      trace("forecast_published", value.source_ns, value.revision, value.token.generation, value.reason);
      if (full_rate_hz_ > 0 && wire.prediction_reference_time_ns >= next_full_ns_) {
        wire.waypoints = std::move(points);
        wire.published_at_ns = node_.now().nanoseconds();
        full_->publish(wire);
        trace("trajectory_published", value.source_ns, value.revision, value.token.generation);
        next_full_ns_ = wire.prediction_reference_time_ns + std::llround(1e9 / full_rate_hz_);
      }
    }
    candidate(value);
    return true;
  });
}

void V2IncomingRuntime::continuous(const IncomingPlane& plane) {
  if (!pending_.exchange(false)) return;
  const auto state = owner_->state();
  const bool frozen = selected_plane_ && selected_generation_ == state.generation &&
      selected_plane_->selection.epoch == plane.selection.epoch;
  auto query = owner_->continuous(frozen ? *selected_plane_ : plane);
  const auto start = steady_ns();
  trace("forecast_queued", query.pipeline.shot().samples().empty() ? 0 :
        query.pipeline.shot().samples().back().source_time_ns, query.revision,
        query.pipeline.shot().generation());
  auto result = V2IncomingCoordinator::evaluate(query, node_.now().nanoseconds());
  trace("forecast_completed", result.candidate.source_ns, query.revision,
        result.candidate.token.generation, result.candidate.reason, start);
  flush_statuses();
  if (!forecast(query, result)) trace("forecast_retired", result.candidate.source_ns,
      query.revision, result.candidate.token.generation, "OBSOLETE_OR_EXPIRED");
}

StrikeTarget V2IncomingRuntime::packet(const TrajectorySnapshot& packet,
                                     const IncomingPlane& plane, BallState& state) {
  command_query_.reset(); command_evaluation_.reset();
  std::string reason;
  command_query_ = owner_->packet(packet, plane, node_.now().nanoseconds(), reason);
  if (!command_query_) {
    trace("command_rejected", packet.latest_sample()->source_time_ns, packet.packet.flight_sequence, 0, reason);
    StrikeTarget rejected; rejected.reason = reason; return rejected;
  }
  trace("command_admitted", packet.latest_sample()->source_time_ns,
        command_query_->revision, command_query_->pipeline.shot().generation());
  selected_plane_ = plane;
  selected_generation_ = command_query_->pipeline.shot().generation();
  command_evaluation_ = V2IncomingCoordinator::evaluate(*command_query_, node_.now().nanoseconds());
  const auto& belief = command_evaluation_->forecast.belief;
  state.valid = belief.initialized;
  state.position = belief.mean.head<3>(); state.velocity = belief.mean.tail<3>();
  state.source_time_s = static_cast<double>(command_evaluation_->forecast.reference_ns) * 1e-9;
  state.sample_count = belief.observation_count;
  state.sample_span_s = belief.last_measurement_time_s - belief.first_measurement_time_s;
  state.reason = command_evaluation_->candidate.reason;
  return v2_strike_target(command_evaluation_->candidate, node_.now().nanoseconds());
}

bool V2IncomingRuntime::command(const std::function<bool(V2StrikeCandidate&)>& publication) {
  if (!command_query_ || !command_evaluation_) return false;
  const auto now = node_.now().nanoseconds();
  if (static_cast<long double>(command_evaluation_->candidate.expires_ns) - now < 5000000.L)
    return false;
  const bool published = owner_->publish(*command_query_, *command_evaluation_, now,
      [&](V2StrikeCandidate& value) {
        if (static_cast<long double>(value.expires_ns) - node_.now().nanoseconds() < 5000000.L)
          return false;
        return publication(value);
      });
  const auto& value = command_evaluation_->candidate;
  trace(published ? "command_published" : "command_rejected", value.source_ns,
        published ? value.command_sequence : value.revision, value.token.generation,
        published ? "PUBLISHED" : value.reason);
  if (published) candidate(command_evaluation_->candidate);
  flush_statuses();
  return published;
}
}  // namespace hope_planner_cpp
