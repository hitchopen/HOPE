#include "hope_planner_cpp/v2_incoming_coordinator.hpp"
#include <algorithm>

namespace hope_planner_cpp {
namespace {
ShotSettings continuous_settings(ShotSettings settings) {
  settings.automatic_rearm = true;
  settings.game_legality_advisory = true;
  return settings;
}
}
V2IncomingCoordinator::V2IncomingCoordinator(ProbabilisticConfig config, ShotSettings settings,
                                           IncomingSourceContract contract)
    : contract_(std::move(contract)), pipeline_(config, continuous_settings(settings)) {
  if (contract_.session.empty() || contract_.frame.empty() || contract_.registration.empty() ||
      contract_.clock_domain.empty() || contract_.clock_mapping.empty() || contract_.ball.empty() ||
      contract_.history_capacity < 32 || contract_.history_capacity > 2048 ||
      contract_.checkpoint_capacity < 2 || contract_.checkpoint_capacity > 16 ||
      contract_.checkpoint_period_ns < 1000000 || contract_.checkpoint_period_ns > 30000000)
    throw std::invalid_argument("invalid V2 incoming source/history contract");
}

bool V2IncomingCoordinator::observe(const BallSample& sample, std::int64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.offered;
  const long double age = static_cast<long double>(now_ns) - sample.source_time_ns;
  if (!sample.position_valid || !sample.position.allFinite() || !sample.source_time_valid ||
      !sample.clock_mapping_valid || !std::isfinite(sample.clock_uncertainty_s) ||
      sample.clock_uncertainty_s < 0 || sample.clock_uncertainty_s > .001 ||
      sample.frame_id != contract_.frame || sample.calibration_id != contract_.registration ||
      sample.clock_domain != contract_.clock_domain || sample.clock_mapping_id != contract_.clock_mapping ||
      sample.ball_id != contract_.ball || age < -1000000.L ||
      age > pipeline_.settings().max_source_age_s * 1e9L) {
    if (!source_fault_) {
      revoke_locked("SOURCE_UNQUALIFIED");
      unavailable_locked("SOURCE_UNQUALIFIED", IncomingStatus::Unavailable);
    }
    source_fault_ = true;
    return false;
  }
  if (!history_.empty() && sample.source_time_ns <= history_.back().source_time_ns) return false;
  source_fault_ = false;
  source_expired_ = false;
  ++counters_.qualified;
  const auto generation = pipeline_.shot().generation(), revision = pipeline_.shot().revision();
  const bool accepted = pipeline_.observe(sample);
  accepted ? ++counters_.tracker_accepted : ++counters_.tracker_rejected;
  history_.push_back(sample);
  while (history_.size() > contract_.history_capacity) history_.pop_front();
  if (pipeline_.shot().generation() != generation) {
    revoke_locked("PHYSICAL_FLIGHT_CHANGED");
    checkpoints_.clear();
  } else if (pipeline_.shot().revision() != revision) {
    int receiver_bounces = 0;
    bool terminal = false;
    for (const auto& event : pipeline_.shot().events()) {
      if (event.status != EventStatus::CONFIRMED) continue;
      if (event.kind == PhysicalEventKind::TABLE &&
          event.pre.position.x() < pipeline_.config().table.net_x && event.pre.velocity.x() < -.5)
        ++receiver_bounces;
      terminal = terminal || event.kind == PhysicalEventKind::NET_COLLISION ||
          event.kind == PhysicalEventKind::GROUND;
    }
    if (terminal || receiver_bounces > 1) revoke_locked("PHYSICAL_EVIDENCE_INVALIDATED");
    else ++epoch_;  // Recheck workers; corroboration alone does not revoke an accepted shot.
  }
  if (checkpoints_.empty() || static_cast<long double>(sample.source_time_ns) -
      checkpoints_.back().source_ns >= contract_.checkpoint_period_ns) {
    checkpoints_.push_back({sample.source_time_ns, pipeline_});
    while (checkpoints_.size() > contract_.checkpoint_capacity) checkpoints_.pop_front();
  }
  return accepted;
}

IncomingQuery V2IncomingCoordinator::query(FlightPipeline pipeline, const IncomingPlane& plane,
                                         bool command) {
  ++counters_.queries;
  latest_plane_ = plane;
  return {std::move(pipeline), plane, epoch_, ++revision_, command, {}};
}
IncomingQuery V2IncomingCoordinator::continuous(const IncomingPlane& plane) {
  std::lock_guard<std::mutex> lock(mutex_);
  return query(pipeline_, plane, false);
}

bool V2IncomingCoordinator::same_sample(const BallSample& a, const BallSample& b) {
  if (a.source_time_ns != b.source_time_ns || a.position != b.position) return false;
  // Legacy PoseArray packets carry numeric quaternions without an authority
  // flag. Only the typed source can qualify orientation; a placeholder cannot
  // promote unknown spin into measured zero or make a position-only join fail.
  if (!a.orientation_valid) return true;
  if (!b.orientation_valid) return false;
  // A quaternion and its negation represent the same acquisition orientation.
  return std::abs(a.orientation.normalized().coeffs().dot(b.orientation.normalized().coeffs())) >= 1 - 1e-12;
}

std::optional<IncomingQuery> V2IncomingCoordinator::packet(
    const TrajectorySnapshot& packet, const IncomingPlane& plane,
    std::int64_t now_ns, std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  reason = "PACKET_IDENTITY_UNQUALIFIED";
  if (!packet.packet.present || packet.packet.session_id != contract_.session ||
      packet.packet.producer_instance_id.empty() || packet.packet.payload_hash.empty() ||
      packet.packet.frame_id != contract_.frame || !packet.packet.trajectory_epoch ||
      !packet.packet.flight_sequence || !packet.one_shot.commit_due ||
      packet.sample_count == 0 || packet.sample_count > packet.samples.size() ||
      plane.tentative || !plane.selection.valid || !plane.selection.epoch || !plane.selection.flight)
    return {};
  if ((!packet_producer_.empty() && packet.packet.producer_instance_id != packet_producer_) ||
      packet.packet.flight_sequence <= last_packet_sequence_) {
    reason = "PACKET_PRODUCER_OR_SEQUENCE_MISMATCH";
    return {};
  }
  const auto cutoff = packet.samples[packet.sample_count - 1].source_time_ns;
  const long double age = static_cast<long double>(now_ns) - cutoff;
  if (source_fault_ || status_overflow_ || age < -1000000.L ||
      age > pipeline_.settings().max_source_age_s * 1e9L) {
    reason = "PACKET_SOURCE_EXPIRED_OR_UNQUALIFIED";
    return {};
  }
  auto first = std::find_if(history_.begin(), history_.end(), [&](const BallSample& sample) {
    return sample.source_time_ns == packet.samples[0].source_time_ns;
  });
  for (std::size_t i = 0; i < packet.sample_count; ++i) {
    if (first == history_.end() || !same_sample(*first, packet.samples[i])) {
      reason = "PACKET_OBSERVATION_PREFIX_MISMATCH";
      return {};
    }
    ++first;
  }
  auto checkpoint = checkpoints_.rend();
  for (auto it = checkpoints_.rbegin(); it != checkpoints_.rend(); ++it)
    if (it->source_ns <= cutoff) { checkpoint = it; break; }
  if (checkpoint == checkpoints_.rend() || history_.empty() ||
      checkpoint->source_ns < history_.front().source_time_ns) {
    reason = "PACKET_CHECKPOINT_UNAVAILABLE";
    return {};
  }
  auto frozen = checkpoint->pipeline;
  for (const auto& sample : history_)
    if (sample.source_time_ns > checkpoint->source_ns && sample.source_time_ns <= cutoff)
      frozen.observe(sample);
  if (!frozen.shot().active() || frozen.shot().samples().empty() ||
      frozen.shot().samples().back().source_time_ns != cutoff ||
      frozen.shot().generation() != pipeline_.shot().generation() ||
      frozen.shot().revision() != pipeline_.shot().revision()) {
    reason = "PACKET_PHYSICAL_GENERATION_RETIRED";
    return {};
  }
  const auto flight = std::make_pair(plane.selection.epoch, frozen.shot().generation());
  if (std::find(claimed_flights_.begin(), claimed_flights_.end(), flight) != claimed_flights_.end()) {
    reason = "PHYSICAL_FLIGHT_ALREADY_CLAIMED";
    return {};
  }
  claimed_flights_.push_back(flight);
  while (claimed_flights_.size() > 128) claimed_flights_.pop_front();
  packet_producer_ = packet.packet.producer_instance_id;
  last_packet_sequence_ = packet.packet.flight_sequence;
  auto result = query(std::move(frozen), plane, true);
  result.packet_identity = packet.packet.session_id + ":" + packet_producer_ + ":" +
      std::to_string(packet.packet.trajectory_epoch) + ":" +
      std::to_string(packet.packet.flight_sequence) + ":" + packet.packet.payload_hash;
  reason = "PACKET_RECONCILED";
  return result;
}

IncomingEvaluation V2IncomingCoordinator::evaluate(IncomingQuery& query, std::int64_t now_ns) {
  IncomingEvaluation result;
  result.forecast = query.pipeline.forecast(query.plane.snapshot, now_ns);
  result.candidate = evaluate_v2_strike(result.forecast, query.pipeline.shot().export_snapshot(),
                                      query.pipeline.config(), query.plane, now_ns);
  result.candidate.semantic_epoch = query.semantic_epoch;
  result.candidate.revision = query.revision;
  return result;
}

bool V2IncomingCoordinator::current(const IncomingQuery& query, std::int64_t now_ns) const {
  if (reference_received_ && ((query.command && !reference_valid_) ||
      (query.plane.snapshot.calibrated && query.plane.selection.epoch != home_epoch_))) return false;
  if (source_fault_ || status_overflow_ || query.semantic_epoch != epoch_ ||
      query.pipeline.shot().generation() != pipeline_.shot().generation() ||
      query.pipeline.shot().revision() != pipeline_.shot().revision()) return false;
  if (query.pipeline.shot().samples().empty()) return !query.command;
  const long double age = static_cast<long double>(now_ns) - query.pipeline.shot().samples().back().source_time_ns;
  return age >= -1000000.L && age <= pipeline_.settings().max_source_age_s * 1e9L;
}

bool V2IncomingCoordinator::publish(const IncomingQuery& query, IncomingEvaluation& result,
    std::int64_t now_ns, const std::function<bool(V2StrikeCandidate&)>& publication) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.completed;
  if (!current(query, now_ns) || (!query.command && query.revision <= published_revision_)) {
    ++counters_.obsolete;
    return false;
  }
  auto& candidate = result.candidate;
  candidate.session_id = contract_.session;
  if (reference_received_ && !reference_valid_) {
    candidate.command_eligible = false;
    candidate.status = IncomingStatus::Unavailable;
    candidate.reason = "REFERENCE_UNAVAILABLE";
  }
  if (query.command && (query.revision <= last_command_revision_ ||
      !v2_strike_target(candidate, now_ns).valid)) return false;
  if (!query.command && !candidate.strike_contact_supported &&
      ((latest_candidate_ && latest_candidate_->strike_contact_supported) || command_candidate_)) {
    revoke_locked(candidate.reason);
    candidate.status = IncomingStatus::Revoked;
    candidate.semantic_epoch = epoch_;
    candidate.revision = ++revision_;
  }
  const auto flight = std::make_pair(candidate.plane.selection.epoch, candidate.token.generation);
  if (!query.command && std::find(claimed_flights_.begin(), claimed_flights_.end(), flight) != claimed_flights_.end()) {
    candidate.command_eligible = false;
    if (candidate.strike_contact_supported) candidate.reason = "ONE_SHOT_ALREADY_COMMITTED";
  }
  if (!publication(candidate)) return false;
  if (query.command) {
    command_candidate_ = candidate;
    last_command_revision_ = query.revision;
    ++counters_.commands;
  } else {
    published_revision_ = candidate.revision;
    latest_candidate_ = candidate;
  }
  return true;
}

void V2IncomingCoordinator::revoke_locked(const std::string& reason) {
  ++epoch_;
  auto revoke = [&](std::optional<V2StrikeCandidate>& previous) {
    if (!previous || !previous->strike_contact_supported) return;
    auto status = *previous;
    if (status.token.generation == pipeline_.shot().generation())
      status.observed_events = pipeline_.shot().events();
    status.status = IncomingStatus::Revoked;
    status.command_eligible = status.strike_contact_supported = false;
    status.reason = reason;
    status.semantic_epoch = epoch_;
    status.revokes_revision = previous->revision;
    status.revision = ++revision_;
    published_revision_ = status.revision;
    if (statuses_.size() >= 64) status_overflow_ = true;
    else statuses_.push_back(std::move(status));
    previous.reset();
    ++counters_.revoked;
  };
  revoke(command_candidate_);
  revoke(latest_candidate_);
}
void V2IncomingCoordinator::revoke(const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  revoke_locked(reason);
}
void V2IncomingCoordinator::unavailable_locked(const std::string& reason, IncomingStatus state) {
  V2StrikeCandidate status;
  if (latest_plane_) status.plane = *latest_plane_;
  status.session_id = contract_.session;
  status.model_config_id = flight_model_config_id(pipeline_.config(), pipeline_.settings(), contract_.frame);
  status.status = state; status.reason = reason;
  status.semantic_epoch = epoch_; status.revision = ++revision_;
  status.token.generation = pipeline_.shot().generation();
  status.token.context_revision = pipeline_.shot().revision();
  if (!history_.empty()) status.source_ns = history_.back().source_time_ns;
  status.token.source_ns = status.source_ns;
  status.expires_ns = status.source_ns + std::llround(pipeline_.settings().max_source_age_s * 1e9);
  published_revision_ = status.revision;
  if (statuses_.size() >= 64) status_overflow_ = true;
  else statuses_.push_back(std::move(status));
}
bool V2IncomingCoordinator::update_reference(
    const std::function<std::pair<std::uint64_t, bool>()>& update) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto [epoch, valid] = update();
  const bool changed = !reference_received_ || epoch != home_epoch_ || valid != reference_valid_;
  if (epoch != home_epoch_) revoke_locked("HOME_CHANGED");
  else if (reference_valid_ && !valid) revoke_locked("REFERENCE_UNAVAILABLE");
  reference_received_ = true;
  home_epoch_ = epoch;
  reference_valid_ = valid;
  return changed;
}
void V2IncomingCoordinator::expire(std::int64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  // An accepted one-shot command keeps its original source timestamp. Its
  // flight may remain supported by newer authoritative observations; do not
  // impose a 30 ms execution lease on the existing absolute strike deadline.
  if (!history_.empty() && !source_expired_ &&
      static_cast<long double>(now_ns) - history_.back().source_time_ns >
      pipeline_.settings().max_source_age_s * 1e9L) {
    revoke_locked("SOURCE_EXPIRED");
    source_expired_ = true;
    unavailable_locked("SOURCE_EXPIRED", IncomingStatus::Expired);
  } else if (command_candidate_ && command_candidate_->contact &&
           now_ns >= command_candidate_->contact->arrival_ns)
    revoke_locked("CONTACT_EXPIRED");
}
std::vector<V2StrikeCandidate> V2IncomingCoordinator::take_statuses() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<V2StrikeCandidate> result(statuses_.begin(), statuses_.end());
  statuses_.clear();
  // Overflow remains latched: lost revocations require an explicit new owner/session.
  return result;
}
IncomingCounters V2IncomingCoordinator::counters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = counters_;
  result.history = history_.size(); result.checkpoints = checkpoints_.size();
  result.latest_source_ns = history_.empty() ? 0 : history_.back().source_time_ns;
  return result;
}
ShotSnapshot V2IncomingCoordinator::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pipeline_.shot().export_snapshot();
}
}  // namespace hope_planner_cpp
