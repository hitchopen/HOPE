#include "hope_planner_cpp/ball_flight_state_machine.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace hope_planner_cpp {
BallFlightStateMachine::BallFlightStateMachine(ProbabilisticConfig c)
    : config_(std::move(c)) {
  config_.validate();
}
void BallFlightStateMachine::begin_rally(std::uint64_t id, bool serving) {
  const auto epoch = state_.trajectory_epoch;
  state_ = FlightState{};
  history_ = BounceContext{};
  complete_history_pending_ = verified_hint_pending_ = false;
  state_.trajectory_epoch = epoch;
  state_.rally_id = id;
  state_.rally_context =
      serving ? RallyContext::kNormalReturn : RallyContext::kServeReturn;
  state_.planner_state = serving ? PlannerBallState::kServe
                                 : PlannerBallState::kOutgoing;
  state_.expected_bounces = serving ? config_.normal_expected_bounces
                                    : config_.serve_expected_bounces;
  hint_pending_ = false;
  track_active_ = false;
  first_incoming_seen_ = false;
  return_departed_ = false;
  direction_.clear();
  bounce_samples_.clear();
  incoming_confirmations_ = outgoing_confirmations_ = 0;
}
void BallFlightStateMachine::set_bounce_history_hint(int count) {
  if (count < 0)
    throw std::invalid_argument("negative bounce hint");
  state_.prior_bounce_hint = count;
  history_.count_only_hint = count;
  if(count>0)history_.continuity=HistoryContinuity::kUnknown;
  hint_pending_ = !track_active_;
}
void BallFlightStateMachine::new_track() {
  if (!hint_pending_)
    state_.prior_bounce_hint = -1;
  hint_pending_ = false;
  if(!complete_history_pending_){
    if(verified_hint_pending_)history_.events.erase(std::remove_if(history_.events.begin(),history_.events.end(),[](const auto&e){return e.provenance!=ContactProvenance::kExternalSideVerified;}),history_.events.end());
    else history_.events.clear();
  }
  history_.continuity = complete_history_pending_ ? HistoryContinuity::kComplete : HistoryContinuity::kUnknown;
  complete_history_pending_ = verified_hint_pending_ = false;
  track_active_ = true;
  ++state_.trajectory_epoch;
  state_.revision_id = 0;
  state_.incoming_motion_confidence = .5;
  state_.motion_state = BallMotionState::kUncertain;
  state_.planner_state =
      state_.rally_context == RallyContext::kNormalReturn &&
              !first_incoming_seen_
          ? PlannerBallState::kServe
          : PlannerBallState::kOutgoing;
  state_.active_incoming = false;
  state_.observed_bounces = 0;
  state_.last_observed_bounce_time_s = -1;
  direction_.clear();
  bounce_samples_.clear();
  incoming_confirmations_ = outgoing_confirmations_ = 0;
}
void BallFlightStateMachine::update(const BallSample *s,
                                    const StateBelief &belief,
                                    const TrackerUpdate &update) {
  if (!update.changed)
    return;
  if (update.new_track)
    new_track();
  if (update.terminated) {
    track_active_ = false;
    history_.continuity = HistoryContinuity::kUnknown;
    state_.incoming_motion_confidence = .5;
    state_.motion_state = BallMotionState::kUncertain;
    state_.planner_state = PlannerBallState::kOutgoing;
    state_.active_incoming = false;
    state_.ready_stand_requested = true;
    direction_.clear();
    bounce_samples_.clear();
    ++state_.revision_id;
    return;
  }
  if(belief.last_measurement_gap_s>config_.gap_warning_s && history_.continuity==HistoryContinuity::kComplete)
    history_.continuity=HistoryContinuity::kIncomplete;
  if (s) {
    direction_.push_back(*s);
    while (direction_.size() > 3 &&
           s->source_time_s - direction_.front().source_time_s >
               config_.direction_window_s)
      direction_.pop_front();
    if (direction_.size() >= 3) {
      const double origin = direction_.front().source_time_s;
      double mt = 0, mx = 0;
      for (const auto &v : direction_) {
        mt += v.source_time_s - origin;
        mx += v.position.x();
      }
      mt /= direction_.size();
      mx /= direction_.size();
      double tt = 0, tx = 0;
      for (const auto &v : direction_) {
        double t = v.source_time_s - origin - mt;
        tt += t * t;
        tx += t * (v.position.x() - mx);
      }
      if (tt > 1e-12) {
        const double vx = tx / tt;
        double residual = 0;
        for (const auto &v : direction_) {
          double d = v.position.x() - mx - vx * (v.source_time_s - origin - mt);
          residual += d * d;
        }
        const double variance = std::max(config_.measurement_std_m.x() *
                                             config_.measurement_std_m.x(),
                                         residual / (direction_.size() - 2));
        const double sigma = std::sqrt(variance / tt);
        state_.incoming_motion_confidence =
            .5 * std::erfc((vx + config_.direction_speed_mps) /
                           (std::sqrt(2.) * sigma));
        incoming_confirmations_ =
            vx + config_.direction_sigma * sigma < -config_.direction_speed_mps
                ? incoming_confirmations_ + 1
                : 0;
        outgoing_confirmations_ =
            vx - config_.direction_sigma * sigma > config_.direction_speed_mps
                ? outgoing_confirmations_ + 1
                : 0;
        if (incoming_confirmations_ >= config_.direction_confirmations &&
            state_.motion_state != BallMotionState::kIncoming) {
          if (return_departed_) {
            history_ = BounceContext{};
            ++state_.trajectory_epoch;
            state_.revision_id = 0;
            state_.observed_bounces = 0;
            state_.prior_bounce_hint = -1;
            state_.last_observed_bounce_time_s = -1;
            bounce_samples_.clear();
            if (state_.rally_context != RallyContext::kUnknown) {
              state_.rally_context = RallyContext::kNormalReturn;
              state_.expected_bounces = config_.normal_expected_bounces;
            }
          }
          state_.motion_state = BallMotionState::kIncoming;
          state_.planner_state =
              state_.rally_context == RallyContext::kServeReturn
                  ? PlannerBallState::kReturnServe
                  : PlannerBallState::kIncoming;
          first_incoming_seen_ = true;
          return_departed_ = false;
        }
        if (outgoing_confirmations_ >= config_.direction_confirmations) {
          state_.motion_state = BallMotionState::kOutgoing;
          state_.planner_state =
              state_.rally_context == RallyContext::kNormalReturn &&
                      !first_incoming_seen_
                  ? PlannerBallState::kServe
                  : PlannerBallState::kOutgoing;
          if (first_incoming_seen_)
            return_departed_ = true;
        }
      }
    }
    bounce_samples_.push_back(*s);
    while (bounce_samples_.size() > 3 &&
           s->source_time_s - bounce_samples_.front().source_time_s > .08)
      bounce_samples_.pop_front();
    // A causal reversal can precede the five-sample confirmation window.
    // Record it as unconfirmed so zero confirmed contacts is not mistaken for
    // proof of no prior bounce during this short observation interval.
    if (bounce_samples_.size() >= 3) {
      auto low =
          std::min_element(bounce_samples_.begin(), bounce_samples_.end(),
                           [](const auto &a, const auto &b) {
                             return a.position.z() < b.position.z();
                           });
      const auto before = std::distance(bounce_samples_.begin(), low),
                 after = std::distance(low, bounce_samples_.end()) - 1;
      const double excursion = std::max(.001, config_.measurement_std_m.z());
      if (before >= 1 && after >= 1 &&
          low->position.z() < config_.physics.radius + .06 &&
          low->position.z() >= config_.min_tracking_z_m &&
          low->position.x() >= 0 && low->position.x() <= config_.table.length &&
          low->position.y() >= config_.table.y_max - config_.table.width &&
          low->position.y() <= config_.table.y_max &&
          bounce_samples_.front().position.z() - low->position.z() >
              excursion &&
          s->position.z() - low->position.z() > excursion) {
        bool present = false;
        for (const auto &e : history_.events)
          if (std::abs(e.source_time_s - low->source_time_s) < .08) {
            present = true;
            break;
          }
        if (!present) {
          ContactHistoryEvent e;
          e.source_time_ns = low->source_time_ns;
          e.source_time_s = low->source_time_s;
          e.position = low->position;
          e.side = contact_side(e.position.x(), config_, ReturnRegionPolicy{});
          e.provenance = ContactProvenance::kInferredUnconfirmed;
          e.confidence = .5;
          e.epoch = state_.trajectory_epoch;
          history_.events.push_back(e);
          if (history_.events.size() > 16) {
            history_.events.erase(history_.events.begin());
            history_.continuity = HistoryContinuity::kUnknown;
          }
        }
      }
    }
    if (bounce_samples_.size() >= 5) {
      auto low =
          std::min_element(bounce_samples_.begin(), bounce_samples_.end(),
                           [](const auto &a, const auto &b) {
                             return a.position.z() < b.position.z();
                           });
      const auto before = std::distance(bounce_samples_.begin(), low),
                 after = std::distance(low, bounce_samples_.end()) - 1;
      const double excursion =
          std::max(.001, 2 * config_.measurement_std_m.z());
      if (before >= 2 && after >= 2 &&
          low->position.z() < config_.physics.radius + .06 &&
          low->position.z() >= config_.min_tracking_z_m &&
          low->position.x() >= 0 && low->position.x() <= config_.table.length &&
          low->position.y() >= config_.table.y_max - config_.table.width &&
          low->position.y() <= config_.table.y_max &&
          bounce_samples_.front().position.z() - low->position.z() >
              excursion &&
          s->position.z() - low->position.z() > excursion &&
          low->source_time_s - state_.last_observed_bounce_time_s > .08) {
        ContactHistoryEvent contact;
        contact.source_time_ns = low->source_time_ns;
        contact.source_time_s = low->source_time_s;
        contact.position = low->position;
        contact.side =
            contact_side(contact.position.x(), config_, ReturnRegionPolicy{});
        contact.provenance = ContactProvenance::kObservedConfirmed;
        contact.epoch = state_.trajectory_epoch;
        // Do not add a second copy of an externally verified same contact.
        bool duplicate = false;
        for (auto &e : history_.events)
          if (std::abs(e.source_time_s - contact.source_time_s) < .08 &&
              (e.position - contact.position).norm() < .10) {
            if (e.side != contact.side)
              history_.conflict = true;
            e = contact;
            duplicate = true;
            break;
          }
        if (!duplicate)
          history_.events.push_back(contact);
        if (history_.events.size() > 16) {
          history_.events.erase(history_.events.begin());
          history_.continuity = HistoryContinuity::kUnknown;
        }
        ++state_.observed_bounces;
        state_.last_observed_bounce_time_s = low->source_time_s;
      }
    }
  }
  // Classification is latched through uninformative samples/short gaps; height
  // disables incoming activity but does not discard the flexible general track.
  state_.active_incoming = belief.initialized &&
                           state_.motion_state == BallMotionState::kIncoming &&
                           belief.mean[2] >= config_.min_tracking_z_m &&
                           (!s || s->position.z() >= config_.min_tracking_z_m);
  state_.ready_stand_requested = !state_.active_incoming;
  ++state_.revision_id;
}
} // namespace hope_planner_cpp

namespace hope_planner_cpp {
void BallFlightStateMachine::set_history_continuity(HistoryContinuity c) {
  history_.continuity = c;
  complete_history_pending_ =
      !track_active_ && c == HistoryContinuity::kComplete;
}
void BallFlightStateMachine::set_return_history_count_hint(int count) {
  if (count < -1)
    throw std::invalid_argument("invalid count-only history hint");
  if (count >= 0)
    set_bounce_history_hint(count);
  else {
    state_.prior_bounce_hint = -1;
    history_.count_only_hint = -1;
    history_.continuity = HistoryContinuity::kUnknown;
    complete_history_pending_ = false;
  }
}
void BallFlightStateMachine::add_verified_contact(
    const ContactHistoryEvent &contact) {
  if (contact.provenance != ContactProvenance::kExternalSideVerified ||
      contact.side == ContactSide::kAmbiguous)
    throw std::invalid_argument("side-verified contact required");
  if (!track_active_)
    verified_hint_pending_ = true;
  for (const auto &e : history_.events)
    if (std::abs(e.source_time_s - contact.source_time_s) < .08) {
      if (e.side != contact.side)
        history_.conflict = true;
      return;
    }
  if (!std::isfinite(contact.source_time_s) || !contact.position.allFinite())
    throw std::invalid_argument("invalid verified contact");
  history_.events.push_back(contact);
  if (history_.events.size() > 16) {
    history_.events.erase(history_.events.begin());
    history_.continuity = HistoryContinuity::kUnknown;
  }
  std::sort(history_.events.begin(), history_.events.end(),
            [](const auto &a, const auto &b) {
              return a.source_time_s < b.source_time_s;
            });
}
} // namespace hope_planner_cpp
