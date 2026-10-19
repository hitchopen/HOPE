#include "hope_planner_cpp/strike_target_selector.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace hope_planner_cpp {
bool strike_target_selector_hit_y_allowed(double y,double min_y,double max_y) {
  return std::isfinite(y) && std::isfinite(min_y) && std::isfinite(max_y) &&
    min_y<max_y && y>=min_y && y<=max_y;
}
namespace {
bool fresh(std::int64_t source,std::int64_t now,std::int64_t age) {
  return source>0 && source<=now && static_cast<long double>(now)-source<=age;
}
bool add_ns(std::int64_t a,std::int64_t b,std::int64_t& result) {
  if((b>0 && a>INT64_MAX-b) || (b<0 && a<INT64_MIN-b))return false;
  result=a+b;return true;
}
bool absolute_ns(std::int64_t origin,double local,std::int64_t& result) {
  const long double delta=std::round(static_cast<long double>(local)*1e9L);
  if(!std::isfinite(delta) || delta<INT64_MIN || delta>INT64_MAX)return false;
  return add_ns(origin,static_cast<std::int64_t>(delta),result);
}
bool same_owner(const hope_msgs::msg::BallForecast& a,const hope_msgs::msg::BallForecast& b) {
  return a.publisher_instance_id==b.publisher_instance_id &&
    a.hit_plane_generation==b.hit_plane_generation && a.x_hit_m==b.x_hit_m &&
    a.x_hit_calibrated==b.x_hit_calibrated && a.known_test_plane==b.known_test_plane &&
    a.source_clock_registered==b.source_clock_registered &&
    a.calibration_id==b.calibration_id && a.world_frame_id==b.world_frame_id &&
    a.frame_registration_id==b.frame_registration_id && a.identity.frame_id==b.identity.frame_id &&
    a.source_clock_domain==b.source_clock_domain &&
    a.identity.return_region_config_id==b.identity.return_region_config_id;
}
hope_msgs::msg::BallForecast compact(const hope_msgs::msg::BallForecast& m) {
  auto copy=m;copy.waypoints.clear();copy.physics_events.clear();return copy;
}
bool identity_valid(const hope_msgs::msg::BallForecast& m,const StrikeTargetSelectorConfig& c) {
  return m.schema=="ball-forecast-v1" && !m.publisher_instance_id.empty() &&
    !m.identity.return_region_config_id.empty() && m.hit_plane_generation && m.event_sequence &&
    m.identity.frame_id==c.frame && m.frame_registration_id==c.registration &&
    m.source_clock_registered && m.source_clock_domain==c.clock && m.x_hit_calibrated &&
    (m.known_test_plane ? c.synthetic : (m.calibration_id && m.world_frame_id)) &&
    std::isfinite(m.x_hit_m) && m.x_hit_m>=-.30 && m.x_hit_m<=.30;
}
}
StrikeTargetSelector::StrikeTargetSelector(StrikeTargetSelectorConfig c)
    :config_(std::move(c)),incoming_(config_.incoming),plane_(config_.frozen_plane) {
  if(config_.frame.empty() || config_.registration.empty() ||
     (config_.clock!="ros_system" && !(config_.synthetic && config_.clock=="ros_sim")) ||
     config_.max_age_ns<=0 || config_.max_age_ns>30000000 || config_.grace_ns<0 ||
     config_.grace_ns>30000000 || config_.min_lead_ns<0 ||
     !std::isfinite(config_.incoming.commit_delay_s) || config_.incoming.commit_delay_s<0 || config_.incoming.commit_delay_s>.2)
    throw std::invalid_argument("invalid strike_target_selector registration/timing");
  if(plane_ && (!plane_->generation || !std::isfinite(plane_->x) || std::abs(plane_->x)>.30 ||
      (plane_->synthetic ? !config_.synthetic : (!plane_->world || !plane_->calibration))))
    throw std::invalid_argument("invalid explicitly registered frozen fallback plane");
}
StrikeTargetSelectorCandidate StrikeTargetSelector::extract(const hope_msgs::msg::BallForecast& m,
    const StrikeTargetSelectorConfig& c,std::int64_t now) {
  // Expiry and uncertainty are advisory for a saved same-flight crossing.
  StrikeTargetSelectorCandidate out;
  auto reject=[&](const char* reason){out.strike.reason=reason;return out;};
  if(!identity_valid(m,c))return reject("unregistered_forecast_identity");
  if(m.kind!=m.FORECAST || !m.identity.revision_id || !m.crossing_exists)
    return reject("crossing_unavailable");
  const auto& h=m.hit_plane;std::int64_t hit=0;
  if(!h.absolute_time_valid || h.dt_ns<=0 || m.identity.source_time_ns<=0 ||
     m.identity.source_time_ns>now || !add_ns(m.identity.source_time_ns,h.dt_ns,hit) || hit!=h.absolute_time_ns ||
     m.last_measurement_time_ns>m.identity.source_time_ns)
    return reject("inconsistent_hit_time");
  for(double v:h.pre_state)if(!std::isfinite(v))return reject("nonfinite_crossing");
  if(h.pre_state[3]>=0 || h.pre_state[2]<=0 || std::abs(h.pre_state[0]-m.x_hit_m)>1e-8)
    return reject("invalid_crossing_geometry");
  if(hit<=now)return reject("past_strike_deadline");
  if(m.physics_event_sequence.size()==m.physics_event_dt_ns.size())
    for(std::size_t i=0;i<m.physics_event_sequence.size();++i)
      if(m.physics_event_sequence[i]=="ground" && m.physics_event_dt_ns[i]>0 && m.physics_event_dt_ns[i]<h.dt_ns)
        return reject("ground_before_hit_plane");
  out.strike.ball_position=Vec3(h.pre_state[0],h.pre_state[1],h.pre_state[2]);
  out.strike.ball_velocity=Vec3(h.pre_state[3],h.pre_state[4],h.pre_state[5]);
  out.strike.strike_source_time_s=static_cast<double>(hit)*1e-9;
  out.strike.valid=true;out.strike.reason="selected_forecast_crossing";
  out.absolute_hit_ns=hit;out.revision=m.identity.revision_id;out.linearization_quality=h.linearization_quality;
  return out;
}
void StrikeTargetSelector::fence(const std::string& reason) {
  halted_=true;cache_.clear();best_crossing_.reset();states_.clear();contacts_.clear();complete_=false;reason_=reason;
  // Do not silently cancel a flight already detected by the segmenter.
  if(flight_ && !decided_ && !pending_) {pending_=true;commit_=last_pose_;deadline_=0;}
}
bool StrikeTargetSelector::establish_owner(const hope_msgs::msg::BallForecast& m) {
  if(retired_.count(m.publisher_instance_id))return false;
  if(!identity_valid(m,config_)) {fence("unregistered_forecast_identity");return false;}
  if(owner_ && !same_owner(*owner_,m)) {
    if(m.publisher_instance_id!=owner_->publisher_instance_id) {
      if(retired_.size()<8)retired_.insert(owner_->publisher_instance_id);else retirement_exhausted_=true;
    }
    fence("forecast_owner_or_plane_changed");return false;
  }
  SelectorFrozenPlane p{m.x_hit_m,m.hit_plane_generation,m.world_frame_id,m.calibration_id,m.known_test_plane};
  if(plane_ && (plane_->x!=p.x || plane_->generation!=p.generation || plane_->synthetic!=p.synthetic ||
       plane_->world!=p.world || plane_->calibration!=p.calibration)) {
    fence("forecast_plane_registration_changed");return false;
  }
  plane_=p;if(!owner_)owner_=compact(m);return true;
}
void StrikeTargetSelector::schedule(std::int64_t net,std::int64_t now,std::int64_t steady) {
  if(pending_ || decided_)return;
  net_=net;
  std::int64_t final_due=0;
  if(!add_ns(net,std::llround(config_.incoming.commit_delay_s*1e9),commit_) ||
     !add_ns(commit_,config_.grace_ns,final_due) ||
     !add_ns(steady,std::max<std::int64_t>(0,final_due-now),deadline_)) {fence("commit_time_overflow");return;}
  pending_=true;reason_="net_crossed";
  // A post-cutoff estimate must not overwrite the durable causal register.
  best_crossing_.reset();
  for(const auto& e:cache_)if(e.source_ns>=start_ && e.source_ns<=commit_ &&
      (!best_crossing_ || e.source_ns>best_crossing_->source_ns))best_crossing_=e;
}
void StrikeTargetSelector::observe(BallSample s,const std::string& frame,std::int64_t now,std::int64_t steady) {
  if(frame!=config_.frame || !s.position.allFinite() || s.source_time_ns<=0 || s.source_time_ns>now) {fence("invalid_pose");return;}
  if(last_pose_ && s.source_time_ns<=last_pose_) {
    if(s.source_time_ns<last_pose_-1000000)fence("pose_clock_discontinuity");
    return;
  }
  if(halted_)return;
  if(!origin_)origin_=s.source_time_ns;
  last_pose_=s.source_time_ns;last_position_=s.position;
  s.source_time_s=static_cast<double>(s.source_time_ns-origin_)*1e-9;
  samples_.push_back(s);if(samples_.size()>kMaxEstimatorSamples)samples_.pop_front();
  const auto u=incoming_.observe(s);
  if(u.source_epoch_reset) {fence("pose_source_gap");return;}
  if(u.incoming_started) {
    if(flight_ && !decided_) {auto old=decision_metadata(now);old.reason="incoming_replaced_before_commit";terminal_=old;}
    ++flight_;start_=u.segment_start_ns;
    pending_=decided_=false;net_=commit_=0;late_acquisition_=false;best_crossing_.reset();reason_="incoming_started";
    while(!samples_.empty() && samples_.front().source_time_ns<start_)samples_.pop_front();
    cache_.erase(std::remove_if(cache_.begin(),cache_.end(),[&](const auto& e){return e.source_ns<start_;}),cache_.end());
    states_.erase(std::remove_if(states_.begin(),states_.end(),[&](const auto& e){return e.source_ns<start_;}),states_.end());
    for(auto& e:cache_) {e.flight_id=flight_;if(!best_crossing_ || e.source_ns>best_crossing_->source_ns)best_crossing_=e;}
  }
  if(u.net_crossed && !decided_ && !pending_) {
    late_acquisition_=u.late_acquisition;schedule(u.net_cross_ns,now,steady);
  }
}
bool StrikeTargetSelector::authorized_reset_ack(const hope_msgs::msg::BallForecast& m) const {
  return m.kind==m.INVALIDATION && m.reason=="explicit_reset" && !m.publisher_instance_id.empty() &&
    m.identity.frame_id==config_.frame && m.frame_registration_id==config_.registration &&
    m.source_clock_registered && m.source_clock_domain==config_.clock &&
    std::find(authorized_reset_sources_.begin(),authorized_reset_sources_.end(),m.identity.source_time_ns)!=authorized_reset_sources_.end();
}
bool StrikeTargetSelector::forecast(const hope_msgs::msg::BallForecast& m,std::int64_t receipt) {
  if(halted_ || retired_.count(m.publisher_instance_id))return false;
  // Exact authorized request stamps make acknowledgment delivery idempotent. An
  // acknowledgment never resets state or rearms a physical flight.
  if(authorized_reset_ack(m))return false;
  if(owner_ && m.publisher_instance_id==owner_->publisher_instance_id && m.event_sequence<=event_)return false;
  if(!establish_owner(m))return false;
  if(m.kind==m.INVALIDATION) {
    event_=m.event_sequence;
    const bool ordinary=m.reason=="initial_observation" || m.reason=="tracking_gap_restart" ||
      m.reason=="reacquired_after_rejections" || m.reason=="tracking_timeout" ||
      m.reason=="forecast_expired" || m.reason=="source_age_expired" || m.reason=="accepted_measurement_expired" ||
      m.reason=="compact_prediction_expired" || m.reason=="trajectory_horizon_expired" || m.reason=="return_ineligible";
    if(!ordinary)fence("forecast_invalidated:"+m.reason);else reason_="retained_after:"+m.reason;
    return false;
  }
  if(m.kind!=m.FORECAST)return false;
  if(last_forecast_ && m.identity.source_time_ns<last_forecast_-1000000) {fence("forecast_clock_discontinuity");return false;}
  if(m.event_sequence<=event_ || m.identity.source_time_ns<=last_forecast_ ||
      m.identity.trajectory_epoch<ckf_epoch_ || (m.identity.trajectory_epoch==ckf_epoch_ && m.identity.revision_id<=revision_))return false;
  event_=m.event_sequence;revision_=m.identity.revision_id;ckf_epoch_=m.identity.trajectory_epoch;
  last_forecast_=m.identity.source_time_ns;
  auto candidate=extract(m,config_,receipt);
  if(!candidate.strike.valid) {reason_=candidate.strike.reason;return false;}
  BestPlaneEstimate e{candidate.strike,candidate.absolute_hit_ns,m.identity.source_time_ns,receipt,flight_,compact(m)};
  cache_.push_back(e);if(cache_.size()>16)cache_.pop_front();
  if(e.source_ns>=start_ && (!pending_ || e.source_ns<=commit_) &&
      (!best_crossing_ || e.source_ns>best_crossing_->source_ns))best_crossing_=e;
  // A reset may pre-cache registered evidence before the next physical flight;
  // accepting it does not rearm an already committed flight.
  return !decided_ || start_==0;
}
bool StrikeTargetSelector::full_forecast(const hope_msgs::msg::BallForecast& m,std::int64_t receipt) {
  if(halted_ || retired_.count(m.publisher_instance_id))return false;
  if(authorized_reset_ack(m))return false;
  if(m.kind==m.INVALIDATION && m.reason=="explicit_reset") {fence("unrequested_forecaster_reset");return false;}
  // The full and compact topics have independent event counters.
  if(owner_ && m.publisher_instance_id==owner_->publisher_instance_id && m.event_sequence<=full_event_)return false;
  if(!establish_owner(m) || m.kind!=m.FORECAST)return false;
  if(m.identity.source_time_ns<=0 || m.identity.source_time_ns>receipt)return false;
  if(last_full_ && m.identity.source_time_ns<last_full_-1000000) {fence("full_forecast_clock_discontinuity");return false;}
  if(m.identity.source_time_ns<=last_full_ || m.identity.trajectory_epoch<full_epoch_ ||
      (m.identity.trajectory_epoch==full_epoch_ && m.identity.revision_id<=full_revision_))return false;
  full_event_=m.event_sequence;full_epoch_=m.identity.trajectory_epoch;full_revision_=m.identity.revision_id;last_full_=m.identity.source_time_ns;
  if(m.waypoints.empty())return false;
  const auto& w=m.waypoints.front();
  if(w.dt_ns!=0 || !w.absolute_time_valid || w.absolute_time_ns!=m.identity.source_time_ns || !w.physical_continuation)return false;
  for(double v:w.state)if(!std::isfinite(v))return false;
  BallState state;state.position=Vec3(w.state[0],w.state[1],w.state[2]);state.velocity=Vec3(w.state[3],w.state[4],w.state[5]);
  state.valid=true;state.reason=w.linearization_quality.empty()?"uncertainty_unavailable":w.linearization_quality;
  if(pending_ && m.identity.source_time_ns>commit_)return false;
  states_.push_back(SavedState{state,m.identity.source_time_ns,receipt,compact(m),w.covariance_upper});if(states_.size()>16)states_.pop_front();return true;
}
void StrikeTargetSelector::fallback(StrikeTargetSelection& out,std::int64_t now) {
  using Clock=std::chrono::steady_clock;
  TrajectoryPredictor predictor(config_.physics,config_.planner,TableParams{});
  auto rollout=[&](BallState state,std::int64_t source,const std::string& origin) {
    if(!state.valid || !state.position.allFinite() || !state.velocity.allFinite() || state.position.x()<=plane_->x || state.position.z()<=0) {
      out.reason="invalid_or_passed_source_state";return false;
    }
    state.source_time_s=static_cast<double>(source-origin_)*1e-9;
    const auto started=Clock::now();
    auto strike=predictor.predict_with_spin(state,plane_->x,Vec3::Zero(),SpinPhysicsMode::kVenueGripBounce);
    out.rollout_ms+=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    out.strike=strike;out.source_ns=source;
    // Legacy Stage 2 also labels descending crossings below 5 cm as dead_ball.
    // Keep its numerical rollout unchanged, but distinguish that advisory band
    // from a genuinely nonpositive or missing crossing for shadow planning.
    const bool low_height=strike.reason=="dead_ball" && strike.ball_position.z()>0 &&
      strike.ball_position.allFinite() && strike.ball_velocity.allFinite() &&
      strike.ball_velocity.x()<0 && std::abs(strike.ball_position.x()-plane_->x)<1e-8;
    if(!strike.valid && !low_height) {out.reason="legacy_stage2:"+strike.reason;return false;}
    if(low_height)out.strike.valid=true;
    std::int64_t hit=0;
    if(!absolute_ns(origin_,strike.strike_source_time_s,hit)) {out.reason="legacy_hit_time_overflow";out.strike.valid=false;return false;}
    out.absolute_hit_ns=hit;
    if(hit<=now || strike.ball_position.z()<=0 || strike.ball_velocity.x()>=0) {
      out.reason="legacy_past_or_below_ground_crossing";out.strike.valid=false;return false;
    }
    out.strike.strike_source_time_s=static_cast<double>(hit)*1e-9;
    out.origin=origin;out.fallback_used=true;out.quality=state.reason+(origin=="batch_fit"?":uncertainty_unavailable":"")+(low_height?":legacy_low_height":"");
    out.eligibility_status=contact_illegal_?"known_illegal":"unknown";
    out.reason="selected_"+origin;return true;
  };
  for(auto it=states_.rbegin();it!=states_.rend();++it) {
    if(it->source_ns<start_ || it->source_ns>commit_)continue;
    if(rollout(it->state,it->source_ns,"ckf_state_rollout")) {
      out.forecast=it->metadata;out.forecast_receipt_ns=it->receipt_ns;return;
    }
  }
  // Immutable causal copy; the existing estimator applies its own unchanged
  // 192-sample / 0.18-second latest-source window and numerical readiness rules.
  std::vector<BallSample> causal;
  for(const auto& s:samples_)if(s.source_time_ns>=start_ && s.source_time_ns<=commit_)causal.push_back(s);
  if(causal.empty()) {out.reason="no_usable_observations";return;}
  const auto started=Clock::now();
  BatchPhysicsEstimator estimator(config_.physics,config_.estimator);
  estimator.begin_flight();for(const auto& s:causal)estimator.push(s);
  auto state=estimator.estimate();
  out.estimator_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
  out.sample_count=estimator.sample_count();out.sample_span_s=estimator.sample_span_s();
  if(state.valid) {rollout(state,causal.back().source_time_ns,"batch_fit");return;}
  if(state.reason!="estimator_not_ready") {out.reason="batch_fit:"+state.reason;return;}
  while(causal.size()>1 && causal.back().source_time_s-causal.front().source_time_s>config_.estimator.window_s)causal.erase(causal.begin());
  if(causal.size()<2) {out.reason="insufficient_distinct_observations";return;}
  double mean=0;Vec3 position=Vec3::Zero();
  for(const auto& s:causal) {mean+=s.source_time_s;position+=s.position;}
  mean/=causal.size();position/=causal.size();double denominator=0;Vec3 numerator=Vec3::Zero();
  for(const auto& s:causal) {const double dt=s.source_time_s-mean;denominator+=dt*dt;numerator+=dt*(s.position-position);}
  if(denominator<=1e-16) {out.reason="insufficient_distinct_observations";return;}
  state.position=causal.back().position;state.velocity=numerator/denominator;state.valid=state.velocity.allFinite();
  state.reason="low_sample_linear_fit_uncertainty_unavailable";
  out.sample_count=causal.size();out.sample_span_s=causal.back().source_time_s-causal.front().source_time_s;
  rollout(state,causal.back().source_time_ns,"sparse_fit");
}
void StrikeTargetSelector::context(const hope_msgs::msg::HybridTrajectoryContext& m,std::int64_t now) {
  if(config_.context_authority.empty() || m.evidence_source_id!=config_.context_authority ||
     m.frame_id!=config_.frame || !fresh(m.source_time_ns,now,config_.max_age_ns) || m.source_time_ns<=context_last_)return;
  context_last_=m.source_time_ns;
  if(m.kind==m.RESET) {
    if(flight_ && !decided_) {auto old=decision_metadata(now);old.reason="explicit_reset_before_decision";terminal_=old;}
    if(retirement_exhausted_) {fence("retired_owner_capacity_exhausted");return;}
    ++session_;authorized_reset_sources_.push_back(m.source_time_ns);
    if(authorized_reset_sources_.size()>16)authorized_reset_sources_.pop_front();
    incoming_.reset_phase();cache_.clear();best_crossing_.reset();states_.clear();samples_.clear();
    contacts_.clear();owner_.reset();plane_=config_.frozen_plane;
    event_=revision_=ckf_epoch_=full_event_=full_revision_=full_epoch_=0;
    last_forecast_=last_full_=last_pose_=origin_=start_=commit_=net_=0;
    halted_=pending_=complete_=contact_illegal_=false;decided_=true;reason_="explicit_session_reset";return;
  }
  if(m.kind==m.BEGIN) {rally_=m.rally_id;context_begin_=m.source_time_ns;contacts_.clear();contact_illegal_=false;complete_=m.history_complete && rally_;return;}
  if(m.kind!=m.VERIFIED_CONTACT || m.rally_id!=rally_ || !complete_ || m.source_time_ns<context_begin_)return;
  const auto& p=m.contact_position;
  const bool receiver=p.x>0 && p.x<config_.incoming.net_x;
  const bool opponent=p.x>config_.incoming.net_x && p.x<2.74;
  if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.y < -1.525 || p.y>0 || std::abs(p.z)>.05 ||
     !((receiver && m.contact_side=="receiver") || (opponent && m.contact_side=="opponent")) || contacts_.size()==16) {
    complete_=false;contact_illegal_=true;reason_="invalid_contact_history";return;
  }
  contacts_.push_back(m);
}
StrikeTargetSelection StrikeTargetSelector::decision_metadata(std::int64_t now) const {
  StrikeTargetSelection out;out.physical_flight_id=flight_;out.session_id=session_;out.commit_ns=commit_;
  out.net_ns=net_;out.selected_ns=now;out.late_acquisition=late_acquisition_;
  if(plane_) {out.plane_x=plane_->x;out.plane_generation=plane_->generation;out.world_frame_id=plane_->world;
    out.calibration_id=plane_->calibration;out.known_test_plane=plane_->synthetic;}
  std::int64_t first=0,last=0,latest=0;
  for(const auto& s:samples_)if(s.source_time_ns>=start_ && s.source_time_ns<=commit_)latest=s.source_time_ns;
  for(const auto& s:samples_)if(s.source_time_ns>=start_ && s.source_time_ns<=commit_ &&
      static_cast<long double>(latest)-s.source_time_ns<=config_.estimator.window_s*1e9) {
    if(!first)first=s.source_time_ns;
    last=s.source_time_ns;++out.sample_count;
  }
  out.sample_span_s=static_cast<double>(last-first)*1e-9;return out;
}
void StrikeTargetSelector::classify_history(StrikeTargetSelection& out) {
  const auto& m=out.forecast;std::vector<std::string> sides;
  bool known=complete_ && rally_ && rally_==m.identity.rally_id && context_begin_<=out.source_ns;
  bool bad=contact_illegal_;bool geometry_known=out.source_ns>=net_;
  int verified_receivers=0,verified_opponents=0;
  for(const auto& c:contacts_)if(c.source_time_ns<=out.source_ns) {
    if((c.contact_side=="receiver" && c.source_time_ns<net_) || (c.contact_side=="opponent" && c.source_time_ns>=net_))bad=true;
    if(c.contact_side=="receiver")++verified_receivers;else ++verified_opponents;
    sides.push_back(c.contact_side);++out.verified_contacts;
  }
  std::int64_t previous=0;
  if(m.physics_event_sequence.size()!=m.physics_event_dt_ns.size())geometry_known=false;
  if(geometry_known)for(std::size_t i=0;i<m.physics_event_sequence.size();++i) {
    const auto dt=m.physics_event_dt_ns[i];if(dt<=0 || dt<=previous) {geometry_known=false;break;}previous=dt;
    if(dt>=m.hit_plane.dt_ns)continue;
    if(m.physics_event_sequence[i]=="table_bounce") {sides.push_back("receiver");++out.strike.predicted_bounces;}
    else if(m.physics_event_sequence[i]!="table_miss")geometry_known=false;
  }
  bad=bad || verified_receivers>1 || verified_opponents>1;
  // Before-net events still supply future counts, without asserting a side.
  if(!geometry_known) {
    out.strike.predicted_bounces=0;
    if(m.physics_event_sequence.size()==m.physics_event_dt_ns.size())
      for(std::size_t i=0;i<m.physics_event_sequence.size();++i)
        if(m.physics_event_sequence[i]=="table_bounce" && m.physics_event_dt_ns[i]>0 && m.physics_event_dt_ns[i]<m.hit_plane.dt_ns)
          ++out.strike.predicted_bounces;
  }
  const bool legal=(m.rally_context=="normal_return" && sides==std::vector<std::string>{"receiver"}) ||
    (m.rally_context=="serve_return" && sides==std::vector<std::string>{"opponent","receiver"});
  // Missing contact updates are not proof that a legal contact never happened.
  // Future contacts are modeled, so do not describe them as verified legality.
  out.eligibility_status=bad?"known_illegal":known && geometry_known && legal?"eligible_with_verified_history":"unknown";
}
std::optional<StrikeTargetSelection> StrikeTargetSelector::poll(std::int64_t now,std::int64_t steady) {
  if(terminal_) {auto out=*terminal_;terminal_.reset();return out;}
  if(pending_ && (now<=0 || (last_pose_>1000000 && now<last_pose_-1000000)))fence("selection_clock_discontinuity");
  if(!pending_ || steady<deadline_)return std::nullopt;
  if(config_.clock=="ros_sim" && !halted_ && net_) {
    std::int64_t due=0;
    if(add_ns(commit_,config_.grace_ns,due) && now<due)return std::nullopt;
  }
  pending_=false;decided_=true;auto out=decision_metadata(now);out.reason="no_usable_crossing";
  if(halted_)out.reason=reason_;
  else if(!net_)out.reason="no_net_timing_evidence";
  else if(!plane_)out.reason="frozen_plane_unavailable";
  else {
    std::optional<BestPlaneEstimate> chosen;
    auto consider=[&](const BestPlaneEstimate& e) {
      if(e.source_ns<start_ || e.source_ns>commit_ || e.absolute_hit_ns<=now)return;
      if(!chosen || e.source_ns>chosen->source_ns)chosen=e;
    };
    if(best_crossing_)consider(*best_crossing_);
    for(const auto& e:cache_)consider(e);
    if(chosen) {
      out.strike=chosen->strike;out.absolute_hit_ns=chosen->absolute_hit_ns;out.source_ns=chosen->source_ns;
      out.forecast=chosen->metadata;out.forecast_receipt_ns=chosen->receipt_ns;
      const auto& m=chosen->metadata;
      const bool nominal=fresh(out.source_ns,now,config_.max_age_ns) && fresh(m.last_measurement_time_ns,now,config_.max_age_ns) &&
        now<=m.valid_until_source_ns && m.event_future_at_publish;
      out.origin=nominal?"nominal_ckf":"retained_ckf";out.fallback_used=!nominal;
      out.reason="selected_forecast_crossing";out.quality=m.hit_plane.linearization_quality.empty()?"uncertainty_unavailable":m.hit_plane.linearization_quality;classify_history(out);
    } else fallback(out,now);
    if(last_pose_>=out.source_ns && last_position_.x()<=plane_->x) {out.strike.valid=false;out.reason="observed_hit_plane_passed";}
  }
  if(out.strike.valid && (out.absolute_hit_ns<=now || !out.strike.ball_position.allFinite() || !out.strike.ball_velocity.allFinite() ||
      out.strike.ball_position.z()<=0 || out.strike.ball_velocity.x()>=0)) {out.strike.valid=false;out.reason="invalid_or_past_crossing";}
  if(out.eligibility_status=="known_illegal" || contact_illegal_) {out.strike.valid=false;out.reason="known_illegal_route";}
  for(const auto& sample:samples_)if(sample.source_time_ns==out.source_ns)out.observation_receipt_ns=sample.receipt_wall_ns;
  out.strike.reason=out.reason;cache_.clear();best_crossing_.reset();states_.clear();contacts_.clear();complete_=false;
  reason_="decision_emitted:"+out.reason;return out;
}
double strike_target_selector_swing(double y,double base,double previous,double split,double h) {
  const double rel=y-base;if(previous>.5)return rel>split+h?-1:1;if(previous<-.5)return rel<split-h?1:-1;return rel<split?1:-1;
}
}  // namespace hope_planner_cpp
