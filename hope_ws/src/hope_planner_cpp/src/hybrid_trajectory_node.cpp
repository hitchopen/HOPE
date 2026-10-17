#include "hope_planner_cpp/hybrid_trajectory_node.hpp"
#include <chrono>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
namespace hope_planner_cpp {
namespace {
std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::int64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
template<class T> void xyz(T &to, const Vec3 &from) {
  to.x = from.x(); to.y = from.y(); to.z = from.z();
}
hope_msgs::msg::HybridTrajectoryIdentity identity(const StateBelief &b,
    const FlightState &f, const std::string &config) {
  hope_msgs::msg::HybridTrajectoryIdentity id;
  id.rally_id=f.rally_id; id.trajectory_epoch=f.trajectory_epoch;
  id.revision_id=f.revision_id; id.source_time_ns=b.source_time_ns;
  id.frame_id=b.frame_id; id.return_region_config_id=config;
  return id;
}
hope_msgs::msg::BallForecastWaypoint waypoint(const State6 &s, double dt,
                                                   std::int64_t source) {
  hope_msgs::msg::BallForecastWaypoint p;
  p.dt_ns = std::llround(dt*1e9);
  p.absolute_time_valid=p.dt_ns>=0 ? source<=std::numeric_limits<std::int64_t>::max()-p.dt_ns :
    source>=std::numeric_limits<std::int64_t>::min()-p.dt_ns;
  if(p.absolute_time_valid) p.absolute_time_ns=source+p.dt_ns;
  for(int i=0;i<6;++i)p.state[i]=s[i];
  return p;
}
} // namespace
HybridTrajectoryNode::HybridTrajectoryNode(const rclcpp::NodeOptions &options)
    : Node("hope_hybrid_trajectory_node",options) {
  for(const std::string topic : {"/ball/forecast/v1/event", "/ball/forecast/v1/trajectory", "/ball/hybrid/diagnostics"})
    if(get_node_topics_interface()->resolve_topic_name(topic)!=topic)
      throw std::invalid_argument("shadow forecast output remapping forbidden");
  {
    forecast_pub_=create_publisher<hope_msgs::msg::BallForecast>("/ball/forecast/v1/trajectory",rclcpp::QoS(512).reliable());
    forecast_event_pub_=create_publisher<hope_msgs::msg::BallForecast>("/ball/forecast/v1/event",rclcpp::QoS(512).reliable());
  }
  frame_=declare_parameter<std::string>("registered_frame","table");
  registration_id_=declare_parameter<std::string>("frame_registration_id","");
  frame_registered_=declare_parameter<bool>("frame_registration_confirmed",false);
  source_clock_registered_=declare_parameter<bool>("source_clock_registered",false);
  clock_domain_=declare_parameter<std::string>("source_clock_domain","ros_system");
  if(get_parameter("use_sim_time").as_bool()!=(clock_domain_=="ros_sim"))
    throw std::invalid_argument("registered clock must match ROS clock");
  ball_index_=declare_parameter<int>("ball_pose_index",0);
  auto input=declare_parameter<std::string>("input_pose_topic","/poses");
  auto context_topic=declare_parameter<std::string>("context_topic","/ball/hybrid/context");
  const std::string output="/ball/forecast/v1/trajectory";
  const auto depth=declare_parameter<int>("input_depth",256);
  if (depth<1 || depth>4096 || ball_index_<0 || frame_.empty() || frame_.size()>128 ||
      registration_id_.size()>128 || (frame_registered_ && registration_id_.empty()) ||
      (clock_domain_!="ros_system" && clock_domain_!="ros_sim"))
    throw std::invalid_argument("invalid input/frame/clock registration");
  ProbabilisticConfig c;
  c.physics.drag_k=declare_parameter<double>("drag_k",.1261);
  c.physics.restitution_h=declare_parameter<double>("restitution_h",.64);
  c.physics.restitution_v=declare_parameter<double>("restitution_v",.9215);
  c.physics.table_tangential_gain=declare_parameter<double>("table_tangential_gain",.369);
  c.physics.table_friction_cap_mu=declare_parameter<double>("table_friction_cap_mu",2.0);
  c.measurement_std_m.setConstant(.0019); c.measurement_bias_std_m.setConstant(.0052);
  forecast_noise_=ForecastNoiseConfig::inherited(c);
  forecast_noise_.acceleration_spectral_density=declare_parameter<double>("forecast_acceleration_spectral_density",.10);
  const auto bounce_noise=declare_parameter<std::vector<double>>("forecast_bounce_velocity_std",{.65,.30,.45});
  if(bounce_noise.size()!=3)throw std::invalid_argument("three forecast bounce deviations required");
  for(int i=0;i<3;++i)forecast_noise_.bounce_velocity_std[i]=bounce_noise[i];
  forecast_noise_.validate();
  ReturnRegionPolicy p;
  p.post_hit_distance_m=declare_parameter<double>("post_hit_distance_m",.50);
  if (p.post_hit_distance_m!=.50 && p.post_hit_distance_m!=.75)
    throw std::invalid_argument("registered endpoints are .50 or .75m");
  p.publication_period_s=declare_parameter<double>("publication_period_s",.010);
  p.max_source_age_s=max_age_=declare_parameter<double>("max_source_age_s",.030);
  p.validate();
  if (p.publication_period_s > .020)
    throw std::invalid_argument("nominal publication target requires period <=20ms");
  std::random_device random;
  std::ostringstream instance; instance<<std::hex<<random()<<random()<<steady_ns();
  instance_=instance.str();
  config_=c;policy_=p;
  HitPlaneSettings settings;
  settings.offset_m=declare_parameter<double>("hit_plane_offset_m",.58);
  settings.window_s=declare_parameter<double>("base_window_s",.5);
  settings.max_age_s=declare_parameter<double>("base_max_age_s",.2);
  settings.max_span_m=declare_parameter<double>("base_max_span_m",.01);
  const auto min_samples=declare_parameter<int>("base_min_samples",10);
  if(min_samples<2) throw std::invalid_argument("base_min_samples must be >=2");
  settings.min_samples=static_cast<std::size_t>(min_samples);
  settings.min_x_m=declare_parameter<double>("hit_plane_min_x_m",-.30);
  settings.max_x_m=declare_parameter<double>("hit_plane_max_x_m",.30);
  const auto world_id=declare_parameter<std::int64_t>("base_world_frame_id",0);
  if(world_id<0 || world_id>4503599627370496LL) throw std::invalid_argument("invalid world ID");
  plane_=std::make_unique<HitPlaneManager>(settings,frame_,registration_id_,world_id);
  const bool test_plane=declare_parameter<bool>("use_known_test_plane",false);
  const double test_x=declare_parameter<double>("known_test_plane_x_m",.15);
  if(test_plane) plane_->use_test_plane(test_x,0);
  diagnostic_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/ball/hybrid/diagnostics",10);
  activate_plane(0);
  base_sub_=create_subscription<std_msgs::msg::Float64MultiArray>(
    declare_parameter<std::string>("base_pose_topic","/a3/base_pose_flat"),rclcpp::QoS(64).best_effort(),
    [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
      plane_->observe(message->data,steady_ns(),get_clock()->now().nanoseconds(),source_clock_registered_ && frame_registered_);
    });
  freeze_service_=create_service<std_srvs::srv::Trigger>("~/freeze_x_hit",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto now=get_clock()->now().nanoseconds();
      response->success=plane_->freeze(steady_ns(),now,rally_id_,!rally_active_);
      response->message=plane_->reason();
      if(response->success) activate_plane(now);
      diagnostics();
    });
  diagnostic_timer_=create_wall_timer(std::chrono::seconds(1),[this]{diagnostics();});
  observation_sub_=create_subscription<geometry_msgs::msg::PoseArray>(input,
      rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){observation(*m);});
  context_sub_=create_subscription<hope_msgs::msg::HybridTrajectoryContext>(context_topic,
      rclcpp::QoS(64).reliable(),
      [this](hope_msgs::msg::HybridTrajectoryContext::ConstSharedPtr m){context(*m);});
  timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{tick();});
  callback_ns_=steady_ns(); coordinator_->invalidate("startup",get_clock()->now().nanoseconds());
  RCLCPP_INFO(get_logger(),"Nominal trajectory only; frame registered=%d, source clock registered=%d",
      frame_registered_,source_clock_registered_);
}
void HybridTrajectoryNode::observation(const geometry_msgs::msg::PoseArray &m) {
  ++observation_callbacks_;
  callback_ns_=steady_ns(); ckf_complete_ns_=0;
  const auto now=get_clock()->now().nanoseconds();
  if (m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000u) {
    coordinator_->reset(now,"invalid_observation_envelope"); return;
  }
  const auto ns=rclcpp::Time(m.header.stamp).nanoseconds();
  if(!plane_->snapshot().calibrated) {
    coordinator_->reset(ns,"hit_plane_not_calibrated");return;
  }
  if(ns<=plane_->snapshot().activated_source_ns) {
    coordinator_->invalidate("pre_refresh_observation",ns);return;
  }
  if (!frame_registered_ || m.header.frame_id!=frame_) {
    coordinator_->reset(ns,"frame_unknown_or_mismatch"); return;
  }
  if (!source_clock_registered_) {
    coordinator_->reset(ns,"source_clock_unregistered"); return;
  }
  if (m.header.stamp.nanosec>=1000000000u || ns<=0 ||
      static_cast<std::size_t>(ball_index_)>=m.poses.size()) {
    coordinator_->reset(ns,"invalid_observation_envelope"); return;
  }
  const double age=double((static_cast<long double>(now)-ns)*1e-9L);
  if (age<-.001) { coordinator_->reset(ns,"future_source_timestamp"); return; }
  BallSample sample; sample.source_time_ns=ns;
  const auto &p=m.poses[ball_index_].position; sample.position=Vec3(p.x,p.y,p.z);
  sample.receipt_steady_ns=callback_ns_; sample.receipt_wall_ns=wall_ns();
  coordinator_->observe(sample,m.header.frame_id,age);
}
void HybridTrajectoryNode::context(const hope_msgs::msg::HybridTrajectoryContext &m) {
  callback_ns_=steady_ns(); ckf_complete_ns_=0;
  const auto now=get_clock()->now().nanoseconds();
  const bool metadata_only=m.kind!=m.RESET;
  if(clock_domain_=="ros_sim" && source_clock_registered_ && frame_registered_ &&
      m.frame_id==frame_ && m.source_time_ns>0 && now==0) {
    if(pending_sim_context_.size()==64) {
      pending_sim_context_.clear();coordinator_->reset(now,"sim_context_queue_exceeded");
    } else pending_sim_context_.push_back(m);
    return;
  }
  if (!frame_registered_ || m.frame_id!=frame_ || m.source_time_ns<=0 ||
      static_cast<long double>(m.source_time_ns)-now>1000000.L) {
    if(!metadata_only)coordinator_->reset(now,"invalid_context_envelope");
    return;
  }
  if(!source_clock_registered_ || m.source_time_ns<=plane_->snapshot().activated_source_ns) {
    if(!metadata_only)coordinator_->invalidate("unregistered_or_pre_refresh_context",now);
    return;
  }
  if (m.kind==m.BEGIN) {
    rally_active_=true;rally_id_=m.rally_id;
    coordinator_->begin(m.source_time_ns,m.rally_id,m.we_are_serving,m.history_complete);
  } else if (m.kind==m.RESET) {
    rally_active_=false;coordinator_->reset(m.source_time_ns,"explicit_reset");
  }
  else if (m.kind==m.VERIFIED_CONTACT) {
    auto side=m.contact_side=="receiver" ? ContactSide::kReceiver :
        m.contact_side=="opponent" ? ContactSide::kOpponent : ContactSide::kAmbiguous;
    try {
      coordinator_->verified_contact(m.source_time_ns,m.rally_id,side,
          Vec3(m.contact_position.x,m.contact_position.y,m.contact_position.z),m.evidence_source_id);
    } catch (const std::exception &) { /* Context metadata cannot reset the CKF. */ }
  } else if(!metadata_only)coordinator_->reset(now,"invalid_context_kind");
}
void HybridTrajectoryNode::tick() {
  const auto ros=get_clock()->now().nanoseconds(), steady=steady_ns();
  if (last_ros_ns_ && *last_ros_ns_>0 && last_steady_ns_) {
    const long double jump=(static_cast<long double>(ros)-*last_ros_ns_)-
                          (static_cast<long double>(steady)-*last_steady_ns_);
    if (std::abs(jump)>100000000.L) {
      pending_sim_context_.clear();
      plane_->revoke("receipt_clock_jump");active_plane_=plane_->snapshot();rally_active_=false;
      coordinator_->reset(ros,"receipt_clock_jump");
    }
  }
  last_ros_ns_=ros; last_steady_ns_=steady;
  if(ros>0) {
    while(!pending_sim_context_.empty()) {
      auto message=std::move(pending_sim_context_.front());pending_sim_context_.pop_front();
      context(message);
    }
  }
  coordinator_->expire(ros);
}
void HybridTrajectoryNode::activate_plane(std::int64_t now) {
  // The old owner emits invalidations first. A new owner binds CKF, state machine,
  // predictor and configuration identity to one immutable generation.
  if(coordinator_) coordinator_->reset(now,"hit_plane_refresh");
  pending_sim_context_.clear();
  active_plane_=plane_->snapshot();
  config_.x_hit=active_plane_.x_hit_m;
  coordinator_=std::make_unique<HybridTrajectoryCoordinator>(config_,policy_,frame_,
    [this](const ForecastUpdate&e){return publish_forecast(e,false);},
    [this](const ForecastUpdate&e){return publish_forecast(e,true);},plane_->snapshot().generation,forecast_noise_);
}
void HybridTrajectoryNode::diagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  const auto& plane=plane_->snapshot();
  status.name="hybrid/frozen_hit_plane";status.hardware_id=instance_;
  status.level=plane.calibrated ? 0 : 1;status.message=plane_->reason();
  auto add=[&](const std::string& key,const std::string& value) {
    diagnostic_msgs::msg::KeyValue pair;pair.key=key;pair.value=value;status.values.push_back(pair);
  };
  add("x_hit_calibrated",plane.calibrated ? "true" : "false");
  add("known_test_plane",plane.test_plane ? "true" : "false");
  add("x_hit_m",std::to_string(plane.x_hit_m));add("generation",std::to_string(plane.generation));
  add("frame_id",plane.frame_id);add("frame_registration_id",plane.registration_id);
  add("frame_registration_confirmed",frame_registered_ ? "true" : "false");
  add("source_clock_registered",source_clock_registered_ ? "true" : "false");
  add("source_clock_domain",clock_domain_);
  add("use_sim_time",get_parameter("use_sim_time").as_bool() ? "true" : "false");
  add("clock_now_ns",std::to_string(get_clock()->now().nanoseconds()));
  add("tracker_initialized",coordinator_->belief().initialized ? "true" : "false");
  add("observation_callbacks",std::to_string(observation_callbacks_));
  add("accepted_updates",std::to_string(accepted_updates_));
  add("last_accepted_measurement_ns",std::to_string(last_accepted_ns_));
  add("last_forecast_source_ns",std::to_string(last_forecast_ns_));
  add("last_invalidation_reason",last_invalidation_reason_);
  add("compact_evaluated",std::to_string(compact_evaluated_));
  add("compact_published",std::to_string(compact_published_));
  add("full_published",std::to_string(full_published_));
  add("last_message_build_ns",std::to_string(last_message_build_ns_));
  add("last_publish_call_ns",std::to_string(last_publish_call_ns_));
  add("refresh_locked_by_begin",rally_active_ ? "true" : "false");
  add("input_pose_topic",get_parameter("input_pose_topic").as_string());
  add("input_publishers",std::to_string(observation_sub_->get_publisher_count()));
  array.status.push_back(status);diagnostic_pub_->publish(array);
}
bool HybridTrajectoryNode::publish_forecast(const ForecastUpdate &e,bool compact) {
  const auto build_begin=steady_ns();
  if(e.kind==ForecastUpdate::Invalidation)last_invalidation_reason_=e.reason;
  if(e.kind==ForecastUpdate::StateUpdate) {
    if(e.update.accepted){++accepted_updates_;last_accepted_ns_=e.state.last_measurement_time_ns;}
    ckf_complete_ns_=steady_ns();return true;
  }
  if(e.reason=="source_epoch_reset") {
    plane_->revoke(e.reason);active_plane_=plane_->snapshot();rally_active_=false;
  }
  if(compact && e.forecast)++compact_evaluated_;
  hope_msgs::msg::BallForecast m;m.schema="ball-forecast-v1";
  m.kind=static_cast<std::uint8_t>(e.kind);m.publisher_instance_id=instance_;
  m.event_sequence=compact?++projection_sequence_:++sequence_;
  m.identity=identity(e.state,e.flight,e.config_id);
  m.hit_plane_generation=active_plane_.generation;m.x_hit_m=active_plane_.x_hit_m;
  m.x_hit_calibrated=active_plane_.calibrated;m.known_test_plane=active_plane_.test_plane;
  m.calibration_id=active_plane_.calibration_id;m.world_frame_id=active_plane_.world_frame_id;
  m.frame_registration_id=registration_id_;m.source_clock_domain=clock_domain_;
  m.source_clock_registered=source_clock_registered_;m.last_measurement_time_ns=e.state.last_measurement_time_ns;
  m.receipt_steady_ns=e.state.receipt_steady_ns;m.callback_steady_ns=callback_ns_;
  m.ckf_complete_steady_ns=ckf_complete_ns_;m.rollout_complete_steady_ns=e.rollout_complete_steady_ns;
  m.clock_uncertainty_status="unavailable_not_modeled";m.plane_uncertainty_status="unavailable_not_modeled";
  m.reason=e.reason;m.measurement_accepted=e.update.accepted;
  m.rally_context=name(e.flight.rally_context);m.motion_state=name(e.flight.motion_state);
  m.history_continuity=name(e.history.continuity);
  auto convert=[&](const ForecastPhysicsEvent &v) {
    hope_msgs::msg::BallForecastPhysicsEvent o;o.kind=v.kind;
    auto stamp=waypoint(v.pre,v.dt_s,e.state.source_time_ns);
    o.dt_ns=stamp.dt_ns;o.absolute_time_ns=stamp.absolute_time_ns;o.absolute_time_valid=stamp.absolute_time_valid;
    for(int i=0;i<6;++i) {o.pre_state[i]=v.pre[i];o.post_state[i]=v.post[i];}
    unsigned k=0;for(int i=0;i<7;++i)for(int j=i;j<7;++j)o.covariance_upper[k++]=v.covariance(i,j);
    if(v.pre.tail<3>().norm()>1e-12)xyz(o.incoming_direction,v.pre.tail<3>().normalized());
    xyz(o.surface_normal,(v.kind=="hit_plane" || v.kind=="net")?Vec3(1,0,0):Vec3(0,0,1));
    o.approach_speed_mps=std::abs((v.kind=="hit_plane" || v.kind=="net")?v.pre[3]:v.pre[5]);
    o.time_std_s=v.time_std_s;o.linearization_quality=quality_name(v.quality);
    o.physically_usable=v.physically_usable;return o;
  };
  if(e.forecast) {
    const auto &f=*e.forecast;m.forecast_available=f.available;m.reason=f.termination_reason;
    m.uncertainty_provenance=f.uncertainty_provenance;
    for(const auto &event:f.events) {
      m.physics_event_sequence.push_back(event.kind);
      m.physics_event_dt_ns.push_back(std::llround(event.dt_s*1e9));
    }
    const auto expiry=static_cast<long double>(e.state.last_measurement_time_ns)+max_age_*1e9L;
    m.valid_until_source_ns=expiry<std::numeric_limits<std::int64_t>::max()?static_cast<std::int64_t>(expiry):e.state.source_time_ns;
    if(f.hit_plane) {
      m.crossing_exists=true;m.hit_plane=convert(*f.hit_plane);
      m.height_above_zero=f.hit_plane->pre[2]>0;m.physically_usable=f.hit_plane->physically_usable;
    }
    if(!compact) {
      for(const auto &v:f.waypoints) {
        hope_msgs::msg::BallForecastWaypoint o;auto stamp=waypoint(v.state,v.dt_s,e.state.source_time_ns);
        o.dt_ns=stamp.dt_ns;o.absolute_time_ns=stamp.absolute_time_ns;o.absolute_time_valid=stamp.absolute_time_valid;
        for(int i=0;i<6;++i)o.state[i]=v.state[i];
        unsigned k=0;for(int i=0;i<6;++i)for(int j=i;j<6;++j)o.covariance_upper[k++]=v.covariance(i,j);
        o.physical_continuation=v.physical_continuation;
        o.linearization_quality=quality_name(v.quality);m.waypoints.push_back(o);
      }
      for(const auto &v:f.events)m.physics_events.push_back(convert(v));
    }
  }
  m.publish_ros_ns=get_clock()->now().nanoseconds();m.publish_steady_ns=steady_ns();m.publish_wall_ns=wall_ns();
  m.event_future_at_publish=m.crossing_exists && m.hit_plane.absolute_time_valid &&
    m.hit_plane.absolute_time_ns>m.publish_ros_ns;
  if(m.forecast_available && (m.publish_ros_ns>=m.valid_until_source_ns ||
     static_cast<long double>(m.publish_ros_ns)-e.state.source_time_ns>max_age_*1e9L)) {
    coordinator_->invalidate("source_age_expired",e.state.source_time_ns);return false;
  }
  last_message_build_ns_=steady_ns()-build_begin;
  const auto publish_begin=steady_ns();
  if(compact)forecast_event_pub_->publish(m);else forecast_pub_->publish(m);
  last_publish_call_ns_=steady_ns()-publish_begin;
  if(m.forecast_available){
    last_forecast_ns_=m.identity.source_time_ns;
    if(compact)++compact_published_;else ++full_published_;
  }
  return m.forecast_available;
}
} // namespace hope_planner_cpp
