#include "hope_planner_cpp/strike_target_selector.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/hit_plane_manager.hpp"
#include "hope_msgs/msg/racket_command.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace hope_planner_cpp {
namespace {
std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<class T> void xyz(T& out,const Vec3& v) {out.x=v.x();out.y=v.y();out.z=v.z();}
}
class StrikeTargetSelectorNode : public rclcpp::Node {
 public:
  StrikeTargetSelectorNode():Node("hope_strike_target_selector_node") {
    enabled_=declare_parameter<bool>("enabled",true);
    pose_audit_=declare_parameter<bool>("pose_audit",false);
    StrikeTargetSelectorConfig c;
    c.frame=declare_parameter<std::string>("registered_frame","table");
    c.registration=declare_parameter<std::string>("frame_registration_id","");
    c.clock=declare_parameter<std::string>("source_clock_domain","ros_system");
    c.synthetic=declare_parameter<bool>("tagged_synthetic_test",false);
    c.context_authority=declare_parameter<std::string>("context_evidence_source_id","");
    hit_y_min_table_m_=declare_parameter<double>("hit_y_min_table_m",std::numeric_limits<double>::quiet_NaN());
    hit_y_max_table_m_=declare_parameter<double>("hit_y_max_table_m",std::numeric_limits<double>::quiet_NaN());
    if(!std::isfinite(hit_y_min_table_m_) || !std::isfinite(hit_y_max_table_m_) ||
       hit_y_min_table_m_>=hit_y_max_table_m_)
      throw std::invalid_argument("finite hit_y_min_table_m < hit_y_max_table_m required in registered table frame");
    auto duration_ns=[this](const char* name,double value,double maximum) {
      const double seconds=declare_parameter<double>(name,value);
      if(!std::isfinite(seconds) || seconds<0 || seconds>maximum)
        throw std::invalid_argument(std::string("invalid timing parameter: ")+name);
      return static_cast<std::int64_t>(std::llround(seconds*1e9));
    };
    c.max_age_ns=duration_ns("max_forecast_source_age_s",.030,.030);
    c.grace_ns=duration_ns("selection_receive_grace_s",.010,.030);
    c.min_lead_ns=duration_ns("min_remaining_lead_s",.050,2.);
    c.incoming.net_x=declare_parameter<double>("net_x_m",1.37);
    c.incoming.commit_delay_s=declare_parameter<double>("commit_delay_s",.05);
    if(!std::isfinite(c.incoming.net_x) || c.incoming.net_x!=1.37 ||
       !std::isfinite(c.incoming.commit_delay_s) || c.incoming.commit_delay_s<0 ||
       c.incoming.commit_delay_s>.2) throw std::invalid_argument("invalid one-shot geometry/timing");
    const bool registered=declare_parameter<bool>("source_clock_registered",false) &&
      declare_parameter<bool>("frame_registration_confirmed",false);
    if(!registered || ((get_clock()->get_clock_type()==RCL_ROS_TIME &&
        get_parameter("use_sim_time").as_bool()) != (c.clock=="ros_sim")))
      throw std::invalid_argument("source/frame/ROS clock must be explicitly registered");
    frame_=c.frame;min_lead_=c.min_lead_ns;
    nominal_base_y_=declare_parameter<double>("nominal_base_y_m",std::numeric_limits<double>::quiet_NaN());
    const auto world=declare_parameter<std::int64_t>("base_world_frame_id",0);
    if(world<=0)throw std::invalid_argument("base_world_frame_id required even in tagged tests");
    base_world_=world;
    if(declare_parameter<bool>("fallback_plane_confirmed",false)) {
      SelectorFrozenPlane plane;
      plane.x=declare_parameter<double>("fallback_hit_plane_x_m",std::numeric_limits<double>::quiet_NaN());
      const auto generation=declare_parameter<std::int64_t>("fallback_plane_generation",0);
      const auto calibration=declare_parameter<std::int64_t>("fallback_calibration_id",0);
      if(generation<=0 || calibration<0)throw std::invalid_argument("invalid frozen fallback plane identity");
      plane.generation=generation;plane.calibration=calibration;
      plane.synthetic=declare_parameter<bool>("fallback_known_test_plane",false);
      plane.world=plane.synthetic?0:base_world_;c.frozen_plane=plane;
    }
    index_=declare_parameter<int>("ball_pose_index",0);
    const int depth=declare_parameter<int>("input_depth",256);
    if(index_<0 || depth<1 || depth>4096)throw std::invalid_argument("invalid pose subscription");
    if(declare_parameter<double>("x_hit_bh_delta_m",0.)!=0.)
      throw std::invalid_argument("one shared frozen plane required");
    offset_=declare_parameter<double>("policy_z_offset_m",.76);
    split_=declare_parameter<double>("swing_side_split_y",-.25);
    hysteresis_=declare_parameter<double>("swing_side_hysteresis_y",.04);
    land_fh_=declare_parameter<double>("target_land_y_fh",-.7625);
    land_bh_=declare_parameter<double>("target_land_y_bh",-.7625);
    duration_fh_=declare_parameter<double>("delta_t_flight_fh",.50);
    duration_bh_=declare_parameter<double>("delta_t_flight_bh",.50);
    if(!std::isfinite(offset_) || !std::isfinite(split_) || !std::isfinite(hysteresis_) || hysteresis_<0 ||
       !std::isfinite(land_fh_) || !std::isfinite(land_bh_) ||
       !std::isfinite(duration_fh_) || !std::isfinite(duration_bh_) || duration_fh_<=0 || duration_bh_<=0 ||
       duration_fh_>2. || duration_bh_>2.)
      throw std::invalid_argument("invalid deployment settings");
    PlannerConfig p;
    p.target_land.x()=declare_parameter<double>("target_land_x",2.055);
    BallPhysics physics; TableParams table;
    physics.drag_k=declare_parameter<double>("drag_k",physics.drag_k);
    p.restitution_racket=declare_parameter<double>("restitution_racket",p.restitution_racket);
    p.restitution_exp_g1=declare_parameter<double>("restitution_exp_g1",p.restitution_exp_g1);
    p.restitution_exp_g2=declare_parameter<double>("restitution_exp_g2",p.restitution_exp_g2);
    if(!std::isfinite(p.target_land.x()) || !std::isfinite(physics.drag_k) || physics.drag_k<0 ||
       !std::isfinite(p.restitution_racket) || !std::isfinite(p.restitution_exp_g1) ||
       !std::isfinite(p.restitution_exp_g2))throw std::invalid_argument("invalid stage3 settings");
    land_x_=p.target_land.x();planner_=std::make_unique<RacketTargetPlanner>(physics,p,table);
    c.physics=physics;c.planner=p;c.planner.adaptive_predict_horizon=true;
    selector_=std::make_unique<StrikeTargetSelector>(c);
    // Fixed names plus resolved-name checks reject remapping to any other output.
    check_output("/planner/shadow_racket_command");
    check_output("/planner/shadow_racket_command_flat");
    check_output("/planner/strike_target_selector/diagnostics");
    typed_=create_publisher<hope_msgs::msg::RacketCommand>("/planner/shadow_racket_command",10);
    const bool publish_flat=declare_parameter<bool>("publish_shadow_flat",false);
    if(publish_flat && c.clock!="ros_system")
      throw std::invalid_argument("schema-2 wall deadline requires ros_system");
    if(publish_flat)
      flat_=create_publisher<std_msgs::msg::Float64MultiArray>("/planner/shadow_racket_command_flat",10);
    diagnostics_=create_publisher<std_msgs::msg::String>("/planner/strike_target_selector/diagnostics",10);
    pose_=create_subscription<geometry_msgs::msg::PoseArray>(
      declare_parameter<std::string>("input_pose_topic","/poses"),rclcpp::QoS(depth).best_effort(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){
        if(!enabled_)return;
        BallSample s;s.source_time_ns=rclcpp::Time(m->header.stamp).nanoseconds();
        if(static_cast<std::size_t>(index_)<m->poses.size()) {
          auto& p=m->poses[index_].position;s.position=Vec3(p.x,p.y,p.z);
        } else s.position.setConstant(std::numeric_limits<double>::quiet_NaN());
        s.receipt_wall_ns=now().nanoseconds();s.receipt_steady_ns=steady_ns();
        const auto previous=selector_->flight_id();
        selector_->observe(s,m->header.frame_id,s.receipt_wall_ns,s.receipt_steady_ns);
        if(previous!=selector_->flight_id()) {
          std_msgs::msg::String event;
          event.data="transition=incoming_started physical_flight_id="+std::to_string(selector_->flight_id())+
            " new_flight_rearmed="+(previous?"1":"0")+" source_ns="+std::to_string(s.source_time_ns)+
            " receipt_ns="+std::to_string(s.receipt_wall_ns);
          diagnostics_->publish(event);
        }
        if(pose_audit_)RCLCPP_INFO(get_logger(),"pose_observed source_ns=%ld flight=%lu phase=%s state=%s",
          s.source_time_ns,selector_->flight_id(),selector_->phase(),selector_->reason().c_str());
      });
    forecast_=create_subscription<hope_msgs::msg::BallForecast>(
      declare_parameter<std::string>("forecast_event_topic","/ball/forecast/v1/event"),rclcpp::QoS(512).reliable(),
      [this](hope_msgs::msg::BallForecast::ConstSharedPtr m){if(enabled_)selector_->forecast(*m,now().nanoseconds());});
    const auto full_topic=declare_parameter<std::string>("forecast_trajectory_topic","/ball/forecast/v1/trajectory");
    if(!full_topic.empty())full_=create_subscription<hope_msgs::msg::BallForecast>(full_topic,rclcpp::QoS(32).reliable(),
      [this](hope_msgs::msg::BallForecast::ConstSharedPtr m){if(enabled_)selector_->full_forecast(*m,now().nanoseconds());});
    context_=create_subscription<hope_msgs::msg::HybridTrajectoryContext>(
      declare_parameter<std::string>("context_topic","/ball/hybrid/context"),rclcpp::QoS(32).reliable(),
      [this](hope_msgs::msg::HybridTrajectoryContext::ConstSharedPtr m){if(enabled_) {
        const auto before=selector_->session_id();selector_->context(*m,now().nanoseconds());
        if(before!=selector_->session_id()) {
          base_valid_=base_ever_valid_=false;base_integrity_fault_=false;
          std_msgs::msg::String ack;ack.data="transition=session_reset source_ns="+std::to_string(m->source_time_ns);
          diagnostics_->publish(ack);
        }
      }});
    base_validator_=std::make_unique<HitPlaneManager>(HitPlaneSettings{},c.frame,c.registration,base_world_);
    base_=create_subscription<std_msgs::msg::Float64MultiArray>(
      declare_parameter<std::string>("base_pose_topic","/a3/base_pose_flat"),10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){
        base_valid_=base_validator_->observe(m->data,steady_ns(),now().nanoseconds(),true);
        if(m->data.size()==16 && std::isfinite(m->data[15]) && m->data[15]!=static_cast<double>(base_world_))base_integrity_fault_=true;
        if(base_valid_) {base_ever_valid_=true;base_session_=selector_->session_id();base_y_=m->data[6];base_stamp_=static_cast<std::int64_t>(m->data[3])*1000000000LL+
          static_cast<std::int64_t>(m->data[4]);base_calibration_=static_cast<std::uint64_t>(m->data[14]);}
      });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{poll();});
  }
 private:
  void check_output(const std::string& name) {
    if(get_node_topics_interface()->resolve_topic_name(name)!=name)
      throw std::invalid_argument("shadow output remapping forbidden");
  }
  void poll() {
    if(!enabled_)return;
    const auto start=steady_ns();auto selected=selector_->poll(now().nanoseconds(),start);
    if(!selected) {
      if(selector_->reason()!=last_reason_) {
        last_reason_=selector_->reason();
        std_msgs::msg::String diagnostic;
        diagnostic.data="state="+last_reason_+" physical_flight_id="+std::to_string(selector_->flight_id())+" halted="+(selector_->halted()?"true":"false");
        diagnostics_->publish(diagnostic);
      }
      return;
    }
    auto& s=*selected;RacketCommand command;
    const bool valid_crossing=s.strike.valid;
    bool attempted=false;double stage3_ms=0;
    std::string base_source="unavailable";
    const auto age=static_cast<long double>(now().nanoseconds())-base_stamp_;
    if(base_ever_valid_ && base_session_==s.session_id)base_source=(base_valid_ && age>=0 && age<=200000000)?"registered_current":"registered_retained";
    else if(std::isfinite(nominal_base_y_)) {base_y_=nominal_base_y_;base_source="nominal_configured";}
    if(s.strike.valid && (base_integrity_fault_ || (!s.known_test_plane && base_ever_valid_ &&
       (s.world_frame_id!=base_world_ || s.calibration_id!=base_calibration_)))) {
      s.strike.valid=false;s.reason="base_reference_mismatched";
    }
    if(s.strike.valid && base_source=="unavailable") {s.strike.valid=false;s.reason="base_y_unavailable";}
    const double contact_y_table=s.strike.ball_position.y();
    if(s.strike.valid && !strike_target_selector_hit_y_allowed(contact_y_table,hit_y_min_table_m_,hit_y_max_table_m_)) {
      s.strike.valid=false;
      s.reason=std::isfinite(contact_y_table)?"hit_y_out_of_range":"nonfinite_contact_y";
    }
    if(s.strike.valid) {
      sign_=strike_target_selector_swing(s.strike.ball_position.y(),base_y_,sign_,split_,hysteresis_);
      const auto began=steady_ns();attempted=true;
      command=planner_->plan(s.strike,Vec3(land_x_,sign_>0?land_fh_:land_bh_,0),sign_>0?duration_fh_:duration_bh_);
      stage3_ms=(steady_ns()-began)*1e-6;
      if(!command.valid)s.reason="stage3_rejected:"+command.reason;
    }
    const bool solver_valid=command.valid;
    const auto emit=now().nanoseconds();const auto hit=s.absolute_hit_ns;
    const double tts=hit?static_cast<double>((static_cast<long double>(hit)-emit)*1e-9L):0;
    if(command.valid && hit<=emit) {command.valid=false;s.reason="deadline_expired_before_publish";}
    Schema2Packet packet;bool flat_published=false;
    if(command.valid && flat_) {
      packet=Schema2Packer::pack(&command,sign_,command.strike_source_time_s,offset_,emit,
        WireIdentity{sequence_+1,s.physical_flight_id,1},s.sample_count,s.sample_span_s);
      packet.values[9]=tts;
      if(!packet.valid) {command.valid=false;s.reason="wire_deadline_or_numeric_precision";}
    }
    hope_msgs::msg::RacketCommand out;out.header.stamp=rclcpp::Time(emit);out.header.frame_id=frame_;
    if(command.valid) {
      xyz(out.position,command.position);out.position.z+=offset_;xyz(out.velocity,command.velocity);
      xyz(out.normal,command.normal);xyz(out.ball_velocity_outgoing,command.outgoing_ball_velocity);
      out.strike_time=command.strike_source_time_s;out.time_to_strike=tts;out.valid=true;
      out.clears_net=command.clears_net;out.bypasses_net_posts=command.bypasses_net_posts;out.predicted_bounces=command.predicted_bounces;
      if(flat_) {
        ++sequence_;std_msgs::msg::Float64MultiArray msg;msg.data.assign(packet.values.begin(),packet.values.end());flat_->publish(msg);flat_published=true;
      }
    }
    // One visible decision even when no executable numerical target exists.
    typed_->publish(out);
    const double prediction_age=s.source_ns?static_cast<double>((static_cast<long double>(s.selected_ns)-s.source_ns)*1e-6L):-1;
    const double emission_age=s.source_ns?static_cast<double>((static_cast<long double>(emit)-s.source_ns)*1e-6L):-1;
    std::string warnings=tts>0 && tts*1e9<min_lead_?"late_lead":"none";
    if(prediction_age>30)warnings+=",stale_estimate";
    if(s.quality.find("legacy_low_height")!=std::string::npos)warnings+=",legacy_low_height";
    std::ostringstream trace;trace<<std::setprecision(17)
      <<"transition=decision_emitted commit_due=1 fatal_identity_reset="<<selector_->halted()
      <<" reason="<<s.reason<<" command_valid="<<command.valid<<" solver_valid="<<solver_valid
      <<" stage3_attempted="<<attempted<<" valid_crossing="<<valid_crossing<<" typed_published=1 flat_published="<<flat_published
      <<" contact_y_table_m="<<contact_y_table<<" hit_y_min_table_m="<<hit_y_min_table_m_
      <<" hit_y_max_table_m="<<hit_y_max_table_m_
      <<" physical_flight_id="<<s.physical_flight_id<<" session_id="<<s.session_id
      <<" rally_id="<<s.forecast.identity.rally_id<<" owner="<<s.forecast.publisher_instance_id
      <<" config="<<s.forecast.identity.return_region_config_id<<" plane="<<s.plane_generation<<" plane_x="<<s.plane_x
      <<" net_ns="<<s.net_ns<<" commit_ns="<<s.commit_ns<<" commit_cutoff_ns="<<s.commit_ns<<" source_ns="<<s.source_ns
      <<" observation_receipt_ns="<<s.observation_receipt_ns<<" forecast_publish_ns="<<s.forecast.publish_ros_ns<<" forecast_receipt_ns="<<s.forecast_receipt_ns
      <<" revision="<<s.forecast.identity.revision_id<<" hit_ns="<<hit<<" selected_ns="<<s.selected_ns
      <<" emit_ns="<<emit<<" tts="<<tts<<" lead_ms="<<tts*1000<<" swing_sign="<<sign_
      <<" verified_contacts="<<s.verified_contacts<<" predicted_bounces="<<s.strike.predicted_bounces
      <<" origin="<<s.origin<<" fallback_used="<<s.fallback_used<<" eligibility_status="<<s.eligibility_status
      <<" base_y_source="<<base_source<<" prediction_age_ms="<<prediction_age<<" emission_age_ms="<<emission_age
      <<" estimator_samples="<<s.sample_count<<" estimator_span_s="<<s.sample_span_s
      <<" late_acquisition="<<s.late_acquisition<<" warnings="<<warnings
      <<" estimator_ms="<<s.estimator_ms<<" rollout_ms="<<s.rollout_ms<<" stage3_ms="<<stage3_ms
      <<" quality="<<s.quality<<" ball_position="<<s.strike.ball_position.transpose()<<" ball_velocity="<<s.strike.ball_velocity.transpose()
      <<" racket_position="<<command.position.transpose()<<" racket_velocity="<<command.velocity.transpose()
      <<" normal="<<command.normal.transpose()<<" runtime_us="<<(steady_ns()-start)*1e-3;
    std_msgs::msg::String diagnostic;diagnostic.data=trace.str();diagnostics_->publish(diagnostic);
    RCLCPP_INFO(get_logger(),"%s",diagnostic.data.c_str());
  }

  bool pose_audit_=false;
  bool enabled_=false,base_valid_=false,base_ever_valid_=false,base_integrity_fault_=false;
  double nominal_base_y_=std::numeric_limits<double>::quiet_NaN();
  double hit_y_min_table_m_=0,hit_y_max_table_m_=0;
  std::uint64_t base_session_=0;
  int index_=0;std::int64_t base_stamp_=0,min_lead_=0;
  std::uint64_t sequence_=0,base_world_=0,base_calibration_=0;
  double offset_=0,base_y_=0,sign_=0,split_=0,hysteresis_=0,land_x_=0,land_fh_=0,land_bh_=0,duration_fh_=0,duration_bh_=0;
  std::string frame_,last_reason_;
  std::unique_ptr<StrikeTargetSelector> selector_;
  std::unique_ptr<RacketTargetPlanner> planner_;
  std::unique_ptr<HitPlaneManager> base_validator_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_;
  rclcpp::Subscription<hope_msgs::msg::BallForecast>::SharedPtr forecast_,full_;
  rclcpp::Subscription<hope_msgs::msg::HybridTrajectoryContext>::SharedPtr context_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_;
  rclcpp::Publisher<hope_msgs::msg::RacketCommand>::SharedPtr typed_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flat_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<hope_planner_cpp::StrikeTargetSelectorNode>());}
  catch(const std::exception& e) {std::cerr<<"strike_target_selector: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
