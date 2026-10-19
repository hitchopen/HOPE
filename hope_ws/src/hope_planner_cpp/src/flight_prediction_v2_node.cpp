#include "hope_msgs/msg/flight_context_v2.hpp"
#include "hope_msgs/msg/flight_decision_v2.hpp"
#include "hope_planner_cpp/flight_transport_v2.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/reach_permission.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/target_turn_guard.hpp"
#include <chrono>
#include <iomanip>
#include <locale>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

namespace hope_planner_cpp {
namespace {
std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
template <class T> void put(T &out, const Vec3 &v) {
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
}
} // namespace
class FlightPredictionV2Node : public rclcpp::Node {
public:
  FlightPredictionV2Node() : Node("hope_flight_prediction_v2") {
    if (!declare_parameter("shadow_only", true))
      throw std::invalid_argument("v2 is shadow-only pending qualification");
    frame_ = declare_parameter<std::string>("frame_id", "table");
    clock_domain_ =
        declare_parameter<std::string>("source_clock_domain", "ros_system");
    session_ = declare_parameter<std::string>(
        "session_id", "shadow_" + std::to_string(steady_ns()));
    producer_ = "flight_v2_" + std::to_string(steady_ns());
    authority_ = declare_parameter<std::string>("context_authority", "fixture");
    config_.integration_dt_s = declare_parameter("integration_dt_s", .001);
    config_.horizon_s = declare_parameter("horizon_s", 1.5);
    config_.physics.drag_k =
        declare_parameter("physics.drag_k", config_.physics.drag_k);
    config_.physics.magnus_k =
        declare_parameter("physics.magnus_k", config_.physics.magnus_k);
    config_.physics.restitution_v = declare_parameter(
        "physics.restitution_v", config_.physics.restitution_v);
    config_.physics.table_tangential_gain = declare_parameter(
        "physics.table_tangential_gain", config_.physics.table_tangential_gain);
    config_.physics.table_friction_cap_mu = declare_parameter(
        "physics.table_friction_cap_mu", config_.physics.table_friction_cap_mu);
    config_.physics.radius =
        declare_parameter("physics.radius", config_.physics.radius);
    settings_.max_source_age_s = declare_parameter("max_source_age_s", .030);
    settings_.service_armed = declare_parameter("service_armed", false);
    min_y_ = declare_parameter("planner.min_y_m", -1.20);
    max_y_ = declare_parameter("planner.max_y_m", -.30);
    min_lead_s_ = declare_parameter("planner.min_lead_s", .050);
    policy_z_offset_ = declare_parameter("planner.policy_z_offset_m", .76);
    max_turn_deg_ = declare_parameter("planner.max_turn_deg", 40.);
    if (!std::isfinite(policy_z_offset_) || !std::isfinite(max_turn_deg_) ||
        max_turn_deg_ <= 0 || max_turn_deg_ > 180)
      throw std::invalid_argument("invalid command adapter geometry");
    if (!std::isfinite(min_y_) || !std::isfinite(max_y_) || min_y_ >= max_y_ ||
        !std::isfinite(min_lead_s_) || min_lead_s_ < 0)
      throw std::invalid_argument("invalid planner provisional bounds");
    HitPlaneSettings plane_settings;
    const auto policy =
        declare_parameter<std::string>("plane_policy", "FOLLOW_POSE");
    if (policy != "FOLLOW_POSE" && policy != "ON_REFRESH")
      throw std::invalid_argument("unknown plane policy");
    plane_settings.policy = policy == "FOLLOW_POSE" ? PlanePolicy::FOLLOW_POSE
                                                    : PlanePolicy::ON_REFRESH;
    plane_settings.offset_m = declare_parameter("plane_offset_m", .58);
    const auto registration =
        declare_parameter<std::string>("registration_id", "UNQUALIFIED_SHADOW");
    plane_ = std::make_unique<HitPlaneManager>(
        plane_settings, frame_, registration,
        declare_parameter<int>("world_frame_id", 1));
    synthetic_ = declare_parameter("synthetic_plane", false);
    if (synthetic_) {
      plane_->use_test_plane(declare_parameter("test_plane_x_m", .15),
                             now().nanoseconds());
      base_position_ = Vec3(plane_->snapshot().base_x_m,
                            declare_parameter("test_base_y_m", -.7), 0);
      base_valid_ = true;
    }
    pipeline_ = std::make_unique<FlightPipeline>(config_, settings_);
    std::ostringstream bundle;
    bundle.imbue(std::locale::classic());
    bundle << std::hexfloat;
    bundle << return_region_config_identity(config_, {}, frame_)
           << ";flight-v2-rk4;conditional-uncertainty;plane=" << policy;
    for (double value : {plane_settings.offset_m, plane_settings.window_s,
                         plane_settings.max_age_s, plane_settings.max_span_m,
                         plane_settings.min_x_m, plane_settings.max_x_m,
                         settings_.max_source_age_s, min_y_, max_y_,
                         min_lead_s_, policy_z_offset_, max_turn_deg_})
      bundle << ';' << value;
    bundle << ';' << settings_.service_armed << ';' << clock_domain_;
    config_id_ = "flight-v2:sha256:" + sha256(bundle.str());
    for (const auto *topic :
         {"/hope/shadow/flight_v2/prediction",
          "/hope/shadow/flight_v2/decision", "/hope/shadow/flight_v2/packet",
          "/hope/shadow/flight_v2/trajectory"}) {
      if (get_node_topics_interface()->resolve_topic_name(topic).rfind(
              "/hope/shadow/flight_v2/", 0) != 0)
        throw std::invalid_argument(
            "remapping must retain v2 shadow namespace");
    }
    auto qos = rclcpp::QoS(1).best_effort();
    publisher_ = create_publisher<hope_msgs::msg::FlightPredictionV2>(
        "/hope/shadow/flight_v2/prediction", qos);
    full_ = create_publisher<hope_msgs::msg::FlightPredictionV2>(
        "/hope/shadow/flight_v2/trajectory", qos);
    packet_ = create_publisher<hope_msgs::msg::FlightPacketV2>(
        "/hope/shadow/flight_v2/packet", qos);
    decision_ = create_publisher<hope_msgs::msg::FlightDecisionV2>(
        "/hope/shadow/flight_v2/decision", qos);
    for (const auto &topic : {std::string(publisher_->get_topic_name()),
                              std::string(decision_->get_topic_name()),
                              std::string(packet_->get_topic_name()),
                              std::string(full_->get_topic_name())})
      if (topic.rfind("/hope/shadow/flight_v2/", 0) != 0)
        throw std::invalid_argument(
            "remapping must retain v2 shadow namespace");
    observation_ = create_subscription<hope_msgs::msg::BallObservationV2>(
        "/hope/ball/observation_v2", qos,
        [this](hope_msgs::msg::BallObservationV2::ConstSharedPtr message) {
          auto sample = decode_observation(*message);
          sample.receipt_steady_ns = steady_ns();
          std::lock_guard<std::mutex> guard(mutex_);
          ++inputs_;
          if (sample.frame_id != frame_) {
            pipeline_->invalidate("registered_frame_mismatch");
            revoke_locked();
            return;
          }
          if (!sample.clock_mapping_valid ||
              sample.clock_domain != clock_domain_) {
            mapping_valid_ = false;
            pipeline_->shot().revoke_outputs("clock_mapping_unavailable");
            revoke_locked("clock_mapping_unavailable");
            return;
          }
          mapping_valid_ = true;
          const auto previous_revision = pipeline_->shot().revision(),
                     previous_generation = pipeline_->shot().generation();
          pipeline_->observe(sample);
          if (previous_generation != pipeline_->shot().generation() ||
              previous_revision != pipeline_->shot().revision())
            revoke_locked(pipeline_->shot().reason());
          // Sequence is reserved before copying; ordinary observations never
          // supersede semantic context.
          pipeline_->shot().snapshot();
          Job job{*pipeline_,
                  plane_->for_solve(steady_ns(), sample.source_time_ns),
                  sample.source_time_ns};
          work_.push(std::move(job));
        });
    context_ = create_subscription<hope_msgs::msg::FlightContextV2>(
        "/hope/shadow/flight_v2/context", rclcpp::QoS(32).reliable(),
        [this](hope_msgs::msg::FlightContextV2::ConstSharedPtr m) {
          std::lock_guard<std::mutex> guard(mutex_);
          if (m->schema_version != 2 || m->session_id != session_ ||
              m->authority != authority_)
            return;
          const auto current_source = now().nanoseconds();
          if (static_cast<long double>(m->source_time_ns) - current_source >
              1000000.L)
            return;
          if (!pipeline_->shot().samples().empty()) {
            const auto latest =
                pipeline_->shot().samples().back().source_time_ns;
            if ((m->operation == "CONTEXT" && m->source_time_ns > latest) ||
                (m->operation == "CONTACT" && m->confirmed_at_ns > latest))
              return;
          }
          if (m->operation == "START") {
            if (last_boundary_source_ &&
                m->source_time_ns <= *last_boundary_source_) {
              const bool clock_restart =
                  m->source_time_ns < *last_boundary_source_ &&
                  m->boundary_evidence == "RESTART" &&
                  m->shot_generation == pipeline_->shot().generation();
              if (!clock_restart)
                return;
            }
            BoundaryEvidence evidence;
            if (m->boundary_evidence == "RACKET_CONTACT")
              evidence = BoundaryEvidence::RACKET_CONTACT;
            else if (m->boundary_evidence == "FREE_FLIGHT_LAUNCH")
              evidence = BoundaryEvidence::FREE_FLIGHT_LAUNCH;
            else if (m->boundary_evidence == "RESTART")
              evidence = BoundaryEvidence::RESTART;
            else if (m->boundary_evidence == "VERIFIED_CORRECTION")
              evidence = BoundaryEvidence::VERIFIED_CORRECTION;
            else
              return;
            if (m->direction != 1 && m->direction != -1)
              return;
            last_boundary_source_ = m->source_time_ns;
            pipeline_->start(m->source_time_ns, m->direction, evidence,
                             m->history_continuity == "complete"
                                 ? HistoryContinuity::kComplete
                                 : HistoryContinuity::kUnknown);
          } else if (m->operation == "RESET")
            pipeline_->invalidate("explicit_reset");
          else if (m->operation == "CONTEXT") {
            const bool accepted = pipeline_->shot().context(
                m->match_context == "SERVE" ? MatchContext::SERVE
                : m->match_context == "NORMAL_RALLY"
                    ? MatchContext::NORMAL_RALLY
                    : MatchContext::UNKNOWN,
                m->source_time_ns, m->authority, m->shot_generation);
            if (accepted && m->history_continuity == "complete")
              pipeline_->shot().set_history_continuity(
                  HistoryContinuity::kComplete);
            else if (accepted && m->history_continuity == "incomplete")
              pipeline_->shot().set_history_continuity(
                  HistoryContinuity::kIncomplete);
          } else if (m->operation == "CONTACT") {
            if (m->shot_generation != pipeline_->shot().generation())
              return;
            PhysicalEventKind kind;
            if (m->event_kind == "table_bounce")
              kind = PhysicalEventKind::TABLE;
            else if (m->event_kind == "net_collision")
              kind = PhysicalEventKind::NET_COLLISION;
            else if (m->event_kind == "net_crossing")
              kind = PhysicalEventKind::NET_CROSSING;
            else
              return;
            ModelState pre;
            pre.position = Vec3(m->position.x, m->position.y, m->position.z);
            pre.velocity =
                Vec3(m->pre_velocity.x, m->pre_velocity.y, m->pre_velocity.z);
            pipeline_->shot().confirm(kind, m->event_time_ns,
                                      m->confirmed_at_ns, pre, m->authority);
          } else if (m->operation == "REFRESH_PLANE")
            plane_->freeze(steady_ns(), m->source_time_ns,
                           pipeline_->shot().generation(),
                           !pipeline_->shot().active());
          revoke_locked();
        });
    base_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hope/robot/base_pose", qos,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
          std::lock_guard<std::mutex> guard(mutex_);
          const auto old = plane_->snapshot();
          if (plane_->observe(m->data, steady_ns(), now().nanoseconds(),
                              mapping_valid_)) {
            base_position_ = Vec3(m->data[5], m->data[6], m->data[7]);
            base_orientation_ = Eigen::Quaterniond(m->data[11], m->data[8],
                                                   m->data[9], m->data[10])
                                    .normalized();
            base_source_ns_ =
                static_cast<std::int64_t>(m->data[3]) * 1000000000LL +
                static_cast<std::int64_t>(m->data[4]);
            base_valid_ = true;
          } else
            base_valid_ = false;
          auto current = plane_->for_solve(steady_ns(), now().nanoseconds());
          const bool incompatible =
              plane_->reason() == "base_world_frame_mismatch" ||
              (old.calibrated && base_valid_ && m->data.size() == 16 &&
               old.calibration_id != static_cast<std::uint64_t>(m->data[14]));
          if (incompatible ||
              (old.calibrated && current.calibrated &&
               (old.world_frame_id != current.world_frame_id ||
                old.calibration_id != current.calibration_id))) {
            plane_->revoke("plane_registration_changed");
            pipeline_->invalidate("plane_registration_changed");
            revoke_locked();
          }
        });
    expiry_ = create_wall_timer(std::chrono::milliseconds(5), [this] {
      std::lock_guard<std::mutex> guard(mutex_);
      if (last_expiry_ && now().nanoseconds() >= *last_expiry_) {
        last_expiry_.reset();
        revoke_locked("forecast_expired");
      }
    });
    worker_ = std::thread([this] {
      while (auto job = work_.take()) {
        auto result = job->pipeline.forecast(job->plane, job->source);
        std::lock_guard<std::mutex> guard(mutex_);
        const auto current = now().nanoseconds();
        if (!mapping_valid_ || !pipeline_->accept(result.token, current))
          continue;
        pipeline_->shot().predict_events(result.events, result.reference_ns);
        auto wire = encode_prediction(result, pipeline_->shot());
        wire.session_id = session_;
        wire.producer_id = producer_;
        wire.config_id = config_id_;
        wire.context_source = job->pipeline.shot().context_source();
        wire.context_time_ns = job->pipeline.shot().context_time_ns();
        wire.generated_at_ns = current;
        wire.published_at_ns = now().nanoseconds();
        wire.input_count = inputs_;
        wire.superseded_count = work_.superseded();
        const auto publication_fresh = [&] {
          wire.published_at_ns = now().nanoseconds();
          const long double age =
              (static_cast<long double>(wire.published_at_ns) -
               wire.prediction_reference_time_ns) *
              1e-9L;
          return age >= -.001 && age <= settings_.max_source_age_s;
        };
        if (!publication_fresh()) {
          revoke_locked("source_expired_before_publication");
          continue;
        }
        // The job owns the exact posterior/reference; newer callback samples
        // remain in the next job.
        if (!last_full_source_ ||
            static_cast<long double>(result.reference_ns) -
                    *last_full_source_ >=
                20000000) {
          full_->publish(wire);
          last_full_source_ = result.reference_ns;
          auto packet = encode_packet_v2(job->pipeline, result, session_,
                                         producer_, config_id_);
          packet.event_ledger = wire.events;
          packet.payload_sha256 = packet_v2_hash(packet);
          packet_->publish(packet);
        }
        wire.waypoints.clear();
        if (!publication_fresh()) {
          revoke_locked("source_expired_before_publication");
          continue;
        }
        publisher_->publish(wire);
        last_expiry_ = wire.expires_at_ns;
        planner_locked(wire, now().nanoseconds());
      }
    });
  }
  ~FlightPredictionV2Node() override {
    work_.close();
    if (worker_.joinable())
      worker_.join();
  }

private:
  struct Job {
    FlightPipeline pipeline;
    HitPlaneSnapshot plane;
    std::int64_t source;
  };
  void
  revoke_locked(const std::string &reason = "semantic_context_superseded") {
    hope_msgs::msg::FlightPredictionV2 message;
    message.session_id = session_;
    message.producer_id = producer_;
    message.config_id = config_id_;
    message.shot_generation = pipeline_->shot().generation();
    message.context_revision = pipeline_->shot().revision();
    message.play_state = name(pipeline_->shot().state());
    message.forecast_status = "REVOKED";
    message.reason = reason;
    message.input_count = inputs_;
    message.superseded_count = work_.superseded();
    publisher_->publish(message);
    if (dispatched_generation_) {
      hope_msgs::msg::FlightDecisionV2 d;
      d.session_id = session_;
      d.shot_generation = *dispatched_generation_;
      d.previously_dispatched = true;
      d.reason = "revoked;hardware_cancellation_unqualified";
      decision_->publish(d);
    }
  }
  void planner_locked(const hope_msgs::msg::FlightPredictionV2 &m,
                      std::int64_t current) {
    hope_msgs::msg::FlightDecisionV2 decision;
    decision.session_id = session_;
    decision.shot_generation = m.shot_generation;
    decision.context_revision = m.context_revision;
    decision.forecast_revision = m.forecast_revision;
    decision.predicted_arrival_time_ns = m.predicted_arrival_time_ns;
    decision.planner_now_ns = current;
    decision.lead_time_s = double(
        (static_cast<long double>(m.predicted_arrival_time_ns) - current) *
        1e-9L);
    decision.previously_dispatched =
        dispatched_generation_ && *dispatched_generation_ == m.shot_generation;
    decision.reason = "target_unavailable";
    if (m.expires_at_ns < current)
      decision.reason = "forecast_expired_at_planner";
    else if (m.target_valid && m.target_incoming && !m.target_historical) {
      if (decision.previously_dispatched)
        decision.reason = "one_shot_already_selected";
      else if (m.target_position.y < min_y_ || m.target_position.y > max_y_)
        decision.reason = "provisional_contact_y_bounds";
      else if (decision.lead_time_s < min_lead_s_)
        decision.reason = "planner_insufficient_lead";
      else {
        const Vec3 position(m.target_position.x, m.target_position.y,
                            m.target_position.z);
        const long double base_age =
            (static_cast<long double>(current) - base_source_ns_) * 1e-9L;
        const auto turn = check_target_turn(
            position, base_position_, base_orientation_,
            base_valid_ &&
                (synthetic_ || (base_age >= -.001 && base_age <= .2)),
            max_turn_deg_);
        const auto reach = classify_reach_permission(
            ReachPermissionConfig{}, m.target_position.y, base_position_.y(),
            1., decision.lead_time_s);
        if (!turn.allowed || !reach.target_admitted) {
          decision.reason = !turn.allowed ? turn.reason : "reach_rejected";
          decision_->publish(decision);
          return;
        }
        StrikeTarget target;
        target.valid = true;
        target.ball_position =
            Vec3(m.target_position.x, m.target_position.y, m.target_position.z);
        target.ball_velocity =
            Vec3(m.target_velocity.x, m.target_velocity.y, m.target_velocity.z);
        target.strike_source_time_s = m.predicted_arrival_time_ns * 1e-9;
        for (const auto &event : m.events)
          if (event.kind == "table_bounce" && event.status == "PREDICTED" &&
              event.event_time_ns > m.prediction_reference_time_ns &&
              event.event_time_ns <= m.predicted_arrival_time_ns)
            ++target.predicted_bounces;
        const auto command =
            RacketTargetPlanner(config_.physics, {}, config_.table)
                .plan(target, Vec3(2.055, -.7625, 0), .5);
        decision.admitted = command.valid;
        decision.reason = command.reason;
        decision.command.valid = command.valid;
        put(decision.command.position, command.position);
        put(decision.command.velocity, command.velocity);
        put(decision.command.normal, command.normal);
        put(decision.command.ball_velocity_outgoing,
            command.outgoing_ball_velocity);
        decision.command.strike_time = target.strike_source_time_s;
        decision.command.time_to_strike = decision.lead_time_s;
        decision.command.clears_net = command.clears_net;
        decision.command.bypasses_net_posts = command.bypasses_net_posts;
        decision.command.predicted_bounces = command.predicted_bounces;
        decision.command.header.frame_id = frame_;
        decision.command.header.stamp = rclcpp::Time(current);
        decision.policy_z_offset_m = policy_z_offset_;
        decision.frame_contract =
            "table_xy_policy_world_z_offset_once;ball_center_reference;paddle_"
            "registration_unqualified";
        if (command.valid) {
          const auto packed = Schema2Packer::pack(
              &command, 1., command.strike_source_time_s, policy_z_offset_,
              current, WireIdentity{m.forecast_revision, m.shot_generation, 1},
              0, 0);
          decision.flat_preview = packed.values;
          decision.flat_preview[9] = decision.lead_time_s;
          decision.command.position.z += policy_z_offset_;
          if (!packed.valid) {
            decision.admitted = false;
            decision.command.valid = false;
            decision.reason = "schema2_preview_rejected";
          } else
            dispatched_generation_ = m.shot_generation;
        }
      }
    }
    decision_->publish(decision);
  }
  std::mutex mutex_;
  ProbabilisticConfig config_;
  ShotSettings settings_;
  std::unique_ptr<FlightPipeline> pipeline_;
  std::unique_ptr<HitPlaneManager> plane_;
  LatestWork<Job> work_;
  std::thread worker_;
  std::optional<std::uint64_t> dispatched_generation_;
  std::optional<std::int64_t> last_expiry_, last_full_source_,
      last_boundary_source_;
  std::string frame_, session_, authority_, config_id_, producer_,
      clock_domain_;
  double min_y_, max_y_, min_lead_s_, policy_z_offset_, max_turn_deg_;
  bool mapping_valid_ = true, synthetic_ = false, base_valid_ = false;
  Vec3 base_position_ = Vec3::Zero();
  Eigen::Quaterniond base_orientation_ = Eigen::Quaterniond::Identity();
  std::int64_t base_source_ns_ = 0;
  std::uint64_t inputs_ = 0;
  rclcpp::Publisher<hope_msgs::msg::FlightPredictionV2>::SharedPtr publisher_;
  rclcpp::Publisher<hope_msgs::msg::FlightPacketV2>::SharedPtr packet_;
  rclcpp::Publisher<hope_msgs::msg::FlightPredictionV2>::SharedPtr full_;
  rclcpp::Publisher<hope_msgs::msg::FlightDecisionV2>::SharedPtr decision_;
  rclcpp::Subscription<hope_msgs::msg::BallObservationV2>::SharedPtr
      observation_;
  rclcpp::Subscription<hope_msgs::msg::FlightContextV2>::SharedPtr context_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_;
  rclcpp::TimerBase::SharedPtr expiry_;
};
} // namespace hope_planner_cpp
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hope_planner_cpp::FlightPredictionV2Node>());
  } catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
