#include "hope_msgs/msg/flight_context_v2.hpp"
#include "hope_planner_cpp/flight_transport_v2.hpp"
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
    full_rate_hz_ = declare_parameter("full_output_rate_hz", 50.);
    packet_rate_hz_ = declare_parameter("packet_output_rate_hz", 50.);
    worker_delay_ms_ = declare_parameter("test.worker_delay_ms", 0);
    worker_delay_every_ = declare_parameter("test.worker_delay_every", 100);
    if (!std::isfinite(full_rate_hz_) || full_rate_hz_ < 0 ||
        full_rate_hz_ > 300 || !std::isfinite(packet_rate_hz_) ||
        packet_rate_hz_ < 0 || packet_rate_hz_ > 300 || worker_delay_ms_ < 0 ||
        worker_delay_ms_ > 1000 || worker_delay_every_ < 1)
      throw std::invalid_argument("invalid forecast cadence/test delay");
    // Zero selects every completed latest result, with bounded existing
    // backpressure.
    config_.trajectory_dt_s =
        declare_parameter("trajectory_dt_s", config_.trajectory_dt_s);
    for (const auto &entry : std::vector<std::pair<std::string, double>>{
             {"physics.gravity_x", config_.physics.gravity.x()},
             {"physics.gravity_y", config_.physics.gravity.y()},
             {"physics.gravity_z", config_.physics.gravity.z()},
             {"table.length", config_.table.length},
             {"table.width", config_.table.width},
             {"table.y_max", config_.table.y_max},
             {"table.net_x", config_.table.net_x},
             {"table.net_height", config_.table.net_height},
             {"table.net_overhang", config_.table.net_overhang}})
      set_flight_parameter(config_, settings_, entry.first,
                           declare_parameter(entry.first, entry.second));
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
    }
    pipeline_ = std::make_unique<FlightPipeline>(config_, settings_);
    std::ostringstream bundle;
    bundle.imbue(std::locale::classic());
    bundle << std::hexfloat;
    bundle << flight_model_config_id(config_, settings_, frame_)
           << ";flight-v2-rk4;conditional-uncertainty;plane=" << policy;
    for (double value :
         {plane_settings.offset_m, plane_settings.window_s,
          plane_settings.max_age_s, plane_settings.max_span_m,
          plane_settings.min_x_m, plane_settings.max_x_m,
          settings_.max_source_age_s, full_rate_hz_, packet_rate_hz_})
      bundle << ';' << value;
    bundle << ';' << settings_.service_armed << ';' << clock_domain_;
    config_id_ = "flight-v2:sha256:" + sha256(bundle.str());
    for (const auto *topic :
         {"/hope/shadow/flight_v2/prediction", "/hope/shadow/flight_v2/packet",
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
    for (const auto &topic : {std::string(publisher_->get_topic_name()),
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
          if (!sample.source_time_valid || !sample.position_valid)
            return;
          if (sample.frame_id != frame_) {
            pipeline_->invalidate("registered_frame_mismatch");
            revoke_locked();
            return;
          }
          if (sample.clock_domain != clock_domain_) {
            mapping_valid_ = false;
            pipeline_->invalidate("source_clock_domain_mismatch");
            revoke_locked("source_clock_domain_mismatch");
            return;
          }
          if (!sample.clock_mapping_valid) {
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
          if (!pipeline_->shot().active() ||
              pipeline_->shot().samples().empty() ||
              pipeline_->shot().samples().back().source_time_ns !=
                  sample.source_time_ns)
            return;
          // Sequence is reserved before copying; ordinary observations never
          // supersede semantic context.
          pipeline_->shot().snapshot();
          Job job{*pipeline_,
                  plane_->for_solve(steady_ns(), now().nanoseconds()),
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
          const bool pose_valid = plane_->observe(
              m->data, steady_ns(), now().nanoseconds(), mapping_valid_);
          auto current = plane_->for_solve(steady_ns(), now().nanoseconds());
          const bool incompatible =
              plane_->reason() == "base_world_frame_mismatch" ||
              (old.calibrated && pose_valid && m->data.size() == 16 &&
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
      std::uint64_t solves = 0;
      while (auto job = work_.take()) {
        if (worker_delay_ms_ && ++solves % worker_delay_every_ == 0)
          std::this_thread::sleep_for(
              std::chrono::milliseconds(worker_delay_ms_));
        auto result = job->pipeline.forecast(job->plane, job->source);
        std::unique_lock<std::mutex> guard(mutex_);
        const auto current = now().nanoseconds();
        if (!mapping_valid_ || !pipeline_->accept(result.token, current))
          continue;
        pipeline_->shot().predict_events(result.events, result.reference_ns);
        auto wire = encode_prediction(result, pipeline_->shot());
        wire.forecast_revision = pipeline_->shot().snapshot().solve_sequence;
        wire.session_id = session_;
        wire.producer_id = producer_;
        wire.config_id = config_id_;
        wire.model_config_id = flight_model_config_id(
            job->pipeline.config(), job->pipeline.settings(), frame_);
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
        // Publish compact output before full serialization; keep the original
        // immutable source state for all channels.
        auto points = std::move(wire.waypoints);
        publisher_->publish(wire);
        wire.waypoints = std::move(points);
        if (due(result.reference_ns, full_rate_hz_, next_full_source_) &&
            publication_fresh())
          full_->publish(wire);
        last_expiry_ = wire.expires_at_ns;
        if (due(result.reference_ns, packet_rate_hz_, next_packet_source_)) {
          const auto ledger = wire.events;
          guard.unlock();
          auto packet = encode_packet_v2(job->pipeline, result, session_,
                                         producer_, config_id_);
          packet.event_ledger = ledger;
          packet.payload_sha256 = packet_v2_hash(packet);
          guard.lock();
          const long double age =
              (static_cast<long double>(now().nanoseconds()) -
               result.reference_ns) *
              1e-9L;
          if (pipeline_->shot().active() &&
              pipeline_->shot().generation() == result.token.generation &&
              pipeline_->shot().revision() == result.token.context_revision &&
              age >= -.001 && age <= settings_.max_source_age_s)
            packet_->publish(packet);
        }
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
    last_expiry_.reset();
    hope_msgs::msg::FlightPredictionV2 message;
    message.session_id = session_;
    message.producer_id = producer_;
    message.config_id = config_id_;
    message.shot_generation = pipeline_->shot().generation();
    message.shot_id = message.shot_generation;
    message.forecast_revision = pipeline_->shot().snapshot().solve_sequence;
    message.generated_at_ns = message.published_at_ns = now().nanoseconds();
    message.context_revision = pipeline_->shot().revision();
    message.play_state = name(pipeline_->shot().state());
    message.forecast_status = "REVOKED";
    message.reason = reason;
    message.input_count = inputs_;
    message.superseded_count = work_.superseded();
    publisher_->publish(message);
  }
  static bool due(std::int64_t source, double rate,
                  std::optional<std::int64_t> &next) {
    if (rate == 0)
      return true;
    const auto period = static_cast<std::int64_t>(std::llround(1e9 / rate));
    if (!next || static_cast<long double>(source) <
                     static_cast<long double>(*next) - period)
      next = source;
    if (source < *next)
      return false;
    const long double future =
        static_cast<long double>(*next) +
        period * (1 + std::floor((static_cast<long double>(source) - *next) /
                                 period));
    next = future > std::numeric_limits<std::int64_t>::max()
               ? source
               : static_cast<std::int64_t>(future);
    return true;
  }
  std::mutex mutex_;
  ProbabilisticConfig config_;
  ShotSettings settings_;
  std::unique_ptr<FlightPipeline> pipeline_;
  std::unique_ptr<HitPlaneManager> plane_;
  LatestWork<Job> work_;
  std::thread worker_;
  std::optional<std::int64_t> last_expiry_, next_full_source_,
      next_packet_source_, last_boundary_source_;
  std::string frame_, session_, authority_, config_id_, producer_,
      clock_domain_;
  double full_rate_hz_ = 50., packet_rate_hz_ = 50.;
  int worker_delay_ms_ = 0, worker_delay_every_ = 100;
  bool mapping_valid_ = true, synthetic_ = false;
  std::uint64_t inputs_ = 0;
  rclcpp::Publisher<hope_msgs::msg::FlightPredictionV2>::SharedPtr publisher_;
  rclcpp::Publisher<hope_msgs::msg::FlightPacketV2>::SharedPtr packet_;
  rclcpp::Publisher<hope_msgs::msg::FlightPredictionV2>::SharedPtr full_;
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
