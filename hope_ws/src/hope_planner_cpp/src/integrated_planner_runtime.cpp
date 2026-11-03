#include "hope_planner_cpp/integrated_planner_runtime.hpp"
#include "hope_planner_cpp/integrated_command_lifecycle.hpp"
#include "hope_planner_cpp/flight_transport_v2.hpp"
#include "hope_planner_cpp/target_turn_guard.hpp"
#include "hope_planner_cpp/base_pose_orientation.hpp"
#include "hope_planner_cpp/bounded_stage_trace.hpp"
#include "hope_planner_cpp/ros_context_guard.hpp"
#include "hope_msgs/msg/flight_context_v2.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <hope_msgs/msg/racket_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <thread>

namespace hope_planner_cpp {
namespace {
std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<class T> void xyz(T &to,const Vec3 &from) { to.x=from.x();to.y=from.y();to.z=from.z(); }
}
struct IntegratedPlannerRuntime::Impl {
  struct Base {
    Vec3 position=Vec3::Zero();
    Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
    std::int64_t source_ns=0,receipt_ns=0;
    std::uint64_t calibration=0;
    bool valid=false;
  };
  struct Job { FlightPipeline pipeline; HitPlaneSnapshot plane; Base base; std::uint64_t epoch; };
  rclcpp::Node &node;
  ProbabilisticConfig config;
  ShotSettings settings;
  std::unique_ptr<FlightPipeline> pipeline;
  std::unique_ptr<HitPlaneManager> plane;
  std::unique_ptr<ValidatedReturnPlanner> planner;
  IntegratedCommandLifecycle lifecycle;
  LatestWork<Job> work;
  BoundedStageTrace accounting;
  BoundedStageTrace lifecycle_accounting;
  std::mutex mutex;
  std::thread worker;
  Base base;
  std::string frame,clock,registration,ball_id,clock_mapping,profile;
  std::string reason="WAITING_BALL_AND_BASE", game="UNKNOWN";
  std::uint64_t epoch=0,inputs=0,accepted=0,commands=0,rejected=0;
  std::int64_t last_observation_ns=0,last_context_ns=std::numeric_limits<std::int64_t>::min();
  double policy_z=0.76,max_turn=40,min_lead=.08,max_reach=.95,split_y=-.25;
  int worker_delay_ms=0,first_worker_delay_ms=0;
  std::uint64_t stale_work_discarded=0,last_discarded_generation=0,generation_at_discard=0;
  bool pending_valid=false,return_eligible=false,source_fault=false;
  bool outgoing_valid=false,execution_admitted=false;
  double last_solver_ms=0,last_source_age_ms=0;
  bool physical_available=false;
  std::int64_t physical_source_ns=0;
  std::string physical_reason="WAITING", return_reason="WAITING";
  rclcpp::Subscription<hope_msgs::msg::BallObservationV2>::SharedPtr observation;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_sub;
  rclcpp::Subscription<hope_msgs::msg::FlightContextV2>::SharedPtr context;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flat;
  rclcpp::Publisher<hope_msgs::msg::RacketCommand>::SharedPtr rich;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr trace;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refresh;

  explicit Impl(rclcpp::Node &n):node(n) {
    auto string=[&](const char *key,const char *value) { return node.declare_parameter<std::string>(key,value); };
    accounting.configure(string("test.stage_trace_path",""));
    lifecycle_accounting.configure(string("test.lifecycle_trace_path",""),8192);
    if (string("return_backend","shared_physics")!="shared_physics")
      throw std::invalid_argument("V2 requires shared_physics; legacy ablations use offline evaluation");
    if (string("output_mode","shadow")!="shadow")
      throw std::invalid_argument("integrated live promotion requires measured simulation/physical qualification; use shadow");
    frame=string("frame_id","world"); clock=string("source_clock_domain","ros_system");
    registration=string("registration_id","UNQUALIFIED_SHADOW");
    ball_id=string("ball_id","Ball"); clock_mapping=string("clock_mapping_id","natnet_exposure_unix_v1");
    profile=string("runner_contract","schema2_ball_center_velocity_normal_v1");
    if (profile!="schema2_ball_center_velocity_normal_v1" || frame.empty() ||
        clock!="ros_system" || registration.empty() || ball_id.empty() || clock_mapping.empty() ||
        node.get_clock()->get_clock_type()!=RCL_ROS_TIME || node.declare_parameter("use_sim_epoch",false))
      throw std::invalid_argument("unsupported runner/source contract: wire requires registered Unix source time");
    policy_z=node.declare_parameter("policy_z_offset",.76);
    max_turn=node.declare_parameter("target_turn_max_deg",40.);
    min_lead=node.declare_parameter("minimum_lead_s",.08);
    max_reach=node.declare_parameter("maximum_reach_m",.95);
    split_y=node.declare_parameter("swing_side_split_y",-.25);
    worker_delay_ms=node.declare_parameter("test.worker_delay_ms",0);
    first_worker_delay_ms=node.declare_parameter("test.first_worker_delay_ms",0);
    if (node.declare_parameter("x_hit_bh_delta",0.)!=0)
      throw std::invalid_argument("nonzero backhand plane offset is unsupported by integrated profile");
    for (double value : {policy_z,max_turn,min_lead,max_reach,split_y})
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite execution settings");
    if (max_turn<=0 || max_turn>40 || min_lead<.08 || min_lead>.5 ||
        max_reach<=0 || max_reach>1.2 || worker_delay_ms<0 || worker_delay_ms>1000 ||
        first_worker_delay_ms<0 || first_worker_delay_ms>1000)
      throw std::invalid_argument("unsupported execution limits");
    settings.game_legality_advisory=true;
    settings.automatic_rearm=node.declare_parameter("automatic_rearm_enabled",true);
    config.integration_dt_s=node.declare_parameter("integration_dt_s",.001);
    config.horizon_s=node.declare_parameter("horizon_s",1.5);
    config.physics.drag_k=node.declare_parameter("physics.drag_k",config.physics.drag_k);
    config.physics.magnus_k=node.declare_parameter("physics.magnus_k",config.physics.magnus_k);
    config.physics.restitution_v=node.declare_parameter("physics.restitution_v",config.physics.restitution_v);
    config.validate(); settings.validate();
    HitPlaneSettings ps;
    const auto pp=string("plane_policy","FOLLOW_POSE");
    if (pp!="FOLLOW_POSE" && pp!="ON_REFRESH") throw std::invalid_argument("unknown plane policy");
    ps.policy=pp=="FOLLOW_POSE"?PlanePolicy::FOLLOW_POSE:PlanePolicy::ON_REFRESH;
    ps.offset_m=node.declare_parameter("plane_offset_m",.58);
    const auto world=node.declare_parameter<std::int64_t>("world_frame_id",1);
    if (world<=0 || world>static_cast<std::int64_t>(IntegratedCommandLifecycle::max_wire_integer))
      throw std::invalid_argument("invalid world receipt ID");
    plane=std::make_unique<HitPlaneManager>(ps,frame,registration,world);
    pipeline=std::make_unique<FlightPipeline>(config,settings);
    ValidatedReturnSettings rs;
    rs.landing_surface=Vec3(node.declare_parameter("target_land_x",2.055),
        node.declare_parameter("target_land_y",-.7625),0);
    rs.durations_s=node.declare_parameter<std::vector<double>>("outgoing_durations_s",{.5,.4,.6,.7});
    planner=std::make_unique<ValidatedReturnPlanner>(config,rs);
    constexpr auto prefix="/hope/shadow/integrated_planner/";
    for (const auto *suffix : {"racket_target","command","status","trace"}) {
      const auto topic=std::string(prefix)+suffix;
      if (node.get_node_topics_interface()->resolve_topic_name(topic)!=topic)
        throw std::invalid_argument("integrated shadow outputs cannot be remapped");
    }
    flat=node.create_publisher<std_msgs::msg::Float64MultiArray>(std::string(prefix)+"racket_target",rclcpp::QoS(1).reliable());
    rich=node.create_publisher<hope_msgs::msg::RacketCommand>(std::string(prefix)+"command",rclcpp::QoS(1).best_effort());
    status=node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(std::string(prefix)+"status",rclcpp::QoS(1).reliable());
    trace=node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(std::string(prefix)+"trace",rclcpp::QoS(10).best_effort());
    observation=node.create_subscription<hope_msgs::msg::BallObservationV2>(
        string("observation_topic","/hope/ball/observation_v2"),rclcpp::QoS(8).best_effort(),
        [this](hope_msgs::msg::BallObservationV2::ConstSharedPtr msg) { on_observation(*msg); });
    base_sub=node.create_subscription<std_msgs::msg::Float64MultiArray>(
        string("base_pose_topic","/a3/base_pose_flat"),rclcpp::QoS(10).best_effort(),
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex);
          const auto now=node.now().nanoseconds(), steady=monotonic_ns();
          if (!plane->observe(msg->data,steady,now,true)) {
            if (plane->reason()=="base_world_frame_mismatch") plane->revoke("BASE_WORLD_CHANGED");
            base.valid=false; ++epoch; invalidate_locked(plane->reason()); return;
          }
          const auto &w=msg->data;
          if (base.calibration && base.calibration!=static_cast<std::uint64_t>(w[14])) {
            plane->revoke("BASE_REGISTRATION_CHANGED");++epoch; invalidate_locked("BASE_REGISTRATION_CHANGED");
          }
          base.position=Vec3(w[5],w[6],w[7]);
          if (static_cast<unsigned>(w[13])&16u) base.position.z()-=policy_z;
          base.orientation=*base_orientation_wxyz(w);
          base.source_ns=static_cast<std::int64_t>(w[3])*1000000000LL+static_cast<std::int64_t>(w[4]);
          base.receipt_ns=steady; base.calibration=static_cast<std::uint64_t>(w[14]);base.valid=true;
          if (!pipeline->shot().active() && !plane->snapshot().calibrated)
            plane->freeze(steady,now,0,true);
        });
    // Game context has no authority to reset physical tracking. Physical
    // boundaries come from the measured input stream, never a control decision.
    const auto context_authority=string("context_authority","referee");
    context=node.create_subscription<hope_msgs::msg::FlightContextV2>(
        "/hope/integrated_planner/game_context",rclcpp::QoS(4).reliable(),
        [this,context_authority](hope_msgs::msg::FlightContextV2::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex);
          if (m->schema_version==2 && m->authority==context_authority && m->operation=="CONTEXT" &&
              m->shot_generation==pipeline->shot().generation() && m->source_time_ns<=last_observation_ns &&
              m->source_time_ns>last_context_ns &&
              (m->match_context=="SERVE" || m->match_context=="NORMAL_RALLY" || m->match_context=="UNKNOWN")) {
            game=m->match_context; last_context_ns=m->source_time_ns;
          }
        });
    refresh=node.create_service<std_srvs::srv::Trigger>("/hope/integrated_planner/refresh_plane",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lock(mutex);
          res->success=plane->freeze(monotonic_ns(),node.now().nanoseconds(),pipeline->shot().generation(),
              !pending_valid && lifecycle.refresh_idle(node.now().nanoseconds()));
          res->message=plane->reason(); if (res->success) { ++epoch; invalidate_locked("PLANE_REFRESHED"); }
        });
    timer=node.create_wall_timer(std::chrono::milliseconds(20),[this] {
      with_active_ros_context(node.get_node_base_interface()->get_context(), [this] { report(); });
    });
    worker=std::thread([this] { run(); });
  }
  ~Impl() {
    timer.reset(); work.close(); if (worker.joinable()) worker.join();
    try { accounting.flush(); lifecycle_accounting.flush(); }
    catch (const std::exception &e) { RCLCPP_ERROR(node.get_logger(), "%s", e.what()); }
  }
  bool base_fresh(std::int64_t now,std::int64_t steady) const {
    const long double age=static_cast<long double>(now)-base.source_ns;
    return base.valid && age>=-1000000.L && age<=200000000.L &&
        steady>=base.receipt_ns && steady-base.receipt_ns<=200000000LL;
  }
  void invalidate_locked(const std::string &why) {
    reason=why;execution_admitted=false;
    if (!pending_valid) return;
    if (lifecycle.committed()) { pending_valid=false;return; }
    const auto packet=Schema2Packer::pack(nullptr,1,0,policy_z,node.now().nanoseconds(),lifecycle.invalidation(),0,0);
    std_msgs::msg::Float64MultiArray wire; wire.data.assign(packet.values.begin(),packet.values.end());flat->publish(wire);
    pending_valid=false;
  }
  void on_observation(const hope_msgs::msg::BallObservationV2 &message) {
    const auto receipt=monotonic_ns();
    accounting.record("planner_receipt",message.source_time_ns,message.sequence,0,
                      node.now().nanoseconds(),receipt);
    auto sample=decode_observation(message);sample.receipt_steady_ns=monotonic_ns();
    const auto waiting=monotonic_ns();
    std::lock_guard<std::mutex> lock(mutex); ++inputs;
    const auto now=node.now().nanoseconds();
    accounting.record("planner_lock",sample.source_time_ns,sample.sequence,pipeline->shot().generation(),
                      now,monotonic_ns(),monotonic_ns()-waiting);
    auto rejected_stage=[&](const std::string &why) {
      accounting.record("planner_reject",sample.source_time_ns,sample.sequence,pipeline->shot().generation(),
                        node.now().nanoseconds(),monotonic_ns(),0,why);
    };
    if (!sample.position_valid || !sample.source_time_valid || !sample.position.allFinite() ||
        sample.frame_id!=frame || sample.clock_domain!=clock || sample.ball_id!=ball_id ||
        sample.calibration_id!=registration || sample.clock_mapping_id!=clock_mapping ||
        !sample.clock_mapping_valid || !std::isfinite(sample.clock_uncertainty_s) ||
        sample.clock_uncertainty_s<0 || sample.clock_uncertainty_s>.001 ||
        static_cast<long double>(now)-sample.source_time_ns>30000000.L ||
        static_cast<long double>(now)-sample.source_time_ns < -1000000.L) {
      ++rejected; ++epoch;source_fault=true;return_eligible=false;outgoing_valid=false;
      physical_available=false;physical_reason="SOURCE_UNQUALIFIED";
      pipeline->shot().revoke_outputs("SOURCE_UNQUALIFIED");
      invalidate_locked("SOURCE_UNQUALIFIED"); rejected_stage("SOURCE_UNQUALIFIED"); return;
    }
    if (last_observation_ns && sample.source_time_ns<=last_observation_ns) {
      ++rejected;rejected_stage("SOURCE_REORDERED");return;
    }
    source_fault=false;
    last_observation_ns=sample.source_time_ns;
    const auto generation=pipeline->shot().generation();
    const auto revision=pipeline->shot().revision();
    pipeline->observe(sample);
    if (lifecycle_accounting.enabled()) lifecycle_accounting.record(name(pipeline->shot().state()),sample.source_time_ns,
        sample.sequence,pipeline->shot().generation(),node.now().nanoseconds(),monotonic_ns(),0,
        pipeline->shot().active() ? pipeline->shot().reason() : pipeline->shot().rejection_reason());
    accounting.record("pipeline_observed",sample.source_time_ns,sample.sequence,pipeline->shot().generation(),
                      node.now().nanoseconds(),monotonic_ns(),monotonic_ns()-waiting,pipeline->shot().reason());
    if (generation!=pipeline->shot().generation()) {
      physical_available=false;
      ++epoch; invalidate_locked("PHYSICAL_SHOT_CHANGED"); game="UNKNOWN";
      last_context_ns=std::numeric_limits<std::int64_t>::min();
    } else if (revision!=pipeline->shot().revision()) {
      ++epoch;invalidate_locked("PHYSICAL_CONTEXT_CHANGED");
    }
    if (!pipeline->shot().active() || pipeline->shot().samples().empty() ||
        pipeline->shot().samples().back().source_time_ns!=sample.source_time_ns) {
      rejected_stage(pipeline->shot().reason());return;
    }
    ++accepted;
    if (!base_fresh(now,sample.receipt_steady_ns)) {
      invalidate_locked("BASE_UNAVAILABLE");rejected_stage("BASE_UNAVAILABLE");return;
    }
    const auto copying=monotonic_ns();
    work.push(Job{*pipeline,plane->for_solve(sample.receipt_steady_ns,now),base,epoch});
    accounting.record("queued",sample.source_time_ns,sample.sequence,pipeline->shot().generation(),
                      node.now().nanoseconds(),monotonic_ns(),monotonic_ns()-copying);
  }
  void run() {
    bool first_solve=true;
    while (auto job=work.take()) {
      const auto &source=job->pipeline.shot().samples().back();
      auto stage=[&](std::string_view label,std::int64_t duration=0,std::string_view why={}) {
        accounting.record(label,source.source_time_ns,source.sequence,job->pipeline.shot().generation(),
                          node.now().nanoseconds(),monotonic_ns(),duration,why);
      };
      try {
        const auto started=monotonic_ns();
        stage("worker_start");
        auto forecast=job->pipeline.forecast(job->plane,node.now().nanoseconds());
        stage("forecast_end",monotonic_ns()-started,forecast.reason);
        auto eligibility=select_physical_return(forecast,job->pipeline.shot().export_snapshot(),config,node.now().nanoseconds());
        const auto solving=monotonic_ns();
        ValidatedReturn result;
        if (eligibility.input) result=planner->solve(*eligibility.input);
        stage("solve_end",monotonic_ns()-solving,result.reason);
        const int delay=first_solve && first_worker_delay_ms ? first_worker_delay_ms : worker_delay_ms;
        first_solve=false;
        if (delay) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        const auto locking=monotonic_ns();
        std::lock_guard<std::mutex> lock(mutex);
        stage("worker_lock",monotonic_ns()-locking);
        const auto now=node.now().nanoseconds(),steady=monotonic_ns();
        last_solver_ms=(steady-started)*1e-6;
        last_source_age_ms=(static_cast<long double>(now)-forecast.reference_ns)*1e-6L;
        if (job->epoch!=epoch || forecast.token.generation!=pipeline->shot().generation() ||
            forecast.token.context_revision!=pipeline->shot().revision()) {
          ++stale_work_discarded;last_discarded_generation=forecast.token.generation;
          generation_at_discard=pipeline->shot().generation();stage("worker_reject",0,"RETIRED_EPOCH");continue;
        }
        physical_available=forecast.physical_available;
        physical_source_ns=forecast.reference_ns;
        physical_reason=forecast.reason;
        return_reason=eligibility.reason;
        return_eligible=eligibility.eligible;
        outgoing_valid=result.return_ok;
        if (!eligibility.input) { invalidate_locked(eligibility.reason);stage("worker_reject",0,eligibility.reason);continue; }
        if (!result.command.valid) { invalidate_locked(result.reason);stage("worker_reject",0,result.reason);continue; }
        auto current_plane=plane->for_solve(steady,now);
        if (!base_fresh(now,steady) || !current_plane.calibrated ||
            current_plane.calibration_id!=job->plane.calibration_id ||
            std::abs(current_plane.x_hit_m-job->plane.x_hit_m)>.001) {
          invalidate_locked("BASE_OR_PLANE_SUPERSEDED");stage("worker_reject",0,"BASE_OR_PLANE_SUPERSEDED");continue;
        }
        const auto &input=*eligibility.input;
        const auto turn=check_target_turn(result.command.position,base.position,base.orientation,true,max_turn);
        const Vec3 reach=result.command.position-base.position;
        if (!turn.allowed || reach.head<2>().norm()>max_reach ||
            result.command.position.z()<.05 || result.command.position.z()>1.2) {
          invalidate_locked(turn.allowed?"REACH_UNSUPPORTED":turn.reason);stage("worker_reject",0,reason);continue;
        }
        const auto admission_ns=node.now().nanoseconds();
        auto identity=lifecycle.admit(input,result,pipeline->shot().generation(),admission_ns,min_lead);
        if (!identity) {
          if (!lifecycle.committed()) invalidate_locked("TIMING_OR_REVISION_REJECTED");
          stage("worker_reject",0,"TIMING_OR_REVISION_REJECTED");continue;
        }
        const double swing=lifecycle.latch_swing(result.command.position.y()-base.position.y()>=split_y?1.:-1.);
        auto packet=Schema2Packer::pack(&result.command,swing,static_cast<double>(input.arrival_ns)*1e-9,
            policy_z,admission_ns,*identity,input.observation_count,input.observation_span_s);
        if (!packet.valid) { invalidate_locked("ENCODING_FAILED");stage("worker_reject",0,"ENCODING_FAILED");continue; }
        std_msgs::msg::Float64MultiArray wire;wire.data.assign(packet.values.begin(),packet.values.end());
        const auto publication_begin_ns=node.now().nanoseconds();
        stage("publication_begin");
        flat->publish(wire);
        const auto published_ns=node.now().nanoseconds();
        last_source_age_ms=(static_cast<long double>(published_ns)-input.source.source_time_ns)*1e-6L;
        // Command sequence joins the actual receiver; source_ns joins ingress.
        accounting.record("published",input.source.source_time_ns,identity->command_sequence,input.token.generation,
                          published_ns,monotonic_ns(),0,std::to_string(input.arrival_ns));
        hope_msgs::msg::RacketCommand view;view.header.stamp=node.now();view.header.frame_id=frame;
        xyz(view.position,result.command.position);xyz(view.velocity,result.command.velocity);
        xyz(view.normal,result.command.normal);xyz(view.ball_velocity_outgoing,result.reconstructed.velocity);
        view.valid=true;view.clears_net=result.command.clears_net;view.strike_time=packet.values[10];
        view.time_to_strike=packet.values[9];rich->publish(view);
        ++commands;pending_valid=true;execution_admitted=true;reason="SHADOW_COMMAND_VALID";
        diagnostic_msgs::msg::DiagnosticArray receipt;receipt.header.stamp=node.now();
        diagnostic_msgs::msg::DiagnosticStatus detail;detail.name="integrated_planner/command_trace";
        detail.message=reason;detail.hardware_id="non_actuating_shadow";
        auto record=[&](const std::string &key,const std::string &value) {
          diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;detail.values.push_back(kv);
        };
        record("source_time_ns",std::to_string(input.source.source_time_ns));
        record("producer_time_ns",std::to_string(admission_ns));record("arrival_ns",std::to_string(input.arrival_ns));
        record("publication_begin_ns",std::to_string(publication_begin_ns));
        record("publication_time_ns",std::to_string(published_ns));
        record("command_sequence",std::to_string(identity->command_sequence));
        record("physical_shot",std::to_string(input.token.generation));record("wire_flight",std::to_string(identity->flight_id));
        record("plane_generation",std::to_string(input.plane.generation));record("model_config_id",result.model_config_id);
        record("game_history",input.history==HistoryContinuity::kComplete?"COMPLETE":
            input.history==HistoryContinuity::kIncomplete?"INCOMPLETE":"UNKNOWN");
        record("bounce_observed",input.bounce_observed?"true":"false");record("game_context",game);
        record("source_age_ms",std::to_string(last_source_age_ms));record("solve_ms",std::to_string(last_solver_ms));
        record("solver_residual_m",std::to_string(result.solver_residual_m));
        record("impact_residual_mps",std::to_string(result.impact_residual_mps));
        record("landing_x",std::to_string(result.first_landing.x()));record("landing_y",std::to_string(result.first_landing.y()));
        const auto readiness=job->pipeline.shot().spin_readiness();
        record("spin_readiness",readiness.state);
        record("spin_measurement_ns",std::to_string(readiness.measurement_time_ns));
        record("spin_collected_span_s",std::to_string(readiness.span_s));
        record("spin_collected_increments",std::to_string(readiness.increments));
        record("acquisition_state",job->pipeline.shot().acquisition_state());
        record("source_origin_ns",std::to_string(job->pipeline.source_origin_ns()));
        record("base_yaw_deg",std::to_string(turn.base_yaw_deg));
        record("target_bearing_deg",std::to_string(turn.target_bearing_deg));
        record("turn_deg",std::to_string(turn.turn_deg));
        record("outgoing_duration_s",std::to_string(result.selected_duration_s));
        record("net_clearance_m",std::to_string(result.net_clearance_m));record("outgoing_spin","POSITION_ONLY");
        receipt.status.push_back(detail);trace->publish(receipt);
        stage("diagnostics_end",monotonic_ns()-steady);
      } catch (const std::exception &e) {
        stage("worker_exception",0,e.what());
        if (!node.get_node_base_interface()->get_context()->is_valid()) break;
        std::lock_guard<std::mutex> lock(mutex);++epoch;invalidate_locked(std::string("WORKER_FAILURE:")+e.what());
      }
    }
  }
  void report() {
    std::lock_guard<std::mutex> lock(mutex);
    const auto now=node.now().nanoseconds(),steady=monotonic_ns();
    if (!last_observation_ns || static_cast<long double>(now)-last_observation_ns>30000000.L) {
      return_eligible=false;outgoing_valid=false;
      physical_available=false;
    }
    if (source_fault) {return_eligible=false;outgoing_valid=false;invalidate_locked("SOURCE_UNQUALIFIED");}
    else if (!base_fresh(now,steady)) invalidate_locked("BASE_UNAVAILABLE");
    else if (!last_observation_ns || static_cast<long double>(now)-last_observation_ns>30000000.L)
      {return_eligible=false;outgoing_valid=false;invalidate_locked("SOURCE_EXPIRED");}
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=node.now();
    diagnostic_msgs::msg::DiagnosticStatus d;d.name="integrated_planner/input_preflight";
    d.hardware_id="non_actuating_shadow";d.message=reason;d.level=pending_valid?0:1;
    auto add=[&](const std::string &key,const std::string &value) {
      diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;d.values.push_back(kv);
    };
    add("prediction_backend","v2");add("return_backend","shared_physics");add("output_mode","shadow");
    add("source_frame",frame);add("registration_id",registration);add("runner_contract",profile);
    add("observation_topic",observation->get_topic_name());add("base_pose_topic",base_sub->get_topic_name());
    add("observation_history_depth",std::to_string(observation->get_actual_qos().get_rmw_qos_profile().depth));
    add("publication_reserve_ns",std::to_string(IntegratedCommandLifecycle::publication_reserve_ns));
    add("observation_publishers",std::to_string(node.count_publishers(observation->get_topic_name())));
    add("base_publishers",std::to_string(node.count_publishers(base_sub->get_topic_name())));
    add("game_context",game);add("game_context_required","false");
    add("game_history",pipeline->shot().continuity()==HistoryContinuity::kComplete?"COMPLETE":
        pipeline->shot().continuity()==HistoryContinuity::kIncomplete?"INCOMPLETE":"UNKNOWN");
    add("return_eligible",return_eligible?"true":"false");add("command_valid",pending_valid?"true":"false");
    add("outgoing_return_valid",outgoing_valid?"true":"false");
    add("physical_available",physical_available?"true":"false");
    add("physical_source_ns",std::to_string(physical_source_ns));
    add("physical_reason",physical_reason);add("return_reason",return_reason);
    add("physical_shot",std::to_string(pipeline->shot().generation()));
    add("physical_state",name(pipeline->shot().state()));
    add("observed_contacts",std::to_string(pipeline->shot().observed_contacts()));
    add("spin_mode",std::to_string(static_cast<int>(pipeline->shot().spin().mode)));
    const auto readiness=pipeline->shot().spin_readiness();
    add("spin_readiness",readiness.state); add("spin_reason",readiness.reason);
    add("spin_collected_span_s",std::to_string(readiness.span_s));
    add("spin_collected_increments",std::to_string(readiness.increments));
    add("spin_measurement_ns",std::to_string(readiness.measurement_time_ns));
    add("acquisition_state",pipeline->shot().acquisition_state());
    add("acquisition_reason",pipeline->shot().reason());
    add("rejection_reason",pipeline->shot().rejection_reason());
    add("rejection_source_ns",std::to_string(pipeline->shot().rejection_source_ns()));
    add("automatic_rearm_enabled",settings.automatic_rearm?"true":"false");
    add("latest_source_ns",std::to_string(last_observation_ns));
    add("candidate_samples",std::to_string(pipeline->shot().candidates().size()));
    add("retained_samples",std::to_string(pipeline->shot().samples().size()));
    add("source_origin_ns",std::to_string(pipeline->source_origin_ns()));
    const auto forward=base.orientation*Vec3::UnitX();
    add("base_yaw_deg",std::to_string(std::atan2(forward.y(),forward.x())*180./3.141592653589793));
    add("execution_admitted",execution_admitted?"true":"false");
    add("inputs",std::to_string(inputs));add("accepted",std::to_string(accepted));add("rejected",std::to_string(rejected));
    add("commands",std::to_string(commands));add("superseded",std::to_string(work.superseded()));
    add("stale_work_discarded",std::to_string(stale_work_discarded));
    add("last_discarded_generation",std::to_string(last_discarded_generation));
    add("generation_at_discard",std::to_string(generation_at_discard));
    add("solver_ms",std::to_string(last_solver_ms));add("source_age_ms",std::to_string(last_source_age_ms));
    array.status.push_back(d);status->publish(array);
  }
};
IntegratedPlannerRuntime::IntegratedPlannerRuntime(rclcpp::Node &n):impl_(std::make_unique<Impl>(n)) {}
IntegratedPlannerRuntime::~IntegratedPlannerRuntime()=default;
} // namespace hope_planner_cpp
