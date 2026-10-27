// Native replacement for hope_monitor.py. Same ROS graph; no motion on startup.
#include "monitor_core.hpp"
#include "network_addresses.hpp"
#include <atomic>
#include <random>
#include <set>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#ifdef HOPE_MONITOR_VENDOR_RPC
#include <ros2_plugin_proto/srv/ros_rpc_wrapper.hpp>
#endif

namespace hm = hope_monitor;
using Bool = std_msgs::msg::Bool;
using Float = std_msgs::msg::Float64;
using Array = std_msgs::msg::Float64MultiArray;
using String = std_msgs::msg::String;
using Pose = geometry_msgs::msg::PoseStamped;
using Trigger = std_srvs::srv::Trigger;
using Marker = visualization_msgs::msg::Marker;
using TF = tf2_msgs::msg::TFMessage;
using namespace std::chrono_literals;

static std::string identity() {
  std::random_device random; std::ostringstream out;
  for (int i = 0; i < 4; ++i) out << std::hex << std::setw(8) << std::setfill('0') << random();
  return out.str();
}
struct CpuProbe { std::optional<double> load; std::string top = "TOP CPU PROCESS WARMING UP", error; };
struct Runner { unsigned mode = 0; bool pd = false, fault = false; uint64_t received = 0, applied = 0; int result = 0; };

class HopeMonitor final : public rclcpp::Node {
 public:
  HopeMonitor() : Node("hope_monitor") {
    period_ = startup("period_s", 1.0);
    ref_ = declare_parameter<std::string>("reference_frame", "world");
    pelvis_ = declare_parameter<std::string>("pelvis_frame", "pelvis_link");
    max_offset_ = declare_parameter("ntp_max_offset_ms", 10.0);
    max_skew_ = declare_parameter("ntp_max_skew_ppm", 0.0);
    latency_topic_ = startup<std::string>("message_latency_topic", "/ros2/body_drive/pelvis_imu/data");
    const auto sample = startup<std::string>("message_latency_sample_topic", "");
    native_clock_ = startup("native_clock_topics", false);
    latency_stale_ = declare_parameter("message_latency_stale_after_s", 0.5);
    const double latency_hz = startup("message_latency_publish_hz", 20.0);
    pm_unit_ = declare_parameter<std::string>("agibot_pm_unit", "agibot_pm.service");
    runtime_unit_ = declare_parameter<std::string>("hdu_runtime_unit", "hope-lifecycle-supervisor.service");
    const double cpu_period = startup("cpu_publish_period_s", 1.0);
    tf_stale_ = declare_parameter("tf_stale_after_s", 0.5);
    const bool fallback = startup("enable_tf_fallback", true);
    runner_stale_ = declare_parameter("runner_state_stale_after_s", 2.5);
    base_stale_ = declare_parameter("base_pose_stale_after_s", 1.0);
    calibration_timeout_ = declare_parameter("calibration_timeout_s", 25.0);
    latch_path_ = startup<std::string>("estop_latch_path", "/var/lib/hope-monitor/estop-latched");
    for (auto v : {period_, latency_hz, latency_stale_, cpu_period, tf_stale_,
                    runner_stale_, base_stale_, calibration_timeout_, max_offset_})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Monitor periods/limits must be positive and finite");
    if (!std::isfinite(max_skew_) || max_skew_ < 0) throw std::invalid_argument("Invalid skew limit");
    parameter_validation_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      for (const auto &p : parameters) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
            (!std::isfinite(p.as_double()) || (p.get_name() == "ntp_max_skew_ppm" ? p.as_double() < 0 : p.as_double() <= 0))) {
          result.successful = false; result.reason = "Monitor limits/periods must be finite and positive (skew may be zero)";
        }
      }
      return result;
    });
    for (const auto &name : {"/hope/ntp/offset_ms", "/hope/ntp/skew_ppm", "/hope/ntp/root_dispersion_ms", "/hope/system/cpu_load_percent"})
      floats_[name] = create_publisher<Float>(name, 10);
    for (const auto &name : {"/hope/ntp/utc_qualified", "/hope/ntp/gate_pass", "/hope/vendor/agibot_pm_active",
         "/hope/system/hdu_active", "/hope/vendor/tf_ready", "/hope/safety/estop_ready", "/hope/safety/estop_full_ready",
         "/hope/safety/estop_latched", "/hope/control/runner_ready", "/hope/control/calibration_ready"})
      bools_[name] = create_publisher<Bool>(name, 10);
    for (const auto &name : {"/hope/ntp/text", "/hope/system/cpu_text", "/hope/system/cpu_top_process",
         "/hope/vendor/agibot_pm_text", "/hope/system/hdu_text", "/hope/safety/estop_text", "/hope/pelvis/text", "/hope/control/state_text"})
      strings_[name] = create_publisher<String>(name, 10);
    strings_["/hope/ntp/qualification"] = create_publisher<String>("/hope/ntp/qualification", 1);
    if (!native_clock_) {
      floats_["/hope/clock/message_latency_ms"] = create_publisher<Float>("/hope/clock/message_latency_ms", 10);
      bools_["/hope/clock/message_fresh"] = create_publisher<Bool>("/hope/clock/message_fresh", 10);
      strings_["/hope/clock/message_text"] = create_publisher<String>("/hope/clock/message_text", 10);
      if (sample.empty()) imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(latency_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });
      else latency_sub_ = create_subscription<Array>(sample, 1, [this](Array::ConstSharedPtr msg) { on_latency(msg->data); });
    }
    pose_pub_ = create_publisher<Pose>("/hope/pelvis/pose", 10);
    marker_pub_ = create_publisher<Marker>("/hope/pelvis/marker", 10);
    tf_pub_ = create_publisher<TF>("/hope/pelvis/tf", 10);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (fallback) {
      buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
    }
    pose_sub_ = create_subscription<Pose>(startup<std::string>("pelvis_pose_topic", "/a3/mocap/pelvis_pose"),
      rclcpp::SensorDataQoS(), [this](Pose::ConstSharedPtr msg) { on_pose(*msg); });
    mode_pub_ = create_publisher<Array>(startup<std::string>("mode_command_topic", "/hope/runner/mode_command"), 10);
    mode_sub_ = create_subscription<Array>(startup<std::string>("mode_state_topic", "/hope/runner/mode_state"),
      10, [this](Array::ConstSharedPtr msg) { on_runner(msg->data); });
    base_sub_ = create_subscription<Array>(startup<std::string>("base_pose_topic", "/a3/base_pose_flat"),
      10, [this](Array::ConstSharedPtr msg) { on_base(msg->data); });
    rpc_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    emergency_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mode_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    calibration_client_ = create_client<Trigger>(startup<std::string>("calibration_service", "/a3/calibration/recompute_ucb_robot_v2"),
      rclcpp::ServicesQoS(), rpc_group_);
    runner_client_ = create_client<Trigger>(startup<std::string>("runner_estop_service", "/hope/runner/emergency_stop"),
      rclcpp::ServicesQoS(), rpc_group_);
#ifdef HOPE_MONITOR_VENDOR_RPC
    vendor_client_ = create_client<Vendor>("/aimdk_2Eprotocol_2EHalEmergencyService/SetEmergencyCommand",
      rclcpp::SystemDefaultsQoS(), rpc_group_);
#else
    RCLCPP_WARN(get_logger(), "Built without ros2_plugin_proto: vendor emergency path unavailable (Runner path retained)");
#endif
    latched_ = std::filesystem::exists(latch_path_);
    sequence_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
    service("/hope/safety/trigger_estop", [this](auto response) { trigger_estop(*response); }, emergency_group_);
    service("/hope/safety/reset_software_estop", [this](auto response) { reset_estop(*response); }, emergency_group_);
    service("/hope/control/enter_prepare", [this](auto response) { enter_prepare(*response); }, mode_group_);
    service("/hope/control/enter_policy", [this](auto response) { enter_policy(*response); }, mode_group_);
    service("/hope/control/exit_policy", [this](auto response) { exit_policy(*response, false); }, mode_group_);
    service("/hope/control/enter_passive", [this](auto response) { exit_policy(*response, true); }, mode_group_);
    try { boot_ = hm::trim(hm::read_file("/proc/sys/kernel/random/boot_id")); } catch (...) {}
    monitor_id_ = identity(); signature_ = hope_network::ipv4_interface_signature();
    timer(period_, [this] { poll_ntp(); });
    timer(period_, [this] { poll_service(pm_worker_, pm_future_, get_parameter("agibot_pm_unit").as_string(), false); });
    timer(period_, [this] { poll_service(runtime_worker_, runtime_future_, get_parameter("hdu_runtime_unit").as_string(), true); });
    timer(cpu_period, [this] { poll_cpu(); });
    timer(.5, [this] { poll_estop(); });
    timer(.2, [this] { poll_control(); });
    timer(.2, [this] { poll_pose(); });
    timer(1, [this] {
      try {
        if (hope_network::ipv4_interface_signature() != signature_) restart("Network interfaces changed; reconnecting monitor DDS");
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Interface probe unavailable: %s", e.what());
      }
    });
    if (!native_clock_) timer(1.0 / latency_hz, [this] { publish_latency(); });
    RCLCPP_INFO(get_logger(), "Native HOPE monitor ready; no startup control request; vendor_rpc=%s", protocol() ? "yes" : "no");
  }
  ~HopeMonitor() override {
    // Workers never invoke ROS. Wait before their captured state is destroyed.
    ntp_worker_.stop(); pm_worker_.stop(); runtime_worker_.stop(); cpu_worker_.stop();
  }
  bool restart_requested() const { return restart_requested_; }
 private:
  template<class T> T startup(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    descriptor.description = "Startup-only: restart the stopped monitor to change this setting";
    return declare_parameter<T>(name, value, descriptor);
  }
  void restart(const std::string &reason) {
    restart_requested_ = true;
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    // Never throw out of a MultiThreadedExecutor worker (that would abort).
    get_node_base_interface()->get_context()->shutdown(reason);
  }
  template<class F> void timer(double seconds, F callback) {
    timers_.push_back(create_wall_timer(std::chrono::duration<double>(seconds), [this, callback] {
      try { callback(); }
      catch (const std::exception &e) { restart(std::string("Monitor callback failed: ") + e.what()); }
    }));
  }
  template<class F> void service(const std::string &name, F callback, rclcpp::CallbackGroup::SharedPtr group) {
    services_.push_back(create_service<Trigger>(name,
      [callback](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try { callback(response); }
        catch (const std::exception &e) { response->success = false; response->message = std::string("Monitor service failed: ") + e.what(); }
      },
      rclcpp::ServicesQoS(), group));
  }
  void boolean(const std::string &topic, bool value) { Bool msg; msg.data = value; bools_.at(topic)->publish(msg); }
  void number(const std::string &topic, double value) { Float msg; msg.data = value; floats_.at(topic)->publish(msg); }
  void text(const std::string &topic, const std::string &value) { String msg; msg.data = value; strings_.at(topic)->publish(msg); }
  bool protocol() const {
#ifdef HOPE_MONITOR_VENDOR_RPC
    return true;
#else
    return false;
#endif
  }
  bool vendor_ready() const {
#ifdef HOPE_MONITOR_VENDOR_RPC
    return vendor_client_->service_is_ready();
#else
    return false;
#endif
  }
  bool inhibited() const { return latched_.load() || asserting_.load(); }
  void poll_ntp();
  void poll_service(hm::ProbeWorker<hm::Service> &, std::future<hm::Service> &, const std::string &, bool);
  CpuProbe probe_cpu();
  void poll_cpu();
  void on_imu(const sensor_msgs::msg::Imu &);
  void on_latency(const std::vector<double> &);
  void publish_latency();
  void on_pose(const Pose &);
  void poll_pose();
  void markers(const Pose &, const std::array<double, 3> &);
  void on_runner(const std::vector<double> &);
  void on_base(const std::vector<double> &);
  bool fresh_runner() const { return runner_.has_value() && hm::mono() - runner_received_ <= get_parameter("runner_state_stale_after_s").as_double(); }
  bool matches_calibration() const;
  uint64_t publish_mode(unsigned);
  void cancel_prepare(const std::string &, bool clear = true);
  void enter_prepare(Trigger::Response &);
  void enter_policy(Trigger::Response &);
  void exit_policy(Trigger::Response &, bool);
  void poll_control();
  void finish_calibration();
  void poll_estop();
  void trigger_estop(Trigger::Response &);
  void reset_estop(Trigger::Response &);

  double period_, max_offset_, max_skew_, latency_stale_, tf_stale_, runner_stale_, base_stale_, calibration_timeout_;
  bool native_clock_;
  std::atomic_bool latched_{false};
  std::atomic_bool asserting_{false};
  std::atomic_bool restart_requested_{false};
  std::string ref_, pelvis_, latency_topic_, pm_unit_, runtime_unit_, monitor_id_, boot_;
  std::filesystem::path latch_path_;
  hope_network::InterfaceSignature signature_;
  std::map<std::string, rclcpp::Publisher<Bool>::SharedPtr> bools_;
  std::map<std::string, rclcpp::Publisher<Float>::SharedPtr> floats_;
  std::map<std::string, rclcpp::Publisher<String>::SharedPtr> strings_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<TF>::SharedPtr tf_pub_;
  rclcpp::Publisher<Array>::SharedPtr mode_pub_;
  rclcpp::Subscription<Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<Array>::SharedPtr mode_sub_, base_sub_, latency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::CallbackGroup::SharedPtr rpc_group_, emergency_group_, mode_group_;
  rclcpp::Client<Trigger>::SharedPtr runner_client_, calibration_client_;
#ifdef HOPE_MONITOR_VENDOR_RPC
  using Vendor = ros2_plugin_proto::srv::RosRpcWrapper;
  rclcpp::Client<Vendor>::SharedPtr vendor_client_;
#endif
  std::future<hm::Ntp> ntp_future_;
  hm::ProbeWorker<hm::Ntp> ntp_worker_;
  hm::ProbeWorker<hm::Service> pm_worker_, runtime_worker_;
  hm::ProbeWorker<CpuProbe> cpu_worker_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validation_;
  double ntp_started_ = 0;
  uint64_t ntp_sequence_ = 0;
  std::future<hm::Service> pm_future_, runtime_future_;
  std::string pm_probe_unit_, runtime_probe_unit_;
  std::future<CpuProbe> cpu_future_;
  std::optional<hm::Cpu> previous_cpu_;
  std::map<int, hm::Process> previous_processes_;
  double latency_ = NAN, latency_received_ = 0;
  std::optional<Pose> pose_;
  double pose_received_ = 0;
  mutable std::mutex state_mutex_;
  std::mutex emergency_mutex_;
  // Only command publication/stop dispatch use this lock; never probes, storage
  // or RPC waits. This orders local send calls only, NOT reception across DDS
  // topic/service writers. Runner-side command inhibition remains necessary.
  std::mutex dispatch_mutex_;
  std::string backend_status_;
  std::optional<Runner> runner_;
  double runner_received_ = 0, base_received_ = 0;
  bool base_valid_ = false;
  uint64_t base_calibration_ = 0, sequence_ = 1, prepare_sequence_ = 0, generation_ = 0;
  bool preparing_ = false;
  std::string calibration_sha_, control_detail_ = "RUNNER UNAVAILABLE | waiting for mode state";
  std::optional<rclcpp::Client<Trigger>::FutureAndRequestId> calibration_future_;
  uint64_t calibration_generation_ = 0;
  double calibration_started_ = 0;
};

void HopeMonitor::poll_ntp() {
  if (ntp_future_.valid()) {
    if (ntp_future_.wait_for(0s) != std::future_status::ready) return;
    hm::Ntp result;
    try { result = ntp_future_.get(); } catch (const std::exception &e) { result.error = e.what(); }
    const auto offset = get_parameter("ntp_max_offset_ms").as_double();
    const auto skew = get_parameter("ntp_max_skew_ppm").as_double();
    result.pass = result.error.empty() && result.qualified && std::isfinite(result.offset) && std::isfinite(result.skew) &&
      std::abs(result.offset) <= offset && (skew == 0 || result.skew <= skew);
    if (!result.error.empty()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Chrony probe unavailable: %s", result.error.c_str());
    std::ostringstream qualification;
    qualification << "{\"schema_version\":1,\"clock_boot_id\":" << hm::json_string(boot_)
      << ",\"monitor_id\":" << hm::json_string(monitor_id_) << ",\"sequence\":" << ++ntp_sequence_
      << ",\"observed_monotonic_s\":" << std::setprecision(17) << ntp_started_
      << ",\"gate_pass\":" << (result.pass ? "true" : "false") << '}';
    text("/hope/ntp/qualification", qualification.str());
    number("/hope/ntp/offset_ms", result.offset); number("/hope/ntp/skew_ppm", result.skew);
    number("/hope/ntp/root_dispersion_ms", result.dispersion);
    boolean("/hope/ntp/utc_qualified", result.qualified); boolean("/hope/ntp/gate_pass", result.pass);
    text("/hope/ntp/text", result.error.empty() ? "NTP world-clock offset = " + hm::fixed(result.offset, 3, true) +
      " ms | root dispersion = " + hm::fixed(result.dispersion) + " ms | skew = " + hm::fixed(result.skew) + " ppm" :
      "NTP UNAVAILABLE | " + result.error);
  }
  // Original observation time is recorded BEFORE work is queued. Publication
  // and DDS delays must consume freshness, never refresh old qualification.
  ntp_started_ = hm::mono();
  const auto offset = get_parameter("ntp_max_offset_ms").as_double();
  const auto skew = get_parameter("ntp_max_skew_ppm").as_double();
  ntp_future_ = ntp_worker_.submit([offset, skew] { return hm::probe_ntp(offset, skew); });
}
void HopeMonitor::poll_service(hm::ProbeWorker<hm::Service> &worker, std::future<hm::Service> &future, const std::string &unit, bool runtime) {
  auto &probed_unit = runtime ? runtime_probe_unit_ : pm_probe_unit_;
  if (future.valid()) {
    if (future.wait_for(0s) != std::future_status::ready) return;
    hm::Service r;
    try { r = future.get(); } catch (const std::exception &e) { r.error = e.what(); }
    // A runtime parameter edit must not relabel an old unit's queued result.
    if (unit == probed_unit) {
    boolean(runtime ? "/hope/system/hdu_active" : "/hope/vendor/agibot_pm_active", r.active);
    text(runtime ? "/hope/system/hdu_text" : "/hope/vendor/agibot_pm_text",
      (runtime ? "HDU runtime " : "") + unit + ": " + r.state + (r.error.empty() ? "" : " | " + r.error));
    }
  }
  probed_unit = unit;
  future = worker.submit([unit] { return hm::probe_service(unit); });
}
CpuProbe HopeMonitor::probe_cpu() {
  CpuProbe result;
  try {
    auto cpu = hm::parse_cpu(hm::read_file("/proc/stat")); auto procs = hm::processes();
    if (previous_cpu_) {
      result.load = hm::cpu_load(*previous_cpu_, cpu);
      uint64_t best = 0; std::optional<hm::Process> top;
      for (const auto &[pid, p] : procs) {
        const auto it = previous_processes_.find(pid);
        if (it == previous_processes_.end() || it->second.name != p.name || it->second.start != p.start || p.ticks < it->second.ticks) continue;
        const auto delta = p.ticks - it->second.ticks;
        if (!top || delta > best) { best = delta; top = p; }
      }
      result.top = "TOP CPU PROCESS UNAVAILABLE";
      if (top) {
        auto load = 100.0 * best / (cpu.total - previous_cpu_->total);
        auto cores = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
        result.top = top->name + " pid=" + std::to_string(top->pid) + " core=" + hm::fixed(load * cores, 1) +
          "% system=" + hm::fixed(load, 1) + "%";
      }
    }
    previous_cpu_ = cpu; previous_processes_ = std::move(procs);
  } catch (const std::exception &e) {
    result.error = e.what(); result.top = "TOP CPU PROCESS UNAVAILABLE";
    previous_cpu_.reset(); previous_processes_.clear();
  }
  return result;
}
void HopeMonitor::poll_cpu() {
  if (cpu_future_.valid()) {
    if (cpu_future_.wait_for(0s) != std::future_status::ready) return;
    CpuProbe r;
    try { r = cpu_future_.get(); }
    catch (const std::exception &e) { r.error = e.what(); r.top = "TOP CPU PROCESS UNAVAILABLE"; }
    text("/hope/system/cpu_top_process", r.top);
    if (!r.error.empty()) {
      text("/hope/system/cpu_text", "CPU LOAD UNAVAILABLE | " + r.error);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "CPU probe unavailable: %s", r.error.c_str());
    }
    else if (!r.load) text("/hope/system/cpu_text", "CPU LOAD WARMING UP");
    else { number("/hope/system/cpu_load_percent", *r.load);
      text("/hope/system/cpu_text", "A3 aggregate CPU load = " + hm::fixed(*r.load, 1) + "%"); }
  }
  cpu_future_ = cpu_worker_.submit([this] { return probe_cpu(); });
}
void HopeMonitor::on_imu(const sensor_msgs::msg::Imu &msg) {
  const int64_t ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  latency_received_ = hm::mono(); latency_ = ns > 0 ? (now().nanoseconds() - ns) / 1e6 : NAN;
}
void HopeMonitor::on_latency(const std::vector<double> &v) {
  if (v.size() != 3 || v[0] != 1 || !std::isfinite(v[1]) || !std::isfinite(v[2]) || v[2] <= 0 || v[2] > hm::mono()) {
    latency_ = NAN;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Malformed message-latency sample rejected"); return;
  }
  latency_ = v[1]; latency_received_ = v[2];
}
void HopeMonitor::publish_latency() {
  const bool fresh = latency_received_ > 0 && hm::mono() - latency_received_ <= get_parameter("message_latency_stale_after_s").as_double() && std::isfinite(latency_);
  boolean("/hope/clock/message_fresh", fresh);
  if (fresh) number("/hope/clock/message_latency_ms", latency_);
  text("/hope/clock/message_text", fresh ? "A3 ROS clock - " + latency_topic_ + " header stamp = " + hm::fixed(latency_, 3, true) + " ms" :
    "NO FRESH TIMESTAMP | " + latency_topic_);
}
void HopeMonitor::on_pose(const Pose &msg) {
  const auto &p = msg.pose.position; const auto &q = msg.pose.orientation;
  const auto norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  bool finite = true;
  for (auto v : {p.x,p.y,p.z,q.x,q.y,q.z,q.w}) finite = finite && std::isfinite(v);
  if (!finite || norm < .5 || norm > 1.5) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Malformed pelvis pose rejected"); return;
  }
  pose_ = msg; pose_received_ = hm::mono();
}
void HopeMonitor::poll_pose() {
  ref_ = get_parameter("reference_frame").as_string(); pelvis_ = get_parameter("pelvis_frame").as_string();
  tf_stale_ = get_parameter("tf_stale_after_s").as_double();
  auto pose = pose_; std::string source = "authoritative mocap pelvis pose";
  std::string error = "no authoritative mocap pelvis pose";
  auto valid_stamp = [&](const std_msgs::msg::Header &h) {
    const auto stamp = static_cast<int64_t>(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
    const auto age = (now().nanoseconds() - stamp) / 1e9;
    return stamp > 0 && age >= -.1 && age <= tf_stale_;
  };
  if (pose && (pose->header.frame_id != ref_ || hm::mono() - pose_received_ > tf_stale_ || !valid_stamp(pose->header))) {
    pose.reset(); error = "authoritative mocap pelvis pose is stale/future-dated or frame does not match";
  }
  if (!pose && buffer_) {
    try {
      const auto tf = buffer_->lookupTransform(ref_, pelvis_, tf2::TimePointZero);
      if (!valid_stamp(tf.header)) throw std::runtime_error("existing TF is stale/future-dated");
      Pose p; p.header = tf.header; p.pose.position.x = tf.transform.translation.x;
      p.pose.position.y = tf.transform.translation.y; p.pose.position.z = tf.transform.translation.z;
      p.pose.orientation = tf.transform.rotation; pose = p; source = "existing TF";
    } catch (const std::exception &e) { error += "; TF lookup failed: " + std::string(e.what()); }
  }
  boolean("/hope/vendor/tf_ready", pose.has_value());
  if (!pose) { text("/hope/pelvis/text", "PELVIS UNAVAILABLE | " + error + (buffer_ ? "" : "; TF fallback disabled")); return; }
  pose_pub_->publish(*pose);
  const auto &p = pose->pose.position; const auto &q = pose->pose.orientation;
  geometry_msgs::msg::TransformStamped tf;
  tf.header = pose->header; tf.child_frame_id = pelvis_;
  tf.transform.translation.x = p.x; tf.transform.translation.y = p.y; tf.transform.translation.z = p.z;
  tf.transform.rotation = q; broadcaster_->sendTransform(tf); TF batch; batch.transforms.push_back(tf); tf_pub_->publish(batch);
  constexpr double deg = 180.0 / 3.14159265358979323846;
  std::array<double, 3> rpy{deg*std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
    deg*std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.0,1.0)),
    deg*std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
  text("/hope/pelvis/text", ref_ + " -> " + pelvis_ + " | " + source +
    " | pos [m] x=" + hm::fixed(p.x,3,true) + " y=" + hm::fixed(p.y,3,true) + " z=" + hm::fixed(p.z,3,true) +
    " | quat x=" + hm::fixed(q.x,4,true) + " y=" + hm::fixed(q.y,4,true) + " z=" + hm::fixed(q.z,4,true) + " w=" + hm::fixed(q.w,4,true) +
    " | rpy [deg] r=" + hm::fixed(rpy[0],1,true) + " p=" + hm::fixed(rpy[1],1,true) + " y=" + hm::fixed(rpy[2],1,true));
  markers(*pose, rpy);
}
void HopeMonitor::markers(const Pose &pose, const std::array<double, 3> &rpy) {
  Marker point; point.header = pose.header; point.ns = "hope_pelvis"; point.id = 0;
  point.type = Marker::SPHERE; point.action = Marker::ADD; point.pose = pose.pose;
  point.scale.x = point.scale.y = point.scale.z = .08;
  point.color.r = .1; point.color.g = .85; point.color.b = 1; point.color.a = 1;
  point.lifetime.nanosec = 500000000; point.frame_locked = true; marker_pub_->publish(point);
  Marker label; label.header = pose.header; label.ns = "hope_pelvis"; label.id = 1;
  label.type = Marker::TEXT_VIEW_FACING; label.action = Marker::ADD;
  label.pose.position = pose.pose.position; label.pose.position.z += .18; label.pose.orientation.w = 1;
  label.scale.z = .1; label.color.r = 1; label.color.g = .78; label.color.b = .4; label.color.a = 1;
  label.lifetime = point.lifetime; label.frame_locked = true;
  label.text = pelvis_ + "\nx " + hm::fixed(pose.pose.position.x,3,true) + "  y " + hm::fixed(pose.pose.position.y,3,true) +
    "  z " + hm::fixed(pose.pose.position.z,3,true) + "\nrpy " + hm::fixed(rpy[0],1,true) + " deg  " +
    hm::fixed(rpy[1],1,true) + " deg  " + hm::fixed(rpy[2],1,true) + " deg";
  marker_pub_->publish(label);
}

// Legacy compatibility commands remain serialized and never publish on startup.
void HopeMonitor::on_runner(const std::vector<double> &v) {
  try {
    if (v.size() != 8 || v[0] != 1) throw std::invalid_argument("bad mode-state schema/length");
    Runner r; r.mode = hm::exact(v[1],0,8); r.pd = hm::exact(v[2],0,1);
    r.received = hm::exact(v[3]); r.applied = hm::exact(v[4]);
    r.result = static_cast<int>(hm::exact(std::abs(v[5]),0,3)) * (v[5] < 0 ? -1 : 1);
    hm::exact(v[6],0,1); r.fault = hm::exact(v[7],0,1);
    std::lock_guard<std::mutex> lock(state_mutex_); runner_ = r; runner_received_ = hm::mono();
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Malformed Runner state rejected: %s", e.what());
  }
}
void HopeMonitor::on_base(const std::vector<double> &v) {
  bool valid = false; uint64_t id = 0;
  try { if (v.size() >= 16 && v[0] == 2 && v[1] == 1) { id = hm::exact(v[14],1); valid = true; } } catch (...) {}
  if (!valid) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Invalid base-pose calibration sample");
  std::lock_guard<std::mutex> lock(state_mutex_); base_valid_ = valid; base_calibration_ = id; base_received_ = hm::mono();
}
bool HopeMonitor::matches_calibration() const {
  return !calibration_sha_.empty() && base_valid_ && hm::mono() - base_received_ <= get_parameter("base_pose_stale_after_s").as_double() &&
    base_calibration_ == std::stoull(calibration_sha_.substr(0,13),nullptr,16);
}
uint64_t HopeMonitor::publish_mode(unsigned command) {
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
  if (inhibited()) return 0;
  Array msg; msg.data = {1, static_cast<double>(++sequence_), static_cast<double>(command)};
  mode_pub_->publish(msg); return sequence_;
}
void HopeMonitor::cancel_prepare(const std::string &detail, bool clear) {
  ++generation_; preparing_ = false; prepare_sequence_ = 0;
  if (clear) calibration_sha_.clear();
  control_detail_ = detail;
}
void HopeMonitor::enter_prepare(Trigger::Response &out) {
  const bool calibration_ready = calibration_client_->service_is_ready();
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (inhibited()) { out.message = "E-stop is latched; click Reset Software E-stop"; return; }
  if (!fresh_runner()) { out.message = "runner mode state is unavailable; PD_STAND was NOT requested"; return; }
  if (runner_->fault) { out.message = "runner command fault is latched; PD_STAND was NOT requested"; return; }
  if (runner_->received > runner_->applied && runner_->result == 0) { out.message = "a runner command is already in progress"; return; }
  if (!calibration_ready) { out.message = "laptop calibration service is unavailable; PD_STAND was NOT requested"; return; }
  if (calibration_future_) { out.message = "marker calibration is already running"; return; }
  ++generation_; calibration_sha_.clear(); preparing_ = true;
  prepare_sequence_ = publish_mode(1);
  if (!prepare_sequence_) { cancel_prepare("E-STOP LATCHED | PREPARE not requested"); out.message = control_detail_; return; }
  control_detail_ = "PREPARE REQUESTED seq=" + std::to_string(prepare_sequence_) + " | waiting for PD_STAND readiness";
  out.success = true; out.message = "PD_STAND requested (seq=" + std::to_string(prepare_sequence_) +
    "); this init's marker calibration will run on the laptop after settle and atomically replace the laptop JSON";
}
void HopeMonitor::enter_policy(Trigger::Response &out) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (inhibited()) { out.message = "E-stop is latched; MOTION was NOT requested"; return; }
  if (!fresh_runner()) { out.message = "runner mode state is unavailable; MOTION was NOT requested"; return; }
  if (runner_->fault) { out.message = "runner command fault is latched; MOTION was NOT requested"; return; }
  if (runner_->mode != 1 || !runner_->pd) { out.message = "robot is not in settled PD_STAND; MOTION was NOT requested"; return; }
  if (calibration_sha_.empty()) { out.message = "this run has no approved laptop calibration; run PREPARE again before MOTION"; return; }
  if (!matches_calibration()) { out.message = "approved calibration is not yet present in valid /a3/base_pose_flat; MOTION was NOT requested"; return; }
  const auto seq = publish_mode(2);
  if (!seq) { out.message = "E-stop is latched; MOTION was NOT requested"; return; }
  control_detail_ = "POLICY REQUESTED seq=" + std::to_string(seq) + " | waiting for runner ack";
  out.success = true; out.message = "MOTION/policy requested (seq=" + std::to_string(seq) + ")";
}
void HopeMonitor::exit_policy(Trigger::Response &out, bool passive) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (inhibited()) { out.message = "E-stop is latched; runner transition was NOT requested"; return; }
  if (!fresh_runner()) { out.message = "runner mode state is unavailable; transition was NOT requested"; return; }
  cancel_prepare(passive ? "PASSIVE REQUESTED | waiting for runner ack" : "EXIT POLICY REQUESTED | waiting for PD_STAND ack", passive);
  const auto seq = publish_mode(passive ? 0 : 1);
  if (!seq) { out.message = "E-stop is latched; runner transition was NOT requested"; return; }
  out.success = true; out.message = std::string(passive ? "PASSIVE" : "PD_STAND") + " requested (seq=" + std::to_string(seq) + "); no calibration started";
}
void HopeMonitor::finish_calibration() {
  if (!calibration_future_) return;
  bool failed = false; std::string detail;
  if (calibration_future_->wait_for(0s) != std::future_status::ready) {
    if (hm::mono() - calibration_started_ <= get_parameter("calibration_timeout_s").as_double()) return;
    calibration_client_->remove_pending_request(calibration_future_->request_id);
    failed = true; detail = "laptop service timed out";
  } else {
    try {
      const auto result = calibration_future_->get();
      detail = result ? hm::trim(result->message) : "laptop calibration returned no response";
      if (!result || !result->success || detail.size() != 64 || detail.find_first_not_of("0123456789abcdef") != std::string::npos) failed = true;
    } catch (const std::exception &e) { failed = true; detail = e.what(); }
  }
  calibration_future_.reset();
  if (calibration_generation_ != generation_) return;
  if (failed) {
    calibration_sha_.clear(); control_detail_ = "CALIBRATION FAILED | " + detail;
    RCLCPP_WARN(get_logger(), "%s", control_detail_.c_str());
  } else {
    calibration_sha_ = detail; control_detail_ = "LAPTOP CALIBRATION SAVED | waiting for matching /a3/base_pose_flat";
    RCLCPP_INFO(get_logger(), "Laptop calibration saved: SHA=%s", detail.c_str());
  }
}
void HopeMonitor::poll_control() {
  const bool calibration_ready = calibration_client_->service_is_ready();
  std::lock_guard<std::mutex> lock(state_mutex_); finish_calibration();
  const bool fresh = fresh_runner();
  boolean("/hope/control/runner_ready", fresh && runner_->mode != 7 && runner_->mode != 8 && !runner_->fault);
  const bool calibrated = matches_calibration(); boolean("/hope/control/calibration_ready", calibrated);
  if (!fresh) { text("/hope/control/state_text", "RUNNER UNAVAILABLE | start the approved MDU policy runner"); return; }
  if (preparing_ && runner_->mode == 1 && runner_->pd && prepare_sequence_ > 0 && runner_->applied >= prepare_sequence_) {
    preparing_ = false; prepare_sequence_ = 0;
    if (!inhibited() && calibration_ready) {
      try {
        calibration_future_.emplace(calibration_client_->async_send_request(std::make_shared<Trigger::Request>()));
        calibration_generation_ = generation_; calibration_started_ = hm::mono();
        control_detail_ = "PD_STAND READY | laptop calibration running";
        RCLCPP_INFO(get_logger(), "Laptop calibration started");
      } catch (const std::exception &e) {
        control_detail_ = "CALIBRATION START FAILED | " + std::string(e.what()); RCLCPP_WARN(get_logger(), "%s", control_detail_.c_str());
      }
    } else {
      control_detail_ = "CALIBRATION START FAILED | laptop service unavailable or E-stop latched";
      RCLCPP_WARN(get_logger(), "%s", control_detail_.c_str());
    }
  } else if (preparing_ && runner_->received >= prepare_sequence_ && runner_->applied < prepare_sequence_ && runner_->result < 0)
    cancel_prepare("PREPARE REJECTED | runner result=" + std::to_string(runner_->result));
  static const std::array<std::string, 9> names{"PASSIVE","PD_STAND","SHADOW","MOTION","REFERENCE_PLAYBACK","SERVE","IDLE","STOPPED","STARTING"};
  std::string detail;
  if (runner_->mode == 7 && !runner_->fault) detail = "STOPPED | ENTER PREPARE starts the approved runner";
  else if (runner_->mode == 8) detail = "STARTING | waiting for all six stock runner inputs";
  else if (calibrated) detail = names[runner_->mode] + " | PD_READY=" + std::to_string(runner_->pd) + " | CALIBRATION READY | policy gate open";
  else detail = names[runner_->mode] + " | " + (calibration_future_ ? "MARKER CALIBRATION RUNNING" : control_detail_);
  if (runner_->result < 0) detail += " | last runner request rejected (" + std::to_string(runner_->result) + ")";
  if (runner_->fault) detail += " | COMMAND FAULT LATCHED";
  text("/hope/control/state_text", detail);
}
void HopeMonitor::poll_estop() {
  const auto vendor = vendor_ready(), runner = runner_client_->service_is_ready();
  const bool inhibit = inhibited();
  const auto s = hm::estop_status(vendor, runner, inhibit, protocol());
  if (s.text != backend_status_) {
    RCLCPP_INFO(get_logger(), "Emergency backend status: %s", s.text.c_str()); backend_status_ = s.text;
  }
  boolean("/hope/safety/estop_ready", s.ready); boolean("/hope/safety/estop_full_ready", s.full);
  boolean("/hope/safety/estop_latched", inhibit); text("/hope/safety/estop_text", s.text);
}
void HopeMonitor::reset_estop(Trigger::Response &out) {
  if (asserting_) { out.message = "E-stop is still being asserted; retry after completion"; return; }
  std::unique_lock<std::mutex> emergency(emergency_mutex_, std::try_to_lock);
  if (!emergency.owns_lock()) { out.message = "E-stop is still being asserted; retry after completion"; return; }
  if (asserting_) { out.message = "E-stop is still being asserted; retry after completion"; return; }
  try { hm::reset_latch(latch_path_); }
  catch (const std::exception &e) { out.message = e.what(); return; }
  {
    std::lock_guard<std::mutex> state(state_mutex_);
    cancel_prepare("SOFTWARE E-STOP RESET | no motion requested", false);
    std::lock_guard<std::mutex> dispatch(dispatch_mutex_); latched_ = false;
  }
  boolean("/hope/safety/estop_latched", inhibited());
  RCLCPP_INFO(get_logger(), "Software E-stop inhibit reset; no hardware reset or motion requested");
  out.success = true; out.message = "Software E-stop reset. No motion started; vendor/hardware stop unchanged. Release controller buttons, then explicitly start/select the next mode.";
}
void HopeMonitor::trigger_estop(Trigger::Response &out) {
  if (asserting_.exchange(true)) { out.message = "E-stop assertion already in progress; use the physical E-stop"; return; }
  struct ClearAssertion {
    std::atomic_bool &flag;
    ~ClearAssertion() { flag.store(false); }
  } clear_assertion{asserting_};
  // A trigger may be refused only for another trigger, never for a reset.
  // Wait for an active reset to finish. `asserting_` already inhibits normal
  // mode publication, including the interval when that reset clears latched_.
  std::unique_lock<std::mutex> emergency(emergency_mutex_);
  // The assertion flag inhibited mode publication before any reset-lock wait.
  // Now commit the new latch intent after that preceding reset has completed.
  const bool reassert = latched_.exchange(true);
  const auto deadline = hm::Steady::now() + 2700ms;
  bool runner_ok = false, vendor_ok = false;
  std::string rd = "managed runner stop service unavailable", vd = "ros2_plugin_proto unavailable";
  std::optional<rclcpp::Client<Trigger>::FutureAndRequestId> runner_future;
#ifdef HOPE_MONITOR_VENDOR_RPC
  std::optional<rclcpp::Client<Vendor>::FutureAndRequestId> vendor_future;
  hm::Steady::time_point vendor_deadline;
  auto send_vendor = [&] {
    try {
      auto request = std::make_shared<Vendor::Request>(); request->serialization_type = "pb";
      request->context = {"aimdk.protocol.EmergencyCommandReq"};
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      request->data = hm::estop_request(static_cast<uint64_t>(ns), "hope-foxglove-" + identity());
      vendor_future.emplace(vendor_client_->async_send_request(request)); vendor_deadline = hm::Steady::now() + 2s;
    } catch (const std::exception &e) { vd = "vendor dispatch failed: " + std::string(e.what()); }
  };
  vd = "vendor emergency service unavailable";
  const bool vendor_available = vendor_ready();
#endif
  const bool runner_available = runner_client_->service_is_ready();
  {
    std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
    if (runner_available) {
      try { runner_future.emplace(runner_client_->async_send_request(std::make_shared<Trigger::Request>())); rd = "managed runner stop pending"; }
      catch (const std::exception &e) { rd = e.what(); }
    }
#ifdef HOPE_MONITOR_VENDOR_RPC
    if (vendor_available) send_vendor();
#endif
  }
#ifdef HOPE_MONITOR_VENDOR_RPC
  // Discovery grace never delays dispatch to an already-available Runner.
  if (!vendor_available && vendor_client_->wait_for_service(500ms)) {
    std::lock_guard<std::mutex> dispatch(dispatch_mutex_); send_vendor();
  }
#endif
  // The available stop send calls have now been attempted. Only now touch shared
  // legacy state, status publications, logs or slow durable storage. This is
  // local dispatch ordering, not a cross-writer DDS reception guarantee.
  RCLCPP_WARN(get_logger(), "%s | stop requests dispatched where available; physical stop unconfirmed",
    reassert ? "E-STOP REASSERTING" : "E-STOP ASSERTING");
  { std::lock_guard<std::mutex> state(state_mutex_); cancel_prepare("E-STOP LATCHED | click Reset Software E-stop"); }
  boolean("/hope/safety/estop_latched", true);
  std::string persistence_error;
  try { hm::persist_latch(latch_path_, static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count())); }
  catch (const std::exception &e) { persistence_error = e.what(); RCLCPP_ERROR(get_logger(), "%s", e.what()); }
#ifdef HOPE_MONITOR_VENDOR_RPC
  if (vendor_future) {
    try {
      if (vendor_future->wait_until(vendor_deadline) != std::future_status::ready) {
        vendor_client_->remove_pending_request(vendor_future->request_id); vd = "vendor call timed out";
      }
      else {
        const auto response = vendor_future->get();
        if (!response) vd = "vendor returned no response";
        else if (response->code != 0) vd = "vendor wrapper rejected request: code=" + std::to_string(response->code);
        else if (response->serialization_type != "pb") vd = "vendor returned unexpected serialization type";
        else { const auto result = hm::estop_response(response->data); vendor_ok = result.first == 0;
          vd = vendor_ok ? "vendor request accepted" : "vendor application rejected request: code=" + std::to_string(result.first) + ", msg=" +
            (result.second.empty() ? "no detail" : result.second); }
      }
    } catch (const std::exception &e) { vd = "vendor call/decode failed: " + std::string(e.what()); }
  }
#endif
  if (runner_future) {
    if (runner_future->wait_until(deadline) != std::future_status::ready) {
      runner_client_->remove_pending_request(runner_future->request_id); rd = "managed runner stop timed out";
    } else {
      try { const auto response = runner_future->get(); runner_ok = response && response->success;
        rd = runner_ok ? "managed runner stopped" : (response ? (response->message.empty() ? "managed runner rejected request: no detail" : response->message) : "managed runner returned no response"); }
      catch (const std::exception &e) { rd = e.what(); }
    }
  }
  const auto result = hm::combine_estop(vendor_ok, vd, runner_ok, rd); out.success = result.first;
  out.message = (reassert ? "E-STOP REASSERTED | " : "") + result.second;
  if (!persistence_error.empty()) { out.success = false; out.message += "; LOCAL LATCH PERSISTENCE FAILED: " + persistence_error + "; keep the physical E-stop asserted"; }
  RCLCPP_WARN(get_logger(), "%s", out.message.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<HopeMonitor>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node); executor.spin(); executor.remove_node(node);
    result = node->restart_requested() ? 1 : 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "hope-monitor: %s\n", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
