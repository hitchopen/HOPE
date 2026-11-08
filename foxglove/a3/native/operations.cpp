// One ROS participant for operations; Python retains audited hardware actions.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <spawn.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <deque>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "operations_observer.hpp"
#include "operations_runner.hpp"
#include <csignal>

extern char **environ;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Array = std_msgs::msg::Float64MultiArray;
using String = std_msgs::msg::String;
using Trigger = std_srvs::srv::Trigger;
using Parameters = rcl_interfaces::srv::SetParameters;
using namespace std::chrono_literals;

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;
void stop_signal(int) { shutdown_requested = 1; }
constexpr std::size_t kMaxFrame = 32 * 1024 * 1024;
double steady_now() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
Json number(double value) {
  if (std::isfinite(value)) return value;
  return std::isnan(value) ? "NaN" : value > 0 ? "Infinity" : "-Infinity";
}
double real(const Json &value) {
  return value.is_string() ? std::stod(value.get<std::string>()) : value.get<double>();
}
Json numbers(const std::vector<double> &values) {
  Json out = Json::array();
  for (double value : values) out.push_back(number(value));
  return out;
}
template<class T> T decode(const rclcpp::SerializedMessage &serialized) {
  T value;
  rclcpp::Serialization<T> codec;
  codec.deserialize_message(&serialized, &value);
  return value;
}
rclcpp::QoS qos(const Json &spec) {
  rclcpp::QoS out(rclcpp::KeepLast(spec.at("depth").get<std::size_t>()));
  if (spec.at("reliable").get<bool>()) out.reliable(); else out.best_effort();
  if (spec.at("transient").get<bool>()) out.transient_local(); else out.durability_volatile();
  return out;
}
bool exact_io(int fd, void *buffer, std::size_t size, bool writing) {
  auto *p = static_cast<char *>(buffer);
  while (size) {
    const auto count = writing ? ::send(fd, p, size, MSG_NOSIGNAL) : ::recv(fd, p, size, 0);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    p += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}
bool send_until(int fd,const void *buffer,std::size_t size,Clock::time_point deadline,std::size_t &sent) {
  const auto *p=static_cast<const char*>(buffer);
  while(size) {
    auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
    if(remaining<=0)return false;
    pollfd event{fd,POLLOUT,0};
    int rc=::poll(&event,1,static_cast<int>(remaining));
    if(rc<0&&errno==EINTR)continue;
    if(rc<=0)return false;
    auto n=::send(fd,p,size,MSG_NOSIGNAL|MSG_DONTWAIT);
    if(n<0&&(errno==EINTR||errno==EAGAIN||errno==EWOULDBLOCK))continue;
    if(n<=0)return false;
    p+=n;size-=n;sent+=n;
  }
  return true;
}
Json parameter_value(const rclcpp::Parameter &p) {
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return p.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING: return p.as_string();
    default: throw std::runtime_error("unsupported operations parameter type");
  }
}
Json parameters_request(const Parameters::Request &request) {
  Json values = Json::array();
  for (const auto &p : request.parameters) {
    const auto &v = p.value;
    values.push_back({{"name", p.name}, {"value", {
      {"type", v.type}, {"bool_value", v.bool_value}, {"integer_value", v.integer_value},
      {"double_value", number(v.double_value)}, {"string_value", v.string_value},
      {"byte_array_value", v.byte_array_value}, {"bool_array_value", v.bool_array_value},
      {"integer_array_value", v.integer_array_value}, {"double_array_value", numbers(v.double_array_value)},
      {"string_array_value", v.string_array_value}}}});
  }
  return {{"parameters", values}};
}
}  // namespace

class Operations final : public rclcpp::Node {
 public:
  explicit Operations(const rclcpp::NodeOptions &options) : Node("hope_operations", options) {
    worker_path_ = declare_parameter("worker_path", "/usr/local/libexec/hope_operations_worker.py");
    python_ = declare_parameter("worker_python", "/usr/bin/python3");
    // Module deadlines remain authoritative. Long asset/cleanup operations
    // must not acquire an arbitrary shorter transport deadline after porting.
    // A positive override is available only for bounded diagnostic tests.
    rpc_timeout_ = declare_parameter("worker_rpc_timeout_s", 0.0);
    if (!std::isfinite(rpc_timeout_) || rpc_timeout_ < 0.) throw std::invalid_argument("invalid worker RPC timeout");
    status_ = create_publisher<String>("/hope/operations/status", 10);
    metrics_ = create_publisher<String>("/hope/operations/metrics", 10);
    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // As in rclpy (spin starts after module construction), requests may queue
    // in DDS during initialization but must not execute against partial state.
    request_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    urgent_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    observer_ = std::make_unique<hope_operations::NativeObserver>(*this);
    observer_->runner_sink([this](const Json &state){runner_.observe(state,steady_now());});
    runner_publisher_=create_publisher<Array>("/hope/runner/control_request_hdu_flat",10);
    runner_timer_=create_wall_timer(5ms,[this]{runner_.poll(steady_now());},urgent_group_);
    observer_->fault_sink([this](const std::string &source,const std::string &error) {
      // Recovery/failure edges only. Unchanged successful 5 Hz snapshots
      // never wake the worker for diagnostic book-keeping.
      {
        std::lock_guard<std::mutex> lock(fault_mutex_);
        auto it=native_faults_.find(source);
        if(it==native_faults_.end()&&error.empty())return;
        if(error.empty())native_faults_.erase(source);else native_faults_[source]=error;
      }
      queue_control({{"op","native_fault"},{"source",source},{"error",error}});
    });
    expiry_timer_ = create_wall_timer(50ms, [this] {
      expire(); observer_->expire_waiters();
      if (shutdown_requested && !shutdown_enqueued_) {
        ready_=false;draining_=true;
        shutdown_enqueued_=queue_control({{"op","shutdown"}});
      }
      // Enable once: normal startup drains DDS's existing bounded service
      // queues; failed/interrupted startup returns explicit failures instead.
      if (!requests_enabled_ && (ready_ || dead_ || draining_)) {
        enable_requests_();
        requests_enabled_ = true;
      }
      // Failed initialization/cleanup uses the existing failed-worker status
      // grace below, allowing queued DDS requests to receive a failure first.
      if (drained_ && drain_success_ && runner_.seal_if_idle()) rclcpp::shutdown();
    });
    status_timer_ = create_wall_timer(1s, [this] {
      String status;
      { std::lock_guard<std::mutex> lock(status_mutex_);
        status.data = draining_ ? "DRAINING: finishing accepted operations; new requests are rejected"
                            : ready_ ? worker_status_
                            : dead_ ? "ERROR: operations worker disconnected; actions not replayed"
                                    : "STARTING: operations worker initializing"; }
      status_->publish(status);
      Json metrics{{"schema_version",1},{"worker_ready",bool(ready_)},{"worker_dead",bool(dead_)},
                   {"runner",runner_.metrics()},{"sample_channel_alive",bool(samples_alive_)}};
      {std::lock_guard<std::mutex> lock(sample_mutex_);
        metrics["native_samples_received"]=samples_received_;
        metrics["native_samples_coalesced"]=samples_coalesced_;
        metrics["native_pending_samples"]=samples_.size();}
      {std::lock_guard<std::mutex> lock(status_mutex_);metrics["worker_transport"]=worker_metrics_;}
      String encoded;encoded.data=metrics.dump();metrics_->publish(encoded);
      // Reconstruct the whole participant through systemd after a failed
      // worker. First deliver explicit failures/status; never replay actions.
      if (dead_ && ++failed_status_ticks_ >= 2 && runner_.seal_if_idle()) rclcpp::shutdown();
    });
  }

  void start(rclcpp::Executor &executor) {
    enable_requests_ = [this, &executor] {
      executor.add_callback_group(request_group_, get_node_base_interface());
      urgent_executor_.add_callback_group(urgent_group_,get_node_base_interface());
    };
    int sockets[2];
    if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets))
      throw std::runtime_error("cannot create private worker socket");
    fd_ = sockets[0];
    int sample_sockets[2];
    if(::socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,sample_sockets))
      throw std::runtime_error("cannot create diagnostics socket");
    sample_fd_=sample_sockets[0];
    // Relocate child sources before duplicating onto fixed inherited fds.
    const int child_control=::fcntl(sockets[1],F_DUPFD_CLOEXEC,10);
    const int child_samples=::fcntl(sample_sockets[1],F_DUPFD_CLOEXEC,10);
    ::close(sockets[1]);::close(sample_sockets[1]);
    if(child_control<0||child_samples<0)throw std::runtime_error("cannot inherit operations channels");
    posix_spawn_file_actions_t actions;
    ::posix_spawn_file_actions_init(&actions);
    ::posix_spawn_file_actions_addclose(&actions, sockets[0]);
    ::posix_spawn_file_actions_addclose(&actions,sample_fd_);
    ::posix_spawn_file_actions_adddup2(&actions,child_control,3);
    ::posix_spawn_file_actions_adddup2(&actions,child_samples,4);
    ::posix_spawn_file_actions_addclose(&actions,child_control);
    ::posix_spawn_file_actions_addclose(&actions,child_samples);
    std::vector<char *> argv{python_.data(), worker_path_.data(), const_cast<char *>("--fd"), const_cast<char *>("3"),
      const_cast<char *>("--samples-fd"),const_cast<char *>("4"),nullptr};
    const int result = ::posix_spawn(&worker_pid_, python_.c_str(), &actions, nullptr, argv.data(), environ);
    ::posix_spawn_file_actions_destroy(&actions);
    ::close(child_control);::close(child_samples);
    if (result) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("cannot spawn operations worker: " + std::string(std::strerror(result)));
    }
    reader_ = std::thread([this] { read_loop(); });
    samples_alive_=true;
    sample_sender_=std::thread([this]{sample_loop();});
    completion_sender_=std::thread([this]{completion_loop();});
    urgent_thread_=std::thread([this]{urgent_executor_.spin();});
  }

  bool worker_failed() const { return drained_ ? !drain_success_ : bool(dead_); }

  ~Operations() override {
    ready_ = false;
    stopping_ = true;
    urgent_executor_.cancel();
    if(urgent_thread_.joinable())urgent_thread_.join();
    if (worker_pid_ > 0) ::kill(worker_pid_, SIGTERM);
    if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
    if(sample_fd_>=0)::shutdown(sample_fd_,SHUT_RDWR);
    sample_cv_.notify_all();completion_cv_.notify_all();
    if(sample_sender_.joinable())sample_sender_.join();
    if(completion_sender_.joinable())completion_sender_.join();
    if (reader_.joinable()) reader_.join();
    if (fd_ >= 0) ::close(fd_);
    if(sample_fd_>=0)::close(sample_fd_);
    if (worker_pid_ > 0) {
      // The systemd unit owns the outer shutdown deadline. Never force-kill
      // an accepted hardware operation from an internal timer.
      while (::waitpid(worker_pid_, nullptr, 0) < 0 && errno == EINTR) {}
    }
  }

 private:
  struct Reader {
    std::uint64_t id{};
    std::string topic, type;
    rclcpp::GenericSubscription::SharedPtr subscription;
  };
  struct Sample {
    std::shared_ptr<Reader> reader;
    std::shared_ptr<rclcpp::SerializedMessage> data;
    rclcpp::MessageInfo info;
    double received;
    std::int64_t wall_ns;
  };
  struct Pending {
    Clock::time_point deadline;
    std::function<void(const Json &)> complete;
  };
  struct ClientPending {
    rclcpp::Client<Trigger>::SharedPtr client;
    std::int64_t request_id;
    Clock::time_point deadline;
  };
  bool send(const Json &message) {
    const auto deadline=Clock::now()+5s;
    // ROS strings are byte containers. A malformed diagnostic must not throw
    // out of an executor callback; use explicit replacement at the IPC boundary.
    const auto text = message.dump(-1, ' ', false, Json::error_handler_t::replace);
    if (text.empty() || text.size() > kMaxFrame) throw std::runtime_error("operations IPC frame exceeds limit");
    std::uint32_t size = htonl(static_cast<std::uint32_t>(text.size()));
    std::unique_lock<std::timed_mutex> guard(write_mutex_,std::defer_lock);
    if(!guard.try_lock_until(deadline))throw std::runtime_error("operations send deadline before dispatch; not replayed");
    if (dead_ || fd_ < 0) return false;
    std::size_t sent=0;
    if (!send_until(fd_, &size, sizeof(size),deadline,sent) ||
        !send_until(fd_,text.data(),text.size(),deadline,sent)) {
      // A partial frame cannot be retried or safely reused. Zero bytes sent
      // is an individual request failure, not evidence that the worker died.
      if(sent){::shutdown(fd_,SHUT_RDWR);disconnected();}
      else throw std::runtime_error("operations send deadline before dispatch; not replayed");
      return false;
    }
    return true;
  }
  void reply(std::uint64_t id, const Json &value) {
    queue_control({{"op", "reply"}, {"id", id}, {"value", value}});
  }
  bool queue_control(Json value) {
    std::lock_guard<std::mutex> lock(completion_mutex_);
    if(completions_.size()>=128) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Operations reply queue full; caller must time out, never replay");
      return false;
    }
    completions_.push_back(std::move(value));completion_cv_.notify_one();return true;
  }
  void disconnected() {
    dead_ = true;
    ready_ = false;
    std::map<std::uint64_t, Pending> pending;
    { std::lock_guard<std::mutex> lock(pending_mutex_); pending.swap(pending_); }
    for (auto &entry : pending) {
      finish(entry.second.complete, {{"error", "operations worker disconnected; request not replayed"}});
    }
    std::lock_guard<std::mutex> lock(client_mutex_);
    for (auto &entry : client_pending_) entry.second.client->remove_pending_request(entry.second.request_id);
    client_pending_.clear();
  }
  void finish(const std::function<void(const Json &)> &complete, const Json &response) {
    // A client disappearing or ROS shutting down must not terminate the owner
    // while other accepted hardware actions are still draining.
    try { complete(response); }
    catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Operations response delivery failed: %s", error.what());
    }
  }
  Clock::time_point rpc_deadline() const {
    return rpc_timeout_ > 0 ? Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(rpc_timeout_))
                            : Clock::time_point::max();
  }
  void call(std::uint64_t entity, const Json &request, std::function<void(const Json &)> complete, bool urgent=false) {
    const auto received=steady_now();
    const auto runner_epoch=runner_.epoch();
    if (!ready_ || draining_ || stopping_ || dead_) { finish(complete, {{"error", "operations worker unavailable"}}); return; }
    const auto id = ++call_id_;
    const auto deadline = rpc_deadline();
    bool full;
    { std::lock_guard<std::mutex> lock(pending_mutex_);
      full = pending_.size() >= (urgent ? 64U : 63U);
      if (!full) pending_[id] = Pending{deadline, std::move(complete)};
    }
    if (full) { finish(complete, {{"error", "operations request backlog full"}}); return; }
    try {
      if (!send({{"op", "call"}, {"id", id}, {"entity", entity}, {"request", request},
                 {"runner_epoch",runner_epoch},{"received",received}})) disconnected();
    } catch (const std::exception &error) {
      // Encoding/size failure belongs to this request, not to the worker link.
      std::function<void(const Json &)> rejected;
      { std::lock_guard<std::mutex> lock(pending_mutex_);
        auto it = pending_.find(id);
        if (it != pending_.end()) { rejected = std::move(it->second.complete); pending_.erase(it); }
      }
      if (rejected) finish(rejected, {{"error", error.what()}});
    }
  }
  void expire() {
    std::vector<Pending> expired;
    { std::lock_guard<std::mutex> lock(pending_mutex_);
      for (auto it = pending_.begin(); it != pending_.end();) {
        if (Clock::now() >= it->second.deadline) { expired.push_back(std::move(it->second)); it = pending_.erase(it); }
        else ++it;
      }
    }
    for (const auto &request : expired)
      finish(request.complete, {{"error", "operations response timed out; action outcome unknown; not retried"}});
    std::vector<std::uint64_t> client_expired;
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      for (auto it = client_pending_.begin(); it != client_pending_.end();) {
        if (Clock::now() >= it->second.deadline) {
          it->second.client->remove_pending_request(it->second.request_id);
          client_expired.push_back(it->first);
          it = client_pending_.erase(it);
        } else ++it;
      }
    }
    for (auto id : client_expired)
      queue_control({{"op", "reply"}, {"id", id}, {"error", "operations downstream response timed out; not retried"}});
  }
  void read_loop() {
    try {
      while (!stopping_) {
        std::uint32_t size;
        if (!exact_io(fd_, &size, sizeof(size), false)) break;
        size = ntohl(size);
        if (!size || size > kMaxFrame) throw std::runtime_error("invalid operations frame size");
        std::string text(size, '\0');
        if (!exact_io(fd_, text.data(), text.size(), false)) break;
        Json message = Json::parse(text);
        try { dispatch(message); }
        catch (const std::exception &error) {
          if (message.contains("id"))
            queue_control({{"op", "reply"}, {"id", message["id"]}, {"error", error.what()}});
          RCLCPP_ERROR(get_logger(), "Operations worker message failed: %s", error.what());
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Operations IPC failure: %s", error.what());
    }
    disconnected();
  }
  void dispatch(const Json &message) {
    const auto op = message.at("op").get<std::string>();
    if (op == "ready") { if(!draining_)ready_ = true; return; }
    if (op == "drained") {
      ready_=false;
      draining_=true;
      drain_success_=message.at("success");
      if(!drain_success_) disconnected();
      drained_=true;
      return;
    }
    if (op == "player_status") {reply(message.at("id"),observer_->player_status(steady_now()));return;}
    if (op == "runner_busy") {reply(message.at("id"),runner_.busy());return;}
    if (op == "observer_runtime") {
      observer_->runtime(message.at("runtime"));
      if(message.contains("transport_metrics")){std::lock_guard<std::mutex> lock(status_mutex_);worker_metrics_=message["transport_metrics"];}
      return;
    }
    if(op=="runner_config") {
      runner_.configure(message.at("timeout"),message.at("seed"));reply(message.at("id"),true);return;
    }
    if(op=="runner_action") {
      const auto id=message.at("id").get<std::uint64_t>();
      runner_.start(message.at("action"),steady_now(),[this,id](const Json &value){
        // Never block native stop dispatch or sensor ingress on a worker reply.
        queue_control({{"op","reply"},{"id",id},{"value",value}});
      },[this](const auto &data){Array m;m.data=data;runner_publisher_->publish(m);},
        message.at("epoch").get<std::uint64_t>(),message.at("received").get<double>());
      return;
    }
    if (op == "wire_snapshot") { reply(message.at("id"),observer_->snapshot(message.at("name"))); return; }
    if (op == "wire_wait") {
      const auto id=message.at("id").get<std::uint64_t>();
      observer_->wait(message.at("name"),message.at("generation"),message.at("timeout"),
        [this,id](const Json &value){reply(id,value);});return;
    }
    if (op == "operations_status") {
      String status;
      status.data = message.at("status").get<std::string>();
      { std::lock_guard<std::mutex> lock(status_mutex_); worker_status_ = status.data; }
      status_->publish(status);
      return;
    }
    if (op == "client_cancel") {
      std::lock_guard<std::mutex> lock(client_mutex_);
      const auto it = client_pending_.find(message.at("call_id").get<std::uint64_t>());
      if (it != client_pending_.end()) {
        it->second.client->remove_pending_request(it->second.request_id);
        client_pending_.erase(it);
      }
      return;
    }
    if (op == "result") {
      std::function<void(const Json &)> complete;
      { std::lock_guard<std::mutex> lock(pending_mutex_);
        auto it = pending_.find(message.at("id").get<std::uint64_t>());
        if (it != pending_.end()) { complete = std::move(it->second.complete); pending_.erase(it); } }
      if (complete) finish(complete, message.contains("error") ? message : message.at("response"));
      return;
    }
    if (op == "publish") {
      std::function<void(const Json &)> publish;
      { std::lock_guard<std::mutex> lock(entity_mutex_); publish = publishers_.at(message.at("entity")); }
      publish(message.at("message"));
      return;
    }
    const auto id = message.at("id").get<std::uint64_t>();
    if (op == "parameter") {
      const auto name = message.at("name").get<std::string>();
      if (!has_parameter(name)) {
        const auto &value = message.at("default");
        if (value.is_boolean()) declare_parameter(name, value.get<bool>());
        else if (value.is_number_integer()) declare_parameter(name, value.get<std::int64_t>());
        else if (value.is_number()) declare_parameter(name, value.get<double>());
        else declare_parameter(name, value.get<std::string>());
      }
      reply(id, parameter_value(get_parameter(name))); return;
    }
    const auto entity = message.value("entity", std::uint64_t{});
    if (op == "publisher") add_publisher(entity, message);
    else if (op == "subscription") add_subscription(entity, message);
    else if (op == "remove_subscription") {
      std::lock_guard<std::mutex> lock(entity_mutex_); readers_.erase(entity);
    } else if (op == "service") add_service(entity, message);
    else if (op == "client") {
      if (message.at("type") != "std_srvs/srv/Trigger") throw std::runtime_error("unsupported worker client");
      auto client = create_client<Trigger>(message.at("topic"), rclcpp::ServicesQoS(), service_group_);
      std::lock_guard<std::mutex> lock(entity_mutex_); clients_[entity] = client;
    } else if (op == "client_ready" || op == "client_call") {
      rclcpp::Client<Trigger>::SharedPtr client;
      { std::lock_guard<std::mutex> lock(entity_mutex_); client = clients_.at(entity); }
      if (op == "client_ready") { reply(id, client->service_is_ready()); return; }
      // Keep cancellation/timeout cleanup equivalent to rclpy's pending
      // request cleanup. A missing laptop service must not leak client futures.
      std::lock_guard<std::mutex> lock(client_mutex_);
      const auto request = client->async_send_request(std::make_shared<Trigger::Request>(),
          [this, id](rclcpp::Client<Trigger>::SharedFuture future) {
            { std::lock_guard<std::mutex> guard(client_mutex_); client_pending_.erase(id); }
            try {
              const auto response = future.get();
              reply(id, {{"success", response->success}, {"message", response->message}});
            } catch (const std::exception &error) {
              queue_control({{"op", "reply"}, {"id", id}, {"error", error.what()}});
            }
          });
      client_pending_.emplace(id, ClientPending{client, request.request_id,
          rpc_deadline()});
      return;
    } else if (op == "publishers_info") {
      Json infos = Json::array();
      for (const auto &info : get_publishers_info_by_topic(message.at("topic")))
        infos.push_back({{"node_name", info.node_name()}, {"node_namespace", info.node_namespace()},
                         {"endpoint_gid", info.endpoint_gid()}});
      reply(id, infos); return;
    } else throw std::runtime_error("unknown operations worker operation");
    reply(id, true);
  }
  template<class T, class Fill> void publisher(std::uint64_t id, const Json &spec, Fill fill) {
    auto pub = create_publisher<T>(spec.at("topic"), qos(spec.at("qos")));
    std::lock_guard<std::mutex> lock(entity_mutex_);
    publishers_[id] = [pub, fill](const Json &value) { T message; fill(message, value); pub->publish(message); };
  }
  void add_publisher(std::uint64_t id, const Json &spec) {
    const std::string type = spec.at("type");
    if (type == "std_msgs/msg/String") publisher<String>(id, spec, [](auto &m, const Json &v) { m.data = v.at("data").get<std::string>(); });
    else if (type == "std_msgs/msg/Bool") publisher<std_msgs::msg::Bool>(id, spec, [](auto &m, const Json &v) { m.data = v.at("data").get<bool>(); });
    else if (type == "std_msgs/msg/Float64") publisher<std_msgs::msg::Float64>(id, spec, [](auto &m, const Json &v) { m.data = real(v.at("data")); });
    else if (type == "std_msgs/msg/UInt64") publisher<std_msgs::msg::UInt64>(id, spec, [](auto &m, const Json &v) { m.data = v.at("data").get<std::uint64_t>(); });
    else if (type == "std_msgs/msg/UInt32") publisher<std_msgs::msg::UInt32>(id, spec, [](auto &m, const Json &v) { m.data = v.at("data").get<std::uint32_t>(); });
    else if (type == "std_msgs/msg/Float64MultiArray") publisher<Array>(id, spec, [](auto &m, const Json &v) {
      for (const auto &x : v.at("data")) m.data.push_back(real(x));
    });
    else throw std::runtime_error("unsupported worker publisher: " + type);
  }
  void add_service(std::uint64_t id, const Json &spec) {
    const std::string topic = spec.at("topic"), type = spec.at("type");
    rclcpp::ServiceBase::SharedPtr service;
    if (type == "std_srvs/srv/Trigger") {
      service = create_service<Trigger>(topic, [this, id, urgent=topic=="/hope/runner/emergency_passive"](rclcpp::Service<Trigger>::SharedPtr owner,
          std::shared_ptr<rmw_request_id_t> header, Trigger::Request::SharedPtr) {
        const auto complete=[owner, header](const Json &result) {
          Trigger::Response response;
          response.success = result.value("success", false);
          response.message = result.value("error", result.value("message", std::string{}));
          owner->send_response(*header, response);
        };
        if(urgent) {
          if(!ready_||draining_||dead_){finish(complete,{{"error","operations worker unavailable"}});return;}
          try {runner_.start("EMERGENCY_PASSIVE",steady_now(),[this,complete](const Json &v){finish(complete,v);},
            [this](const auto &data){Array m;m.data=data;runner_publisher_->publish(m);});}
          catch(const std::exception &e){finish(complete,{{"error",e.what()}});}
        }else call(id,Json::object(),complete);
      }, rclcpp::ServicesQoS(),topic=="/hope/runner/emergency_passive"?urgent_group_:request_group_);
    } else if (type == "rcl_interfaces/srv/SetParameters") {
      service = create_service<Parameters>(topic, [this, id](rclcpp::Service<Parameters>::SharedPtr owner,
          std::shared_ptr<rmw_request_id_t> header, Parameters::Request::SharedPtr request) {
        const auto count = request->parameters.size();
        const auto complete = [owner, header, count](const Json &result) {
          Parameters::Response response;
          if (result.contains("error")) {
            rcl_interfaces::msg::SetParametersResult value;
            value.successful = false; value.reason = result.at("error").get<std::string>();
            response.results.assign(count, value);
          } else for (const auto &item : result.at("results")) {
            rcl_interfaces::msg::SetParametersResult value;
            value.successful = item.at("successful").get<bool>(); value.reason = item.at("reason").get<std::string>();
            response.results.push_back(value);
          }
          owner->send_response(*header, response);
        };
        // Include request construction in the callback boundary, not just send().
        try {
          // Preserve the existing field API rejection before JSON escaping or
          // copying a potentially huge upload into the worker protocol.
          if (request->parameters.size() == 1 && request->parameters[0].name == "request" &&
              request->parameters[0].value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
              request->parameters[0].value.string_value.size() > 9U * 1024U * 1024U) {
            finish(complete, {{"error", "field request exceeds 9 MiB"}});
          } else call(id, parameters_request(*request), complete);
        } catch (const std::exception &error) {
          finish(complete, {{"error", error.what()}});
        }
      }, rclcpp::ServicesQoS(), request_group_);
    } else throw std::runtime_error("unsupported worker service: " + type);
    std::lock_guard<std::mutex> lock(entity_mutex_); services_[id] = service;
  }
  void add_subscription(std::uint64_t id, const Json &spec) {
    auto reader = std::make_shared<Reader>();
    reader->id = id; reader->topic = spec.at("topic"); reader->type = spec.at("type");
    rclcpp::SubscriptionOptions options; options.callback_group = input_group_;
    reader->subscription = create_generic_subscription(reader->topic, reader->type, qos(spec.at("qos")),
        [this, weak = std::weak_ptr<Reader>(reader)](std::shared_ptr<rclcpp::SerializedMessage> message, const rclcpp::MessageInfo &info) {
          if (auto input = weak.lock()) sample(input, message, info);
        }, options);
    std::lock_guard<std::mutex> lock(entity_mutex_); readers_[id] = reader;
  }
  void sample(const std::shared_ptr<Reader> &reader, const std::shared_ptr<rclcpp::SerializedMessage> &data, const rclcpp::MessageInfo &info) {
    const auto received=steady_now();
    const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if(!samples_alive_)return;
    std::lock_guard<std::mutex> lock(sample_mutex_);
    ++samples_received_;if(samples_.count(reader->id))++samples_coalesced_;
    samples_.insert_or_assign(reader->id,Sample{reader,data,info,received,wall});
    sample_cv_.notify_one();
  }
  void sample_loop() {
    while(!stopping_) {
      std::map<std::uint64_t,Sample> batch;
      {
        std::unique_lock<std::mutex> lock(sample_mutex_);
        sample_cv_.wait(lock,[this]{return stopping_||!samples_.empty();});
        if(stopping_)return;
        batch.swap(samples_);
      }
      for(const auto &entry:batch) {
        const auto &sample=entry.second;
        const auto &reader=sample.reader;const auto &data=*sample.data;const auto &info=sample.info;
    // Only qualification/configuration and explicitly enabled vendor diagnostics
    // cross this path. Sensor/Runner/player wires are owned by NativeObserver.
    try {
      Json packet{{"op","sample"},{"entity",reader->id},{"received",sample.received},
                  {"received_wall_ns",sample.wall_ns}};
      if (reader->type == "std_msgs/msg/String")
        packet["message"] = {{"data",decode<String>(data).data}};
      else if (reader->type == "std_msgs/msg/Bool")
        packet["message"] = {{"data",decode<std_msgs::msg::Bool>(data).data}};
      else {
        // Keep generated vendor CDR decoding and publisher identity in Python.
        const auto &raw = data.get_rcl_serialized_message();
        packet["cdr"] = std::vector<std::uint8_t>(raw.buffer,raw.buffer+raw.buffer_length);
        packet["publisher_gid"] = std::vector<std::uint8_t>(
          std::begin(info.get_rmw_message_info().publisher_gid.data),
          std::end(info.get_rmw_message_info().publisher_gid.data));
      }
      const auto text=packet.dump(-1,' ',false,Json::error_handler_t::replace);
      if(text.size()>kMaxFrame)throw std::runtime_error("diagnostic packet exceeds limit");
      std::uint32_t size=htonl(static_cast<std::uint32_t>(text.size()));
      // Backpressure affects only this dedicated diagnostic channel. The ROS
      // callback retains one raw sample per source, before CDR/JSON expansion.
      if(!exact_io(sample_fd_,&size,sizeof(size),true)||
         !exact_io(sample_fd_,const_cast<char*>(text.data()),text.size(),true)) {
        samples_alive_=false;::shutdown(sample_fd_,SHUT_RDWR);
        RCLCPP_WARN(get_logger(),"Optional diagnostic channel disconnected; retaining control owner");return;
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Dropped operations input on %s: %s", reader->topic.c_str(), error.what());
    }
      }
    }
  }
  void completion_loop() {
    while(!stopping_) {
      Json value;
      {std::unique_lock<std::mutex> lock(completion_mutex_);
        completion_cv_.wait(lock,[this]{return stopping_||!completions_.empty();});
        if(stopping_)return;
        value=std::move(completions_.front());completions_.pop_front();}
      try{send(value);}catch(const std::exception &e){RCLCPP_WARN(get_logger(),"Runner reply not delivered: %s",e.what());}
    }
  }
  std::string worker_path_, python_;
  double rpc_timeout_{};
  int fd_ = -1;
  int sample_fd_=-1;
  pid_t worker_pid_ = -1;
  std::thread reader_;
  std::thread sample_sender_,completion_sender_,urgent_thread_;
  rclcpp::executors::SingleThreadedExecutor urgent_executor_;
  std::atomic<bool> samples_alive_{false};
  std::mutex sample_mutex_,completion_mutex_;
  std::condition_variable sample_cv_,completion_cv_;
  std::map<std::uint64_t,Sample> samples_;
  std::deque<Json> completions_;
  std::uint64_t samples_received_=0,samples_coalesced_=0;
  Json worker_metrics_=Json::object();
  hope_operations::RunnerCommands runner_;
  rclcpp::Publisher<Array>::SharedPtr runner_publisher_;
  std::atomic<bool> ready_{false}, dead_{false}, stopping_{false};
  std::atomic<std::uint64_t> call_id_{0};
  std::timed_mutex write_mutex_;
  std::mutex pending_mutex_, entity_mutex_, client_mutex_, status_mutex_;
  std::string worker_status_ = "READY: native ROS operations, Python orchestration worker";
  std::map<std::uint64_t, Pending> pending_;
  std::map<std::uint64_t, std::function<void(const Json &)>> publishers_;
  std::map<std::uint64_t, std::shared_ptr<Reader>> readers_;
  std::map<std::uint64_t, rclcpp::ServiceBase::SharedPtr> services_;
  std::map<std::uint64_t, rclcpp::Client<Trigger>::SharedPtr> clients_;
  std::map<std::uint64_t, ClientPending> client_pending_;
  rclcpp::CallbackGroup::SharedPtr input_group_, service_group_, request_group_,urgent_group_;
  std::function<void()> enable_requests_;
  bool requests_enabled_ = false;  // Default mutually-exclusive timer group only.
  bool shutdown_enqueued_=false;
  rclcpp::Publisher<String>::SharedPtr status_,metrics_;
  rclcpp::TimerBase::SharedPtr status_timer_, expiry_timer_,runner_timer_;
  unsigned failed_status_ticks_ = 0;
  std::unique_ptr<hope_operations::NativeObserver> observer_;
  std::mutex fault_mutex_;
  std::map<std::string,std::string> native_faults_;
  std::atomic<bool> draining_{false},drained_{false};
  bool drain_success_=false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGTERM,stop_signal);std::signal(SIGINT,stop_signal);
  try {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false).start_parameter_event_publisher(false);
    auto node = std::make_shared<Operations>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    node->start(executor);
    executor.spin();
    executor.remove_node(node);
    if (node->worker_failed()) return 1;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "hope-operations: %s\n", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
