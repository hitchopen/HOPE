#include "a3_pingpong/pp_gripper_worker.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <charconv>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <filesystem>
#include <limits>
#include <mutex>
#include <optional>
#include <poll.h>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace a3_pingpong {
namespace {

constexpr std::size_t kMaxPacketBytes = 512;
constexpr std::size_t kResponseFields = 12;
constexpr std::uint64_t kBurstCount = 5;

bool PollFd(int fd, short events, std::chrono::milliseconds timeout,
            std::string& error) {
  pollfd descriptor{fd, events, 0};
  int result = 0;
  do {
    result = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  } while (result < 0 && errno == EINTR);
  if (result == 0) {
    error = "gripper IPC timed out";
    return false;
  }
  if (result < 0) {
    error = "gripper IPC poll failed: " +
            std::string(std::strerror(errno));
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 ||
      (descriptor.revents & events) == 0) {
    error = "gripper IPC became unavailable";
    return false;
  }
  return true;
}

std::vector<std::string_view> SplitTabs(const std::string& packet) {
  std::vector<std::string_view> fields;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = packet.find('\t', begin);
    fields.emplace_back(
        packet.data() + begin,
        (end == std::string::npos ? packet.size() : end) - begin);
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return fields;
}

bool ParseUint(std::string_view text, std::uint64_t& value,
               bool positive) {
  if (text.empty()) return false;
  value = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  return parsed.ec == std::errc{} &&
         parsed.ptr == text.data() + text.size() &&
         (!positive || value > 0);
}

bool IsSafeToken(std::string_view value) {
  if (value.empty()) return false;
  for (const unsigned char character : value) {
    if (!((character >= 'a' && character <= 'z') ||
          (character >= 'A' && character <= 'Z') ||
          (character >= '0' && character <= '9') || character == '_' ||
          character == '.' || character == ':' || character == '-')) {
      return false;
    }
  }
  return true;
}

bool IsBridgeState(std::string_view state) {
  return state == "IDLE" || state == "OPEN" || state == "GRABBED" ||
         state == "RELEASED" || state == "STOPPING" || state == "FAULT";
}

bool ParseResponse(const std::string& packet, std::uint64_t request_id,
                   PpGripperCommand command, PpGripperReceipt& receipt,
                   std::string& error) {
  if (packet.empty() || packet.size() > kMaxPacketBytes ||
      packet.find_first_of("\r\n\0") != std::string::npos) {
    error = "malformed gripper IPC response bytes";
    return false;
  }
  const auto fields = SplitTabs(packet);
  if (fields.size() != kResponseFields ||
      fields[0] != kPpGripperProtocolVersion) {
    error = "gripper IPC response schema/version mismatch";
    return false;
  }
  std::uint64_t parsed_request = 0;
  if (!ParseUint(fields[1], parsed_request, true) ||
      parsed_request != request_id ||
      (fields[2] != "OK" && fields[2] != "ERR") ||
      fields[3] != PpGripperCommandName(command) ||
      !IsBridgeState(fields[4])) {
    error = "gripper IPC response identity/state mismatch";
    return false;
  }
  std::array<std::uint64_t, 4> numbers{};
  for (std::size_t index = 0; index < numbers.size(); ++index) {
    if (!ParseUint(fields[5 + index], numbers[index], false)) {
      error = "gripper IPC response contains an invalid integer";
      return false;
    }
  }
  if ((fields[9] != "0" && fields[9] != "1") ||
      (fields[10] != "0" && fields[10] != "1") ||
      !IsSafeToken(fields[11])) {
    error = "gripper IPC response capability/detail mismatch";
    return false;
  }
  receipt = {};
  receipt.request_id = parsed_request;
  receipt.ok = fields[2] == "OK";
  receipt.command = command;
  receipt.state = std::string(fields[4]);
  receipt.first_publish_monotonic_ns = numbers[0];
  receipt.published_before_ack = numbers[1];
  receipt.planned_publish_count = numbers[2];
  receipt.matched_subscribers = numbers[3];
  receipt.physical_confirmation = fields[9] == "1";
  receipt.state_observable = fields[10] == "1";
  receipt.detail = std::string(fields[11]);
  if (receipt.physical_confirmation || receipt.state_observable) {
    error =
        "gripper publish receipt must not claim physical confirmation/state";
    return false;
  }
  return true;
}

bool ReceiptValid(const PpGripperReceipt& receipt,
                  PpGripperCommand command) {
  // The V3 IPC wraps five independent HAL RPC dispatches using c89e5ab's
  // hardware-proven presets and ordering. A receipt is not physical gripper
  // feedback and never gates the body Serve program.
  if (!receipt.ok || receipt.command != command ||
      receipt.matched_subscribers == 0 ||
      receipt.physical_confirmation || receipt.state_observable) {
    return false;
  }
  switch (command) {
    case PpGripperCommand::kStatus:
      return receipt.state != "FAULT";
    case PpGripperCommand::kOpen:
      return receipt.state == "OPEN" &&
             receipt.first_publish_monotonic_ns > 0 &&
             receipt.published_before_ack >= 1 &&
             receipt.published_before_ack <= kBurstCount &&
             receipt.planned_publish_count == kBurstCount;
    case PpGripperCommand::kGrab:
      return receipt.state == "GRABBED" &&
             receipt.first_publish_monotonic_ns > 0 &&
             receipt.published_before_ack >= 1 &&
             receipt.published_before_ack <= kBurstCount &&
             receipt.planned_publish_count == kBurstCount;
    case PpGripperCommand::kRelease:
      return receipt.state == "RELEASED" &&
             receipt.first_publish_monotonic_ns > 0 &&
             receipt.published_before_ack >= 1 &&
             receipt.published_before_ack <= kBurstCount &&
             receipt.planned_publish_count == kBurstCount;
  }
  return false;
}

bool ValidateSocketPath(const std::string& socket_path,
                        std::string& error) {
  namespace fs = std::filesystem;
  const fs::path path(socket_path);
  if (!path.is_absolute() || path.filename().empty()) {
    error = "gripper IPC socket path must be absolute";
    return false;
  }
  struct stat parent {};
  if (::lstat(path.parent_path().c_str(), &parent) != 0 ||
      !S_ISDIR(parent.st_mode) || S_ISLNK(parent.st_mode) ||
      parent.st_uid != ::geteuid() || (parent.st_mode & 0777) != 0700) {
    error = "gripper IPC parent must be an owner-only 0700 directory";
    return false;
  }
  struct stat endpoint {};
  if (::lstat(path.c_str(), &endpoint) != 0 ||
      !S_ISSOCK(endpoint.st_mode) || endpoint.st_uid != ::geteuid() ||
      (endpoint.st_mode & 0777) != 0600) {
    error = "gripper IPC endpoint must be an owner-only 0600 Unix socket";
    return false;
  }
  return true;
}

class IpcClient final {
 public:
  ~IpcClient() { Close(); }

  bool Connect(const std::string& socket_path,
               std::chrono::milliseconds timeout, std::string& error) {
    Close();
    if (timeout.count() <= 0 ||
        !ValidateSocketPath(socket_path, error)) {
      return false;
    }
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
      error = "gripper IPC socket path is too long";
      return false;
    }
    const int descriptor =
        ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (descriptor < 0) {
      error = "cannot create gripper IPC socket: " +
              std::string(std::strerror(errno));
      return false;
    }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socket_path.c_str(),
                socket_path.size() + 1);
    int result =
        ::connect(descriptor, reinterpret_cast<sockaddr*>(&address),
                  sizeof(address));
    if (result != 0 && errno == EINPROGRESS) {
      if (!PollFd(descriptor, POLLOUT, timeout, error)) {
        ::close(descriptor);
        return false;
      }
      int socket_error = 0;
      socklen_t length = sizeof(socket_error);
      if (::getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &socket_error,
                       &length) != 0 ||
          socket_error != 0) {
        error = "cannot connect gripper IPC socket";
        ::close(descriptor);
        return false;
      }
    } else if (result != 0) {
      error = "cannot connect gripper IPC socket: " +
              std::string(std::strerror(errno));
      ::close(descriptor);
      return false;
    }
#ifdef SO_PEERCRED
    ucred credentials{};
    socklen_t credentials_length = sizeof(credentials);
    if (::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials,
                     &credentials_length) != 0 ||
        credentials_length != sizeof(credentials) ||
        credentials.uid != ::geteuid()) {
      error = "gripper IPC peer UID does not match the Runner";
      ::close(descriptor);
      return false;
    }
#else
    error = "gripper IPC requires SO_PEERCRED";
    ::close(descriptor);
    return false;
#endif
    fd_ = descriptor;
    return true;
  }

  bool Exchange(PpGripperCommand command,
                std::chrono::milliseconds timeout,
                PpGripperReceipt& receipt, std::string& error) {
    if (fd_ < 0 || timeout.count() <= 0) {
      error = "gripper IPC is not connected or timeout is invalid";
      return false;
    }
    if (next_request_id_ == 0 ||
        next_request_id_ >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max())) {
      error = "gripper IPC request id exhausted";
      return false;
    }
    const std::uint64_t request_id = next_request_id_++;
    const std::string packet =
        std::string(kPpGripperProtocolVersion) + "\t" +
        std::to_string(request_id) + "\t" +
        PpGripperCommandName(command);
    if (!PollFd(fd_, POLLOUT, timeout, error)) return false;
    const ssize_t sent =
        ::send(fd_, packet.data(), packet.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(packet.size())) {
      error = "gripper IPC request send failed";
      return false;
    }
    if (!PollFd(fd_, POLLIN, timeout, error)) return false;
    std::array<char, kMaxPacketBytes + 1> response{};
    const ssize_t received =
        ::recv(fd_, response.data(), kMaxPacketBytes, MSG_TRUNC);
    if (received <= 0 ||
        received > static_cast<ssize_t>(kMaxPacketBytes)) {
      error = "gripper IPC response is empty or truncated";
      return false;
    }
    if (!ParseResponse(std::string(response.data(), received), request_id,
                       command, receipt, error)) {
      return false;
    }
    if (!receipt.ok) {
      error = "gripper bridge rejected " +
              std::string(PpGripperCommandName(command)) + ": " +
              receipt.detail;
      return false;
    }
    if (!ReceiptValid(receipt, command)) {
      error = "gripper bridge returned an invalid publish receipt for " +
              std::string(PpGripperCommandName(command));
      return false;
    }
    return true;
  }

  void Close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  int fd_{-1};
  std::uint64_t next_request_id_{1};
};

}  // namespace

const char* PpGripperCommandName(PpGripperCommand command) noexcept {
  switch (command) {
    case PpGripperCommand::kStatus: return "STATUS";
    case PpGripperCommand::kOpen: return "OPEN";
    case PpGripperCommand::kGrab: return "GRAB";
    case PpGripperCommand::kRelease: return "RELEASE";
  }
  return "INVALID";
}

class PpGripperWorker::Impl final {
 public:
  struct Request {
    PpGripperCommand command{PpGripperCommand::kStatus};
    std::uint64_t generation{0};
  };

  Impl(PpGripperWorkerTransport transport,
       std::chrono::milliseconds exchange_timeout)
      : transport_(std::move(transport)),
        exchange_timeout_(exchange_timeout) {}

  ~Impl() { Stop(); }

  bool Start(std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (start_attempted_) {
      error = "gripper worker Start is one-shot";
      return false;
    }
    start_attempted_ = true;
    if (!transport_.exchange || exchange_timeout_.count() <= 0 ||
        exchange_timeout_ > std::chrono::milliseconds(5000)) {
      error = "gripper worker requires a transport and timeout in (0,5000] ms";
      faulted_ = true;
      fault_reason_ = error;
      return false;
    }
    try {
      thread_ = std::thread(&Impl::Run, this);
    } catch (const std::exception& exception) {
      error = "cannot start gripper worker: " +
              std::string(exception.what());
      faulted_ = true;
      fault_reason_ = error;
      return false;
    }
    started_ = true;
    return true;
  }

  bool Submit(PpGripperCommand command, std::uint64_t generation,
              std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stop_requested_ || worker_exited_) {
      error = "gripper worker is not running";
      return false;
    }
    if (command == PpGripperCommand::kStatus ||
        generation == 0 || generation <= submitted_generation_) {
      error = "invalid gripper command/generation";
      return false;
    }
    if (pending_.has_value() || request_in_flight_) {
      error = "gripper worker already has a request in flight";
      return false;
    }
    submitted_generation_ = generation;
    // OPEN, GRAB and RELEASE are independent best-effort edges. A failed
    // preparation request must never suppress READY_TO_SERVE's later RELEASE.
    faulted_ = false;
    fault_reason_.clear();
    pending_ = Request{command, generation};
    condition_.notify_one();
    return true;
  }

  PpGripperWorkerSnapshot Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    PpGripperWorkerSnapshot snapshot;
    snapshot.running =
        started_ && !stop_requested_ && !worker_exited_;
    snapshot.request_in_flight =
        request_in_flight_ || pending_.has_value();
    snapshot.faulted = faulted_;
    snapshot.stopped = stopped_;
    snapshot.submitted_generation = submitted_generation_;
    snapshot.acknowledged_generation = acknowledged_generation_;
    snapshot.acknowledged_command = acknowledged_command_;
    snapshot.bridge_state = bridge_state_;
    snapshot.first_publish_monotonic_ns = first_publish_monotonic_ns_;
    snapshot.fault_reason = fault_reason_;
    return snapshot;
  }

  void Stop() noexcept {
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) return;
      stop_requested_ = true;
      pending_.reset();
      condition_.notify_all();
    }
    if (thread_.joinable()) thread_.join();
    if (transport_.close) {
      try {
        transport_.close();
      } catch (...) {
      }
      transport_.close = {};
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }

 private:
  void Run() noexcept {
    while (true) {
      Request request;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
          return stop_requested_ || pending_.has_value();
        });
        if (stop_requested_) break;
        request = *pending_;
        pending_.reset();
        request_in_flight_ = true;
      }

      PpGripperReceipt receipt;
      std::string error;
      bool exchanged = false;
      try {
        exchanged = transport_.exchange(
            request.command, exchange_timeout_, receipt, error);
      } catch (const std::exception& exception) {
        error = "gripper transport threw: " +
                std::string(exception.what());
      } catch (...) {
        error = "gripper transport threw";
      }

      std::lock_guard<std::mutex> lock(mutex_);
      request_in_flight_ = false;
      if (!exchanged || !ReceiptValid(receipt, request.command)) {
        faulted_ = true;
        fault_reason_ =
            error.empty() ? "invalid gripper publish receipt" : error;
        // Keep the bounded worker alive. The next generation is allowed to
        // retry any independent actuator edge, especially timed RELEASE.
        continue;
      }
      faulted_ = false;
      fault_reason_.clear();
      acknowledged_generation_ = request.generation;
      acknowledged_command_ = request.command;
      bridge_state_ = receipt.state;
      first_publish_monotonic_ns_ =
          receipt.first_publish_monotonic_ns;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    worker_exited_ = true;
  }

  PpGripperWorkerTransport transport_;
  std::chrono::milliseconds exchange_timeout_;
  std::mutex stop_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread thread_;
  std::optional<Request> pending_;
  bool start_attempted_{false};
  bool started_{false};
  bool stop_requested_{false};
  bool worker_exited_{false};
  bool request_in_flight_{false};
  bool faulted_{false};
  bool stopped_{false};
  std::uint64_t submitted_generation_{0};
  std::uint64_t acknowledged_generation_{0};
  PpGripperCommand acknowledged_command_{PpGripperCommand::kStatus};
  std::string bridge_state_{"UNKNOWN"};
  std::uint64_t first_publish_monotonic_ns_{0};
  std::string fault_reason_;
};

PpGripperWorker::PpGripperWorker(
    PpGripperWorkerTransport transport,
    std::chrono::milliseconds exchange_timeout)
    : impl_(std::make_unique<Impl>(std::move(transport),
                                   exchange_timeout)) {}

PpGripperWorker::~PpGripperWorker() = default;

std::unique_ptr<PpGripperWorker> PpGripperWorker::Connect(
    const std::string& socket_path, std::string& error) {
  auto client = std::make_shared<IpcClient>();
  if (!client->Connect(socket_path, std::chrono::milliseconds(1000),
                       error)) {
    return nullptr;
  }
  PpGripperReceipt status;
  if (!client->Exchange(PpGripperCommand::kStatus,
                        std::chrono::milliseconds(1000), status, error)) {
    client->Close();
    return nullptr;
  }
  auto worker = std::make_unique<PpGripperWorker>(
      PpGripperWorkerTransport{
          [client](PpGripperCommand command,
                   std::chrono::milliseconds timeout,
                   PpGripperReceipt& receipt, std::string& exchange_error) {
            return client->Exchange(command, timeout, receipt,
                                    exchange_error);
          },
          [client] { client->Close(); }});
  if (!worker->Start(error)) return nullptr;
  return worker;
}

bool PpGripperWorker::Start(std::string& error) {
  return impl_->Start(error);
}

bool PpGripperWorker::Submit(PpGripperCommand command,
                             std::uint64_t generation,
                             std::string& error) {
  return impl_->Submit(command, generation, error);
}

PpGripperWorkerSnapshot PpGripperWorker::Snapshot() const {
  return impl_->Snapshot();
}

void PpGripperWorker::Stop() noexcept { impl_->Stop(); }

}  // namespace a3_pingpong
