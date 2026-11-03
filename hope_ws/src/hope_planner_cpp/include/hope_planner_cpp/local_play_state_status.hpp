#pragma once

// Operator-local display and button intent. Never creates a DDS publisher.
// Socket I/O runs on one worker; the ROS callback only replaces an immutable
// snapshot. A stalled/disconnected Console cannot delay ball or pelvis data.
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace hope_planner_cpp {
class LocalPlayStateStatus {
 public:
  struct Intent { std::string action; std::uint64_t revision; };
  explicit LocalPlayStateStatus(const std::string& bind_address, int port,
                                std::string producer = "", std::string desktop_key = "")
      : producer_(std::move(producer)), desktop_key_(std::move(desktop_key)) {
    if (port < 0 || port > 65535) throw std::invalid_argument("invalid local status port");
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<unsigned short>(port));
    if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1)
      throw std::invalid_argument("invalid local status bind address");
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) throw std::runtime_error("cannot create local status socket");
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (::fcntl(fd_, F_SETFL, O_NONBLOCK) < 0 ||
        ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
        ::listen(fd_, 8) < 0) {
      ::close(fd_); throw std::runtime_error("cannot bind local ball-status listener");
    }
    socklen_t len = sizeof(address);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address), &len);
    port_ = ntohs(address.sin_port);
    try { worker_ = std::thread([this] { run(); }); }
    catch (...) { ::close(fd_); throw; }
  }
  ~LocalPlayStateStatus() {
    stopped_.store(true);
    if (worker_.joinable()) worker_.join();
    ::close(fd_);
  }
  LocalPlayStateStatus(const LocalPlayStateStatus&) = delete;
  LocalPlayStateStatus& operator=(const LocalPlayStateStatus&) = delete;
  int port() const { return port_; }
  void update(std::string text) {
    std::shared_ptr<const Snapshot> next = std::make_shared<Snapshot>(Snapshot{std::move(text), Clock::now()});
    std::atomic_store(&snapshot_, std::move(next));
  }
  std::vector<Intent> take_intents() {
    std::lock_guard<std::mutex> lock(intent_mutex_);
    std::vector<Intent> output(intents_.begin(), intents_.end());
    intents_.clear();return output;
  }

 private:
  using Clock = std::chrono::steady_clock;
  struct Snapshot { std::string text; Clock::time_point time; };
  static std::string header(const std::string& request, const std::string& name) {
    std::string result;bool found=false;
    auto start=request.find("\r\n")+2;
    while(start<request.size()) {
      const auto end=request.find("\r\n",start);
      if(end==std::string::npos || end==start)break;
      const auto colon=request.find(':',start);
      if(colon<end) {
        auto key=request.substr(start,colon-start);
        for(auto& c:key)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if(key==name) {
          if(found)return "\n"; // Distinguish duplicate headers from an absent Origin.
          found=true;auto value=colon+1;
          while(value<end && request[value]==' ')++value;
          result=request.substr(value,end-value);
        }
      }
      start=end+2;
    }
    return result;
  }
  static bool revision(const std::string& text, std::uint64_t& value) {
    if(text.empty() || text.size()>20)return false;
    value=0;
    for(char c:text) {
      if(c<'0' || c>'9' || value>(std::numeric_limits<std::uint64_t>::max()-(c-'0'))/10)return false;
      value=value*10+(c-'0');
    }
    return true;
  }
  bool key_matches(const std::string& key) const {
    if(key.size()!=64 || desktop_key_.size()!=64)return false;
    unsigned difference=0;
    for(std::size_t i=0;i<64;++i)difference|=static_cast<unsigned char>(key[i]^desktop_key_[i]);
    return difference==0;
  }
  bool ready(int fd, short events, Clock::time_point deadline) const {
    while (!stopped_.load() && Clock::now() < deadline) {
      pollfd p{fd, events, 0};
      const int n = ::poll(&p, 1, 10);
      if (n > 0) return (p.revents & events) != 0;
    }
    return false;
  }
  void client(int fd) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(100);
    std::string request;
    while (request.size() < 4096 && request.find("\r\n\r\n") == std::string::npos) {
      if (!ready(fd, POLLIN, deadline)) return;
      char buffer[1024]; const auto n = ::recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0) return;
      request.append(buffer, static_cast<std::size_t>(n));
    }
    if (request.size() >= 4096) return;
    const auto origin=header(request,"origin");
    const bool preflight=request.rfind("OPTIONS ",0)==0;
    const bool read_only=request.rfind("GET /ball-state HTTP/1.",0)==0;
    const bool web_origin=origin=="https://app.foxglove.dev";
    // Electron Desktop has an opaque file:// origin, shared with hostile
    // sandboxed web pages. Never authorize it using Origin:null alone.
    const bool desktop_origin=origin=="null" && !desktop_key_.empty();
    const bool desktop_authorized=desktop_origin && key_matches(header(request,"x-hope-local-key"));
    const bool authorized=web_origin || desktop_authorized || (origin.empty() && read_only);
    // Preflight carries header NAMES, never the credential. Its success permits
    // the browser to attempt a POST; the actual request still needs the key.
    const bool preflight_allowed=web_origin || desktop_origin;
    const bool cors_allowed=web_origin || desktop_authorized || (preflight && preflight_allowed);
    std::string code = "404 Not Found", body = "{}";
    if ((!preflight && !authorized) || (preflight && !preflight_allowed)) {
      code="403 Forbidden";
    } else if (request.rfind("OPTIONS /ball-state HTTP/1.", 0) == 0 ||
               request.rfind("OPTIONS /ball-state/intent/",0)==0) {
      code = "204 No Content"; body.clear();
    } else if (request.rfind("GET /ball-state HTTP/1.", 0) == 0) {
      const auto snapshot = std::atomic_load(&snapshot_);
      if (snapshot && Clock::now() - snapshot->time < std::chrono::seconds(1)) {
        code = "200 OK"; body = snapshot->text;
      } else { code = "503 Service Unavailable"; }
    } else {
      std::string action;
      for(const char* name:{"READY","SERVE","STOP"})
        if(request.rfind(std::string("POST /ball-state/intent/")+name+" HTTP/1.",0)==0)action=name;
      if(!action.empty()) {
        const auto snapshot=std::atomic_load(&snapshot_);
        std::uint64_t expected=0;
        const auto length=header(request,"content-length");
        if((!length.empty() && length!="0") || !header(request,"transfer-encoding").empty()) {
          code="400 Bad Request";
        } else if(!snapshot || Clock::now()-snapshot->time>=std::chrono::seconds(1)) {
          code="503 Service Unavailable";
        } else if(producer_.empty() || header(request,"x-hope-producer")!=producer_ ||
                  !revision(header(request,"x-hope-intent-revision"),expected)) {
          code="409 Conflict";
        } else {
          std::lock_guard<std::mutex> lock(intent_mutex_);
          if(expected!=intent_revision_ || intent_revision_==std::numeric_limits<std::uint64_t>::max()) {
            code="409 Conflict";
          } else if(intents_.size()>=8) {
            code="429 Too Many Requests";
          } else {
            intents_.push_back(Intent{action,++intent_revision_});
            code="202 Accepted";body="{\"intent_revision\":\""+std::to_string(intent_revision_)+"\"}";
          }
        }
      }
    }
    const std::string response = "HTTP/1.1 " + code + "\r\nContent-Type: application/json\r\n"
        "Cache-Control: no-store\r\nVary: Origin\r\n" +
        (cors_allowed ? "Access-Control-Allow-Origin: "+origin+"\r\n"
          "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
          "Access-Control-Allow-Headers: X-Hope-Producer, X-Hope-Intent-Revision, X-Hope-Local-Key\r\n"
          "Access-Control-Allow-Private-Network: true\r\n" : "") +
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
    std::size_t sent = 0;
    while (sent < response.size() && ready(fd, POLLOUT, deadline)) {
#ifdef MSG_NOSIGNAL
      constexpr int flags = MSG_NOSIGNAL;
#else
      constexpr int flags = 0; // macOS uses SO_NOSIGPIPE below.
#endif
      const auto n = ::send(fd, response.data() + sent, response.size() - sent, flags);
      if (n <= 0) return;
      sent += static_cast<std::size_t>(n);
    }
  }
  void run() noexcept {
    while (!stopped_.load()) {
      if (!ready(fd_, POLLIN, Clock::now() + std::chrono::milliseconds(50))) continue;
      const int client_fd = ::accept(fd_, nullptr, nullptr);
      if (client_fd < 0) continue;
#ifdef SO_NOSIGPIPE
      const int one = 1;
      ::setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
      if (::fcntl(client_fd, F_SETFL, O_NONBLOCK) >= 0) {
        try { client(client_fd); } catch (...) { /* Display failures never stop the gate. */ }
      }
      ::close(client_fd);
    }
  }
  int fd_ = -1, port_ = 0;
  const std::string producer_, desktop_key_;
  std::mutex intent_mutex_;
  std::deque<Intent> intents_;
  std::uint64_t intent_revision_ = 0;
  std::atomic<bool> stopped_{false};
  std::shared_ptr<const Snapshot> snapshot_;
  std::thread worker_;
};
}  // namespace hope_planner_cpp
