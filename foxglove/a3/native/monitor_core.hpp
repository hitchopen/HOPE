// Pure monitor rules and bounded read-only probes. No ROS or robot-control calls.
#pragma once
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;

namespace hope_monitor {
using Steady = std::chrono::steady_clock;
// One persistent thread per probe, one job at a time, no queue growth. Workers
// never call ROS. Even thread-creation failure is returned as probe unavailability.
template<class T> class ProbeWorker {
 public:
  ~ProbeWorker() { stop(); }
  std::future<T> submit(std::function<T()> function) {
    std::packaged_task<T()> task([this, function = std::move(function)] {
      try {
        auto result = function();
        { std::lock_guard<std::mutex> lock(mutex_); busy_ = false; }
        return result;
      } catch (...) {
        { std::lock_guard<std::mutex> lock(mutex_); busy_ = false; }
        throw;
      }
    });
    auto future = task.get_future();
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (stopping_ || busy_) throw std::runtime_error("probe worker unavailable/busy");
      if (!thread_.joinable()) thread_ = std::thread([this] { run(); });
      job_.emplace(std::move(task)); busy_ = true; condition_.notify_one();
    } catch (...) {
      std::promise<T> error; future = error.get_future(); error.set_exception(std::current_exception());
    }
    return future;
  }
  void stop() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; condition_.notify_one(); }
    if (thread_.joinable()) thread_.join();
  }
 private:
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      condition_.wait(lock, [this] { return stopping_ || job_.has_value(); });
      if (!job_) return;
      auto job = std::move(*job_); job_.reset(); lock.unlock();
      job();  // packaged_task captures exceptions, keeping this worker alive.
      lock.lock();
    }
  }
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<std::packaged_task<T()>> job_;
  bool stopping_ = false, busy_ = false;
  std::thread thread_;
};
inline double mono() {
  return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
}
inline std::string trim(const std::string &s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  return a == std::string::npos ? "" : s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}
inline std::string read_file(const std::filesystem::path &p) {
  std::ifstream in(p);
  if (!in) throw std::runtime_error("Cannot read " + p.string());
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
inline std::string fixed(double v, int precision = 3, bool sign = false) {
  std::ostringstream out;
  if (sign) out << std::showpos;
  out << std::fixed << std::setprecision(precision) << v;
  return out.str();
}
inline std::string json_string(const std::string &value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c);
    else out << c;
  }
  out << '"'; return out.str();
}
struct CommandResult { int status; std::string output; };
// posix_spawn avoids unsafe fork-side work in a threaded DDS process. No shell,
// fixed argument vectors, capped output, monotonic timeout, and always reap.
inline CommandResult command(const std::vector<std::string> &args, double timeout = 3) {
  if (args.empty()) throw std::invalid_argument("empty command");
  int fds[2];
  if (pipe2(fds, O_CLOEXEC) != 0) throw std::runtime_error("probe pipe failed");
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attributes;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
  // Match subprocess.capture_output semantics: stderr must not corrupt a valid
  // chronyc CSV or systemctl state on a successful command.
  posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
  posix_spawn_file_actions_addclose(&actions, fds[0]);
  posix_spawn_file_actions_addclose(&actions, fds[1]);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  posix_spawnattr_init(&attributes);
  posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attributes, 0);
  std::vector<char *> argv;
  for (const auto &s : args) argv.push_back(const_cast<char *>(s.c_str()));
  argv.push_back(nullptr);
  pid_t pid{};
  const int error = posix_spawnp(&pid, argv[0], &actions, &attributes, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attributes);
  close(fds[1]);
  if (error) { close(fds[0]); throw std::runtime_error("probe spawn: " + std::string(strerror(error))); }
  fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL) | O_NONBLOCK);
  const auto deadline = Steady::now() + std::chrono::duration<double>(timeout);
  std::string output;
  bool eof = false, exited = false, failed = false;
  int status = 0;
  while (!eof || !exited) {
    char buffer[4096];
    ssize_t n;
    while ((n = read(fds[0], buffer, sizeof(buffer))) > 0) {
      output.append(buffer, static_cast<size_t>(n));
      if (output.size() > 65536) { failed = true; break; }
    }
    if (n == 0) eof = true;
    if (!exited) exited = waitpid(pid, &status, WNOHANG) == pid;
    if (failed || Steady::now() >= deadline) { failed = true; break; }
    if (!eof || !exited) { pollfd fd{fds[0], POLLIN, 0}; poll(&fd, 1, 10); }
  }
  if (failed) {
    kill(-pid, SIGKILL);  // Only the probe's new process group, never a vendor service.
    if (!exited) while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
  }
  close(fds[0]);
  if (failed) throw std::runtime_error("probe timeout/output limit: " + args[0]);
  return {WIFEXITED(status) ? WEXITSTATUS(status) : 128, output};
}
struct Ntp {
  double offset = NAN, skew = NAN, dispersion = NAN;
  bool qualified = false, pass = false;
  std::string error;
};
inline Ntp parse_ntp(const std::string &tracking, const std::string &sources,
                     double max_offset, double max_skew) {
  if (!std::isfinite(max_offset) || max_offset <= 0 || !std::isfinite(max_skew) || max_skew < 0)
    throw std::invalid_argument("Invalid NTP limits");
  std::vector<std::string> fields;
  std::istringstream csv(trim(tracking)); std::string s;
  while (std::getline(csv, s, ',')) fields.push_back(s);
  if (fields.size() < 14) throw std::runtime_error("chronyc tracking expected at least 14 fields");
  auto number = [&](size_t i) { size_t n; auto v = std::stod(trim(fields[i]), &n);
    if (n != trim(fields[i]).size()) throw std::runtime_error("Invalid chrony number");
    return v;
  };
  Ntp r; r.offset = number(4) * 1000; r.skew = number(9); r.dispersion = number(11) * 1000;
  bool selected = false;
  std::istringstream lines(sources);
  while (std::getline(lines, s)) if (s.rfind("^*", 0) == 0) selected = true;
  r.qualified = trim(fields[13]) == "Normal" && selected;
  r.pass = r.qualified && std::isfinite(r.offset) && std::isfinite(r.skew) &&
    std::abs(r.offset) <= max_offset && (max_skew <= 0 || r.skew <= max_skew);
  return r;
}
inline Ntp probe_ntp(double max_offset, double max_skew) {
  try {
    auto t = command({"chronyc", "-c", "tracking"});
    auto s = command({"chronyc", "-n", "sources"});
    if (t.status || s.status) throw std::runtime_error("chronyc read failed");
    return parse_ntp(t.output, s.output, max_offset, max_skew);
  } catch (const std::exception &e) { Ntp r; r.error = e.what(); return r; }
}
struct Service { bool active = false; std::string state = "unknown", error; };
inline Service probe_service(const std::string &unit) {
  try {
    auto c = command({"systemctl", "is-active", "--", unit});
    Service r; r.state = trim(c.output); if (r.state.empty()) r.state = "unknown";
    r.active = c.status == 0 && r.state == "active";
    if (c.status != 0 && c.status != 3) r.error = "systemctl probe failed: " + std::to_string(c.status);
    return r;
  } catch (const std::exception &e) { Service r; r.error = e.what(); return r; }
}
struct Cpu { uint64_t total = 0, idle = 0; };
inline Cpu parse_cpu(const std::string &text) {
  std::istringstream in(text); std::string label; in >> label;
  if (label != "cpu") throw std::runtime_error("Missing aggregate CPU row");
  Cpu r;
  for (int i = 0; i < 8; ++i) {
    std::string s; if (!(in >> s) || s.empty() || s[0] == '-') throw std::runtime_error("Invalid CPU row");
    size_t n; const auto v = std::stoull(s, &n);
    if (n != s.size()) throw std::runtime_error("Invalid CPU counter");
    r.total += v; if (i == 3 || i == 4) r.idle += v;
  }
  return r;
}
inline double cpu_load(Cpu before, Cpu after) {
  if (after.total <= before.total || after.idle < before.idle) throw std::runtime_error("CPU counters reset");
  return std::clamp(100.0 * (1.0 - static_cast<double>(after.idle - before.idle) /
    static_cast<double>(after.total - before.total)), 0.0, 100.0);
}
struct Process { int pid; std::string name; uint64_t ticks, start; };
inline Process parse_process(const std::string &s) {
  auto a = s.find('('), b = s.rfind(')');
  if (a == std::string::npos || b <= a) throw std::runtime_error("Invalid process stat");
  Process r{std::stoi(s.substr(0, a)), s.substr(a + 1, b - a - 1), 0, 0};
  std::istringstream in(s.substr(b + 1)); std::string v;
  for (int i = 0; i <= 19; ++i) {
    if (!(in >> v)) throw std::runtime_error("Truncated process stat");
    if (i == 11 || i == 12 || i == 19) {
      if (v[0] == '-') throw std::runtime_error("Negative process counter");
      if (i == 19) r.start = std::stoull(v); else r.ticks += std::stoull(v);
    }
  }
  return r;
}
inline std::map<int, Process> processes() {
  std::map<int, Process> out;
  for (const auto &entry : std::filesystem::directory_iterator("/proc")) {
    const auto name = entry.path().filename().string();
    if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
    try { auto p = parse_process(read_file(entry.path() / "stat")); out.emplace(p.pid, p); }
    catch (const std::exception &) {}  // Process may exit while procfs is read.
  }
  return out;
}
inline uint64_t exact(double x, uint64_t minimum = 0, uint64_t maximum = (1ULL << 52)) {
  if (!std::isfinite(x) || x < minimum || x > maximum || std::floor(x) != x)
    throw std::invalid_argument("Integer field out of range");
  return static_cast<uint64_t>(x);
}
using Bytes = std::vector<uint8_t>;
inline void varint(Bytes &out, uint64_t n) {
  while (n > 127) { out.push_back(static_cast<uint8_t>((n & 127) | 128)); n >>= 7; }
  out.push_back(static_cast<uint8_t>(n));
}
inline void uint_field(Bytes &out, unsigned field, uint64_t n) { varint(out, field << 3); varint(out, n); }
inline void bytes_field(Bytes &out, unsigned field, const Bytes &v) {
  varint(out, (field << 3) | 2); varint(out, v.size()); out.insert(out.end(), v.begin(), v.end());
}
inline Bytes bytes(const std::string &s) { return {s.begin(), s.end()}; }
inline Bytes estop_request(uint64_t ns, const std::string &trace) {
  if (trace.empty()) throw std::invalid_argument("empty trace");
  Bytes stamp, header, out;
  uint_field(stamp, 1, ns / 1000000000); uint_field(stamp, 2, ns % 1000000000);
  uint_field(stamp, 3, ns / 1000000);
  bytes_field(header, 1, stamp); bytes_field(header, 4, bytes(trace));
  bytes_field(header, 5, bytes("hope-foxglove")); bytes_field(out, 1, header);
  // No API or encoding for a false software-emergency bit.
  bytes_field(out, 2, Bytes{0x10, 0x01}); return out;
}
struct Field { uint64_t number, wire, integer = 0; Bytes data; };
inline uint64_t read_varint(const Bytes &data, size_t &i) {
  uint64_t value = 0;
  for (unsigned shift = 0; shift < 70; shift += 7) {
    if (i >= data.size()) throw std::runtime_error("Truncated protobuf varint");
    const uint8_t b = data[i++];
    if (shift == 63 && b > 1) throw std::runtime_error("Protobuf varint overflow");
    value |= static_cast<uint64_t>(b & 127) << shift;
    if (!(b & 128)) return value;
  }
  throw std::runtime_error("Protobuf varint too long");
}
inline std::vector<Field> fields(const Bytes &data) {
  size_t i = 0; std::vector<Field> out;
  while (i < data.size()) {
    const auto key = read_varint(data, i); Field f{key >> 3, key & 7, 0, {}};
    if (!f.number) throw std::runtime_error("Protobuf field zero");
    if (f.wire == 0) f.integer = read_varint(data, i);
    else {
      uint64_t n;
      if (f.wire == 2) n = read_varint(data, i);
      else if (f.wire == 1) n = 8;
      else if (f.wire == 5) n = 4;
      else throw std::runtime_error("Unsupported protobuf wire type");
      if (n > data.size() - i) throw std::runtime_error("Truncated protobuf field");
      f.data.assign(data.begin() + i, data.begin() + i + n); i += n;
    }
    out.push_back(std::move(f));
  }
  return out;
}
inline bool valid_utf8(const Bytes &data) {
  size_t i = 0;
  while (i < data.size()) {
    const auto first = data[i++];
    if (first < 128) continue;
    unsigned count; uint32_t code, minimum;
    if (first >= 0xc2 && first <= 0xdf) { count = 1; code = first & 31; minimum = 128; }
    else if (first >= 0xe0 && first <= 0xef) { count = 2; code = first & 15; minimum = 2048; }
    else if (first >= 0xf0 && first <= 0xf4) { count = 3; code = first & 7; minimum = 65536; }
    else return false;
    if (count > data.size() - i) return false;
    while (count--) { auto next = data[i++]; if ((next & 0xc0) != 0x80) return false; code = (code << 6) | (next & 63); }
    if (code < minimum || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff)) return false;
  }
  return true;
}
inline std::pair<uint64_t, std::string> estop_response(const Bytes &data) {
  std::optional<Bytes> header;
  for (const auto &f : fields(data)) if (f.number == 1) {
    if (f.wire != 2 || header) throw std::runtime_error("Invalid/duplicate response header");
    header = f.data;
  }
  if (!header) throw std::runtime_error("Missing response header");
  uint64_t code = 0; std::string msg; bool seen_code = false, seen_msg = false;
  for (const auto &f : fields(*header)) {
    if (f.number == 1) {
      if (f.wire != 0 || seen_code) throw std::runtime_error("Invalid/duplicate response code");
      code = f.integer; seen_code = true;
    } else if (f.number == 2) {
      if (f.wire != 2 || seen_msg) throw std::runtime_error("Invalid/duplicate response message");
      if (!valid_utf8(f.data)) throw std::runtime_error("Response message is not valid UTF-8");
      msg.assign(f.data.begin(), f.data.end()); seen_msg = true;
    }
  }
  return {code, msg};
}
struct EstopStatus { bool ready, full; std::string text; };
inline EstopStatus estop_status(bool vendor, bool runner, bool latched, bool protocol) {
  if (latched) return {false, false, "E-STOP LATCHED | use Reset Software E-stop to clear the software inhibit"};
  if (vendor && runner) return {true, true, "DUAL-PATH E-STOP READY | VENDOR + RUNNER EMERGENCY PASSIVE"};
  if (vendor) return {true, false, "VENDOR E-STOP READY | RUNNER STOP DEGRADED | PARTIAL ONLY"};
  if (runner) return {true, false, "RUNNER EMERGENCY PASSIVE READY | VENDOR E-STOP UNAVAILABLE | PARTIAL ONLY | USE PHYSICAL E-STOP"};
  return {false, false, protocol ? "E-STOP UNAVAILABLE | no live vendor or Runner emergency backend" : "E-STOP UNAVAILABLE | ros2_plugin_proto is not installed"};
}
inline std::pair<bool, std::string> combine_estop(bool vendor, const std::string &vd,
                                               bool runner, const std::string &rd) {
  if (vendor && runner) return {true, "vendor software E-stop request accepted and managed runner stopped; actuator stop is not independently confirmed, verify the physical E-stop"};
  if (vendor) return {true, "PARTIAL E-STOP: vendor request accepted, but managed runner stop is unconfirmed (" + rd + "); use the physical E-stop"};
  if (runner) return {true, "PARTIAL E-STOP: managed runner stopped, but vendor E-stop request was not accepted (" + vd + "); use the physical E-stop"};
  return {false, "E-STOP UNCONFIRMED: vendor path failed (" + vd + "); runner path failed (" + rd + "); use the physical E-stop"};
}
// Called with assertion/reset serialization, AFTER stop requests are dispatched.
// Never hold the shared health/control state lock across storage operations.
inline void persist_latch(const std::filesystem::path &path, uint64_t ns) {
  if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK, 0600);
  if (fd < 0) throw std::runtime_error("Cannot open E-stop latch: " + std::string(strerror(errno)));
  struct stat info{};
  if (fstat(fd, &info) || !S_ISREG(info.st_mode)) {
    close(fd); throw std::runtime_error("E-stop latch is not a regular file");
  }
  const std::string s = "asserted_utc_ns=" + std::to_string(ns) + "\n";
  size_t offset = 0; bool ok = true;
  while (offset < s.size()) {
    auto n = write(fd, s.data() + offset, s.size() - offset);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) { ok = false; break; } offset += static_cast<size_t>(n);
  }
  if (fsync(fd) != 0) ok = false;
  close(fd);
  if (!ok) throw std::runtime_error("Cannot persist E-stop latch");
  const auto parent = path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path();
  fd = open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) throw std::runtime_error("Cannot open latch directory");
  const bool synced = fsync(fd) == 0; close(fd);
  if (!synced) throw std::runtime_error("Cannot sync latch directory");
}
inline void reset_latch(const std::filesystem::path &path) {
  if (unlink(path.c_str()) != 0 && errno != ENOENT)
    throw std::runtime_error("Software reset failed; latch retained: " + std::string(strerror(errno)));
}
}  // namespace hope_monitor
