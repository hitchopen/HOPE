#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace hope_planner_cpp {
inline std::string default_local_control_key_path() {
  const char* ros_home=std::getenv("ROS_HOME");
  const char* home=std::getenv("HOME");
  if(ros_home && *ros_home)return std::string(ros_home)+"/hope-ball-controls.key";
  if(home && *home)return std::string(home)+"/.ros/hope-ball-controls.key";
  return {}; // No credential means Desktop requests are refused.
}

// The key never enters ROS parameters, HTTP responses, logs, layouts or Git.
// Only its file PATH is configurable. Persist across ordinary runtime restarts.
inline std::string load_local_control_key(const std::string& path) {
  namespace fs=std::filesystem;
  if(path.empty())return {};
  if(!fs::path(path).is_absolute())throw std::runtime_error("local control key path must be absolute");
  fs::create_directories(fs::path(path).parent_path());
  if(!fs::exists(path)) {
    std::array<unsigned char,32> random{};
    int fd=::open("/dev/urandom",O_RDONLY|O_CLOEXEC);
    if(fd<0)throw std::runtime_error("local control key randomness unavailable");
    const auto n=::read(fd,random.data(),random.size());::close(fd);
    if(n!=static_cast<ssize_t>(random.size()))throw std::runtime_error("local control key randomness incomplete");
    std::string key;const char* hex="0123456789abcdef";
    for(auto c:random){key+=hex[c>>4];key+=hex[c&15];}
    std::vector<char> temporary(path.begin(),path.end());
    const std::string suffix=".XXXXXX";temporary.insert(temporary.end(),suffix.begin(),suffix.end());temporary.push_back(0);
    fd=::mkstemp(temporary.data());
    if(fd<0)throw std::runtime_error("cannot create private local control key");
    const bool written=::write(fd,key.data(),key.size())==static_cast<ssize_t>(key.size()) && ::fsync(fd)==0;
    ::close(fd);
    // Atomic no-clobber publication: another startup may already own the key.
    const bool published=written && (::link(temporary.data(),path.c_str())==0 || errno==EEXIST);
    ::unlink(temporary.data());
    if(!published)throw std::runtime_error("cannot persist private local control key");
  }
  const int fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);
  if(fd<0)throw std::runtime_error("cannot open private local control key");
  struct stat st{};
  const bool private_file=::fstat(fd,&st)==0 && S_ISREG(st.st_mode) && st.st_uid==::geteuid() &&
      (st.st_mode & 0777)==0600 && st.st_size==64;
  std::array<char,64> value{};
  const auto n=private_file?::read(fd,value.data(),value.size()):-1;::close(fd);
  if(n!=64)throw std::runtime_error("local control key must be an owned, regular, mode-0600 file of 64 hex characters");
  for(char c:value)if(!((c>='0' && c<='9') || (c>='a' && c<='f')))
    throw std::runtime_error("invalid local control key format");
  return {value.data(),value.size()};
}
} // namespace hope_planner_cpp
