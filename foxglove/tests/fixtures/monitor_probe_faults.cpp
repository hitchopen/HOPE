// Test-only LD_PRELOAD fault injection. Never installed or linked into runtime.
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

static bool fail_cpu(const char *path) {
  return std::getenv("HOPE_TEST_FAIL_PROC_STAT") && std::strcmp(path, "/proc/stat") == 0;
}
extern "C" FILE *fopen(const char *path, const char *mode) {
  if (fail_cpu(path)) { errno = EIO; return nullptr; }
  static const auto original = reinterpret_cast<FILE *(*)(const char *, const char *)>(dlsym(RTLD_NEXT, "fopen"));
  return original(path, mode);
}
extern "C" FILE *fopen64(const char *path, const char *mode) {
  if (fail_cpu(path)) { errno = EIO; return nullptr; }
  static const auto original = reinterpret_cast<FILE *(*)(const char *, const char *)>(dlsym(RTLD_NEXT, "fopen64"));
  return original(path, mode);
}
extern "C" int fsync(int fd) {
  const auto *directory = std::getenv("HOPE_TEST_SLOW_SYNC_DIR");
  if (directory) {
    char path[4096];
    const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), path, sizeof(path) - 1);
    if (n > 0) {
      path[n] = 0;
      const std::string prefix(directory), target(path);
      if (target == prefix || target.rfind(prefix + '/', 0) == 0) {
        const auto marker = prefix + "/sync_entered";
        const int receipt = open(marker.c_str(), O_CREAT | O_WRONLY | O_CLOEXEC, 0600);
        if (receipt >= 0) close(receipt);
        usleep(800000);
      }
    }
  }
  static const auto original = reinterpret_cast<int (*)(int)>(dlsym(RTLD_NEXT, "fsync"));
  return original(fd);
}
extern "C" int unlink(const char *path) {
  const auto *delayed = std::getenv("HOPE_TEST_SLOW_UNLINK_PATH");
  const auto *release = std::getenv("HOPE_TEST_UNLINK_RELEASE_FILE");
  if (delayed && release && std::strcmp(path, delayed) == 0) {
    const std::string marker = std::string(path) + ".unlink_entered";
    const int receipt = open(marker.c_str(), O_CREAT | O_WRONLY | O_CLOEXEC, 0600);
    if (receipt >= 0) close(receipt);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (access(release, F_OK) != 0 && std::chrono::steady_clock::now() < deadline) usleep(1000);
  }
  static const auto original = reinterpret_cast<int (*)(const char *)>(dlsym(RTLD_NEXT, "unlink"));
  return original(path);
}
