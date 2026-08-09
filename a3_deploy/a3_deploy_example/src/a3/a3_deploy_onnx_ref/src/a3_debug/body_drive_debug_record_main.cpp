// Copyright (c) 2026, AgiBot Inc. All rights reserved.

#include "core/aimrt_core.h"

#include <csignal>
#include <future>
#include <iostream>
#include <string>
#include <string_view>

namespace {

aimrt::runtime::core::AimRTCore* g_core = nullptr;

void SignalHandler(int sig) {
  if ((sig == SIGINT || sig == SIGTERM) && g_core != nullptr) {
    g_core->Shutdown();
    return;
  }
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

std::string ParseCfgPath(int argc, char** argv) {
  constexpr std::string_view kPrefix = "--cfg_file_path=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: a3_body_drive_debug_record --cfg_file_path <yaml>\n"
          << "   or: a3_body_drive_debug_record --cfg_file_path=<yaml>\n";
      std::exit(0);
    }
    if (arg == "--cfg_file_path" || arg == "--cfg") {
      if (i + 1 >= argc) return {};
      return argv[++i];
    }
    if (arg.rfind(kPrefix.data(), 0) == 0) {
      return arg.substr(kPrefix.size());
    }
    if (!arg.empty() && arg[0] != '-') {
      return arg;
    }
  }
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  const std::string cfg_path = ParseCfgPath(argc, argv);
  if (cfg_path.empty()) {
    std::cerr << "missing AimRT cfg path; pass --cfg_file_path <yaml>\n";
    return 64;
  }

  try {
    aimrt::runtime::core::AimRTCore core;
    g_core = &core;

    aimrt::runtime::core::AimRTCore::Options options;
    options.cfg_file_path = cfg_path;

    std::cout << "[a3_body_drive_debug_record] cfg=" << cfg_path << "\n";
    core.Initialize(options);

    std::future<void> shutdown_future = core.AsyncStart();
    std::cout << "[a3_body_drive_debug_record] recording; press Ctrl+C to stop\n";
    shutdown_future.get();

    g_core = nullptr;
    std::cout << "[a3_body_drive_debug_record] stopped\n";
    return 0;
  } catch (const std::exception& e) {
    g_core = nullptr;
    std::cerr << "[a3_body_drive_debug_record] error: " << e.what() << "\n";
    return 1;
  }
}
