// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "robot_io/robot_io_backend.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace robot_io {

// Forward-declared factory helper. The implementation is defined in the
// backend translation unit; the weak fallback below returns nullptr when the
// backend is not compiled in for this build.
std::unique_ptr<RobotIOBackend> CreateA3AimrtBackend();

std::unique_ptr<RobotIOBackend> CreateBackend(const std::string& name) {
  if (name == "a3") {
    auto backend = CreateA3AimrtBackend();
    if (!backend) {
      std::cerr << "[robot_io] A3 backend requested but not compiled in "
                   "(build with -DENABLE_A3_BACKEND=ON)."
                << std::endl;
    }
    return backend;
  }
  std::cerr << "[robot_io] CreateBackend: unknown backend name '" << name
            << "'. Known names: 'a3'." << std::endl;
  return nullptr;
}

__attribute__((weak)) std::unique_ptr<RobotIOBackend> CreateA3AimrtBackend() {
  return nullptr;
}

}  // namespace robot_io
