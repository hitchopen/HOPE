#pragma once

#include <bit>
#include <cstdint>

namespace a3_deploy::numeric_safety {

// Do not implement these checks with std::isfinite. This header is used at
// process and motor-command trust boundaries, where the predicate must keep
// working even if a downstream build accidentally enables finite-math-only.
inline bool IsFinite(double value) noexcept {
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
  return (bits & 0x7ff0000000000000ULL) != 0x7ff0000000000000ULL;
}

inline bool IsFinite(float value) noexcept {
  const std::uint32_t bits = std::bit_cast<std::uint32_t>(value);
  return (bits & 0x7f800000U) != 0x7f800000U;
}

}  // namespace a3_deploy::numeric_safety
