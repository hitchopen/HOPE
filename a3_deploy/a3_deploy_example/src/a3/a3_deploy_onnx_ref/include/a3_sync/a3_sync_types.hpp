// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#pragma once

// A3 6-channel sync primitives.
// See notes/a3_backend_plan.md §8 / PR 3.
//
// Pure C++ types shared between the ring buffers, the sync loop, and the
// interpolation / assembly helpers. Zero AimRT dependency — AimRT glue lives
// in PR 5.

#include <array>
#include <cstdint>

namespace a3_sync {

struct TimestampNs {
  std::int64_t value{0};
  friend constexpr auto operator<=>(const TimestampNs&, const TimestampNs&) = default;
};

enum class SyncClockSource : std::uint8_t {
  Fixed = 0,
  Imu   = 1,  // Reserved; not implemented in PR 3 (deferred).
};

enum class SyncMode : std::uint8_t {
  HeaderInterp = 0,
  MinSkewPair  = 1,
  LatestFrame  = 2,
};

// Per-topic joint samples. DOF is hard-coded per topic to match A3 topology:
//   waist = 3, leg = 12, arm = 14, neck = 2.
// `stamp` is the producer/header timestamp used for cross-channel skew.
// `recv_stamp` is local wall-clock receipt time used only for freshness.
struct WaistSample {
  TimestampNs stamp{};
  TimestampNs recv_stamp{};
  bool source_stamp_valid{false};
  bool source_sequence_valid{false};
  std::uint32_t source_sequence{0};
  std::array<double, 3> pos{};
  std::array<double, 3> vel{};
  std::array<double, 3> eff{};
};

struct LegSample {
  TimestampNs stamp{};
  TimestampNs recv_stamp{};
  bool source_stamp_valid{false};
  bool source_sequence_valid{false};
  std::uint32_t source_sequence{0};
  std::array<double, 12> pos{};
  std::array<double, 12> vel{};
  std::array<double, 12> eff{};
};

struct ArmSample {
  TimestampNs stamp{};
  TimestampNs recv_stamp{};
  bool source_stamp_valid{false};
  bool source_sequence_valid{false};
  std::uint32_t source_sequence{0};
  std::array<double, 14> pos{};
  std::array<double, 14> vel{};
  std::array<double, 14> eff{};
};

struct NeckSample {
  TimestampNs stamp{};
  TimestampNs recv_stamp{};
  bool source_stamp_valid{false};
  bool source_sequence_valid{false};
  std::uint32_t source_sequence{0};
  std::array<double, 2> pos{};
  std::array<double, 2> vel{};
  std::array<double, 2> eff{};
};

struct ImuSample {
  TimestampNs stamp{};
  TimestampNs recv_stamp{};
  bool source_stamp_valid{false};
  std::array<double, 4> quat_wxyz{1.0, 0.0, 0.0, 0.0};  // WXYZ — already canonicalised.
  std::array<double, 3> gyro{};
  std::array<double, 3> acc{0.0, 0.0, 9.81};
};

struct SyncConfig {
  double sync_hz{100.0};
  SyncMode sync_mode{SyncMode::MinSkewPair};
  std::int64_t max_skew_ns{3'000'000};        // 3 ms
  // Max age of a source-channel sample relative to the sync tick.
  // <=0 disables freshness checking.
  std::int64_t max_sample_age_ns{50'000'000}; // 50 ms
  int max_backtrack{200};
  SyncClockSource clock_source{SyncClockSource::Fixed};
  std::int64_t align_delay_ns{2'000'000};  // 2 ms
  std::int64_t phase_ns{1'500'000};             // 1.5 ms
  std::int64_t sync_ready_after_input_ns{200'000};   // 0.2 ms
  std::int64_t max_group_internal_skew_ns{250'000};  // 0.25 ms
  std::int64_t max_group_pair_skew_ns{1'000'000};    // 1 ms
  int group_pair_search_depth{8};
};

}  // namespace a3_sync
