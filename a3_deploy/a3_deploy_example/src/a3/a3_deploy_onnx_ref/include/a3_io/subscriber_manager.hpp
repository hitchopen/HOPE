// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
//
// A3SubscriberManager owns the six concrete subscribers and provides a
// WaitAllReady() launch barrier used at startup. The ring buffers are
// owned externally (typically by A3SyncLoop / A3AimrtBackend) and passed
// in as non-owning pointers. AimRT registration (creating real channel
// handles, hooking OnMessage as the AimRT callback) is deferred to PR 7.

#pragma once

#include "a3_io/subscribers/arm_state_sub.hpp"
#include "a3_io/subscribers/leg_state_sub.hpp"
#include "a3_io/subscribers/neck_state_sub.hpp"
#include "a3_io/subscribers/pelvis_imu_sub.hpp"
#include "a3_io/subscribers/torso_imu_sub.hpp"
#include "a3_io/subscribers/waist_state_sub.hpp"
#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_types.hpp"

#include <functional>

namespace a3_io {

class A3SubscriberManager {
 public:
  struct Options {
    a3_sync::RingBuffer<a3_sync::WaistSample>* waist_ring{nullptr};
    a3_sync::RingBuffer<a3_sync::LegSample>*   leg_ring{nullptr};
    a3_sync::RingBuffer<a3_sync::ArmSample>*   arm_ring{nullptr};
    a3_sync::RingBuffer<a3_sync::NeckSample>*  neck_ring{nullptr};
    a3_sync::RingBuffer<a3_sync::ImuSample>*   pelvis_imu_ring{nullptr};
    a3_sync::RingBuffer<a3_sync::ImuSample>*   torso_imu_ring{nullptr};

    WaistStateSub::SampleCallback waist_sample_cb{};
    LegStateSub::SampleCallback leg_sample_cb{};
    ArmStateSub::SampleCallback arm_sample_cb{};
    NeckStateSub::SampleCallback neck_sample_cb{};
    PelvisImuSub::SampleCallback pelvis_imu_sample_cb{};
    TorsoImuSub::SampleCallback torso_imu_sample_cb{};
  };

  explicit A3SubscriberManager(const Options& opt);

  WaistStateSub& waist() { return waist_; }
  LegStateSub&   leg()   { return leg_; }
  ArmStateSub&   arm()   { return arm_; }
  NeckStateSub&  neck()  { return neck_; }
  PelvisImuSub&  pelvis_imu() { return pelvis_imu_; }
  TorsoImuSub&   torso_imu()  { return torso_imu_; }

  // Blocks up to `timeout_seconds`, polling IsReady() on all six subs
  // every 10ms. Returns true when all are ready, false on timeout.
  bool WaitAllReady(double timeout_seconds);

  // True iff all 6 subs report IsReady() right now.
  bool AllReady() const noexcept;

 private:
  WaistStateSub waist_;
  LegStateSub   leg_;
  ArmStateSub   arm_;
  NeckStateSub  neck_;
  PelvisImuSub  pelvis_imu_;
  TorsoImuSub   torso_imu_;
};

}  // namespace a3_io
