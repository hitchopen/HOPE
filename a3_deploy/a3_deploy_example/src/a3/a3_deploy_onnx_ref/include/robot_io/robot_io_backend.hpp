// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace robot_io {

// Snapshot of the robot state synthesised by a backend.
// The layout (ordering of `q`, `dq`, `tau_est`) is defined by
// RobotIOBackend::GetLayout(). A3 uses the 31-DOF MuJoCo/SDK layout at this
// boundary, with the 29-DOF policy view extracted explicitly.
struct RobotState {
  int64_t timestamp_ns = 0;       // Synthesis timestamp (monotonic clock, ns).
  int64_t tick = 0;               // Monotonically increasing tick (drop detection).

  // Local timing metadata for deploy latency accounting. `timestamp_ns` is the
  // logical state time. `state_data_ready_ns` is the latest local receive time
  // among raw samples needed to synthesize this state. `state_sync_ready_ns` is
  // when the backend finished assembling this RobotState.
  int64_t state_data_ready_ns = 0;
  int64_t state_sync_ready_ns = 0;

  // True when the backend assembled this frame from a complete, time-aligned
  // sensor set. Backends without an explicit synchronizer leave these true.
  bool sync_complete = true;
  bool sync_aligned = true;
  int64_t sync_skew_ns = 0;

  Eigen::VectorXd q;              // Joint positions.
  Eigen::VectorXd dq;             // Joint velocities.
  Eigen::VectorXd tau_est;        // Measured / estimated joint torques.

  // Primary IMU (A3 pelvis IMU embedded in LowState, A3 /pelvis_imu/data).
  Eigen::Vector4d imu_quat_wxyz = Eigen::Vector4d(1, 0, 0, 0);
  Eigen::Vector3d imu_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_accel = Eigen::Vector3d(0, 0, 9.81);

  // Secondary IMU (A3 secondary_imu, A3 torso_imu).
  // `has_secondary_imu` is false until the backend has received at least one
  // message on the secondary channel.
  bool has_secondary_imu = false;
  Eigen::Vector4d sec_imu_quat_wxyz = Eigen::Vector4d(1, 0, 0, 0);
  Eigen::Vector3d sec_imu_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d sec_imu_accel = Eigen::Vector3d(0, 0, 9.81);
};

// Command packet sent to a backend. All vectors must have
// size == GetLayout().dof(); the backend is responsible for packing them
// into the underlying wire format.
struct RobotCommand {
  Eigen::VectorXd q_des;
  Eigen::VectorXd dq_des;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

struct JointLayout {
  std::vector<std::string> names;
  int dof() const { return static_cast<int>(names.size()); }
};

class RobotIOBackend {
 public:
  virtual ~RobotIOBackend() = default;

  // Initialise transport (DDS domain + iface for A3, AimRT runtime for A3).
  // `config` is a backend-specific key=value string, e.g. "domain=0,iface=eth0".
  virtual bool Init(const std::string& config) = 0;

  // Start I/O: open publishers/subscribers and begin delivering state
  // callbacks. After Start(), SendCommand() is expected to be safe to call.
  virtual bool Start() = 0;

  // Stop I/O: tear down publishers/subscribers; further SendCommand() calls
  // are no-ops.
  virtual void Stop() = 0;

  virtual const JointLayout& GetLayout() const = 0;

  using StateCallback = std::function<void(const RobotState&)>;
  virtual void RegisterStateCallback(StateCallback cb) = 0;

  virtual bool SendCommand(const RobotCommand& cmd) = 0;

  virtual std::string Name() const = 0;
  virtual double StateRateHz() const = 0;
};

// Factory. Returns nullptr if `name` is unknown or the requested backend
// was not compiled in (e.g. A3 when ENABLE_A3_BACKEND=OFF).
std::unique_ptr<RobotIOBackend> CreateBackend(const std::string& name);

}  // namespace robot_io
