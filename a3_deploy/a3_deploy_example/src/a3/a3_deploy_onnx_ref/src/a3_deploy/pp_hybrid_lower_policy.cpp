#include "a3_pingpong/pp_hybrid_lower_policy.hpp"

#include <onnxruntime_cxx_api.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace a3_pingpong {
namespace {

constexpr int kFrameDim = 55;
constexpr int kHistoryFrames = 66;
constexpr int kInputDim = kFrameDim * kHistoryFrames;
constexpr int kActionDim = 15;
constexpr double kTwoPi = 6.283185307179586476925286766559;

// HumanLike action/observation joint order is L leg, R leg, waist.
constexpr std::array<int, kActionDim> kHumanLikeToSdk = {
    19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30,
    0, 1, 2,
};
constexpr std::array<double, kActionDim> kDefaultQ = {
    -0.1311, 0.0056, -0.0348, 0.2468, -0.1204, -0.0078,
    -0.1311, -0.0056, 0.0348, 0.2468, -0.1204, 0.0078,
    0.0, 0.0, 0.0,
};
constexpr std::array<double, kActionDim> kActionScale = {
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
};
constexpr std::array<double, kActionDim> kKp = {
    200.0, 200.0, 200.0, 360.0, 80.0, 80.0,
    200.0, 200.0, 200.0, 360.0, 80.0, 80.0,
    200.0, 80.0, 80.0,
};
constexpr std::array<double, kActionDim> kKd = {
    7.0, 7.0, 7.0, 13.0, 3.0, 3.0,
    7.0, 7.0, 7.0, 13.0, 3.0, 3.0,
    7.0, 3.0, 3.0,
};
constexpr std::array<double, 3> kCommandFilterAlpha = {
    0.002, 0.002, 0.003};
constexpr std::array<double, 3> kCommandLowerLimit = {
    -0.3, -0.3, -1.0};
constexpr std::array<double, 3> kCommandUpperLimit = {
    0.8, 0.3, 1.0};
constexpr int kHumanLikeControlTicksPerActor = 5;

double FootEndXInPelvis(const robot_io::RobotState& state,
                        bool left) noexcept {
  const int q0 = left ? 19 : 25;
  Eigen::Vector3d position(
      left ? 0.0 : 0.0, left ? 0.12298 : -0.122983,
      left ? -0.17875 : -0.178753);
  Eigen::Matrix3d rotation =
      Eigen::Quaterniond(0.991445, left ? -0.130526 : 0.130526, 0.0, 0.0)
          .normalized()
          .toRotationMatrix();
  rotation *= Eigen::AngleAxisd(state.q[q0], Eigen::Vector3d::UnitY())
                  .toRotationMatrix();

  const Eigen::Vector3d hip_roll_offset(
      left ? 0.0 : -0.00109999, left ? 0.0113163 : -0.0113163,
      -0.042233);
  position += rotation * hip_roll_offset;
  rotation *=
      Eigen::Quaterniond(0.991445, left ? 0.130526 : -0.130526, 0.0, 0.0)
          .normalized()
          .toRotationMatrix();
  rotation *= Eigen::AngleAxisd(state.q[q0 + 1], Eigen::Vector3d::UnitX())
                  .toRotationMatrix();
  rotation *= Eigen::AngleAxisd(state.q[q0 + 2], Eigen::Vector3d::UnitZ())
                  .toRotationMatrix();

  position += rotation * Eigen::Vector3d(0.0, 0.0, -0.37);
  rotation *= Eigen::AngleAxisd(state.q[q0 + 3], Eigen::Vector3d::UnitY())
                  .toRotationMatrix();
  position += rotation * Eigen::Vector3d(
                             0.0, left ? 0.0015 : -0.0015, -0.415);
  rotation *= Eigen::AngleAxisd(state.q[q0 + 4], Eigen::Vector3d::UnitY())
                  .toRotationMatrix();
  rotation *= Eigen::AngleAxisd(state.q[q0 + 5], Eigen::Vector3d::UnitX())
                  .toRotationMatrix();
  position += rotation * Eigen::Vector3d(0.04, 0.0, -0.067);
  return position.x();
}

void AtomicMax(std::atomic<std::uint64_t>& dst, std::uint64_t value) noexcept {
  std::uint64_t old = dst.load(std::memory_order_relaxed);
  while (old < value &&
         !dst.compare_exchange_weak(old, value, std::memory_order_relaxed)) {
  }
}

std::size_t ElementCount(std::vector<std::int64_t> shape) {
  std::size_t count = 1;
  for (auto& dim : shape) {
    if (dim <= 0) dim = 1;
    count *= static_cast<std::size_t>(dim);
  }
  return count;
}

}  // namespace

class PpHumanLikeOrtRuntime {
 public:
  bool Initialize(const std::string& model_path) {
    try {
      Ort::SessionOptions options;
      options.SetIntraOpNumThreads(1);
      options.SetInterOpNumThreads(1);
      options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      options.AddConfigEntry("session.intra_op.allow_spinning", "0");
      options.AddConfigEntry("session.inter_op.allow_spinning", "0");
      session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);

      if (session_->GetInputCount() != 1) {
        std::cerr << "[hybrid-lower] HumanLike expected one input, got "
                  << session_->GetInputCount() << "\n";
        return false;
      }
      auto input_name = session_->GetInputNameAllocated(0, allocator_);
      input_name_ = input_name.get();
      const auto input_type_info = session_->GetInputTypeInfo(0);
      const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
      input_shape_ = input_info.GetShape();
      if (input_name_ != "input" ||
          input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
          ElementCount(input_shape_) != kInputDim) {
        std::cerr << "[hybrid-lower] HumanLike input contract mismatch: name="
                  << input_name_ << " elements=" << ElementCount(input_shape_)
                  << " (expected input/" << kInputDim << ")\n";
        return false;
      }

      int action_output_index = -1;
      for (std::size_t i = 0; i < session_->GetOutputCount(); ++i) {
        auto output_name = session_->GetOutputNameAllocated(i, allocator_);
        if (std::string(output_name.get()) == "output") {
          action_output_index = static_cast<int>(i);
          output_name_ = output_name.get();
          const auto output_type_info = session_->GetOutputTypeInfo(i);
          const auto output_info =
              output_type_info.GetTensorTypeAndShapeInfo();
          output_shape_ = output_info.GetShape();
          if (output_info.GetElementType() !=
                  ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
              ElementCount(output_shape_) != kActionDim) {
            std::cerr << "[hybrid-lower] HumanLike action shape mismatch\n";
            return false;
          }
          break;
        }
      }
      if (action_output_index < 0) {
        std::cerr << "[hybrid-lower] HumanLike output tensor not found\n";
        return false;
      }

      // The exported actor uses a dynamic batch dimension. Deployment is
      // always batch=1; Ort::CreateTensor requires that concrete shape.
      for (auto& dim : input_shape_) {
        if (dim <= 0) dim = 1;
      }
      for (auto& dim : output_shape_) {
        if (dim <= 0) dim = 1;
      }

      memory_info_ = std::make_unique<Ort::MemoryInfo>(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
      input_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, input_.data(), input_.size(), input_shape_.data(),
          input_shape_.size());
      output_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, action_.data(), action_.size(), output_shape_.data(),
          output_shape_.size());
      return true;
    } catch (const Ort::Exception& e) {
      std::cerr << "[hybrid-lower] HumanLike ORT init failed: " << e.what()
                << "\n";
      session_.reset();
      return false;
    } catch (const std::exception& e) {
      std::cerr << "[hybrid-lower] HumanLike init failed: " << e.what() << "\n";
      session_.reset();
      return false;
    }
  }

  bool Infer(const std::array<float, kInputDim>& input) noexcept {
    if (!session_) return false;
    std::copy(input.begin(), input.end(), input_.begin());
    try {
      const char* input_names[] = {input_name_.c_str()};
      const char* output_names[] = {output_name_.c_str()};
      Ort::RunOptions run_options;
      session_->Run(run_options, input_names, &input_tensor_, 1, output_names,
                    &output_tensor_, 1);
      return std::all_of(action_.begin(), action_.end(),
                         [](float value) { return std::isfinite(value); });
    } catch (const Ort::Exception& e) {
      std::cerr << "[hybrid-lower] HumanLike inference failed: " << e.what()
                << "\n";
      return false;
    } catch (const std::exception& e) {
      std::cerr << "[hybrid-lower] HumanLike inference failed: " << e.what()
                << "\n";
      return false;
    }
  }

  const std::array<float, kActionDim>& action() const noexcept {
    return action_;
  }

 private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "pp_humanlike"};
  Ort::AllocatorWithDefaultOptions allocator_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::string input_name_;
  std::string output_name_;
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;
  std::array<float, kInputDim> input_{};
  std::array<float, kActionDim> action_{};
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
};

const char* PpHybridLowerOwnerName(PpHybridLowerOwner owner) noexcept {
  switch (owner) {
    case PpHybridLowerOwner::kLegs:
      return "legs";
    case PpHybridLowerOwner::kWaistRollAndLegs:
      return "roll_legs";
    case PpHybridLowerOwner::kWaistRollPitchAndLegs:
      return "roll_pitch_legs";
    case PpHybridLowerOwner::kWaistAndLegs:
      return "waist_legs";
  }
  return "unknown";
}

PpHybridLowerPolicy::PpHybridLowerPolicy(PpHybridLowerConfig config)
    : config_(std::move(config)) {}

PpHybridLowerPolicy::~PpHybridLowerPolicy() = default;

bool PpHybridLowerPolicy::Initialize() {
  if (config_.backend != "ort_cpu" && config_.backend != "ort" &&
      config_.backend != "cpu") {
    std::cerr << "[hybrid-lower] HumanLike x86 test supports ort_cpu only\n";
    return false;
  }
  runtime_ = std::make_unique<PpHumanLikeOrtRuntime>();
  if (!runtime_->Initialize(config_.model_path)) return false;
  if (!(config_.outer_policy_dt > 0.0) ||
      !(config_.humanlike_policy_dt > 0.0)) {
    std::cerr << "[hybrid-lower] invalid policy timing\n";
    return false;
  }
  substeps_per_outer_tick_ = std::max(
      1, static_cast<int>(std::lround(config_.outer_policy_dt /
                                      config_.humanlike_policy_dt)));

  // Allocate kernels once, then reset every temporal input before control.
  std::array<float, kInputDim> zero_input{};
  if (!runtime_->Infer(zero_input)) {
    std::cerr << "[hybrid-lower] HumanLike warmup failed\n";
    return false;
  }
  Reset();
  std::cout << "[hybrid-lower] READY: receive policy + HumanLike3630x15 "
            << PpHybridLowerOwnerName(config_.owner)
            << " | outer=" << (1.0 / config_.outer_policy_dt)
            << "Hz history=" << (1.0 / config_.humanlike_policy_dt)
            << "Hz actor_eval=" << (1.0 / config_.outer_policy_dt)
            << "Hz held_frames=" << substeps_per_outer_tick_
            << " cmd={" << config_.command_velocity_xyz[0] << ","
            << config_.command_velocity_xyz[1] << ","
            << config_.command_velocity_xyz[2] << "} phase="
            << (config_.smart_walk_zero_settled ? "SMART_WALK" : "WALK")
            << "\n";
  return true;
}

void PpHybridLowerPolicy::Reset() noexcept {
  current_obs_.fill(0.0f);
  history_.fill(0.0f);
  raw_action_.fill(0.0f);
  target_command_velocity_xyz_ = config_.command_velocity_xyz;
  filtered_command_velocity_xyz_.fill(0.0);
  needs_entry_seed_ = true;
  phase_index_ = static_cast<int>(std::lround(
      100.0 * std::fmod(config_.phase_offset + 0.5 + 1.0, 1.0)));
  low_speed_count_ = 41;
  phase_active_ = true;
}

void PpHybridLowerPolicy::SetCommandVelocityTarget(
    const std::array<double, 3>& command_velocity_xyz) noexcept {
  for (int i = 0; i < 3; ++i) {
    const double value = std::isfinite(command_velocity_xyz[i])
                             ? command_velocity_xyz[i]
                             : 0.0;
    target_command_velocity_xyz_[i] =
        std::clamp(value, kCommandLowerLimit[i], kCommandUpperLimit[i]);
  }
}

void PpHybridLowerPolicy::UpdateFilteredCommand() noexcept {
  // HumanLike's outer loop is 500 Hz and the actor runs every five ticks.
  // Replaying all five EMA updates here preserves the exact actor-visible
  // command sequence while the shared Runner publishes one 100 Hz command.
  for (int tick = 0; tick < kHumanLikeControlTicksPerActor; ++tick) {
    for (int i = 0; i < 3; ++i) {
      filtered_command_velocity_xyz_[i] +=
          kCommandFilterAlpha[i] *
          (target_command_velocity_xyz_[i] -
           filtered_command_velocity_xyz_[i]);
    }
  }
}

void PpHybridLowerPolicy::SeedEntryState(
    const robot_io::RobotState& state) noexcept {
  // AgiBot's HumanLike onEnter reconstructs the equivalent preceding action
  // from the measured PD state. It seeds actions/previous_action, while the
  // already-cleared 66-frame history remains zero until the first observation
  // is written into its final frame below in Apply().
  const bool has_tau = state.tau_est.size() == 31;
  for (int i = 0; i < kActionDim; ++i) {
    const int sdk = kHumanLikeToSdk[i];
    const double tau =
        has_tau && std::isfinite(state.tau_est[sdk]) ? state.tau_est[sdk] : 0.0;
    const double q_equiv =
        state.q[sdk] + (tau + kKd[i] * state.dq[sdk]) / kKp[i];
    raw_action_[i] =
        static_cast<float>((q_equiv - kDefaultQ[i]) / kActionScale[i]);
  }
  needs_entry_seed_ = false;
}

double PpHybridLowerPolicy::UpdatePhase(const robot_io::RobotState& state,
                                        double waist_roll,
                                        double waist_pitch) noexcept {
  double phase = std::fmod(static_cast<double>(phase_index_) *
                               config_.humanlike_policy_dt,
                           1.0);
  const double command_norm = std::sqrt(
      filtered_command_velocity_xyz_[0] * filtered_command_velocity_xyz_[0] +
      filtered_command_velocity_xyz_[1] * filtered_command_velocity_xyz_[1] +
      filtered_command_velocity_xyz_[2] * filtered_command_velocity_xyz_[2]);

  // Exact HumanLike SMART_WALK state machine. onEnter seeds phase_index=96
  // and low_speed_count=41. Therefore a level, feet-aligned zero-command
  // entry resets the very first actor observation to phase 0; it does not
  // execute a gratuitous gait cycle first. receive policy is an external upper
  // command, selecting the vendor swing thresholds below.
  if (!config_.smart_walk_zero_settled || command_norm >= 0.05) {
    low_speed_count_ = 0;
    ++phase_index_;
    if (!phase_active_) {
      phase_index_ = static_cast<int>(100.0 * config_.phase_offset);
      phase_active_ = true;
    }
    return phase;
  }

  ++low_speed_count_;
  if (low_speed_count_ <= 40 || phase < 0.95) {
    ++phase_index_;
    if (!phase_active_) {
      phase_index_ = static_cast<int>(100.0 * config_.phase_offset);
      phase_active_ = true;
    }
    return phase;
  }
  const double foot_delta_x =
      std::abs(FootEndXInPelvis(state, true) -
               FootEndXInPelvis(state, false));
  constexpr double kExternalUpperRollLimit = 0.10;
  constexpr double kExternalUpperPitchLimit = 0.14;
  constexpr double kExternalUpperPitchOffset = 0.06;
  constexpr double kExternalUpperFootDeltaXLimit = 0.15;
  if (foot_delta_x >= kExternalUpperFootDeltaXLimit ||
      std::abs(waist_roll) >= kExternalUpperRollLimit ||
      std::abs(waist_pitch + kExternalUpperPitchOffset) >=
          kExternalUpperPitchLimit) {
    ++phase_index_;
    if (!phase_active_) {
      phase_index_ = static_cast<int>(100.0 * config_.phase_offset);
      phase_active_ = true;
    }
    return phase;
  }
  phase_active_ = false;
  return 0.0;
}

void PpHybridLowerPolicy::BuildCurrentObservation(
    const robot_io::RobotState& state) noexcept {
  Eigen::Vector4d quat = state.imu_quat_wxyz;
  if (!quat.allFinite() || quat.norm() < 1.0e-9) {
    quat = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  } else {
    quat.normalize();
  }
  const double w = quat[0];
  const double x = quat[1];
  const double y = quat[2];
  const double z = quat[3];
  const double roll =
      std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sin_pitch);
  const double phase = UpdatePhase(state, roll, pitch);
  current_obs_[0] = static_cast<float>(std::sin(kTwoPi * phase));
  current_obs_[1] = static_cast<float>(std::cos(kTwoPi * phase));
  for (int i = 0; i < 3; ++i) {
    current_obs_[2 + i] =
        static_cast<float>(2.0 * filtered_command_velocity_xyz_[i]);
  }
  for (int i = 0; i < kActionDim; ++i) {
    const int sdk = kHumanLikeToSdk[i];
    current_obs_[5 + i] =
        static_cast<float>(state.q[sdk] - kDefaultQ[i]);
    current_obs_[20 + i] = static_cast<float>(0.05 * state.dq[sdk]);
    current_obs_[35 + i] = raw_action_[i];
  }
  for (int i = 0; i < 3; ++i) {
    current_obs_[50 + i] = static_cast<float>(2.0 * state.imu_gyro[i]);
  }

  current_obs_[53] = static_cast<float>(roll);
  current_obs_[54] = static_cast<float>(pitch);
  for (float& value : current_obs_) {
    value = std::clamp(value, -100.0f, 100.0f);
  }
}

void PpHybridLowerPolicy::PushOneHumanLikeHistoryFrame(
    const robot_io::RobotState& state) noexcept {
  BuildCurrentObservation(state);
  std::memmove(history_.data(), history_.data() + kFrameDim,
               sizeof(float) * (kInputDim - kFrameDim));
  std::copy(current_obs_.begin(), current_obs_.end(),
            history_.end() - kFrameDim);
}

bool PpHybridLowerPolicy::OwnsActionIndex(int action_index,
                                         bool legs_enabled,
                                         bool waist_enabled) const noexcept {
  if (action_index < 12) return legs_enabled;
  if (!waist_enabled) return false;
  if (config_.owner == PpHybridLowerOwner::kWaistAndLegs) return true;
  if (config_.owner == PpHybridLowerOwner::kWaistRollAndLegs)
    return action_index == 13;
  return config_.owner == PpHybridLowerOwner::kWaistRollPitchAndLegs &&
         (action_index == 13 || action_index == 14);
}

void PpHybridLowerPolicy::ApplyOwnedJoints(
    robot_io::RobotCommand& command, bool legs_enabled,
    bool waist_enabled) const noexcept {
  for (int i = 0; i < kActionDim; ++i) {
    if (!OwnsActionIndex(i, legs_enabled, waist_enabled)) continue;
    const int sdk = kHumanLikeToSdk[i];
    command.q_des[sdk] = kDefaultQ[i] + kActionScale[i] * raw_action_[i];
    command.dq_des[sdk] = 0.0;
    command.tau_ff[sdk] = 0.0;
    command.kp[sdk] = kKp[i];
    command.kd[sdk] = kKd[i];
  }
}

bool PpHybridLowerPolicy::Apply(const robot_io::RobotState& state,
                                robot_io::RobotCommand& command,
                                bool legs_enabled,
                                bool waist_enabled) noexcept {
  if (!runtime_ || state.q.size() != 31 || state.dq.size() != 31 ||
      command.q_des.size() != 31 || command.dq_des.size() != 31 ||
      command.tau_ff.size() != 31 || command.kp.size() != 31 ||
      command.kd.size() != 31) {
    failures_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (needs_entry_seed_) SeedEntryState(state);
  UpdateFilteredCommand();
  // HumanLike is evaluated at its native 100 Hz. With the hybrid Runner this
  // loop normally has one iteration; retaining the computed ratio also keeps
  // the temporal contract explicit if the caller rate is changed later.
  for (int i = 0; i < substeps_per_outer_tick_; ++i)
    PushOneHumanLikeHistoryFrame(state);
  const auto begin = std::chrono::steady_clock::now();
  const bool infer_ok = runtime_->Infer(history_);
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());
  if (!infer_ok) {
    failures_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  inference_ns_total_.fetch_add(elapsed_ns, std::memory_order_relaxed);
  AtomicMax(inference_ns_max_, elapsed_ns);
  raw_action_ = runtime_->action();
  inference_steps_.fetch_add(1, std::memory_order_relaxed);
  ApplyOwnedJoints(command, legs_enabled, waist_enabled);
  outer_ticks_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

PpHybridLowerDiag PpHybridLowerPolicy::diag() const noexcept {
  PpHybridLowerDiag out;
  out.outer_ticks = outer_ticks_.load(std::memory_order_relaxed);
  out.inference_steps = inference_steps_.load(std::memory_order_relaxed);
  out.failures = failures_.load(std::memory_order_relaxed);
  const auto total_ns = inference_ns_total_.load(std::memory_order_relaxed);
  const auto max_ns = inference_ns_max_.load(std::memory_order_relaxed);
  out.mean_inference_ms =
      out.inference_steps == 0
          ? 0.0
          : static_cast<double>(total_ns) /
                static_cast<double>(out.inference_steps) / 1.0e6;
  out.max_inference_ms = static_cast<double>(max_ns) / 1.0e6;
  return out;
}

}  // namespace a3_pingpong
