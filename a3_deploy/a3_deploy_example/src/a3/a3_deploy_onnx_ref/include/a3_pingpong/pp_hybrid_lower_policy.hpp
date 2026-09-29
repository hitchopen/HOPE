#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace a3_pingpong {

// receive policy always runs first. HumanLike then owns the selected lower-body
// joints inside the same Runner and before the one RobotCommand publisher.
enum class PpHybridLowerOwner {
  kLegs,
  kWaistRollAndLegs,
  kWaistRollPitchAndLegs,
  kWaistAndLegs,
};

const char* PpHybridLowerOwnerName(PpHybridLowerOwner owner) noexcept;

struct PpHybridLowerConfig {
  std::string model_path;
  std::string backend{"ort_cpu"};
  PpHybridLowerOwner owner{PpHybridLowerOwner::kLegs};

  // The shared Runner calls HumanLike at its native 100 Hz. receive policy is
  // evaluated every other Runner tick and its upper-body command is held.
  double outer_policy_dt{0.01};
  double humanlike_policy_dt{0.01};
  double phase_offset{0.46};
  bool smart_walk_zero_settled{true};

  // Zero is the standing/in-place command used for the first MuJoCo A/B.
  std::array<double, 3> command_velocity_xyz{0.0, 0.0, 0.0};
};

struct PpHybridLowerDiag {
  std::uint64_t outer_ticks{0};
  std::uint64_t inference_steps{0};
  std::uint64_t failures{0};
  double mean_inference_ms{0.0};
  double max_inference_ms{0.0};
};

class PpHumanLikeOrtRuntime;

// Robot-side HumanLike contract:
//   input  "input"  [1, 66*55]
//   output "output" [1, 15]
// The 15 actions are L leg 6, R leg 6, waist yaw/roll/pitch. History and raw
// previous-action state are private to this actor; receive policy keeps its own
// original 31-action feedback history.
class PpHybridLowerPolicy {
 public:
  explicit PpHybridLowerPolicy(PpHybridLowerConfig config);
  ~PpHybridLowerPolicy();

  bool Initialize();
  void Reset() noexcept;

  bool Apply(const robot_io::RobotState& state,
             robot_io::RobotCommand& command,
             bool legs_enabled = true,
             bool waist_enabled = true) noexcept;

  // HumanLike's locomotion input is updated independently from actor state.
  // The target is clamped and passed through the original 500 Hz command EMA
  // inside Apply() before it reaches the 100 Hz observation.
  void SetCommandVelocityTarget(
      const std::array<double, 3>& command_velocity_xyz) noexcept;

  const PpHybridLowerConfig& config() const noexcept { return config_; }
  PpHybridLowerDiag diag() const noexcept;

 private:
  void SeedEntryState(const robot_io::RobotState& state) noexcept;
  double UpdatePhase(const robot_io::RobotState& state, double waist_roll,
                     double waist_pitch) noexcept;
  void BuildCurrentObservation(const robot_io::RobotState& state) noexcept;
  void UpdateFilteredCommand() noexcept;
  void PushOneHumanLikeHistoryFrame(
      const robot_io::RobotState& state) noexcept;
  bool OwnsActionIndex(int action_index, bool legs_enabled,
                       bool waist_enabled) const noexcept;
  void ApplyOwnedJoints(robot_io::RobotCommand& command, bool legs_enabled,
                        bool waist_enabled) const noexcept;

  PpHybridLowerConfig config_;
  std::unique_ptr<PpHumanLikeOrtRuntime> runtime_;

  std::array<float, 55> current_obs_{};
  std::array<float, 66 * 55> history_{};
  std::array<float, 15> raw_action_{};
  std::array<double, 3> target_command_velocity_xyz_{};
  std::array<double, 3> filtered_command_velocity_xyz_{};
  bool needs_entry_seed_{true};
  int phase_index_{96};
  int low_speed_count_{41};
  bool phase_active_{true};
  int substeps_per_outer_tick_{1};

  std::atomic<std::uint64_t> outer_ticks_{0};
  std::atomic<std::uint64_t> inference_steps_{0};
  std::atomic<std::uint64_t> failures_{0};
  std::atomic<std::uint64_t> inference_ns_total_{0};
  std::atomic<std::uint64_t> inference_ns_max_{0};
};

}  // namespace a3_pingpong
