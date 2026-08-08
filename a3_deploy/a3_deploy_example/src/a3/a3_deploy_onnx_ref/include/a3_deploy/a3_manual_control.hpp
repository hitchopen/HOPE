// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Table-driven manual control state machine for A3 deploy.
#pragma once

#include <atomic>
#include <cstdint>
#include <iosfwd>
#include <span>
#include <string>
#include <vector>

namespace a3_deploy {

enum class DeployMode : int {
  kIdle = 0,
  kPassive = 1,
  kPdStand = 2,
  kMotion = 3,
  kTeleop = 4,
};

enum class ManualKey {
  kUnknown,
  kPassive,
  kPdStand,
  kMotion,
  kTeleop,
  kSpace,
  kMotionForward,
  kMotionBackward,
  kMotionTurnLeft,
  kMotionTurnRight,
  kRestartMotionPlay,
  kRestartMotionPause,
  kPrevMotion,
  kNextMotion,
  kTeleopSourceA3,
  kTeleopSourceSmpl,
  kTeleopSourceA3Fast,
  kHelp,
  kQuit,
};

enum class TransitionGuard {
  kAlways,
  kPdStandReady,
  kMotionOnly,
  kTeleopOnly,
};

enum class TransitionAction {
  kEnterPassive,
  kEnterPdStand,
  kEnterMotionPaused,
  kEnterTeleop,
  kToggleMotionPlay,
  kRestartMotionPlay,
  kRestartMotionPause,
  kSelectPrevMotion,
  kSelectNextMotion,
  kToggleTeleopPause,
  kSelectTeleopSourceA3,
  kSelectTeleopSourceSmpl,
  kSelectTeleopSourceA3Fast,
  kPrintHelp,
  kQuit,
  kIgnore,
};

enum class ManualKeyOutcome {
  kUnknown,
  kHandled,
  kBlocked,
  kQuit,
};

struct ManualTransitionRule {
  DeployMode mode;
  ManualKey key;
  TransitionGuard guard;
  TransitionAction action;
  const char* blocked_message;
};

struct ManualControlState {
  std::atomic<int> mode{static_cast<int>(DeployMode::kIdle)};
  std::atomic<std::uint64_t> epoch{0};
  std::atomic<std::uint64_t> pd_ticks{0};
  std::atomic<bool> pd_stand_ready{false};
  std::atomic<std::uint64_t> motion_tick{0};
  std::atomic<bool> motion_held{false};
  std::atomic<int> selected_motion_index{0};
  std::atomic<bool> motion_playing{false};
  std::atomic<std::uint64_t> motion_command_epoch{0};
  std::atomic<int> selected_remote_motion_index{-1};
  std::atomic<bool> remote_motion_active{false};
  std::atomic<bool> teleop_input_paused{false};
  std::atomic<std::uint64_t> teleop_command_epoch{0};
  std::atomic<int> teleop_source{0};  // 0=a3, 1=smpl, 2=a3_fast
  std::atomic<bool> policy_yaw_offset_valid{false};
  std::atomic<int> policy_yaw_offset_mdeg{0};
};

struct ManualMotionShortcuts {
  int forward = -1;
  int backward = -1;
  int turn_left = -1;
  int turn_right = -1;
};

const char* DeployModeName(DeployMode mode) noexcept;
const char* ManualKeyName(ManualKey key) noexcept;

DeployMode LoadDeployMode(const ManualControlState& control) noexcept;
ManualKey ParseManualKey(char ch) noexcept;

std::span<const ManualTransitionRule> ManualTransitionTable() noexcept;
const ManualTransitionRule* FindManualTransitionRule(
    DeployMode mode,
    ManualKey key) noexcept;

void RequestDeployMode(ManualControlState& control,
                       DeployMode next,
                       std::ostream& log);
void SelectMotion(ManualControlState& control,
                  int selected_index,
                  const std::vector<std::string>& motion_names,
                  std::ostream& log);
void SelectMotionDelta(ManualControlState& control,
                       int delta,
                       const std::vector<std::string>& motion_names,
                       std::ostream& log);
void PrintMotionHelp(const ManualControlState& control,
                     const std::vector<std::string>& motion_names,
                     std::ostream& log,
                     const ManualMotionShortcuts* shortcuts = nullptr);
void PrintMotionHelp(const ManualControlState& control,
                     const std::vector<std::string>& motion_names,
                     const std::vector<std::string>& remote_motion_names,
                     std::ostream& log,
                     const ManualMotionShortcuts* shortcuts = nullptr);

ManualKeyOutcome HandleManualKey(
    ManualControlState& control,
    ManualKey key,
    const std::vector<std::string>& motion_names,
    std::ostream& log,
    const ManualMotionShortcuts* shortcuts = nullptr);
ManualKeyOutcome HandleManualKey(
    ManualControlState& control,
    ManualKey key,
    const std::vector<std::string>& motion_names,
    const std::vector<std::string>& remote_motion_names,
    std::ostream& log,
    const ManualMotionShortcuts* shortcuts = nullptr);

}  // namespace a3_deploy
