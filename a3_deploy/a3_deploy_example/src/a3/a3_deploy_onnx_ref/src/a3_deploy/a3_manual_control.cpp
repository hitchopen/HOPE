// Copyright (c) 2026, AgiBot Inc. All rights reserved.

#include "a3_deploy/a3_manual_control.hpp"

#include "a3_deploy/a3_motion_library.hpp"

#include <algorithm>
#include <array>
#include <iostream>

namespace a3_deploy {
namespace {

constexpr const char* kNeedPdStand =
    "press s and wait for pd_stand ready before entering motion/teleop";
constexpr const char* kEnterMotionFirst = "enter motion with m first";
constexpr const char* kMotionOnly = "motion controls are only active in motion";
constexpr const char* kTeleopOnly = "teleop pause is only active in teleop";
constexpr const char* kAlreadyMotion = "already in motion";
constexpr const char* kAlreadyTeleop = "already in teleop";

constexpr auto kManualTransitionRules = std::to_array<ManualTransitionRule>({
    // Global safety/help.
    {DeployMode::kIdle, ManualKey::kPassive, TransitionGuard::kAlways,
     TransitionAction::kEnterPassive, nullptr},
    {DeployMode::kPassive, ManualKey::kPassive, TransitionGuard::kAlways,
     TransitionAction::kEnterPassive, nullptr},
    {DeployMode::kPdStand, ManualKey::kPassive, TransitionGuard::kAlways,
     TransitionAction::kEnterPassive, nullptr},
    {DeployMode::kMotion, ManualKey::kPassive, TransitionGuard::kAlways,
     TransitionAction::kEnterPassive, nullptr},
    {DeployMode::kTeleop, ManualKey::kPassive, TransitionGuard::kAlways,
     TransitionAction::kEnterPassive, nullptr},
    {DeployMode::kIdle, ManualKey::kPdStand, TransitionGuard::kAlways,
     TransitionAction::kEnterPdStand, nullptr},
    {DeployMode::kPassive, ManualKey::kPdStand, TransitionGuard::kAlways,
     TransitionAction::kEnterPdStand, nullptr},
    {DeployMode::kPdStand, ManualKey::kPdStand, TransitionGuard::kAlways,
     TransitionAction::kEnterPdStand, nullptr},
    {DeployMode::kMotion, ManualKey::kPdStand, TransitionGuard::kAlways,
     TransitionAction::kEnterPdStand, nullptr},
    {DeployMode::kTeleop, ManualKey::kPdStand, TransitionGuard::kAlways,
     TransitionAction::kEnterPdStand, nullptr},
    {DeployMode::kIdle, ManualKey::kHelp, TransitionGuard::kAlways,
     TransitionAction::kPrintHelp, nullptr},
    {DeployMode::kPassive, ManualKey::kHelp, TransitionGuard::kAlways,
     TransitionAction::kPrintHelp, nullptr},
    {DeployMode::kPdStand, ManualKey::kHelp, TransitionGuard::kAlways,
     TransitionAction::kPrintHelp, nullptr},
    {DeployMode::kMotion, ManualKey::kHelp, TransitionGuard::kAlways,
     TransitionAction::kPrintHelp, nullptr},
    {DeployMode::kTeleop, ManualKey::kHelp, TransitionGuard::kAlways,
     TransitionAction::kPrintHelp, nullptr},
    {DeployMode::kIdle, ManualKey::kQuit, TransitionGuard::kAlways,
     TransitionAction::kQuit, nullptr},
    {DeployMode::kPassive, ManualKey::kQuit, TransitionGuard::kAlways,
     TransitionAction::kQuit, nullptr},
    {DeployMode::kPdStand, ManualKey::kQuit, TransitionGuard::kAlways,
     TransitionAction::kQuit, nullptr},
    {DeployMode::kMotion, ManualKey::kQuit, TransitionGuard::kAlways,
     TransitionAction::kQuit, nullptr},
    {DeployMode::kTeleop, ManualKey::kQuit, TransitionGuard::kAlways,
     TransitionAction::kQuit, nullptr},

    // Idle/passive cannot enter high-level policy modes directly.
    {DeployMode::kIdle, ManualKey::kMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kNeedPdStand},
    {DeployMode::kIdle, ManualKey::kTeleop, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kNeedPdStand},
    {DeployMode::kIdle, ManualKey::kSpace, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kIdle, ManualKey::kRestartMotionPlay, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kIdle, ManualKey::kRestartMotionPause, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kIdle, ManualKey::kPrevMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kIdle, ManualKey::kNextMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kPassive, ManualKey::kMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kNeedPdStand},
    {DeployMode::kPassive, ManualKey::kTeleop, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kNeedPdStand},
    {DeployMode::kPassive, ManualKey::kSpace, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPassive, ManualKey::kRestartMotionPlay,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPassive, ManualKey::kRestartMotionPause,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPassive, ManualKey::kPrevMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kPassive, ManualKey::kNextMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},

    // PD stand is the protected gateway to motion/teleop.
    {DeployMode::kPdStand, ManualKey::kMotion, TransitionGuard::kPdStandReady,
     TransitionAction::kEnterMotionPaused, kNeedPdStand},
    {DeployMode::kPdStand, ManualKey::kTeleop, TransitionGuard::kPdStandReady,
     TransitionAction::kEnterTeleop, kNeedPdStand},
    {DeployMode::kPdStand, ManualKey::kSpace, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPdStand, ManualKey::kRestartMotionPlay,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPdStand, ManualKey::kRestartMotionPause,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kEnterMotionFirst},
    {DeployMode::kPdStand, ManualKey::kPrevMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kPdStand, ManualKey::kNextMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},

    // Motion controls.
    {DeployMode::kMotion, ManualKey::kMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kAlreadyMotion},
    {DeployMode::kMotion, ManualKey::kTeleop, TransitionGuard::kAlways,
     TransitionAction::kEnterTeleop, nullptr},
    {DeployMode::kMotion, ManualKey::kSpace, TransitionGuard::kMotionOnly,
     TransitionAction::kToggleMotionPlay, nullptr},
    {DeployMode::kMotion, ManualKey::kRestartMotionPlay,
     TransitionGuard::kMotionOnly, TransitionAction::kRestartMotionPlay,
     nullptr},
    {DeployMode::kMotion, ManualKey::kRestartMotionPause,
     TransitionGuard::kMotionOnly, TransitionAction::kRestartMotionPause,
     nullptr},
    {DeployMode::kMotion, ManualKey::kPrevMotion, TransitionGuard::kMotionOnly,
     TransitionAction::kSelectPrevMotion, nullptr},
    {DeployMode::kMotion, ManualKey::kNextMotion, TransitionGuard::kMotionOnly,
     TransitionAction::kSelectNextMotion, nullptr},

    // Teleop controls. Motion keys are intentionally inert here.
    {DeployMode::kTeleop, ManualKey::kTeleop, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kAlreadyTeleop},
    {DeployMode::kTeleop, ManualKey::kMotion, TransitionGuard::kAlways,
     TransitionAction::kEnterMotionPaused, nullptr},
    {DeployMode::kTeleop, ManualKey::kSpace, TransitionGuard::kTeleopOnly,
     TransitionAction::kToggleTeleopPause, nullptr},
    {DeployMode::kTeleop, ManualKey::kRestartMotionPlay,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kTeleop, ManualKey::kRestartMotionPause,
     TransitionGuard::kAlways, TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kTeleop, ManualKey::kPrevMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kTeleop, ManualKey::kNextMotion, TransitionGuard::kAlways,
     TransitionAction::kIgnore, kMotionOnly},
    {DeployMode::kTeleop, ManualKey::kTeleopSourceA3,
     TransitionGuard::kAlways, TransitionAction::kSelectTeleopSourceA3,
     nullptr},
    {DeployMode::kTeleop, ManualKey::kTeleopSourceSmpl,
     TransitionGuard::kAlways, TransitionAction::kSelectTeleopSourceSmpl,
     nullptr},
    {DeployMode::kTeleop, ManualKey::kTeleopSourceA3Fast,
     TransitionGuard::kAlways, TransitionAction::kSelectTeleopSourceA3Fast,
     nullptr},
});

bool GuardPasses(const ManualControlState& control,
                 TransitionGuard guard) noexcept {
  switch (guard) {
    case TransitionGuard::kAlways:
      return true;
    case TransitionGuard::kPdStandReady:
      return control.pd_stand_ready.load(std::memory_order_acquire);
    case TransitionGuard::kMotionOnly:
      return LoadDeployMode(control) == DeployMode::kMotion;
    case TransitionGuard::kTeleopOnly:
      return LoadDeployMode(control) == DeployMode::kTeleop;
  }
  return false;
}

bool RequireMotion(const std::vector<std::string>& motion_names,
                   std::ostream& log) {
  if (!motion_names.empty()) return true;
  log << "[motion] ignored: no motions loaded\n";
  return false;
}

bool RequireRemoteMotion(const std::vector<std::string>& motion_names,
                         std::ostream& log) {
  if (!motion_names.empty()) return true;
  log << "[remote] ignored: no remote motions loaded\n";
  return false;
}

void ClearRemoteMotion(ManualControlState& control) noexcept {
  control.remote_motion_active.store(false, std::memory_order_release);
}

bool IsMotionShortcutKey(ManualKey key) noexcept {
  return key == ManualKey::kMotionForward ||
         key == ManualKey::kMotionBackward ||
         key == ManualKey::kMotionTurnLeft ||
         key == ManualKey::kMotionTurnRight;
}

int MotionShortcutIndex(ManualKey key,
                        const ManualMotionShortcuts* shortcuts) noexcept {
  if (!shortcuts) return -1;
  switch (key) {
    case ManualKey::kMotionForward: return shortcuts->forward;
    case ManualKey::kMotionBackward: return shortcuts->backward;
    case ManualKey::kMotionTurnLeft: return shortcuts->turn_left;
    case ManualKey::kMotionTurnRight: return shortcuts->turn_right;
    default: return -1;
  }
}

const char* MotionShortcutLabel(ManualKey key) noexcept {
  switch (key) {
    case ManualKey::kMotionForward: return "arrow-up";
    case ManualKey::kMotionBackward: return "arrow-down";
    case ManualKey::kMotionTurnLeft: return "arrow-left";
    case ManualKey::kMotionTurnRight: return "arrow-right";
    default: return "shortcut";
  }
}

ManualKeyOutcome PlayMotionShortcut(
    ManualControlState& control,
    ManualKey key,
    const std::vector<std::string>& remote_motion_names,
    const ManualMotionShortcuts* shortcuts,
    std::ostream& log) {
  if (!RequireRemoteMotion(remote_motion_names, log)) {
    return ManualKeyOutcome::kBlocked;
  }

  const int selected_index = MotionShortcutIndex(key, shortcuts);
  if (selected_index < 0 ||
      selected_index >= static_cast<int>(remote_motion_names.size())) {
    log << "[remote] ignored " << MotionShortcutLabel(key)
        << ": shortcut is not configured\n";
    return ManualKeyOutcome::kBlocked;
  }

  const DeployMode mode = LoadDeployMode(control);
  if (mode == DeployMode::kIdle || mode == DeployMode::kPassive) {
    log << "[mode] ignored '" << MotionShortcutLabel(key)
        << "' in " << DeployModeName(mode) << ": " << kNeedPdStand
        << "\n";
    return ManualKeyOutcome::kBlocked;
  }
  if (mode == DeployMode::kPdStand &&
      !control.pd_stand_ready.load(std::memory_order_acquire)) {
    log << "[mode] ignored '" << MotionShortcutLabel(key)
        << "' in " << DeployModeName(mode) << ": " << kNeedPdStand
        << "\n";
    return ManualKeyOutcome::kBlocked;
  }

  control.selected_remote_motion_index.store(selected_index,
                                             std::memory_order_release);
  control.remote_motion_active.store(true, std::memory_order_release);
  control.motion_playing.store(true, std::memory_order_release);
  control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
  log << "[remote] " << MotionShortcutLabel(key) << " -> ["
      << (selected_index + 1) << "/" << remote_motion_names.size() << "] "
      << remote_motion_names[selected_index]
      << " (play from frame 0; normal motion selection unchanged)\n";
  RequestDeployMode(control, DeployMode::kMotion, log);
  return ManualKeyOutcome::kHandled;
}

}  // namespace

const char* DeployModeName(DeployMode mode) noexcept {
  switch (mode) {
    case DeployMode::kIdle: return "idle";
    case DeployMode::kPassive: return "passive";
    case DeployMode::kPdStand: return "pd_stand";
    case DeployMode::kMotion: return "motion";
    case DeployMode::kTeleop: return "teleop";
  }
  return "unknown";
}

const char* ManualKeyName(ManualKey key) noexcept {
  switch (key) {
    case ManualKey::kPassive: return "p";
    case ManualKey::kPdStand: return "s";
    case ManualKey::kMotion: return "m";
    case ManualKey::kTeleop: return "t";
    case ManualKey::kSpace: return "space";
    case ManualKey::kMotionForward: return "arrow-up";
    case ManualKey::kMotionBackward: return "arrow-down";
    case ManualKey::kMotionTurnLeft: return "arrow-left";
    case ManualKey::kMotionTurnRight: return "arrow-right";
    case ManualKey::kRestartMotionPlay: return "r";
    case ManualKey::kRestartMotionPause: return "x";
    case ManualKey::kPrevMotion: return "[";
    case ManualKey::kNextMotion: return "]";
    case ManualKey::kTeleopSourceA3: return "1";
    case ManualKey::kTeleopSourceSmpl: return "2";
    case ManualKey::kTeleopSourceA3Fast: return "3";
    case ManualKey::kHelp: return "h";
    case ManualKey::kQuit: return "q";
    case ManualKey::kUnknown: return "unknown";
  }
  return "unknown";
}

DeployMode LoadDeployMode(const ManualControlState& control) noexcept {
  return static_cast<DeployMode>(control.mode.load(std::memory_order_acquire));
}

ManualKey ParseManualKey(char ch) noexcept {
  switch (ch) {
    case 'p':
    case 'P': return ManualKey::kPassive;
    case 's':
    case 'S': return ManualKey::kPdStand;
    case 'm':
    case 'M': return ManualKey::kMotion;
    case 't':
    case 'T': return ManualKey::kTeleop;
    case ' ': return ManualKey::kSpace;
    case 'r':
    case 'R': return ManualKey::kRestartMotionPlay;
    case 'x':
    case 'X': return ManualKey::kRestartMotionPause;
    case '[':
    case ',': return ManualKey::kPrevMotion;
    case ']':
    case '.': return ManualKey::kNextMotion;
    case '1': return ManualKey::kTeleopSourceA3;
    case '2': return ManualKey::kTeleopSourceSmpl;
    case '3': return ManualKey::kTeleopSourceA3Fast;
    case 'h':
    case 'H': return ManualKey::kHelp;
    case 'q':
    case 'Q': return ManualKey::kQuit;
    default: return ManualKey::kUnknown;
  }
}

std::span<const ManualTransitionRule> ManualTransitionTable() noexcept {
  return {kManualTransitionRules.data(), kManualTransitionRules.size()};
}

const ManualTransitionRule* FindManualTransitionRule(
    DeployMode mode,
    ManualKey key) noexcept {
  for (const auto& rule : kManualTransitionRules) {
    if (rule.mode == mode && rule.key == key) return &rule;
  }
  return nullptr;
}

void RequestDeployMode(ManualControlState& control,
                       DeployMode next,
                       std::ostream& log) {
  const DeployMode prev = LoadDeployMode(control);
  control.mode.store(static_cast<int>(next), std::memory_order_release);
  control.epoch.fetch_add(1, std::memory_order_acq_rel);
  control.pd_ticks.store(0, std::memory_order_release);
  control.pd_stand_ready.store(false, std::memory_order_release);
  control.motion_tick.store(0, std::memory_order_release);
  control.motion_held.store(false, std::memory_order_release);
  control.policy_yaw_offset_valid.store(false, std::memory_order_release);
  control.policy_yaw_offset_mdeg.store(0, std::memory_order_release);
  if (next != DeployMode::kMotion) {
    control.motion_playing.store(false, std::memory_order_release);
    ClearRemoteMotion(control);
  }
  if (next != DeployMode::kTeleop) {
    control.teleop_input_paused.store(false, std::memory_order_release);
  }
  log << "[mode] " << DeployModeName(prev) << " -> "
      << DeployModeName(next);
  if (prev == next) log << " (restart)";
  log << "\n";
}

void SelectMotion(ManualControlState& control,
                  int selected_index,
                  const std::vector<std::string>& motion_names,
                  std::ostream& log) {
  if (motion_names.empty()) return;
  const int clamped =
      std::clamp(selected_index, 0,
                 static_cast<int>(motion_names.size()) - 1);
  const DeployMode mode = LoadDeployMode(control);
  const bool was_playing =
      control.motion_playing.exchange(false, std::memory_order_acq_rel);
  const bool was_remote =
      control.remote_motion_active.exchange(false, std::memory_order_acq_rel);
  control.selected_motion_index.store(clamped, std::memory_order_release);
  if (mode == DeployMode::kMotion && (was_playing || was_remote)) {
    control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
  }
  log << "[motion] selected [" << (clamped + 1) << "/"
      << motion_names.size() << "] " << motion_names[clamped]
      << " (pending; standing idle until r/space starts playback)\n";
}

void SelectMotionDelta(ManualControlState& control,
                       int delta,
                       const std::vector<std::string>& motion_names,
                       std::ostream& log) {
  if (motion_names.empty()) return;
  const std::size_t current = static_cast<std::size_t>(
      std::max(0, control.selected_motion_index.load(std::memory_order_acquire)));
  const std::size_t next =
      WrapMotionIndex(current, delta, motion_names.size());
  SelectMotion(control, static_cast<int>(next), motion_names, log);
}

void PrintMotionHelp(const ManualControlState& control,
                     const std::vector<std::string>& motion_names,
                     std::ostream& log,
                     const ManualMotionShortcuts* shortcuts) {
  static const std::vector<std::string> kNoRemoteMotions;
  PrintMotionHelp(control, motion_names, kNoRemoteMotions, log, shortcuts);
}

void PrintMotionHelp(const ManualControlState& control,
                     const std::vector<std::string>& motion_names,
                     const std::vector<std::string>& remote_motion_names,
                     std::ostream& log,
                     const ManualMotionShortcuts* shortcuts) {
  const DeployMode mode = LoadDeployMode(control);
  const int selected = control.selected_motion_index.load(
      std::memory_order_acquire);
  log << "[mode] current=" << DeployModeName(mode)
      << " keys: p=passive, s=pd_stand, m=motion, t=teleop, "
         "arrows=remote clip, space=mode action, h=help, q=quit\n"
      << "[mode] guards: motion/teleop entry from idle/passive requires "
         "pd_stand ready; r/x/[ ] only work in motion; space in teleop "
         "pauses teleop input; 1/2/3 select a3/smpl/a3_fast teleop source\n";
  log << "[motion] controls in motion: space=play/pause current source, "
         "r=restart normal+play, x=restart normal+pause, [=prev normal, "
         "]=next normal\n";
  if (shortcuts) {
    auto print_shortcut = [&](const char* key, int index) {
      log << "[remote] " << key << "=";
      if (index >= 0 &&
          index < static_cast<int>(remote_motion_names.size())) {
        log << remote_motion_names[index];
      } else {
        log << "unconfigured";
      }
      log << "\n";
    };
    print_shortcut("arrow-up forward", shortcuts->forward);
    print_shortcut("arrow-down backward", shortcuts->backward);
    print_shortcut("arrow-left turn_left", shortcuts->turn_left);
    print_shortcut("arrow-right turn_right", shortcuts->turn_right);
  }
  if (motion_names.empty()) return;
  log << "[motion] motions:";
  for (std::size_t i = 0; i < motion_names.size(); ++i) {
    log << "\n  " << ((static_cast<int>(i) == selected) ? "*" : " ")
        << " [" << (i + 1) << "] " << motion_names[i];
  }
  log << "\n";
}

ManualKeyOutcome HandleManualKey(
    ManualControlState& control,
    ManualKey key,
    const std::vector<std::string>& motion_names,
    std::ostream& log,
    const ManualMotionShortcuts* shortcuts) {
  static const std::vector<std::string> kNoRemoteMotions;
  return HandleManualKey(control, key, motion_names, kNoRemoteMotions, log,
                         shortcuts);
}

ManualKeyOutcome HandleManualKey(
    ManualControlState& control,
    ManualKey key,
    const std::vector<std::string>& motion_names,
    const std::vector<std::string>& remote_motion_names,
    std::ostream& log,
    const ManualMotionShortcuts* shortcuts) {
  if (key == ManualKey::kUnknown) return ManualKeyOutcome::kUnknown;

  if (IsMotionShortcutKey(key)) {
    return PlayMotionShortcut(control, key, remote_motion_names, shortcuts,
                              log);
  }

  const DeployMode mode = LoadDeployMode(control);
  const ManualTransitionRule* rule = FindManualTransitionRule(mode, key);
  if (rule == nullptr) {
    log << "[mode] ignored '" << ManualKeyName(key)
        << "' in " << DeployModeName(mode) << "\n";
    return ManualKeyOutcome::kBlocked;
  }

  if (!GuardPasses(control, rule->guard)) {
    log << "[mode] ignored '" << ManualKeyName(key)
        << "' in " << DeployModeName(mode);
    if (rule->blocked_message) log << ": " << rule->blocked_message;
    log << "\n";
    return ManualKeyOutcome::kBlocked;
  }

  switch (rule->action) {
    case TransitionAction::kEnterPassive:
      RequestDeployMode(control, DeployMode::kPassive, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kEnterPdStand:
      RequestDeployMode(control, DeployMode::kPdStand, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kEnterMotionPaused:
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      ClearRemoteMotion(control);
      control.motion_playing.store(false, std::memory_order_release);
      control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      RequestDeployMode(control, DeployMode::kMotion, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kEnterTeleop:
      control.teleop_input_paused.store(false, std::memory_order_release);
      control.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      RequestDeployMode(control, DeployMode::kTeleop, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kToggleMotionPlay: {
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      const bool next =
          !control.motion_playing.load(std::memory_order_acquire);
      control.motion_playing.store(next, std::memory_order_release);
      control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[motion] " << (next ? "play" : "pause") << "\n";
      return ManualKeyOutcome::kHandled;
    }
    case TransitionAction::kRestartMotionPlay:
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      ClearRemoteMotion(control);
      control.motion_playing.store(true, std::memory_order_release);
      control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      RequestDeployMode(control, DeployMode::kMotion, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kRestartMotionPause:
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      ClearRemoteMotion(control);
      control.motion_playing.store(false, std::memory_order_release);
      control.motion_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[motion] restart paused; standing idle\n";
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kSelectPrevMotion:
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      SelectMotionDelta(control, -1, motion_names, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kSelectNextMotion:
      if (!RequireMotion(motion_names, log)) return ManualKeyOutcome::kBlocked;
      SelectMotionDelta(control, 1, motion_names, log);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kToggleTeleopPause: {
      const bool next =
          !control.teleop_input_paused.load(std::memory_order_acquire);
      control.teleop_input_paused.store(next, std::memory_order_release);
      control.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[teleop] input " << (next ? "pause" : "resume") << "\n";
      return ManualKeyOutcome::kHandled;
    }
    case TransitionAction::kSelectTeleopSourceA3:
      control.teleop_source.store(0, std::memory_order_release);
      control.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[teleop] requested source a3\n";
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kSelectTeleopSourceSmpl:
      control.teleop_source.store(1, std::memory_order_release);
      control.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[teleop] requested source smpl\n";
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kSelectTeleopSourceA3Fast:
      control.teleop_source.store(2, std::memory_order_release);
      control.teleop_command_epoch.fetch_add(1, std::memory_order_acq_rel);
      log << "[teleop] requested source a3_fast\n";
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kPrintHelp:
      PrintMotionHelp(control, motion_names, remote_motion_names, log,
                      shortcuts);
      return ManualKeyOutcome::kHandled;
    case TransitionAction::kQuit:
      return ManualKeyOutcome::kQuit;
    case TransitionAction::kIgnore:
      log << "[mode] ignored '" << ManualKeyName(key)
          << "' in " << DeployModeName(mode);
      if (rule->blocked_message) log << ": " << rule->blocked_message;
      log << "\n";
      return ManualKeyOutcome::kBlocked;
  }
  return ManualKeyOutcome::kBlocked;
}

}  // namespace a3_deploy
