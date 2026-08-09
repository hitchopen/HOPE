#include "a3_deploy/a3_manual_control.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

namespace {

using a3_deploy::DeployMode;
using a3_deploy::HandleManualKey;
using a3_deploy::LoadDeployMode;
using a3_deploy::ManualControlState;
using a3_deploy::ManualKey;
using a3_deploy::ManualKeyOutcome;
using a3_deploy::ManualMotionShortcuts;
using a3_deploy::RequestDeployMode;
using a3_deploy::TransitionAction;
using a3_deploy::TransitionGuard;

const std::vector<std::string>& Motions() {
  static const std::vector<std::string> motions{"alpha.csv", "bravo.csv",
                                                "charlie.csv"};
  return motions;
}

const std::vector<std::string>& RemoteMotions() {
  static const std::vector<std::string> motions{
      "remote_forward_short", "remote_backward_short",
      "remote_turn_left_45", "remote_turn_right_45"};
  return motions;
}

ManualKeyOutcome Press(ManualControlState& control,
                       ManualKey key,
                       std::ostringstream& log) {
  return HandleManualKey(control, key, Motions(), log);
}

void SetMode(ManualControlState& control,
             DeployMode mode,
             std::ostringstream& log) {
  RequestDeployMode(control, mode, log);
  log.str("");
  log.clear();
}

}  // namespace

TEST(A3ManualControl, IdleAndPassiveBlockMotionAndTeleopEntries) {
  ManualControlState control;
  std::ostringstream log;

  for (ManualKey key : {ManualKey::kMotion, ManualKey::kTeleop,
                        ManualKey::kRestartMotionPlay, ManualKey::kSpace,
                        ManualKey::kPrevMotion, ManualKey::kNextMotion,
                        ManualKey::kRestartMotionPause}) {
    EXPECT_EQ(Press(control, key, log), ManualKeyOutcome::kBlocked);
    EXPECT_EQ(LoadDeployMode(control), DeployMode::kIdle);
  }

  EXPECT_EQ(Press(control, ManualKey::kPassive, log),
            ManualKeyOutcome::kHandled);
  ASSERT_EQ(LoadDeployMode(control), DeployMode::kPassive);
  for (ManualKey key : {ManualKey::kMotion, ManualKey::kTeleop,
                        ManualKey::kRestartMotionPlay, ManualKey::kSpace,
                        ManualKey::kPrevMotion, ManualKey::kNextMotion,
                        ManualKey::kRestartMotionPause}) {
    EXPECT_EQ(Press(control, key, log), ManualKeyOutcome::kBlocked);
    EXPECT_EQ(LoadDeployMode(control), DeployMode::kPassive);
  }
}

TEST(A3ManualControl, TransitionTableExposesSafetyRules) {
  const auto* passive_to_motion = a3_deploy::FindManualTransitionRule(
      DeployMode::kPassive, ManualKey::kMotion);
  ASSERT_NE(passive_to_motion, nullptr);
  EXPECT_EQ(passive_to_motion->action, TransitionAction::kIgnore);

  const auto* pd_to_motion = a3_deploy::FindManualTransitionRule(
      DeployMode::kPdStand, ManualKey::kMotion);
  ASSERT_NE(pd_to_motion, nullptr);
  EXPECT_EQ(pd_to_motion->guard, TransitionGuard::kPdStandReady);
  EXPECT_EQ(pd_to_motion->action, TransitionAction::kEnterMotionPaused);

  const auto* teleop_space = a3_deploy::FindManualTransitionRule(
      DeployMode::kTeleop, ManualKey::kSpace);
  ASSERT_NE(teleop_space, nullptr);
  EXPECT_EQ(teleop_space->action, TransitionAction::kToggleTeleopPause);
}

TEST(A3ManualControl, PdStandReadyAllowsMotionAndTeleop) {
  ManualControlState control;
  std::ostringstream log;

  EXPECT_EQ(Press(control, ManualKey::kPdStand, log),
            ManualKeyOutcome::kHandled);
  ASSERT_EQ(LoadDeployMode(control), DeployMode::kPdStand);
  EXPECT_EQ(Press(control, ManualKey::kMotion, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kPdStand);
  EXPECT_EQ(Press(control, ManualKey::kTeleop, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kPdStand);

  control.pd_stand_ready.store(true, std::memory_order_release);
  EXPECT_EQ(Press(control, ManualKey::kMotion, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kMotion);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));

  SetMode(control, DeployMode::kPdStand, log);
  control.pd_stand_ready.store(true, std::memory_order_release);
  EXPECT_EQ(Press(control, ManualKey::kTeleop, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kTeleop);
}

TEST(A3ManualControl, GlobalSafetyKeysWorkFromActiveModes) {
  ManualControlState control;
  std::ostringstream log;

  SetMode(control, DeployMode::kMotion, log);
  control.motion_playing.store(true, std::memory_order_release);
  EXPECT_EQ(Press(control, ManualKey::kPassive, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kPassive);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));

  SetMode(control, DeployMode::kTeleop, log);
  control.teleop_input_paused.store(true, std::memory_order_release);
  EXPECT_EQ(Press(control, ManualKey::kPdStand, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kPdStand);
  EXPECT_FALSE(control.teleop_input_paused.load(std::memory_order_acquire));
}

TEST(A3ManualControl, MotionControlsOnlyAffectMotionMode) {
  ManualControlState control;
  std::ostringstream log;
  SetMode(control, DeployMode::kMotion, log);

  EXPECT_EQ(Press(control, ManualKey::kSpace, log),
            ManualKeyOutcome::kHandled);
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));
  EXPECT_EQ(Press(control, ManualKey::kSpace, log),
            ManualKeyOutcome::kHandled);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));

  EXPECT_EQ(Press(control, ManualKey::kRestartMotionPlay, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kMotion);
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));

  EXPECT_EQ(Press(control, ManualKey::kRestartMotionPause, log),
            ManualKeyOutcome::kHandled);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));

  control.selected_motion_index.store(0, std::memory_order_release);
  EXPECT_EQ(Press(control, ManualKey::kPrevMotion, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 2);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));
}

TEST(A3ManualControl, SelectingMotionWhileIdleOnlyUpdatesPendingPointer) {
  ManualControlState control;
  std::ostringstream log;
  SetMode(control, DeployMode::kMotion, log);
  control.motion_playing.store(false, std::memory_order_release);
  control.remote_motion_active.store(false, std::memory_order_release);
  const auto mode_epoch = control.epoch.load(std::memory_order_acquire);
  const auto command_epoch =
      control.motion_command_epoch.load(std::memory_order_acquire);

  EXPECT_EQ(Press(control, ManualKey::kNextMotion, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kMotion);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 1);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));
  EXPECT_FALSE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_EQ(control.epoch.load(std::memory_order_acquire), mode_epoch);
  EXPECT_EQ(control.motion_command_epoch.load(std::memory_order_acquire),
            command_epoch);
}

TEST(A3ManualControl, ArrowShortcutsRequirePdStandAndPlayFromStart) {
  ManualControlState control;
  ManualMotionShortcuts shortcuts;
  shortcuts.forward = 0;
  shortcuts.backward = 1;
  shortcuts.turn_left = 2;
  shortcuts.turn_right = 3;
  std::ostringstream log;
  control.selected_motion_index.store(2, std::memory_order_release);

  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kIdle);

  SetMode(control, DeployMode::kPdStand, log);
  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kPdStand);

  control.pd_stand_ready.store(true, std::memory_order_release);
  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kMotion);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 2);
  EXPECT_EQ(control.selected_remote_motion_index.load(
                std::memory_order_acquire),
            0);
  EXPECT_TRUE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));

  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionBackward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 2);
  EXPECT_EQ(control.selected_remote_motion_index.load(
                std::memory_order_acquire),
            1);
  EXPECT_TRUE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));

  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionTurnRight, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 2);
  EXPECT_EQ(control.selected_remote_motion_index.load(
                std::memory_order_acquire),
            3);
  EXPECT_TRUE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));
}

TEST(A3ManualControl, UnconfiguredArrowShortcutIsBlocked) {
  ManualControlState control;
  std::ostringstream log;
  SetMode(control, DeployMode::kMotion, log);

  EXPECT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(), log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 0);
  EXPECT_FALSE(control.remote_motion_active.load(std::memory_order_acquire));
}

TEST(A3ManualControl, NormalMotionControlsClearRemoteSource) {
  ManualControlState control;
  ManualMotionShortcuts shortcuts;
  shortcuts.forward = 0;
  std::ostringstream log;

  SetMode(control, DeployMode::kPdStand, log);
  control.pd_stand_ready.store(true, std::memory_order_release);
  ASSERT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  ASSERT_TRUE(control.remote_motion_active.load(std::memory_order_acquire));
  control.selected_motion_index.store(1, std::memory_order_release);

  EXPECT_EQ(HandleManualKey(control, ManualKey::kRestartMotionPlay, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  EXPECT_FALSE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 1);
  EXPECT_TRUE(control.motion_playing.load(std::memory_order_acquire));

  ASSERT_EQ(HandleManualKey(control, ManualKey::kMotionForward, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  ASSERT_TRUE(control.remote_motion_active.load(std::memory_order_acquire));
  EXPECT_EQ(HandleManualKey(control, ManualKey::kNextMotion, Motions(),
                            RemoteMotions(), log, &shortcuts),
            ManualKeyOutcome::kHandled);
  EXPECT_FALSE(control.remote_motion_active.load(std::memory_order_acquire));
}

TEST(A3ManualControl, TeleopSpacePausesInputAndMotionKeysAreBlocked) {
  ManualControlState control;
  std::ostringstream log;
  SetMode(control, DeployMode::kTeleop, log);
  control.motion_playing.store(false, std::memory_order_release);
  control.selected_motion_index.store(1, std::memory_order_release);

  EXPECT_EQ(Press(control, ManualKey::kRestartMotionPlay, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(Press(control, ManualKey::kPrevMotion, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(control.selected_motion_index.load(std::memory_order_acquire), 1);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));

  EXPECT_EQ(Press(control, ManualKey::kSpace, log),
            ManualKeyOutcome::kHandled);
  EXPECT_TRUE(control.teleop_input_paused.load(std::memory_order_acquire));
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));
  EXPECT_EQ(Press(control, ManualKey::kSpace, log),
            ManualKeyOutcome::kHandled);
  EXPECT_FALSE(control.teleop_input_paused.load(std::memory_order_acquire));

  EXPECT_EQ(Press(control, ManualKey::kMotion, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(LoadDeployMode(control), DeployMode::kMotion);
  EXPECT_FALSE(control.motion_playing.load(std::memory_order_acquire));
}

TEST(A3ManualControl, TeleopSourceKeysOnlyAffectTeleopMode) {
  ManualControlState control;
  std::ostringstream log;

  EXPECT_EQ(Press(control, ManualKey::kTeleopSourceSmpl, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(Press(control, ManualKey::kTeleopSourceA3Fast, log),
            ManualKeyOutcome::kBlocked);
  EXPECT_EQ(control.teleop_source.load(std::memory_order_acquire), 0);

  SetMode(control, DeployMode::kTeleop, log);
  const auto initial_epoch =
      control.teleop_command_epoch.load(std::memory_order_acquire);

  EXPECT_EQ(Press(control, ManualKey::kTeleopSourceSmpl, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.teleop_source.load(std::memory_order_acquire), 1);
  EXPECT_EQ(control.teleop_command_epoch.load(std::memory_order_acquire),
            initial_epoch + 1);

  EXPECT_EQ(Press(control, ManualKey::kTeleopSourceA3Fast, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.teleop_source.load(std::memory_order_acquire), 2);
  EXPECT_EQ(control.teleop_command_epoch.load(std::memory_order_acquire),
            initial_epoch + 2);

  EXPECT_EQ(Press(control, ManualKey::kTeleopSourceA3, log),
            ManualKeyOutcome::kHandled);
  EXPECT_EQ(control.teleop_source.load(std::memory_order_acquire), 0);
  EXPECT_EQ(control.teleop_command_epoch.load(std::memory_order_acquire),
            initial_epoch + 3);
}

TEST(A3ManualControl, ParseManualKeys) {
  EXPECT_EQ(a3_deploy::ParseManualKey('p'), ManualKey::kPassive);
  EXPECT_EQ(a3_deploy::ParseManualKey('s'), ManualKey::kPdStand);
  EXPECT_EQ(a3_deploy::ParseManualKey('m'), ManualKey::kMotion);
  EXPECT_EQ(a3_deploy::ParseManualKey('t'), ManualKey::kTeleop);
  EXPECT_EQ(a3_deploy::ParseManualKey(' '), ManualKey::kSpace);
  EXPECT_EQ(a3_deploy::ParseManualKey('r'), ManualKey::kRestartMotionPlay);
  EXPECT_EQ(a3_deploy::ParseManualKey('x'), ManualKey::kRestartMotionPause);
  EXPECT_EQ(a3_deploy::ParseManualKey('['), ManualKey::kPrevMotion);
  EXPECT_EQ(a3_deploy::ParseManualKey(']'), ManualKey::kNextMotion);
  EXPECT_EQ(a3_deploy::ParseManualKey('1'), ManualKey::kTeleopSourceA3);
  EXPECT_EQ(a3_deploy::ParseManualKey('2'), ManualKey::kTeleopSourceSmpl);
  EXPECT_EQ(a3_deploy::ParseManualKey('3'), ManualKey::kTeleopSourceA3Fast);
  EXPECT_EQ(a3_deploy::ParseManualKey('?'), ManualKey::kUnknown);
}
