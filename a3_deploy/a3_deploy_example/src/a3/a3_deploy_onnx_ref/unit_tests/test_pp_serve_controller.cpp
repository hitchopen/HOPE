#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <chrono>
#include <iterator>
#include <thread>
#include <vector>

#include "a3_pingpong/pp_serve_controller.hpp"
#include "a3_pingpong/pp_sha256.hpp"

namespace a3_pingpong {
namespace {

std::filesystem::path A3ExampleRoot() {
  return std::filesystem::path(__FILE__).parent_path()  // unit_tests
      .parent_path()                                    // a3_deploy_onnx_ref
      .parent_path()                                    // a3
      .parent_path()                                    // src
      .parent_path();                                   // a3_deploy_example
}

Eigen::VectorXd DefaultSource() {
  Eigen::VectorXd q(31);
  q << -0.131, -0.131, 0.0, 0.006, -0.006, 0.0,
      -0.035, 0.035, 0.0, 0.247, 0.247, 0.0,
      0.3, 0.3, -0.120, -0.120, 0.0, 0.12, -0.12,
      -0.008, 0.008, 0.0, 0.0, 0.8, 0.8, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0;
  return q;
}

PpServeClip LoadFixedClip() {
  const auto root = A3ExampleRoot();
  return PpServeClip::Load(
      (root / "assets/a3_runtime/motions/pp_serve_v1_fixed.csv").string(),
      (root / "assets/a3_runtime/motions/"
              "pp_serve_v1_fixed.manifest.json")
          .string(),
      DefaultSource());
}

struct ServeTestInputs {
  std::shared_ptr<PpBasePoseInput> base =
      std::make_shared<PpBasePoseInput>();
  std::shared_ptr<PpBallStateInput> ball =
      std::make_shared<PpBallStateInput>();

  void PublishBase() {
    base->SetFromFlat(
        {1.0, 1.0, -0.50, -0.7625, 1.07,
         1.0, 0.0, 0.0, 0.0});
  }

  void PublishBall(double vx = 0.10, double vy = 0.02,
                   double vz = 1.20) {
    ball->SetFromFlat(
        {1.0, 1.0, -0.06, -0.6025, 1.10, vx, vy, vz,
         1.0, 0.0, 31.0});
  }

  void Publish(double vx = 0.10, double vy = 0.02,
               double vz = 1.20) {
    PublishBase();
    PublishBall(vx, vy, vz);
  }
};

robot_io::RobotState ReadyState(const Eigen::VectorXd& default_sdk) {
  robot_io::RobotState state;
  state.timestamp_ns = 1;
  state.sync_complete = true;
  state.sync_aligned = true;
  state.q = default_sdk;
  state.dq = Eigen::VectorXd::Zero(31);
  state.tau_est = Eigen::VectorXd::Zero(31);
  state.imu_quat_wxyz = Eigen::Vector4d(1, 0, 0, 0);
  state.imu_gyro.setZero();
  return state;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> StaticHandoffGains() {
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(31);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(31);
  for (int policy = 0; policy < robot_io::kA3PolicyDof; ++policy) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[policy];
    kp[sdk] = a3_pd_stand_kps[policy];
    kd[sdk] = a3_pd_stand_kds[policy];
  }
  kp[3] = kp[4] = kServeHeadKp;
  kd[3] = kd[4] = kServeHeadKd;
  return {kp, kd};
}

TEST(PpSha256, MatchesKnownVector) {
  EXPECT_EQ(
      PpSha256::String("abc"),
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(PpServeClip, LoadsQualifiedPalmOnlyArtifact) {
  const PpServeClip fixed = LoadFixedClip();
  EXPECT_EQ(fixed.size(), 333u);
  EXPECT_EQ(fixed.events().ready_pose, 59);
  EXPECT_EQ(fixed.events().release, 167);
  EXPECT_EQ(fixed.events().contact, 189);
  EXPECT_EQ(fixed.events().handoff_complete, 332);
  EXPECT_LE(fixed.measured_max_step(), 0.0800005);
  EXPECT_LE(fixed.measured_max_second(), 0.1000005);
  EXPECT_LE(fixed.measured_max_dq(), 4.0000005);
  EXPECT_EQ(fixed.strike_selection().name, "fixed");
  EXPECT_EQ(
      fixed.strike_selection().selector,
      "fresh_ball_envelope_fixed_clip");
  EXPECT_DOUBLE_EQ(
      fixed.strike_selection().right_retract_duration_ticks, 2.0);
  EXPECT_EQ(fixed.strike_selection().right_retract_lead_ticks, 1);
  EXPECT_EQ(
      fixed.strike_selection().right_retract_anchor_frame,
      kServeBranchSharedPrefixEndFrame);
  EXPECT_EQ(fixed.right_safe_hold_frame(), 145);
  EXPECT_EQ(
      fixed.strike_selection().shared_prefix_end_frame,
      kServeBranchSharedPrefixEndFrame);
  for (int frame = kServeLeftWristDqCapStartFrame;
       frame < fixed.events().handoff_begin; ++frame) {
    for (const int source_joint : {27, 29}) {
      EXPECT_LE(
          std::abs(fixed.frame(frame).dq_source[source_joint]),
          kServeWristDqReferenceCapRadS + 1.0e-12);
    }
  }
  for (int frame = kServeRightWristDqCapStartFrame;
       frame < fixed.events().handoff_begin; ++frame) {
    for (const int source_joint : {28, 30}) {
      EXPECT_LE(
          std::abs(fixed.frame(frame).dq_source[source_joint]),
          kServeWristDqReferenceCapRadS + 1.0e-12);
    }
  }
}

TEST(PpServeClip, RejectsManifestThatRelabelsDorsalSurfaceAsPalm) {
  const auto root = A3ExampleRoot();
  const auto csv =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.csv";
  const auto manifest =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.manifest.json";
  const auto temp = std::filesystem::temp_directory_path() /
                    "pp_serve_v1_wrong_palm_surface_test.json";
  std::ifstream input(manifest);
  ASSERT_TRUE(input.good());
  std::string contents(
      (std::istreambuf_iterator<char>(input)),
      std::istreambuf_iterator<char>());
  const std::string confirmed = "B_confirmed_minus_obb_z";
  const std::size_t position = contents.find(confirmed);
  ASSERT_NE(position, std::string::npos);
  contents.replace(position, confirmed.size(), "A_dorsal_surface");
  {
    std::ofstream output(temp);
    output << contents;
  }
  EXPECT_THROW(
      PpServeClip::Load(csv.string(), temp.string(), DefaultSource()),
      std::runtime_error);
  std::error_code error;
  std::filesystem::remove(temp, error);
}

TEST(PpServeClip, RejectsCsvWhoseShaDoesNotMatchManifest) {
  const auto root = A3ExampleRoot();
  const auto original =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.csv";
  const auto manifest =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.manifest.json";
  const auto temp = std::filesystem::temp_directory_path() /
                    "pp_serve_v1_tampered_test.csv";
  std::filesystem::copy_file(
      original, temp, std::filesystem::copy_options::overwrite_existing);
  {
    std::ofstream stream(temp, std::ios::app);
    stream << "\n";
  }
  EXPECT_THROW(
      PpServeClip::Load(temp.string(), manifest.string(), DefaultSource()),
      std::runtime_error);
  std::error_code error;
  std::filesystem::remove(temp, error);
}

TEST(PpServeClip, RejectsExpectedDropSpeedThatDiffersFromControllerContract) {
  const auto root = A3ExampleRoot();
  const auto csv =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.csv";
  const auto manifest =
      root / "assets/a3_runtime/motions/pp_serve_v1_fixed.manifest.json";
  const auto temp = std::filesystem::temp_directory_path() /
                    "pp_serve_v1_wrong_drop_speed_test.json";
  std::ifstream input(manifest);
  ASSERT_TRUE(input.good());
  std::string contents(
      (std::istreambuf_iterator<char>(input)),
      std::istreambuf_iterator<char>());
  const std::string qualified =
      "\"palm_drop_min_vertical_velocity_mps\": -2.0";
  const std::size_t position = contents.rfind(qualified);
  ASSERT_NE(position, std::string::npos);
  contents.replace(
      position, qualified.size(),
      "\"palm_drop_min_vertical_velocity_mps\": -2.1");
  {
    std::ofstream output(temp);
    output << contents;
  }
  EXPECT_THROW(
      PpServeClip::Load(csv.string(), temp.string(), DefaultSource()),
      std::runtime_error);
  std::error_code error;
  std::filesystem::remove(temp, error);
}

TEST(PpBallStateInput, StaleAndMalformedPacketsFailClosed) {
  PpBallStateInput input;
  const std::vector<double> valid{
      1.0, 1.0, 0.44, 0.16, 1.10, 0.10, 0.02, 1.20,
      1.0, 0.0, 31.0};
  input.SetFromFlat(valid);
  PpBallSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.050));
  EXPECT_EQ(sample.estimator_samples, 31);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_FALSE(input.Latest(sample, 0.050));

  input.SetFromFlat(valid);
  ASSERT_TRUE(input.Latest(sample, 0.050));
  input.SetFromFlat({1.0, 1.0, 0.44});
  EXPECT_FALSE(input.Latest(sample, 0.050));
}

TEST(PpServeController, FullClipEndsAtExactDefaultAndRequestsHandoff) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 2;
  config.handoff_dwell_ticks = 2;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool handed_off = false;
  bool ball_confirmed = false;
  bool saw_nonzero_velocity_reference = false;
  double max_kp_step = 0.0;
  double max_kd_step = 0.0;
  Eigen::VectorXd previous_kp;
  Eigen::VectorXd previous_kd;
  std::size_t await_frame = 0;
  for (std::uint64_t tick = 0; tick < 500; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    ASSERT_EQ(command.q_des.size(), 31);
    ASSERT_EQ(command.dq_des.size(), 31);
    saw_nonzero_velocity_reference |=
        command.dq_des.cwiseAbs().maxCoeff() > 1.0e-6;
    if (previous_kp.size() == 31) {
      max_kp_step = std::max(
          max_kp_step,
          (command.kp - previous_kp).cwiseAbs().maxCoeff());
      max_kd_step = std::max(
          max_kd_step,
          (command.kd - previous_kd).cwiseAbs().maxCoeff());
    }
    previous_kp = command.kp;
    previous_kd = command.kd;
    state.q = command.q_des;  // perfect-tracking unit plant
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall) {
      EXPECT_FALSE(ball_confirmed);
      await_frame = controller.TakeDiag().frame;
      controller.ConfirmBallOnPalm();
      ball_confirmed = true;
    }
    if (controller.ConsumeHandoffRequest()) {
      handed_off = true;
      break;
    }
  }
  EXPECT_TRUE(ball_confirmed);
  EXPECT_TRUE(saw_nonzero_velocity_reference);
  EXPECT_EQ(await_frame,
            static_cast<std::size_t>(controller.clip().events().toss_commit));
  EXPECT_TRUE(handed_off);
  EXPECT_EQ(controller.state(), ServeControllerState::kComplete);
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 0.0));
  const auto [static_kp, static_kd] = StaticHandoffGains();
  EXPECT_TRUE(command.kp.isApprox(static_kp, 1.0e-12));
  EXPECT_TRUE(command.kd.isApprox(static_kd, 1.0e-12));
  EXPECT_LT(max_kp_step, 15.0);
  EXPECT_LT(max_kd_step, 0.20);
}

TEST(PpServeController, UsesExactlyQualifiedArmGains) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, {}, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();
  inputs.Publish();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));

  for (int policy = 3; policy <= 9; ++policy) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[policy];
    const double proximal_boost = policy <= 6
                                      ? kServeLeftProximalArmKpBoost
                                      : 1.0;
    const double proximal_damping_boost =
        policy <= 6 ? kServeLeftProximalArmKdBoost : 1.0;
    EXPECT_NEAR(
        command.kp[sdk],
        a3_kps[policy] * kServeArmKpScale * proximal_boost,
        1.0e-12);
    EXPECT_NEAR(
        command.kd[sdk],
        a3_kds[policy] * kServeArmKdScale *
            proximal_damping_boost,
        1.0e-12);
  }
  for (int policy = 10; policy <= 16; ++policy) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[policy];
    EXPECT_NEAR(
        command.kp[sdk],
        a3_kps[policy] * kServeArmKpScale *
            kServeRightArmKpBoost,
        1.0e-12);
    EXPECT_NEAR(
        command.kd[sdk],
        a3_kds[policy] * kServeArmKdScale *
            kServeRightArmKdBoost,
        1.0e-12);
  }
}

TEST(PpServeController, LowInEnvelopeVxSelectsFixedClipExactlyOnce) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool confirmed = false;
  ServeControllerDiag selected;
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    inputs.Publish(0.100);
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !confirmed) {
      controller.ConfirmBallOnPalm();
      confirmed = true;
    }
    const auto diag = controller.TakeDiag();
    if (diag.branch_selected) {
      selected = diag;
      break;
    }
  }
  ASSERT_TRUE(confirmed);
  ASSERT_TRUE(selected.branch_selected);
  EXPECT_EQ(selected.selected_branch, "fixed");
  EXPECT_EQ(
      selected.branch_reason, "fresh_ball_envelope_fixed_clip");
  EXPECT_EQ(selected.ball_estimator_samples, 31);
  EXPECT_NEAR(selected.ball_vx_mps, 0.100, 1.0e-12);
}

TEST(PpServeController, HighInEnvelopeVxSelectsSameFixedClip) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool confirmed = false;
  ServeControllerDiag selected;
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    inputs.Publish(0.200);
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !confirmed) {
      controller.ConfirmBallOnPalm();
      confirmed = true;
    }
    const auto diag = controller.TakeDiag();
    if (diag.branch_selected) {
      selected = diag;
      break;
    }
  }
  ASSERT_TRUE(confirmed);
  ASSERT_TRUE(selected.branch_selected);
  EXPECT_EQ(selected.selected_branch, "fixed");
  EXPECT_EQ(
      selected.branch_reason, "fresh_ball_envelope_fixed_clip");
  EXPECT_NEAR(selected.ball_vx_mps, 0.200, 1.0e-12);
}

TEST(PpServeController, GrossVelocityFinishesTossOnlyWithoutHandoff) {
  PpServeClip clip = LoadFixedClip();
  const int recovery_start = clip.events().recovery_start;
  const Eigen::VectorXd right_hold =
      clip.frame(static_cast<std::size_t>(clip.right_safe_hold_frame())).q_sdk;
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.handoff_dwell_ticks = 2;
  config.abort_return_ticks = 20;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool confirmed = false;
  bool saw_toss_only = false;
  bool saw_abort_return = false;
  std::string reason;
  for (std::uint64_t tick = 0; tick < 500; ++tick) {
    inputs.Publish(1.0);
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !confirmed) {
      controller.ConfirmBallOnPalm();
      confirmed = true;
    }
    const auto diag = controller.TakeDiag();
    if (!diag.branch_reason.empty()) reason = diag.branch_reason;
    saw_abort_return |=
        controller.state() == ServeControllerState::kAbortReturn;
    if (diag.toss_only_abort) {
      saw_toss_only = true;
      if (static_cast<int>(diag.frame) <= recovery_start) {
        for (int sdk = 12; sdk <= 18; ++sdk) {
          EXPECT_NEAR(command.q_des[sdk], right_hold[sdk], 1.0e-12);
        }
      }
    }
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  ASSERT_TRUE(confirmed);
  EXPECT_TRUE(saw_toss_only);
  EXPECT_FALSE(saw_abort_return);
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_EQ(reason, "ball_estimate_out_of_prevalidated_envelope");
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

TEST(PpServeController, LateralVelocityOutsideEnvelopeUsesTossOnlyReturn) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.handoff_dwell_ticks = 2;
  config.abort_return_ticks = 20;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool confirmed = false;
  bool saw_abort_return = false;
  bool saw_toss_only = false;
  std::string branch_reason;
  for (std::uint64_t tick = 0; tick < 500; ++tick) {
    inputs.Publish(0.180, 0.30);
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !confirmed) {
      controller.ConfirmBallOnPalm();
      confirmed = true;
    }
    const auto diag = controller.TakeDiag();
    saw_abort_return |=
        controller.state() == ServeControllerState::kAbortReturn;
    saw_toss_only |= diag.toss_only_abort;
    if (!diag.branch_reason.empty()) branch_reason = diag.branch_reason;
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  ASSERT_TRUE(confirmed);
  EXPECT_TRUE(saw_toss_only);
  EXPECT_FALSE(saw_abort_return);
  EXPECT_EQ(
      branch_reason, "ball_estimate_out_of_prevalidated_envelope");
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

TEST(PpServeController, FrozenPreConfirmBallStateUsesTossOnlyReturn) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.abort_return_ticks = 20;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  bool confirmed = false;
  bool saw_toss_only = false;
  bool saw_abort_return = false;
  std::string branch_reason;
  for (std::uint64_t tick = 0; tick < 500; ++tick) {
    inputs.PublishBase();
    if (!confirmed) inputs.PublishBall(0.10);
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !confirmed) {
      controller.ConfirmBallOnPalm();
      confirmed = true;
    }
    const auto diag = controller.TakeDiag();
    if (!diag.branch_reason.empty()) {
      branch_reason = diag.branch_reason;
    }
    saw_toss_only |= diag.toss_only_abort;
    saw_abort_return |=
        controller.state() == ServeControllerState::kAbortReturn;
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  ASSERT_TRUE(confirmed);
  EXPECT_TRUE(saw_toss_only);
  EXPECT_FALSE(saw_abort_return);
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_EQ(branch_reason, "no_post_confirm_ball_update");
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
}

TEST(PpServeController, ApproachesDefaultFromMeasuredPoseWithoutEntrySnap) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.approach_ticks = 8;
  config.preflight_dwell_ticks = 1;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  state.q[5] += 0.10;
  const Eigen::VectorXd entry_q = state.q;
  robot_io::RobotCommand command;
  controller.Start();

  inputs.Publish();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  EXPECT_TRUE(command.q_des.isApprox(entry_q, 0.0));
  Eigen::VectorXd previous = command.q_des;
  for (std::uint64_t tick = 1; tick < 10; ++tick) {
    inputs.Publish();
    state.q = command.q_des;
    state.dq.setZero();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    EXPECT_LT(
        (command.q_des - previous).cwiseAbs().maxCoeff(), 0.05);
    previous = command.q_des;
  }
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

TEST(PpServeController, HeadingDriftBlocksPreflightDwell) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.approach_ticks = 2;
  config.preflight_dwell_ticks = 2;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  inputs.Publish();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  const double half_yaw = 0.10;  // total yaw=0.20 rad > 0.15 threshold
  state.imu_quat_wxyz =
      Eigen::Vector4d(std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw));
  for (std::uint64_t tick = 1; tick < 10; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
  }
  EXPECT_EQ(
      controller.state(), ServeControllerState::kPreflightReady);
  EXPECT_FALSE(controller.TakeDiag().local_ready);
}

TEST(PpServeController, RequiredMocapNeedsFreshVelocityBeforeReady) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.approach_ticks = 2;
  config.preflight_dwell_ticks = 1;
  config.require_external_base = true;
  auto base_input = std::make_shared<PpBasePoseInput>();
  auto ball_input = std::make_shared<PpBallStateInput>();
  PpServeController controller(
      std::move(clip), default_sdk, config, base_input, ball_input);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  EXPECT_FALSE(controller.TakeDiag().base_valid);
  const std::vector<double> base_packet{
      1.0, 1.0, -0.50, -0.7625, 1.07,
      1.0, 0.0, 0.0, 0.0};
  base_input->SetFromFlat(base_packet);
  ASSERT_TRUE(controller.ComputeCommand(1, state, command));
  EXPECT_FALSE(controller.TakeDiag().base_valid);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  base_input->SetFromFlat(base_packet);
  ASSERT_TRUE(controller.ComputeCommand(2, state, command));
  EXPECT_TRUE(controller.TakeDiag().base_valid);
  ASSERT_TRUE(controller.ComputeCommand(3, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kPlaying);
}

TEST(PpServeController, XmlDefaultOriginCannotArmGate3Serve) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.approach_ticks = 2;
  config.preflight_dwell_ticks = 1;
  config.require_external_base = true;
  auto base_input = std::make_shared<PpBasePoseInput>();
  auto ball_input = std::make_shared<PpBallStateInput>();
  PpServeController controller(
      std::move(clip), default_sdk, config, base_input, ball_input);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  const std::vector<double> wrong_default_origin{
      1.0, 1.0, 0.0, 0.0, 1.07, 1.0, 0.0, 0.0, 0.0};
  base_input->SetFromFlat(wrong_default_origin);
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  base_input->SetFromFlat(wrong_default_origin);
  ASSERT_TRUE(controller.ComputeCommand(1, state, command));
  const ServeControllerDiag diag = controller.TakeDiag();
  EXPECT_TRUE(diag.base_valid);
  EXPECT_GT(diag.base_position_error_m, 0.90);
  EXPECT_FALSE(diag.local_ready);
  EXPECT_EQ(
      controller.state(), ServeControllerState::kPreflightReady);
}

TEST(PpServeController, HoldsPreTossPoseUntilRigidPalmBallConfirmation) {
  PpServeClip clip = LoadFixedClip();
  const int toss_commit = clip.events().toss_commit;
  const Eigen::VectorXd expected_hold =
      clip.frame(static_cast<std::size_t>(toss_commit - 1)).q_sdk;
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  for (std::uint64_t tick = 0; tick < 250; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall) break;
  }
  ASSERT_EQ(controller.state(), ServeControllerState::kAwaitBall);
  EXPECT_EQ(controller.TakeDiag().frame,
            static_cast<std::size_t>(toss_commit));
  for (std::uint64_t tick = 250; tick < 270; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    EXPECT_TRUE(command.q_des.isApprox(expected_hold, 0.0));
    EXPECT_TRUE(command.dq_des.isZero(0.0));
    state.q = command.q_des;
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kAwaitBall);
}

TEST(PpServeController, PreCommitAbortReturnsSmoothlyWithoutPolicyHandoff) {
  PpServeClip clip = LoadFixedClip();
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.handoff_dwell_ticks = 2;
  config.abort_return_ticks = 20;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  for (std::uint64_t tick = 0; tick < 15; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
  }
  controller.RequestAbort();
  for (std::uint64_t tick = 15; tick < 100; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

TEST(PpServeController,
     AbortAfterTossBeforeBranchUsesSeparatedTossOnlyReturn) {
  PpServeClip clip = LoadFixedClip();
  const int toss_commit = clip.events().toss_commit;
  const Eigen::VectorXd safe_hold =
      clip.frame(
              static_cast<std::size_t>(clip.right_safe_hold_frame()))
          .q_sdk;
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.handoff_dwell_ticks = 2;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  std::uint64_t tick = 0;
  bool ball_confirmed = false;
  while (tick < 300) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick++, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !ball_confirmed) {
      controller.ConfirmBallOnPalm();
      ball_confirmed = true;
    }
    const ServeControllerDiag diag = controller.TakeDiag();
    if (ball_confirmed &&
        diag.state == ServeControllerState::kPlaying &&
        static_cast<int>(diag.frame) >= toss_commit + 5) {
      ASSERT_LT(
          static_cast<int>(diag.frame), kServeBranchSelectionFrame);
      break;
    }
  }
  ASSERT_TRUE(ball_confirmed);

  controller.RequestAbort();
  inputs.Publish();
  ASSERT_TRUE(controller.ComputeCommand(tick++, state, command));
  const ServeControllerDiag abort_diag = controller.TakeDiag();
  ASSERT_TRUE(abort_diag.toss_only_abort);
  EXPECT_FALSE(abort_diag.branch_selected);
  EXPECT_EQ(
      abort_diag.branch_reason, "operator_abort_after_toss_commit");
  for (int sdk = 12; sdk <= 18; ++sdk) {
    EXPECT_DOUBLE_EQ(command.q_des[sdk], safe_hold[sdk]);
  }
  state.q = command.q_des;
  state.dq.setZero();

  for (; tick < 600; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

TEST(PpServeController, PostCommitAbortFinishesFollowThroughButNeverHandsOff) {
  PpServeClip clip = LoadFixedClip();
  const int swing_commit = clip.events().swing_commit;
  const Eigen::VectorXd default_sdk =
      to_sdk_order(DefaultSource(), clip.src_to_sdk());
  ServeControllerConfig config;
  config.preflight_dwell_ticks = 1;
  config.handoff_dwell_ticks = 2;
  ServeTestInputs inputs;
  PpServeController controller(
      std::move(clip), default_sdk, config, inputs.base, inputs.ball);
  robot_io::RobotState state = ReadyState(default_sdk);
  robot_io::RobotCommand command;
  controller.Start();

  std::uint64_t tick = 0;
  bool ball_confirmed = false;
  while (static_cast<int>(controller.TakeDiag().frame) <= swing_commit + 2 &&
         tick < 250) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick++, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAwaitBall &&
        !ball_confirmed) {
      controller.ConfirmBallOnPalm();
      ball_confirmed = true;
    }
  }
  ASSERT_TRUE(ball_confirmed);
  controller.RequestAbort();
  for (; tick < 500; ++tick) {
    inputs.Publish();
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    state.q = command.q_des;
    state.dq.setZero();
    if (controller.state() == ServeControllerState::kAborted) break;
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  EXPECT_FALSE(controller.ConsumeHandoffRequest());
  EXPECT_TRUE(command.q_des.isApprox(default_sdk, 1.0e-12));
}

}  // namespace
}  // namespace a3_pingpong
