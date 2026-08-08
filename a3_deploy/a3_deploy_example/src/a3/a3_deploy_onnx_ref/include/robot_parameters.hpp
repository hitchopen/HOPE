/**
 * @file robot_parameters.hpp
 * @brief Hardware-level constants, data structures, and joint indices for the
 *        Unitree A3 humanoid robot.
 *
 * This header defines:
 *  - DDS topic names for the Unitree SDK (low-level command / state channels).
 *  - Motor count and per-joint MotorCommand structure.
 *  - HeadingState – a compact struct bundling the captured IMU quaternion
 *    with a user-controlled delta-heading offset.
 *  - OperatorState – high-level start / stop / play flags.
 *  - Control mode enum (series vs. parallel ankle actuation).
 *  - PolicyJointIndex enum mapping symbolic joint names to hardware motor indices.
 *
 * All indices in this file use the **hardware / URDF ordering** (not the
 * IsaacLab training ordering – see `policy_parameters.hpp` for the mapping).
 */

#ifndef ROBOT_PARAMETERS_HPP
#define ROBOT_PARAMETERS_HPP

#include <array>

// ---------------------------------------------------------------------------
// Unitree SDK DDS topic names
// ---------------------------------------------------------------------------
static const std::string HG_CMD_TOPIC = "rt/lowcmd";       ///< Low-level motor command topic.
static const std::string HG_IMU_TORSO = "rt/secondary_imu";///< Secondary (torso) IMU topic.
static const std::string HG_STATE_TOPIC = "rt/lowstate";    ///< Low-level motor / sensor state topic.

/// Total number of actuated joints on the A3 (29-DOF configuration).
const int A3_POLICY_DOF = 29;

/**
 * @brief Per-joint motor command sent to the low-level controller.
 *
 * Each field is an array of size A3_POLICY_DOF (29), indexed by hardware joint
 * index (see PolicyJointIndex).
 */
struct MotorCommand {
    std::array<float, A3_POLICY_DOF> q_target = {};   ///< Target position (rad).
    std::array<float, A3_POLICY_DOF> dq_target = {};  ///< Target velocity (rad/s).
    std::array<float, A3_POLICY_DOF> kp = {};          ///< Position gain (Nm/rad).
    std::array<float, A3_POLICY_DOF> kd = {};          ///< Velocity gain (Nm·s/rad).
    std::array<float, A3_POLICY_DOF> tau_ff = {};      ///< Feed-forward torque (Nm).
};

/**
 * @brief Bundled heading state for thread-safe access via DataBuffer.
 *
 * Captures both the initial IMU base quaternion (set when heading is
 * reinitialised) and a user-adjustable delta heading offset (adjusted via
 * keyboard Q/E or D-pad).
 */
struct HeadingState {
    std::array<double, 4> init_base_quat;  ///< Captured IMU base quaternion (w,x,y,z) at init.
    double delta_heading;                   ///< Cumulative heading offset (radians).
    
    HeadingState(const std::array<double, 4>& quat = {1.0, 0.0, 0.0, 0.0}, double delta = 0.0)
        : init_base_quat(quat), delta_heading(delta) {}
};

/**
 * @brief High-level operator signals (set by input interfaces, read by control loop).
 */
struct OperatorState {
  bool stop = false;   ///< Emergency stop requested.
  bool start = false;  ///< Control-system start requested.
  bool play = false;   ///< Motion playback active.
};

/**
 * @brief Ankle actuation mode.
 *
 * The A3's ankle uses a coupled 2-DOF mechanism that can be controlled
 * in series (pitch/roll) or parallel (A/B motor) modes.
 */
enum class Mode {
  PR = 0, ///< Series control for Pitch / Roll joints.
  AB = 1  ///< Parallel control for A / B motors.
};

/**
 * @brief Symbolic names for A3 hardware joint indices (0–28).
 *
 * Ankle joints have dual names because they can be addressed in either
 * series (Pitch/Roll) or parallel (A/B) mode.  Joints marked "INVALID"
 * are not present on the 23-DOF or waist-locked 29-DOF variants.
 */
enum PolicyJointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13, // NOTE INVALID for a3 23dof/29dof with waist locked
  WaistA = 13, // NOTE INVALID for a3 23dof/29dof with waist locked
  WaistPitch = 14, // NOTE INVALID for a3 23dof/29dof with waist locked
  WaistB = 14, // NOTE INVALID for a3 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20, // NOTE INVALID for a3 23dof
  LeftWristYaw = 21, // NOTE INVALID for a3 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27, // NOTE INVALID for a3 23dof
  RightWristYaw = 28 // NOTE INVALID for a3 23dof
};

#endif // ROBOT_PARAMETERS_HPP
