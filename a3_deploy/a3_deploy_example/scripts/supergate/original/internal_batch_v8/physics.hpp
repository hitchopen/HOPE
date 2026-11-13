#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include "mujoco/mujoco.h"
#include "mujoco_sim_module/common/gate3_ball_layout.h"
#include "mujoco_sim_module/common/gate3_ball_contact_model.h"
namespace gate3 = aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball;
struct BallPhysicsRuntime {
 const mjModel* m_; mjData* d_;
 int gate3_racket_geom_id_, gate3_table_geom_id_, gate3_net_geom_id_, gate3_racket_site_id_;
 double gate3_ball_drag_k_=.12453915600992894,gate3_ball_magnus_k_=.00399279130564425;
 double gate3_ball_restitution_v_=.9373319241133231,gate3_ball_tangential_gain_=.41642144985036184,gate3_ball_friction_cap_mu_=.218520906499864;
 std::array<std::array<double,6>,gate3::kFlightSlots> flight_velocities_{};
 std::array<std::uint64_t,gate3::kFlightSlots> flight_shot_ids_{};
 std::array<bool,gate3::kFlightSlots> flight_racket_prev_{},flight_table_prev_{},flight_net_prev_{};
 std::array<gate3::TableRestTracker,gate3::kFlightSlots> flight_rest_{};
 BallPhysicsRuntime(const mjModel* m,mjData* d):m_(m),d_(d) {
  gate3_racket_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kRacketGeomName);
  gate3_table_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kTableGeomName);
  gate3_net_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kNetGeomName);
  gate3_racket_site_id_=mj_name2id(m,mjOBJ_SITE,"right_racket");
 }
 void ApplyGate3BallDrag(int slot);
 void UpdateGate3BallContacts(int slot);
};
void BallPhysicsRuntime::ApplyGate3BallDrag(int slot) {

  const int joint = gate3::SlotJoint(m_, slot);
  const int gate3_ball_body_id_ = m_->jnt_bodyid[joint];
  const int gate3_ball_qpos_addr_ = m_->jnt_qposadr[joint];
  const int gate3_ball_dof_addr_ = m_->jnt_dofadr[joint];
  const int gate3_ball_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::SlotName(gate3::kGeomName, slot).c_str());
  auto* telemetry = d_->userdata + slot * gate3::kRequiredUserData;
  auto& gate3_ball_pre_step_velocity_ = flight_velocities_[slot];
  auto& gate3_last_shot_id_ = flight_shot_ids_[slot];
  auto& gate3_racket_contact_prev_ = flight_racket_prev_[slot];
  auto& gate3_table_contact_prev_ = flight_table_prev_[slot];
  auto& gate3_net_contact_prev_ = flight_net_prev_[slot];
  // Natural endpoints: floor crossing or two seconds of bounded table rest.
  // A launch schedule never retires a moving flight.
  if (telemetry[gate3::kActive] > 0.5 &&
      ((d_->qpos[gate3_ball_qpos_addr_ + 2] < gate3::kBallRadius &&
        d_->qvel[gate3_ball_dof_addr_ + 2] < 0.0) ||
       flight_rest_[slot].Update(telemetry[gate3::kShotId], d_->time,
          d_->qpos + gate3_ball_qpos_addr_, d_->qvel + gate3_ball_dof_addr_,
          telemetry[gate3::kTableContactCount]))) {
    telemetry[gate3::kActive] = 0.0;
  }
  if (gate3_ball_body_id_ < 0 || gate3_ball_dof_addr_ < 0) return;
  mjtNum* wrench = d_->xfrc_applied + 6 * gate3_ball_body_id_;
  wrench[0] = wrench[1] = wrench[2] = 0.0;
  if (telemetry[gate3::kActive] <= 0.5) return;

  const mjtNum* velocity = d_->qvel + gate3_ball_dof_addr_;
  const auto world_spin = gate3::WorldSpin(d_, gate3_ball_qpos_addr_, gate3_ball_dof_addr_);
  gate3_ball_pre_step_velocity_ = {
      static_cast<double>(velocity[0]),
      static_cast<double>(velocity[1]),
      static_cast<double>(velocity[2]),
      world_spin[0], world_spin[1], world_spin[2]};
  const gate3::Vec3 linear_velocity{
      gate3_ball_pre_step_velocity_[0],
      gate3_ball_pre_step_velocity_[1],
      gate3_ball_pre_step_velocity_[2]};
  const gate3::Vec3 angular_velocity{
      gate3_ball_pre_step_velocity_[3],
      gate3_ball_pre_step_velocity_[4],
      gate3_ball_pre_step_velocity_[5]};
  const auto acceleration = gate3::FlightAcceleration(
      linear_velocity, angular_velocity, gate3_ball_drag_k_, gate3_ball_magnus_k_);
  const double mass = m_->body_mass[gate3_ball_body_id_];
  wrench[0] = mass * acceleration[0];
  wrench[1] = mass * acceleration[1];
  wrench[2] = mass * acceleration[2];
}

void BallPhysicsRuntime::UpdateGate3BallContacts(int slot) {

  const int joint = gate3::SlotJoint(m_, slot);
  const int gate3_ball_body_id_ = m_->jnt_bodyid[joint];
  const int gate3_ball_qpos_addr_ = m_->jnt_qposadr[joint];
  const int gate3_ball_dof_addr_ = m_->jnt_dofadr[joint];
  const int gate3_ball_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::SlotName(gate3::kGeomName, slot).c_str());
  auto* telemetry = d_->userdata + slot * gate3::kRequiredUserData;
  auto& gate3_ball_pre_step_velocity_ = flight_velocities_[slot];
  auto& gate3_last_shot_id_ = flight_shot_ids_[slot];
  auto& gate3_racket_contact_prev_ = flight_racket_prev_[slot];
  auto& gate3_table_contact_prev_ = flight_table_prev_[slot];
  auto& gate3_net_contact_prev_ = flight_net_prev_[slot];
  if (telemetry[gate3::kActive] <= 0.5) return;
  if (gate3_ball_geom_id_ < 0) return;

  const auto shot_id = static_cast<std::uint64_t>(
      std::max<mjtNum>(0.0, telemetry[gate3::kShotId]));
  if (shot_id != gate3_last_shot_id_) {
    gate3_last_shot_id_ = shot_id;
    gate3_racket_contact_prev_ = slot > 0 && (static_cast<int>(telemetry[gate3::kContactBits]) & gate3::kContactRacket);
    gate3_table_contact_prev_ = slot > 0 && (static_cast<int>(telemetry[gate3::kContactBits]) & gate3::kContactTable);
    gate3_net_contact_prev_ = slot > 0 && (static_cast<int>(telemetry[gate3::kContactBits]) & gate3::kContactNet);
  }

  bool racket = false;
  bool table = false;
  bool net = false;
  double racket_normal_force = 0.0;
  gate3::Vec3 racket_contact_position{};
  bool racket_contact_position_valid = false;
  for (int contact_id = 0; contact_id < d_->ncon; ++contact_id) {
    const auto& contact = d_->contact[contact_id];
    const int other =
        contact.geom1 == gate3_ball_geom_id_
            ? contact.geom2
            : contact.geom2 == gate3_ball_geom_id_ ? contact.geom1 : -1;
    if (other < 0) continue;
    if (other == gate3_racket_geom_id_) {
      racket = true;
      if (!racket_contact_position_valid) {
        racket_contact_position = {
            static_cast<double>(contact.pos[0]),
            static_cast<double>(contact.pos[1]),
            static_cast<double>(contact.pos[2])};
        racket_contact_position_valid = true;
      }
      std::array<mjtNum, 6> wrench{};
      mj_contactForce(m_, d_, contact_id, wrench.data());
      racket_normal_force += std::abs(static_cast<double>(wrench[0]));
    } else if (other == gate3_table_geom_id_) {
      table = true;
    } else if (other == gate3_net_geom_id_) {
      net = true;
    }
  }

  unsigned char bits = 0;
  if (racket) bits |= gate3::kContactRacket;
  if (table) bits |= gate3::kContactTable;
  if (net) bits |= gate3::kContactNet;
  telemetry[gate3::kContactBits] = static_cast<mjtNum>(bits);
  // State is published at 250 Hz, while contacts are sampled at 1 kHz.
  // Latch the per-shot peak so diagnostic force evidence cannot disappear
  // between publisher ticks. ResetTelemetry clears it at the next launch.
  telemetry[gate3::kRacketNormalForce] = std::max<mjtNum>(
      telemetry[gate3::kRacketNormalForce], racket_normal_force);
  if (racket && !gate3_racket_contact_prev_)
    telemetry[gate3::kRacketContactCount] += 1.0;
  if (table && !gate3_table_contact_prev_)
    telemetry[gate3::kTableContactCount] += 1.0;
  if (net && !gate3_net_contact_prev_)
    telemetry[gate3::kNetContactCount] += 1.0;

  // The venue fit and training virtual ball use an impulse map; MuJoCo's
  // underdamped soft-contact parameter is not that map and previously injected
  // up to 40 m/s into a ball hit by a roughly 2 m/s racket. Keep collision
  // detection physical, but on exactly one rising edge replace the solver's
  // outgoing ball state with the fitted planner/training law. The racket
  // surface velocity includes omega x r at the actual contact point.
  if (racket && !gate3_racket_contact_prev_ &&
      racket_contact_position_valid) {
    std::array<mjtNum, 6> racket_spatial_velocity{};
    mj_objectVelocity(
        m_, d_, mjOBJ_GEOM, gate3_racket_geom_id_,
        racket_spatial_velocity.data(), 0);
    const gate3::Vec3 racket_angular_velocity{
        static_cast<double>(racket_spatial_velocity[0]),
        static_cast<double>(racket_spatial_velocity[1]),
        static_cast<double>(racket_spatial_velocity[2])};
    const gate3::Vec3 racket_origin_velocity{
        static_cast<double>(racket_spatial_velocity[3]),
        static_cast<double>(racket_spatial_velocity[4]),
        static_cast<double>(racket_spatial_velocity[5])};
    const mjtNum* racket_position =
        d_->geom_xpos + 3 * gate3_racket_geom_id_;
    const gate3::Vec3 contact_offset{
        racket_contact_position[0] - racket_position[0],
        racket_contact_position[1] - racket_position[1],
        racket_contact_position[2] - racket_position[2]};
    const gate3::Vec3 racket_contact_velocity = gate3::Add(
        racket_origin_velocity,
        gate3::Cross(racket_angular_velocity, contact_offset));

    // The deploy/training observation contract defines the racket face normal
    // as the right_racket SITE's local +Y.  Do not use the mesh geom frame:
    // MuJoCo applies mesh_quat principal-axis alignment, so geom local +Y is
    // not the wrist/racket-frame +Y used by Isaac and the planner.
    const mjtNum* racket_rotation =
        d_->site_xmat + 9 * gate3_racket_site_id_;
    const gate3::Vec3 racket_face_normal{
        static_cast<double>(racket_rotation[1]),
        static_cast<double>(racket_rotation[4]),
        static_cast<double>(racket_rotation[7])};
    const gate3::Vec3 incoming_velocity{
        gate3_ball_pre_step_velocity_[0],
        gate3_ball_pre_step_velocity_[1],
        gate3_ball_pre_step_velocity_[2]};
    const gate3::Vec3 incoming_spin{
        gate3_ball_pre_step_velocity_[3],
        gate3_ball_pre_step_velocity_[4],
        gate3_ball_pre_step_velocity_[5]};
    const auto outgoing = gate3::PredictPaddleContact(
        incoming_velocity, racket_contact_velocity, racket_face_normal,
        incoming_spin, gate3::kBallRadius);
    for (int axis = 0; axis < 3; ++axis) {
      d_->qvel[gate3_ball_dof_addr_ + axis] =
          outgoing.linear_velocity[axis];
      d_->qpos[gate3_ball_qpos_addr_ + axis] =
          racket_contact_position[axis] +
          (gate3::kBallRadius + 1.0e-4) *
              outgoing.oriented_normal[axis];
    }
    gate3::SetWorldSpin(d_, gate3_ball_qpos_addr_, gate3_ball_dof_addr_, outgoing.angular_velocity);
  }

  // MuJoCo's soft-contact damping ratio is not a coefficient of restitution.
  // Apply the fitted slip/friction-cap and angular momentum impulse once
  // per physical table-contact rising edge, matching Planner.
  // Moving the ball center just clear of the table prevents the stable
  // critically-damped contact from applying a second impulse next step.
  if (table && !gate3_table_contact_prev_ &&
      gate3_ball_pre_step_velocity_[2] < 0.0) {
    const auto outgoing = gate3::PredictTableContact(
        {gate3_ball_pre_step_velocity_[0], gate3_ball_pre_step_velocity_[1], gate3_ball_pre_step_velocity_[2]},
        {gate3_ball_pre_step_velocity_[3], gate3_ball_pre_step_velocity_[4], gate3_ball_pre_step_velocity_[5]},
        gate3::kBallRadius, gate3_ball_restitution_v_, gate3_ball_tangential_gain_, gate3_ball_friction_cap_mu_);
    for (int axis = 0; axis < 3; ++axis)
      d_->qvel[gate3_ball_dof_addr_ + axis] = outgoing.linear_velocity[axis];
    gate3::SetWorldSpin(d_, gate3_ball_qpos_addr_, gate3_ball_dof_addr_, outgoing.angular_velocity);
    d_->qpos[gate3_ball_qpos_addr_ + 2] = std::max<mjtNum>(
        d_->qpos[gate3_ball_qpos_addr_ + 2],
        gate3::kTableSurfaceZ + gate3::kBallRadius + 1.0e-4);
  }
  gate3_racket_contact_prev_ = racket;
  gate3_table_contact_prev_ = table;
  gate3_net_contact_prev_ = net;
}

