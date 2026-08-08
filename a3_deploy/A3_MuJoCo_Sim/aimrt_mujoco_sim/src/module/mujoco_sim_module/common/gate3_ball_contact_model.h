// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball {

using Vec3 = std::array<double, 3>;

inline constexpr double kBallInertiaCoeff = 2.0 / 3.0;
inline constexpr double kPaddleTangentialA = 0.52;
inline constexpr double kPaddleTangentialB = 0.0;
inline constexpr double kPaddleFrictionCap = 0.5;
inline constexpr double kPaddleRestitutionG1 = 0.759;
inline constexpr double kPaddleRestitutionG2 = -0.0441;
inline constexpr double kMagnusAccelerationCoeff = 0.00444;

struct PaddleContactResult {
  Vec3 linear_velocity{};
  Vec3 angular_velocity{};
  Vec3 oriented_normal{};
  double effective_restitution = 0.0;
};

inline double Dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline Vec3 Add(const Vec3& a, const Vec3& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline Vec3 Subtract(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline Vec3 Scale(const Vec3& value, double scale) {
  return {scale * value[0], scale * value[1], scale * value[2]};
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]};
}

inline double Norm(const Vec3& value) {
  return std::sqrt(Dot(value, value));
}

inline Vec3 OrientPaddleNormal(
    const Vec3& normal, const Vec3& incoming_velocity,
    const Vec3& racket_contact_velocity) {
  constexpr double kEpsilon = 1.0e-9;
  const double magnitude = Norm(normal);
  Vec3 oriented = Scale(normal, 1.0 / (magnitude + kEpsilon));
  if (Dot(Subtract(incoming_velocity, racket_contact_velocity), oriented) >
      0.0) {
    oriented = Scale(oriented, -1.0);
  }
  return oriented;
}

// Exact C++ port of hope_planner.ball_contact.predict_paddle_contact and
// training-side virtual_ball.predict_paddle_contact. The discrete update is
// deliberately applied only on a physical racket-contact rising edge.
inline PaddleContactResult PredictPaddleContact(
    const Vec3& incoming_velocity, const Vec3& racket_contact_velocity,
    const Vec3& face_normal, const Vec3& incoming_spin,
    double ball_radius) {
  constexpr double kEpsilon = 1.0e-9;
  const Vec3 normal = OrientPaddleNormal(
      face_normal, incoming_velocity, racket_contact_velocity);
  const Vec3 contact_offset = Scale(normal, -ball_radius);
  const Vec3 relative = Subtract(
      Add(incoming_velocity, Cross(incoming_spin, contact_offset)),
      racket_contact_velocity);
  const double normal_signed = Dot(relative, normal);
  const Vec3 tangential =
      Subtract(relative, Scale(normal, normal_signed));
  const double tangential_speed = Norm(tangential);
  const double normal_speed = std::abs(normal_signed);
  const double restitution = std::clamp(
      kPaddleRestitutionG1 *
          std::exp(kPaddleRestitutionG2 * normal_speed),
      0.05, 0.95);
  const double cosine =
      normal_speed /
      (std::hypot(tangential_speed, normal_signed) + kEpsilon);
  const double raw_tangential_impulse =
      (kPaddleTangentialA + kPaddleTangentialB * cosine) *
      tangential_speed;
  const double friction_cap =
      kPaddleFrictionCap * (1.0 + restitution) * normal_speed;
  const double tangential_impulse = std::min(
      std::max(raw_tangential_impulse, 0.0), friction_cap);

  Vec3 delta_tangential{};
  if (tangential_speed > kEpsilon) {
    delta_tangential = Scale(
        tangential,
        -tangential_impulse / (tangential_speed + kEpsilon));
  }
  const Vec3 delta_normal =
      Scale(normal, -(1.0 + restitution) * normal_signed);
  const Vec3 delta_spin = Scale(
      Cross(normal, delta_tangential),
      -1.0 / (kBallInertiaCoeff * ball_radius));

  return {
      .linear_velocity =
          Add(incoming_velocity, Add(delta_normal, delta_tangential)),
      .angular_velocity = Add(incoming_spin, delta_spin),
      .oriented_normal = normal,
      .effective_restitution = restitution};
}

inline Vec3 FlightAcceleration(
    const Vec3& linear_velocity, const Vec3& angular_velocity,
    double quadratic_drag_coeff) {
  const Vec3 drag = Scale(
      linear_velocity, -quadratic_drag_coeff * Norm(linear_velocity));
  const Vec3 magnus =
      Scale(Cross(angular_velocity, linear_velocity),
            kMagnusAccelerationCoeff);
  return Add(drag, magnus);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball
