// Quaternion / frame math for the model_15200 (ping-pong) front-end.
// Ported VERBATIM from hope_ws/.../hope_wbc_runner/frame_math.py (which was in
// turn ported from mujoco_eval_onnx.py). Scalar-first quaternions (w,x,y,z),
// world<-body rotation convention. Do NOT "improve" these: they must match the
// training/sim2sim convention exactly or the obs is wrong.
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace a3_pingpong {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;  // quaternion (w, x, y, z)
using Mat3 = Eigen::Matrix3d;

inline Vec4 quat_mul(const Vec4& a, const Vec4& b) {
  const double aw = a[0], ax = a[1], ay = a[2], az = a[3];
  const double bw = b[0], bx = b[1], by = b[2], bz = b[3];
  Vec4 r;
  r << aw * bw - ax * bx - ay * by - az * bz,
       aw * bx + ax * bw + ay * bz - az * by,
       aw * by - ax * bz + ay * bw + az * bx,
       aw * bz + ax * by - ay * bx + az * bw;
  return r;
}

inline Vec4 quat_inv(const Vec4& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double n = w * w + x * x + y * y + z * z;
  Vec4 r;
  r << w, -x, -y, -z;
  return r / n;
}

// 3x3 rotation matrix R (world<-body) from quaternion (w,x,y,z).
inline Mat3 mat_from_quat(const Vec4& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  Mat3 R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
       2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
       2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y);
  return R;
}

inline Vec3 quat_rotate(const Vec4& q, const Vec3& v) { return mat_from_quat(q) * v; }
inline Vec3 quat_rotate_inverse(const Vec4& q, const Vec3& v) {
  return mat_from_quat(q).transpose() * v;
}

// Propagate world<-body attitude with a body-frame angular-rate sample.
// q_next = q_world_body * Exp(omega_body * dt). This is used only to bridge
// the short mocap transport age; absolute yaw remains anchored to mocap.
inline Vec4 quat_integrate_body_rate(const Vec4& q_world_body,
                                     const Vec3& omega_body,
                                     double dt) {
  if (!std::isfinite(dt) || dt <= 0.0 || !omega_body.allFinite())
    return q_world_body.normalized();
  const double angle = omega_body.norm() * dt;
  Vec4 delta;
  if (angle < 1.0e-9) {
    delta << 1.0, 0.5 * omega_body[0] * dt,
             0.5 * omega_body[1] * dt, 0.5 * omega_body[2] * dt;
  } else {
    const Vec3 axis = omega_body / omega_body.norm();
    const double half = 0.5 * angle;
    delta << std::cos(half), axis[0] * std::sin(half),
             axis[1] * std::sin(half), axis[2] * std::sin(half);
  }
  return quat_mul(q_world_body.normalized(), delta).normalized();
}

// Quaternion of the yaw-only (Z) rotation of q.
inline Vec4 yaw_quat(const Vec4& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  Vec4 r;
  r << std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0);
  return r;
}

// Pose of frame 2 expressed in frame 1. Outputs (t12, q12).
inline void subtract_frame_transforms(const Vec3& t01, const Vec4& q01,
                                      const Vec3& t02, const Vec4& q02,
                                      Vec3& t12, Vec4& q12) {
  const Vec4 q10 = quat_inv(q01);
  t12 = quat_rotate(q10, t02 - t01);
  q12 = quat_mul(q10, q02);
}

// Gravity unit vector in the body frame: R_body^T @ [0,0,-1].
inline Vec3 projected_gravity_body(const Vec4& base_quat_w) {
  return quat_rotate_inverse(base_quat_w, Vec3(0.0, 0.0, -1.0));
}

}  // namespace a3_pingpong
