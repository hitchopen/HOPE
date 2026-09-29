// Local simulation ABI for the production Gate3 ball/contact model.
#include "gate3_ball_contact_model.h"
#include <algorithm>

namespace ball = aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball;

extern "C" void contact(const double* velocity, const double* racket_velocity,
                        const double* normal, const double* spin, double* out) {
  const auto result = ball::PredictPaddleContact(
      {velocity[0], velocity[1], velocity[2]},
      {racket_velocity[0], racket_velocity[1], racket_velocity[2]},
      {normal[0], normal[1], normal[2]}, {spin[0], spin[1], spin[2]}, .02);
  std::copy(result.linear_velocity.begin(), result.linear_velocity.end(), out);
  std::copy(result.angular_velocity.begin(), result.angular_velocity.end(), out + 3);
  std::copy(result.oriented_normal.begin(), result.oriented_normal.end(), out + 6);
}

extern "C" void flight(const double* velocity, const double* spin, double* out) {
  const auto result = ball::FlightAcceleration(
      {velocity[0], velocity[1], velocity[2]}, {spin[0], spin[1], spin[2]}, .122);
  std::copy(result.begin(), result.end(), out);
}
