#pragma once

#include <array>

namespace a3_pingpong {

inline constexpr double kVelocityBoxContractTolerance = 1e-9;

// Per-axis membership for a hitter_pure velocity box encoded as
// {x_lo,x_hi,y_lo,y_hi,z_lo,z_hi}.  Keeping this helper independent of ONNX/Eigen lets the
// exact deploy support-set rule be exercised by the host-only C++ test target.
inline bool velocity_in_box(const std::array<double, 6>& box,
                            double vx, double vy, double vz, double margin = 0.0) {
  return vx >= box[0] - margin && vx <= box[1] + margin &&
         vy >= box[2] - margin && vy <= box[3] + margin &&
         vz >= box[4] - margin && vz <= box[5] + margin;
}

// V10 samples a mixture of two boxes, not their Cartesian bounding union.  A velocity is
// supported only when all three axes belong to the same sampled component.
inline bool velocity_in_component_support(const std::array<double, 6>& core,
                                           const std::array<double, 6>& planner,
                                           double vx, double vy, double vz,
                                           double margin = 0.0) {
  return velocity_in_box(core, vx, vy, vz, margin) ||
         velocity_in_box(planner, vx, vy, vz, margin);
}

}  // namespace a3_pingpong
