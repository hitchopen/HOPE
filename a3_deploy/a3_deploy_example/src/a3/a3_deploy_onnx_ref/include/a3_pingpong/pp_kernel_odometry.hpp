#pragma once
#include "a3_pingpong/pp_base_estimator.hpp"

namespace a3_pingpong {
// Local, contact-assumed odometry for Kernel WAIT. No external world-frame
// claim: foot slip/flight cannot be observed without contact/localization input.
class KernelStanceOdometry {
 public:
  void Reset() { initialized_ = false; position_.setZero(); }
  Vec3 Step(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
            const Vec4& orientation, const Vec3& gyro, double dt) {
    const auto feet = sole_positions_w(q, Vec3::Zero(), orientation);
    (void)dq; (void)gyro; (void)dt;
    const double floor_z = std::min(feet[0].z(), feet[1].z());
    if (!initialized_) anchors_ = feet;
    Vec3 estimate = Vec3::Zero();
    double weight_sum = 0.;
    for (int i = 0; i < 2; ++i) {
      const double dz = feet[i].z() - floor_z;
      const double weight = std::exp(-dz / .008);
      // A raised foot is not a world anchor. Follow it during swing, then
      // freeze its anchor on landing instead of pulling back to its old spot.
      if (dz > .018) anchors_[i] = position_ + feet[i];
      estimate += weight * (anchors_[i] - feet[i]);
      weight_sum += weight;
    }
    position_ = estimate / weight_sum;
    position_.z() = -floor_z;
    initialized_ = true;
    return position_;
  }
 private:
  bool initialized_ = false;
  Vec3 position_ = Vec3::Zero();
  std::array<Vec3, 2> anchors_;
};
}  // namespace a3_pingpong
