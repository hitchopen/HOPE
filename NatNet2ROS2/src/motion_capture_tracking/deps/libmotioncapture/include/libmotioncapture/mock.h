#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureMockImpl;

  class MotionCaptureMock : public MotionCapture{
  public:
    MotionCaptureMock(
      float dt,
      const std::vector<RigidBody>& objects,
      const PointCloud& pointCloud);

    virtual ~MotionCaptureMock();

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();

    const std::map<std::string, RigidBody>& rigidBodies() const;

    const PointCloud& pointCloud() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }

    virtual bool supportsPointCloud() const
    {
      return true;
    }

  private:
    MotionCaptureMockImpl * pImpl;
  };
}

