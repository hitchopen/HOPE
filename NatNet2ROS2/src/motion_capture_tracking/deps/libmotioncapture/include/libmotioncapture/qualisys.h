#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureQualisysImpl;

  class MotionCaptureQualisys
    : public MotionCapture
  {
  public:
    MotionCaptureQualisys(
      const std::string& hostname,
      int basePort,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureQualisys();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual RigidBody rigidBodyByName(const std::string &name) const;
    virtual const PointCloud& pointCloud() const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }

    virtual bool supportsPointCloud() const
    {
      return true;
    }

    virtual bool supportsTimeStamp() const
    {
      return true;
    }

  private:
    MotionCaptureQualisysImpl* pImpl;
  };

} // namespace libobjecttracker


