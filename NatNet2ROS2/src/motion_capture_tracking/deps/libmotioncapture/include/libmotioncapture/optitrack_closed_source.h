#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureOptitrackClosedSourceImpl;

  class MotionCaptureOptitrackClosedSource : public MotionCapture{
  public:
    MotionCaptureOptitrackClosedSource(
      const std::string &hostname,
      int port_command = 1510);

    virtual ~MotionCaptureOptitrackClosedSource();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual const PointCloud& pointCloud() const;
    virtual const std::vector<LatencyInfo> &latency() const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }
    virtual bool supportsPointCloud() const
    {
      return true;
    }

    virtual bool supportsLatencyEstimate() const
    {
      return true;
    }

    virtual bool supportsTimeStamp() const
    {
      return true;
    }

  private:
    MotionCaptureOptitrackClosedSourceImpl * pImpl;
  };
}

