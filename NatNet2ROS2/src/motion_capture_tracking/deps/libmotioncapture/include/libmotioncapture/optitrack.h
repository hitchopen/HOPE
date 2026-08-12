#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureOptitrackImpl;

  class MotionCaptureOptitrack : public MotionCapture{
  public:
    MotionCaptureOptitrack(
      const std::string &hostname,
      const std::string& interface_ip = "0.0.0.0",
      int port_command = 1510,
      bool enable_clock_sync = false);

    virtual ~MotionCaptureOptitrack();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual const PointCloud& pointCloud() const;
    virtual const std::map<int32_t, RigidBodyDefinition>& rigidBodyDefinitions() const;
    virtual const std::vector<LabeledMarker>& labeledMarkers() const;
    virtual const std::vector<LatencyInfo> &latency() const;
    virtual uint64_t timeStamp() const;
    virtual double timeStampAge() const;
    virtual double timeStampAgeUncertainty() const;

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

    virtual bool supportsTimeStampAge() const;

  private:
    MotionCaptureOptitrackImpl * pImpl;
  };
}
