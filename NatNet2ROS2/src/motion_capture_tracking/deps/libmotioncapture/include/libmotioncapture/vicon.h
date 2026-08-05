#pragma once
#include "libmotioncapture/motioncapture.h"

// GetSegmentGlobalTranslation
// GetSegmentGlobalRotationQuaternion
// Connect
// IsConnected
// EnableSegmentData
// EnableUnlabeledMarkerData
// EnableMarkerData
// GetVersion
// GetFrame
// GetLatencyTotal,
// GetUnlabeledMarkerCount
// GetUnlabeledMarkerGlobalTranslation

namespace libmotioncapture {

  class MotionCaptureViconImpl;

  class MotionCaptureVicon
    : public MotionCapture
  {
  public:
    MotionCaptureVicon(
      const std::string& hostname,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureVicon();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual RigidBody rigidBodyByName(const std::string& name) const;
    virtual const PointCloud& pointCloud() const;
    virtual const std::vector<LatencyInfo>& latency() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }

    virtual bool supportsLatencyEstimate() const
    {
      return true;
    }

    virtual bool supportsPointCloud() const
    {
      return true;
    }

  private:
    MotionCaptureViconImpl* pImpl;
  };

} // namespace libobjecttracker


