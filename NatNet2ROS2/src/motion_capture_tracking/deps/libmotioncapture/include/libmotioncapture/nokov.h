#pragma once

#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureNokovImpl;

  class MotionCaptureNokov: public MotionCapture
  {
  public:
      MotionCaptureNokov(
      const std::string& hostname,
      bool enableFrequency = false, 
      int updateFrequency = 100);

    virtual ~MotionCaptureNokov();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
	virtual const std::map<std::string, RigidBody>& rigidBodies() const override;
	virtual RigidBody rigidBodyByName(const std::string& name) const override;
	virtual const PointCloud& pointCloud() const override;
	virtual bool supportsRigidBodyTracking() const override;
    virtual bool supportsPointCloud() const;

  private:
    MotionCaptureNokovImpl* pImpl;
  };

} // namespace libobjecttracker


