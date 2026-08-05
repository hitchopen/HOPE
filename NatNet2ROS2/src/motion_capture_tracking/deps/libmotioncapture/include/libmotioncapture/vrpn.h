#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureVrpnImpl;

  class MotionCaptureVrpn
    : public MotionCapture
  {
  public:
    MotionCaptureVrpn(
      const std::string& hostname,
      int updateFrequency = 100);

    virtual ~MotionCaptureVrpn();

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody> &rigidBodies() const;
    virtual RigidBody rigidBodyByName(const std::string &name) const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }

    virtual bool supportsTimeStamp() const
    {
      return true;
    }

  private:
    MotionCaptureVrpnImpl* pImpl;
  };

} // namespace libobjecttracker


