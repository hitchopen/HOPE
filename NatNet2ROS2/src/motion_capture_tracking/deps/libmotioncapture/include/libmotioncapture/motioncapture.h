#pragma once
#include <cstddef>
#include <limits>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

// Eigen
#include <Eigen/Geometry>

namespace libmotioncapture {

  typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> PointCloud;

  const char* version();

  class RigidBody
  {
  public:
    RigidBody(
      const std::string& name,
      const Eigen::Vector3f& position,
      Eigen::Quaternionf& rotation)
      : m_name(name)
      , m_position(position)
      , m_rotation(rotation)
    {
    }

    const std::string& name() const {
      return m_name;
    }

    const Eigen::Vector3f& position() const {
      return m_position;
    }

    const Eigen::Quaternionf& rotation() const {
      return m_rotation;
    }

  private:
    std::string m_name;
    Eigen::Vector3f m_position;
    Eigen::Quaternionf m_rotation;
  };

  class LatencyInfo
  {
  public:
    LatencyInfo(
      const std::string& name,
      double value)
      : m_name(name)
      , m_value(value)
    {
    }

    const std::string& name() const {
      return m_name;
    }

    // seconds
    double value() const {
      return m_value;
    }
  private:
    std::string m_name;
    double m_value;
  };

  class MotionCapture
  {
  public:
    static MotionCapture *connect(
      const std::string &type,
      const std::map<std::string, std::string> &cfg);

    virtual ~MotionCapture()
    {
    }

    // waits until a new frame is available
    virtual void waitForNextFrame() = 0;

    // Query data

    // returns reference to rigid bodies available in the current frame
    virtual const std::map<std::string, RigidBody>& rigidBodies() const
    {
      rigidBodies_.clear();
      return rigidBodies_;
    }

    // returns copy of rigid body with a specified name
    virtual RigidBody rigidBodyByName(
      const std::string& name) const;

    // returns pointer to point cloud (all unlabled markers)
    virtual const PointCloud& pointCloud() const
    {
      pointcloud_.resize(0, Eigen::NoChange);
      return pointcloud_;
    }

    // return latency information
    virtual const std::vector<LatencyInfo>& latency() const
    {
      latencies_.clear();
      return latencies_;
    }

    // returns timestamp in microseconds
    virtual uint64_t timeStamp() const
    {
      return 0;
    }

    // Return the age, in seconds, of the current capture timestamp relative
    // to the adapter host's monotonic clock.  Backends with a remote vendor
    // clock must synchronize that clock to the adapter before implementing
    // this method.  The ROS adapter combines this duration with RCL_SYSTEM_TIME
    // so the resulting header is in the host's NTP/PTP-disciplined Unix epoch.
    virtual double timeStampAge() const
    {
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Conservative clock-mapping uncertainty for timeStampAge(), in seconds.
    // This does not include the adapter host's own NTP/PTP synchronization
    // error, which must be qualified separately by deployment health checks.
    virtual double timeStampAgeUncertainty() const
    {
      return std::numeric_limits<double>::infinity();
    }

    // Query API capabilities

    // return true, if tracking of objects is supported
    virtual bool supportsRigidBodyTracking() const
    {
      return false;
    }
    // returns true, if latency can be estimated
    virtual bool supportsLatencyEstimate() const
    {
      return false;
    }
    // returns true if raw point cloud is available
    virtual bool supportsPointCloud() const
    {
      return false;
    }
    // returns true if timestamp is available
    virtual bool supportsTimeStamp() const
    {
      return false;
    }

    virtual bool supportsTimeStampAge() const
    {
      return false;
    }

  protected:
    mutable std::map<std::string, RigidBody> rigidBodies_;
    mutable PointCloud pointcloud_;
    mutable std::vector<LatencyInfo> latencies_;
    mutable uint64_t timestamp_;
  };

} // namespace libobjecttracker

