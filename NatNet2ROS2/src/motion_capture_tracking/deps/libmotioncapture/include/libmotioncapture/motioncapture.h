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
      const Eigen::Quaternionf& rotation,
      int32_t id = -1,
      float meanMarkerError = std::numeric_limits<float>::quiet_NaN())
      : m_name(name)
      , m_position(position)
      , m_rotation(rotation)
      , m_id(id)
      , m_meanMarkerError(meanMarkerError)
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

    int32_t id() const {
      return m_id;
    }

    float meanMarkerError() const {
      return m_meanMarkerError;
    }

  private:
    std::string m_name;
    Eigen::Vector3f m_position;
    Eigen::Quaternionf m_rotation;
    int32_t m_id;
    float m_meanMarkerError;
  };

  struct RigidBodyMarkerDefinition
  {
    uint32_t memberId;
    std::string name;
    Eigen::Vector3f position;
    int32_t requiredActiveLabel;
  };

  struct RigidBodyDefinition
  {
    std::string name;
    int32_t id;
    int32_t parentId;
    Eigen::Vector3f parentOffset;
    std::vector<RigidBodyMarkerDefinition> markers;
  };

  struct LabeledMarker
  {
    uint32_t id;
    uint32_t modelId;
    uint32_t memberId;
    Eigen::Vector3f position;
    float size;
    uint16_t params;
    float residual;
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

    // Static marker layouts reported by the motion-capture server.
    virtual const std::map<int32_t, RigidBodyDefinition>& rigidBodyDefinitions() const
    {
      rigidBodyDefinitions_.clear();
      return rigidBodyDefinitions_;
    }

    // Per-frame labeled marker samples. IDs retain the vendor model/member
    // split so callers can associate samples with a rigid-body definition.
    virtual const std::vector<LabeledMarker>& labeledMarkers() const
    {
      labeledMarkers_.clear();
      return labeledMarkers_;
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
    mutable std::map<int32_t, RigidBodyDefinition> rigidBodyDefinitions_;
    mutable std::vector<LabeledMarker> labeledMarkers_;
    mutable std::vector<LatencyInfo> latencies_;
    mutable uint64_t timestamp_;
  };

} // namespace libobjecttracker
