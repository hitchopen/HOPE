#pragma once
#include <cstddef>
#include <stdint.h>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <set>

namespace librigidbodytracker {

  enum TrackingMode {
    PositionMode,
    PoseMode,
    HybridMode
  };

  struct DynamicsConfiguration
  {
    double maxXVelocity;
    double maxYVelocity;
    double maxZVelocity;
    double maxPitchRate;
    double maxRollRate;
    double maxYawRate;
    double maxRoll;
    double maxPitch;
    double maxFitnessScore;
  };

  class RigidBodyTracker;
  class PointCloudDebugger;
  class RigidBody
  {
  public:
    RigidBody(
      size_t markerConfigurationIdx,
      size_t dynamicsConfigurationIdx,
      const Eigen::Affine3f& initialTransformation,
      const std::string& name);

    const Eigen::Affine3f& transformation() const;
    Eigen::Vector3f center() const { return m_lastTransformation.translation(); }
    bool orientationAvailable() const { return m_hasOrientation; }

    const Eigen::Affine3f& initialTransformation() const;
    Eigen::Vector3f initialCenter() const { return m_initialTransformation.translation(); }

    bool lastTransformationValid() const;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastValidTime() const {
      return m_lastValidTransform;
    }

    const std::string& name() const {
      return m_name;
    }

  private:
    size_t m_markerConfigurationIdx;
    size_t m_dynamicsConfigurationIdx;
    Eigen::Affine3f m_lastTransformation;
    bool m_hasOrientation;
    const Eigen::Affine3f m_initialTransformation;
    Eigen::Vector3f m_velocity;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastValidTransform;
    bool m_lastTransformationValid;
    std::string m_name;

    friend RigidBodyTracker;
    friend PointCloudDebugger;
  };

  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr MarkerConfiguration;

  class RigidBodyTracker
  {
  public:
    RigidBodyTracker(
      const std::vector<DynamicsConfiguration>& dynamicsConfigurations,
      const std::vector<MarkerConfiguration>& markerConfigurations,
      const std::vector<RigidBody>& rigidBodies);

    void update(
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    // for faster-than-real-time file playback
    void update(std::chrono::high_resolution_clock::time_point stamp,
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,std::string inputPath = "");

    const std::vector<RigidBody>& rigidBodies() const;

    void setLogWarningCallback(
      std::function<void(const std::string&)> logWarn);

  private:
    // Update and init using ICP
    void updatePose(std::chrono::high_resolution_clock::time_point stamp,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    bool initializePose(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    // Update and init using nearest neighbor
    void updatePosition(std::chrono::high_resolution_clock::time_point stamp,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    bool initializePosition(std::chrono::high_resolution_clock::time_point stamp,
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    bool initializeHybrid(std::chrono::high_resolution_clock::time_point stamp,
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    void updateHybrid(std::chrono::high_resolution_clock::time_point stamp,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers); 

    void logWarn(const std::string& msg);

  private:
    std::vector<MarkerConfiguration> m_markerConfigurations;
    std::vector<DynamicsConfiguration> m_dynamicsConfigurations;
    std::vector<RigidBody> m_rigidBodies;
    bool m_initialized;
    int m_init_attempts;
    bool m_trackPositionOnly;
    TrackingMode m_trackingMode;
    std::function<void(const std::string&)> m_logWarn;
    std::string m_inputPath;

  };

} // namespace librigidbodytracker


