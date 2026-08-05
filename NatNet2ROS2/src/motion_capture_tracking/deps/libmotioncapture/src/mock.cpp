#include "libmotioncapture/mock.h"

#include <thread>

namespace libmotioncapture {

  class MotionCaptureMockImpl{
  public:
    MotionCaptureMockImpl()
    {
    }

  public:
    float dt;
  };

  MotionCaptureMock::MotionCaptureMock(
    float dt,
    const std::vector<RigidBody>& objects,
    const PointCloud& pointCloud)
  {
    pImpl = new MotionCaptureMockImpl;
    pImpl->dt = dt;
    for (const auto& obj : objects) {
      rigidBodies_.insert(std::make_pair(obj.name(), obj));
    }
    pointcloud_ = pointCloud;
  }

  void MotionCaptureMock::waitForNextFrame()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(pImpl->dt * 1000)));
  }

  const std::map<std::string, RigidBody>& MotionCaptureMock::rigidBodies() const
  {
    return rigidBodies_;
  }

  const PointCloud& MotionCaptureMock::pointCloud() const
  {
    return pointcloud_;
  }

  MotionCaptureMock::~MotionCaptureMock()
  {
    delete pImpl;
  }
}

