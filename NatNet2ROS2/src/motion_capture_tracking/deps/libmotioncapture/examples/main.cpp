#include <iostream>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

int main(int argc, char **argv)
{
  if (argc < 3) {
    std::cerr << "Use ./motioncapture_example <type> <hostname> [option1] [value1] ..." << std::endl;
    return 1;
  }

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = argv[2];
  for (int i = 3; i+1 < argc; i+=2) {
    cfg[argv[i]] = argv[i+1];
  }

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(argv[1], cfg);

  std::cout << "supportsRigidBodyTracking: " << mocap->supportsRigidBodyTracking() << std::endl;
  std::cout << "supportsLatencyEstimate: " << mocap->supportsLatencyEstimate() << std::endl;
  std::cout << "supportsPointCloud: " << mocap->supportsPointCloud() << std::endl;
  std::cout << "supportsTimeStamp: " << mocap->supportsTimeStamp() << std::endl;

  for (size_t frameId = 0;; ++frameId)
  {
    // Get a frame
    mocap->waitForNextFrame();

    std::cout << "frame " << frameId << std::endl;
    if (mocap->supportsTimeStamp()) {
      std::cout << "  timestamp: " << mocap->timeStamp() << " us" << std::endl;
    }
    if (mocap->supportsLatencyEstimate()) {
      std::cout << "  latency: " << std::endl;
      for (const auto& latency : mocap->latency()) {
        std::cout << "    " << latency.name() << " " << latency.value() << " s" << std::endl;
      }
    }

    if (mocap->supportsPointCloud()) {
      std::cout << "  pointcloud:" << std::endl;
      auto pointcloud = mocap->pointCloud();
      for (size_t i = 0; i < pointcloud.rows(); ++i) {
        const auto& point = pointcloud.row(i);
        std::cout << "    \"" << i << "\": [" << point(0) << "," << point(1) << "," << point(2) << "]" << std::endl;
      }
    }

    if (mocap->supportsRigidBodyTracking()) {
      auto rigidBodies = mocap->rigidBodies();

      std::cout << "  rigid bodies:" << std::endl;

      for (auto const& item: rigidBodies) {
        const auto& rigidBody = item.second;

        std::cout << "    \"" << rigidBody.name() << "\":" << std::endl;

        const auto& position = rigidBody.position();
        const auto& rotation = rigidBody.rotation();
        std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
        std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                            << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
      }
    }
  }

  return 0;
}
