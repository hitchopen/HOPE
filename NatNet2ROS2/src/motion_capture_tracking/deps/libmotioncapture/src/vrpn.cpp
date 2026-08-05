#include "libmotioncapture/vrpn.h"

#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <thread>
#include <memory>
#include <chrono>

// VRPN
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

namespace
{
  std::unordered_set<std::string> name_blacklist_({"VRPN Control"});
}

namespace libmotioncapture {

  class MotionCaptureVrpnImpl
  {
  public:
    static MotionCaptureVrpnImpl* instance;
    std::shared_ptr<vrpn_Connection> connection;
    std::unordered_map<std::string, std::shared_ptr<vrpn_Tracker_Remote> > trackers;
    std::unordered_map<std::string, vrpn_TRACKERCB> trackerData;
    int updateFrequency;

    void updateTrackers()
    {
      const char* name = nullptr;
      for (int i = 0; (name = connection->sender_name(i)) != NULL; ++i) {
        if (trackers.count(name) == 0 && name_blacklist_.count(name) == 0)
        {
          std::cerr << "tracker: " << name << std::endl;
          trackers.insert(std::make_pair(name,
            std::make_shared<vrpn_Tracker_Remote>(name, connection.get())));

          trackers[name]->register_change_handler((void*)name, &MotionCaptureVrpnImpl::handle_pose);
        }
      }
    }

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
    {
      // std::cerr << "pos " << (const char*)userData << tracker_pose.pos[0] << std::endl;
      std::string name((const char*)userData);
      instance->trackerData[name] = tracker_pose;
    }
  };

 MotionCaptureVrpnImpl*  MotionCaptureVrpnImpl::instance;

  MotionCaptureVrpn::MotionCaptureVrpn(
    const std::string& hostname,
    int updateFrequency)
  {
    pImpl = new MotionCaptureVrpnImpl;
    pImpl->instance = pImpl;
    pImpl->updateFrequency = updateFrequency;

    pImpl->connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(hostname.c_str()));
  }

  MotionCaptureVrpn::~MotionCaptureVrpn()
  {
    delete pImpl;
  }

  void MotionCaptureVrpn::waitForNextFrame()
  {
    // We use a fixed update frequency here, because VRPN is stateless
    // with respect to the active trackers. Since users might enable/disable
    // trackers at any time, this approach keeps the active trackers updated.
    // Disadvantage: higher latency, since we do not attempt to synchronize

    static auto lastTime = std::chrono::high_resolution_clock::now();

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - lastTime;
    auto desiredPeriod = std::chrono::milliseconds(1000 / pImpl->updateFrequency);
    if (elapsed < desiredPeriod) {
      std::this_thread::sleep_for(desiredPeriod - elapsed);
    }


    pImpl->updateTrackers();
    pImpl->trackerData.clear();
    pImpl->connection->mainloop();
    for (auto tracker : pImpl->trackers) {
      tracker.second->mainloop();
    }

    // Note: This implementation does not support stamp per rigid body, but will assume that all rigid bodies have the same timestamp
    for (const auto& data : pImpl->trackerData) {
      struct timeval stamp = data.second.msg_time;
      timestamp_ = stamp.tv_sec * 1000000ULL + stamp.tv_usec;

      break;
    }
    lastTime = now;
  }

  const std::map<std::string, RigidBody>& MotionCaptureVrpn::rigidBodies() const
  {
    rigidBodies_.clear();
    for (const auto& data : pImpl->trackerData) {
      Eigen::Vector3f position(
        data.second.pos[0],
        data.second.pos[1],
        data.second.pos[2]);

      Eigen::Quaternionf rotation(
        data.second.quat[3], // w
        data.second.quat[0], // x
        data.second.quat[1], // y
        data.second.quat[2]  // z
        );

      rigidBodies_.emplace(data.first, RigidBody(data.first, position, rotation));
    }
    return rigidBodies_;
  }

  RigidBody MotionCaptureVrpn::rigidBodyByName(const std::string &name) const
  {
    const auto data = pImpl->trackerData.find(name);
    if (data != pImpl->trackerData.end()) {

      Eigen::Vector3f position(
        data->second.pos[0],
        data->second.pos[1],
        data->second.pos[2]);

      Eigen::Quaternionf rotation(
        data->second.quat[3], // w
        data->second.quat[0], // x
        data->second.quat[1], // y
        data->second.quat[2]  // z
        );

      return RigidBody(name, position, rotation);
    }
    throw std::runtime_error("Unknown rigid body!");
  }

  uint64_t MotionCaptureVrpn::timeStamp() const
  {
    return timestamp_;
  }

}
