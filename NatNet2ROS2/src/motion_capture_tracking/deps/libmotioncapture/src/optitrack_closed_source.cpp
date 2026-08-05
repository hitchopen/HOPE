#include "libmotioncapture/optitrack_closed_source.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

// #include <iostream>
#include <condition_variable>
#include <mutex>

namespace libmotioncapture {

  class MotionCaptureOptitrackClosedSourceImpl{
  public:
    MotionCaptureOptitrackClosedSourceImpl()
      : received(false)
    {
    }

    void FrameReceivedCallback(sFrameOfMocapData *data)
    {
      // We can only receive data, if we can hold onto the data mutex to update the data structure
      std::lock_guard<std::mutex> lk(data_m);

      // update state
      rigidBodies->clear();
      for (size_t i = 0; i < data->nRigidBodies; ++i) {
        const auto& rb = data->RigidBodies[i];
        if (rb.params & 0x01) {
          const auto &def = rigidBodyDefinitions[rb.ID];

          Eigen::Vector3f position(
              rb.x + def.xoffset,
              rb.y + def.yoffset,
              rb.z + def.zoffset);

          Eigen::Quaternionf rotation(
              rb.qw, // w
              rb.qx, // x
              rb.qy, // y
              rb.qz  // z
          );
          rigidBodies->emplace(def.name, RigidBody(def.name, position, rotation));
        }
      }

      pointcloud->resize(data->nOtherMarkers, Eigen::NoChange);
      for (size_t r = 0; r < data->nOtherMarkers; ++r)
      {
        pointcloud->row(r) << 
            data->OtherMarkers[r][0], 
            data->OtherMarkers[r][1], 
            data->OtherMarkers[r][2];
      }

      latencies->clear();
      const bool timingValid = clockFrequency != 0 &&
        data->CameraMidExposureTimestamp != 0 &&
        data->CameraDataReceivedTimestamp >= data->CameraMidExposureTimestamp &&
        data->TransmitTimestamp >= data->CameraDataReceivedTimestamp;
      if (timingValid) {
        const uint64_t cameraLatencyTicks =
          data->CameraDataReceivedTimestamp - data->CameraMidExposureTimestamp;
        const double cameraLatencySeconds =
          cameraLatencyTicks / static_cast<double>(clockFrequency);
        latencies->emplace_back(LatencyInfo("Camera", cameraLatencySeconds));

        const uint64_t swLatencyTicks =
          data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
        const double swLatencySeconds =
          swLatencyTicks / static_cast<double>(clockFrequency);
        latencies->emplace_back(LatencyInfo("Motive", swLatencySeconds));

        // Preserve the legacy vendor-relative microsecond field for V2.
        *timestamp =
          (data->CameraMidExposureTimestamp / clockFrequency) * 1000000ULL +
          ((data->CameraMidExposureTimestamp % clockFrequency) * 1000000ULL) /
            clockFrequency;
      } else {
        *timestamp = 0;
      }

      // notify main thread of update
      {
        std::lock_guard<std::mutex> lk(cv_m);
        received = true;
      }
      cv.notify_all();
    }

  public:
    NatNetClient client;
    std::condition_variable cv;
    std::mutex cv_m;
    std::mutex data_m;
    bool received;

    uint64_t clockFrequency; // ticks/second for timestamps
    struct rigidBodyDefinition
    {
      std::string name;
      int ID;
      int parentID;
      float xoffset;
      float yoffset;
      float zoffset;
    };
    std::map<int, rigidBodyDefinition> rigidBodyDefinitions;

    std::map<std::string, RigidBody>* rigidBodies;
    PointCloud* pointcloud;
    std::vector<LatencyInfo>* latencies;
    uint64_t* timestamp;
  };

  static void FrameReceivedCallback(sFrameOfMocapData *data, void *pUserData)
  {
    auto client = reinterpret_cast<MotionCaptureOptitrackClosedSourceImpl *>(pUserData);
    client->FrameReceivedCallback(data);
  }

  MotionCaptureOptitrackClosedSource::MotionCaptureOptitrackClosedSource(
    const std::string &hostname,
    int port_command)
  {
    pImpl = new MotionCaptureOptitrackClosedSourceImpl;
    pImpl->rigidBodies = &rigidBodies_;
    pImpl->pointcloud = &pointcloud_;
    pImpl->latencies = &latencies_;
    pImpl->timestamp = &timestamp_;

    ErrorCode err;

    err = pImpl->client.SetFrameReceivedCallback(FrameReceivedCallback, pImpl);
    if (err != ErrorCode_OK) {
      throw std::runtime_error("NatNetSDK Error " + std::to_string(err));
    }

    // Connect first to find out server configuration
    sNatNetClientConnectParams connectParams;
    connectParams.serverAddress = hostname.c_str();
    connectParams.serverCommandPort = port_command;
    err = pImpl->client.Connect(connectParams);
    if (err != ErrorCode_OK) {
      throw std::runtime_error("NatNetSDK Error " + std::to_string(err));
    }

    // Find server configuration and update connection settings
    sServerDescription serverDesc;
    err = pImpl->client.GetServerDescription(&serverDesc);
    if (err != ErrorCode_OK) {
      throw std::runtime_error("NatNetSDK Error " + std::to_string(err));
    }
    pImpl->clockFrequency = serverDesc.HighResClockFrequency;

    connectParams.serverDataPort = serverDesc.ConnectionDataPort;
    connectParams.connectionType = serverDesc.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;

    std::string multicastAddress = 
      std::to_string(serverDesc.ConnectionMulticastAddress[0]) + "." + 
      std::to_string(serverDesc.ConnectionMulticastAddress[1]) + "." + 
      std::to_string(serverDesc.ConnectionMulticastAddress[2]) + "." +
      std::to_string(serverDesc.ConnectionMulticastAddress[3]);
    connectParams.multicastAddress = multicastAddress.c_str();

    // Reconnect
    pImpl->client.Disconnect();
    err = pImpl->client.Connect(connectParams);
    if (err != ErrorCode_OK)
    {
      throw std::runtime_error("NatNetSDK Error " + std::to_string(err));
    }

    // get data description
    sDataDescriptions *pDataDefs = NULL;
    err = pImpl->client.GetDataDescriptionList(&pDataDefs, 1 << Descriptor_RigidBody);
    if (err != ErrorCode_OK || pDataDefs == NULL)
    {
      throw std::runtime_error("NatNetSDK Error " + std::to_string(err));
    }
    for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
    {
      if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
        const auto pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
        auto& def = pImpl->rigidBodyDefinitions[pRB->ID];
        def.ID = pRB->ID;
        def.name = pRB->szName;
        def.parentID = pRB->parentID;
        def.xoffset = pRB->offsetx;
        def.yoffset = pRB->offsety;
        def.zoffset = pRB->offsety;
      }
    }
  }

  const std::string & MotionCaptureOptitrackClosedSource::version() const
  {
    // return pImpl->version;
  }

  void MotionCaptureOptitrackClosedSource::waitForNextFrame()
  {
    // unlock data mutex to allow asynchronous callback to update
    pImpl->received = false;
    pImpl->data_m.unlock();

    // wait for update
    std::unique_lock<std::mutex> lk(pImpl->cv_m);
    pImpl->cv.wait(lk, [this] { return pImpl->received; });
    
    // lock data mutex for user to use data safely
    pImpl->data_m.lock();
  }

  const std::map<std::string, RigidBody>& MotionCaptureOptitrackClosedSource::rigidBodies() const
  {
    return rigidBodies_;
  }

  const PointCloud& MotionCaptureOptitrackClosedSource::pointCloud() const
  {
    return pointcloud_;
  }

  const std::vector<LatencyInfo> &MotionCaptureOptitrackClosedSource::latency() const
  {
    return latencies_;
  }

  uint64_t MotionCaptureOptitrackClosedSource::timeStamp() const
  {
    return timestamp_;
  }

  MotionCaptureOptitrackClosedSource::~MotionCaptureOptitrackClosedSource()
  {
    delete pImpl;
  }

}
