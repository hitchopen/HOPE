// MIT License
//
// Copyright (c) 2022 Alvin Sun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef VRPN_MOCAP__TRACKER_HPP_
#define VRPN_MOCAP__TRACKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vrpn_Connection.h"
#include "vrpn_Tracker.h"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vrpn_mocap
{

  /**
   * @brief a ROS2 node for tracking a single object in a VRPN network
   */
  class Tracker : public rclcpp::Node
  {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(Tracker)

    /**
     * @brief constructor
     *
     * @param tracker_name name of the object to track
     */
    explicit Tracker(const std::string &tracker_name);

    /**
     * @brief destructor
     */
    ~Tracker();

  protected:
    /**
     * @brief single object tracker created only from Client
     *
     * @param base_node VRPNClient node
     * @param name tracker name
     * @param connection vrpn connection pointer (looked up from tracker name if nullptr)
     * @see vrpn_mocap::Client
     */
    Tracker(
        const rclcpp::Node &base_node, const std::string &tracker_name,
        const std::shared_ptr<vrpn_Connection> &connection = nullptr);

  private:
    template <typename MsgT>
    using PublisherT = rclcpp::Publisher<MsgT>;

    template <typename... Args>
    static Tracker::SharedPtr private_make_shared(Args &&...args)
    {
      class TrackerDerived : public Tracker
      {
      public:
        explicit TrackerDerived(Args &&...args)
            : Tracker(std::forward<Args>(args)...) {}
      };

      return std::make_shared<TrackerDerived>(std::forward<Args>(args)...);
    }

    void Init();

    void MainLoop();

    std::string ValidNodeName(const std::string &name);

    const std::string name_;
    const bool multi_sensor_;
    const std::string frame_id_;
    const bool sensor_data_qos_;
    const std::shared_ptr<vrpn_Connection> connection_;

    vrpn_Tracker_Remote vrpn_tracker_;

    std::vector<PublisherT<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
    std::vector<PublisherT<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_pubs_;
    std::vector<PublisherT<geometry_msgs::msg::AccelStamped>::SharedPtr> accel_pubs_;

    rclcpp::TimerBase::SharedPtr timer_;

    template <typename MsgT>
    typename PublisherT<MsgT>::SharedPtr GetOrCreatePublisher(
        const size_t &sensor_idx, const std::string &channel,
        std::vector<typename PublisherT<MsgT>::SharedPtr> *pubs)
    {
      // expand publisher array size if needed
      if (pubs->size() <= sensor_idx)
      {
        pubs->resize(sensor_idx + 1);
      }

      // create publisher if needed
      const std::string sensor_channel =
          multi_sensor_ ? channel + "_id_" + std::to_string(sensor_idx) : channel;
      if (!pubs->at(sensor_idx))
      {
        if (sensor_data_qos_)
        {
          pubs->at(sensor_idx) = this->create_publisher<MsgT>(
              sensor_channel,
              rclcpp::SensorDataQoS());
        }
        else
        {
          pubs->at(sensor_idx) = this->create_publisher<MsgT>(
              sensor_channel,
              rclcpp::SystemDefaultsQoS());
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Creating sensor " << sensor_idx);
      }

      return pubs->at(sensor_idx);
    }

    static void VRPN_CALLBACK HandlePose(void *tracker, const vrpn_TRACKERCB tracker_pose);
    static void VRPN_CALLBACK HandleTwist(void *tracker, const vrpn_TRACKERVELCB tracker_vel);
    static void VRPN_CALLBACK HandleAccel(void *tracker, const vrpn_TRACKERACCCB tracker_acc);

    friend class Client;
  };

} // namespace vrpn_mocap

#endif // VRPN_MOCAP__TRACKER_HPP_
