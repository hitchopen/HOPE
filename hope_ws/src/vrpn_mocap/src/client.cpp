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

#include "vrpn_mocap/client.hpp"

#include <chrono>
#include <string>
#include <unordered_set>

namespace vrpn_mocap
{

  using namespace std::chrono_literals;

  const std::unordered_set<std::string> tracker_name_blacklist_({"VRPN Control"});

  Client::Client(const std::string &name)
      : Node(name),
        frame_id_(declare_parameter("frame_id", "world")),
        connection_(vrpn_get_connection_by_name(ParseHost().c_str()))
  {
    this->declare_parameter("multi_sensor", false);
    const double refresh_freq = this->declare_parameter("refresh_freq", 1.);
    refresh_timer_ =
        this->create_wall_timer(1s / refresh_freq, std::bind(&Client::RefreshConnection, this));

    const double update_freq = this->declare_parameter("update_freq", 100.);
    mainloop_timer_ = this->create_wall_timer(1s / update_freq, std::bind(&Client::MainLoop, this));

    this->declare_parameter("sensor_data_qos", true);
  }

  std::string Client::ParseHost()
  {
    const std::string server = this->declare_parameter("server", "");
    const int port = this->declare_parameter("port", 0);
    if (server.empty() || port == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "server or port invalid");
    }

    return server + ":" + std::to_string(port);
  }

  void Client::RefreshConnection()
  {
    for (int i = 0; connection_->sender_name(i); i++)
    {
      const std::string tracker_name = connection_->sender_name(i);
      if (trackers_.count(tracker_name) == 0 && tracker_name_blacklist_.count(tracker_name) == 0)
      {
        trackers_[tracker_name] = Tracker::private_make_shared(*this, tracker_name, connection_);
      }
    }
  }

  void Client::MainLoop()
  {
    connection_->mainloop();

    if (!connection_->doing_okay())
    {
      RCLCPP_WARN(this->get_logger(), "VRPN connection is bad");
    }

    for (const auto &tracker : trackers_)
    {
      tracker.second->MainLoop();
    }
  }

} // namespace vrpn_mocap
