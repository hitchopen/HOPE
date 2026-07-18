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

#ifndef VRPN_MOCAP__CLIENT_HPP_
#define VRPN_MOCAP__CLIENT_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "vrpn_mocap/tracker.hpp"

namespace vrpn_mocap
{

  class Client : public rclcpp::Node
  {
  public:
    /**
     * @brief constructor
     *
     * @param name name of this VRPN Client node
     */
    explicit Client(const std::string &name);

  private:
    std::string ParseHost();

    void RefreshConnection();

    void MainLoop();

    std::unordered_map<std::string, Tracker::SharedPtr> trackers_;

    rclcpp::TimerBase::SharedPtr refresh_timer_;
    rclcpp::TimerBase::SharedPtr mainloop_timer_;

    const std::string frame_id_;
    const std::shared_ptr<vrpn_Connection> connection_;
  };

} // namespace vrpn_mocap

#endif // VRPN_MOCAP__CLIENT_HPP_
