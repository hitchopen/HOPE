#!/bin/bash

set -ex

cd $(dirname $0)

# You can change the install path by setting CMAKE_INSTALL_PREFIX
cmake -B build -DCMAKE_INSTALL_PREFIX=../../../prebuilt/audio_msgs_proto_x86_64

cmake --build build --target install --parallel 8

# After sourcing the setup file, you can use planning_msgs in your ROS2 project
# source ../ros2_plugin_proto_install/share/ros2_plugin_proto/setup.bash
