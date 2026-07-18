# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get aimrt ...")

set(aimrt_DOWNLOAD_URL
    "https://github.com/AimRT/AimRT/archive/refs/tags/v1.6.0.tar.gz"
    CACHE STRING "AimRT source archive URL.")

if(aimrt_LOCAL_SOURCE)
  FetchContent_Declare(
    aimrt
    SOURCE_DIR ${aimrt_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    aimrt
    URL ${aimrt_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_aimrt)
  FetchContent_GetProperties(aimrt)
  if(NOT aimrt_POPULATED)
    set(AIMRT_BUILD_RUNTIME ON)

    set(AIMRT_BUILD_WITH_PROTOBUF ON)

    if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
      set(AIMRT_BUILD_WITH_ROS2 ON)
    else()
      set(AIMRT_BUILD_WITH_ROS2 OFF)
    endif()

    set(AIMRT_BUILD_NET_PLUGIN ON)
    set(AIMRT_BUILD_ICEORYX_PLUGIN ON)
    if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
      set(AIMRT_BUILD_ROS2_PLUGIN ON)
    endif()
    set(AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN ON)
    set(AIMRT_BUILD_ECHO_PLUGIN ON)

    FetchContent_MakeAvailable(aimrt)
  endif()
endfunction()

get_aimrt()
