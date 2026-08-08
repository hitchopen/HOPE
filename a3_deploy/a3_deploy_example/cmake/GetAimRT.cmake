# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get aimrt ...")

set(_gs_default_aimrt_download_url
    "https://github.com/AimRT/AimRT/archive/refs/tags/v1.6.0.tar.gz")
if(NOT DEFINED aimrt_DOWNLOAD_URL
    OR NOT aimrt_DOWNLOAD_URL STREQUAL "${_gs_default_aimrt_download_url}")
  set(aimrt_DOWNLOAD_URL
      "${_gs_default_aimrt_download_url}"
      CACHE STRING "AimRT source archive URL" FORCE)
endif()
message(STATUS "AimRT download URL: ${aimrt_DOWNLOAD_URL}")

set(aimrt_PATCH_DIR "${CMAKE_CURRENT_LIST_DIR}/aimrt_patches")
set(aimrt_PATCH_SCRIPT "${aimrt_PATCH_DIR}/ApplyAimRTPatches.cmake")

if(aimrt_LOCAL_SOURCE)
  FetchContent_Declare(
    aimrt
    SOURCE_DIR ${aimrt_LOCAL_SOURCE}
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${aimrt_PATCH_SCRIPT}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    aimrt
    URL ${aimrt_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${aimrt_PATCH_SCRIPT}
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_aimrt)
  FetchContent_GetProperties(aimrt)
  if(NOT aimrt_POPULATED)
    set(AIMRT_BUILD_RUNTIME ON)
    set(AIMRT_BUILD_WITH_PROTOBUF ON)
    set(AIMRT_BUILD_WITH_ROS2 ON)
    set(AIMRT_BUILD_ROS2_PLUGIN ON)
    set(AIMRT_BUILD_ICEORYX_PLUGIN ON)
    set(AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN ON)
    if(CMAKE_CROSSCOMPILING)
      find_program(_gs_host_protoc protoc REQUIRED)
      set(AIMRT_USE_LOCAL_PROTOC_COMPILER
          ON
          CACHE BOOL "Use host protoc while cross-compiling AimRT" FORCE)
      set(AIMRT_USE_PROTOC_PYTHON_PLUGIN
          ON
          CACHE BOOL "Use host-runnable Python protoc plugin while cross-compiling AimRT" FORCE)
      set(Protobuf_PROTOC_EXECUTABLE
          "${_gs_host_protoc}"
          CACHE FILEPATH "Host protoc executable" FORCE)
    endif()

    set(CMAKE_POLICY_VERSION_MINIMUM
        3.5
        CACHE STRING "Minimum CMake policy version" FORCE)
    set(YAML_CPP_BUILD_TESTS
        OFF
        CACHE BOOL "Disable yaml-cpp tests" FORCE)
    FetchContent_MakeAvailable(aimrt)
  endif()
endfunction()

get_aimrt()
