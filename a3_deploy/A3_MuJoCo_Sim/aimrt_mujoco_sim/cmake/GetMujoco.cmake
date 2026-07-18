# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get mujoco ...")

set(mujoco_DOWNLOAD_URL
    "https://github.com/deepmind/mujoco/archive/3.1.6.tar.gz"
    CACHE STRING "")

if(mujoco_LOCAL_SOURCE)
  FetchContent_Declare(
    mujoco
    SOURCE_DIR ${mujoco_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    mujoco
    URL ${mujoco_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_mujoco)
  FetchContent_GetProperties(mujoco)
  if(NOT mujoco_POPULATED)
    set(MUJOCO_BUILD_EXAMPLES OFF)
    set(MUJOCO_BUILD_SIMULATE ON)
    set(MUJOCO_BUILD_TESTS OFF)
    set(MUJOCO_TEST_PYTHON_UTIL OFF)
    set(SIMULATE_BUILD_EXECUTABLE OFF)

    FetchContent_MakeAvailable(mujoco)
  endif()
endfunction()

get_mujoco()
