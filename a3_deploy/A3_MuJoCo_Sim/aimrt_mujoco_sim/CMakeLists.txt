# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.24)

project(aimrt_mujoco_sim LANGUAGES C CXX)

if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Some option
option(AIMRT_MUJOCO_SIM_BUILD_TESTS "AimRT Mujoco Sim build tests." OFF)
option(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES "AimRT Mujoco Sim build examples." OFF)
option(AIMRT_MUJOCO_SIM_INSTALL "Enable installation of AimRT Mujoco Sim." ON)

# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type as Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(INSTALL_CONFIG_NAME ${PROJECT_NAME}-config)

# Master project configuration
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

  set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/_deps)

  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_C_VISIBILITY_PRESET hidden)
  set(CMAKE_C_VISIBILITY_INLINES_HIDDEN ON)
  set(CMAKE_CXX_VISIBILITY_PRESET hidden)
  set(CMAKE_CXX_VISIBILITY_INLINES_HIDDEN ON)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(AIMRT_MUJOCO_SIM_BUILD_TESTS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    endif()
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Include cmake module
include(NamespaceTool)
include(GetAimRT)
include(GetMujoco)

if(AIMRT_MUJOCO_SIM_BUILD_TESTS)
  include(GetGTest)
  enable_testing()
endif()

# Add subdirectory
add_subdirectory(src)

if(AIMRT_MUJOCO_SIM_INSTALL)
  # Install
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    DESTINATION ./
    FILES_MATCHING
    PATTERN "*.cmake")

  install(EXPORT ${INSTALL_CONFIG_NAME} DESTINATION lib/cmake/${PROJECT_NAME})
endif()
