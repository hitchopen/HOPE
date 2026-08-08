if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_gs_default_deploy_arch "rockchip")
else()
  set(_gs_default_deploy_arch "x86_64")
endif()
set(GS_PACKAGE_ARCH_NAME "${_gs_default_deploy_arch}" CACHE STRING
  "Package architecture suffix used by the default deploy output directory")
set(_gs_default_runtime_output_dir
  "${PROJECT_SOURCE_DIR}/dist/a3_deploy_${GS_PACKAGE_ARCH_NAME}")
set(_gs_legacy_runtime_output_dir "${PROJECT_SOURCE_DIR}/target/release")
set(GS_RUNTIME_OUTPUT_DIR
  "${_gs_default_runtime_output_dir}" CACHE PATH
  "Directory for deploy executables and runtime-shared libraries")
if(GS_RUNTIME_OUTPUT_DIR STREQUAL "${_gs_legacy_runtime_output_dir}" OR
   GS_RUNTIME_OUTPUT_DIR STREQUAL "${_gs_legacy_runtime_output_dir}/")
  set(GS_RUNTIME_OUTPUT_DIR "${_gs_default_runtime_output_dir}" CACHE PATH
    "Directory for deploy executables and runtime-shared libraries" FORCE)
endif()
file(MAKE_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}")
message(STATUS "Deploy runtime output dir: ${GS_RUNTIME_OUTPUT_DIR}")

# ROS2 Humble's rosidl generators ship cpython-3.10 extensions; running them
# under a different Python (e.g. 3.11 from a venv) breaks import. When
# Python3_EXECUTABLE is passed on the command line, propagate it to
# PYTHON_EXECUTABLE (used by ament/rosidl) so every sub-project and build
# rule uses the same interpreter.
if(Python3_EXECUTABLE)
  set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH
    "Python interpreter for ament/rosidl (forwarded from Python3_EXECUTABLE)" FORCE)
endif()

# Pre-declare fmt FetchContent BEFORE any find_package(rclcpp/sensor_msgs/...)
# so that OVERRIDE_FIND_PACKAGE is in effect when rcl_logging_spdlog
# transitively calls find_package(fmt). Without this, the system libfmt-dev
# fmt::fmt IMPORTED target gets created first, then AimRT's FetchContent'd
# fmt 10.2.1 collides at add_library(fmt::fmt ALIAS fmt).
option(ENABLE_A3_AIMRT_BACKEND
  "Compile A3AimrtBackend with AimRT Core linkage (uses FetchContent)" OFF)
if(ENABLE_A3_AIMRT_BACKEND)
  include(FetchContent)
  FetchContent_Declare(fmt
    URL https://github.com/fmtlib/fmt/archive/10.2.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# ROS2 discovery can be controlled via env var HAS_ROS2:
# - 1: try ROS2 (find_package QUIET)
# - 0 or unset: disable ROS2 entirely (skip find_package)
set(ROS2_ENV_REQUEST "off")
if(DEFINED ENV{HAS_ROS2})
  if("$ENV{HAS_ROS2}" STREQUAL "1")
    set(ROS2_ENV_REQUEST "on")
  elseif("$ENV{HAS_ROS2}" STREQUAL "0")
    set(ROS2_ENV_REQUEST "off")
  else()
    message(WARNING "HAS_ROS2 should be 0 or 1; got '$ENV{HAS_ROS2}'. Falling back to auto-detect.")
  endif()
endif()

if(ROS2_ENV_REQUEST STREQUAL "off")
  set(rclcpp_FOUND FALSE)
  set(std_msgs_FOUND FALSE)
  message(STATUS "ROS2 disabled (HAS_ROS2 unset or 0)")
else()
  # Try to locate ROS2 only when explicitly enabled
  find_package(rclcpp QUIET)
  find_package(std_msgs QUIET)
endif()

# Find msgpack for message serialization (used by ROS2 and ZMQ handlers)
# msgpack is header-only and doesn't provide pkg-config, so we check for the header directly
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp
  PATHS /usr/include /usr/local/include
  DOC "msgpack C++ header location"
)

if(MSGPACK_INCLUDE_DIR)
  message(STATUS "✅ msgpack found at: ${MSGPACK_INCLUDE_DIR}")
  set(MSGPACK_INCLUDE_DIRS ${MSGPACK_INCLUDE_DIR})
else()
  message(FATAL_ERROR "msgpack not found. Install libmsgpack-dev (Ubuntu/Debian) or msgpack-cxx (Arch) or msgpack-devel (RHEL/CentOS)")
endif()

file(GLOB_RECURSE PROJECT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(APPEND PROJECT_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_manual_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_motion_library.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_encoder_decoder_runtime.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_zmq_smpl_source.cpp)
list(REMOVE_DUPLICATES PROJECT_SRCS)
list(REMOVE_ITEM PROJECT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy_onnx_ref.cpp)
# a3_deploy/main.cpp is the entry point for the a3_deploy_onnx_ref executable
# (PR 9.5); exclude from a3_deploy_shared so only the executable target pulls
# it in.
list(REMOVE_ITEM PROJECT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/main.cpp)
list(REMOVE_ITEM PROJECT_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_policy_runtime_probe.cpp)
list(REMOVE_ITEM PROJECT_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_debug/body_drive_debug_record_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_debug/body_drive_debug_type_support_pkg_main.cpp)

# a3_sync (PR 3): 6-channel ring-buffer + sync loop, pure C++ (no AimRT).
# Sources under src/a3_sync/ are picked up by the GLOB above and compiled into
# a3_deploy_shared, so both the main exe and the test binary can use them.
# Public headers live in include/a3_sync/.
#
# a3_rt (PR 4): RT primitives + A3BasedTask (pthread + clock_nanosleep +
# SCHED_FIFO + overrun detection). Self-contained (no AimRT dep). Sources
# under src/a3_rt/ are likewise picked up by the GLOB above. Public headers
# live in include/a3_rt/.

# -----------------------------------------------------------------------------
# robot_io backend abstraction (see notes/robot_io_backend_design.md)
#
# robot_io/*.cpp are compiled directly into the main executable rather than
# the a3_deploy_shared static lib so that the backend set (and any
# ENABLE_A3_BACKEND-only dependencies) stays localised to the exe.
# -----------------------------------------------------------------------------
option(ENABLE_A3_BACKEND "Enable A3 AimRT backend" OFF)

list(FILTER PROJECT_SRCS EXCLUDE REGEX "/robot_io/.*\\.cpp$")

set(ROBOT_IO_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_io/backend_factory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_io/layouts.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_io/a3_layout_extra.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_io/a3_aimrt_backend.cpp
)

# -----------------------------------------------------------------------------
# A3 AimRT ROS message types (PR 5)
#
# joint_msgs is vendored under thirdparty/joint_msgs/ and built in-tree via
# add_subdirectory (the whole package is 6 .msg files + a 20-line CMakeLists),
# so the build no longer depends on a machine-specific rl_deploy install
# prefix. sensor_msgs still comes from the host ROS 2 install because it's a
# standard ROS 2 package.
#
# rosidl_generate_interfaces (inside the vendored CMakeLists) needs
# ament_cmake + rosidl_default_generators, which live in the ROS 2 install —
# i.e. cmake still has to be invoked with `source /opt/ros/humble/setup.bash`
# (or jazzy) for this branch to light up. When those are missing we gracefully
# degrade to OFF: subscriber sources still compile (#ifdef HAS_A3_ROS_MSGS
# guards the ROS2-dependent code), so Part A unit tests (pure ConvertMessage)
# always run.
#
# AimRT Core wiring — Initialize() / AsyncStart() / GetChannelHandle() — is
# deferred to PR 7.
# -----------------------------------------------------------------------------
option(ENABLE_A3_ROS_MSGS
  "Build vendored joint_msgs + link sensor_msgs for A3 subscribers (PR 5)" OFF)

set(A3_ROS_MSGS_AVAILABLE FALSE)
if(ENABLE_A3_ROS_MSGS)
  list(APPEND CMAKE_PREFIX_PATH
    /opt/ros/humble
    /opt/ros/jazzy
  )
  find_package(ament_cmake QUIET)
  find_package(rosidl_default_generators QUIET)
  find_package(sensor_msgs CONFIG QUIET)
  if(ament_cmake_FOUND AND rosidl_default_generators_FOUND AND sensor_msgs_FOUND)
    # Build vendored joint_msgs. rosidl_generate_interfaces (inside the
    # nested CMakeLists) emits raw targets like joint_msgs__rosidl_generator_cpp;
    # the package only aliases the two typesupport_cpp variants itself, so we
    # mirror the remaining ::namespaced:: aliases that find_package(joint_msgs)
    # would have produced, to keep the link lines below unchanged.
    # Resolve relative to this file (src/a3/a3_deploy_onnx_ref/) so it works
    # whether cmake is invoked from the top-level a3_deploy_example/
    # project or directly from this directory.
    add_subdirectory(
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/joint_msgs
      ${CMAKE_CURRENT_BINARY_DIR}/joint_msgs_build)

    foreach(_jm_suffix
        rosidl_generator_cpp
        rosidl_typesupport_introspection_cpp)
      if(TARGET joint_msgs__${_jm_suffix}
         AND NOT TARGET joint_msgs::joint_msgs__${_jm_suffix})
        add_library(joint_msgs::joint_msgs__${_jm_suffix} ALIAS
                    joint_msgs__${_jm_suffix})
      endif()
    endforeach()

    # rosidl_typesupport_cpp resolves the concrete fastrtps/introspection
    # library via dlopen("libjoint_msgs__rosidl_typesupport_*.so"). That
    # dlopen uses the wrapper library's own RUNPATH, not the executable's, so
    # keep sibling generated libraries discoverable both in the build tree and
    # after copying them next to the deploy executable.
    foreach(_jm_rpath_target
        joint_msgs__rosidl_typesupport_cpp
        joint_msgs__rosidl_typesupport_c)
      if(TARGET ${_jm_rpath_target})
        set_target_properties(${_jm_rpath_target} PROPERTIES
          BUILD_WITH_INSTALL_RPATH TRUE
          BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
          INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      endif()
    endforeach()

    message(STATUS
      "✅ A3 ROS msgs: ENABLED (joint_msgs vendored under thirdparty/, sensor_msgs found)")
    set(A3_ROS_MSGS_AVAILABLE TRUE)
  else()
    message(WARNING
      "A3 ROS msgs: DISABLED "
      "(ament_cmake_FOUND=${ament_cmake_FOUND} "
      "rosidl_default_generators_FOUND=${rosidl_default_generators_FOUND} "
      "sensor_msgs_FOUND=${sensor_msgs_FOUND}); "
      "did you `source /opt/ros/humble/setup.bash` before cmake? "
      "subscriber code will compile without HAS_A3_ROS_MSGS — "
      "AimRT subscriber wiring will be skipped.")
  endif()
else()
  message(STATUS "A3 ROS msgs: disabled (ENABLE_A3_ROS_MSGS=OFF)")
endif()

set(A3_ROS_MSGS_LIBS)
if(A3_ROS_MSGS_AVAILABLE)
  set(A3_ROS_MSGS_LIBS
    joint_msgs::joint_msgs__rosidl_generator_cpp
    joint_msgs::joint_msgs__rosidl_typesupport_cpp
    joint_msgs::joint_msgs__rosidl_typesupport_fastrtps_cpp
    joint_msgs::joint_msgs__rosidl_typesupport_introspection_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_cpp
  )
endif()

# -----------------------------------------------------------------------------
# A3 AimRT Backend (PR 7)
#
# ENABLE_A3_AIMRT_BACKEND option() is declared at the top of this file so that
# fmt's FetchContent OVERRIDE_FIND_PACKAGE is active before find_package(rclcpp)
# triggers rcl_logging_spdlog → find_package(fmt).
#
# When OFF (default): a3_aimrt_backend.cpp compiles with AimRT code excised
# via #ifdef. No new dependencies.
# When ON: FetchContent pulls AimRT (cmake/GetAimRT.cmake).
# -----------------------------------------------------------------------------

set(A3_AIMRT_BACKEND_AVAILABLE FALSE)
set(A3_AIMRT_BACKEND_LIBS)
if(ENABLE_A3_AIMRT_BACKEND)
  if(NOT A3_ROS_MSGS_AVAILABLE)
    message(WARNING
      "A3 AimRT backend: requested ON but A3_ROS_MSGS_AVAILABLE=FALSE; "
      "falling back to OFF — backend compiles but AimRT glue is excised. "
      "Fix: install joint_msgs + sensor_msgs, then reconfigure.")
  else()
    # Option C: FetchContent AimRT + apply local patches when still needed.
    # cmake/GetAimRT.cmake declares the FetchContent; cmake/aimrt_patches/
    # holds the two ROS2 compatibility patches required for the build.
    include("${CMAKE_SOURCE_DIR}/cmake/GetAimRT.cmake")

    if(TARGET aimrt::runtime::core
        AND TARGET aimrt::interface::aimrt_module_cpp_interface
        AND TARGET aimrt::interface::aimrt_module_ros2_interface
        AND TARGET aimrt::interface::aimrt_module_protobuf_interface)
      message(STATUS
        "✅ A3 AimRT backend: ENABLED (FetchContent)")
      set(A3_AIMRT_BACKEND_AVAILABLE TRUE)
      set(A3_AIMRT_BACKEND_LIBS
        aimrt::interface::aimrt_module_cpp_interface
        aimrt::interface::aimrt_module_protobuf_interface
        aimrt::interface::aimrt_module_ros2_interface
        aimrt::runtime::core
      )
    else()
      message(WARNING
        "A3 AimRT backend: GetAimRT.cmake ran but expected targets "
        "(aimrt::runtime::core etc.) are missing. Falling back to OFF.")
    endif()
  endif()
else()
  message(STATUS "A3 AimRT backend: disabled (ENABLE_A3_AIMRT_BACKEND=OFF)")
endif()

# -----------------------------------------------------------------------------
# A3 Teleop Avatar protobuf messages
#
# The required AIMA protocol .proto files are copied into this repo under
# proto/aimdk/protocol/... so deploy builds do not depend on ~/aimde paths.
# Generate only when the AimRT backend is active; the teleop subscriber is an
# AimRT/protobuf input and the rest of A3 deploy still builds without it.
# -----------------------------------------------------------------------------
set(A3_TA_PROTO_AVAILABLE FALSE)
if(A3_AIMRT_BACKEND_AVAILABLE)
  if(TARGET protobuf::libprotobuf)
    if(NOT CMAKE_CROSSCOMPILING AND TARGET protobuf::protoc)
      set(A3_TA_PROTOC_COMMAND protobuf::protoc)
      set(A3_TA_PROTOC_DEPENDS protobuf::protoc)
      set(A3_TA_PROTOC_DESC "protobuf::protoc")
    elseif(Protobuf_PROTOC_EXECUTABLE)
      set(A3_TA_PROTOC_COMMAND "${Protobuf_PROTOC_EXECUTABLE}")
      set(A3_TA_PROTOC_DESC "${Protobuf_PROTOC_EXECUTABLE}")
    else()
      find_program(A3_HOST_PROTOC_EXECUTABLE protoc)
      if(A3_HOST_PROTOC_EXECUTABLE)
        set(A3_TA_PROTOC_COMMAND "${A3_HOST_PROTOC_EXECUTABLE}")
        set(A3_TA_PROTOC_DESC "${A3_HOST_PROTOC_EXECUTABLE}")
      endif()
    endif()
  endif()
  if(A3_TA_PROTOC_COMMAND AND TARGET protobuf::libprotobuf)
    set(A3_TA_PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/proto")
    set(A3_TA_PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
    set(A3_TA_PROTO_FILES
      "${A3_TA_PROTO_ROOT}/aimdk/protocol/common/timestamp.proto"
      "${A3_TA_PROTO_ROOT}/aimdk/protocol/common/control_source.proto"
      "${A3_TA_PROTO_ROOT}/aimdk/protocol/common/header.proto"
      "${A3_TA_PROTO_ROOT}/aimdk/protocol/ta/ta_whole_body_command.proto"
      "${A3_TA_PROTO_ROOT}/aimdk/protocol/ta/ta_channel.proto")
    set(A3_TA_PROTO_SRCS
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/timestamp.pb.cc"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/control_source.pb.cc"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/header.pb.cc"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/ta/ta_whole_body_command.pb.cc"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/ta/ta_channel.pb.cc")
    set(A3_TA_PROTO_HDRS
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/timestamp.pb.h"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/control_source.pb.h"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/common/header.pb.h"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/ta/ta_whole_body_command.pb.h"
      "${A3_TA_PROTO_GEN_DIR}/aimdk/protocol/ta/ta_channel.pb.h")

    add_custom_command(
      OUTPUT ${A3_TA_PROTO_SRCS} ${A3_TA_PROTO_HDRS}
      COMMAND ${CMAKE_COMMAND} -E make_directory "${A3_TA_PROTO_GEN_DIR}"
      COMMAND ${A3_TA_PROTOC_COMMAND}
              --proto_path="${A3_TA_PROTO_ROOT}"
              --cpp_out="${A3_TA_PROTO_GEN_DIR}"
              ${A3_TA_PROTO_FILES}
      DEPENDS ${A3_TA_PROTO_FILES} ${A3_TA_PROTOC_DEPENDS}
      COMMENT "Generating local A3 teleop protobuf bindings with ${A3_TA_PROTOC_DESC}")

    add_library(a3_ta_proto STATIC ${A3_TA_PROTO_SRCS} ${A3_TA_PROTO_HDRS})
    target_include_directories(a3_ta_proto PUBLIC "${A3_TA_PROTO_GEN_DIR}")
    target_link_libraries(a3_ta_proto PUBLIC protobuf::libprotobuf)
    set_target_properties(a3_ta_proto PROPERTIES
      CXX_STANDARD 20
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF)
    set(A3_TA_PROTO_AVAILABLE TRUE)
    message(STATUS "✅ A3 teleop protobuf: generated from local proto copy")
  else()
    message(WARNING
      "A3 teleop protobuf: host protoc or protobuf::libprotobuf missing; "
      "/ta/whole_body_command subscriber will be disabled.")
  endif()
endif()


# Optional AddressSanitizer support
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)

# Detect architecture and conditionally set CUDA library paths.
# Only needed for the optional TensorRT path.
if(ENABLE_TRT_INFERENCE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # Check for Thor (no DLA support)
  set(IS_THOR FALSE)
  if(DEFINED ENV{IS_THOR} AND "$ENV{IS_THOR}" STREQUAL "true")
    set(IS_THOR TRUE)
  elseif(EXISTS "/proc/device-tree/model")
    file(READ "/proc/device-tree/model" DEVICE_MODEL)
    string(FIND "${DEVICE_MODEL}" "Thor" THOR_POS)
    if(NOT THOR_POS EQUAL -1)
      set(IS_THOR TRUE)
    endif()
  endif()

  # ARM/Jetson platform - conditionally add DLA support
  # Dynamically find CUDA installation for ARM
  if(DEFINED ENV{CUDAToolkit_ROOT})
    set(CUDA_ARM_LIB_PATH "$ENV{CUDAToolkit_ROOT}/targets/aarch64-linux/lib")
  elseif(EXISTS "/usr/local/cuda/targets/aarch64-linux/lib")
    set(CUDA_ARM_LIB_PATH "/usr/local/cuda/targets/aarch64-linux/lib")
  else()
    # Find any available CUDA version for ARM
    file(GLOB CUDA_ARM_DIRS "/usr/local/cuda-*/targets/aarch64-linux/lib")
    if(CUDA_ARM_DIRS)
      list(GET CUDA_ARM_DIRS 0 CUDA_ARM_LIB_PATH)
    else()
      message(WARNING "No CUDA ARM libraries found, using fallback path")
      set(CUDA_ARM_LIB_PATH "/usr/local/cuda/lib64")
    endif()
  endif()

  set(CUDA_LINK_DIRS ${CUDA_ARM_LIB_PATH})

  # DLA support: only for non-Thor Jetson devices
  if(IS_THOR)
    set(CUDLA_LIBRARY "")
    message(STATUS "🚀 ARM64 (Thor) detected - building without DLA support")
    message(STATUS "   Using CUDA library path: ${CUDA_ARM_LIB_PATH}")
    message(STATUS "   Thor doesn't have DLA engines")
  else()
    # Match bare-metal Jetson builds: link cudla from the CUDA toolkit (targets/aarch64-linux/lib)
    set(CUDLA_LIBRARY "cudla")
    message(STATUS "✅ ARM64 detected - enabling DLA (cudla) support")
    message(STATUS "   Using CUDA library path: ${CUDA_ARM_LIB_PATH}")
    message(STATUS "   Using CUDLA library: -l${CUDLA_LIBRARY}")
  endif()

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  # x86_64 platform - no DLA support needed
  # Dynamically find CUDA installation for x86_64
  if(DEFINED ENV{CUDAToolkit_ROOT})
    set(CUDA_X64_LIB_PATH "$ENV{CUDAToolkit_ROOT}/lib64")
  elseif(EXISTS "/usr/local/cuda/lib64")
    set(CUDA_X64_LIB_PATH "/usr/local/cuda/lib64")
  else()
    # Find any available CUDA version for x86_64
    file(GLOB CUDA_X64_DIRS "/usr/local/cuda-*/lib64")
    if(CUDA_X64_DIRS)
      list(GET CUDA_X64_DIRS 0 CUDA_X64_LIB_PATH)
    else()
      message(WARNING "No CUDA x86_64 libraries found, using fallback path")
      set(CUDA_X64_LIB_PATH "/usr/local/cuda/lib64")
    endif()
  endif()

  set(CUDA_LINK_DIRS ${CUDA_X64_LIB_PATH})
  set(CUDLA_LIBRARY "")
  message(STATUS "✅ x86_64 detected - building without DLA support")
  message(STATUS "   Using CUDA library path: ${CUDA_X64_LIB_PATH}")
endif()
else()
  set(CUDA_LINK_DIRS "")
  set(CUDLA_LIBRARY "")
endif()

# ZMQ support (required for main executable and tests)
find_package(PkgConfig QUIET)
if(PkgConfig)
  pkg_check_modules(ZMQ QUIET libzmq)
endif()

find_path(ZMQ_INCLUDE_DIR zmq.h
  PATHS /usr/include /usr/local/include)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp
  PATHS ${ZMQ_INCLUDE_DIR} /usr/include /usr/local/include)
find_library(ZMQ_LIBRARY zmq
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)

if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "❌ libzmq is required but not found. Please install libzmq-dev (apt) or zeromq (brew)")
endif()
if(NOT CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "❌ cppzmq is required but zmq.hpp was not found. Please install cppzmq-dev (apt) or cppzmq (brew)")
endif()

# Add ZMQ to main executable
message(STATUS "✅ ZMQ support enabled for main executable")
message(STATUS "   libzmq include: ${ZMQ_INCLUDE_DIR}")
message(STATUS "   cppzmq include: ${CPPZMQ_INCLUDE_DIR}")

# Build ZMQ tests
if(ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
  add_executable(zmq_pose_subscriber_test
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/zmq_pose_subscriber_test.cpp)
  target_include_directories(zmq_pose_subscriber_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ZMQ_INCLUDE_DIR}
    ${CPPZMQ_INCLUDE_DIR})
  target_link_libraries(zmq_pose_subscriber_test PRIVATE ${ZMQ_LIBRARY})
  set_target_properties(zmq_pose_subscriber_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
  message(STATUS "✅ Added zmq_pose_subscriber_test (ZMQ found)")

  # Test for connecting to Python pose estimation server
  add_executable(zmq_python_sender_test
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/zmq_python_sender_test.cpp)
  target_include_directories(zmq_python_sender_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ZMQ_INCLUDE_DIR}
    ${CPPZMQ_INCLUDE_DIR})
  target_link_libraries(zmq_python_sender_test PRIVATE ${ZMQ_LIBRARY})
  set_target_properties(zmq_python_sender_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
  message(STATUS "✅ Added zmq_python_sender_test (ZMQ found)")
else()
  message(WARNING "⚠️  libzmq not found - skipping zmq_pose_subscriber_test")
endif()

find_package(Eigen3 REQUIRED)

# yaml-cpp is used by a3_tokenizer_replay (PR 9.1) to parse meta.json produced
# by the offline export script. Also parses a3_runtime_config.yaml for the
# a3_deploy_onnx_ref binary (PR 9.5).
find_package(yaml-cpp REQUIRED)

set(EXE_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${ZMQ_INCLUDE_DIR}
  ${CPPZMQ_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)


# Build a static library, used by both the main executable and the tests:
add_library(a3_deploy_shared STATIC ${PROJECT_SRCS})
target_include_directories(a3_deploy_shared PUBLIC ${EXE_INCLUDES})
target_link_libraries(a3_deploy_shared PUBLIC yaml-cpp onnxruntime::onnxruntime)

if(ENABLE_RKNN_INFERENCE)
  target_compile_definitions(a3_deploy_shared PUBLIC ENABLE_A3_RKNN_POLICY_RUNTIME=1)
  target_link_libraries(a3_deploy_shared PUBLIC rknnrt::rknnrt)
endif()

if(ENABLE_TRT_INFERENCE)
  target_compile_definitions(a3_deploy_shared PUBLIC ENABLE_A3_TRT_POLICY_RUNTIME=1)
  target_include_directories(a3_deploy_shared PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${TensorRT_INCLUDE_DIRS})
  if(CUDAToolkit_FOUND)
    target_include_directories(a3_deploy_shared PRIVATE
      ${CUDAToolkit_INCLUDE_DIRS})
  elseif(CUDA_INCLUDE_DIR)
    target_include_directories(a3_deploy_shared PRIVATE ${CUDA_INCLUDE_DIR})
  endif()
endif()

# A3 ROS msg wiring for the a3_io subscriber sources (PR 5). When available,
# the shared lib gains HAS_A3_ROS_MSGS=1 and links the rosidl targets.
if(A3_ROS_MSGS_AVAILABLE)
  target_compile_definitions(a3_deploy_shared PUBLIC HAS_A3_ROS_MSGS=1)
  target_link_libraries(a3_deploy_shared PUBLIC ${A3_ROS_MSGS_LIBS})
endif()

if(A3_TA_PROTO_AVAILABLE)
  target_compile_definitions(a3_deploy_shared PUBLIC HAS_A3_TA_PROTO=1)
  target_link_libraries(a3_deploy_shared PUBLIC a3_ta_proto)
endif()

# List of libs to link for both the main executable and the tests
set(EXE_LIBS
  a3_deploy_shared
  unitree_sdk2 ZLIB::ZLIB
  onnxruntime::onnxruntime
  ${ZMQ_LIBRARY}
)

if(ENABLE_TRT_INFERENCE)
  list(APPEND EXE_LIBS
    TRTInference
    CUDA::cudart
    ${TensorRT_nvinfer_LIBRARY} ${TensorRT_nvonnxparser_LIBRARY}
    ${CUDLA_LIBRARY})
endif()

if(rclcpp_FOUND AND std_msgs_FOUND)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
endif()

# -----------------------------------------------------------------------------
# A3 deploy executable (PR 9.5)
#
# `a3_deploy_onnx_ref --runtime-cfg=<path>` drives the A3 policy loop
# against mujoco_sim. Reuses:
#   a3_deploy_shared (contains A3PolicyDriver, A3ObsBuilder, A3TokenizerReplay,
#                     A3ActionDecoder, A3PolicyRuntime, a3_sync, a3_rt, a3_io)
#   ${ROBOT_IO_SOURCES} (so A3AimrtBackend is linked into the exe, mirroring
#                        the deploy executable's treatment of backend sources)
#   onnxruntime::onnxruntime (default ORT CPU inference)
# -----------------------------------------------------------------------------
set(A3_EXECUTABLE_NAME a3_deploy_onnx_ref)
add_executable(${A3_EXECUTABLE_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/main.cpp)
target_sources(${A3_EXECUTABLE_NAME} PRIVATE ${ROBOT_IO_SOURCES})
target_include_directories(${A3_EXECUTABLE_NAME}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${PROJECT_SOURCE_DIR}/src)
target_include_directories(${A3_EXECUTABLE_NAME} BEFORE PRIVATE
  ${PROJECT_SOURCE_DIR}/thirdparty/unitree_sdk2/include
  ${PROJECT_SOURCE_DIR}/thirdparty/unitree_sdk2/thirdparty/include
  ${PROJECT_SOURCE_DIR}/thirdparty/unitree_sdk2/thirdparty/include/ddscxx)
target_link_directories(${A3_EXECUTABLE_NAME} PRIVATE ${CUDA_LINK_DIRS})
target_link_libraries(${A3_EXECUTABLE_NAME} PRIVATE ${PROJECT_NAME}
  ${EXE_LIBS})

if(A3_AIMRT_BACKEND_AVAILABLE)
  target_compile_definitions(${A3_EXECUTABLE_NAME}
    PRIVATE ENABLE_A3_AIMRT_BACKEND=1)
  target_link_libraries(${A3_EXECUTABLE_NAME}
    PRIVATE ${A3_AIMRT_BACKEND_LIBS})
endif()

# ROS2 wiring is harmless no-op when ROS2 isn't found.
if(rclcpp_FOUND AND std_msgs_FOUND)
  target_include_directories(${A3_EXECUTABLE_NAME}
    PRIVATE ${MSGPACK_INCLUDE_DIRS})
  target_link_libraries(${A3_EXECUTABLE_NAME}
    PRIVATE ${ROS2_LIBS} pthread)
  target_compile_definitions(${A3_EXECUTABLE_NAME} PRIVATE HAS_ROS2=1)
else()
  target_compile_definitions(${A3_EXECUTABLE_NAME} PRIVATE HAS_ROS2=0)
endif()

set_target_properties(${A3_EXECUTABLE_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
             OUTPUT_NAME ${A3_EXECUTABLE_NAME}
             CXX_STANDARD 20
             CXX_STANDARD_REQUIRED ON
             CXX_EXTENSIONS OFF
             BUILD_WITH_INSTALL_RPATH TRUE
             BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
             INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
if(ENABLE_ASAN)
  message(STATUS "Building ${A3_EXECUTABLE_NAME} with AddressSanitizer")
  target_compile_options(${A3_EXECUTABLE_NAME} PRIVATE
    -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(${A3_EXECUTABLE_NAME} PRIVATE -fsanitize=address)
endif()

if(GS_BUNDLED_ONNXRUNTIME_LIB_DIR)
  file(GLOB _gs_ort_runtime_libs
    "${GS_BUNDLED_ONNXRUNTIME_LIB_DIR}/libonnxruntime*.so*")
  foreach(_gs_ort_lib IN LISTS _gs_ort_runtime_libs)
    add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_gs_ort_lib}"
        "${GS_RUNTIME_OUTPUT_DIR}"
      COMMENT "Copying bundled ONNX Runtime library to runtime output dir")
  endforeach()
endif()

if(ENABLE_RKNN_INFERENCE)
  add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:rknnrt::rknnrt>
      "${GS_RUNTIME_OUTPUT_DIR}"
    COMMENT "Copying bundled RKNN runtime library to runtime output dir")
endif()

set(A3_POLICY_RUNTIME_PROBE_NAME a3_policy_runtime_probe)
add_executable(${A3_POLICY_RUNTIME_PROBE_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_deploy/a3_policy_runtime_probe.cpp)
target_include_directories(${A3_POLICY_RUNTIME_PROBE_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_link_directories(${A3_POLICY_RUNTIME_PROBE_NAME} PRIVATE ${CUDA_LINK_DIRS})
target_link_libraries(${A3_POLICY_RUNTIME_PROBE_NAME}
  PRIVATE ${PROJECT_NAME}
  ${EXE_LIBS})
set_target_properties(${A3_POLICY_RUNTIME_PROBE_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
             OUTPUT_NAME ${A3_POLICY_RUNTIME_PROBE_NAME}
             CXX_STANDARD 20
             CXX_STANDARD_REQUIRED ON
             CXX_EXTENSIONS OFF
             BUILD_WITH_INSTALL_RPATH TRUE
             BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
             INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
if(ENABLE_RKNN_INFERENCE)
  add_custom_command(TARGET ${A3_POLICY_RUNTIME_PROBE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:rknnrt::rknnrt>
      "${GS_RUNTIME_OUTPUT_DIR}"
    COMMENT "Copying bundled RKNN runtime library to probe output dir")
endif()

# Copy AimRT iceoryx plugin .so next to the a3 binary so dlopen finds it
# via the $ORIGIN RPATH set above — no LD_LIBRARY_PATH needed.
if(A3_AIMRT_BACKEND_AVAILABLE)
  add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:aimrt_plugins_iceoryx_plugin>
      "${GS_RUNTIME_OUTPUT_DIR}"
    COMMENT "Copying libaimrt_iceoryx_plugin.so to runtime output dir")
endif()

# Copy AimRT ROS2 plugin .so as an optional fallback. The A3 default config in
# this branch remains iceoryx-only.
if(A3_AIMRT_BACKEND_AVAILABLE AND TARGET aimrt_plugins_ros2_plugin)
  set_target_properties(aimrt_plugins_ros2_plugin PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
    INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
  add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:aimrt_plugins_ros2_plugin>
      "${GS_RUNTIME_OUTPUT_DIR}"
    COMMENT "Copying libaimrt_ros2_plugin.so to runtime output dir")

  foreach(_ros2_proto_copy_target
      ros2_plugin_proto__rosidl_generator_c
      ros2_plugin_proto__rosidl_typesupport_cpp
      ros2_plugin_proto__rosidl_typesupport_fastrtps_cpp
      ros2_plugin_proto__rosidl_typesupport_introspection_cpp
      ros2_plugin_proto__rosidl_typesupport_c
      ros2_plugin_proto__rosidl_typesupport_fastrtps_c
      ros2_plugin_proto__rosidl_typesupport_introspection_c)
    if(TARGET ${_ros2_proto_copy_target})
      set_target_properties(${_ros2_proto_copy_target} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
        INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${_ros2_proto_copy_target}>
          "${GS_RUNTIME_OUTPUT_DIR}"
        COMMENT "Copying ${_ros2_proto_copy_target}.so to runtime output dir")
    endif()
  endforeach()
endif()

# Copy joint_msgs runtime .so files. The typesupport wrappers link against the
# generated C support library and dlopen concrete typesupport variants at
# runtime; keep all concrete shared-library targets next to the executable.
#
# Different ROS2 installs/generator versions may not emit every target for this
# vendored package, so copy only the concrete targets that exist.
if(A3_ROS_MSGS_AVAILABLE)
  foreach(_jm_copy_target
      joint_msgs__rosidl_generator_c
      joint_msgs__rosidl_typesupport_cpp
      joint_msgs__rosidl_typesupport_fastrtps_cpp
      joint_msgs__rosidl_typesupport_introspection_cpp
      joint_msgs__rosidl_typesupport_c
      joint_msgs__rosidl_typesupport_fastrtps_c
      joint_msgs__rosidl_typesupport_introspection_c)
    if(TARGET ${_jm_copy_target})
      set_target_properties(${_jm_copy_target} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
        INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      add_custom_command(TARGET ${A3_EXECUTABLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${_jm_copy_target}>
          "${GS_RUNTIME_OUTPUT_DIR}"
        COMMENT "Copying ${_jm_copy_target}.so to runtime output dir")
    endif()
  endforeach()
endif()

# -----------------------------------------------------------------------------
# A3 body-drive debug recorder
#
# This is a separate, read-only AimRT host process. It lets the
# record_playback_plugin subscribe to the body_drive topics and write raw MCAP
# with rolling storage, while the offline converter derives Foxglove topics.
# -----------------------------------------------------------------------------
set(A3_BODY_DRIVE_DEBUG_RECORD_NAME a3_body_drive_debug_record)
set(A3_BODY_DRIVE_DEBUG_TS_NAME a3_body_drive_debug_ros2_ts)

if(A3_AIMRT_BACKEND_AVAILABLE AND A3_ROS_MSGS_AVAILABLE)
  add_library(${A3_BODY_DRIVE_DEBUG_TS_NAME} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_debug/body_drive_debug_type_support_pkg_main.cpp)
  target_link_libraries(${A3_BODY_DRIVE_DEBUG_TS_NAME} PRIVATE
    aimrt::interface::aimrt_type_support_pkg_c_interface
    aimrt::interface::aimrt_module_ros2_interface
    ${A3_ROS_MSGS_LIBS})
  set_target_properties(${A3_BODY_DRIVE_DEBUG_TS_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
               OUTPUT_NAME ${A3_BODY_DRIVE_DEBUG_TS_NAME}
               CXX_STANDARD 20
               CXX_STANDARD_REQUIRED ON
               CXX_EXTENSIONS OFF
               BUILD_WITH_INSTALL_RPATH TRUE
               BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
               INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")

  add_executable(${A3_BODY_DRIVE_DEBUG_RECORD_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/a3_debug/body_drive_debug_record_main.cpp)
  target_link_libraries(${A3_BODY_DRIVE_DEBUG_RECORD_NAME} PRIVATE
    aimrt::runtime::core)
  add_dependencies(${A3_BODY_DRIVE_DEBUG_RECORD_NAME}
    ${A3_BODY_DRIVE_DEBUG_TS_NAME})
  set_target_properties(${A3_BODY_DRIVE_DEBUG_RECORD_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
               OUTPUT_NAME ${A3_BODY_DRIVE_DEBUG_RECORD_NAME}
               CXX_STANDARD 20
               CXX_STANDARD_REQUIRED ON
               CXX_EXTENSIONS OFF
               BUILD_WITH_INSTALL_RPATH TRUE
               BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
               INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")

  foreach(_a3_debug_plugin_target
      aimrt_plugins_iceoryx_plugin
      aimrt_plugins_ros2_plugin
      aimrt_plugins_record_playback_plugin)
    if(TARGET ${_a3_debug_plugin_target})
      set_target_properties(${_a3_debug_plugin_target} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
        INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      add_dependencies(${A3_BODY_DRIVE_DEBUG_RECORD_NAME}
        ${_a3_debug_plugin_target})
      add_custom_command(TARGET ${A3_BODY_DRIVE_DEBUG_RECORD_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${_a3_debug_plugin_target}>
          "${GS_RUNTIME_OUTPUT_DIR}"
        COMMENT "Copying ${_a3_debug_plugin_target} to runtime output dir")
    endif()
  endforeach()

  foreach(_a3_debug_ros2_proto_target
      ros2_plugin_proto__rosidl_generator_c
      ros2_plugin_proto__rosidl_typesupport_cpp
      ros2_plugin_proto__rosidl_typesupport_fastrtps_cpp
      ros2_plugin_proto__rosidl_typesupport_introspection_cpp
      ros2_plugin_proto__rosidl_typesupport_c
      ros2_plugin_proto__rosidl_typesupport_fastrtps_c
      ros2_plugin_proto__rosidl_typesupport_introspection_c)
    if(TARGET ${_a3_debug_ros2_proto_target})
      set_target_properties(${_a3_debug_ros2_proto_target} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
        INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      add_custom_command(TARGET ${A3_BODY_DRIVE_DEBUG_RECORD_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${_a3_debug_ros2_proto_target}>
          "${GS_RUNTIME_OUTPUT_DIR}"
        COMMENT "Copying ${_a3_debug_ros2_proto_target}.so to runtime output dir")
    endif()
  endforeach()

  foreach(_a3_debug_jm_copy_target
      joint_msgs__rosidl_generator_c
      joint_msgs__rosidl_typesupport_cpp
      joint_msgs__rosidl_typesupport_fastrtps_cpp
      joint_msgs__rosidl_typesupport_introspection_cpp
      joint_msgs__rosidl_typesupport_c
      joint_msgs__rosidl_typesupport_fastrtps_c
      joint_msgs__rosidl_typesupport_introspection_c)
    if(TARGET ${_a3_debug_jm_copy_target})
      set_target_properties(${_a3_debug_jm_copy_target} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
        INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib")
      add_custom_command(TARGET ${A3_BODY_DRIVE_DEBUG_RECORD_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:${_a3_debug_jm_copy_target}>
          "${GS_RUNTIME_OUTPUT_DIR}"
        COMMENT "Copying ${_a3_debug_jm_copy_target}.so to runtime output dir")
    endif()
  endforeach()

  file(MAKE_DIRECTORY
    "${GS_RUNTIME_OUTPUT_DIR}/config"
    "${GS_RUNTIME_OUTPUT_DIR}/tools")

  foreach(_a3_debug_cfg
      ${CMAKE_CURRENT_SOURCE_DIR}/config/a3_body_drive_debug_record.iceoryx.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/a3_body_drive_debug_record.ros2.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/a3_body_drive_debug.layout.json)
    if(EXISTS "${_a3_debug_cfg}")
      file(COPY "${_a3_debug_cfg}" DESTINATION "${GS_RUNTIME_OUTPUT_DIR}/config")
    endif()
  endforeach()

  foreach(_a3_debug_script
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_a3_body_drive_debug_record.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_a3_body_drive_debug_convert.sh)
    if(EXISTS "${_a3_debug_script}")
      file(COPY "${_a3_debug_script}" DESTINATION "${GS_RUNTIME_OUTPUT_DIR}"
        FILE_PERMISSIONS
          OWNER_READ OWNER_WRITE OWNER_EXECUTE
          GROUP_READ GROUP_EXECUTE
          WORLD_READ WORLD_EXECUTE)
    endif()
  endforeach()

  foreach(_a3_debug_tool
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/tools/a3_body_drive_debug_convert.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/tools/serve_foxglove_assets.py)
    if(EXISTS "${_a3_debug_tool}")
      file(COPY "${_a3_debug_tool}" DESTINATION "${GS_RUNTIME_OUTPUT_DIR}/tools"
        FILE_PERMISSIONS
          OWNER_READ OWNER_WRITE OWNER_EXECUTE
          GROUP_READ GROUP_EXECUTE
          WORLD_READ WORLD_EXECUTE)
    endif()
  endforeach()

else()
  message(STATUS
    "Skipping ${A3_BODY_DRIVE_DEBUG_RECORD_NAME} "
    "(requires ENABLE_A3_AIMRT_BACKEND=ON and ENABLE_A3_ROS_MSGS=ON)")
endif()

# test executable:
include(FetchContent)

# Download and make GoogleTest available
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

find_package(GTest REQUIRED)

file(GLOB_RECURSE TEST_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/*.cpp)
list(APPEND TEST_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/test_a3_manual_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/test_a3_encoder_obs_builder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/test_packed_motion_decoder.cpp)
list(REMOVE_DUPLICATES TEST_SRCS)

add_executable(run_tests ${TEST_SRCS})
target_sources(run_tests PRIVATE ${ROBOT_IO_SOURCES})
target_link_directories(run_tests PRIVATE ${CUDA_LINK_DIRS})
target_link_libraries(run_tests PRIVATE GTest::GTest GTest::Main ${EXE_LIBS})
target_include_directories(run_tests PUBLIC ${EXE_INCLUDES})

# A3 AimRT backend wiring (PR 7): only active when configure succeeded.
if(A3_AIMRT_BACKEND_AVAILABLE)
  target_compile_definitions(run_tests PRIVATE ENABLE_A3_AIMRT_BACKEND=1)
  target_link_libraries(run_tests PRIVATE ${A3_AIMRT_BACKEND_LIBS})
endif()

# Setting our output directory and C++ standard
set_target_properties(
  run_tests
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
             OUTPUT_NAME run_tests
             CXX_STANDARD 20
             CXX_STANDARD_REQUIRED ON
             CXX_EXTENSIONS OFF)
