# =============================================================================
# ROS2 Configuration
# =============================================================================
# This file contains ROS2 library discovery and test configuration
# Used by both main executable and test executable

if(NOT rclcpp_FOUND OR NOT std_msgs_FOUND)
  message(WARNING "ROS2.cmake included but ROS2 not found")
  return()
endif()

message(STATUS "Configuring ROS2 support...")

# =============================================================================
# ROS2 Library Discovery
# =============================================================================

# Auto-discover ROS2 include directories dynamically. Keep this list at the
# ROS package include-root level; adding every subdirectory under
# /opt/ros/<distro>/include can shadow C/POSIX headers such as <string.h> and
# also collides with Unitree's bundled CycloneDDS <dds/...> headers.
set(ROS2_INCLUDE_DIRS "")

# Common ROS2 distributions in order of preference (newest first)  
set(ROS2_DISTROS "jazzy" "iron" "humble" "galactic" "foxy" "eloquent" "dashing" "crystal")
set(ROS2_INSTALL_PATHS "/opt/ros" "/usr/local/ros")

set(ROS2_DISTRO_FOUND "")
foreach(install_path ${ROS2_INSTALL_PATHS})
  if(ROS2_DISTRO_FOUND)
    break()
  endif()
  
  foreach(distro ${ROS2_DISTROS})
    set(ros2_include_path "${install_path}/${distro}/include")
    if(EXISTS "${ros2_include_path}")
      set(_ros2_include_packages
        action_msgs
        builtin_interfaces
        geometry_msgs
        libstatistics_collector
        lifecycle_msgs
        rcl
        rcl_action
        rcl_interfaces
        rcl_logging_interface
        rcl_yaml_param_parser
        rclcpp
        rclcpp_action
        rclcpp_components
        rclcpp_lifecycle
        rcpputils
        rcutils
        rmw
        rmw_dds_common
        rmw_fastrtps_cpp
        rmw_fastrtps_shared_cpp
        rosgraph_msgs
        rosidl_dynamic_typesupport
        rosidl_runtime_c
        rosidl_runtime_cpp
        rosidl_typesupport_c
        rosidl_typesupport_cpp
        rosidl_typesupport_fastrtps_c
        rosidl_typesupport_fastrtps_cpp
        rosidl_typesupport_interface
        rosidl_typesupport_introspection_c
        rosidl_typesupport_introspection_cpp
        sensor_msgs
        service_msgs
        statistics_msgs
        std_msgs
        tracetools
        type_description_interfaces
        unique_identifier_msgs)
      foreach(_ros2_pkg ${_ros2_include_packages})
        set(_ros2_pkg_include "${ros2_include_path}/${_ros2_pkg}")
        if(IS_DIRECTORY "${_ros2_pkg_include}")
          list(APPEND ROS2_INCLUDE_DIRS "${_ros2_pkg_include}")
        endif()
      endforeach()
      set(ROS2_DISTRO_FOUND "${distro}")
      set(ROS2_INSTALL_BASE "${install_path}/${distro}")
      message(STATUS "📦 Found ROS2 ${distro} at ${install_path}/${distro}")
      break()
    endif()
  endforeach()
endforeach()

if(NOT ROS2_DISTRO_FOUND)
  message(WARNING "❌ No ROS2 installation found in common locations")
  return()
endif()

list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)

# Essential ROS2 libraries for basic pub/sub functionality
set(ESSENTIAL_LIB_PATTERNS
  "librclcpp.so*" "librcl.so*" "librcl_yaml_param_parser.so*"
  "librcpputils.so*" "librcutils.so*" "librmw.so*"
  "librmw_implementation.so*" "librmw_fastrtps_cpp.so*"
  "libstd_msgs__rosidl_typesupport_cpp.so*" "libstd_msgs__rosidl_generator_cpp.so*"
  "librosidl_typesupport_cpp.so*" "librosidl_runtime_cpp.so*" "librosidl_runtime_c.so*"
  "libbuiltin_interfaces__rosidl_typesupport_cpp.so*" "libbuiltin_interfaces__rosidl_generator_cpp.so*"
  "libstatistics_msgs__rosidl_typesupport_cpp.so*" "libstatistics_msgs__rosidl_generator_cpp.so*"
  "libtracetools.so*" "liblibstatistics_collector.so*"
  "libfastrtps.so*" "libfastcdr.so*" "libtinyxml2.so*" "libspdlog.so*" "libfmt.so*"
)

# Use CMAKE_SYSTEM_PROCESSOR for consistency with other CMakeLists
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(SYSTEM_LIB_DIR "/usr/lib/x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(SYSTEM_LIB_DIR "/usr/lib/aarch64-linux-gnu")
else()
  set(SYSTEM_LIB_DIR "/usr/lib")
endif()

# Auto-discover ROS2 libraries using dynamic distro path
set(ROS2_LIBS "")
foreach(pattern ${ESSENTIAL_LIB_PATTERNS})
  # Use dynamically detected ROS2 installation
  file(GLOB ros2_libs "${ROS2_INSTALL_BASE}/lib/${pattern}")
  list(APPEND ROS2_LIBS ${ros2_libs})
  # Also check system library paths
  file(GLOB system_libs "/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/${pattern}")
  list(APPEND ROS2_LIBS ${system_libs})
  file(GLOB usr_system_libs "${SYSTEM_LIB_DIR}/${pattern}")
  list(APPEND ROS2_LIBS ${usr_system_libs})
endforeach()

list(REMOVE_DUPLICATES ROS2_LIBS)
list(LENGTH ROS2_LIBS ROS2_LIB_COUNT)

message(STATUS "✅ ROS2 configuration: ${ROS2_LIB_COUNT} libraries discovered")

# =============================================================================
# ROS2 Test Executable
# =============================================================================

set(TEST_EXECUTABLE_NAME test_ros2)
add_executable(${TEST_EXECUTABLE_NAME} tests/test_ros2.cpp)

target_include_directories(${TEST_EXECUTABLE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(${TEST_EXECUTABLE_NAME} PRIVATE ${ROS2_INCLUDE_DIRS})
target_link_libraries(${TEST_EXECUTABLE_NAME} PRIVATE ${ROS2_LIBS} pthread)
target_compile_definitions(${TEST_EXECUTABLE_NAME} PRIVATE HAS_ROS2=1)

set_target_properties(${TEST_EXECUTABLE_NAME} PROPERTIES 
  RUNTIME_OUTPUT_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}"
  OUTPUT_NAME ${TEST_EXECUTABLE_NAME}
)

message(STATUS "✅ ROS2 test executable configured")

# =============================================================================
# Test Configuration Files
# =============================================================================

file(MAKE_DIRECTORY "${GS_RUNTIME_OUTPUT_DIR}/config")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config/fastrtps_profile.xml"
  "${GS_RUNTIME_OUTPUT_DIR}/config/fastrtps_profile.xml"
  COPYONLY
)

message(STATUS "✅ FastRTPS profile copied for test deployment")
