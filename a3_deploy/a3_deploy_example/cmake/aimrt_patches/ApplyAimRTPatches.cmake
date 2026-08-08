set(_patch_dir "${CMAKE_CURRENT_LIST_DIR}")

if(NOT EXISTS "src/interface/aimrt_module_ros2_interface/util/ros2_type_support.h")
  message(STATUS "AimRT source tree is not ready, skipping local patches")
  return()
endif()

function(_apply_patch path)
  execute_process(
      COMMAND patch -p1 -N -r - -i "${path}"
      RESULT_VARIABLE _res)
  if(NOT (_res EQUAL 0 OR _res EQUAL 1))
    message(FATAL_ERROR "AimRT patch failed (${_res}): ${path}")
  endif()
endfunction()

set(_aimrt_version "")
if(EXISTS "VERSION")
  file(READ "VERSION" _aimrt_version)
  string(STRIP "${_aimrt_version}" _aimrt_version)
endif()
set(_aimrt_has_native_ros2_fixes FALSE)
if(_aimrt_version MATCHES "^1\\.[6-9]\\." OR _aimrt_version MATCHES "^[2-9]\\.")
  set(_aimrt_has_native_ros2_fixes TRUE)
endif()

if(_aimrt_has_native_ros2_fixes)
  message(STATUS
    "AimRT ${_aimrt_version} has native ROS2 type-support fixes; "
    "skipping legacy local patch files")
else()
  _apply_patch("${_patch_dir}/aimrt_ros2_typesupport_rmw_fix.patch")
  _apply_patch("${_patch_dir}/aimrt_ros2_serialize_buffer_fix.patch")
endif()

function(_replace_once file needle replacement desc)
  file(READ "${file}" _contents)
  string(FIND "${_contents}" "${replacement}" _already)
  if(NOT _already EQUAL -1)
    return()
  endif()

  string(FIND "${_contents}" "${needle}" _pos)
  if(_pos EQUAL -1)
    message(STATUS "AimRT patch context not found, skipping: ${desc}")
    return()
  endif()

  string(REPLACE "${needle}" "${replacement}" _contents "${_contents}")
  file(WRITE "${file}" "${_contents}")
endfunction()

function(_apply_rosidl_generator_filter)
  string(CONCAT _helper
    "include(GSRosidl OPTIONAL)\n"
    "if(COMMAND gs_disable_rosidl_generator_py_if_requested)\n"
    "  gs_disable_rosidl_generator_py_if_requested()\n"
    "endif()\n")

  foreach(_rosidl_cmake_file
      "src/protocols/plugins/ros2_plugin_proto/CMakeLists.txt"
      "src/protocols/ros2/aimrt_msgs/CMakeLists.txt"
      "src/protocols/ros2/example_ros2/CMakeLists.txt")
    if(NOT EXISTS "${_rosidl_cmake_file}")
      continue()
    endif()
    _replace_once(
      "${_rosidl_cmake_file}"
      "find_package(rosidl_default_generators REQUIRED)\n"
      "find_package(rosidl_default_generators REQUIRED)\n${_helper}"
      "ROSIDL generator filter in ${_rosidl_cmake_file}")
  endforeach()
endfunction()

function(_apply_ros2_jazzy_subscription_fix)
  set(_header "src/plugins/ros2_plugin/ros2_adapter_subscription.h")
  set(_source "src/plugins/ros2_plugin/ros2_channel_backend.cc")

  if(_aimrt_has_native_ros2_fixes)
    message(STATUS "AimRT ${_aimrt_version} has native ROS2 Jazzy support; skipping local subscription compatibility patch")
    return()
  endif()

  _replace_once(
    "${_header}"
    "#include \"rclcpp/subscription_options.hpp\"\n"
    "#include \"rclcpp/subscription_options.hpp\"\n#include \"rclcpp/version.h\"\n"
    "ros2_adapter_subscription.h include rclcpp/version.h")

  _replace_once(
    "${_header}"
    "      : rclcpp::SubscriptionBase(node_base, type_support_handle, topic_name, subscription_options, is_serialized),"
    "      :\n#if RCLCPP_VERSION_GTE(28, 0, 0)\n        rclcpp::SubscriptionBase(\n            node_base,\n            type_support_handle,\n            topic_name,\n            subscription_options,\n            rclcpp::SubscriptionEventCallbacks(),\n            true,\n            is_serialized ? rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE\n                          : rclcpp::DeliveredMessageKind::ROS_MESSAGE),\n#else\n        rclcpp::SubscriptionBase(node_base, type_support_handle, topic_name, subscription_options, is_serialized),\n#endif"
    "ros2_adapter_subscription.h SubscriptionBase constructor")

  _replace_once(
    "${_header}"
    "  void handle_loaned_message(void* loaned_message,\n                             const rclcpp::MessageInfo& message_info) override;"
    "  void handle_loaned_message(void* loaned_message,\n                             const rclcpp::MessageInfo& message_info) override;\n\n#if RCLCPP_VERSION_GTE(28, 0, 0)\n  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr\n  get_shared_dynamic_message_type() override { return nullptr; }\n\n  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr\n  get_shared_dynamic_message() override { return nullptr; }\n\n  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr\n  get_shared_dynamic_serialization_support() override { return nullptr; }\n\n  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr\n  create_dynamic_message() override { return nullptr; }\n\n  void return_dynamic_message(\n      rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr&) override {}\n\n  void handle_dynamic_message(\n      const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr&,\n      const rclcpp::MessageInfo&) override {}\n#endif"
    "ros2_adapter_subscription.h Jazzy dynamic message pure virtuals")

  _replace_once(
    "${_source}"
    "                    // todo: ros2 bug, remove template parameters after the new version is fixed\n                    options.to_rcl_subscription_options<void>(qos),"
    "#if RCLCPP_VERSION_GTE(28, 0, 0)\n                    options.to_rcl_subscription_options(qos),\n#else\n                    // todo: ros2 bug, remove template parameters after the new version is fixed\n                    options.to_rcl_subscription_options<void>(qos),\n#endif"
    "ros2_channel_backend.cc to_rcl_subscription_options")
endfunction()

_apply_rosidl_generator_filter()
_apply_ros2_jazzy_subscription_fix()
