# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

find_package(fastcdr REQUIRED)

# Add target
add_library(${CUR_TARGET_NAME} SHARED)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE aimrt::interface::aimrt_pkg_c_interface
          ${CUR_SUPERIOR_NAMESPACE}::mujoco_sim_module)

# Set installation of target
if(AIMRT_MUJOCO_SIM_INSTALL)
  set_target_properties(${CUR_TARGET_NAME} PROPERTIES EXPORT_NAME ${CUR_TARGET_ALIAS_NAME} INSTALL_RPATH "$ORIGIN")
  install(
    TARGETS ${CUR_TARGET_NAME}
    EXPORT ${INSTALL_CONFIG_NAME}
    LIBRARY DESTINATION lib)
endif()

# Set misc of target
set_target_properties(${CUR_TARGET_NAME} PROPERTIES OUTPUT_NAME ${CUR_DIR})
