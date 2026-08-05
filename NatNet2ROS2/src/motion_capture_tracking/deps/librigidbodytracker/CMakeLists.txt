cmake_minimum_required(VERSION 3.5)
project(librigidbodytracker)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)
find_package(PkgConfig)
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

pkg_check_modules(YamlCpp yaml-cpp)

###########
## Build ##
###########

## Additional include folders
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(librigidbodytracker
  src/rigid_body_tracker.cpp
)
target_link_libraries(librigidbodytracker
  ${PCL_LIBRARIES}
)

add_executable(playclouds
  src/playclouds.cpp
)
target_link_libraries(playclouds
  librigidbodytracker
  ${PCL_LIBRARIES}
  yaml-cpp
)

## assignment
# add_executable(assignment
#   src/assignment.cpp
# )
# target_link_libraries(assignment
#   ${Boost_LIBRARIES}
# )

add_executable(standalone
  src/standalone.cpp
)

add_executable(cbs_group_constraint
  src/cbs_group_constraint.cpp
)

target_link_libraries(cbs_group_constraint
  ${Boost_LIBRARIES}
)

target_link_libraries(
  standalone
  librigidbodytracker
  ${PCL_LIBRARIES}
)
