cmake_minimum_required(VERSION 3.5)
project(libmotioncapture)

# define some options
option(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS "Generate Python Bindings" ON)
option(LIBMOTIONCAPTURE_ENABLE_QUALISYS "Enable Qualisys" ON)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK "Enable Optitrack" ON)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE "Enable Optitrack (Closed Source)" OFF)
option(LIBMOTIONCAPTURE_ENABLE_VICON "Enable Vicon" ON)
option(LIBMOTIONCAPTURE_ENABLE_NOKOV "Enable Nokov" OFF)
option(LIBMOTIONCAPTURE_ENABLE_VRPN "Enable VRPN" ON)
option(LIBMOTIONCAPTURE_ENABLE_FZMOTION "Enable FZMOTION" OFF)
option(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS "Enable MotionAnalysis" OFF)
option(LIBMOTIONCAPTURE_BUILD_EXAMPLE "Enable Example application" ON)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Boost) # for optitrack
add_definitions(
  -DBOOST_DATE_TIME_NO_LIB
  -DBOOST_REGEX_NO_LIB
  -D_BUILD_STATIC_LIB
)

find_package(Eigen3 REQUIRED)

set(VICON_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/vicon-datastream-sdk/)
set(QUALISYS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/qualisys_cpp_sdk/)
set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDKCrossplatform/)
set(NOKOV_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/nokov_sdk/)
set(VRPN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/vrpn/)
set(MOTIONANALYSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/cortex_sdk_linux/)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

## Additional include folders
set(my_include_directories
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
set(my_link_directories
  ${Boost_LIBRARY_DIRS}
)
set(my_files
  src/motioncapture.cpp
  src/mock.cpp
)

if (LIBMOTIONCAPTURE_ENABLE_VICON)
  message("including vicon")

  add_definitions(-DENABLE_VICON)
  add_subdirectory(deps/vicon-datastream-sdk EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${VICON_SDK_DIR}/Vicon/CrossMarket/DataStream
  )
  set(my_files
    ${my_files}
    src/vicon.cpp
  )
  set(my_libraries
    ${my_libraries}
    ViconDataStreamSDK_CPP
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_OPTITRACK)
  message("including optitrack")

  add_definitions(-DENABLE_OPTITRACK)
  set(my_include_directories
    ${my_include_directories}
  )
  set(my_files
    ${my_files}
    src/optitrack.cpp
  )
  set(my_libraries
    ${my_libraries}
    Boost::boost
    Threads::Threads
  )
  if (Boost_FOUND)
    set(my_libraries
      ${my_libraries}
      Boost::boost
    )
  endif()
endif()

if (BUILD_TESTING)
  enable_testing()
  add_executable(test_natnet_clock_sync tests/test_natnet_clock_sync.cpp)
  target_include_directories(test_natnet_clock_sync PRIVATE include)
  add_test(NAME test_natnet_clock_sync COMMAND test_natnet_clock_sync)
endif()

if (LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE)
  if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
    message("Including optitrack_closed_source")

    add_definitions(-DENABLE_OPTITRACK_CLOSED_SOURCE)
    add_subdirectory(deps/NatNetSDKCrossplatform EXCLUDE_FROM_ALL)
    set(my_include_directories
      ${my_include_directories}
      ${NATNET_SDK_DIR}/include
    )
    set(my_files
      ${my_files}
      src/optitrack_closed_source.cpp
    )
    set(my_link_directories
      ${my_link_directories}
      ${NATNET_SDK_DIR}/lib/ubuntu
    )
    set(my_libraries
      ${my_libraries}
      NatNet
    )
  else()
    message(WARNING "Can not include optitrack_closed_source due to missing SDK. (${CMAKE_SYSTEM_NAME}, ${CMAKE_SYSTEM_PROCESSOR})")
  endif()
endif()

if (LIBMOTIONCAPTURE_ENABLE_QUALISYS)
  message("including qualisys")

  add_definitions(-DENABLE_QUALISYS)
  add_subdirectory(deps/qualisys_cpp_sdk EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${QUALISYS_DIR}
  )
  set(my_files
    ${my_files}
    src/qualisys.cpp
  )
  set(my_libraries
    ${my_libraries}
    qualisys_cpp_sdk
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_NOKOV)
  message("including nokov")

  EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
  message( STATUS "Architecture: ${ARCHITECTURE}" )

  add_definitions(-DENABLE_NOKOV)
  set(my_include_directories
    ${my_include_directories}
    ${NOKOV_SDK_DIR}/include
  )
  set(my_files
    ${my_files}
    src/nokov.cpp
  )

  if (${ARCHITECTURE} MATCHES "arm")
    set(my_link_directories ${my_link_directories} ${NOKOV_SDK_DIR}/lib/armhf)    
  elseif (${ARCHITECTURE} MATCHES "aarch")
    set(my_link_directories ${my_link_directories} ${NOKOV_SDK_DIR}/lib/aarch64)  
  else()
    set(my_link_directories ${my_link_directories} ${NOKOV_SDK_DIR}/lib/x64)
  endif()

  message(STATUS "Link to nokov library directory:${my_link_directories}")

  set(my_libraries
    ${my_libraries}
    nokov_sdk
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_VRPN)
  message("including vrpn")

  add_definitions(-DENABLE_VRPN)
  option(VRPN_BUILD_CLIENT_LIBRARY "" ON)
  add_subdirectory(deps/vrpn EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${VRPN_DIR}
  )
  set(my_files
    ${my_files}
    src/vrpn.cpp
  )
  set(my_libraries
    ${my_libraries}
    ${VRPN_CLIENT_LIBRARY}
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_FZMOTION)
  message("including fzmotion")

  add_definitions(-DENABLE_FZMOTION)
  set(my_files
    ${my_files}
    src/fzmotion.cpp
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS)
  message("including motionanalysis")

  add_definitions(-DENABLE_MOTIONANALYSIS)
  add_subdirectory(deps/cortex_sdk_linux EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${MOTIONANALYSIS_DIR}
  )
  set(my_files
    ${my_files}
    src/motionanalysis.cpp
  )
  set(my_libraries
    ${my_libraries}
    cortex_sdk pthread m
  )
endif()

include_directories(
  ${my_include_directories}
)

## Declare a cpp library
add_library(libmotioncapture
  ${my_files}
)

## Specify libraries to link a library or executable target against
target_link_directories(libmotioncapture PUBLIC
  ${my_link_directories}
)
target_link_libraries(libmotioncapture
  Eigen3::Eigen
  ${my_libraries}
)
set_property(TARGET libmotioncapture PROPERTY POSITION_INDEPENDENT_CODE ON)

if (LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS)
  # Python bindings
  find_package (Python3 COMPONENTS Interpreter Development)

  add_subdirectory(deps/pybind11 EXCLUDE_FROM_ALL)
  # find_package(Python COMPONENTS Interpreter Development)
  # find_package(pybind11 CONFIG)

  pybind11_add_module(motioncapture
    src/python_bindings.cpp
  )

  target_link_libraries(motioncapture
    PRIVATE
      libmotioncapture
  )
endif()

if (LIBMOTIONCAPTURE_BUILD_EXAMPLE)
  add_executable(motioncapture_example
    examples/main.cpp
  )
  target_link_libraries(motioncapture_example
    libmotioncapture
  )

endif()
