cmake_minimum_required(VERSION 3.14 FATAL_ERROR) # Set the minimum required
                                                 # version of CMake

project(
  a3_deploy
  VERSION 1.0.0
  LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Safety predicates throughout the deploy process must observe NaN/Inf. Global
# -ffast-math (specifically finite-math-only) lets the compiler fold those
# checks away, including checks immediately before motor-command publication.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++ Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")

# Library target
add_library(${PROJECT_NAME} INTERFACE)

include(CheckLanguage)

# Optional builds
option(BUILD_SRCS "Build sources" ON)
option(ENABLE_TRT_INFERENCE "Build TensorRT/CUDA inference targets" OFF)
option(ENABLE_RKNN_INFERENCE "Build Rockchip RKNN inference backend" OFF)
option(USE_BUNDLED_ONNXRUNTIME_AARCH64
       "Use repo-local ONNX Runtime package on aarch64/arm64" ON)
set(A3_THIRDPARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty" CACHE PATH
    "A3 vendor dependency root (onnxruntime, RKNN, unitree_sdk2, joint_msgs)")
get_filename_component(A3_THIRDPARTY_ROOT "${A3_THIRDPARTY_ROOT}" ABSOLUTE)
set(A3_UNITREE_SDK2_ROOT "${A3_THIRDPARTY_ROOT}/unitree_sdk2" CACHE PATH
    "Unitree SDK2 root containing CMakeLists.txt and architecture libraries")
get_filename_component(A3_UNITREE_SDK2_ROOT "${A3_UNITREE_SDK2_ROOT}" ABSOLUTE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Threads REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64"
   AND USE_BUNDLED_ONNXRUNTIME_AARCH64)
  set(GS_ONNXRUNTIME_AARCH64_TARBALL
      "${A3_THIRDPARTY_ROOT}/onnxruntime/onnxruntime_gpu_1.19.2_aarch64.tar.gz")
  if(EXISTS "${GS_ONNXRUNTIME_AARCH64_TARBALL}")
    set(GS_ONNXRUNTIME_AARCH64_EXTRACT_DIR
        "${CMAKE_BINARY_DIR}/_deps/onnxruntime_aarch64")
    set(GS_ONNXRUNTIME_AARCH64_PREFIX
        "${GS_ONNXRUNTIME_AARCH64_EXTRACT_DIR}/onnxruntime_gpu_1.19.2")
    set(GS_ONNXRUNTIME_AARCH64_CONFIG_DIR
        "${GS_ONNXRUNTIME_AARCH64_PREFIX}/lib/cmake/onnxruntime")

    if(NOT EXISTS "${GS_ONNXRUNTIME_AARCH64_CONFIG_DIR}/onnxruntimeConfig.cmake")
      file(MAKE_DIRECTORY "${GS_ONNXRUNTIME_AARCH64_EXTRACT_DIR}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${GS_ONNXRUNTIME_AARCH64_TARBALL}"
        WORKING_DIRECTORY "${GS_ONNXRUNTIME_AARCH64_EXTRACT_DIR}"
        RESULT_VARIABLE GS_ONNXRUNTIME_EXTRACT_RESULT)
      if(NOT GS_ONNXRUNTIME_EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR
          "Failed to extract ${GS_ONNXRUNTIME_AARCH64_TARBALL}")
      endif()
    endif()

    set(onnxruntime_DIR "${GS_ONNXRUNTIME_AARCH64_CONFIG_DIR}" CACHE PATH
        "Repo-local aarch64 ONNX Runtime config dir" FORCE)
    set(GS_BUNDLED_ONNXRUNTIME_LIB_DIR
        "${GS_ONNXRUNTIME_AARCH64_PREFIX}/lib" CACHE INTERNAL
        "Repo-local ONNX Runtime library directory")
    list(PREPEND CMAKE_PREFIX_PATH "${GS_ONNXRUNTIME_AARCH64_PREFIX}")
    message(STATUS
      "Using bundled aarch64 ONNX Runtime: ${GS_ONNXRUNTIME_AARCH64_PREFIX}")
  else()
    message(WARNING
      "Bundled aarch64 ONNX Runtime tarball not found: "
      "${GS_ONNXRUNTIME_AARCH64_TARBALL}; falling back to find_package")
  endif()
endif()

find_package(onnxruntime REQUIRED)

if(ENABLE_RKNN_INFERENCE)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(FATAL_ERROR
      "ENABLE_RKNN_INFERENCE is only supported for aarch64/arm64 builds")
  endif()

  set(GS_RKNN_RUNTIME_ROOT
      "${A3_THIRDPARTY_ROOT}/rknn_runtime/2.3.2"
      CACHE PATH "Repo-local RKNN runtime root")
  set(GS_RKNN_INCLUDE_DIR "${GS_RKNN_RUNTIME_ROOT}/include")
  set(GS_RKNN_LIBRARY
      "${GS_RKNN_RUNTIME_ROOT}/lib/aarch64/librknnrt.so")

  if(NOT EXISTS "${GS_RKNN_INCLUDE_DIR}/rknn_api.h")
    message(FATAL_ERROR "RKNN header not found: ${GS_RKNN_INCLUDE_DIR}/rknn_api.h")
  endif()
  if(NOT EXISTS "${GS_RKNN_LIBRARY}")
    message(FATAL_ERROR "RKNN runtime library not found: ${GS_RKNN_LIBRARY}")
  endif()

  add_library(rknnrt::rknnrt SHARED IMPORTED GLOBAL)
  set_target_properties(rknnrt::rknnrt PROPERTIES
    IMPORTED_LOCATION "${GS_RKNN_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GS_RKNN_INCLUDE_DIR}")
  set(GS_RKNN_RUNTIME_LIB_DIR
      "${GS_RKNN_RUNTIME_ROOT}/lib/aarch64" CACHE INTERNAL
      "Repo-local RKNN runtime library directory")
  message(STATUS "Using bundled RKNN runtime: ${GS_RKNN_RUNTIME_ROOT}")
endif()

if(ENABLE_TRT_INFERENCE)
find_package(TensorRT REQUIRED)
# Find CUDA Toolkit with proper environment variable support
if(DEFINED ENV{CUDAToolkit_ROOT})
    set(CUDAToolkit_ROOT $ENV{CUDAToolkit_ROOT})
    message(STATUS "🎯 Using CUDAToolkit_ROOT from environment: ${CUDAToolkit_ROOT}")
endif()

if(DEFINED ENV{CUDA_HOME})
    message(STATUS "🔍 Using CUDA_HOME from environment: $ENV{CUDA_HOME}")
    list(APPEND CMAKE_PREFIX_PATH $ENV{CUDA_HOME})
endif()

find_package(CUDAToolkit 10.2 QUIET)

if(CUDAToolkit_FOUND)
    message(STATUS "✅ CUDA Toolkit found: version ${CUDAToolkit_VERSION}")
    
    # Report CUDA architecture and paths for debugging
    message(STATUS "   CUDA Root: ${CUDAToolkit_TARGET_DIR}")
    message(STATUS "   CUDA Libraries: ${CUDAToolkit_LIBRARY_DIR}")
    
    # For CUDA 12.6+, enable newer features if available
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.6")
        message(STATUS "   🚀 CUDA 12.6+ detected - enabling optimized features")
        # Add any CUDA 12.6+ specific optimizations here
        add_compile_definitions(CUDA_12_6_PLUS=1)
    elseif(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.0")
        message(STATUS "   🔧 CUDA 12.x detected")
        add_compile_definitions(CUDA_12_PLUS=1)
    endif()
else()
    # Fallback: Look for CUDA runtime libraries and headers
    message(STATUS "🔍 CUDA Toolkit not found via find_package, searching for runtime libraries...")
    message(STATUS "   This is normal for systems with runtime-only CUDA installations")
    
    # Debug: Show environment variables
    if(DEFINED ENV{CUDAToolkit_ROOT})
        message(STATUS "   🔍 Environment CUDAToolkit_ROOT: $ENV{CUDAToolkit_ROOT}")
    endif()
    if(DEFINED ENV{CUDA_HOME})
        message(STATUS "   🔍 Environment CUDA_HOME: $ENV{CUDA_HOME}")
    endif()
    
    # Enhanced search with better path prioritization - now includes dynamic version detection
    # First, build dynamic path lists for any CUDA versions
    file(GLOB CUDA_SBSA_PATHS "/usr/local/cuda*/targets/sbsa-linux/lib")
    file(GLOB CUDA_AARCH64_PATHS "/usr/local/cuda*/targets/aarch64-linux/lib") 
    file(GLOB CUDA_LIB64_PATHS "/usr/local/cuda*/lib64")
    
    find_library(CUDA_RUNTIME_LIBRARY
        NAMES cudart libcudart
        HINTS
            # Environment variable paths
            $ENV{CUDAToolkit_ROOT}/targets/sbsa-linux/lib
            $ENV{CUDAToolkit_ROOT}/targets/aarch64-linux/lib
            $ENV{CUDAToolkit_ROOT}/lib64
            $ENV{CUDAToolkit_ROOT}/lib
            $ENV{CUDA_HOME}/targets/sbsa-linux/lib
            $ENV{CUDA_HOME}/targets/aarch64-linux/lib
            $ENV{CUDA_HOME}/lib64
            $ENV{CUDA_HOME}/lib
        PATHS
            # Dynamic CUDA version paths (automatically found)
            ${CUDA_SBSA_PATHS}
            ${CUDA_AARCH64_PATHS}
            ${CUDA_LIB64_PATHS}
            # Static fallback paths
            /usr/local/cuda/targets/sbsa-linux/lib
            /usr/local/cuda/targets/aarch64-linux/lib
            /usr/local/cuda/lib64
            /usr/local/cuda/lib
            # System library paths
            /usr/lib/aarch64-linux-gnu
            /usr/lib/x86_64-linux-gnu
            /usr/lib64
            /usr/lib
        NO_DEFAULT_PATH
        )
    
    # Build dynamic include paths for any CUDA versions
    file(GLOB CUDA_SBSA_INCLUDE_PATHS "/usr/local/cuda*/targets/sbsa-linux/include")
    file(GLOB CUDA_AARCH64_INCLUDE_PATHS "/usr/local/cuda*/targets/aarch64-linux/include")
    file(GLOB CUDA_INCLUDE_PATHS "/usr/local/cuda*/include")
    
    find_path(CUDA_INCLUDE_DIR cuda_runtime.h
        HINTS
            # Environment variable paths
            $ENV{CUDAToolkit_ROOT}/targets/sbsa-linux/include
            $ENV{CUDAToolkit_ROOT}/targets/aarch64-linux/include
            $ENV{CUDAToolkit_ROOT}/include
            $ENV{CUDA_HOME}/targets/sbsa-linux/include
            $ENV{CUDA_HOME}/targets/aarch64-linux/include
            $ENV{CUDA_HOME}/include
        PATHS
            # Dynamic CUDA version paths (automatically found)
            ${CUDA_SBSA_INCLUDE_PATHS}
            ${CUDA_AARCH64_INCLUDE_PATHS}
            ${CUDA_INCLUDE_PATHS}
            # Static fallback paths
            /usr/local/cuda/targets/sbsa-linux/include
            /usr/local/cuda/targets/aarch64-linux/include
            /usr/local/cuda/include
            # System include paths
            /usr/include
            /usr/include/cuda
        PATH_SUFFIXES
            cuda
        NO_DEFAULT_PATH
        )
    
    # Debug: Check if files exist in Jetson-specific locations
    message(STATUS "🔍 Manually checking Jetson CUDA locations...")
    
    # Check for libcudart with version number (as found in your system)
    if(EXISTS "/usr/local/cuda/targets/sbsa-linux/lib/libcudart.so.12.6.77")
        message(STATUS "✅ Found libcudart.so.12.6.77 in Jetson sbsa-linux location!")
        set(CUDA_RUNTIME_LIBRARY "/usr/local/cuda/targets/sbsa-linux/lib/libcudart.so.12.6.77")
    elseif(EXISTS "/usr/local/cuda/targets/sbsa-linux/lib")
        # Check if any libcudart exists in sbsa-linux directory
        file(GLOB CUDART_FILES "/usr/local/cuda/targets/sbsa-linux/lib/libcudart*")
        if(CUDART_FILES)
            list(GET CUDART_FILES 0 FIRST_CUDART)
            message(STATUS "✅ Found CUDA runtime in sbsa-linux: ${FIRST_CUDART}")
            set(CUDA_RUNTIME_LIBRARY ${FIRST_CUDART})
        endif()
    elseif(EXISTS "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
        message(STATUS "✅ Found libcudart.so in Jetson aarch64-linux location!")
        set(CUDA_RUNTIME_LIBRARY "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
    elseif(EXISTS "/usr/local/cuda/lib64/libcudart.so")
        message(STATUS "✅ Found libcudart.so in standard location!")
        set(CUDA_RUNTIME_LIBRARY "/usr/local/cuda/lib64/libcudart.so")
    else()
        message(STATUS "❌ libcudart.so not found in expected locations")
    endif()
    
    if(EXISTS "/usr/local/cuda/targets/sbsa-linux/include/cuda_runtime.h")
        message(STATUS "✅ Found cuda_runtime.h in Jetson sbsa-linux location!")
        set(CUDA_INCLUDE_DIR "/usr/local/cuda/targets/sbsa-linux/include")
    elseif(EXISTS "/usr/local/cuda/targets/aarch64-linux/include/cuda_runtime.h")
        message(STATUS "✅ Found cuda_runtime.h in Jetson aarch64-linux location!")
        set(CUDA_INCLUDE_DIR "/usr/local/cuda/targets/aarch64-linux/include")
    elseif(EXISTS "/usr/local/cuda/include/cuda_runtime.h")
        message(STATUS "✅ Found cuda_runtime.h in standard location!")
        set(CUDA_INCLUDE_DIR "/usr/local/cuda/include")
    else()
        message(STATUS "❌ cuda_runtime.h not found in expected locations")
    endif()
    
    if(CUDA_RUNTIME_LIBRARY AND CUDA_INCLUDE_DIR)
        message(STATUS "✅ CUDA runtime components verified")
        
        # Try to detect CUDA version from headers
        if(EXISTS "${CUDA_INCLUDE_DIR}/cuda.h")
            file(STRINGS "${CUDA_INCLUDE_DIR}/cuda.h" CUDA_VERSION_DEFINES
                REGEX "#define CUDA_VERSION ")
            if(CUDA_VERSION_DEFINES MATCHES "#define CUDA_VERSION ([0-9]+)")
                math(EXPR CUDA_VERSION_MAJOR "${CMAKE_MATCH_1} / 1000")
                math(EXPR CUDA_VERSION_MINOR "(${CMAKE_MATCH_1} % 1000) / 10")
                set(CUDA_VERSION_STRING "${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}")
                message(STATUS "   Detected CUDA version: ${CUDA_VERSION_STRING}")
                
                # Enable version-specific features
                if(CUDA_VERSION_MAJOR GREATER_EQUAL 12 AND CUDA_VERSION_MINOR GREATER_EQUAL 6)
                    message(STATUS "   🚀 CUDA 12.6+ detected - enabling optimized features")
                    add_compile_definitions(CUDA_12_6_PLUS=1)
                elseif(CUDA_VERSION_MAJOR GREATER_EQUAL 12)
                    message(STATUS "   🔧 CUDA 12.x detected")
                    add_compile_definitions(CUDA_12_PLUS=1)
                endif()
            endif()
        endif()
        
        # Create imported target for compatibility with modern CMake
        add_library(CUDA::cudart SHARED IMPORTED)
        set_target_properties(CUDA::cudart PROPERTIES
            IMPORTED_LOCATION "${CUDA_RUNTIME_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CUDA_INCLUDE_DIR}")
    else()
        message(STATUS "❌ CUDA components not found:")
        if(NOT CUDA_RUNTIME_LIBRARY)
            message(STATUS "   - CUDA runtime library (libcudart) not found")
        endif()
        if(NOT CUDA_INCLUDE_DIR)
            message(STATUS "   - CUDA headers (cuda_runtime.h) not found")
        endif()
        message(FATAL_ERROR "
CUDA installation incomplete.

Please install the CUDA/TensorRT development packages required by your target
platform, then rerun CMake. For Jetson systems with CUDA 12.6:
  - Ensure JetPack SDK development components are installed.
  - Check whether CUDA is installed at /usr/local/cuda-12.6/.
  - Run 'ls -la /usr/local/cuda*' to verify CUDA installation.

See README.md for the supported build dependencies and target-specific
packaging flow.")
    endif()
endif()
else()
  message(STATUS "TensorRT/CUDA inference disabled (ENABLE_TRT_INFERENCE=OFF)")
  set(CUDA_LINK_DIRS "")
  set(CUDLA_LIBRARY "")
endif()
find_package(ZLIB REQUIRED)

# Use local unitree_sdk2 subfolder
# Disable building examples for unitree_sdk2
set(BUILD_EXAMPLES
    OFF
    CACHE BOOL "Build unitree_sdk2 examples" FORCE)

if(NOT EXISTS "${A3_UNITREE_SDK2_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR
    "Unitree SDK2 is missing. Source setup_a3_env.sh before configuring, or "
    "set A3_UNITREE_SDK2_ROOT to a complete Unitree SDK2 checkout.")
endif()
add_subdirectory("${A3_UNITREE_SDK2_ROOT}"
                 "${CMAKE_BINARY_DIR}/thirdparty/unitree_sdk2")

# Add unitree_sdk2 include directories
target_include_directories(${PROJECT_NAME}
                           INTERFACE ${A3_UNITREE_SDK2_ROOT}/include)

# Link to unitree_sdk2 which includes all necessary dependencies
target_link_libraries(${PROJECT_NAME} INTERFACE unitree_sdk2)

# Examples
if(BUILD_SRCS)
  add_subdirectory(src)
endif()
