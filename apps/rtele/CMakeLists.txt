cmake_minimum_required(VERSION 3.16)
project(rtele VERSION 0.1.0 LANGUAGES CXX C)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(USE_ARM64 "Build for ARM64/aarch64 platform" OFF)
option(BUILD_WITH_ROS2 "Build with ROS2 integration" OFF)

# Detect if we're being built with colcon (ROS2)
if(DEFINED ENV{COLCON_PREFIX_PATH} OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(BUILD_WITH_ROS2 ON)
    message(STATUS "Detected ROS2/colcon build environment")
endif()

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Detect platform if not explicitly set
if(NOT USE_ARM64)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(USE_ARM64 ON)
        message(STATUS "Detected ARM64 platform")
    else()
        message(STATUS "Detected x86_64 platform")
    endif()
endif()

# Include FetchContent module
include(FetchContent)

# Find system dependencies
find_package(Threads REQUIRED)

# ROS2 dependencies (if building with ROS2)
if(BUILD_WITH_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(hdas_msg REQUIRED)
    find_package(system_manager_msg REQUIRED)
    find_package(teleoperation_msg_ros2 REQUIRED)
    
    message(STATUS "Building with ROS2 support")
    add_definitions(-DUSE_ROS2)
endif()

# ================ Abseil C++ ================
message(STATUS "Fetching Abseil C++...")
FetchContent_Declare(
    absl
    GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
    GIT_TAG        20240116.2
    GIT_SHALLOW    TRUE
)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(absl)

# Get Abseil include directory
FetchContent_GetProperties(absl)
if(absl_POPULATED)
    set(ABSL_INCLUDE_DIRS ${absl_SOURCE_DIR})
    message(STATUS "Abseil source dir: ${absl_SOURCE_DIR}")
endif()

# ================ spdlog and fmt (use system packages) ================
# ROS2 provides these, so we use find_package instead of FetchContent
if(BUILD_WITH_ROS2)
    # ROS2 includes spdlog and fmt, no need to fetch
    message(STATUS "Using ROS2-provided spdlog and fmt")
else()
    # For standalone builds, try system packages first
    find_package(spdlog QUIET)
    find_package(fmt QUIET)
    
    if(NOT spdlog_FOUND OR NOT fmt_FOUND)
        message(STATUS "System spdlog/fmt not found, fetching from source...")
        
        # Fetch fmt first (spdlog dependency)
        FetchContent_Declare(
            fmt
            GIT_REPOSITORY https://github.com/fmtlib/fmt.git
            GIT_TAG        10.2.1
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(fmt)
        
        FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG        v1.12.0
            GIT_SHALLOW    TRUE
        )
        set(SPDLOG_BUILD_SHARED OFF)
        set(SPDLOG_FMT_EXTERNAL ON)
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        FetchContent_MakeAvailable(spdlog)
    endif()
endif()

# ================ OpenCV ================
message(STATUS "Fetching OpenCV...")
FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG        4.8.1
    GIT_SHALLOW    TRUE
)
set(BUILD_LIST "core,imgcodecs,imgproc" CACHE STRING "")
set(BUILD_opencv_apps OFF CACHE BOOL "")
set(BUILD_opencv_java OFF CACHE BOOL "")
set(BUILD_opencv_python OFF CACHE BOOL "")
set(BUILD_EXAMPLES OFF CACHE BOOL "")
set(BUILD_TESTS OFF CACHE BOOL "")
set(BUILD_PERF_TESTS OFF CACHE BOOL "")
set(BUILD_DOCS OFF CACHE BOOL "")
set(WITH_CUDA OFF CACHE BOOL "")
set(WITH_OPENCL OFF CACHE BOOL "")
set(WITH_IPP OFF CACHE BOOL "")
set(WITH_TBB OFF CACHE BOOL "")
set(WITH_EIGEN OFF CACHE BOOL "")
set(WITH_V4L OFF CACHE BOOL "")
set(WITH_GTK OFF CACHE BOOL "")
set(WITH_QT OFF CACHE BOOL "")
set(BUILD_JPEG ON CACHE BOOL "")
set(BUILD_PNG ON CACHE BOOL "")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(opencv)

# Get OpenCV properties and set include directories
FetchContent_GetProperties(opencv)
if(opencv_POPULATED)
    # Include both source modules and the generated config directory
    set(OPENCV_INCLUDE_DIRS
        // CMAKE_BINARY_DIR is where opencv_modules.hpp located
        ${CMAKE_BINARY_DIR}
        ${opencv_SOURCE_DIR}/modules/core/include
        ${opencv_SOURCE_DIR}/modules/imgcodecs/include
        ${opencv_SOURCE_DIR}/modules/imgproc/include
        ${opencv_SOURCE_DIR}/modules/calib3d/include
        ${opencv_SOURCE_DIR}/modules/features2d/include
        ${opencv_SOURCE_DIR}/modules/flann/include
        ${opencv_SOURCE_DIR}/modules/highgui/include
        ${opencv_SOURCE_DIR}/modules/imgcodecs/include
        ${opencv_SOURCE_DIR}/modules/photo/include
        ${opencv_SOURCE_DIR}/modules/video/include
        ${opencv_SOURCE_DIR}/modules/videoio/include
    )
    message(STATUS "OpenCV source dir: ${opencv_SOURCE_DIR}")
    message(STATUS "CMake binary dir: ${CMAKE_BINARY_DIR}")
endif()

# VolcEngineRTC SDK
if(USE_ARM64)
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_3.58.1.500_aarch64)
    message(STATUS "Using VolcEngineRTC SDK for ARM64")
else()
    set(VOLCENGINE_RTC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/VolcEngineRTC_Linux_x86_64)
    message(STATUS "Using VolcEngineRTC SDK for x86_64")
endif()

# Check if VolcEngineRTC SDK exists
if(NOT EXISTS ${VOLCENGINE_RTC_DIR})
    message(WARNING "VolcEngineRTC SDK not found at ${VOLCENGINE_RTC_DIR}")
endif()

# VolcEngineRTC include directories and libraries
set(VOLCENGINE_RTC_INCLUDE_DIRS ${VOLCENGINE_RTC_DIR}/include)
set(VOLCENGINE_RTC_LIBRARY_DIRS ${VOLCENGINE_RTC_DIR}/lib)

# Find VolcEngineRTC libraries
find_library(VOLCENGINE_RTC_LIBRARY 
    NAMES VolcEngineRTC bytertc_sdk rtc_sdk
    PATHS ${VOLCENGINE_RTC_LIBRARY_DIRS}
    NO_DEFAULT_PATH
)

if(NOT VOLCENGINE_RTC_LIBRARY)
    message(FATAL_ERROR "VolcEngineRTC library not found in ${VOLCENGINE_RTC_LIBRARY_DIRS}")
else()
    message(STATUS "Found VolcEngineRTC library: ${VOLCENGINE_RTC_LIBRARY}")
endif()

# Find additional VolcEngineRTC extension libraries
file(GLOB VOLCENGINE_RTC_EXTENSIONS "${VOLCENGINE_RTC_LIBRARY_DIRS}/lib*.so")
if(VOLCENGINE_RTC_EXTENSIONS)
    message(STATUS "Found VolcEngineRTC extensions: ${VOLCENGINE_RTC_EXTENSIONS}")
endif()

# Include third-party header-only libraries
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mpmc
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VOLCENGINE_RTC_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(src)

# Install resources
install(DIRECTORY resources/
    DESTINATION share/${PROJECT_NAME}/resources
    PATTERN "*.yuv"
    PATTERN "*.pcm"
    PATTERN "*.jpg"
)

# Print configuration summary
message(STATUS "=== RTele Configuration Summary ===")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Platform: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "ARM64 Build: ${USE_ARM64}")
message(STATUS "ROS2 Build: ${BUILD_WITH_ROS2}")
message(STATUS "VolcEngineRTC SDK: ${VOLCENGINE_RTC_DIR}")
message(STATUS "===================================")

# ROS2 ament_package must be called last
if(BUILD_WITH_ROS2)
    ament_package()
endif()
