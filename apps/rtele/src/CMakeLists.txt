# Config library
add_library(rtele_config
    config/rtc_config.cc
    config/rtc_config.h
)

target_include_directories(rtele_config PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(rtele_config
    absl::strings
    absl::str_format
)

# Util library
add_library(rtele_util
    util/token_generator.cc
    util/token_generator.h
    util/video_file_reader.cc
    util/video_file_reader.h
)

target_include_directories(rtele_util PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(rtele_util
    absl::strings
)

# RTC library
add_library(rtele_rtc
    rtc/rtc_engine.cc
    rtc/rtc_engine.h
    rtc/message_consumer.cc
    rtc/message_consumer.h
)

target_include_directories(rtele_rtc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VOLCENGINE_RTC_INCLUDE_DIRS}
)

target_link_libraries(rtele_rtc
    rtele_config
    rtele_util
    absl::strings
    absl::str_format
    absl::synchronization
    ${VOLCENGINE_RTC_LIBRARY}
)

# Main executable
add_executable(rtele
    main.cc
)

# Ensure OpenCV is built before rtele
add_dependencies(rtele opencv_core opencv_imgcodecs opencv_imgproc)

target_include_directories(rtele PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ABSL_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
)

# Link libraries - OpenCV targets will automatically provide include directories
target_link_libraries(rtele
    rtele_config
    rtele_rtc
    rtele_util
    absl::flags
    absl::flags_parse
    absl::strings
    absl::str_format
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    Threads::Threads
    ${VOLCENGINE_RTC_LIBRARY}
)

# Link spdlog and fmt
if(BUILD_WITH_ROS2)
    # ROS2 provides spdlog, need to find and link fmt
    find_package(fmt REQUIRED)
    target_link_libraries(rtele fmt::fmt)
    target_link_libraries(rtele_config fmt::fmt)
    target_link_libraries(rtele_util fmt::fmt)
    target_link_libraries(rtele_rtc fmt::fmt)
else()
    # Standalone build uses FetchContent versions
    target_link_libraries(rtele spdlog::spdlog fmt::fmt)
    target_link_libraries(rtele_config spdlog::spdlog fmt::fmt)
    target_link_libraries(rtele_util spdlog::spdlog fmt::fmt)
    target_link_libraries(rtele_rtc spdlog::spdlog fmt::fmt)
endif()

# Add ROS2 dependencies if building with ROS2
if(BUILD_WITH_ROS2)
    ament_target_dependencies(rtele
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        hdas_msg
        system_manager_msg
        teleoperation_msg_ros2
    )
    ament_target_dependencies(rtele_rtc
        rclcpp
        hdas_msg
        system_manager_msg
    )
endif()

# Set output directory
set_target_properties(rtele PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Installation
install(TARGETS rtele
    RUNTIME DESTINATION bin
)

install(TARGETS rtele_config rtele_rtc rtele_util
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY config/ rtc/ util/
    DESTINATION include/rtele
    FILES_MATCHING PATTERN "*.h"
)
