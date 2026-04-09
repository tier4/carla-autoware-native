cmake_minimum_required(VERSION 3.18)
project(RclcppBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(rclcpp REQUIRED)

add_library(RclcppBridge SHARED RclcppBridge.cpp)
target_include_directories(RclcppBridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(RclcppBridge PRIVATE rclcpp::rclcpp)

set_target_properties(RclcppBridge PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)
