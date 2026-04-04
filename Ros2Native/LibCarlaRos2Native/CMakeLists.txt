cmake_minimum_required (
  VERSION
  3.27.2
)

project (
  carla-ros2-native
)

set (
  LIBCARLA_SOURCE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../../LibCarla/source
)

file (
  GLOB
  LIBCARLA_ROS2_COMMON_SOURCES
  ${LIBCARLA_SOURCE_PATH}/carla/ros2/*.cpp
)

file (
  GLOB
  LIBCARLA_ROS2_COMMON_HEADERS
  ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.h
  ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.h
)

if (CARLA_DDS_VENDOR STREQUAL "FastDDS")
  file (
    GLOB
    LIBCARLA_ROS2_DDS_SOURCES
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/types/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/publishers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/subscribers/*.cpp
  )
  file (
    GLOB
    LIBCARLA_ROS2_DDS_HEADERS
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/*.hpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/types/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/publishers/*.hpp
  )
elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")
  file (
    GLOB
    LIBCARLA_ROS2_DDS_SOURCES
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/types/*.c
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/publishers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/subscribers/*.cpp
  )
  file (
    GLOB
    LIBCARLA_ROS2_DDS_HEADERS
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/*.hpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/types/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/publishers/*.hpp
  )
endif()

set (
  LIBCARLA_ROS2_SOURCES
  ${LIBCARLA_ROS2_COMMON_SOURCES}
  ${LIBCARLA_ROS2_DDS_SOURCES}
)

set (
  LIBCARLA_ROS2_HEADERS
  ${LIBCARLA_ROS2_COMMON_HEADERS}
  ${LIBCARLA_ROS2_DDS_HEADERS}
)

add_library (
  carla-ros2-native
  SHARED
  ${LIBCARLA_ROS2_HEADERS}
  ${LIBCARLA_ROS2_SOURCES}
)

if (CARLA_DDS_VENDOR STREQUAL "FastDDS")
  target_include_directories (
    carla-ros2-native
    SYSTEM PRIVATE
    ${LIBCARLA_SOURCE_PATH}
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/fastdds/types
    ${CMAKE_INSTALL_PREFIX}/include
  )
elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")
  target_include_directories (
    carla-ros2-native
    SYSTEM PRIVATE
    ${LIBCARLA_SOURCE_PATH}
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/dds/cyclonedds/types
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/ddsc
  )
endif()

target_compile_definitions (
  carla-ros2-native
  PUBLIC
  BOOST_ASIO_ENABLE_BUFFER_DEBUGGING
)

if (CARLA_DDS_VENDOR STREQUAL "FastDDS")
  target_link_libraries (
    carla-ros2-native PUBLIC
    ${CMAKE_INSTALL_PREFIX}/lib/libfastrtps.so
  )
elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")
  target_link_libraries (
    carla-ros2-native PUBLIC
    ${CMAKE_INSTALL_PREFIX}/lib/libddsc.so
  )
endif()

install (
  TARGETS
  carla-ros2-native
  DESTINATION
  lib
)
