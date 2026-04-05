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

if (CARLA_DDS_VENDOR STREQUAL "FastDDS")
  file (
    GLOB
    LIBCARLA_ROS2_SOURCES
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/listeners/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/types/*.cpp
  )
  file (
    GLOB
    LIBCARLA_ROS2_HEADERS
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/listeners/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/types/*.h
  )
elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")
  file (
    GLOB
    LIBCARLA_ROS2_SOURCES
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/publishers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/subscribers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/types/*.c
  )
  file (
    GLOB
    LIBCARLA_ROS2_HEADERS
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/types/*.h
  )
endif()

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
    ${CMAKE_INSTALL_PREFIX}/include
  )
elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")
  target_include_directories (
    carla-ros2-native
    SYSTEM PRIVATE
    ${LIBCARLA_SOURCE_PATH}
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/publishers
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/cyclonedds/types
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
