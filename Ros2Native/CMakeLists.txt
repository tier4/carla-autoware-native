cmake_minimum_required (
  VERSION
  ${CARLA_CMAKE_MINIMUM_REQUIRED_VERSION}
)

project (carla-ros2-native-project)

include (ExternalProject)

set (PROJECT_INSTALL_PATH ${CMAKE_CURRENT_BINARY_DIR}/install)

if (CARLA_DDS_VENDOR STREQUAL "FastDDS")

  ExternalProject_add (
    foonathan_memory
    URL https://github.com/eProsima/foonathan_memory_vendor/archive/refs/heads/${CARLA_FOONATHAN_MEMORY_VENDOR_TAG}.zip
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
      -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
      -DBUILD_SHARED_LIBS=ON
      -DFOONATHAN_MEMORY_FORCE_VENDORED_BUILD=ON
  )

  # Note: We are using GIT_REPOSITORY instead URL for fastdds because fastdss contains submodules
  #       and we need the .git folders to download the submodules
  ExternalProject_add (
    fastdds
    GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
    GIT_TAG ${CARLA_FASTDDS_TAG}
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
      -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
      -DBUILD_SHARED_LIBS=ON
      -DBUILD_TESTING=OFF
      -DCOMPILE_EXAMPLES=OFF
      -DCOMPILE_TOOLS=OFF
      -DTHIRDPARTY_Asio=FORCE
      -DTHIRDPARTY_fastcdr=FORCE
      -DTHIRDPARTY_TinyXML2=FORCE
      -DSQLITE3_SUPPORT=OFF
      -DOPENSSL_FOUND:BOOL=ON
      -DOPENSSL_INCLUDE_DIR:FILEPATH=${UE_OPENSSL_INCLUDE}
      -DOPENSSL_SSL_LIBRARY:FILEPATH=${UE_OPENSSL_LIBS}/libssl.a
      -DOPENSSL_CRYPTO_LIBRARY:FILEPATH=${UE_OPENSSL_LIBS}/libcrypto.a
    DEPENDS foonathan_memory
  )

  set (DDS_TARGET fastdds)

elseif (CARLA_DDS_VENDOR STREQUAL "CycloneDDS")

  ExternalProject_add (
    cyclonedds
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
    GIT_TAG ${CARLA_CYCLONEDDS_TAG}
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
      -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
      -DBUILD_SHARED_LIBS=ON
      -DBUILD_EXAMPLES=OFF
      -DBUILD_TESTING=OFF
      -DBUILD_DDSPERF=OFF
      -DENABLE_SSL=OFF
      -DENABLE_SECURITY=OFF
  )

  set (DDS_TARGET cyclonedds)

else()
  message(FATAL_ERROR "Unknown CARLA_DDS_VENDOR: ${CARLA_DDS_VENDOR}")
endif()

ExternalProject_Add (
  carla-ros2-native-lib
  DEPENDS ${DDS_TARGET}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/LibCarlaRos2Native
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCARLA_DDS_VENDOR=${CARLA_DDS_VENDOR}
)

set (CARLA_PLUGIN_BINARY_PATH ${CMAKE_SOURCE_DIR}/Unreal/CarlaUnreal/Plugins/Carla/Binaries/Linux)

make_directory (${CARLA_PLUGIN_BINARY_PATH})

add_custom_command (
  TARGET carla-ros2-native-lib
  POST_BUILD
  COMMAND
    ${CMAKE_COMMAND} -E copy
    ${PROJECT_INSTALL_PATH}/lib/*.so*
    ${CARLA_PLUGIN_BINARY_PATH}
)

carla_add_custom_target (
  carla-ros2-native
  "Build the ROS2-Native CARLA subproject."
  DEPENDS carla-ros2-native-lib
)
