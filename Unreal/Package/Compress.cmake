file (
  GLOB
  CARLA_PACKAGE_FILES
  ${CARLA_PACKAGE_ARCHIVE_PATH}/*
)

if (WIN32)
  set (
    COMPRESS_PACKAGE_COMMAND
    ${CMAKE_COMMAND}
      -E tar cvf
      ${CARLA_CURRENT_PACKAGE_PATH}.zip
      --format=zip
    ${CARLA_PACKAGE_FILES}
  )
elseif (CARLA_PACKAGE_COMPRESSION STREQUAL "zstd")
  find_program (PZSTD_EXECUTABLE pzstd)
  if (NOT PZSTD_EXECUTABLE)
    message (FATAL_ERROR "pzstd not found. Install zstd (apt install zstd) or use -DCARLA_PACKAGE_COMPRESSION=gzip")
  endif ()
  find_program (TAR_EXECUTABLE tar REQUIRED)
  # Pipe tar stdout directly into pzstd to avoid writing a large intermediate
  # .tar file to disk.  CMake execute_process does not support shell pipes,
  # so we invoke bash -c.
  cmake_path (GET CARLA_PACKAGE_ARCHIVE_PATH FILENAME _ARCHIVE_DIRNAME)
  set (
    COMPRESS_PACKAGE_COMMAND
    bash -c "${TAR_EXECUTABLE} cf - -C '${CARLA_PACKAGE_PATH}' '${_ARCHIVE_DIRNAME}' | '${PZSTD_EXECUTABLE}' -o '${CARLA_CURRENT_PACKAGE_PATH}.tar.zst'"
  )
else ()
  set (
    COMPRESS_PACKAGE_COMMAND
    ${CMAKE_COMMAND}
      -E tar cvfz
      ${CARLA_CURRENT_PACKAGE_PATH}.tar.gz
    ${CARLA_PACKAGE_FILES}
  )
endif ()

message (
  STATUS
  "Running \"${COMPRESS_PACKAGE_COMMAND}\""
)

execute_process (
  COMMAND
    ${COMPRESS_PACKAGE_COMMAND}
  WORKING_DIRECTORY
    ${CARLA_PACKAGE_PATH}
)

if (COMPRESS_PACKAGE_ZSTD_COMMAND)
  message (
    STATUS
    "Running \"${COMPRESS_PACKAGE_ZSTD_COMMAND}\""
  )
  execute_process (
    COMMAND
      ${COMPRESS_PACKAGE_ZSTD_COMMAND}
    WORKING_DIRECTORY
      ${CARLA_PACKAGE_PATH}
  )
endif ()
