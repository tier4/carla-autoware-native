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
  # Two-step: tar (no compression) then pzstd (parallel, auto-detects CPU cores)
  set (
    COMPRESS_PACKAGE_COMMAND
    ${CMAKE_COMMAND}
      -E tar cvf
      ${CARLA_CURRENT_PACKAGE_PATH}.tar
    ${CARLA_PACKAGE_FILES}
  )
  set (
    COMPRESS_PACKAGE_ZSTD_COMMAND
    ${PZSTD_EXECUTABLE}
      --rm
      ${CARLA_CURRENT_PACKAGE_PATH}.tar
      -o ${CARLA_CURRENT_PACKAGE_PATH}.tar.zst
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
