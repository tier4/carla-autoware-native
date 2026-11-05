# RGL PythonAPI. Robotec GPU Lidar sensor presets and their helpers exist only in
# this fork, so upstream's copy list in CopyCarlaAdditionalFiles.cmake does not
# mention them and a package built from it ships without PythonAPI/rgl. Consumers
# that drive the RGL sensors from a packaged server - carla_support.x2
# tools/udp_emit_check.py, for one - need the directory next to PythonAPI/carla.

# A source tree that has been used to run the presets carries __pycache__, which
# would otherwise end up in the package.
file (
  COPY ${CARLA_WORKSPACE_PATH}/PythonAPI/rgl/
  DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/rgl/
  PATTERN __pycache__ EXCLUDE
)
