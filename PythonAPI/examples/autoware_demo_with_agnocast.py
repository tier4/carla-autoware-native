#!/usr/bin/env python3
"""
CARLA Autoware Demo with Agnocast Zero-Copy IPC

This script extends autoware_demo.py to work with the Agnocast shared memory
bridge. When CARLA is built with ENABLE_AGNOCAST=ON and launched with --ros2,
Agnocast is automatically enabled and sensor data is written to POSIX shared
memory for zero-copy transfer to Autoware.

Usage:
  1. Start CARLA:     ../../runAgnocastCarla.sh
  2. Start Bridge:    ros2 launch carla_agnocast_bridge carla_agnocast_bridge.launch.xml
  3. Start this demo: python3 autoware_demo_with_agnocast.py [options]
  4. Start Autoware:  ENABLE_AGNOCAST=1 ros2 launch autoware_launch ...

The Agnocast bridge node will automatically detect sensors spawned by this
script via the shared memory registry and create corresponding ROS 2 publishers.
"""

# Re-export everything from autoware_demo so this script can be used as a
# drop-in replacement with the same CLI interface.
from autoware_demo import *
from autoware_demo import main as _autoware_main

import os
import sys


def check_agnocast_shm():
    """Check if Agnocast shared memory registry exists (CARLA is writing to shm)."""
    registry_path = "/dev/shm/carla_agnocast_registry"
    if os.path.exists(registry_path):
        size = os.path.getsize(registry_path)
        log_info(f"Agnocast registry detected: {registry_path} ({size} bytes)")
        return True
    return False


def list_agnocast_sensors():
    """List active Agnocast shared memory segments."""
    shm_dir = "/dev/shm"
    sensors = [f for f in os.listdir(shm_dir) if f.startswith("carla_agnocast_")]
    if sensors:
        log_info(f"Active Agnocast shared memory segments:")
        for s in sorted(sensors):
            size = os.path.getsize(os.path.join(shm_dir, s))
            log_info(f"  {s} ({size / 1024 / 1024:.1f} MB)")
    return sensors


def main():
    log_info("=" * 60)
    log_info("CARLA Autoware Demo with Agnocast Zero-Copy IPC")
    log_info("=" * 60)
    log_info("")
    log_info("Agnocast is auto-enabled when CARLA is built with")
    log_info("ENABLE_AGNOCAST=ON and started with --ros2 flag.")
    log_info("")

    # Run the standard autoware demo (spawns ego + sensors)
    # ROS2::Enable() automatically calls EnableAgnocast() in the C++ side
    _autoware_main()


def post_spawn_check():
    """Called after sensors are spawned to verify Agnocast is working."""
    import time
    time.sleep(1)  # Wait for first frame to be written

    if check_agnocast_shm():
        sensors = list_agnocast_sensors()
        if len(sensors) > 1:  # registry + at least one sensor
            log_info(f"Agnocast is active with {len(sensors) - 1} sensor(s)")
            log_info("")
            log_info("Next steps:")
            log_info("  1. Launch the bridge node:")
            log_info("     ros2 launch carla_agnocast_bridge carla_agnocast_bridge.launch.xml")
            log_info("  2. Verify topics:")
            log_info("     ros2 topic list | grep carla")
            log_info("     ros2 topic hz /carla/sensor_*/image_raw")
        else:
            log_warning("Registry exists but no sensor segments found yet.")
    else:
        log_warning("Agnocast shared memory registry not found.")
        log_warning("Ensure CARLA was built with ENABLE_AGNOCAST=ON and started with --ros2")


if __name__ == '__main__':
    main()
