#!/usr/bin/env python3
"""Visual test for RGL LiDAR presets on Town10HD.

Spawns an RGL LiDAR at 2m height on a road surrounded by buildings,
with the spectator camera positioned 5m behind. LiDAR points are
rendered in the viewport.

Usage:
    python3 test_preset_visual.py VelodyneVLP16
    python3 test_preset_visual.py HesaiAT128E2X --noise --beam-divergence
    python3 test_preset_visual.py --list

    # With return mode:
    python3 test_preset_visual.py VelodyneVLP16 --beam-divergence --return-mode first_last

    # Custom spawn location:
    python3 test_preset_visual.py VelodyneVLP16 --spawn-index 3

Controls:
    Ctrl+C to stop.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import carla

from lidar_models import (
    apply_preset, list_models, set_noise, disable_noise,
    set_beam_divergence, disable_beam_divergence, set_return_mode,
)


def find_urban_spawn_points(world):
    """Find spawn points on roads likely surrounded by buildings.

    Prefers points in the central area of Town10HD where buildings are dense.
    Returns a list of carla.Transform sorted by distance from map center.
    """
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        return []

    # Compute map center from all spawn points
    cx = sum(sp.location.x for sp in spawn_points) / len(spawn_points)
    cy = sum(sp.location.y for sp in spawn_points) / len(spawn_points)

    # Sort by distance from center (urban core)
    def dist_to_center(sp):
        return math.sqrt((sp.location.x - cx)**2 + (sp.location.y - cy)**2)

    return sorted(spawn_points, key=dist_to_center)


def setup_spectator(world, lidar_transform, behind_distance=5.0, height_offset=2.0):
    """Position spectator camera behind and above the LiDAR."""
    spectator = world.get_spectator()

    # Compute position behind the LiDAR based on its yaw
    yaw_rad = math.radians(lidar_transform.rotation.yaw)
    behind_x = lidar_transform.location.x - behind_distance * math.cos(yaw_rad)
    behind_y = lidar_transform.location.y - behind_distance * math.sin(yaw_rad)
    behind_z = lidar_transform.location.z + height_offset

    # Look toward the LiDAR
    dx = lidar_transform.location.x - behind_x
    dy = lidar_transform.location.y - behind_y
    dz = lidar_transform.location.z - behind_z
    pitch = math.degrees(math.atan2(dz, math.sqrt(dx*dx + dy*dy)))
    yaw = math.degrees(math.atan2(dy, dx))

    spectator_tf = carla.Transform(
        carla.Location(x=behind_x, y=behind_y, z=behind_z),
        carla.Rotation(pitch=pitch, yaw=yaw),
    )
    spectator.set_transform(spectator_tf)
    return spectator_tf


def main():
    parser = argparse.ArgumentParser(
        description="Visual test for RGL LiDAR presets on Town10HD")
    parser.add_argument("model", nargs="?", default=None,
                        help="LiDAR model name (e.g. VelodyneVLP16)")
    parser.add_argument("--list", action="store_true",
                        help="List available models and exit")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--noise", action="store_true", default=True,
                        help="Apply preset noise (default: on)")
    parser.add_argument("--no-noise", action="store_true",
                        help="Disable noise")
    parser.add_argument("--beam-divergence", action="store_true",
                        help="Enable beam divergence")
    parser.add_argument("--return-mode", type=str, default=None,
                        help="Return mode (e.g. first, first_last)")
    parser.add_argument("--spawn-index", type=int, default=0,
                        help="Spawn point index (0=closest to center)")
    parser.add_argument("--height", type=float, default=2.0,
                        help="LiDAR height above road (meters)")
    parser.add_argument("--duration", type=float, default=0,
                        help="Run duration in seconds (0=infinite)")
    parser.add_argument("--point-rate", type=float, default=1.0,
                        help="Fraction of points to draw (0.0-1.0)")
    parser.add_argument("--point-lifetime", type=float, default=0.1,
                        help="Point display lifetime in seconds")
    parser.add_argument("--sensor-tick", type=float, default=0.1,
                        help="Sensor tick interval in seconds")
    parser.add_argument("--cam-behind", type=float, default=5.0,
                        help="Spectator distance behind LiDAR (meters)")
    parser.add_argument("--cam-height", type=float, default=2.0,
                        help="Spectator height offset above LiDAR (meters)")
    args = parser.parse_args()

    if args.list:
        print("Available LiDAR models:")
        list_models()
        return

    if args.model is None:
        parser.error("model name is required (or use --list)")

    # Connect
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    # Load Town10HD if needed
    world = client.get_world()
    current_map = world.get_map().name
    if "Town10HD" not in current_map:
        print(f"Current map: {current_map}, loading Town10HD_Opt...")
        world = client.load_world("Town10HD_Opt")
        time.sleep(3)
        print("Map loaded.")
    else:
        print(f"Already on {current_map}")

    # Find spawn point
    spawn_points = find_urban_spawn_points(world)
    if not spawn_points:
        print("ERROR: No spawn points found")
        return

    idx = min(args.spawn_index, len(spawn_points) - 1)
    road_tf = spawn_points[idx]

    lidar_location = carla.Location(
        x=road_tf.location.x,
        y=road_tf.location.y,
        z=road_tf.location.z + args.height,
    )
    lidar_tf = carla.Transform(lidar_location, road_tf.rotation)

    print(f"Spawn point #{idx}: x={lidar_tf.location.x:.1f} "
          f"y={lidar_tf.location.y:.1f} z={lidar_tf.location.z:.1f} "
          f"yaw={lidar_tf.rotation.yaw:.1f}")

    # Configure blueprint
    bp = world.get_blueprint_library().find("sensor.lidar.rgl")

    apply_noise = not args.no_noise
    apply_preset(bp, args.model,
                 apply_noise=apply_noise,
                 apply_beam_divergence=args.beam_divergence)

    bp.set_attribute("sensor_tick", str(args.sensor_tick))

    # Enable viewport point rendering
    bp.set_attribute("rgl_lidar_show_points", "true")
    bp.set_attribute("rgl_lidar_draw_point_rate", str(args.point_rate))
    bp.set_attribute("rgl_lidar_draw_life_time", str(args.point_lifetime))

    # Return mode
    if args.return_mode:
        set_return_mode(bp, args.return_mode)

    # Spawn
    sensor = world.spawn_actor(bp, lidar_tf)
    print(f"Spawned {args.model} (noise={'ON' if apply_noise else 'OFF'}, "
          f"BD={'ON' if args.beam_divergence else 'OFF'}"
          f"{', return=' + args.return_mode if args.return_mode else ''})")

    # Position spectator
    spec_tf = setup_spectator(world, lidar_tf,
                              behind_distance=args.cam_behind,
                              height_offset=args.cam_height)
    print(f"Spectator: x={spec_tf.location.x:.1f} y={spec_tf.location.y:.1f} "
          f"z={spec_tf.location.z:.1f}")

    # Use synchronous mode for stable ticking
    settings = world.get_settings()
    original_sync = settings.synchronous_mode
    original_dt = settings.fixed_delta_seconds
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.sensor_tick
    world.apply_settings(settings)

    print(f"\nRunning (sync mode)... Ctrl+C to stop.")
    print(f"Point cloud is rendered in the CARLA server window.")

    frame_count = 0
    start_time = time.time()
    try:
        while True:
            world.tick()
            frame_count += 1

            if frame_count % 10 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / max(elapsed, 0.001)
                print(f"  frame {frame_count}, {fps:.1f} Hz, {elapsed:.0f}s elapsed",
                      end="\r")

            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                break
    except KeyboardInterrupt:
        print("\nStopping...")

    # Restore settings and cleanup
    settings.synchronous_mode = original_sync
    settings.fixed_delta_seconds = original_dt
    world.apply_settings(settings)

    sensor.destroy()

    elapsed = time.time() - start_time
    print(f"\nSummary: {frame_count} frames in {elapsed:.1f}s "
          f"({frame_count/max(elapsed,0.001):.1f} Hz)")


if __name__ == "__main__":
    main()
