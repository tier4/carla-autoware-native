// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

namespace carla {
namespace ros2 {

/// tier4: sensorless Autoware localization ("ground truth" pose published
/// instead of running Autoware's localization stack). Filled from the
/// sensor.other.vehicle_status attributes. Disabled by default.
///
/// The CARLA world pose of the ego is mapped into the Autoware (lanelet) map
/// frame through one anchor pair (a map pose and the CARLA pose of the same
/// point) plus a scale and two yaw corrections. The anchor is map specific, so
/// the defaults are neutral and the values are passed per spawn.
struct AutowareLocalizationConfig
{
  bool enabled = false;

  double reference_map_x = 0.0;
  double reference_map_y = 0.0;
  double reference_map_z = 0.0;
  double reference_map_yaw = 0.0;

  double reference_carla_base_x = 0.0;
  double reference_carla_base_y = 0.0;
  double reference_carla_base_z = 0.0;
  double reference_carla_base_yaw = 0.0;

  double map_to_carla_scale = 1.0;
  double map_to_carla_xy_yaw = 0.0;
  double map_to_carla_yaw = 0.0;
};

} // namespace ros2
} // namespace carla
