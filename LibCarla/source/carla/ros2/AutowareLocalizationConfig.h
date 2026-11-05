// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

namespace carla {
namespace ros2 {

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
