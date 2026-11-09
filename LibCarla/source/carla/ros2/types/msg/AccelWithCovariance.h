// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <array>

#include "carla/ros2/types/msg/Accel.h"

namespace carla {
namespace ros2 {
namespace msg {

// geometry_msgs/AccelWithCovariance (tier4: sensorless Autoware localization).
struct AccelWithCovariance {
  Accel accel;
  std::array<double, 36> covariance = {};
};

} // namespace msg
} // namespace ros2
} // namespace carla
