// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/types/msg/AccelWithCovariance.h"
#include "carla/ros2/types/msg/Header.h"

namespace carla {
namespace ros2 {
namespace msg {

// geometry_msgs/AccelWithCovarianceStamped (tier4: sensorless Autoware localization).
struct AccelWithCovarianceStamped {
  Header header;
  AccelWithCovariance accel;
};

} // namespace msg
} // namespace ros2
} // namespace carla
