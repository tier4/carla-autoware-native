// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <variant>
#include <functional>

#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

  struct VehicleControl
  {
    float   throttle;
    float   steer;
    float   brake;
    bool    hand_brake;
    bool    reverse;
    int32_t gear;
    bool    manual_gear_shift;
  };

  struct VehicleAckermannControl
  {
    float steer;
    float steer_speed;
    float speed;
    float acceleration;
    float jerk;
  };

  // VehicleAccelerationControl is defined in data_types.h

    struct MessageControl
  {
    const char* message;
  };

  using ROS2CallbackData = std::variant<VehicleControl, VehicleAckermannControl, VehicleAccelerationControl>;
  using ROS2MessageCallbackData = std::variant<MessageControl>;

  using ActorCallback = std::function<void(void *actor, ROS2CallbackData data)>;
  using ActorMessageCallback = std::function<void(void *actor, ROS2MessageCallbackData data)>;

} // namespace ros2
} // namespace carla
