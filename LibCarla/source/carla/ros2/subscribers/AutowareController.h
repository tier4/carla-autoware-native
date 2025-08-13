// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/ROS2CallbackData.h"
#include "carla/ros2/subscribers/AutowareSubscriber.h"

namespace carla {
namespace ros2 {

class AutowareController {
public:
  AutowareController(void* actor, const CarlaSubscriber::DomainId domain_id);

  bool HasNewControl() const;

  VehicleAckermannControl GetControl();
  void* GetVehicle();

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace ros2
}  // namespace carla
