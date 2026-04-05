// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/ros2/ROS2CallbackData.h"
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

class AutowareController {
public:
  AutowareController(void* actor, const DomainId domain_id);
  ~AutowareController();

  bool HasNewControl() const;

  VehicleAccelerationControl GetControl();
  void* GetVehicle();

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace ros2
}  // namespace carla
