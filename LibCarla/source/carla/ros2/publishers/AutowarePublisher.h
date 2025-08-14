// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <mutex>

#include "carla/ros2/publishers/CarlaPublisher.h"

namespace carla {
namespace ros2 {

class AutowarePublisher
{
public:
  AutowarePublisher(void* vehicle, const CarlaPublisher::DomainId domain_id);

  void SetVelocity(const int32_t seconds, const uint32_t nanoseconds,
    const float longitudinal_velocity, const float lateral_velocity, const float heading_rate);

  void Publish();

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace carla
}  // namespace ros2
