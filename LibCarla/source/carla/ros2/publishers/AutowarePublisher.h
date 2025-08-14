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
  void SetSteering(const int32_t seconds, const uint32_t nanoseconds, const float steering_tire_angle);
  void SetControlMode(const int32_t seconds, const uint32_t nanoseconds, const uint8_t mode);
  void SetGear(const int32_t seconds, const uint32_t nanoseconds, const uint8_t gear);
  void SetTurnIndicators(const int32_t seconds, const uint32_t nanoseconds, const uint8_t turn_indicators);
  void SetHazardLights(const int32_t seconds, const uint32_t nanoseconds, const uint8_t hazard_lights);

  void Publish();

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace carla
}  // namespace ros2
