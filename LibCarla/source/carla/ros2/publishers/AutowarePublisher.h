// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <mutex>

#include "carla/ros2/publishers/CarlaPublisher.h"

namespace carla {
namespace ros2 {

// https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg
enum class ControlMode {
  NO_COMMAND,
  AUTONOMOUS,
  AUTONOMOUS_STEER_ONLY,
  AUTONOMOUS_VELOCITY_ONLY,
  MANUAL,
  DISENGAGED,
  NOT_READY
};

// https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg
enum class Gear {
  NONE,
  NEUTRAL,
  DRIVE,
  DRIVE_2,
  DRIVE_3,
  DRIVE_4,
  DRIVE_5,
  DRIVE_6,
  DRIVE_7,
  DRIVE_8,
  DRIVE_9,
  DRIVE_10,
  DRIVE_11,
  DRIVE_12,
  DRIVE_13,
  DRIVE_14,
  DRIVE_15,
  DRIVE_16,
  DRIVE_17,
  DRIVE_18,
  REVERSE,
  REVERSE_2,
  PARK,
  LOW,
  LOW_2
};

enum class TurnIndicatorsStatus {
  OFF,
  LEFT,
  RIGHT
};

class AutowarePublisher
{
public:
  AutowarePublisher(void* vehicle, const DomainId domain_id);
  ~AutowarePublisher();

  void SetVelocity(const float longitudinal_velocity, const float lateral_velocity, const float heading_rate);
  void SetSteering(const float steering_tire_angle);
  void SetControlMode(const ControlMode mode);
  void SetGear(const Gear gear);
  void SetTurnIndicators(const TurnIndicatorsStatus status);
  void SetHazardLights(const bool hazard_lights);

  void Publish(const int32_t seconds, const uint32_t nanoseconds);

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace carla
}  // namespace ros2
