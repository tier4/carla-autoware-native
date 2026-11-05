// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>
#include <memory>

#include "carla/ros2/AutowareLocalizationConfig.h"
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

struct AutowareLocalizationPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_degrees = 0.0;
};

struct AutowareLocalizationStatus
{
  double vel_x_mps = 0.0;
  double vel_y_mps = 0.0;
  double vel_z_mps = 0.0;
  double ang_vel_x_radps = 0.0;
  double ang_vel_y_radps = 0.0;
  double ang_vel_z_radps = 0.0;
};

class AutowareLocalizationPublisher
{
public:
  explicit AutowareLocalizationPublisher(const DomainId domain_id);
  ~AutowareLocalizationPublisher();

  void SetData(
      const int32_t seconds,
      const uint32_t nanoseconds,
      const AutowareLocalizationConfig &config,
      const AutowareLocalizationPose &base_link_pose,
      const AutowareLocalizationStatus &status);

  bool Publish();

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

} // namespace ros2
} // namespace carla
