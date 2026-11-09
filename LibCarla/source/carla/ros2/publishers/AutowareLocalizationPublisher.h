// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/AutowareLocalizationConfig.h"
#include "carla/ros2/publishers/BasePublisher.h"

#include <cstdint>
#include <memory>

namespace carla {
namespace ros2 {

// tier4: sensorless Autoware localization (port of tier4 5cf27b660 onto the
// middleware-neutral PublisherImpl). Publishes the ego's ground-truth pose in
// the Autoware map frame:
//   /localization/kinematic_state  nav_msgs/Odometry (map -> base_link)
//   /localization/acceleration     geometry_msgs/AccelWithCovarianceStamped
//   /tf                            map -> base_link
// QoS RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1 per tier4.
class AutowareLocalizationPublisher : public BasePublisher {
public:
  AutowareLocalizationPublisher();
  ~AutowareLocalizationPublisher() override;

  AutowareLocalizationPublisher(const AutowareLocalizationPublisher &) = delete;
  AutowareLocalizationPublisher &operator=(const AutowareLocalizationPublisher &) = delete;

  /// base_link pose: CARLA world location in meters and yaw in degrees.
  /// Velocities: vehicle-local CARLA frame (m/s, rad/s), as in VehicleStatusData.
  bool Write(
      std::int32_t seconds,
      std::uint32_t nanoseconds,
      const AutowareLocalizationConfig &config,
      double x, double y, double z, double yaw_degrees,
      double vel_x, double vel_y, double vel_z,
      double ang_vel_x, double ang_vel_y, double ang_vel_z);

  bool Publish() override;

private:
  class Impl;
  std::shared_ptr<Impl> _impl;
};

}  // namespace ros2
}  // namespace carla
