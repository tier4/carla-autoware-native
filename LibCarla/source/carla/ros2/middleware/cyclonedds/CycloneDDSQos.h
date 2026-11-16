// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/middleware/QosProfile.h"

#ifndef CARLA_ROS2_MIDDLEWARE_TESTING
#ifdef CARLA_ROS2_MIDDLEWARE_CYCLONEDDS

#include <dds/dds.h>

#include <cstdint>

namespace carla {
namespace ros2 {

/// A QosProfile duration (nanoseconds, QosProfile::kInfinite for none) as a
/// CycloneDDS duration.
inline dds_duration_t ToCycloneDDSDuration(std::int64_t ns) {
  return ns == QosProfile::kInfinite ? DDS_INFINITY : static_cast<dds_duration_t>(ns);
}

/// Writes every policy of @a profile into a CycloneDDS QoS object, for a
/// reader or a writer (rmw_cyclonedds_cpp sets the same policies on both).
/// The reliability max blocking time stays at the one second the CARLA
/// endpoints have always used.
inline void ApplyQosProfile(const QosProfile& profile, dds_qos_t* qos) {
  dds_qset_reliability(qos,
      profile.reliability == QosProfile::Reliability::Reliable
          ? DDS_RELIABILITY_RELIABLE
          : DDS_RELIABILITY_BEST_EFFORT,
      DDS_SECS(1));
  dds_qset_durability(qos,
      profile.durability == QosProfile::Durability::TransientLocal
          ? DDS_DURABILITY_TRANSIENT_LOCAL
          : DDS_DURABILITY_VOLATILE);
  dds_qset_history(qos,
      profile.history == QosProfile::History::KeepLast
          ? DDS_HISTORY_KEEP_LAST
          : DDS_HISTORY_KEEP_ALL,
      profile.history_depth);
  dds_qset_deadline(qos, ToCycloneDDSDuration(profile.deadline_ns));
  dds_qset_lifespan(qos, ToCycloneDDSDuration(profile.lifespan_ns));
  dds_qset_liveliness(qos,
      profile.liveliness == QosProfile::Liveliness::Automatic
          ? DDS_LIVELINESS_AUTOMATIC
          : DDS_LIVELINESS_MANUAL_BY_TOPIC,
      ToCycloneDDSDuration(profile.liveliness_lease_ns));
}

} // namespace ros2
} // namespace carla

#endif // CARLA_ROS2_MIDDLEWARE_CYCLONEDDS
#endif // !CARLA_ROS2_MIDDLEWARE_TESTING
