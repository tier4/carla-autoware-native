// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/middleware/QosProfile.h"

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastrtps/qos/QosPolicies.h>

#include <cstdint>

namespace carla {
namespace ros2 {

/// A QosProfile duration (nanoseconds, QosProfile::kInfinite for none) as a
/// FastDDS Duration_t.
inline eprosima::fastrtps::Duration_t ToFastDDSDuration(std::int64_t ns) {
  if (ns == QosProfile::kInfinite) {
    return eprosima::fastrtps::c_TimeInfinite;
  }
  return eprosima::fastrtps::Duration_t(
      static_cast<std::int32_t>(ns / 1000000000),
      static_cast<std::uint32_t>(ns % 1000000000));
}

/// Writes every policy of @a qos into a FastDDS DataWriterQos or
/// DataReaderQos. With a finite liveliness lease the announcement period is
/// set to two thirds of the lease, as rmw_fastrtps does: FastDDS rejects a
/// writer whose announcement period is not shorter than its lease.
template <typename EntityQos>
void ApplyQosProfile(const QosProfile& qos, EntityQos& entity) {
  namespace efd = eprosima::fastdds::dds;
  entity.reliability().kind =
      qos.reliability == QosProfile::Reliability::Reliable
          ? efd::RELIABLE_RELIABILITY_QOS
          : efd::BEST_EFFORT_RELIABILITY_QOS;
  entity.durability().kind =
      qos.durability == QosProfile::Durability::TransientLocal
          ? efd::TRANSIENT_LOCAL_DURABILITY_QOS
          : efd::VOLATILE_DURABILITY_QOS;
  entity.history().kind =
      qos.history == QosProfile::History::KeepLast
          ? efd::KEEP_LAST_HISTORY_QOS
          : efd::KEEP_ALL_HISTORY_QOS;
  entity.history().depth = qos.history_depth;
  entity.deadline().period = ToFastDDSDuration(qos.deadline_ns);
  entity.lifespan().duration = ToFastDDSDuration(qos.lifespan_ns);
  entity.liveliness().kind =
      qos.liveliness == QosProfile::Liveliness::Automatic
          ? efd::AUTOMATIC_LIVELINESS_QOS
          : efd::MANUAL_BY_TOPIC_LIVELINESS_QOS;
  entity.liveliness().lease_duration = ToFastDDSDuration(qos.liveliness_lease_ns);
  if (qos.liveliness_lease_ns != QosProfile::kInfinite) {
    entity.liveliness().announcement_period =
        ToFastDDSDuration(qos.liveliness_lease_ns * 2 / 3);
  }
}

} // namespace ros2
} // namespace carla
