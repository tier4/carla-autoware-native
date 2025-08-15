// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <type_traits>

#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include "carla/ros2/data_types.h"

/**
 * @brief For internal use only, do not include this in any header that is not internal!!!
 * @note This file has .hpp (rather than .h) extension on purpose to avoid it being included by *.h wildcard in cmake
 */

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;

template<typename T, typename = std::enable_if_t<std::disjunction_v<std::is_same<T, efd::DataReaderQos>, std::is_same<T, efd::DataWriterQos>>>>
void configure_qos(const carla::ros2::TopicConfig &from, T &to) {
  switch (from.reliability_qos) {
    case ReliabilityQoS::RELIABLE:
      to.reliability().kind = efd::RELIABLE_RELIABILITY_QOS;
      break;
    case ReliabilityQoS::BEST_EFFORT:
      to.reliability().kind = efd::BEST_EFFORT_RELIABILITY_QOS;
      break;
  }

  switch (from.durability_qos) {
    case DurabilityQoS::TRANSIENT_LOCAL:
      to.durability().kind = efd::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case DurabilityQoS::VOLATILE:
      to.durability().kind = efd::VOLATILE_DURABILITY_QOS;
      break;
  }

  switch (from.history_qos) {
    case HistoryQoS::KEEP_LAST:
      to.history().kind = efd::KEEP_LAST_HISTORY_QOS;
      break;
    case HistoryQoS::KEEP_ALL:
      to.history().kind = efd::KEEP_ALL_HISTORY_QOS;
      break;
  }

  to.history().depth = from.history_qos_depth;
}

} // namespace ros2
} // namespace carla
