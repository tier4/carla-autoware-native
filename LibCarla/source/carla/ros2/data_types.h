// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>

namespace carla {
namespace ros2 {

using DomainId = uint32_t;

enum class ReliabilityQoS { RELIABLE, BEST_EFFORT };

enum class DurabilityQoS { TRANSIENT_LOCAL, VOLATILE };

enum class HistoryQoS { KEEP_LAST, KEEP_ALL };

struct TopicConfig {
  /**
   * Equivalent to topic type, essentially a suffix added after topic name.
   * Has to either start with '/' or be empty.
   */
  std::string suffix{""};

  DomainId domain_id{0U};

  /**
   * QoS profile
   */
  ReliabilityQoS reliability_qos;
  DurabilityQoS durability_qos;
  HistoryQoS history_qos;
  int32_t history_qos_depth;
};

} // namespace ros2
} // namespace carla
