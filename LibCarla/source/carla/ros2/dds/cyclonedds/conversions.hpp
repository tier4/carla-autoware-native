// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "dds/dds.h"
#include "carla/ros2/data_types.h"

/**
 * @brief For internal use only, do not include this in any header that is not internal!!!
 * @note This file has .hpp (rather than .h) extension on purpose to avoid it being included by *.h wildcard in cmake
 */

namespace carla {
namespace ros2 {

inline void configure_cyclone_qos(const TopicConfig& from, dds_qos_t* to) {
    switch (from.reliability_qos) {
        case ReliabilityQoS::RELIABLE:
            dds_qset_reliability(to, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
            break;
        case ReliabilityQoS::BEST_EFFORT:
            dds_qset_reliability(to, DDS_RELIABILITY_BEST_EFFORT, 0);
            break;
    }
    switch (from.durability_qos) {
        case DurabilityQoS::TRANSIENT_LOCAL:
            dds_qset_durability(to, DDS_DURABILITY_TRANSIENT_LOCAL);
            break;
        case DurabilityQoS::VOLATILE:
            dds_qset_durability(to, DDS_DURABILITY_VOLATILE);
            break;
    }
    switch (from.history_qos) {
        case HistoryQoS::KEEP_LAST:
            dds_qset_history(to, DDS_HISTORY_KEEP_LAST, from.history_qos_depth);
            break;
        case HistoryQoS::KEEP_ALL:
            dds_qset_history(to, DDS_HISTORY_KEEP_ALL, 0);
            break;
    }
}

} // namespace ros2
} // namespace carla
