#pragma once

#include <string>
#include <algorithm>

namespace carla {
namespace ros2 {

/// Sanitize topic name for CycloneDDS.
/// CycloneDDS enforces DDS spec which disallows '.' in topic names.
/// CARLA actor names contain dots (e.g., "vehicle.ue4.ford.crown"),
/// so we replace them with '_'.
inline std::string SanitizeTopicName(const std::string& name) {
    std::string result = name;
    std::replace(result.begin(), result.end(), '.', '_');
    return result;
}

}
}
