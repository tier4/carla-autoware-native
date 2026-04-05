#pragma once

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cstdint>

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

/// Get DDS domain ID from ROS_DOMAIN_ID environment variable.
/// Returns 0 if not set or invalid.
inline uint32_t GetDomainId() {
    const char* env = std::getenv("ROS_DOMAIN_ID");
    if (env) {
        char* end = nullptr;
        long val = std::strtol(env, &end, 10);
        if (end != env && val >= 0 && val <= 232) {
            return static_cast<uint32_t>(val);
        }
    }
    return 0;
}

}
}
