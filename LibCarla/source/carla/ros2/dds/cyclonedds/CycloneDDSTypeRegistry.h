// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <dds/dds.h>

namespace carla {
namespace ros2 {

class CycloneDDSTypeRegistry {
public:
    static const dds_topic_descriptor_t* GetDescriptor(const std::string& type_name);
};

} // namespace ros2
} // namespace carla
