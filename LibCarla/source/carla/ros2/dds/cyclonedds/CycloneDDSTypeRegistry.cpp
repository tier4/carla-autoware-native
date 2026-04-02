// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CycloneDDSTypeRegistry.h"

#include <iostream>

// Type descriptors will be added when CycloneDDS types are generated (Task 15)
// For now, return nullptr for all types

namespace carla {
namespace ros2 {

const dds_topic_descriptor_t* CycloneDDSTypeRegistry::GetDescriptor(const std::string& type_name) {
    // TODO: Task 15 will add idlc-generated type descriptors here
    std::cerr << "CycloneDDSTypeRegistry: type '" << type_name
              << "' not yet available (pending IDL generation)" << std::endl;
    return nullptr;
}

} // namespace ros2
} // namespace carla
