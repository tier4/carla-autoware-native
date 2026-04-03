// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include "CycloneDDSPublisherImpl.h"
#include "CycloneDDSSubscriberImpl.h"
#include "CycloneDDSTypeRegistry.h"

namespace carla {
namespace ros2 {

std::unique_ptr<DDSPublisherImpl> CreateDDSPublisher(const std::string& type_name) {
    auto* descriptor = CycloneDDSTypeRegistry::GetDescriptor(type_name);
    return std::make_unique<CycloneDDSPublisherImpl>(descriptor);
}

std::unique_ptr<DDSSubscriberImpl> CreateDDSSubscriber(const std::string& type_name) {
    auto* descriptor = CycloneDDSTypeRegistry::GetDescriptor(type_name);
    return std::make_unique<CycloneDDSSubscriberImpl>(descriptor);
}

} // namespace ros2
} // namespace carla
