// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include "FastDDSPublisherImpl.h"
#include "FastDDSTypeRegistry.h"

namespace carla {
namespace ros2 {

std::unique_ptr<DDSPublisherImpl> CreateDDSPublisher(const std::string& type_name) {
    auto type_support = FastDDSTypeRegistry::Create(type_name);
    return std::make_unique<FastDDSPublisherImpl>(std::move(type_support));
}

// Subscriber factory placeholder - will be added in Task 8
std::unique_ptr<DDSSubscriberImpl> CreateDDSSubscriber(const std::string& type_name) {
    return nullptr; // TODO: Task 8
}

} // namespace ros2
} // namespace carla
