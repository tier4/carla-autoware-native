// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include "FastDDSPublisherImpl.h"
#include "FastDDSSubscriberImpl.h"
#include "FastDDSTypeRegistry.h"

namespace carla {
namespace ros2 {

std::unique_ptr<DDSPublisherImpl> CreateDDSPublisher(const std::string& type_name) {
    auto type_support = FastDDSTypeRegistry::Create(type_name);
    return std::make_unique<FastDDSPublisherImpl>(std::move(type_support));
}

std::unique_ptr<DDSSubscriberImpl> CreateDDSSubscriber(const std::string& type_name) {
    auto type_support = FastDDSTypeRegistry::Create(type_name);
    return std::make_unique<FastDDSSubscriberImpl>(std::move(type_support));
}

} // namespace ros2
} // namespace carla
