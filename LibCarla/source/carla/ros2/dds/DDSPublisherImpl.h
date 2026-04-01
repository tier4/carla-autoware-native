// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <string>
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

class DDSPublisherImpl {
public:
    virtual ~DDSPublisherImpl() = default;

    virtual bool Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name,
        bool use_preallocated_realloc = false
    ) = 0;

    virtual bool Write(void* data) = 0;
    virtual bool IsConnected() const = 0;
    virtual void Destroy() = 0;
};

std::unique_ptr<DDSPublisherImpl> CreateDDSPublisher(const std::string& type_name);

} // namespace ros2
} // namespace carla
