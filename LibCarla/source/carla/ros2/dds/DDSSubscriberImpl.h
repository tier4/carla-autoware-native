// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <string>
#include <functional>
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

class DDSSubscriberImpl {
public:
    virtual ~DDSSubscriberImpl() = default;

    virtual bool Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name
    ) = 0;

    virtual bool TakeNextSample(void* data) = 0;
    virtual void Destroy() = 0;

    using OnDataCallback = std::function<void()>;
    virtual void SetOnDataCallback(OnDataCallback callback) = 0;
};

std::unique_ptr<DDSSubscriberImpl> CreateDDSSubscriber(const std::string& type_name);

} // namespace ros2
} // namespace carla
