// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include <dds/dds.h>

namespace carla {
namespace ros2 {

class CycloneDDSPublisherImpl : public DDSPublisherImpl {
public:
    explicit CycloneDDSPublisherImpl(const dds_topic_descriptor_t* descriptor);
    ~CycloneDDSPublisherImpl() override;

    bool Init(const TopicConfig& config, const std::string& participant_name,
              const std::string& topic_name, bool use_preallocated_realloc) override;
    bool Write(void* data) override;
    bool IsConnected() const override;
    void Destroy() override;

private:
    dds_entity_t _participant { 0 };
    dds_entity_t _writer { 0 };
    dds_entity_t _topic { 0 };
    const dds_topic_descriptor_t* _descriptor;
    bool _initialized { false };
};

} // namespace ros2
} // namespace carla
