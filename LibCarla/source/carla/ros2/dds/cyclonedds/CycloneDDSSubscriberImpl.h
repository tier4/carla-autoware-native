// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include <dds/dds.h>

namespace carla {
namespace ros2 {

class CycloneDDSSubscriberImpl : public DDSSubscriberImpl {
public:
    explicit CycloneDDSSubscriberImpl(const dds_topic_descriptor_t* descriptor);
    ~CycloneDDSSubscriberImpl() override;

    bool Init(const TopicConfig& config, const std::string& participant_name,
              const std::string& topic_name) override;
    bool TakeNextSample(void* data) override;
    void Destroy() override;
    void SetOnDataCallback(OnDataCallback callback) override;

private:
    static void on_data_available(dds_entity_t reader, void* arg);

    dds_entity_t _participant { 0 };
    dds_entity_t _reader { 0 };
    dds_entity_t _topic { 0 };
    const dds_topic_descriptor_t* _descriptor;
    bool _initialized { false };
    OnDataCallback _callback;
};

} // namespace ros2
} // namespace carla
