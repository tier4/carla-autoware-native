// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "CarlaPublisher.h"

namespace carla {
namespace ros2 {

/**
 * @note This class has to inherit from CarlaPublisher only to be inserted in ROS2::_publishers map.
 * The functionality of CarlaPublisher is used internally.
 */
class CarlaPoseStampedPublisher : public CarlaPublisher {
  public:
    CarlaPoseStampedPublisher(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "");
    ~CarlaPoseStampedPublisher();
    CarlaPoseStampedPublisher(const CarlaPoseStampedPublisher&);
    CarlaPoseStampedPublisher& operator=(const CarlaPoseStampedPublisher&);
    CarlaPoseStampedPublisher(CarlaPoseStampedPublisher&&);
    CarlaPoseStampedPublisher& operator=(CarlaPoseStampedPublisher&&);

    bool Init(DomainId domain_id);
    bool Init(const TopicConfig& config);
    bool Publish();
    void SetData(int32_t seconds, uint32_t nanoseconds, const double* position, const double* orientation);
    const char* type() const override { return "pose stamped"; }

  private:
    class Implementation;
    std::shared_ptr<Implementation> _impl;
};
}
}
