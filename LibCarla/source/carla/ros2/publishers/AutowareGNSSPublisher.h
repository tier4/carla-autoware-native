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
class AutowareGNSSPublisher : public CarlaPublisher {
  public:
    AutowareGNSSPublisher(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "");
    ~AutowareGNSSPublisher() = default;
    AutowareGNSSPublisher(const AutowareGNSSPublisher&);
    AutowareGNSSPublisher& operator=(const AutowareGNSSPublisher&);
    AutowareGNSSPublisher(AutowareGNSSPublisher&&);
    AutowareGNSSPublisher& operator=(AutowareGNSSPublisher&&);

    bool Init(const TopicConfig& pose_config, const TopicConfig& pose_config_with_covariance_stamped);
    bool Publish();
    void SetData(int32_t seconds, uint32_t nanoseconds, const double* position, const double* orientation);
    const char* type() const override { return "Autoware GNSS"; }

  private:
    class Implementation;
    std::shared_ptr<Implementation> _impl;
};
}
}
