// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <vector>

#include "CarlaPublisher.h"
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

  struct CarlaLidarPublisherImpl;

  class CarlaLidarPublisher : public CarlaPublisher {
    public:
      CarlaLidarPublisher(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "");
      ~CarlaLidarPublisher();
      CarlaLidarPublisher(const CarlaLidarPublisher&);
      CarlaLidarPublisher& operator=(const CarlaLidarPublisher&);
      CarlaLidarPublisher(CarlaLidarPublisher&&);
      CarlaLidarPublisher& operator=(CarlaLidarPublisher&&);

      bool Init(const TopicConfig& config);
      bool Publish();
      void SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, float* data);
      const char* type() const override { return "lidar"; }

    private:
      void SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data);

    private:
      std::shared_ptr<CarlaLidarPublisherImpl> _impl;
  };
}
}
