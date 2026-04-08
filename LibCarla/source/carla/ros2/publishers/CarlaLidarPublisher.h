// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
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
      void SetData(const int32_t seconds, const uint32_t nanoseconds, const size_t height, const size_t width, float * data);
      void SetDataEx(const int32_t seconds, const uint32_t nanoseconds, const size_t height, const size_t width,
                     float * data, const size_t header_size, uint32_t * header_data, const std::vector<float> & vertical_angles);

      const char* type() const override { return "lidar"; }

    private:
      void ConvertToRosFormat(const size_t height, const size_t width, float * data);
      void SetData(const int32_t seconds, const uint32_t nanoseconds, const size_t height, const size_t width, std::vector<uint8_t>&& data);

    private:
      std::shared_ptr<CarlaLidarPublisherImpl> _impl;
  };
}
}
