// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <optional>
#include <memory>
#include <mutex>
#include "CarlaSubscriber.h"
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

template <typename Message, typename MessagePubSubType>
class AutowareSubscriber : public CarlaSubscriber {
public:
  AutowareSubscriber(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "");
  ~AutowareSubscriber();
  AutowareSubscriber(const AutowareSubscriber&) = default;
  AutowareSubscriber& operator=(const AutowareSubscriber&) = default;
  AutowareSubscriber(AutowareSubscriber&&) = default;
  AutowareSubscriber& operator=(AutowareSubscriber&&) = default;

  bool HasNewMessage();
  Message GetMessage();

  bool Init(const TopicConfig& config);

  virtual const char* type() const = 0;

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace ros2
}  // namespace carla
