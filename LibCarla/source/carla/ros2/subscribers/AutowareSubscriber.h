// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <optional>
#include <memory>
#include <mutex>
#include "CarlaSubscriber.h"
#include "carla/ros2/listeners/SubscriberListenerBase.h"

namespace carla {
namespace ros2 {

struct AutowareSubscriberConfig {
  CarlaSubscriber::DomainId domain_id{0U};
  std::string publisher_type{ "" };  // Has to either start with '/' or be empty

  enum class ReliabilityQoS { RELIABLE, BEST_EFFORT };
  ReliabilityQoS reliability_qos;

  enum class DurabilityQoS { TRANSIENT_LOCAL, VOLATILE };
  DurabilityQoS durability_qos;

  enum class HistoryQoS { KEEP_LAST, KEEP_ALL };
  HistoryQoS history_qos;

  int32_t history_qos_depth;
};


template <typename Message, typename MessagePubSubType>
class AutowareSubscriber : public CarlaSubscriber {
public:
  AutowareSubscriber(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "");
  ~AutowareSubscriber();
  AutowareSubscriber(const AutowareSubscriber& other);
  AutowareSubscriber& operator=(const AutowareSubscriber& other);
  AutowareSubscriber(AutowareSubscriber&& other);
  AutowareSubscriber& operator=(AutowareSubscriber&& other);

  bool HasNewMessage();
  Message GetMessage();

  bool Init(const AutowareSubscriberConfig& config);

  virtual const char* type() const = 0;

private:
  class Implementation;
  std::shared_ptr<Implementation> _impl;
};

}  // namespace ros2
}  // namespace carla
