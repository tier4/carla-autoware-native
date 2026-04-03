// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#pragma once

#include <memory>
#include <mutex>

namespace carla {
namespace ros2 {

template<typename Message>
class SubscriberListenerBase {
public:
  struct Data {
    std::mutex mutex;
    Message data;
    bool data_changed;
  };

  SubscriberListenerBase(Data* owner_data);
  ~SubscriberListenerBase();

  SubscriberListenerBase(const SubscriberListenerBase&) = delete;
  SubscriberListenerBase& operator=(const SubscriberListenerBase&) = delete;
  SubscriberListenerBase(SubscriberListenerBase&&) = delete;
  SubscriberListenerBase& operator=(SubscriberListenerBase&&) = delete;

  void SetOwnerData(Data* owner_data);

  class Implementation;
  std::unique_ptr<Implementation> _impl;
};

}  // namespace ros2
}  // namespace carla
