// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <memory>
#include <iostream>

#include "CarlaPublisher.h"
#include "carla/ros2/dds/DDSPublisherImpl.h"

/**
 * @brief Template base class for Autoware report publishers.
 *
 * @tparam Message   The ROS2 message type (e.g. autoware_vehicle_msgs::msg::VelocityReport)
 * @tparam PubSubType  The FastDDS PubSubType for Message. Its default constructor exposes
 *                     getName() which returns the type-registry key (e.g.
 *                     "autoware_vehicle_msgs::msg::VelocityReport"). The template is only
 *                     instantiated in AutowarePublisher.cpp where the PubSubType headers
 *                     are already included, so no FastDDS dependency leaks into this header.
 */
namespace carla {
namespace ros2 {

template<typename Message, typename PubSubType>
class AutowarePublisherBase : public CarlaPublisher
{
private:
  struct Implementation
  {
    std::unique_ptr<DDSPublisherImpl> _dds;
    Message _event {};
  };

  std::shared_ptr<Implementation> _impl;

public:
  AutowarePublisherBase(const char* ros_name, const char* parent, const char* ros_topic_name)
  : _impl(std::make_shared<Implementation>())
  {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  ~AutowarePublisherBase() = default;
  AutowarePublisherBase(const AutowarePublisherBase&) = default;
  AutowarePublisherBase& operator=(const AutowarePublisherBase&) = default;
  AutowarePublisherBase(AutowarePublisherBase&&) = default;
  AutowarePublisherBase& operator=(AutowarePublisherBase&&) = default;

  bool Init(const TopicConfig config)
  {
    // Obtain the DDS type-name by constructing a temporary PubSubType instance.
    // This works because AutowarePublisherBase is a template instantiated only in
    // AutowarePublisher.cpp, which already includes all required PubSubType headers.
    _impl->_dds = CreateDDSPublisher(PubSubType().getName());
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += config.suffix;
    if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  void SetData(const Message& message) { _impl->_event = message; }
  Message& Data() { return _impl->_event; }
  const Message& Data() const { return _impl->_event; }

  bool Publish() {
    return _impl->_dds->Write(&_impl->_event);
  }

  virtual const char* type() const = 0;
};

}  // namespace ros2
}  // namespace carla
