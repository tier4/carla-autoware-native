#include "AutowareSubscriber.h"

#include "carla/ros2/dds/DDSSubscriberImpl.h"

namespace carla {
namespace ros2 {

template <typename Message, typename MessagePubSubType>
class AutowareSubscriber<Message, MessagePubSubType>::Implementation {
public:
  std::unique_ptr<DDSSubscriberImpl> _dds;

  std::mutex _mutex;
  Message _data {};
  bool _data_changed {false};
};

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::AutowareSubscriber(
  const char* ros_name, const char* parent, const char* ros_topic_name)
: _impl(std::make_shared<Implementation>())
{
  _name = ros_name;
  _parent = parent;
  _topic_name = ros_topic_name;
}

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::~AutowareSubscriber() = default;

template <typename Message, typename MessagePubSubType>
bool AutowareSubscriber<Message, MessagePubSubType>::HasNewMessage() {
  std::unique_lock lock{_impl->_mutex};
  return _impl->_data_changed;
}

template <typename Message, typename MessagePubSubType>
Message AutowareSubscriber<Message, MessagePubSubType>::GetMessage() {
  std::unique_lock lock{_impl->_mutex};
  _impl->_data_changed = false;
  return _impl->_data;
}

template <typename Message, typename MessagePubSubType>
bool AutowareSubscriber<Message, MessagePubSubType>::Init(const TopicConfig& config) {
  // Obtain the DDS type-name by constructing a temporary PubSubType instance.
  // This works because AutowareSubscriber is a template instantiated only below,
  // which already includes all required PubSubType headers.
  _impl->_dds = CreateDDSSubscriber(MessagePubSubType().getName());
  if (!_impl->_dds) return false;

  std::string base{"rt/carla/"};
  std::string topic_name = base;
  if (!_parent.empty())
    topic_name += _parent + "/";
  topic_name += _name;
  topic_name += config.suffix;

  if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
    topic_name = custom_topic_name.value();
  }

  if (!_impl->_dds->Init(config, _name, topic_name)) {
    return false;
  }

  // Set up callback to replicate the old SubscriberListenerBase behavior:
  // when data arrives, read it and store under lock.
  // Capture _impl by value (shared_ptr) so callback survives moves.
  auto impl = _impl;
  _impl->_dds->SetOnDataCallback([impl]() {
    Message message{};
    if (impl->_dds->TakeNextSample(&message)) {
      std::unique_lock lock{impl->_mutex};
      impl->_data = message;
      impl->_data_changed = true;
    }
  });

  return true;
}

}  // namespace ros2
}  // namespace carla

// Instantiate all types
#include "Control.h"
#include "ControlPubSubTypes.h"
#include "GearCommand.h"
#include "GearCommandPubSubTypes.h"
#include "TurnIndicatorsCommand.h"
#include "TurnIndicatorsCommandPubSubTypes.h"
#include "HazardLightsCommand.h"
#include "HazardLightsCommandPubSubTypes.h"
#include "VehicleEmergencyStamped.h"
#include "VehicleEmergencyStampedPubSubTypes.h"
#include "Engage.h"
#include "EngagePubSubTypes.h"

namespace carla {
namespace ros2 {

template class AutowareSubscriber<autoware_control_msgs::msg::Control,               autoware_control_msgs::msg::ControlPubSubType              >;
template class AutowareSubscriber<autoware_vehicle_msgs::msg::GearCommand,           autoware_vehicle_msgs::msg::GearCommandPubSubType          >;
template class AutowareSubscriber<autoware_vehicle_msgs::msg::TurnIndicatorsCommand, autoware_vehicle_msgs::msg::TurnIndicatorsCommandPubSubType>;
template class AutowareSubscriber<autoware_vehicle_msgs::msg::HazardLightsCommand,   autoware_vehicle_msgs::msg::HazardLightsCommandPubSubType  >;
template class AutowareSubscriber<tier4_vehicle_msgs::msg::VehicleEmergencyStamped,  tier4_vehicle_msgs::msg::VehicleEmergencyStampedPubSubType >;
template class AutowareSubscriber<autoware_vehicle_msgs::msg::Engage,                autoware_vehicle_msgs::msg::EngagePubSubType               >;

}  // namespace ros2
}  // namespace carla
