#include "AutowareSubscriber.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/qos/QosPolicies.h>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>

#include "carla/ros2/util/conversions.hpp"

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;

template <typename Message, typename MessagePubSubType>
class AutowareSubscriber<Message, MessagePubSubType>::Implementation {
public:
  efd::DomainParticipant* _participant{nullptr};
  efd::Subscriber* _subscriber{nullptr};
  efd::Topic* _topic{nullptr};
  efd::DataReader* _datareader{nullptr};
  efd::TypeSupport _type{new MessagePubSubType()};

  typename SubscriberListenerBase<Message>::Data _data;
  SubscriberListenerBase<Message> _listener{&_data};
};

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::AutowareSubscriber(
  const char* ros_name, const char* parent, const char* ros_topic_name)
: _impl(std::make_shared<Implementation>())
{
  _impl->_listener.SetOwnerData(&_impl->_data);
  _name = ros_name;
  _parent = parent;
  _topic_name = ros_topic_name;
}

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::~AutowareSubscriber()
{
  if (!_impl)
    return;

  if (_impl->_datareader)
    _impl->_subscriber->delete_datareader(_impl->_datareader);

  if (_impl->_subscriber)
    _impl->_participant->delete_subscriber(_impl->_subscriber);

  if (_impl->_topic)
    _impl->_participant->delete_topic(_impl->_topic);

  if (_impl->_participant)
    efd::DomainParticipantFactory::get_instance()->delete_participant(_impl->_participant);
}

template <typename Message, typename MessagePubSubType>
bool AutowareSubscriber<Message, MessagePubSubType>::HasNewMessage() {
  std::unique_lock lock{_impl->_data.mutex};
  return _impl->_data.data_changed;
}

template <typename Message, typename MessagePubSubType>
Message AutowareSubscriber<Message, MessagePubSubType>::GetMessage() {
  std::unique_lock lock{_impl->_data.mutex};
  _impl->_data.data_changed = false;
  return _impl->_data.data;
}

template <typename Message, typename MessagePubSubType>
bool AutowareSubscriber<Message, MessagePubSubType>::Init(const TopicConfig& config) {
  if (!_impl->_type) {
    std::cerr << "Invalid TypeSupport" << std::endl;
    return false;
  }

  efd::DomainParticipantQos pqos = efd::PARTICIPANT_QOS_DEFAULT;
  pqos.name(_name);
  auto factory = efd::DomainParticipantFactory::get_instance();
  _impl->_participant = factory->create_participant(config.domain_id, pqos);
  if (!_impl->_participant) {
    std::cerr << "Failed to create DomainParticipant" << std::endl;
    return false;
  }
  _impl->_type.register_type(_impl->_participant);

  efd::SubscriberQos subqos = efd::SUBSCRIBER_QOS_DEFAULT;
  _impl->_subscriber = _impl->_participant->create_subscriber(subqos, nullptr);
  if (!_impl->_subscriber) {
    std::cerr << "Failed to create Subscriber" << std::endl;
    return false;
  }

  efd::TopicQos tqos = efd::TOPIC_QOS_DEFAULT;
  std::string base{"rt/carla/"};
  std::string topic_name = base;
  if (!_parent.empty())
    topic_name += _parent + "/";
  topic_name += _name;
  topic_name += config.suffix;

  if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
    topic_name = custom_topic_name.value();
  }

  _impl->_topic = _impl->_participant->create_topic(topic_name, _impl->_type->getName(), tqos);
  if (!_impl->_topic) {
    std::cerr << "Failed to create Topic" << std::endl;
    return false;
  }

  efd::DataReaderQos rqos = efd::DATAREADER_QOS_DEFAULT;
  configure_qos(config, rqos);

  efd::DataReaderListener* listener = (efd::DataReaderListener*)_impl->_listener._impl.get();
  _impl->_datareader = _impl->_subscriber->create_datareader(_impl->_topic, rqos, listener);
  if (!_impl->_datareader) {
    std::cerr << "Failed to create DataReader" << std::endl;
    return false;
  }

  return true;
}

// Copy and move constructors, and assignment operators
template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::AutowareSubscriber(const AutowareSubscriber& other)
{
  _frame_id = other._frame_id;
  _name = other._name;
  _parent = other._parent;
  _impl = other._impl;
  _impl->_listener.SetOwnerData(&_impl->_data);
}

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>& AutowareSubscriber<Message, MessagePubSubType>::operator=(const AutowareSubscriber& other) {
  if (this != &other) {
    _frame_id = other._frame_id;
    _name = other._name;
    _parent = other._parent;
    _impl = other._impl;
    _impl->_listener.SetOwnerData(&_impl->_data);
  }
  return *this;
}

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>::AutowareSubscriber(AutowareSubscriber&& other)
{
  _frame_id = std::move(other._frame_id);
  _name = std::move(other._name);
  _parent = std::move(other._parent);
  _impl = std::move(other._impl);
  _impl->_listener.SetOwnerData(&_impl->_data);
}

template <typename Message, typename MessagePubSubType>
AutowareSubscriber<Message, MessagePubSubType>& AutowareSubscriber<Message, MessagePubSubType>::operator=(AutowareSubscriber&& other) {
  if (this != &other) {
    _frame_id = std::move(other._frame_id);
    _name = std::move(other._name);
    _parent = std::move(other._parent);
    _impl = std::move(other._impl);
    _impl->_listener.SetOwnerData(&_impl->_data);
  }
  return *this;
}

}  // namespace ros2
}  // namespace carla

// Instantiate all types
#include "carla/ros2/types/Control.h"
#include "carla/ros2/types/ControlPubSubTypes.h"
#include "carla/ros2/types/GearCommand.h"
#include "carla/ros2/types/GearCommandPubSubTypes.h"
#include "carla/ros2/types/TurnIndicatorsCommand.h"
#include "carla/ros2/types/TurnIndicatorsCommandPubSubTypes.h"
#include "carla/ros2/types/HazardLightsCommand.h"
#include "carla/ros2/types/HazardLightsCommandPubSubTypes.h"
#include "carla/ros2/types/VehicleEmergencyStamped.h"
#include "carla/ros2/types/VehicleEmergencyStampedPubSubTypes.h"
#include "carla/ros2/types/Engage.h"
#include "carla/ros2/types/EngagePubSubTypes.h"

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
