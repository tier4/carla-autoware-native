#include "BasicSubscriber.h"

#include <string>

#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include "String.h"

namespace carla {
namespace ros2 {

  struct BasicSubscriberImpl {
    std::unique_ptr<DDSSubscriberImpl> _dds;
    std_msgs_msg_String_ _event {};
    std::string _message {};
    bool _new_message {false};
    bool _alive {true};
    void* _actor {nullptr};
  };

  bool BasicSubscriber::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSSubscriber("std_msgs_msg_String_");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    const std::string subscriber_type {"/basic_subscriber_example"};
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += subscriber_type;

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name)) {
      return false;
    }

    // Set up callback to replicate the old BasicListener behavior:
    // when data arrives, read it and forward the string message.
    _impl->_dds->SetOnDataCallback([this]() {
      if (_impl->_dds->TakeNextSample(&_impl->_event)) {
        ForwardMessage(_impl->_event.data);
      }
    });

    return true;
  }

  void BasicSubscriber::ForwardMessage(const std::string& message) {
    _impl->_message = message;
    _impl->_new_message = true;
  }

  void BasicSubscriber::DestroySubscriber() {
    _impl->_alive = false;
  }

  const char* BasicSubscriber::GetMessage() {
    _impl->_new_message = false;
    return _impl->_message.c_str();
  }

  bool BasicSubscriber::IsAlive() {
    return _impl->_alive;
  }

  bool BasicSubscriber::HasNewMessage() {
    return _impl->_new_message;
  }

  void* BasicSubscriber::GetActor() {
    return _impl->_actor;
  }

  BasicSubscriber::BasicSubscriber(void* actor, const char* ros_name, const char* parent) :
     _impl(std::make_shared<BasicSubscriberImpl>()) {
    _impl->_actor = actor;
    _name = ros_name;
    _parent = parent;
  }

  BasicSubscriber::~BasicSubscriber() = default;
  BasicSubscriber::BasicSubscriber(const BasicSubscriber&) = default;
  BasicSubscriber& BasicSubscriber::operator=(const BasicSubscriber&) = default;
  BasicSubscriber::BasicSubscriber(BasicSubscriber&&) = default;
  BasicSubscriber& BasicSubscriber::operator=(BasicSubscriber&&) = default;
}
}
