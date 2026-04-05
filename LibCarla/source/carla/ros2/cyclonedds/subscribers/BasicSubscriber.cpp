#include "BasicSubscriber.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "String.h"

namespace carla {
namespace ros2 {

  struct BasicSubscriberImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _reader { 0 };
    std_msgs_msg_String_ _event {};
    std::string _message {};
    bool _new_message {false};
    bool _alive {true};
    void* _actor {nullptr};
  };

  static void on_data_available(dds_entity_t reader, void* arg) {
    BasicSubscriber* self = static_cast<BasicSubscriber*>(arg);
    if (!self) return;

    std_msgs_msg_String_ msg {};
    void* samples[1] = { &msg };
    dds_sample_info_t infos[1];

    int32_t n = dds_take(reader, samples, infos, 1, 1);
    if (n > 0 && infos[0].valid_data) {
      if (msg.data) {
        self->ForwardMessage(msg.data);
      }
    }
  }

  bool BasicSubscriber::Init() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    const std::string subscriber_type {"/basic_subscriber_example"};
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += subscriber_type;

    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &std_msgs_msg_String__desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in BasicSubscriber [" << topic_name << "]: " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    // Create listener for data available callback
    dds_listener_t* listener = dds_create_listener(this);
    dds_lset_data_available(listener, on_data_available);

    _impl->_reader = dds_create_reader(_impl->_participant, _impl->_topic, nullptr, listener);
    dds_delete_listener(listener);
    if (_impl->_reader < 0) {
        std::cerr << "Failed to create DataReader" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

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

  BasicSubscriber::~BasicSubscriber() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  BasicSubscriber::BasicSubscriber(const BasicSubscriber&) = default;
  BasicSubscriber& BasicSubscriber::operator=(const BasicSubscriber&) = default;
  BasicSubscriber::BasicSubscriber(BasicSubscriber&&) = default;
  BasicSubscriber& BasicSubscriber::operator=(BasicSubscriber&&) = default;
}
}
