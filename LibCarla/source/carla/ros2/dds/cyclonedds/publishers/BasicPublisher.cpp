#include "BasicPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "String.h"

namespace carla {
namespace ros2 {

  struct BasicPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    std_msgs_msg_String_ _message {};
    std::string _data_store;
  };

  bool BasicPublisher::Init() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string topic_name { "rt/basic_publisher_example" };
    _impl->_topic = dds_create_topic(_impl->_participant, &std_msgs_msg_String__desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, nullptr, nullptr);
    if (_impl->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool BasicPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_message) >= 0;
  }

  void BasicPublisher::SetData(const char* msg) {
    _impl->_data_store = msg;
    _impl->_message.data = const_cast<char*>(_impl->_data_store.c_str());
  }

  BasicPublisher::BasicPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<BasicPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  BasicPublisher::~BasicPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  BasicPublisher::BasicPublisher(const BasicPublisher&) = default;
  BasicPublisher& BasicPublisher::operator=(const BasicPublisher&) = default;
  BasicPublisher::BasicPublisher(BasicPublisher&&) = default;
  BasicPublisher& BasicPublisher::operator=(BasicPublisher&&) = default;
}
}
