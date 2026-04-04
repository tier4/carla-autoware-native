#include "BasicPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "String.h"

namespace carla {
namespace ros2 {

  struct BasicPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    std_msgs_msg_String_ _message {};
  };

  bool BasicPublisher::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("std_msgs_msg_String_");
    if (!_impl->_dds) return false;

    const std::string topic_name { "rt/basic_publisher_example" };

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool BasicPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_message);
  }

  void BasicPublisher::SetData(const char* msg) {
    _impl->_message.data = std::string(msg);
  }

  BasicPublisher::BasicPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<BasicPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  BasicPublisher::~BasicPublisher() = default;
  BasicPublisher::BasicPublisher(const BasicPublisher&) = default;
  BasicPublisher& BasicPublisher::operator=(const BasicPublisher&) = default;
  BasicPublisher::BasicPublisher(BasicPublisher&&) = default;
  BasicPublisher& BasicPublisher::operator=(BasicPublisher&&) = default;
}
}
