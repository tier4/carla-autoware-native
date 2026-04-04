#include "CarlaMapSensorPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "String.h"

namespace carla {
namespace ros2 {

  struct CarlaMapSensorPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    std_msgs_msg_String_ _string {};
    std::string _data_store;
  };

  bool CarlaMapSensorPublisher::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("std_msgs::msg::String");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    if (const auto custom_topic_name = ValidTopicName()) {
      topic_name = custom_topic_name.value();
    }

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaMapSensorPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_string);
  }

  void CarlaMapSensorPublisher::SetData(const char* data) {
    _impl->_data_store = data;
    _impl->_string.data = const_cast<char*>(_impl->_data_store.c_str());
  }

  CarlaMapSensorPublisher::CarlaMapSensorPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaMapSensorPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaMapSensorPublisher::~CarlaMapSensorPublisher() = default;
  CarlaMapSensorPublisher::CarlaMapSensorPublisher(const CarlaMapSensorPublisher&) = default;
  CarlaMapSensorPublisher& CarlaMapSensorPublisher::operator=(const CarlaMapSensorPublisher&) = default;
  CarlaMapSensorPublisher::CarlaMapSensorPublisher(CarlaMapSensorPublisher&&) = default;
  CarlaMapSensorPublisher& CarlaMapSensorPublisher::operator=(CarlaMapSensorPublisher&&) = default;
}
}
