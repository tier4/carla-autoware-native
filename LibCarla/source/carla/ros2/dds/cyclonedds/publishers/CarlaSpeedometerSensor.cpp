#include "CarlaSpeedometerSensor.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Float32.h"

namespace carla {
namespace ros2 {

  struct CarlaSpeedometerSensorImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    std_msgs_msg_Float32 _float {};
  };

  bool CarlaSpeedometerSensor::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("std_msgs::msg::Float32");
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
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaSpeedometerSensor::Publish() {
    return _impl->_dds->Write(&_impl->_float);
  }

  void CarlaSpeedometerSensor::SetData(float data) {
    _impl->_float.data = data;
  }

  CarlaSpeedometerSensor::CarlaSpeedometerSensor(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaSpeedometerSensorImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaSpeedometerSensor::~CarlaSpeedometerSensor() = default;
  CarlaSpeedometerSensor::CarlaSpeedometerSensor(const CarlaSpeedometerSensor&) = default;
  CarlaSpeedometerSensor& CarlaSpeedometerSensor::operator=(const CarlaSpeedometerSensor&) = default;
  CarlaSpeedometerSensor::CarlaSpeedometerSensor(CarlaSpeedometerSensor&&) = default;
  CarlaSpeedometerSensor& CarlaSpeedometerSensor::operator=(CarlaSpeedometerSensor&&) = default;
}
}
