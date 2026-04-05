#include "CarlaSpeedometerSensor.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "Float32.h"

namespace carla {
namespace ros2 {

  struct CarlaSpeedometerSensorImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    std_msgs_msg_Float32 _float {};
  };

  bool CarlaSpeedometerSensor::Init() {
    _impl->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;

    if (const auto custom_topic_name = ValidTopicName()) {
        topic_name = custom_topic_name.value();
    }
    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &std_msgs_msg_Float32_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaSpeedometerSensor::Publish() {
    return dds_write(_impl->_writer, &_impl->_float) >= 0;
  }

  void CarlaSpeedometerSensor::SetData(float data) {
    _impl->_float.data = data;
  }

  CarlaSpeedometerSensor::CarlaSpeedometerSensor(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaSpeedometerSensorImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaSpeedometerSensor::~CarlaSpeedometerSensor() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaSpeedometerSensor::CarlaSpeedometerSensor(const CarlaSpeedometerSensor&) = default;
  CarlaSpeedometerSensor& CarlaSpeedometerSensor::operator=(const CarlaSpeedometerSensor&) = default;
  CarlaSpeedometerSensor::CarlaSpeedometerSensor(CarlaSpeedometerSensor&&) = default;
  CarlaSpeedometerSensor& CarlaSpeedometerSensor::operator=(CarlaSpeedometerSensor&&) = default;
}
}
