#include "CarlaSpeedometerSensor.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/dds/cyclonedds/CycloneDDSTopicHelper.h"
#include "carla/ros2/dds/cyclonedds/conversions.hpp"
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
    TopicConfig config;
    config.domain_id = GetDomainId();
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;
    return Init(config);
  }

  bool CarlaSpeedometerSensor::Init(const TopicConfig& config) {
    _impl->_participant = dds_create_participant(config.domain_id, nullptr, nullptr);
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

    dds_qos_t* qos = dds_create_qos();
    configure_cyclone_qos(config, qos);
    _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, qos, nullptr);
    dds_delete_qos(qos);
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

  CarlaSpeedometerSensor::CarlaSpeedometerSensor(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaSpeedometerSensorImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
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
